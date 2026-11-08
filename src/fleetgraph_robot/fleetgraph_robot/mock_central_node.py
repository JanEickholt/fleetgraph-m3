"""Mock of Module 1's central scene graph for independent M3 testing: fuses
the robots' SceneObservations into the authoritative global graph
(central_graph.py) and serves it the way Module 1 is to.

  ros2 run fleetgraph_robot mock_central --ros-args -p use_sim_time:=true \
      [-p persistent_after_sec:=30.0] [-p ttl_sec.obstacle:=30.0] \
      [-p snapshot_period_sec:=10.0] [-p log_file:=/tmp/central_updates.jsonl]

In:  /fleet/scene_observation (QOS_COMMANDS). Positions must be in the map
     frame; a rejected observation is logged with the reason.
Out: /fleet/scene_graph_update: each change as a versioned delta
     (source_robot_id = the robot whose observation caused it, '' after
     expiry) and a full snapshot every snapshot_period_sec (<= 0: none).
     QoS QOS_EVENTS (reliable, transient local, depth 100): connected
     replicas get every delta in order, a late joiner the last 100
     messages; a receiver whose delta does not apply resyncs through
     /fleet/get_scene_graph (GetSceneGraph, the current snapshot).
Expiry runs at 1 Hz. Robots and this node must share one clock
(use_sim_time in simulation): contributions age by the robots' stamps, and
unset stamps take the receipt time (scene_msgs.observation_from_msg).
log_file: one JSONL line per published update, with latency_sec = update
stamp - stamp of the observation that caused it.
"""
import json
import os

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from fleetgraph_interfaces.msg import SceneGraphUpdate, SceneObservation
from fleetgraph_interfaces.srv import GetSceneGraph
from fleetgraph_robot import contract, scene_msgs
from fleetgraph_robot.central_graph import FUSED, CentralSceneGraph, FusionParams
from fleetgraph_robot.scene_graph import Category

SOURCE_ID = 'm1_central'
EXPIRE_PERIOD_SEC = 1.0


def update_record(update, stamp, seq, observation_stamp):
    """JSONL line of one published update. latency_sec is the central
    update latency; None without a stamped observation behind the update."""
    return {
        'stamp': stamp, 'seq': seq, 'graph_version': update.graph_version,
        'base_version': update.base_version, 'is_snapshot': update.is_snapshot,
        'source_robot_id': update.source_robot_id,
        'added': [_summary(entity) for entity in update.added],
        'updated': [_summary(entity) for entity in update.updated],
        'removed': list(update.removed_ids),
        'relations_added': len(update.added_relations),
        'relations_removed': len(update.removed_relations),
        'latency_sec': (None if observation_stamp is None
                        else round(stamp - observation_stamp, 6)),
    }


def _summary(entity):
    return {'id': entity.id, 'category': Category(entity.category).name,
            'x': round(entity.x, 3), 'y': round(entity.y, 3),
            'confidence': round(entity.confidence, 3), 'sources': list(entity.source_ids),
            'attributes': dict(entity.attributes)}


def _ttl_parameter(category):
    return f'ttl_sec.{Category(category).name.lower()}'


class MockCentral(Node):
    def __init__(self):
        super().__init__('mock_central')
        defaults = FusionParams()
        declared = self.declare_parameters('', [
            ('persistent_after_sec', defaults.persistent_after_sec),
            ('snapshot_period_sec', 10.0),
            ('log_file', ''),
            *[(_ttl_parameter(category), ttl) for category, ttl in defaults.ttl.items()],
        ])
        self.params = {parameter.name: parameter.value for parameter in declared}
        self.graph = CentralSceneGraph(FusionParams(
            ttl={category: self.params[_ttl_parameter(category)] for category in FUSED},
            persistent_after_sec=self.params['persistent_after_sec']))
        self.meta = contract.MetaStamper(SOURCE_ID, self.get_clock())
        self.pub_update = self.create_publisher(
            SceneGraphUpdate, contract.TOPIC_SCENE_GRAPH_UPDATE, contract.QOS_EVENTS)
        self.create_subscription(SceneObservation, contract.TOPIC_SCENE_OBSERVATION,
                                 self._on_observation, contract.QOS_COMMANDS)
        self.create_service(GetSceneGraph, contract.SERVICE_GET_SCENE_GRAPH,
                            self._on_get_scene_graph)
        self.create_timer(EXPIRE_PERIOD_SEC, self._expire)
        snapshot_period = self.params['snapshot_period_sec']
        if snapshot_period > 0:
            self.create_timer(snapshot_period, self._publish_snapshot)
        self.event_log = None
        if self.params['log_file']:
            log_file = os.path.abspath(self.params['log_file'])
            os.makedirs(os.path.dirname(log_file), exist_ok=True)
            self.event_log = open(log_file, 'w')
        snapshots = (f'snapshot every {snapshot_period} s' if snapshot_period > 0
                     else 'no periodic snapshots')
        self.get_logger().info(f'mock M1 scene graph: persistent after '
                               f'{self.params["persistent_after_sec"]} s, {snapshots}')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_observation(self, msg):
        now = self.now_sec()
        try:
            update = self.graph.ingest(scene_msgs.observation_from_msg(msg, now), now)
        except ValueError as error:
            self.get_logger().warning(f'rejected observation from {msg.robot_id!r}: {error}',
                                      throttle_duration_sec=5.0)
            return
        if update is not None:
            stamp = msg.meta.stamp
            unset = scene_msgs.is_unset(stamp)
            self._publish(update, None if unset else scene_msgs.time_to_sec(stamp))

    def _expire(self):
        update = self.graph.expire(self.now_sec())
        if update is not None:
            self._publish(update)

    def _publish_snapshot(self):
        self._publish(self.graph.snapshot())

    def _publish(self, update, observation_stamp=None):
        msg = self.meta.stamp(scene_msgs.graph_update_to_msg(update),
                              contract.TOPIC_SCENE_GRAPH_UPDATE)
        self.pub_update.publish(msg)
        if self.event_log:
            record = update_record(update, scene_msgs.time_to_sec(msg.meta.stamp),
                                   msg.meta.seq, observation_stamp)
            self.event_log.write(json.dumps(record) + '\n')
            self.event_log.flush()

    def _on_get_scene_graph(self, request, response):
        response.snapshot = self.meta.stamp(scene_msgs.graph_update_to_msg(self.graph.snapshot()),
                                            contract.SERVICE_GET_SCENE_GRAPH)
        return response


def main():
    rclpy.init()
    node = MockCentral()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node.event_log:
            node.event_log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
