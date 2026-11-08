"""Robot-side replica of Module 1's global scene graph: keeps this robot's
copy in sync and republishes it robot-locally.

  ros2 run fleetgraph_robot scene_replica --ros-args -r __ns:=/robot_1 \
      -p robot_id:=robot_1 -p use_sim_time:=true

Applies every /fleet/scene_graph_update to a central_graph.GraphReplica.
After a version gap it asks /fleet/get_scene_graph for a snapshot without
blocking the executor, holds the updates that arrive meanwhile and replays
them on top of the snapshot (the ones it already covers drop out as
duplicates). Without a reply within RESYNC_TIMEOUT_SEC it asks again, and
a snapshot on the topic resyncs as well. Whenever the graph changes it
publishes the whole of it on central/scene (relative: /robot_N/central/scene)
as a SceneGraphUpdate with is_snapshot true, reliable and transient local
with depth 1, so a late subscriber gets the current graph. Reads only the
/fleet/* contract topic and service.
"""
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from fleetgraph_interfaces.msg import SceneGraphUpdate
from fleetgraph_interfaces.srv import GetSceneGraph
from fleetgraph_robot import contract, scene_msgs
from fleetgraph_robot.central_graph import GraphReplica, GraphUpdate

SCENE_TOPIC = 'central/scene'
QOS_SCENE = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL)
RESYNC_TIMEOUT_SEC = 2.0
RESYNC_CHECK_SEC = 0.5


class SceneReplica(Node):
    def __init__(self):
        super().__init__('scene_replica')
        self.robot_id = self.declare_parameter('robot_id', 'robot_1').value
        self.replica = GraphReplica()
        self.meta = contract.MetaStamper(self.robot_id, self.get_clock())
        self.pub_scene = self.create_publisher(SceneGraphUpdate, SCENE_TOPIC, QOS_SCENE)
        self.create_subscription(SceneGraphUpdate, contract.TOPIC_SCENE_GRAPH_UPDATE,
                                 self._on_update, contract.QOS_EVENTS)
        self.client = self.create_client(GetSceneGraph, contract.SERVICE_GET_SCENE_GRAPH)
        self._in_sync = True            # version 0 is the empty graph Module 1 starts with
        self._request = None            # pending snapshot request
        self._requested_at = 0.0
        self._held = []                 # updates received while it is pending
        self._published = None          # (version, entities, relations)
        self.create_timer(RESYNC_CHECK_SEC, self._check_resync)
        self.get_logger().info(f'{self.robot_id}: {contract.TOPIC_SCENE_GRAPH_UPDATE} -> '
                               f'{self.pub_scene.topic_name}')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_update(self, msg):
        self._receive(scene_msgs.graph_update_from_msg(msg))

    def _receive(self, update):
        if self._request is not None:
            self._held.append(update)
            return
        result = self.replica.apply(update)
        if result == 'gap':
            if self._in_sync:
                self.get_logger().info(
                    f'version gap at {self.replica.version} (got {update.base_version}->'
                    f'{update.graph_version}): requesting a snapshot')
            self._in_sync = False
            self._request_snapshot()
            if self._request is not None:
                self._held.append(update)       # in case the snapshot predates it
        elif result != 'duplicate':
            self._in_sync = True
            self._publish_scene()

    def _request_snapshot(self):
        if self._request is not None or not self.client.service_is_ready():
            return                              # _check_resync asks again
        self._request = self.client.call_async(GetSceneGraph.Request())
        self._requested_at = self.now_sec()
        self._request.add_done_callback(self._on_snapshot)

    def _on_snapshot(self, future):
        if future is not self._request:
            return                              # abandoned after a timeout
        self._request = None
        snapshot = scene_msgs.graph_update_from_msg(future.result().snapshot)
        self.get_logger().info(f'resynced to version {snapshot.graph_version} '
                               f'({len(snapshot.added)} entities) via '
                               f'{contract.SERVICE_GET_SCENE_GRAPH}')
        self._receive(snapshot)
        self._replay_held()

    def _replay_held(self):
        held, self._held = self._held, []
        for update in held:
            self._receive(update)               # held again behind a new request

    def _check_resync(self):
        if (self._request is not None
                and self.now_sec() - self._requested_at > RESYNC_TIMEOUT_SEC):
            self.get_logger().warning(f'no snapshot within {RESYNC_TIMEOUT_SEC} s: asking again')
            self.client.remove_pending_request(self._request)
            self._request = None
            self._replay_held()
        if not self._in_sync:
            self._request_snapshot()

    def _publish_scene(self):
        scene = (self.replica.version, self.replica.entities(), self.replica.relations())
        if scene == self._published:
            return
        self._published = scene
        version, entities, relations = scene
        msg = scene_msgs.graph_update_to_msg(
            GraphUpdate(version, 0, True, '', entities, [], [], relations, []))
        self.pub_scene.publish(self.meta.stamp(msg, SCENE_TOPIC))


def main():
    rclpy.init()
    node = SceneReplica()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
