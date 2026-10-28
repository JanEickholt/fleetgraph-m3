"""Mock Module 1 (orchestrator) for independent M3 testing: replays a YAML
task script against one robot through the real contract topics.

  ros2 run fleetgraph_robot mock_orchestrator --ros-args \
      -p scenario_file:=/home/ubuntu/mock_data/scenarios/week4_demo.yaml -p use_sim_time:=true

Per task it publishes the M1-side lifecycle (RECEIVED, PARSED, QUEUED,
ASSIGNED) on /fleet/task_update, then a TaskAssignment; it re-sends the
assignment until the robot answers (ack timeout), queues by priority and
assigns the next task once the robot reports a terminal state. Scripted
TaskCommands (cancel) are sent at their time. All TaskUpdates are written
to an optional JSONL event log; a metrics summary is logged at the end.
Times in the script are sim seconds after the first /clock tick.
"""
import json

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from fleetgraph_interfaces.msg import (
    Location, TaskAssignment, TaskCommand, TaskRequest, TaskUpdate)
from fleetgraph_robot import contract as c

TYPES = {'go_to': TaskRequest.TYPE_GO_TO, 'delivery': TaskRequest.TYPE_DELIVERY,
         'patrol': TaskRequest.TYPE_PATROL}
PRIORITIES = {'low': 0, 'normal': 1, 'high': 2, 'urgent': 3}
# rosidl constants are metaclass properties (not in vars()), so list them
STATE_NAMES = {getattr(TaskUpdate, f'STATE_{n}'): n for n in (
    'RECEIVED', 'PARSED', 'QUEUED', 'ASSIGNED', 'EXECUTING', 'COMPLETED',
    'FAILED', 'CANCELLED', 'REASSIGNED', 'REJECTED')}
TERMINAL = (TaskUpdate.STATE_COMPLETED, TaskUpdate.STATE_FAILED,
            TaskUpdate.STATE_CANCELLED, TaskUpdate.STATE_REJECTED)
SOURCE = 'm1_mock'


class MockOrchestrator(Node):
    def __init__(self):
        super().__init__('mock_orchestrator')
        p = self.declare_parameters('', [
            ('scenario_file', ''),
            ('ack_timeout_sec', 5.0),
            ('max_resends', 3),
            ('log_file', ''),
            ('exit_when_done', False),
        ])
        self.prm = {x.name: x.value for x in p}
        with open(self.prm['scenario_file']) as f:
            sc = yaml.safe_load(f)
        self.robot_id = sc.get('robot_id', 'robot_1')
        self.script = sorted(sc.get('tasks', []), key=lambda t: t['t'])
        cmds = sc.get('commands', [])
        self.commands = sorted([x for x in cmds if 't' in x], key=lambda x: x['t'])
        # relative commands: armed when the robot first reports EXECUTING
        self.relative = {x['task_id']: x for x in cmds if 'after_executing_sec' in x}
        self.meta = c.MetaStamper(SOURCE, self.get_clock())
        self.pub_update = self.create_publisher(TaskUpdate, c.TOPIC_TASK_UPDATE, c.QOS_EVENTS)
        self.pub_assign = self.create_publisher(
            TaskAssignment, c.TOPIC_TASK_ASSIGNMENT, c.QOS_COMMANDS)
        self.pub_cmd = self.create_publisher(TaskCommand, c.TOPIC_TASK_COMMAND, c.QOS_COMMANDS)
        self.create_subscription(TaskUpdate, c.TOPIC_TASK_UPDATE, self._on_update,
                                 c.QOS_EVENTS)
        self.log = open(self.prm['log_file'], 'a') if self.prm['log_file'] else None

        self.t0 = None
        self.queue = []            # TaskRequest waiting for the robot
        self.tasks = {}            # task_id -> {received, assigned, done, state, ...}
        self.in_flight = None      # (TaskAssignment, sent_at, resends, acked)
        self.finished = False
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'mock M1: {len(self.script)} tasks, {len(self.commands)} commands '
            f'for {self.robot_id}')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --------------------------------------------------------------- loop
    def _tick(self):
        now = self.now_s()
        if now <= 0.0:
            return                                   # no /clock yet
        if self.t0 is None:
            self.t0 = now
        t = now - self.t0
        while self.script and self.script[0]['t'] <= t:
            self._receive(self.script.pop(0))
        while self.commands and self.commands[0]['t'] <= t:
            self._command(self.commands.pop(0))
        if self.in_flight:
            msg, sent_at, resends, acked = self.in_flight
            if not acked and now - sent_at > self.prm['ack_timeout_sec']:
                if resends >= self.prm['max_resends']:
                    self._lifecycle(msg.task, TaskUpdate.STATE_FAILED,
                                    error=TaskUpdate.ERROR_ROBOT_LOST,
                                    reason='no answer from robot')
                    self.in_flight = None
                else:
                    self.get_logger().warn(f're-sending {msg.task.task_id}')
                    self.pub_assign.publish(self.meta.stamp(msg, c.TOPIC_TASK_ASSIGNMENT))
                    self.in_flight = (msg, now, resends + 1, False)
        if not self.in_flight and self.queue:
            self._assign(self.queue.pop(0))
        if not self.finished and not self.script and not self.queue and \
                not self.in_flight and self.tasks and \
                all(v['state'] in TERMINAL for v in self.tasks.values()):
            self.finished = True
            self._summary()
            if self.prm['exit_when_done']:
                raise SystemExit(0)

    def _receive(self, spec):
        task = TaskRequest(task_id=spec['task_id'], raw_text=spec.get('raw_text', ''),
                           type=TYPES.get(spec.get('type'), TaskRequest.TYPE_UNKNOWN),
                           priority=PRIORITIES[spec.get('priority', 'normal')],
                           required_capabilities=spec.get('required_capabilities', []),
                           patrol_loops=int(spec.get('patrol_loops', 1)),
                           created_at=self.get_clock().now().to_msg())
        # ids only: the robot resolves them from its location registry
        task.locations = [Location(id=lid) for lid in spec.get('locations', [])]
        self.tasks[task.task_id] = {'received': self.now_s(), 'state': None,
                                    'priority': task.priority, 'rev': 0}
        for state in (TaskUpdate.STATE_RECEIVED, TaskUpdate.STATE_PARSED,
                      TaskUpdate.STATE_QUEUED):
            self._lifecycle(task, state)
        self.queue.append(task)
        self.queue.sort(key=lambda r: -r.priority)       # stable: FIFO per priority

    def _assign(self, task):
        rec = self.tasks[task.task_id]
        rec['rev'] += 1
        rec['assigned'] = self.now_s()
        msg = TaskAssignment(robot_id=self.robot_id, assignment_rev=rec['rev'], task=task)
        self._lifecycle(task, TaskUpdate.STATE_ASSIGNED, robot=self.robot_id)
        self.pub_assign.publish(self.meta.stamp(msg, c.TOPIC_TASK_ASSIGNMENT))
        self.in_flight = (msg, self.now_s(), 0, False)
        self.get_logger().info(f'assigned {task.task_id} -> {self.robot_id}')

    def _command(self, spec):
        msg = TaskCommand(robot_id=self.robot_id, task_id=spec['task_id'],
                          command=TaskCommand.COMMAND_CANCEL,
                          reason=spec.get('reason', 'operator cancel'))
        self.pub_cmd.publish(self.meta.stamp(msg, c.TOPIC_TASK_COMMAND))
        self.get_logger().info(f'cancel {spec["task_id"]}')

    def _lifecycle(self, task, state, robot='', error=0, reason=''):
        rec = self.tasks[task.task_id]
        msg = TaskUpdate(task_id=task.task_id, robot_id=robot,
                         assignment_rev=rec['rev'], state=state, progress=-1.0,
                         distance_remaining_m=-1.0, error_code=error, reason=reason)
        self.pub_update.publish(self.meta.stamp(msg, c.TOPIC_TASK_UPDATE))

    # ------------------------------------------------------------- inputs
    def _on_update(self, msg):
        self._log(msg)
        rec = self.tasks.get(msg.task_id)
        if rec is None:
            return
        rec['state'] = msg.state
        if msg.meta.source_id != self.robot_id:
            return
        rel = self.relative.pop(msg.task_id, None)
        if rel is not None and msg.state == TaskUpdate.STATE_EXECUTING:
            self.commands.append(dict(rel, t=self.now_s() - self.t0 +
                                      float(rel['after_executing_sec'])))
            self.commands.sort(key=lambda x: x['t'])
        elif rel is not None:
            self.relative[msg.task_id] = rel
        if self.in_flight and self.in_flight[0].task.task_id == msg.task_id:
            m, sent, resends, _ = self.in_flight
            self.in_flight = (m, sent, resends, True)        # robot answered
            if msg.state in TERMINAL:
                self.in_flight = None
        if msg.state in TERMINAL:
            rec['done'] = self.now_s()
            rec['error_code'] = msg.error_code
            rec['reason'] = msg.reason
            self.get_logger().info(
                f'{msg.task_id}: {STATE_NAMES[msg.state]} '
                f'code {msg.error_code} {msg.reason}')

    def _log(self, msg):
        if not self.log:
            return
        self.log.write(json.dumps({
            'stamp': msg.meta.stamp.sec + msg.meta.stamp.nanosec * 1e-9,
            'source': msg.meta.source_id, 'seq': msg.meta.seq,
            'task_id': msg.task_id, 'robot_id': msg.robot_id,
            'state': STATE_NAMES.get(msg.state, msg.state), 'progress': msg.progress,
            'error_code': msg.error_code, 'reason': msg.reason}) + '\n')
        self.log.flush()

    def _summary(self):
        done = [v for v in self.tasks.values() if 'done' in v]
        ok = [v for v in done if v['state'] == TaskUpdate.STATE_COMPLETED]
        wait = [v['assigned'] - v['received'] for v in self.tasks.values() if 'assigned' in v]
        comp = [v['done'] - v['received'] for v in ok]
        summary = {
            'tasks': len(self.tasks), 'completed': len(ok),
            'success_rate': round(len(ok) / len(self.tasks), 3),
            'avg_wait_s': round(sum(wait) / len(wait), 2) if wait else None,
            'avg_completion_s': round(sum(comp) / len(comp), 2) if comp else None,
            'final': {k: STATE_NAMES.get(v['state'], v['state'])
                      for k, v in self.tasks.items()},
        }
        self.get_logger().info('SUMMARY ' + json.dumps(summary))
        if self.log:
            self.log.write(json.dumps({'summary': summary}) + '\n')
            self.log.flush()


def main():
    rclpy.init()
    node = MockOrchestrator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
