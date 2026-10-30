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
from dataclasses import dataclass

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from fleetgraph_interfaces.msg import (
    Location, TaskAssignment, TaskCommand, TaskRequest, TaskUpdate)
from fleetgraph_robot import contract

TASK_TYPES = {'go_to': TaskRequest.TYPE_GO_TO, 'delivery': TaskRequest.TYPE_DELIVERY,
              'patrol': TaskRequest.TYPE_PATROL}
PRIORITIES = {'low': 0, 'normal': 1, 'high': 2, 'urgent': 3}
# rosidl constants are metaclass properties (not in vars()), so list them
STATE_NAMES = {getattr(TaskUpdate, f'STATE_{name}'): name for name in (
    'RECEIVED', 'PARSED', 'QUEUED', 'ASSIGNED', 'EXECUTING', 'COMPLETED',
    'FAILED', 'CANCELLED', 'REASSIGNED', 'REJECTED')}
TERMINAL_STATES = (TaskUpdate.STATE_COMPLETED, TaskUpdate.STATE_FAILED,
                   TaskUpdate.STATE_CANCELLED, TaskUpdate.STATE_REJECTED)
SOURCE_ID = 'm1_mock'


@dataclass
class InFlightAssignment:
    """The assignment sent to the robot that has not finished yet."""
    msg: TaskAssignment
    sent_at: float
    resends: int = 0
    acked: bool = False


def _by_script_time(entry):
    return entry['t']


class MockOrchestrator(Node):
    def __init__(self):
        super().__init__('mock_orchestrator')
        declared = self.declare_parameters('', [
            ('scenario_file', ''),
            ('ack_timeout_sec', 5.0),
            ('max_resends', 3),
            ('log_file', ''),
            ('exit_when_done', False),
        ])
        self.params = {parameter.name: parameter.value for parameter in declared}
        with open(self.params['scenario_file']) as file:
            scenario = yaml.safe_load(file)
        self.robot_id = scenario.get('robot_id', 'robot_1')
        self.script = sorted(scenario.get('tasks', []), key=_by_script_time)
        commands = scenario.get('commands', [])
        self.commands = sorted([command for command in commands if 't' in command],
                               key=_by_script_time)
        # relative commands: armed when the robot first reports EXECUTING
        self.relative_commands = {command['task_id']: command for command in commands
                                  if 'after_executing_sec' in command}
        self.meta = contract.MetaStamper(SOURCE_ID, self.get_clock())
        self.pub_update = self.create_publisher(
            TaskUpdate, contract.TOPIC_TASK_UPDATE, contract.QOS_EVENTS)
        self.pub_assignment = self.create_publisher(
            TaskAssignment, contract.TOPIC_TASK_ASSIGNMENT, contract.QOS_COMMANDS)
        self.pub_command = self.create_publisher(
            TaskCommand, contract.TOPIC_TASK_COMMAND, contract.QOS_COMMANDS)
        self.create_subscription(TaskUpdate, contract.TOPIC_TASK_UPDATE, self._on_update,
                                 contract.QOS_EVENTS)
        log_file = self.params['log_file']
        self.event_log = open(log_file, 'a') if log_file else None

        self.start_time = None     # sim seconds of the first /clock tick
        self.queue = []            # TaskRequest waiting for the robot
        self.tasks = {}            # task_id -> {received, assigned, done, state, ...}
        self.in_flight = None      # InFlightAssignment or None
        self.finished = False
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'mock M1: {len(self.script)} tasks, {len(self.commands)} commands '
            f'for {self.robot_id}')

    def now_sec(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # --------------------------------------------------------------- loop
    def _tick(self):
        now = self.now_sec()
        if now <= 0.0:
            return                                   # no /clock yet
        if self.start_time is None:
            self.start_time = now
        script_time = now - self.start_time
        while self.script and self.script[0]['t'] <= script_time:
            self._receive(self.script.pop(0))
        while self.commands and self.commands[0]['t'] <= script_time:
            self._send_command(self.commands.pop(0))
        if self.in_flight:
            self._check_ack_timeout(now)
        if not self.in_flight and self.queue:
            self._assign(self.queue.pop(0))
        if not self.finished and self._all_tasks_done():
            self.finished = True
            self._summary()
            if self.params['exit_when_done']:
                raise SystemExit(0)

    def _check_ack_timeout(self, now):
        in_flight = self.in_flight
        if in_flight.acked or now - in_flight.sent_at <= self.params['ack_timeout_sec']:
            return
        task = in_flight.msg.task
        if in_flight.resends >= self.params['max_resends']:
            self._lifecycle(task, TaskUpdate.STATE_FAILED,
                            error_code=TaskUpdate.ERROR_ROBOT_LOST,
                            reason='no answer from robot')
            self.in_flight = None
            return
        self.get_logger().warn(f're-sending {task.task_id}')
        self.pub_assignment.publish(
            self.meta.stamp(in_flight.msg, contract.TOPIC_TASK_ASSIGNMENT))
        in_flight.sent_at = now
        in_flight.resends += 1

    def _all_tasks_done(self):
        return (not self.script and not self.queue and not self.in_flight and
                self.tasks and
                all(record['state'] in TERMINAL_STATES for record in self.tasks.values()))

    def _receive(self, task_entry):
        task = TaskRequest(
            task_id=task_entry['task_id'], raw_text=task_entry.get('raw_text', ''),
            type=TASK_TYPES.get(task_entry.get('type'), TaskRequest.TYPE_UNKNOWN),
            priority=PRIORITIES[task_entry.get('priority', 'normal')],
            required_capabilities=task_entry.get('required_capabilities', []),
            patrol_loops=int(task_entry.get('patrol_loops', 1)),
            created_at=self.get_clock().now().to_msg())
        # ids only: the robot resolves them from its location registry
        task.locations = [Location(id=location_id)
                          for location_id in task_entry.get('locations', [])]
        self.tasks[task.task_id] = {'received': self.now_sec(), 'state': None,
                                    'priority': task.priority, 'rev': 0}
        for state in (TaskUpdate.STATE_RECEIVED, TaskUpdate.STATE_PARSED,
                      TaskUpdate.STATE_QUEUED):
            self._lifecycle(task, state)
        self.queue.append(task)
        self.queue.sort(key=lambda request: -request.priority)  # stable: FIFO per priority

    def _assign(self, task):
        record = self.tasks[task.task_id]
        record['rev'] += 1
        record['assigned'] = self.now_sec()
        msg = TaskAssignment(robot_id=self.robot_id, assignment_rev=record['rev'], task=task)
        self._lifecycle(task, TaskUpdate.STATE_ASSIGNED, robot_id=self.robot_id)
        self.pub_assignment.publish(self.meta.stamp(msg, contract.TOPIC_TASK_ASSIGNMENT))
        self.in_flight = InFlightAssignment(msg, sent_at=self.now_sec())
        self.get_logger().info(f'assigned {task.task_id} -> {self.robot_id}')

    def _send_command(self, command_entry):
        msg = TaskCommand(robot_id=self.robot_id, task_id=command_entry['task_id'],
                          command=TaskCommand.COMMAND_CANCEL,
                          reason=command_entry.get('reason', 'operator cancel'))
        self.pub_command.publish(self.meta.stamp(msg, contract.TOPIC_TASK_COMMAND))
        self.get_logger().info(f'cancel {command_entry["task_id"]}')

    def _lifecycle(self, task, state, robot_id='', error_code=0, reason=''):
        record = self.tasks[task.task_id]
        msg = TaskUpdate(task_id=task.task_id, robot_id=robot_id,
                         assignment_rev=record['rev'], state=state, progress=-1.0,
                         distance_remaining_m=-1.0, error_code=error_code, reason=reason)
        self.pub_update.publish(self.meta.stamp(msg, contract.TOPIC_TASK_UPDATE))

    # ------------------------------------------------------------- inputs
    def _on_update(self, msg):
        self._log(msg)
        record = self.tasks.get(msg.task_id)
        if record is None:
            return
        record['state'] = msg.state
        if msg.meta.source_id != self.robot_id:
            return
        self._arm_relative_command(msg)
        if self.in_flight and self.in_flight.msg.task.task_id == msg.task_id:
            self.in_flight.acked = True                      # robot answered
            if msg.state in TERMINAL_STATES:
                self.in_flight = None
        if msg.state in TERMINAL_STATES:
            record['done'] = self.now_sec()
            record['error_code'] = msg.error_code
            record['reason'] = msg.reason
            self.get_logger().info(
                f'{msg.task_id}: {STATE_NAMES[msg.state]} '
                f'code {msg.error_code} {msg.reason}')

    def _arm_relative_command(self, msg):
        """Schedule a command given as 'after_executing_sec' once its task
        first reports EXECUTING."""
        if msg.state != TaskUpdate.STATE_EXECUTING:
            return
        relative_command = self.relative_commands.pop(msg.task_id, None)
        if relative_command is None:
            return
        script_time = self.now_sec() - self.start_time
        delay = float(relative_command['after_executing_sec'])
        self.commands.append(dict(relative_command, t=script_time + delay))
        self.commands.sort(key=_by_script_time)

    def _log(self, msg):
        if not self.event_log:
            return
        self.event_log.write(json.dumps({
            'stamp': msg.meta.stamp.sec + msg.meta.stamp.nanosec * 1e-9,
            'source': msg.meta.source_id, 'seq': msg.meta.seq,
            'task_id': msg.task_id, 'robot_id': msg.robot_id,
            'state': STATE_NAMES.get(msg.state, msg.state), 'progress': msg.progress,
            'error_code': msg.error_code, 'reason': msg.reason}) + '\n')
        self.event_log.flush()

    def _summary(self):
        records = self.tasks.values()
        done = [record for record in records if 'done' in record]
        completed = [record for record in done
                     if record['state'] == TaskUpdate.STATE_COMPLETED]
        wait_times = [record['assigned'] - record['received']
                      for record in records if 'assigned' in record]
        completion_times = [record['done'] - record['received'] for record in completed]
        summary = {
            'tasks': len(self.tasks), 'completed': len(completed),
            'success_rate': round(len(completed) / len(self.tasks), 3),
            'avg_wait_s': (round(sum(wait_times) / len(wait_times), 2)
                           if wait_times else None),
            'avg_completion_s': (round(sum(completion_times) / len(completion_times), 2)
                                 if completion_times else None),
            'final': {task_id: STATE_NAMES.get(record['state'], record['state'])
                      for task_id, record in self.tasks.items()},
        }
        self.get_logger().info('SUMMARY ' + json.dumps(summary))
        if self.event_log:
            self.event_log.write(json.dumps({'summary': summary}) + '\n')
            self.event_log.flush()


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
