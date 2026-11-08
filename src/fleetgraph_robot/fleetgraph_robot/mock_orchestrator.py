"""Mock Module 1 (orchestrator) for independent M3 testing: replays a YAML
task script against one or more robots through the real contract topics.

  ros2 run fleetgraph_robot mock_orchestrator --ros-args \
      -p scenario_file:=/home/ubuntu/mock_data/scenarios/week4_demo.yaml -p use_sim_time:=true

Per task it publishes the M1-side lifecycle (RECEIVED, PARSED, QUEUED,
ASSIGNED) on /fleet/task_update, then a TaskAssignment to the task's robot
(task robot_id, default the script's robot_id, else `auto` when the script
names an `allocation` policy, else robot_1). Per robot it
re-sends the assignment until that robot answers (ack timeout), queues by
priority and assigns the next task once the robot reports a terminal
state; robots work in parallel. Scripted
TaskCommands (cancel) are sent at their time. All TaskUpdates are written
to an optional JSONL event log; a metrics summary is logged at the end.
Times in the script are sim seconds after the first /clock tick.

Plain text: a script entry with `text:` instead of type/locations/priority
is parsed by task_text.py, and so is every std_msgs/String on
/mock_m1/task_text (ids live-0001, live-0002, ...). Text the parser refuses
is RECEIVED, then REJECTED with ERROR_PARSE and the parser's reason.
Allocation: a task for robot `auto` waits in one shared queue; every tick
allocation.py picks a robot for it from the latest /fleet/robot_state per
robot (policy: the script's `allocation`, default context), or it stays
QUEUED and the reason is logged.
"""
import json
import os
from dataclasses import asdict, dataclass, replace

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from fleetgraph_interfaces.msg import (
    Location, RobotState, TaskAssignment, TaskCommand, TaskRequest, TaskUpdate)
from fleetgraph_robot import allocation, contract, task_text

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
TOPIC_TASK_TEXT = '/mock_m1/task_text'
AUTO = 'auto'                  # robot_id: the allocator chooses
LIVE_ID_PREFIX = 'live-'


@dataclass
class InFlightAssignment:
    """The assignment sent to the robot that has not finished yet."""
    msg: TaskAssignment
    sent_at: float
    resends: int = 0
    acked: bool = False


def _by_script_time(entry):
    return entry['t']


def _by_priority(request):
    return -request.priority       # sort is stable: FIFO per priority


class MockOrchestrator(Node):
    def __init__(self):
        super().__init__('mock_orchestrator')
        default_locations = os.path.join(get_package_share_directory('fleetgraph_sim'),
                                         'config', 'locations.yaml')
        declared = self.declare_parameters('', [
            ('scenario_file', ''),
            ('locations_file', default_locations),
            ('ack_timeout_sec', 5.0),
            ('max_resends', 3),
            ('log_file', ''),
            ('exit_when_done', False),
            # context allocation; the agent itself refuses below 0.15
            ('min_battery', 0.2),
            ('heartbeat_timeout_sec', 3.0),   # RobotState is nominally 2 Hz
        ])
        self.params = {parameter.name: parameter.value for parameter in declared}
        with open(self.params['scenario_file']) as file:
            scenario = yaml.safe_load(file)
        with open(self.params['locations_file']) as file:
            self.locations = yaml.safe_load(file)['locations']    # id -> {label, x, y, yaw}
        self.location_labels = {location_id: entry.get('label', '')
                                for location_id, entry in self.locations.items()}
        self.policy = scenario.get('allocation', 'context')
        if self.policy not in allocation.POLICIES:
            raise ValueError(f'allocation must be one of {allocation.POLICIES}, '
                             f'got {self.policy!r}')
        self.default_robot_id = scenario.get(
            'robot_id', AUTO if 'allocation' in scenario else 'robot_1')
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
        self.create_subscription(RobotState, contract.TOPIC_ROBOT_STATE, self._on_robot_state,
                                 contract.QOS_TELEMETRY)
        self.create_subscription(String, TOPIC_TASK_TEXT, self._on_task_text, 10)
        log_file = self.params['log_file']
        self.event_log = open(log_file, 'a') if log_file else None

        self.start_time = None     # sim seconds of the first /clock tick
        self.queues = {}           # robot_id -> TaskRequests waiting for that robot
        self.tasks = {}            # task_id -> {robot_id, received, assigned, state, ...}
        self.in_flight = {}        # robot_id -> InFlightAssignment
        self.waiting = []          # TaskRequests for robot `auto`, not yet allocated
        self.robots = {}           # robot_id -> allocation.RobotSnapshot (latest RobotState)
        self.waiting_reasons = {}  # task_id -> why it waits, as last logged
        self.live_count = 0
        self.finished = False
        self.create_timer(0.1, self._tick)
        robots = sorted({entry.get('robot_id', self.default_robot_id) for entry in self.script})
        self.get_logger().info(
            f'mock M1: {len(self.script)} tasks, {len(self.commands)} commands '
            f'for {", ".join(robots) or self.default_robot_id}, allocation {self.policy}, '
            f'text input on {TOPIC_TASK_TEXT}')

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
        for robot_id in list(self.in_flight):
            self._check_ack_timeout(robot_id, now)
        for robot_id, queue in self.queues.items():
            if robot_id not in self.in_flight and queue:
                self._assign(robot_id, queue.pop(0))
        self._allocate_waiting(now)
        if not self.finished and self._all_tasks_done():
            self.finished = True
            self._summary()
            if self.params['exit_when_done']:
                raise SystemExit(0)

    def _check_ack_timeout(self, robot_id, now):
        in_flight = self.in_flight[robot_id]
        if in_flight.acked or now - in_flight.sent_at <= self.params['ack_timeout_sec']:
            return
        task = in_flight.msg.task
        if in_flight.resends >= self.params['max_resends']:
            self._lifecycle(task, TaskUpdate.STATE_FAILED,
                            error_code=TaskUpdate.ERROR_ROBOT_LOST,
                            reason='no answer from robot')
            del self.in_flight[robot_id]
            return
        self.get_logger().warn(f're-sending {task.task_id}')
        self.pub_assignment.publish(
            self.meta.stamp(in_flight.msg, contract.TOPIC_TASK_ASSIGNMENT))
        in_flight.sent_at = now
        in_flight.resends += 1

    def _all_tasks_done(self):
        return (not self.script and not any(self.queues.values()) and not self.in_flight and
                not self.waiting and self.tasks and
                all(record['state'] in TERMINAL_STATES for record in self.tasks.values()))

    def _receive(self, task_entry):
        text = task_entry.get('text')
        if text is not None:
            try:
                parsed = task_text.parse_task_text(text, self.location_labels)
            except task_text.ParseError as error:
                self._reject_text(task_entry['task_id'], text, str(error))
                return
            task_entry = dict(task_entry, raw_text=text, **asdict(parsed))
        task = TaskRequest(
            task_id=task_entry['task_id'], raw_text=task_entry.get('raw_text', ''),
            type=TASK_TYPES.get(task_entry.get('type'), TaskRequest.TYPE_UNKNOWN),
            priority=PRIORITIES[task_entry.get('priority', 'normal')],
            required_capabilities=list(task_entry.get('required_capabilities', [])),
            patrol_loops=int(task_entry.get('patrol_loops', 1)),
            created_at=self.get_clock().now().to_msg())
        # ids only: the robot resolves them from its location registry
        task.locations = [Location(id=location_id)
                          for location_id in task_entry.get('locations', [])]
        robot_id = task_entry.get('robot_id', self.default_robot_id)
        self.tasks[task.task_id] = {'robot_id': '' if robot_id == AUTO else robot_id,
                                    'received': self.now_sec(),
                                    'state': None, 'priority': task.priority, 'rev': 0}
        for state in (TaskUpdate.STATE_RECEIVED, TaskUpdate.STATE_PARSED,
                      TaskUpdate.STATE_QUEUED):
            self._lifecycle(task, state)
        queue = self.waiting if robot_id == AUTO else self.queues.setdefault(robot_id, [])
        queue.append(task)
        queue.sort(key=_by_priority)

    def _reject_text(self, task_id, text, reason):
        """Parser refusal: RECEIVED, then REJECTED with ERROR_PARSE + reason."""
        now = self.now_sec()
        self.tasks[task_id] = {'robot_id': '', 'received': now, 'priority': 0, 'rev': 0,
                               'state': TaskUpdate.STATE_REJECTED, 'done': now,
                               'error_code': TaskUpdate.ERROR_PARSE, 'reason': reason}
        task = TaskRequest(task_id=task_id, raw_text=text)
        self._lifecycle(task, TaskUpdate.STATE_RECEIVED)
        self._lifecycle(task, TaskUpdate.STATE_REJECTED,
                        error_code=TaskUpdate.ERROR_PARSE, reason=reason)
        self.get_logger().info(f'{task_id}: REJECTED, cannot parse {text!r}: {reason}')

    def _allocate_waiting(self, now):
        """Each waiting task, in priority order, gets the robot the policy
        picks now. A task without an eligible robot stays QUEUED for the next
        tick; later tasks may still go to other robots meanwhile."""
        for task in list(self.waiting):
            first = self.locations.get(task.locations[0].id) if task.locations else None
            target = (first['x'], first['y']) if first else None
            # rebuilt per task: an assignment made for the previous task counts
            robots = [replace(robot, assigned_task_id=self._assigned_task(robot.robot_id))
                      for robot in self.robots.values()]
            choice = allocation.allocate(
                self.policy, robots, target, task.required_capabilities, now,
                min_battery=self.params['min_battery'],
                heartbeat_timeout_sec=self.params['heartbeat_timeout_sec'])
            excluded = '; '.join(f'{robot_id}: {reason}'
                                 for robot_id, reason in sorted(choice.excluded.items()))
            if choice.robot_id is None:
                reason = excluded or 'no RobotState received yet'
                if self.waiting_reasons.get(task.task_id) != reason:
                    self.waiting_reasons[task.task_id] = reason
                    self.get_logger().info(f'{task.task_id} waits, no eligible robot: {reason}')
                continue
            self.waiting.remove(task)
            self.waiting_reasons.pop(task.task_id, None)
            details = ', '.join(f'{robot_id} {distance:.2f} m'
                                for robot_id, distance in choice.ranking)
            if excluded:
                details += f'; excluded {excluded}'
            self.get_logger().info(
                f'allocated {task.task_id} -> {choice.robot_id} ({self.policy}: {details})')
            self.tasks[task.task_id]['robot_id'] = choice.robot_id
            self._assign(choice.robot_id, task)

    def _assigned_task(self, robot_id):
        in_flight = self.in_flight.get(robot_id)
        return in_flight.msg.task.task_id if in_flight else ''

    def _assign(self, robot_id, task):
        record = self.tasks[task.task_id]
        record['rev'] += 1
        record['assigned'] = self.now_sec()
        msg = TaskAssignment(robot_id=robot_id, assignment_rev=record['rev'], task=task)
        self._lifecycle(task, TaskUpdate.STATE_ASSIGNED, robot_id=robot_id)
        self.pub_assignment.publish(self.meta.stamp(msg, contract.TOPIC_TASK_ASSIGNMENT))
        self.in_flight[robot_id] = InFlightAssignment(msg, sent_at=self.now_sec())
        self.get_logger().info(f'assigned {task.task_id} -> {robot_id}')

    def _send_command(self, command_entry):
        task_id = command_entry['task_id']
        robot_id = self.tasks.get(task_id, {}).get('robot_id', self.default_robot_id)
        msg = TaskCommand(robot_id=robot_id, task_id=task_id,
                          command=TaskCommand.COMMAND_CANCEL,
                          reason=command_entry.get('reason', 'operator cancel'))
        self.pub_command.publish(self.meta.stamp(msg, contract.TOPIC_TASK_COMMAND))
        self.get_logger().info(f'cancel {task_id} on {robot_id}')

    def _lifecycle(self, task, state, robot_id='', error_code=0, reason=''):
        record = self.tasks[task.task_id]
        msg = TaskUpdate(task_id=task.task_id, robot_id=robot_id,
                         assignment_rev=record['rev'], state=state, progress=-1.0,
                         distance_remaining_m=-1.0, error_code=error_code, reason=reason)
        self.pub_update.publish(self.meta.stamp(msg, contract.TOPIC_TASK_UPDATE))

    # ------------------------------------------------------------- inputs
    def _on_task_text(self, msg):
        task_id = None
        while task_id is None or task_id in self.tasks:      # never reuse a script id
            self.live_count += 1
            task_id = f'{LIVE_ID_PREFIX}{self.live_count:04d}'
        self.get_logger().info(f'text task {task_id}: {msg.data!r}')
        self._receive({'task_id': task_id, 'text': msg.data})

    def _on_robot_state(self, msg):
        pose = msg.pose
        localized = pose.header.frame_id == 'map'
        self.robots[msg.robot_id] = allocation.RobotSnapshot(
            msg.robot_id,
            (pose.pose.position.x, pose.pose.position.y) if localized else None,
            status=msg.status, battery=msg.battery, capabilities=frozenset(msg.capabilities),
            current_task_id=msg.current_task_id, heard_at=self.now_sec())

    def _on_update(self, msg):
        self._log(msg)
        record = self.tasks.get(msg.task_id)
        if record is None:
            return
        record['state'] = msg.state
        robot_id = record['robot_id']
        if msg.meta.source_id != robot_id:
            return
        self._arm_relative_command(msg)
        in_flight = self.in_flight.get(robot_id)
        if in_flight and in_flight.msg.task.task_id == msg.task_id:
            in_flight.acked = True                           # robot answered
            if msg.state in TERMINAL_STATES:
                del self.in_flight[robot_id]
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
