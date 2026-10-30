"""Scenario runner: loads a YAML scenario, starts sim + actions, records
topics, evaluates assertions. One fresh sim per scenario (own ROS_DOMAIN_ID,
processes killed via process groups) so scenarios are independent.

Usage: runner.run(path) -> result dict:
  {name, passed, failures, expect_failure, evidence}
Raise ScenarioError for invalid scenario files.
"""
import os
import random
import signal
import subprocess
import time

import rclpy
import yaml

import assertions
from recorder import Recorder

ROS_SETUP = ('source /opt/ros/jazzy/setup.bash && '
             'source /home/ubuntu/ws/install/setup.bash 2>/dev/null')

ACTION_KINDS = ('run', 'launch', 'publish')
MOCK_DATA = os.environ.get('FLEETGRAPH_MOCK_DATA', '/home/ubuntu/mock_data')
DEFAULT_SIM_READY_TIMEOUT_SEC = 240
DEFAULT_CLOCK_STALL_SEC = 30


class ScenarioError(Exception):
    pass


def _assertion_kind(assertion):
    """The single assertion type named in one `assert` entry."""
    return next(key for key in assertion if key in assertions.REGISTRY)


def validate(scenario, origin='scenario'):
    if not isinstance(scenario, dict):
        raise ScenarioError(
            f"{origin}: scenario file is empty or not a mapping")
    for key in ('name', 'topics', 'actions', 'assert', 'window_sec'):
        if key not in scenario:
            raise ScenarioError(f"{origin}: missing required key '{key}'")
    for assertion in scenario['assert']:
        kinds = [key for key in assertion if key in assertions.REGISTRY]
        if len(kinds) != 1:
            raise ScenarioError(
                f"{origin}: each assert needs exactly one type out of "
                f"{sorted(assertions.REGISTRY)}, got: {assertion}")
    for action in scenario['actions']:
        kinds = [key for key in action if key in ACTION_KINDS]
        if len(kinds) != 1 or 't' not in action:
            raise ScenarioError(
                f"{origin}: each action needs 't' plus exactly one of "
                f"{list(ACTION_KINDS)}, got: {action}")
    return scenario


def load(path):
    with open(path) as file:
        return validate(yaml.safe_load(file), origin=path)


class ManagedProcess:
    """A ROS command in its own process group, logged to /tmp/harness_<tag>.log."""

    def __init__(self, command, env=None, tag='proc'):
        process_env = dict(os.environ)
        if env:
            process_env.update({str(key): str(value) for key, value in env.items()})
        self.tag = tag
        self.log_path = f"/tmp/harness_{tag}.log"
        self.log = open(self.log_path, 'w')
        self.process = subprocess.Popen(
            ['bash', '-c', f'{ROS_SETUP} && exec {command}'],
            env=process_env, preexec_fn=os.setsid,
            stdout=self.log, stderr=subprocess.STDOUT)

    def stop(self):
        try:
            os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
            self.process.wait(timeout=8)
        except Exception:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGKILL)
            except Exception:
                pass
        self.log.close()

    def log_tail(self, line_count=5):
        try:
            with open(self.log_path) as file:
                return [line.rstrip() for line in file.readlines()[-line_count:]]
        except OSError:
            return []


class View:
    """What assertions see of a recording: stream(topic) and duration()."""

    def __init__(self, recorder):
        self.recorder = recorder

    def stream(self, topic):
        return self.recorder.streams.get(topic, [])

    def duration(self):
        return self.recorder.elapsed()


def _task_history(task_updates):
    """Per task: compressed state history [(sim t, source, state, code)]."""
    history = {}
    for timestamp, msg in task_updates:
        entries = history.setdefault(msg.task_id, [])
        entry = (round(timestamp, 1), msg.meta.source_id, msg.state, msg.error_code)
        if not entries or entries[-1][1:] != entry[1:]:
            entries.append(entry)
    return history


def _evidence(recorder, processes):
    elapsed = recorder.elapsed()
    evidence = {'window_elapsed_sec': round(elapsed, 1)}
    for topic, stream in recorder.streams.items():
        evidence[topic] = {'count': len(stream),
                           'per_sec': round(len(stream) / max(elapsed, 1e-9), 1)}
    for topic in ('/odom', '/ground_truth/odom'):
        if recorder.streams.get(topic):
            position = recorder.streams[topic][-1][1].pose.pose.position
            key = f'last{topic.replace("/", "_")}'
            evidence[key] = [round(position.x, 2), round(position.y, 2)]
    if recorder.streams.get('/fleet/task_update'):
        evidence['task_history'] = _task_history(recorder.streams['/fleet/task_update'])
    for process in processes:
        tail = process.log_tail()
        if tail:
            evidence[f'log_{process.tag}'] = tail
    return evidence


def _start_action(action):
    """Start the process for a `run` or `launch` action (None for `publish`)."""
    if 'run' in action:
        run_spec = action['run']
        # {mock_data} in args = mounted mock_data/ directory
        args = str(run_spec.get('args', '')).replace('{mock_data}', MOCK_DATA)
        return ManagedProcess(
            f"ros2 run {run_spec['pkg']} {run_spec['exec']} {args}",
            tag=run_spec.get('tag', run_spec['exec']))
    if 'launch' in action:
        launch_spec = action['launch']                 # "<package> <file>.launch.py ..."
        launch_name = launch_spec.split()[1].split('.')[0]
        return ManagedProcess(f"ros2 launch {launch_spec}", tag='launch_' + launch_name)
    return None


def _wait_for_clock(recorder, sim):
    timeout_sec = sim.get('ready_timeout_sec', DEFAULT_SIM_READY_TIMEOUT_SEC)
    deadline = time.time() + float(timeout_sec)
    while recorder.clock_sec is None:
        rclpy.spin_once(recorder, timeout_sec=0.2)
        if time.time() > deadline:
            raise ScenarioError(
                f"sim did not publish /clock within {timeout_sec}s wall time")


def run(path):
    scenario = load(path)
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(2, 99))
    rclpy.init()
    recorder = None
    processes = []
    result = {'name': scenario['name'], 'passed': False, 'failures': [],
              'expect_failure': bool(scenario.get('expect_failure')), 'evidence': {}}
    try:
        recorder = Recorder()
        for topic, type_name in scenario['topics'].items():
            recorder.watch(topic, type_name)

        sim = scenario.get('sim')
        if sim:
            processes.append(ManagedProcess(f"ros2 launch {sim['launch']}",
                                            env=sim.get('env'), tag='sim'))
            _wait_for_clock(recorder, sim)

        recorder.start_window()
        # watchdog: a dead/frozen Gazebo stops publishing /clock, which
        # would freeze elapsed() and hang the suite forever. Fail fast
        # on clock stall instead (wall-time check, sim-present only).
        clock_stall_sec = (float(sim.get('clock_stall_sec', DEFAULT_CLOCK_STALL_SEC))
                           if sim else None)
        last_seen_clock = None
        last_clock_progress_wall = time.time()
        publishers = [recorder.make_publisher(action['publish'])
                      for action in scenario['actions'] if 'publish' in action]
        pending_actions = sorted(scenario['actions'], key=lambda action: action['t'])
        window_sec = float(scenario['window_sec'])

        while True:
            rclpy.spin_once(recorder, timeout_sec=0.05)
            now = recorder.elapsed()
            if clock_stall_sec is not None:
                if recorder.clock_sec is not None and recorder.clock_sec != last_seen_clock:
                    last_seen_clock = recorder.clock_sec
                    last_clock_progress_wall = time.time()
                if time.time() - last_clock_progress_wall > clock_stall_sec:
                    raise ScenarioError(
                        f"sim stalled: no /clock progress for "
                        f"{clock_stall_sec}s wall time "
                        f"(window was {now:.1f}/{window_sec} sim s)")
            while pending_actions and pending_actions[0]['t'] <= now:
                process = _start_action(pending_actions.pop(0))
                if process is not None:
                    processes.append(process)
            for publisher in publishers:
                publisher.tick(now)
            if now >= window_sec:
                break

        view = View(recorder)
        for assertion in scenario['assert']:
            kind = _assertion_kind(assertion)
            failure = assertions.REGISTRY[kind](view, **dict(assertion[kind]))
            if failure:
                result['failures'].append(f"{kind}: {failure}")

        result['passed'] = not result['failures']
        result['evidence'] = _evidence(recorder, processes)
    finally:
        for process in processes:
            process.stop()
        if recorder is not None:
            recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result
