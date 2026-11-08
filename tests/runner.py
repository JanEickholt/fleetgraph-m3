"""Scenario runner: loads a YAML scenario, starts sim + actions, records
topics, evaluates assertions. One fresh sim per scenario (own ROS_DOMAIN_ID
and GZ_PARTITION, processes killed via process groups) so scenarios are
independent.

Usage: runner.run(path) -> result dict:
  {name, passed, failures, expect_failure, evidence}
Raise ScenarioError for invalid scenario files.
"""
import inspect
import os
import random
import re
import signal
import subprocess
import time

import rclpy
import yaml

import assertions
from recorder import Recorder

ROS_SETUP = ('source /opt/ros/jazzy/setup.bash && '
             'source /home/ubuntu/ws/install/setup.bash 2>/dev/null')
# Scenario domains: never the container's own (compose.yaml, where manual
# sims and the Module 2 bridge run), never one an earlier scenario of this
# session used (its leftovers would join).
DOMAIN_IDS = range(2, 100)
CONTAINER_DOMAIN_ID = os.environ.get('ROS_DOMAIN_ID', '0')
_used_domain_ids = set()

ACTION_KINDS = ('run', 'launch', 'publish')
MOCK_DATA = os.environ.get('FLEETGRAPH_MOCK_DATA', '/home/ubuntu/mock_data')
DEFAULT_SIM_READY_TIMEOUT_SEC = 240
DEFAULT_CLOCK_STALL_SEC = 30
SIM_TAG = 'sim'


class ScenarioError(Exception):
    pass


def _assertion_kind(assertion):
    """The single assertion type named in one `assert` entry."""
    return next(key for key in assertion if key in assertions.REGISTRY)


def _process_tag(action):
    """Tag of a run/launch action's process (/tmp/harness_<tag>.log,
    evidence log_<tag>); None for publish actions."""
    if 'run' in action:
        return action['run'].get('tag', action['run'].get('exec'))
    if 'launch' in action:
        launch_file = action['launch'].split()[1]      # "<package> <file>.launch.py ..."
        return 'launch_' + launch_file.split('.')[0]
    return None


def _parameter_error(kind, params):
    """Why `params` do not fit the signature of assertion `kind`, or None:
    an unknown key would otherwise be ignored and its filter fall back to
    the permissive default, a missing one fail only after the run."""
    if not isinstance(params, dict):
        return f"parameters must be a mapping, got {params!r}"
    parameters = list(inspect.signature(assertions.REGISTRY[kind]).parameters.values())[1:]
    names = [parameter.name for parameter in parameters]       # without `view`
    unknown = sorted(set(params) - set(names))
    if unknown:
        return f"unknown parameter(s) {unknown}; {kind} takes {names}"
    missing = [parameter.name for parameter in parameters
               if parameter.default is inspect.Parameter.empty and parameter.name not in params]
    if missing:
        return f"missing required parameter(s) {missing}"
    return None


def validate(scenario, origin='scenario'):
    if not isinstance(scenario, dict):
        raise ScenarioError(
            f"{origin}: scenario file is empty or not a mapping")
    for key in ('name', 'topics', 'actions', 'assert', 'window_sec'):
        if key not in scenario:
            raise ScenarioError(f"{origin}: missing required key '{key}'")
    for assertion in scenario['assert']:
        kinds = [key for key in assertion if key in assertions.REGISTRY]
        if len(kinds) != 1 or len(assertion) != 1:
            raise ScenarioError(
                f"{origin}: each assert needs exactly one type out of "
                f"{sorted(assertions.REGISTRY)} and nothing else, got: {assertion}")
        error = _parameter_error(kinds[0], assertion[kinds[0]])
        if error:
            raise ScenarioError(f"{origin}: {kinds[0]}: {error}, got: {assertion}")
    for action in scenario['actions']:
        kinds = [key for key in action if key in ACTION_KINDS]
        if len(kinds) != 1 or 't' not in action:
            raise ScenarioError(
                f"{origin}: each action needs 't' plus exactly one of "
                f"{list(ACTION_KINDS)}, got: {action}")
        publish = action.get('publish')
        if isinstance(publish, dict) and publish.get('t', action['t']) != action['t']:
            raise ScenarioError(
                f"{origin}: publish action at t={action['t']} also says t={publish['t']} "
                f"inside 'publish'; the action's t is the start time, give it once")
    tags = [SIM_TAG] if scenario.get('sim') else []
    tags += [tag for tag in map(_process_tag, scenario['actions']) if tag is not None]
    for tag in tags:
        if tags.count(tag) > 1:
            raise ScenarioError(
                f"{origin}: process tag '{tag}' used twice (one log per tag); "
                f"give each run action a distinct 'tag'")
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
    """What assertions see of a recording: stream(topic) and duration(), the
    window length when the recording ended (wall-time windows would keep
    growing while the assertions run)."""

    def __init__(self, recorder):
        self.recorder = recorder
        self._duration = recorder.elapsed()

    def stream(self, topic):
        return self.recorder.streams.get(topic, [])

    def duration(self):
        return self._duration


def _task_history(task_updates):
    """Per task: compressed state history [(sim t, source, state, code)], in
    Meta.stamp order like the task assertions (arrival order can put the
    agent's REJECTED before the mock's RECEIVED); t is the arrival time."""
    history = {}
    for timestamp, msg in sorted(task_updates, key=assertions._meta_stamp):
        entries = history.setdefault(msg.task_id, [])
        entry = (round(timestamp, 1), msg.meta.source_id, msg.state, msg.error_code)
        if not entries or entries[-1][1:] != entry[1:]:
            entries.append(entry)
    return history


def _evidence(recorder, processes, wall_sec):
    elapsed = recorder.elapsed()
    evidence = {'window_elapsed_sec': round(elapsed, 1), 'window_wall_sec': round(wall_sec, 1)}
    if recorder.clock_sec is not None:
        evidence['real_time_factor'] = round(elapsed / max(wall_sec, 1e-9), 2)
    for topic, stream in recorder.streams.items():
        evidence[topic] = {'count': len(stream),
                           'per_sec': round(len(stream) / max(elapsed, 1e-9), 1)}
        if len(stream) > 1:     # merged over all sources; min_rate can check one
            evidence[topic]['max_gap_sec'] = round(
                max(later[0] - earlier[0] for earlier, later in zip(stream, stream[1:])), 2)
        if topic.endswith('/odom') and stream:
            position = stream[-1][1].pose.pose.position
            evidence[f'last{topic.replace("/", "_")}'] = [round(position.x, 2),
                                                          round(position.y, 2)]
    if recorder.streams.get('/fleet/task_update'):
        evidence['task_history'] = _task_history(recorder.streams['/fleet/task_update'])
    for process in processes:
        tail = process.log_tail()
        if tail:
            evidence[f'log_{process.tag}'] = tail
    return evidence


def _isolate(name, rng=random):
    """Give the scenario `name` its own ROS domain and Gazebo partition in
    os.environ, which every process the runner starts inherits; returns
    both. Ids repeat only once the range is used up."""
    allowed = [domain_id for domain_id in DOMAIN_IDS if str(domain_id) != CONTAINER_DOMAIN_ID]
    if _used_domain_ids.issuperset(allowed):
        _used_domain_ids.clear()                    # range used up: start over
    domain_id = rng.choice([domain_id for domain_id in allowed
                            if domain_id not in _used_domain_ids])
    _used_domain_ids.add(domain_id)
    partition = f"{re.sub(r'[^A-Za-z0-9_]', '_', name)}_{os.getpid()}_{domain_id}"
    os.environ['ROS_DOMAIN_ID'] = str(domain_id)
    os.environ['GZ_PARTITION'] = partition
    return str(domain_id), partition


def _start_action(action):
    """Start the process for a `run` or `launch` action (None for `publish`)."""
    if 'run' in action:
        run_spec = action['run']
        # {mock_data} in args = mounted mock_data/ directory
        args = str(run_spec.get('args', '')).replace('{mock_data}', MOCK_DATA)
        return ManagedProcess(
            f"ros2 run {run_spec['pkg']} {run_spec['exec']} {args}",
            tag=_process_tag(action))
    if 'launch' in action:
        return ManagedProcess(f"ros2 launch {action['launch']}", tag=_process_tag(action))
    return None


def _publishers(recorder, actions):
    """One publisher per `publish` action, all created before the window
    opens so discovery starts early; each publishes from its action's t."""
    return [recorder.make_publisher(action['publish'], start=action['t'])
            for action in actions if 'publish' in action]


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
    _isolate(scenario['name'])
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
                                            env=sim.get('env'), tag=SIM_TAG))
            _wait_for_clock(recorder, sim)

        recorder.start_window()
        # watchdog: a dead/frozen Gazebo stops publishing /clock, which
        # would freeze elapsed() and hang the suite forever. Fail fast
        # on clock stall instead (wall-time check, sim-present only).
        clock_stall_sec = (float(sim.get('clock_stall_sec', DEFAULT_CLOCK_STALL_SEC))
                           if sim else None)
        last_seen_clock = None
        last_clock_progress_wall = time.time()
        publishers = _publishers(recorder, scenario['actions'])
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
        wall_sec = recorder.wall_elapsed()

        view = View(recorder)
        for assertion in scenario['assert']:
            kind = _assertion_kind(assertion)
            failure = assertions.REGISTRY[kind](view, **dict(assertion[kind]))
            if failure:
                result['failures'].append(f"{kind}: {failure}")

        result['passed'] = not result['failures']
        result['evidence'] = _evidence(recorder, processes, wall_sec)
    finally:
        for process in processes:
            process.stop()
        if recorder is not None:
            recorder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result
