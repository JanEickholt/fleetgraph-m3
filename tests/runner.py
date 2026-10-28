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

SOURCE = ('source /opt/ros/jazzy/setup.bash && '
          'source /home/ubuntu/ws/install/setup.bash 2>/dev/null')


ACTION_KINDS = ('run', 'launch', 'publish')
MOCK_DATA = os.environ.get('FLEETGRAPH_MOCK_DATA', '/home/ubuntu/mock_data')


class ScenarioError(Exception):
    pass


def validate(sc, origin='scenario'):
    if not isinstance(sc, dict):
        raise ScenarioError(
            f"{origin}: scenario file is empty or not a mapping")
    for key in ('name', 'topics', 'actions', 'assert', 'window_sec'):
        if key not in sc:
            raise ScenarioError(f"{origin}: missing required key '{key}'")
    for a in sc['assert']:
        kinds = [k for k in a if k in assertions.REGISTRY]
        if len(kinds) != 1:
            raise ScenarioError(
                f"{origin}: each assert needs exactly one type out of "
                f"{sorted(assertions.REGISTRY)}, got: {a}")
    for act in sc['actions']:
        kinds = [k for k in act if k in ACTION_KINDS]
        if len(kinds) != 1 or 't' not in act:
            raise ScenarioError(
                f"{origin}: each action needs 't' plus exactly one of "
                f"{list(ACTION_KINDS)}, got: {act}")
    return sc


def load(path):
    with open(path) as f:
        return validate(yaml.safe_load(f), origin=path)


class Proc:
    def __init__(self, cmd, env=None, tag='proc'):
        e = dict(os.environ)
        if env:
            e.update({str(k): str(v) for k, v in env.items()})
        self.tag = tag
        self.log_path = f"/tmp/harness_{tag}.log"
        self.log = open(self.log_path, 'w')
        self.p = subprocess.Popen(
            ['bash', '-c', f'{SOURCE} && exec {cmd}'],
            env=e, preexec_fn=os.setsid,
            stdout=self.log, stderr=subprocess.STDOUT)

    def stop(self):
        try:
            os.killpg(os.getpgid(self.p.pid), signal.SIGTERM)
            self.p.wait(timeout=8)
        except Exception:
            try:
                os.killpg(os.getpgid(self.p.pid), signal.SIGKILL)
            except Exception:
                pass
        self.log.close()

    def log_tail(self, lines=5):
        try:
            with open(self.log_path) as f:
                return [l.rstrip() for l in f.readlines()[-lines:]]
        except OSError:
            return []


class View:
    def __init__(self, rec):
        self.rec = rec

    def stream(self, topic):
        return self.rec.streams.get(topic, [])

    def duration(self):
        return self.rec.elapsed()


def _evidence(rec, procs):
    ev = {'window_elapsed_sec': round(rec.elapsed(), 1)}
    for topic, stream in rec.streams.items():
        ev[topic] = {'count': len(stream),
                     'per_sec': round(len(stream) / max(rec.elapsed(), 1e-9), 1)}
    for topic in ('/odom', '/ground_truth/odom'):
        if rec.streams.get(topic):
            p = rec.streams[topic][-1][1].pose.pose.position
            ev[f'last{topic.replace("/", "_")}'] = [round(p.x, 2), round(p.y, 2)]
    if rec.streams.get('/fleet/task_update'):
        # per task: compressed state history [(sim t, source, state, code)]
        hist = {}
        for t, m in rec.streams['/fleet/task_update']:
            h = hist.setdefault(m.task_id, [])
            entry = (round(t, 1), m.meta.source_id, m.state, m.error_code)
            if not h or h[-1][1:] != entry[1:]:
                h.append(entry)
        ev['task_history'] = hist
    for proc in procs:
        tail = proc.log_tail()
        if tail:
            ev[f'log_{proc.tag}'] = tail
    return ev


def run(path):
    sc = load(path)
    os.environ['ROS_DOMAIN_ID'] = str(random.randint(2, 99))
    rclpy.init()
    rec = None
    procs = []
    result = {'name': sc['name'], 'passed': False, 'failures': [],
              'expect_failure': bool(sc.get('expect_failure')), 'evidence': {}}
    try:
        rec = Recorder()
        for topic, type_name in sc['topics'].items():
            rec.watch(topic, type_name)

        sim = sc.get('sim')
        if sim:
            procs.append(Proc(f"ros2 launch {sim['launch']}",
                              env=sim.get('env'), tag='sim'))
            deadline = time.time() + float(sim.get('ready_timeout_sec', 240))
            while rec.clock_t is None:
                rclpy.spin_once(rec, timeout_sec=0.2)
                if time.time() > deadline:
                    raise ScenarioError(
                        f"sim did not publish /clock within "
                        f"{sim.get('ready_timeout_sec', 240)}s wall time")

        rec.start_window()
        # watchdog: a dead/frozen Gazebo stops publishing /clock, which
        # would freeze elapsed() and hang the suite forever. Fail fast
        # on clock stall instead (wall-time check, sim-present only).
        clock_stall_sec = float(sim.get('clock_stall_sec', 30)) if sim else None
        last_clock_wall = time.time()
        publishers = [rec.make_publisher(a['publish'])
                      for a in sc['actions'] if 'publish' in a]
        pending = sorted(sc['actions'], key=lambda a: a['t'])
        window = float(sc['window_sec'])

        while True:
            rclpy.spin_once(rec, timeout_sec=0.05)
            now = rec.elapsed()
            if clock_stall_sec is not None:
                prev = getattr(rec, '_watch_clock_t', None)
                if rec.clock_t is not None and rec.clock_t != prev:
                    rec._watch_clock_t = rec.clock_t
                    last_clock_wall = time.time()
                if time.time() - last_clock_wall > clock_stall_sec:
                    raise ScenarioError(
                        f"sim stalled: no /clock progress for "
                        f"{clock_stall_sec}s wall time "
                        f"(window was {now:.1f}/{window} sim s)")
            while pending and pending[0]['t'] <= now:
                act = pending.pop(0)
                if 'run' in act:
                    r = act['run']
                    # {mock_data} in args = mounted mock_data/ directory
                    args = str(r.get('args', '')).replace('{mock_data}', MOCK_DATA)
                    procs.append(Proc(
                        f"ros2 run {r['pkg']} {r['exec']} {args}",
                        tag=r.get('tag', r['exec'])))
                elif 'launch' in act:
                    spec = act['launch']
                    procs.append(Proc(f"ros2 launch {spec}",
                                      tag='launch_' + spec.split()[1].split('.')[0]))
            for pub in publishers:
                pub.tick(now)
            if now >= window:
                break

        view = View(rec)
        for a in sc['assert']:
            kind = [k for k in a if k in assertions.REGISTRY][0]
            err = assertions.REGISTRY[kind](view, **dict(a[kind]))
            if err:
                result['failures'].append(f"{kind}: {err}")

        result['passed'] = not result['failures']
        result['evidence'] = _evidence(rec, procs)
    finally:
        for proc in procs:
            proc.stop()
        if rec is not None:
            rec.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result
