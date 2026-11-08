#!/usr/bin/env python3
"""Websocket probe standing in for the Module 2 dashboard: rosbridge JSON
protocol against the bridge of m2_bridge.launch.py. Runs in the container.

  python3 tools/ws_probe.py [--text "Go to the hall"] [--url ws://localhost:9090]

Subscribes /fleet/robot_state and /fleet/task_update over the websocket and
echoes every message, unchanged, to /mock_m2/robot_state and
/mock_m2/task_update through the same websocket. The harness records the
echoes, so rates and task states are measured on what crossed the websocket.
With --text it publishes one plain-text task to /mock_m1/task_text, re-sent
until the mock M1 reports a new live-* task (its id scheme for runtime
text): a publish made before the mock's subscription is matched is lost,
and this must be the only runtime text sender. Runs until killed.
Needs python3-websocket (websocket-client).
"""
import argparse
import collections
import json
import time

import websocket

ECHOES = {  # subscribed topic -> (echo topic, type)
    '/fleet/robot_state': ('/mock_m2/robot_state', 'fleetgraph_interfaces/msg/RobotState'),
    '/fleet/task_update': ('/mock_m2/task_update', 'fleetgraph_interfaces/msg/TaskUpdate'),
}
TEXT_TOPIC = '/mock_m1/task_text'
LIVE_ID_PREFIX = 'live-'
STATE_NAMES = ('RECEIVED', 'PARSED', 'QUEUED', 'ASSIGNED', 'EXECUTING', 'COMPLETED',
               'FAILED', 'CANCELLED', 'REASSIGNED', 'REJECTED')
REPORT_EVERY_SEC = 10.0


def log(text):
    print(f'ws_probe: {text}', flush=True)


def connect(url, timeout_sec):
    """The bridge needs a few seconds after launch: retry until timeout."""
    deadline = time.monotonic() + timeout_sec
    while True:
        try:
            return websocket.create_connection(url, timeout=5)
        except (OSError, websocket.WebSocketException) as error:
            if time.monotonic() > deadline:
                raise SystemExit(f'ws_probe: no rosbridge at {url}: {error}')
            time.sleep(1.0)


class TextTask:
    """The --text task: re-sent until the mock M1 reports a new live-* task,
    then followed through its states."""

    def __init__(self, text, resend_sec, first_send):
        self.text = text
        self.resend_sec = resend_sec
        self.next_send = first_send
        self.sends = 0
        self.task_id = None            # id the mock gave it
        self.state = None
        self.known_task_ids = set()    # seen before the acknowledgement

    def due(self, now):
        return self.task_id is None and now >= self.next_send

    def sent(self, now):
        self.sends += 1
        self.next_send = now + self.resend_sec
        log(f'sent text task {self.text!r} (attempt {self.sends})')

    def on_update(self, msg):
        task_id = msg['task_id']
        if (self.sends and self.task_id is None and task_id.startswith(LIVE_ID_PREFIX) and
                task_id not in self.known_task_ids):
            self.task_id = task_id
            log(f'text task acknowledged as {task_id}')
        self.known_task_ids.add(task_id)
        if task_id == self.task_id and msg['state'] != self.state:
            self.state = msg['state']
            log(f'{task_id} {STATE_NAMES[self.state]} from {msg["meta"]["source_id"]}, '
                f'robot {msg["robot_id"] or "-"} {msg["reason"]}'.rstrip())


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--url', default='ws://localhost:9090')
    parser.add_argument('--text', help='plain-text task to send to the mock M1')
    parser.add_argument('--resend-sec', type=float, default=5.0)
    parser.add_argument('--connect-timeout-sec', type=float, default=60.0)
    args = parser.parse_args()

    ws = connect(args.url, args.connect_timeout_sec)

    def send(**operation):
        ws.send(json.dumps(operation))

    for topic, (echo_topic, msg_type) in ECHOES.items():
        send(op='advertise', topic=echo_topic, type=msg_type)
        send(op='subscribe', topic=topic, type=msg_type)
    if args.text:
        send(op='advertise', topic=TEXT_TOPIC, type='std_msgs/msg/String')
    log(f'connected to {args.url}')
    ws.settimeout(0.5)

    started = time.monotonic()
    next_report = started + REPORT_EVERY_SEC
    # first send once the advertisements have settled
    text_task = TextTask(args.text, args.resend_sec, started + 1.0) if args.text else None
    counts = collections.Counter()     # (topic, source_id) -> messages received
    while True:
        now = time.monotonic()
        if text_task and text_task.due(now):
            send(op='publish', topic=TEXT_TOPIC, msg={'data': text_task.text})
            text_task.sent(now)
        if now >= next_report:
            elapsed = now - started
            log(f'{elapsed:.0f} s: ' + ', '.join(
                f'{topic} {source} {count} ({count / elapsed:.2f}/s)'
                for (topic, source), count in sorted(counts.items())))
            next_report += REPORT_EVERY_SEC
        try:
            message = json.loads(ws.recv())
        except websocket.WebSocketTimeoutException:
            continue
        topic = message.get('topic')
        if message.get('op') != 'publish' or topic not in ECHOES:
            continue
        msg = message['msg']
        send(op='publish', topic=ECHOES[topic][0], msg=msg)
        counts[topic, msg['meta']['source_id']] += 1
        if text_task and topic == '/fleet/task_update':
            text_task.on_update(msg)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
