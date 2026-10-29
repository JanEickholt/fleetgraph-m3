#!/usr/bin/env python3
"""Export contract enums + field lists from the .msg/.srv files to
docs/contracts.json — lookup tables for non-ROS consumers (rosbridge sends
enum fields as plain integers; constants are not transmitted).

Usage: python3 tools/export_contracts.py           (pure Python, no ROS needed)
       python3 tools/export_contracts.py --check   verify docs/contracts.json is current
"""
import json
import os
import re
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
PKG = os.path.join(ROOT, 'src', 'fleetgraph_interfaces')
OUT = os.path.join(ROOT, 'docs', 'contracts.json')
CONST = re.compile(r'^(\w+)\s+([A-Z][A-Z0-9_]*)\s*=\s*(\S+)')
FIELD = re.compile(r'^([\w/]+(?:\[\])?)\s+([a-z][a-z0-9_]*)\s*$')


def parse(path):
    fields, enums = [], {}
    for raw in open(path):
        line = raw.split('#', 1)[0].strip()
        if not line or line == '---':
            continue
        m = CONST.match(line)
        if m:
            _, name, value = m.groups()
            prefix = name.split('_', 1)[0]           # STATE_, ERROR_, TYPE_, ...
            enums.setdefault(prefix, {})[int(value)] = name[len(prefix) + 1:]
            continue
        m = FIELD.match(line)
        if m:
            fields.append({'type': m.group(1), 'name': m.group(2)})
    return fields, enums


def main():
    with open(os.path.join(PKG, 'package.xml')) as f:
        version = re.search(r'<version>([^<]+)</version>', f.read()).group(1)
    out = {'schema_version': version, 'messages': {}}
    for sub in ('msg', 'srv'):
        for name in sorted(os.listdir(os.path.join(PKG, sub))):
            fields, enums = parse(os.path.join(PKG, sub, name))
            out['messages'][name.rsplit('.', 1)[0]] = {
                'kind': sub, 'fields': fields,
                # keyed by constant prefix: STATE_* -> "state" (field state),
                # ERROR_* -> "error" (field error_code), LEVEL_* -> "level", ...
                'enums': {k.lower(): v for k, v in enums.items()}}
    text = json.dumps(out, indent=1) + '\n'
    if '--check' in sys.argv[1:]:
        try:
            with open(OUT) as f:
                current = f.read()
        except OSError:
            current = None
        if current != text:
            print('stale generated file: docs/contracts.json '
                  '(run python3 tools/export_contracts.py)')
            return 1
        return 0
    with open(OUT, 'w') as f:
        f.write(text)
    print(f'{OUT}: {len(out["messages"])} types')
    return 0


if __name__ == '__main__':
    sys.exit(main())
