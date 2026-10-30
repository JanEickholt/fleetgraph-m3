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
OUTPUT_PATH = os.path.join(ROOT, 'docs', 'contracts.json')
CONSTANT = re.compile(r'^(\w+)\s+([A-Z][A-Z0-9_]*)\s*=\s*(\S+)')
FIELD = re.compile(r'^([\w/]+(?:\[\])?)\s+([a-z][a-z0-9_]*)\s*$')


def parse(path):
    fields, enums = [], {}
    with open(path) as file:
        raw_lines = file.readlines()
    for raw_line in raw_lines:
        line = raw_line.split('#', 1)[0].strip()
        if not line or line == '---':
            continue
        match = CONSTANT.match(line)
        if match:
            _, name, value = match.groups()
            prefix = name.split('_', 1)[0]           # STATE_, ERROR_, TYPE_, ...
            enums.setdefault(prefix, {})[int(value)] = name[len(prefix) + 1:]
            continue
        match = FIELD.match(line)
        if match:
            fields.append({'type': match.group(1), 'name': match.group(2)})
    return fields, enums


def main():
    with open(os.path.join(PKG, 'package.xml')) as file:
        version = re.search(r'<version>([^<]+)</version>', file.read()).group(1)
    contracts = {'schema_version': version, 'messages': {}}
    for kind in ('msg', 'srv'):
        for filename in sorted(os.listdir(os.path.join(PKG, kind))):
            fields, enums = parse(os.path.join(PKG, kind, filename))
            contracts['messages'][filename.rsplit('.', 1)[0]] = {
                'kind': kind, 'fields': fields,
                # keyed by constant prefix: STATE_* -> "state" (field state),
                # ERROR_* -> "error" (field error_code), LEVEL_* -> "level", ...
                'enums': {prefix.lower(): values for prefix, values in enums.items()}}
    text = json.dumps(contracts, indent=1) + '\n'
    if '--check' in sys.argv[1:]:
        try:
            with open(OUTPUT_PATH) as file:
                current = file.read()
        except OSError:
            current = None
        if current != text:
            print('stale generated file: docs/contracts.json '
                  '(run python3 tools/export_contracts.py)')
            return 1
        return 0
    with open(OUTPUT_PATH, 'w') as file:
        file.write(text)
    print(f'{OUTPUT_PATH}: {len(contracts["messages"])} types')
    return 0


if __name__ == '__main__':
    sys.exit(main())
