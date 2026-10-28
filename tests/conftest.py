import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# harness modules (runner, recorder, assertions)
sys.path.insert(0, HERE)
# pure robot logic, importable without a colcon build: repo layout on the
# host (../src) and the container layout (~/tests next to ~/ws/src)
for src in (os.path.join(HERE, '..', 'src'), os.path.join(HERE, '..', 'ws', 'src')):
    pkg = os.path.join(src, 'fleetgraph_robot')
    if os.path.isdir(pkg):
        sys.path.insert(0, pkg)
        break
