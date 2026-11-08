"""tools/hooks/pre-commit on a scratch repository (git only, no ROS): the
generator checks read the working tree, so a commit must not leave a
generator, a generator input or a generated file unstaged, and a staged
docs/contracts.json alone is code. The generators are stubs that pass and
`docker` is a stub that fails, so a hook that gets past the generator
section stops at the container check and never runs the tests.
"""
import os
import shutil
import subprocess

import pytest

HOOK = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'tools', 'hooks',
                    'pre-commit')
PASSING_GENERATOR = 'import sys\nsys.exit(0)\n'
FILES = {
    'tools/gen_world.py': PASSING_GENERATOR,
    'tools/export_contracts.py': PASSING_GENERATOR,
    'src/fleetgraph_sim/worlds/lab_floor.sdf': '<sdf/>\n',
    'src/fleetgraph_interfaces/msg/Meta.msg': 'uint64 seq\n',
    'docs/contracts.json': '{}\n',
    'tests/test_something.py': 'x = 1\n',
}
CONTAINER_CHECK = 'container of this checkout is not running'


def git(repo, *args):
    subprocess.run(['git', '-C', repo, '-c', 'user.name=t', '-c', 'user.email=t@example.com',
                    *args], check=True, capture_output=True)


def write(repo, path, text):
    os.makedirs(os.path.dirname(os.path.join(repo, path)), exist_ok=True)
    with open(os.path.join(repo, path), 'a') as file:
        file.write(text)


@pytest.fixture
def repo(tmp_path):
    if shutil.which('git') is None:
        pytest.skip('git not available')
    root = str(tmp_path / 'repo')
    os.makedirs(root)
    git(root, 'init', '-q')
    for path, text in FILES.items():
        write(root, path, text)
    git(root, 'add', '-A')
    git(root, 'commit', '-q', '-m', 'init')
    stub_bin = tmp_path / 'bin'
    stub_bin.mkdir()
    (stub_bin / 'docker').write_text('#!/bin/sh\nexit 1\n')
    (stub_bin / 'docker').chmod(0o755)
    return root, str(stub_bin)


def run_hook(repo):
    root, stub_bin = repo
    env = dict(os.environ, PATH=f"{stub_bin}{os.pathsep}{os.environ['PATH']}")
    return subprocess.run(['bash', HOOK], cwd=root, env=env, capture_output=True, text=True)


def test_generator_staged_without_its_regenerated_output_fails(repo):
    root, _ = repo
    write(root, 'tools/gen_world.py', '# new wall\n')
    write(root, 'src/fleetgraph_sim/worlds/lab_floor.sdf', '<!-- regenerated -->\n')
    git(root, 'add', 'tools/gen_world.py')
    result = run_hook(repo)
    assert result.returncode == 1
    assert 'unstaged changes to generators' in result.stderr
    assert '  src/fleetgraph_sim/worlds/lab_floor.sdf' in result.stderr
    assert CONTAINER_CHECK not in result.stderr


def test_untracked_generator_input_fails(repo):
    root, _ = repo
    write(root, 'src/fleetgraph_interfaces/msg/New.msg', 'string id\n')
    write(root, 'tests/test_something.py', 'y = 2\n')
    git(root, 'add', 'tests/test_something.py')
    result = run_hook(repo)
    assert result.returncode == 1
    assert '  src/fleetgraph_interfaces/msg/New.msg' in result.stderr


def test_generator_staged_with_its_output_passes_the_generator_checks(repo):
    root, _ = repo
    write(root, 'tools/gen_world.py', '# new wall\n')
    write(root, 'src/fleetgraph_sim/worlds/lab_floor.sdf', '<!-- regenerated -->\n')
    write(root, 'tests/test_something.py', 'y = 2\n')    # other unstaged code only warns
    git(root, 'add', 'tools/gen_world.py', 'src/fleetgraph_sim/worlds/lab_floor.sdf')
    result = run_hook(repo)
    assert 'unstaged changes to generators' not in result.stderr
    assert CONTAINER_CHECK in result.stderr


def test_contracts_json_alone_triggers_the_checks(repo):
    root, _ = repo
    write(root, 'docs/contracts.json', '\n')
    git(root, 'add', 'docs/contracts.json')
    result = run_hook(repo)
    assert 'no code staged' not in result.stdout
    assert CONTAINER_CHECK in result.stderr


def test_docs_only_commit_skips_everything(repo):
    root, _ = repo
    write(root, 'docs/notes.md', 'text\n')
    git(root, 'add', 'docs/notes.md')
    result = run_hook(repo)
    assert result.returncode == 0 and 'no code staged' in result.stdout
