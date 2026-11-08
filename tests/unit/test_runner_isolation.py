"""Every scenario runs in its own ROS domain and Gazebo partition: never the
container's own domain (manual sims, the Module 2 bridge), never a domain an
earlier scenario of the session used, so leftovers of one never join the
next. No ROS graph: only the choice and the environment it leaves."""
import os
import random

import pytest

import runner


@pytest.fixture
def session(monkeypatch):
    """A fresh session on domains 29..32 in a container on domain 30."""
    monkeypatch.setattr(runner, 'DOMAIN_IDS', range(29, 33))
    monkeypatch.setattr(runner, 'CONTAINER_DOMAIN_ID', '30')
    monkeypatch.setattr(runner, '_used_domain_ids', set())
    monkeypatch.setenv('ROS_DOMAIN_ID', '30')
    monkeypatch.delenv('GZ_PARTITION', raising=False)


def test_domains_skip_the_container_and_do_not_repeat_until_used_up(session):
    rng = random.Random(0)
    first = [runner._isolate('scenario', rng)[0] for _ in range(3)]
    assert sorted(first) == ['29', '31', '32']
    # used up: the next round starts over, still without the container's
    assert runner._isolate('scenario', rng)[0] in {'29', '31', '32'}


def test_the_environment_carries_the_domain_and_a_partition_per_scenario(session):
    domain_id, partition = runner._isolate('weekly_sim_tools', random.Random(1))
    assert os.environ['ROS_DOMAIN_ID'] == domain_id
    assert os.environ['GZ_PARTITION'] == partition
    assert partition == f'weekly_sim_tools_{os.getpid()}_{domain_id}'
    _, other = runner._isolate('weekly sim/tools?', random.Random(1))
    assert other != partition and other.startswith('weekly_sim_tools__')   # gz-safe characters
