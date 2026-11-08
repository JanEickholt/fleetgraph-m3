"""Simulated object detector (fleetgraph_sim_tools.detector_model). Pure — no
rclpy, no sim. Covers visibility (range, field of view, walls, occlusion by
nearer entities, the observing robot itself, detectable categories), the
sensor-frame transform, Gaussian noise, dropout and seeded determinism.
"""
import math
import random
import statistics

import pytest

from fleetgraph_robot.lidar_obstacles import OccupancyMap
from fleetgraph_robot.scene_graph import Category
from fleetgraph_sim_tools.detector_model import DetectorParams, TruthEntity, simulate

EXACT = DetectorParams(position_sigma=0.0, dropout=0.0, confidence_sigma=0.0)
ORIGIN = (0.0, 0.0, 0.0)                    # sensor at the map origin, looking along +x
# 0.1 m cells over x -1..5, y -1..1; a wall fills the cell column x 2.0..2.1
WALL_MAP = OccupancyMap(0.1, -1.0, -1.0, 60, 20,
                        [100 if col == 30 else 0 for row in range(20) for col in range(60)])
ROBOT_SIZE = (0.4, 0.3, 0.2)


def walker(entity_id, x, y, size=(0.5, 0.5, 1.7)):
    return TruthEntity(entity_id, Category.PERSON, 'person', x, y, z=0.85, size=size)


def seen(truth, pose=ORIGIN, occupancy=None, params=EXACT, seed=0, **options):
    return [detection.truth_id for detection in
            simulate(truth, pose, occupancy, params, random.Random(seed), **options)]


def test_reports_truth_in_the_sensor_frame():
    truth = [walker('ahead', 1.0, 4.0), walker('left', 0.0, 3.0)]
    ahead, left = simulate(truth, (1.0, 2.0, math.pi / 2), None, EXACT, random.Random(0))
    assert (ahead.rel_x, ahead.rel_y) == pytest.approx((2.0, 0.0), abs=1e-12)
    assert (left.rel_x, left.rel_y) == pytest.approx((1.0, 1.0))       # x forward, y left
    assert (ahead.truth_id, ahead.category, ahead.label, ahead.z, ahead.size,
            ahead.confidence) == ('ahead', Category.PERSON, 'person', 0.85, (0.5, 0.5, 1.7), 0.85)


def test_misses_entities_out_of_range_or_outside_the_field_of_view():
    params = DetectorParams(position_sigma=0.0, dropout=0.0, confidence_sigma=0.0,
                            fov_deg=120.0, min_range=0.2, max_range=6.0)
    truth = [walker('ahead', 3.0, 0.0), walker('far', 6.5, 0.0),
             walker('close', 0.1, 0.0, size=(0.0, 0.0, 0.0)),
             walker('edge', 1.0, 1.6),              # bearing 58 deg
             walker('beside', 1.0, 1.8),            # bearing 61 deg
             walker('behind', -2.0, 0.0)]
    assert seen(truth, params=params) == ['ahead', 'edge']


def test_walls_hide_what_is_behind_them():
    truth = [walker('front', 1.5, 0.0), walker('behind_wall', 3.0, 0.8)]
    assert seen(truth, occupancy=WALL_MAP) == ['front']
    assert seen(truth) == ['front', 'behind_wall']          # no map, no walls


def test_nearer_entities_occlude_farther_ones():
    assert seen([walker('far', 4.0, 0.1), walker('near', 2.0, 0.0)]) == ['near']
    slim = (0.3, 0.3, 1.7)                  # the ray to 'far' passes 0.25 m from its centre
    assert seen([walker('far', 4.0, 0.5), walker('near', 2.0, 0.0, size=slim)]) == \
        ['far', 'near']
    # overlapping discs: the one behind covers the front centre but cannot hide it
    assert seen([walker('back', 2.2, 0.0), walker('front', 2.0, 0.0)]) == ['front']


def test_the_observing_robot_is_skipped_and_occludes_nothing():
    truth = [TruthEntity('robot_1', Category.ROBOT, 'robot', 0.0, 0.0, size=ROBOT_SIZE),
             walker('p1', 2.0, 0.0),
             TruthEntity('robot_2', Category.ROBOT, 'robot', 2.0, -1.5, size=ROBOT_SIZE)]
    assert seen(truth, exclude_id='robot_1') == ['p1', 'robot_2']
    assert seen(truth) == []                # the sensor sits inside its own robot's disc


def test_only_detect_categories_are_reported():
    truth = [TruthEntity('meeting', Category.GROUP, 'group', 2.0, 0.0),
             TruthEntity('dock', Category.PLACE, 'dock', 2.0, 1.0),
             walker('p', 2.0, -1.0),
             TruthEntity('box', Category.OBJECT, 'box', 3.0, 0.5, z=0.25, size=(0.4, 0.4, 0.5))]
    assert seen(truth) == ['p', 'box']
    persons_only = DetectorParams(position_sigma=0.0, dropout=0.0, confidence_sigma=0.0,
                                  detect_categories={Category.PERSON})
    assert seen(truth, params=persons_only) == ['p']


def test_same_seed_gives_identical_output():
    truth = [walker(f'p{index}', 2.0 + 0.3 * index, -1.5 + 0.6 * index) for index in range(6)]
    runs = [simulate(truth, ORIGIN, None, DetectorParams(), random.Random(seed))
            for seed in (42, 42, 43)]
    assert runs[0] == runs[1]
    assert runs[0] != runs[2]


def test_dropout_rate_matches_the_parameter():
    params = DetectorParams(dropout=0.2)
    rng = random.Random(9)
    trials = 4000
    detected = sum(len(simulate([walker('p', 2.0, 0.0)], ORIGIN, None, params, rng))
                   for _ in range(trials))
    assert detected / trials == pytest.approx(0.8, abs=0.03)       # 4.7 sigma


def test_position_noise_is_gaussian_with_the_configured_sigma():
    params = DetectorParams(position_sigma=0.1, dropout=0.0)
    rng = random.Random(4)
    detections = [simulate([walker('p', 2.0, 0.0)], ORIGIN, None, params, rng)[0]
                  for _ in range(2000)]
    for values, truth in (([d.rel_x for d in detections], 2.0),
                          ([d.rel_y for d in detections], 0.0)):
        assert statistics.fmean(values) == pytest.approx(truth, abs=0.01)
        assert statistics.stdev(values) == pytest.approx(0.1, abs=0.01)


def test_confidence_is_noisy_but_clipped_to_the_unit_range():
    params = DetectorParams(dropout=0.0, confidence_mean=0.95, confidence_sigma=0.2)
    rng = random.Random(2)
    confidences = [simulate([walker('p', 2.0, 0.0)], ORIGIN, None, params, rng)[0].confidence
                   for _ in range(500)]
    assert min(confidences) >= 0.0 and max(confidences) == 1.0
    assert len(set(confidences)) > 100


@pytest.mark.parametrize('changes, message', [
    (dict(fov_deg=0.0), 'fov_deg'), (dict(fov_deg=400.0), 'fov_deg'),
    (dict(min_range=7.0), 'min_range'), (dict(position_sigma=-0.1), 'position_sigma'),
    (dict(dropout=1.5), 'dropout'), (dict(confidence_mean=1.2), 'confidence_mean'),
    (dict(confidence_sigma=-0.1), 'confidence_sigma'),
])
def test_params_are_validated(changes, message):
    with pytest.raises(ValueError, match=message):
        DetectorParams(**changes)
