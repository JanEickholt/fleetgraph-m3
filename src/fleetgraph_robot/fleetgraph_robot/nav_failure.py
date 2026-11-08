"""Why a Nav2 NavigateToPose goal did not succeed: blocked or failed. Pure
Python (no rclpy, tests/unit/test_nav_failure.py): agent_node classifies each
unsuccessful result here, and the TaskMachine waits out a blocked leg instead
of failing it.

bt_navigator (Nav2 1.3.13, Jazzy) puts the lowest non-zero code among the
blackboard entries named in its error_code_names parameter into the result:
follow_path_error_code (FollowPath, 1xx) and compute_path_error_code
(ComputePathToPose, 2xx) in nav2_params.yaml, so a controller code wins over a
planner code. It resets them after every goal; a goal that fails without
either reports 0. It copies no message: error_msg stays empty in this release
(later ones fill it with the failing node's), so the code alone decides.
"""
from dataclasses import dataclass

BLOCKED = 'blocked'
FAILED = 'failed'


class FollowPathError:
    """Mirror of nav2_msgs/action/FollowPath result codes (checked in tests)."""
    UNKNOWN = 100
    INVALID_CONTROLLER = 101
    TF_ERROR = 102
    INVALID_PATH = 103
    PATIENCE_EXCEEDED = 104
    FAILED_TO_MAKE_PROGRESS = 105
    NO_VALID_CONTROL = 106
    CONTROLLER_TIMED_OUT = 107


class ComputePathError:
    """Mirror of nav2_msgs/action/ComputePathToPose result codes."""
    UNKNOWN = 200
    INVALID_PLANNER = 201
    TF_ERROR = 202
    START_OUTSIDE_MAP = 203
    GOAL_OUTSIDE_MAP = 204
    START_OCCUPIED = 205
    GOAL_OCCUPIED = 206
    TIMEOUT = 207
    NO_VALID_PATH = 208


# Blocked: something occupies the way and may move on, so waiting can help. No
# valid path is what social zones in the keepout mask cause when no route avoids
# them; start or goal occupied, a zone or obstacle covering the robot or its
# goal; the controller codes, the robot held up by something in front of it
# (Regulated Pure Pursuit finds only colliding commands: NO_VALID_CONTROL, or
# PATIENCE_EXCEEDED once that lasts failure_tolerance; FAILED_TO_MAKE_PROGRESS,
# the progress checker sees it stand). Everything else fails, no code included:
# TF, a pose off the map, a bad plugin or path, a timeout or an unknown error
# do not clear by waiting.
BLOCKED_CODES = frozenset({
    ComputePathError.NO_VALID_PATH, ComputePathError.START_OCCUPIED,
    ComputePathError.GOAL_OCCUPIED, FollowPathError.PATIENCE_EXCEEDED,
    FollowPathError.FAILED_TO_MAKE_PROGRESS, FollowPathError.NO_VALID_CONTROL,
})


@dataclass(frozen=True)
class NavFailure:
    kind: str                       # BLOCKED or FAILED
    reason: str                     # code name, then Nav2's message; '' when neither

    @property
    def blocked(self) -> bool:
        return self.kind == BLOCKED


def classify_failure(error_code: int, error_msg: str = '') -> NavFailure:
    """NavigateToPose result fields of a goal that did not succeed."""
    kind = BLOCKED if error_code in BLOCKED_CODES else FAILED
    parts = (_code_name(error_code), error_msg)
    return NavFailure(kind, ': '.join(part for part in parts if part))


def _code_name(error_code):
    if not error_code:
        return ''
    return _CODE_NAMES.get(error_code, f'nav2 error code {error_code}')


def _code_names():
    names = {}
    for server, codes in (('controller', FollowPathError), ('planner', ComputePathError)):
        for name, code in vars(codes).items():
            if name.isupper():
                names[code] = f'{server}: {name.lower().replace("_", " ")} ({code})'
    return names


_CODE_NAMES = _code_names()
