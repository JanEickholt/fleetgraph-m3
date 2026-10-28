# Task lifecycle event. Published by Module 1 (RECEIVED..ASSIGNED,
# REASSIGNED, REJECTED by the parser, operator CANCELLED) and by robots
# (EXECUTING, COMPLETED, FAILED, REJECTED, CANCELLED); meta.source_id tells
# which. One stream for both, so the dashboard rebuilds the whole lifecycle
# from one topic.

Meta meta
string task_id
string robot_id                 # empty while unassigned
uint32 assignment_rev           # echo of TaskAssignment.assignment_rev (0 before assignment)

uint8 STATE_RECEIVED=0
uint8 STATE_PARSED=1
uint8 STATE_QUEUED=2
uint8 STATE_ASSIGNED=3
uint8 STATE_EXECUTING=4         # robot accepted and works on it; repeated with progress
uint8 STATE_COMPLETED=5         # terminal
uint8 STATE_FAILED=6            # terminal for this assignment; Module 1 may reassign
uint8 STATE_CANCELLED=7         # terminal
uint8 STATE_REASSIGNED=8        # Module 1 moved the task to another robot
uint8 STATE_REJECTED=9          # parser or robot refused it (error_code/reason say why)
uint8 state

float32 progress                # 0..1 overall task progress, -1 = unknown
uint32 leg_index                # 0-based navigation leg being executed
uint32 leg_count                # GO_TO 1, DELIVERY 2, PATROL n * loops
float32 distance_remaining_m    # current leg, -1 = unknown

# error_code: 0 = none. 1xx parse/validation (M1), 2xx robot refusal,
# 3xx execution, 4xx operator/fleet. reason = human-readable detail.
uint16 ERROR_NONE=0
uint16 ERROR_PARSE=100
uint16 ERROR_INVALID_TASK=101
uint16 ERROR_ROBOT_BUSY=200
uint16 ERROR_UNKNOWN_LOCATION=201
uint16 ERROR_MISSING_CAPABILITY=202
uint16 ERROR_LOW_BATTERY=203
uint16 ERROR_ROBOT_UNAVAILABLE=204   # robot in maintenance / error state
uint16 ERROR_NAVIGATION_FAILED=300
uint16 ERROR_BLOCKED=301
uint16 ERROR_TIMEOUT=302
uint16 ERROR_CANCELLED_BY_OPERATOR=400
uint16 ERROR_ROBOT_LOST=401          # robot disconnected / failed during execution
uint16 error_code
string reason

bool reassignment_requested     # robot asks Module 1 to give the task to another robot
