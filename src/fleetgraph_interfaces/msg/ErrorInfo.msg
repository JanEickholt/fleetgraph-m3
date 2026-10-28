# One active robot-level error (VDA 5050 errors[] analogue). RobotState
# carries ALL currently active errors; an empty list means none.

uint16 code                     # same code space as TaskUpdate.ERROR_*
uint8 LEVEL_WARNING=0           # robot keeps working
uint8 LEVEL_FATAL=1             # robot cannot continue; status = ERROR
uint8 level
string description
diagnostic_msgs/KeyValue[] references  # e.g. {"task_id": "task-0007"}
