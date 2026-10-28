# Module 1 -> robot: out-of-band command for an assigned task
# (VDA 5050 instantActions analogue).

Meta meta
string robot_id
string task_id

uint8 COMMAND_CANCEL=1          # stop navigation, report CANCELLED
uint8 command

string reason                   # copied into the resulting TaskUpdate.reason
