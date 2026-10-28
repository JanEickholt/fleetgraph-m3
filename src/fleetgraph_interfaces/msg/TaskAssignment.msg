# Module 1 -> robot: execute this task. One active task per robot.
# The robot answers on /fleet/task_update: EXECUTING = accepted,
# REJECTED + error_code/reason = refused. Module 1 re-sends if no answer
# arrives within its ack timeout. Robots treat a repeat of the same
# (task_id, assignment_rev) as idempotent and drop a lower assignment_rev
# as outdated (VDA 5050 orderId/orderUpdateId semantics).

Meta meta
string robot_id                 # addressee; all other robots ignore the message
uint32 assignment_rev           # starts at 1, +1 on every re-assignment of task_id
TaskRequest task
