# Structured task, produced by Module 1's parser from plain text.

Meta meta
string task_id                  # globally unique, assigned by Module 1 ("task-0007")
string raw_text                 # original operator text, kept for traceability

uint8 TYPE_UNKNOWN=0
uint8 TYPE_GO_TO=1              # locations = [target]
uint8 TYPE_DELIVERY=2           # locations = [pickup, dropoff]
uint8 TYPE_PATROL=3             # locations = [waypoint_1 .. waypoint_n], visited patrol_loops times
uint8 type

Location[] locations

uint8 PRIORITY_LOW=0
uint8 PRIORITY_NORMAL=1
uint8 PRIORITY_HIGH=2
uint8 PRIORITY_URGENT=3
uint8 priority

# constraints
string[] required_capabilities  # controlled vocabulary, see docs/interfaces.md
builtin_interfaces/Time deadline  # zero = no deadline
uint32 patrol_loops             # TYPE_PATROL only; 0 is treated as 1
diagnostic_msgs/KeyValue[] extra_constraints  # open-ended; receivers ignore unknown keys

builtin_interfaces/Time created_at  # when Module 1 received the raw text
