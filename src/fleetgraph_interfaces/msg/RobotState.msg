# Robot telemetry and heartbeat, published periodically (nominal 2 Hz),
# also when idle. Receivers mark a robot DISCONNECTED when no RobotState
# arrived within their stale timeout; robots never publish
# STATUS_DISCONNECTED themselves.

Meta meta
string robot_id
geometry_msgs/PoseStamped pose  # localized pose, frame "map" (REP-105)
geometry_msgs/Twist velocity    # body frame
float32 localization_score      # 0..1 (from AMCL covariance), -1 = unknown

float32 battery                 # state of charge 0..1 (BatteryState.percentage), NaN = unknown
bool charging

uint8 STATUS_AVAILABLE=0
uint8 STATUS_BUSY=1
uint8 STATUS_MAINTENANCE=2
uint8 STATUS_DISCONNECTED=3     # receiver-derived only
uint8 STATUS_ERROR=4
uint8 status

string[] capabilities           # controlled vocabulary, see docs/interfaces.md
string current_task_id          # empty when idle
float32 connection_quality      # 0..1 robot-side link estimate, -1 = unknown
ErrorInfo[] errors              # all active errors; empty = none
