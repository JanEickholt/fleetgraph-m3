# Robot connection state (VDA 5050 "connection" topic analogue), published
# with reliable + transient_local QoS so late joiners get the latest state.
# Robots publish ONLINE on start and OFFLINE on graceful shutdown. ROS has
# no MQTT last will, so CONNECTION_BROKEN is published only by an observer
# (Module 1) after the RobotState heartbeat went stale.

Meta meta
string robot_id

uint8 STATE_ONLINE=0
uint8 STATE_OFFLINE=1
uint8 STATE_CONNECTION_BROKEN=2
uint8 state
