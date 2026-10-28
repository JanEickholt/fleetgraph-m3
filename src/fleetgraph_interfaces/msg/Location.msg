# A task location: a registry id ("lab_2" — shared locations file, later a
# scene-graph PLACE node), a pose, or both. Receivers use the pose when
# pose.header.frame_id is non-empty, otherwise resolve the id.

string id                       # registry id; empty = pose-only location
string label                    # human-readable, display only ("Laboratory 2")
geometry_msgs/PoseStamped pose  # frame "map"; empty frame_id = unresolved
