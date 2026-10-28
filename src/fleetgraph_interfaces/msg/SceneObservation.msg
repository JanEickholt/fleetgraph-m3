# Robot -> Module 1: what one robot currently perceives (local scene graph
# excerpt). Entity ids are robot-local track ids, stable across messages,
# so Module 1 can deduplicate repeated observations of the same thing.

Meta meta
string robot_id
string frame_id                 # frame of entity positions, normally "map"
geometry_msgs/PoseStamped sensor_pose
SceneEntity[] entities          # added or re-observed entities
SceneRelation[] relations
string[] removed_entity_ids     # previously reported entities no longer present
float32 confidence              # overall observation confidence 0..1
