# Module 1 -> robots + dashboard: change to the authoritative global scene
# graph. Deltas are versioned (InteractiveMarkerUpdate / Hydra DsgUpdate
# pattern, but with readable fields instead of a binary blob): apply a delta
# only if base_version equals your current version, otherwise resync via
# the GetSceneGraph service. is_snapshot = full graph that replaces the
# receiver's state (base_version ignored); Module 1 also sends one
# periodically.

Meta meta
string source_robot_id          # robot whose observation caused this update, empty if none
string frame_id                 # normally "map"
uint64 graph_version            # version after applying this message (+1 per delta)
uint64 base_version             # version this delta applies on top of
bool is_snapshot

SceneEntity[] added_nodes
SceneEntity[] updated_nodes
string[] removed_node_ids
SceneRelation[] added_relations
SceneRelation[] removed_relations
