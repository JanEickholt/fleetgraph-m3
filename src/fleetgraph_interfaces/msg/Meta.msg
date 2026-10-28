# Envelope carried by every FleetGraph contract message (field name: meta).
# Lets every receiver detect lost (seq gap), duplicated (seq repeat),
# reordered and stale (stamp age) events without per-topic logic.
# Prior art: VDA 5050 header (headerId per topic, version), rmf RobotState.seq.

string schema_version           # contract version "MAJOR.MINOR.PATCH"; reject unknown MAJOR
string source_id                # publisher identity: robot id ("robot_1") or module ("m1_orchestrator")
uint64 seq                      # per (source_id, topic): starts at 1, +1 per message sent
builtin_interfaces/Time stamp   # when the event was produced; sim time (/clock) in simulation
