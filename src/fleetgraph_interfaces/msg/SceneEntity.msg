# Node of a local or global 3D scene graph. Field semantics follow
# vision_msgs (label ~ class_id, confidence ~ score, position/size ~
# BoundingBox3D) without depending on it. Spark-DSG layer of each category:
# OBJECTS = OBSTACLE/OBJECT, AGENTS = PERSON/GROUP/ROBOT,
# PLACES = PLACE/TRAVERSABLE.

string id                       # local track id in observations ("robot_1/person/3"), global id in graph updates

uint8 CATEGORY_PLACE=0          # named place / region (room, corridor, dock)
uint8 CATEGORY_TRAVERSABLE=1    # free / traversable area
uint8 CATEGORY_OBSTACLE=2       # static or quasi-static obstacle
uint8 CATEGORY_PERSON=3
uint8 CATEGORY_GROUP=4          # social group; members in member_ids (ROS4HRI Group style)
uint8 CATEGORY_OBJECT=5         # semantic object (door, box, table, ...)
uint8 CATEGORY_ROBOT=6
uint8 category

string label                    # finer class or name ("box", "lab_2")
geometry_msgs/Point position    # centroid, in the frame of the carrying message
geometry_msgs/Vector3 size      # axis-aligned extent in metres, 0 = unknown
geometry_msgs/Vector3 velocity  # m/s, zero for static entities
float32 confidence              # 0..1
builtin_interfaces/Time first_seen
builtin_interfaces/Time last_seen
string[] member_ids             # CATEGORY_GROUP only: ids of member PERSON entities
string[] source_ids             # provenance (robots that observed it); filled by Module 1
diagnostic_msgs/KeyValue[] attributes  # e.g. {"persistence": "temporary"}
