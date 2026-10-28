# Directed edge subject --predicate--> object between scene entities.

string subject_id

uint8 PREDICATE_INSIDE=0        # person INSIDE lab_2
uint8 PREDICATE_NEAR=1          # box NEAR door
uint8 PREDICATE_BLOCKS=2        # group BLOCKS corridor
uint8 PREDICATE_OCCUPIES=3      # box OCCUPIES region
uint8 predicate

string object_id
float32 confidence              # 0..1
