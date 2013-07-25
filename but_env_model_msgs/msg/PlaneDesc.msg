int32 INSERT=0 	# Clear old data, insert new one
int32 MODIFY=1	# Modify plane
int32 REMOVE=2	# Remove data

int32 id
int32 flags 
geometry_msgs/Pose pose
geometry_msgs/Vector3 scale
