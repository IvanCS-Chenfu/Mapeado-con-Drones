std_msgs/Header header

uint32 drone_id
string drone_name

uint32 fiducial_id

geometry_msgs/PoseStamped local_camera_pose

geometry_msgs/Transform world_T_fiducial
geometry_msgs/Transform camera_T_fiducial

float64 confidence
float64 reprojection_error