std_msgs/Header header

uint32 drone_id
uint64 map_epoch
uint64 local_keyframe_id
uint64 source_revision
uint64 tracking_frame_id
bool valid
string failure_reason

# Transform from the KeyFrame coordinate system to the left optical camera.
geometry_msgs/Pose k_t_camera

float32 fx
float32 fy
float32 cx
float32 cy
float32 baseline_m
float32 min_depth_m
float32 max_depth_m
float32 confidence

# Inspection metadata only. k_t_camera remains the geometric authority.
float64 camera_yaw_rad
float64 camera_pitch_rad
bool normal_valid
geometry_msgs/Vector3 normal_k
uint32 normal_support
float32 normal_confidence

# Filtered depth endpoints in KeyFrame coordinates.
geometry_msgs/Point32[] points_k
