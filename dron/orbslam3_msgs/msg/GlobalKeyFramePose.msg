uint8 STATUS_AVAILABLE=0
uint8 STATUS_PENDING=1
uint8 STATUS_UNKNOWN=2
uint8 STATUS_INVALID_EPOCH=3

std_msgs/Header header
uint32 drone_id
uint64 map_epoch
uint64 keyframe_id
uint8 status
uint64 pose_revision
geometry_msgs/Pose w_t_keyframe
