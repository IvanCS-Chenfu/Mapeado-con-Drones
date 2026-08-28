# Estado coherente de navegacion producido por una unica llamada TrackStereo.
int8 TRACKING_SYSTEM_NOT_READY=-1
int8 TRACKING_NO_IMAGES_YET=0
int8 TRACKING_NOT_INITIALIZED=1
int8 TRACKING_OK=2
int8 TRACKING_RECENTLY_LOST=3
int8 TRACKING_LOST=4
int8 TRACKING_OK_KLT=5

uint8 POSE_SOURCE_INVALID=0
uint8 POSE_SOURCE_ORB=1
uint8 POSE_SOURCE_GLOBAL=2
uint8 POSE_SOURCE_GT_FALLBACK=3

uint8 GLOBAL_STATUS_INVALID=0
uint8 GLOBAL_STATUS_PROVISIONAL=1
uint8 GLOBAL_STATUS_AUTHORITATIVE=2

std_msgs/Header header
string child_frame_id
uint32 drone_id
uint64 sample_sequence
uint64 map_epoch
int8 tracking_state
uint8 pose_source
uint8 global_status
uint64 pose_revision

bool local_valid
bool local_continuity_valid
bool global_valid
bool velocity_valid

bool reference_keyframe_valid
uint64 reference_keyframe_id
geometry_msgs/Pose tcr

geometry_msgs/Pose o_t_body
geometry_msgs/Pose w_t_body
geometry_msgs/Twist velocity
