uint8 DEPTH_EMERGENCY=0
uint8 TRACKING_RISK=1
uint8 STOPPED=2
uint8 VISUAL_RETREAT_STARTED=3
uint8 VISUAL_RETREAT_FINISHED=4
uint8 REPLAN_REQUIRED=5

std_msgs/Header header
uint32 drone_id
string task_id
string trajectory_id
uint8 event_type
uint64 map_epoch
uint64 alignment_revision
geometry_msgs/Pose stopped_pose_world
string detail
