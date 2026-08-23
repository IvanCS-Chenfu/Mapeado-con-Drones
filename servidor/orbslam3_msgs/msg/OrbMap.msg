std_msgs/Header header

uint32 drone_id
string drone_name
string map_frame
uint64 map_sequence

uint64 map_epoch

float32 fx
float32 fy
float32 cx
float32 cy
float32 bf
uint32 image_width
uint32 image_height

orbslam3_msgs/OrbMapPoint[] mappoints
orbslam3_msgs/OrbKeyFrame[] keyframes