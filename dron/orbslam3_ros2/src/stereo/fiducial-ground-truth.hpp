#ifndef ORBSLAM3_ROS2_FIDUCIAL_GROUND_TRUTH_HPP
#define ORBSLAM3_ROS2_FIDUCIAL_GROUND_TRUTH_HPP

#include <sophus/se3.hpp>

#include <map>
#include <string>

namespace orbslam3_ros2
{

// Simulation-only reference geometry for opt-in fiducial diagnostics.
class FiducialGroundTruth
{
public:
    void Load(
        const std::string& objects_config_path,
        const std::string& rendering_config_path);

    bool WorldTTag(int tag_id, Sophus::SE3f* world_t_tag) const;

private:
    std::map<int, Sophus::SE3f> world_t_tags_;
};

}  // namespace orbslam3_ros2

#endif
