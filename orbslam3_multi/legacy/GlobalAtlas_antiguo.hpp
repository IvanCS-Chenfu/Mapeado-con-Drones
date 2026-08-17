#pragma once

#include "orbslam3_multi/legacy/ImportedKeyFrame_antiguo.hpp"
#include "orbslam3_multi/legacy/ImportedMapPoint_antiguo.hpp"

#include <unordered_map>
#include <vector>
#include <mutex>
#include <cstdint>

namespace orbslam3_multi
{

struct AtlasCounts
{
    size_t keyframes = 0;
    size_t mappoints = 0;
    size_t bad_keyframes = 0;
    size_t bad_mappoints = 0;
};

class GlobalAtlas
{
public:
    GlobalAtlas() = default;

    void InsertOrUpdateKeyFrame(const ImportedKeyFrame& kf);
    void InsertOrUpdateMapPoint(const ImportedMapPoint& mp);
    bool EraseKeyFrame(uint64_t global_kf_id);
    bool EraseMapPoint(uint64_t global_mp_id);
    size_t EraseDrone(uint32_t drone_id);
    size_t EraseDroneMap(
        uint32_t drone_id,
        uint64_t map_epoch);

    void RemoveKeyFrame(uint64_t global_kf_id);
    void RemoveMapPoint(uint64_t global_mp_id);

    bool HasKeyFrame(uint64_t global_kf_id) const;
    bool HasMapPoint(uint64_t global_mp_id) const;

    ImportedKeyFrame GetKeyFrame(uint64_t global_kf_id) const;
    ImportedMapPoint GetMapPoint(uint64_t global_mp_id) const;

    std::vector<ImportedKeyFrame> GetAllKeyFrames() const;
    std::vector<ImportedMapPoint> GetAllMapPoints() const;

    AtlasCounts GetCounts() const;

    void ClearDroneMap(uint32_t drone_id, uint64_t map_epoch);

private:
    mutable std::mutex mutex_;

    std::unordered_map<uint64_t, ImportedKeyFrame> keyframes_;
    std::unordered_map<uint64_t, ImportedMapPoint> mappoints_;
};

}  // namespace orbslam3_multi
