#include "orbslam3_multi/legacy/GlobalAtlas_antiguo.hpp"

namespace orbslam3_multi
{

void GlobalAtlas::InsertOrUpdateKeyFrame(const ImportedKeyFrame& kf)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (kf.is_bad)
    {
        keyframes_.erase(kf.global_id);
        return;
    }

    keyframes_[kf.global_id] = kf;
}

void GlobalAtlas::InsertOrUpdateMapPoint(const ImportedMapPoint& mp)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (mp.is_bad)
    {
        mappoints_.erase(mp.global_id);
        return;
    }

    mappoints_[mp.global_id] = mp;
}

bool GlobalAtlas::EraseKeyFrame(uint64_t global_kf_id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it =
        keyframes_.find(global_kf_id);

    if (it == keyframes_.end())
        return false;

    keyframes_.erase(it);

    return true;
}


bool GlobalAtlas::EraseMapPoint(uint64_t global_mp_id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it =
        mappoints_.find(global_mp_id);

    if (it == mappoints_.end())
        return false;

    mappoints_.erase(it);

    return true;
}


size_t GlobalAtlas::EraseDrone(uint32_t drone_id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    size_t removed = 0;

    for (auto it = keyframes_.begin(); it != keyframes_.end(); )
    {
        if (it->second.drone_id == drone_id)
        {
            it = keyframes_.erase(it);
            removed++;
        }
        else
        {
            ++it;
        }
    }

    for (auto it = mappoints_.begin(); it != mappoints_.end(); )
    {
        if (it->second.drone_id == drone_id)
        {
            it = mappoints_.erase(it);
            removed++;
        }
        else
        {
            ++it;
        }
    }

    return removed;
}

size_t GlobalAtlas::EraseDroneMap(
    uint32_t drone_id,
    uint64_t map_epoch)
{
    std::lock_guard<std::mutex> lock(mutex_);

    size_t removed = 0;

    for (auto it = keyframes_.begin(); it != keyframes_.end(); )
    {
        if (it->second.drone_id == drone_id &&
            it->second.map_epoch == map_epoch)
        {
            it = keyframes_.erase(it);
            removed++;
        }
        else
        {
            ++it;
        }
    }

    for (auto it = mappoints_.begin(); it != mappoints_.end(); )
    {
        if (it->second.drone_id == drone_id &&
            it->second.map_epoch == map_epoch)
        {
            it = mappoints_.erase(it);
            removed++;
        }
        else
        {
            ++it;
        }
    }

    return removed;
}

void GlobalAtlas::RemoveKeyFrame(uint64_t global_kf_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    keyframes_.erase(global_kf_id);
}

void GlobalAtlas::RemoveMapPoint(uint64_t global_mp_id)
{
    std::lock_guard<std::mutex> lock(mutex_);
    mappoints_.erase(global_mp_id);
}

bool GlobalAtlas::HasKeyFrame(uint64_t global_kf_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return keyframes_.find(global_kf_id) != keyframes_.end();
}

bool GlobalAtlas::HasMapPoint(uint64_t global_mp_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return mappoints_.find(global_mp_id) != mappoints_.end();
}

ImportedKeyFrame GlobalAtlas::GetKeyFrame(uint64_t global_kf_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = keyframes_.find(global_kf_id);

    if (it == keyframes_.end())
        return ImportedKeyFrame();

    return it->second;
}

ImportedMapPoint GlobalAtlas::GetMapPoint(uint64_t global_mp_id) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = mappoints_.find(global_mp_id);

    if (it == mappoints_.end())
        return ImportedMapPoint();

    return it->second;
}

std::vector<ImportedKeyFrame> GlobalAtlas::GetAllKeyFrames() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<ImportedKeyFrame> out;
    out.reserve(keyframes_.size());

    for (const auto& [id, kf] : keyframes_)
    {
        (void)id;
        out.push_back(kf);
    }

    return out;
}

std::vector<ImportedMapPoint> GlobalAtlas::GetAllMapPoints() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<ImportedMapPoint> out;
    out.reserve(mappoints_.size());

    for (const auto& [id, mp] : mappoints_)
    {
        (void)id;
        out.push_back(mp);
    }

    return out;
}

AtlasCounts GlobalAtlas::GetCounts() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    AtlasCounts counts;
    counts.keyframes = keyframes_.size();
    counts.mappoints = mappoints_.size();

    return counts;
}

void GlobalAtlas::ClearDroneMap(uint32_t drone_id, uint64_t map_epoch)
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto it = keyframes_.begin(); it != keyframes_.end(); )
    {
        if (it->second.drone_id == drone_id &&
            it->second.map_epoch == map_epoch)
        {
            it = keyframes_.erase(it);
        }
        else
        {
            ++it;
        }
    }

    for (auto it = mappoints_.begin(); it != mappoints_.end(); )
    {
        if (it->second.drone_id == drone_id &&
            it->second.map_epoch == map_epoch)
        {
            it = mappoints_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

}  // namespace orbslam3_multi
