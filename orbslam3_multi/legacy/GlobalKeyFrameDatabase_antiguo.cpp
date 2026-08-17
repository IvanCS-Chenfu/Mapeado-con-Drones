#include "orbslam3_multi/legacy/GlobalKeyFrameDatabase_antiguo.hpp"

#include <algorithm>
#include <unordered_set>
#include <cmath>

namespace orbslam3_multi
{

GlobalKeyFrameDatabase::GlobalKeyFrameDatabase(
    std::shared_ptr<GlobalAtlas> atlas)
    : atlas_(std::move(atlas))
{
}

void GlobalKeyFrameDatabase::AddOrUpdateKeyFrame(
    const ImportedKeyFrame& kf)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto erase_keyframe_no_lock =
        [this](uint64_t global_kf_id)
        {
            auto words_it =
                words_by_keyframe_.find(global_kf_id);

            if (words_it != words_by_keyframe_.end())
            {
                for (uint32_t word_id : words_it->second)
                {
                    auto inv_it =
                        inverted_file_.find(word_id);

                    if (inv_it == inverted_file_.end())
                        continue;

                    auto& vec =
                        inv_it->second;

                    vec.erase(
                        std::remove(
                            vec.begin(),
                            vec.end(),
                            global_kf_id),
                        vec.end());

                    if (vec.empty())
                    {
                        inverted_file_.erase(inv_it);
                    }
                }

                words_by_keyframe_.erase(words_it);
            }

            keyframes_.erase(global_kf_id);
        };

    if (kf.is_bad)
    {
        erase_keyframe_no_lock(kf.global_id);
        return;
    }

    erase_keyframe_no_lock(kf.global_id);

    keyframes_[kf.global_id] =
        kf;

    std::vector<uint32_t> words;
    words.reserve(kf.bow_word_ids.size());

    for (uint32_t word_id : kf.bow_word_ids)
    {
        inverted_file_[word_id].push_back(kf.global_id);
        words.push_back(word_id);
    }

    words_by_keyframe_[kf.global_id] =
        std::move(words);
}

void GlobalKeyFrameDatabase::EraseKeyFrame(
    uint64_t global_kf_id)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto words_it = words_by_keyframe_.find(global_kf_id);

    if (words_it != words_by_keyframe_.end())
    {
        for (uint32_t word_id : words_it->second)
        {
            auto inv_it = inverted_file_.find(word_id);

            if (inv_it == inverted_file_.end())
                continue;

            auto& vec = inv_it->second;

            vec.erase(
                std::remove(vec.begin(), vec.end(), global_kf_id),
                vec.end());

            if (vec.empty())
            {
                inverted_file_.erase(inv_it);
            }
        }

        words_by_keyframe_.erase(words_it);
    }

    keyframes_.erase(global_kf_id);
}

void GlobalKeyFrameDatabase::Clear()
{
    std::lock_guard<std::mutex> lock(mutex_);

    keyframes_.clear();
    inverted_file_.clear();
    words_by_keyframe_.clear();
}

size_t GlobalKeyFrameDatabase::Size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return keyframes_.size();
}

size_t GlobalKeyFrameDatabase::InvertedIndexWords() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return inverted_file_.size();
}

int GlobalKeyFrameDatabase::CountSharedWords(
    const ImportedKeyFrame& a,
    const ImportedKeyFrame& b) const
{
    if (a.bow_word_ids.empty() || b.bow_word_ids.empty())
        return 0;

    const std::vector<uint32_t>* small = &a.bow_word_ids;
    const std::vector<uint32_t>* large = &b.bow_word_ids;

    if (small->size() > large->size())
        std::swap(small, large);

    std::unordered_set<uint32_t> words_large;
    words_large.reserve(large->size());

    for (uint32_t w : *large)
        words_large.insert(w);

    int shared = 0;

    for (uint32_t w : *small)
    {
        if (words_large.find(w) != words_large.end())
            shared++;
    }

    return shared;
}

double GlobalKeyFrameDatabase::ComputeBowScore(
    const ImportedKeyFrame& a,
    const ImportedKeyFrame& b) const
{
    if (a.bow_word_ids.empty() || b.bow_word_ids.empty())
        return 0.0;

    std::unordered_map<uint32_t, float> bow_b;
    bow_b.reserve(b.bow_word_ids.size());

    for (size_t i = 0; i < b.bow_word_ids.size(); ++i)
    {
        if (i >= b.bow_word_values.size())
            break;

        bow_b[b.bow_word_ids[i]] = b.bow_word_values[i];
    }

    double score = 0.0;

    for (size_t i = 0; i < a.bow_word_ids.size(); ++i)
    {
        if (i >= a.bow_word_values.size())
            break;

        const uint32_t word_id = a.bow_word_ids[i];
        const float value_a = a.bow_word_values[i];

        auto it = bow_b.find(word_id);

        if (it == bow_b.end())
            continue;

        const float value_b = it->second;

        // Aproximación equivalente a intersección de histogramas BoW.
        // Si los BowVector están normalizados, esto funciona bien como score.
        score += std::min<double>(value_a, value_b);
    }

    return score;
}

std::vector<uint64_t> GlobalKeyFrameDatabase::GetStrongCovisibleKeyFrames(
    const ImportedKeyFrame& kf,
    int max_neighbors) const
{
    std::vector<std::pair<uint64_t, uint32_t>> weighted;

    const size_t n =
        std::min(
            kf.connected_keyframe_ids.size(),
            kf.connected_keyframe_weights.size());

    weighted.reserve(n);

    for (size_t i = 0; i < n; ++i)
    {
        weighted.push_back(
            {kf.connected_keyframe_ids[i], kf.connected_keyframe_weights[i]});
    }

    std::sort(
        weighted.begin(),
        weighted.end(),
        [](const auto& a, const auto& b)
        {
            return a.second > b.second;
        });

    std::vector<uint64_t> result;

    for (const auto& [kf_id, weight] : weighted)
    {
        (void)weight;

        if (static_cast<int>(result.size()) >= max_neighbors)
            break;

        result.push_back(kf_id);
    }

    return result;
}

std::vector<BowCandidate> GlobalKeyFrameDatabase::DetectCandidates(
    uint64_t query_global_kf_id,
    const BowQueryParams& params) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<BowCandidate> result;

    auto query_it = keyframes_.find(query_global_kf_id);

    if (query_it == keyframes_.end())
        return result;

    const ImportedKeyFrame& query_kf = query_it->second;

    if (query_kf.is_bad || query_kf.bow_word_ids.empty())
        return result;

    // 1. Buscar keyframes que comparten palabras.
    std::unordered_map<uint64_t, int> shared_words_by_kf;

    for (uint32_t word_id : query_kf.bow_word_ids)
    {
        auto inv_it = inverted_file_.find(word_id);

        if (inv_it == inverted_file_.end())
            continue;

        for (uint64_t candidate_id : inv_it->second)
        {
            if (candidate_id == query_global_kf_id)
                continue;

            shared_words_by_kf[candidate_id]++;
        }
    }

    if (shared_words_by_kf.empty())
        return result;

    // 2. ORB-SLAM suele filtrar por un porcentaje del máximo número de palabras comunes.
    int max_common_words = 0;

    for (const auto& [candidate_id, shared] : shared_words_by_kf)
    {
        (void)candidate_id;
        max_common_words = std::max(max_common_words, shared);
    }

    const int min_words_from_max =
        std::max(
            params.min_common_words,
            static_cast<int>(std::floor(0.8 * static_cast<double>(max_common_words))));

    std::vector<BowCandidate> candidates;

    for (const auto& [candidate_id, shared] : shared_words_by_kf)
    {
        if (shared < min_words_from_max)
            continue;

        auto cand_it = keyframes_.find(candidate_id);

        if (cand_it == keyframes_.end())
            continue;

        const ImportedKeyFrame& candidate_kf = cand_it->second;

        if (candidate_kf.is_bad)
            continue;

        const bool same_drone =
            candidate_kf.drone_id == query_kf.drone_id;

        if (same_drone && !params.enable_intra_drone)
            continue;

        if (!same_drone && !params.enable_inter_drone)
            continue;

        if (same_drone)
        {
            const uint64_t gap =
                candidate_kf.local_id > query_kf.local_id ?
                candidate_kf.local_id - query_kf.local_id :
                query_kf.local_id - candidate_kf.local_id;

            if (gap < params.min_intra_drone_kf_gap)
                continue;
        }

        const double score =
            ComputeBowScore(query_kf, candidate_kf);

        if (score < params.min_bow_score)
            continue;

        BowCandidate c;

        c.query_kf_id = query_kf.global_id;
        c.candidate_kf_id = candidate_kf.global_id;

        c.query_drone_id = query_kf.drone_id;
        c.candidate_drone_id = candidate_kf.drone_id;

        c.query_local_kf_id = query_kf.local_id;
        c.candidate_local_kf_id = candidate_kf.local_id;

        c.shared_words = shared;
        c.bow_score = score;
        c.accumulated_score = score;
        c.same_drone = same_drone;

        candidates.push_back(c);
    }

    if (candidates.empty())
        return result;

    // 3. Acumulación por covisibilidad, parecida a ORB-SLAM3.
    if (params.use_covisibility_accumulation)
    {
        for (auto& candidate : candidates)
        {
            auto cand_it =
                keyframes_.find(candidate.candidate_kf_id);

            if (cand_it == keyframes_.end())
                continue;

            const ImportedKeyFrame& cand_kf =
                cand_it->second;

            std::vector<uint64_t> covisible =
                GetStrongCovisibleKeyFrames(
                    cand_kf,
                    params.covisibility_neighbors_for_score);

            double accumulated = candidate.bow_score;

            for (uint64_t neigh_id : covisible)
            {
                auto neigh_it = keyframes_.find(neigh_id);

                if (neigh_it == keyframes_.end())
                    continue;

                const ImportedKeyFrame& neigh_kf =
                    neigh_it->second;

                if (neigh_kf.is_bad)
                    continue;

                accumulated +=
                    ComputeBowScore(query_kf, neigh_kf);
            }

            candidate.accumulated_score = accumulated;
        }
    }

    // 4. Ordenar por score acumulado.
    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const BowCandidate& a, const BowCandidate& b)
        {
            if (a.accumulated_score == b.accumulated_score)
                return a.shared_words > b.shared_words;

            return a.accumulated_score > b.accumulated_score;
        });

    for (const auto& c : candidates)
    {
        if (static_cast<int>(result.size()) >= params.max_candidates)
            break;

        result.push_back(c);
    }

    return result;
}


void GlobalKeyFrameDatabase::ClearDrone(uint32_t drone_id)
{
    if (drone_id == 0)
        return;

    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<uint64_t> keyframes_to_remove;

    keyframes_to_remove.reserve(keyframes_.size());

    // ============================================================
    // 1. Buscar todos los KeyFrames de ese dron dentro de la DB BoW
    // ============================================================

    for (const auto& [global_kf_id, kf] : keyframes_)
    {
        if (kf.drone_id == drone_id)
        {
            keyframes_to_remove.push_back(global_kf_id);
        }
    }

    if (keyframes_to_remove.empty())
        return;

    std::unordered_set<uint64_t> remove_set(
        keyframes_to_remove.begin(),
        keyframes_to_remove.end());

    // ============================================================
    // 2. Eliminar de keyframes_
    // ============================================================

    for (uint64_t global_kf_id : keyframes_to_remove)
    {
        keyframes_.erase(global_kf_id);
    }

    // ============================================================
    // 3. Eliminar de words_by_keyframe_
    // ============================================================

    for (uint64_t global_kf_id : keyframes_to_remove)
    {
        words_by_keyframe_.erase(global_kf_id);
    }

    // ============================================================
    // 4. Eliminar de inverted_file_
    //
    // inverted_file_:
    //   word_id -> vector<global_kf_id>
    // ============================================================

    for (auto& [word_id, kf_ids] : inverted_file_)
    {
        (void)word_id;

        kf_ids.erase(
            std::remove_if(
                kf_ids.begin(),
                kf_ids.end(),
                [&remove_set](uint64_t global_kf_id)
                {
                    return remove_set.find(global_kf_id) != remove_set.end();
                }),
            kf_ids.end());
    }

    // ============================================================
    // 5. Borrar palabras que se han quedado vacías
    // ============================================================

    for (auto it = inverted_file_.begin(); it != inverted_file_.end(); )
    {
        if (it->second.empty())
        {
            it = inverted_file_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

void GlobalKeyFrameDatabase::ClearDroneEpoch(
    uint32_t drone_id,
    uint64_t map_epoch)
{
    if (drone_id == 0)
        return;

    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<uint64_t> keyframes_to_remove;
    keyframes_to_remove.reserve(keyframes_.size());

    for (const auto& [global_kf_id, kf] : keyframes_)
    {
        if (kf.drone_id == drone_id &&
            kf.map_epoch == map_epoch)
        {
            keyframes_to_remove.push_back(global_kf_id);
        }
    }

    if (keyframes_to_remove.empty())
        return;

    std::unordered_set<uint64_t> remove_set(
        keyframes_to_remove.begin(),
        keyframes_to_remove.end());

    for (uint64_t global_kf_id : keyframes_to_remove)
    {
        keyframes_.erase(global_kf_id);
        words_by_keyframe_.erase(global_kf_id);
    }

    for (auto& [word_id, kf_ids] : inverted_file_)
    {
        (void)word_id;

        kf_ids.erase(
            std::remove_if(
                kf_ids.begin(),
                kf_ids.end(),
                [&remove_set](uint64_t global_kf_id)
                {
                    return remove_set.find(global_kf_id) != remove_set.end();
                }),
            kf_ids.end());
    }

    for (auto it = inverted_file_.begin(); it != inverted_file_.end(); )
    {
        if (it->second.empty())
        {
            it = inverted_file_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

}  // namespace orbslam3_multi
