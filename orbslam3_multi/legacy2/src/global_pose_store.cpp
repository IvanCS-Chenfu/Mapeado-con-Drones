#include "orbslam3_multi/global_pose_store.hpp"

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose.hpp>

#include <cmath>
#include <set>
#include <sstream>

namespace orbslam3_multi
{
namespace
{

bool IsFiniteTransform(const Eigen::Matrix4d& transform)
{
    return transform.allFinite();
}

bool IsPartialOptimizationSource(const std::string& source)
{
    return source.find("SERVER_OPTIMIZATION_PARTIAL") != std::string::npos;
}

bool IsNearIdentity(const Eigen::Matrix4d& transform)
{
    return (transform - Eigen::Matrix4d::Identity()).norm() <= 1e-9;
}

void SetReason(std::string* reason, const std::string& value)
{
    if (reason)
    {
        *reason = value;
    }
}

bool PoseMsgToMatrix(const geometry_msgs::msg::Pose& pose, Eigen::Matrix4d& out)
{
    const Eigen::Quaterniond q_raw(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);

    if (!std::isfinite(q_raw.w()) ||
        !std::isfinite(q_raw.x()) ||
        !std::isfinite(q_raw.y()) ||
        !std::isfinite(q_raw.z()) ||
        q_raw.norm() < 1e-9)
    {
        return false;
    }

    Eigen::Quaterniond q = q_raw.normalized();
    out = Eigen::Matrix4d::Identity();
    out.block<3, 3>(0, 0) = q.toRotationMatrix();
    out(0, 3) = pose.position.x;
    out(1, 3) = pose.position.y;
    out(2, 3) = pose.position.z;
    return out.allFinite();
}

Eigen::Matrix3d BuildBodyCameraRotation(const BodyCameraTransformConfig& config)
{
    // F1F-hotfix: replica la convencion legacy usada por global_pose_corrector.
    // Cuando se activa el frame optico, los RPY se ignoran y se usa la matriz
    // estandar body(x forward,y left,z up) -> camera optical(z forward,x right,y down).
    if (config.use_camera_optical_frame_convention)
    {
        Eigen::Matrix3d rotation;
        rotation << 0.0, 0.0, 1.0,
                   -1.0, 0.0, 0.0,
                    0.0, -1.0, 0.0;
        return rotation;
    }

    constexpr double kPi = 3.14159265358979323846;
    const double deg_to_rad = kPi / 180.0;
    const Eigen::AngleAxisd roll(config.roll_deg * deg_to_rad, Eigen::Vector3d::UnitX());
    const Eigen::AngleAxisd pitch(config.pitch_deg * deg_to_rad, Eigen::Vector3d::UnitY());
    const Eigen::AngleAxisd yaw(config.yaw_deg * deg_to_rad, Eigen::Vector3d::UnitZ());
    return (yaw * pitch * roll).toRotationMatrix();
}

RawSubmapId SubmapOf(const RawKeyFrameId& keyframe_id)
{
    return RawSubmapId{keyframe_id.drone_id, keyframe_id.map_epoch};
}

}  // namespace

const char* ToString(GlobalPoseNewKeyFrameStatus status)
{
    switch (status)
    {
        case GlobalPoseNewKeyFrameStatus::MissingRawKeyFrame:
            return "missing_raw_keyframe";
        case GlobalPoseNewKeyFrameStatus::UnanchoredSubmap:
            return "unanchored_submap";
        case GlobalPoseNewKeyFrameStatus::AlreadyHadPose:
            return "already_had_pose";
        case GlobalPoseNewKeyFrameStatus::PoseSet:
            return "pose_set";
        case GlobalPoseNewKeyFrameStatus::CorrectionInherited:
            return "correction_inherited";
    }
    return "unknown";
}

GlobalPoseStore::GlobalPoseStore()
{
    ConfigureBodyCameraTransform(body_camera_config_);
}

void GlobalPoseStore::ConfigureBodyCameraTransform(
    const BodyCameraTransformConfig& config)
{
    // F1F-hotfix: el fiducial simulado da pose de cuerpo del dron, mientras
    // ORB-SLAM3 expresa KFs/MPs en frame de camara. Guardamos aqui la extrinseca
    // para que cualquier anchor derivado use world_T_camera = world_T_body * body_T_camera.
    body_camera_config_ = config;
    body_T_camera_ = Eigen::Matrix4d::Identity();
    body_T_camera_.block<3, 3>(0, 0) = BuildBodyCameraRotation(config);
    body_T_camera_(0, 3) = config.x;
    body_T_camera_(1, 3) = config.y;
    body_T_camera_(2, 3) = config.z;
}

Eigen::Matrix4d GlobalPoseStore::GetBodyCameraTransform() const
{
    return body_T_camera_;
}

Eigen::Matrix4d GlobalPoseStore::TransformBodyPoseToCameraPose(
    const Eigen::Matrix4d& world_T_body) const
{
    return world_T_body * body_T_camera_;
}

BodyCameraTransformConfig GlobalPoseStore::GetBodyCameraTransformConfig() const
{
    return body_camera_config_;
}

GlobalPoseAnchorResult GlobalPoseStore::AnchorSubmap(const RawSubmapId& submap_id,
                                                     const Eigen::Matrix4d& world_T_local,
                                                     const RawMapDatabase& raw_db,
                                                     const std::string& source)
{
    // F1D: el anchor es la unica entrada legitima para poblar poses world desde
    // datos raw. Si no existe el submapa o la transformacion no es valida, no
    // se crea estado parcial.
    GlobalPoseAnchorResult result;
    if (!raw_db.HasSubmap(submap_id))
    {
        result.reason = "raw_submap_missing";
        return result;
    }
    if (!IsFiniteTransform(world_T_local))
    {
        result.reason = "world_T_local_not_finite";
        return result;
    }

    result.replaced_existing_anchor =
        submap_anchors_.find(submap_id) != submap_anchors_.end();
    submap_anchors_[submap_id] = SubmapAnchor{world_T_local, source};

    const auto keyframe_ids = raw_db.GetKeyFrameIdsForSubmap(submap_id);
    for (const auto& keyframe_id : keyframe_ids)
    {
        Eigen::Matrix4d raw_world_T_kf = Eigen::Matrix4d::Identity();
        std::string reason;
        if (!ComputeRawWorldPose(keyframe_id, raw_db, raw_world_T_kf, &reason))
        {
            continue;
        }

        keyframe_world_poses_[keyframe_id] =
            KeyFrameWorldPose{raw_world_T_kf, source, false};
        accepted_keyframe_anchors_[keyframe_id] =
            AcceptedKeyFrameAnchor{raw_world_T_kf, source, 0};
        ++result.anchored_keyframes;
    }

    result.success = true;
    result.reason = "anchor_set";
    return result;
}

bool GlobalPoseStore::HasSubmapAnchor(const RawSubmapId& submap_id) const
{
    return submap_anchors_.find(submap_id) != submap_anchors_.end();
}

std::optional<Eigen::Matrix4d> GlobalPoseStore::GetSubmapWorldTransform(
    const RawSubmapId& submap_id) const
{
    // F1F: consulta de solo lectura para publicar MapPoints de submapas
    // anclados. No crea anchors ni registra poses nuevas.
    const auto it = submap_anchors_.find(submap_id);
    if (it == submap_anchors_.end())
    {
        return std::nullopt;
    }
    return it->second.world_T_local;
}

bool GlobalPoseStore::HasWorldPose(const RawKeyFrameId& keyframe_id) const
{
    return keyframe_world_poses_.find(keyframe_id) != keyframe_world_poses_.end();
}

std::optional<Eigen::Matrix4d> GlobalPoseStore::GetWorldPose(
    const RawKeyFrameId& keyframe_id) const
{
    const auto it = keyframe_world_poses_.find(keyframe_id);
    if (it == keyframe_world_poses_.end())
    {
        return std::nullopt;
    }
    return it->second.world_T_kf;
}

bool GlobalPoseStore::MarkHardFiducialKeyFrame(const RawKeyFrameId& keyframe_id,
                                               const std::string& source)
{
    // F1E: solo el KF asociado directamente a una observacion fiducial debe
    // quedar hard. Los demas KFs del submapa pueden tener pose world por anchor,
    // pero no se congelan como fiduciales reales.
    if (!HasWorldPose(keyframe_id))
    {
        return false;
    }
    hard_fiducial_keyframes_[keyframe_id] = source;
    return true;
}

bool GlobalPoseStore::IsHardFiducialKeyFrame(const RawKeyFrameId& keyframe_id) const
{
    return hard_fiducial_keyframes_.find(keyframe_id) != hard_fiducial_keyframes_.end();
}

GlobalPoseNewKeyFrameResult GlobalPoseStore::RegisterNewKeyFrameIfAnchored(
    const RawKeyFrameId& keyframe_id,
    const RawMapDatabase& raw_db,
    const std::string& source)
{
    // 1K: una pose world aceptada no se reinterpreta con una correccion rigida
    // de submapa. El KF nuevo se extiende desde el ultimo KF aceptado anterior
    // usando la relacion raw actual entre ambos.
    GlobalPoseNewKeyFrameResult result;
    result.keyframe_id = keyframe_id;
    result.source = source;

    if (!raw_db.HasKeyFrame(keyframe_id))
    {
        result.status = GlobalPoseNewKeyFrameStatus::MissingRawKeyFrame;
        result.reason = "raw_keyframe_missing";
        return result;
    }
    if (HasWorldPose(keyframe_id))
    {
        result.status = GlobalPoseNewKeyFrameStatus::AlreadyHadPose;
        result.reason = "world_pose_already_registered";
        return result;
    }

    const RawSubmapId submap_id = SubmapOf(keyframe_id);
    if (!HasSubmapAnchor(submap_id))
    {
        result.status = GlobalPoseNewKeyFrameStatus::UnanchoredSubmap;
        result.reason = "submap_without_anchor";
        return result;
    }

    Eigen::Matrix4d raw_world_T_kf = Eigen::Matrix4d::Identity();
    if (!ComputeRawWorldPose(keyframe_id, raw_db, raw_world_T_kf, &result.reason))
    {
        result.status = GlobalPoseNewKeyFrameStatus::MissingRawKeyFrame;
        return result;
    }

    Eigen::Matrix4d world_T_kf = raw_world_T_kf;
    const auto active_it = active_tail_anchors_.find(submap_id);
    const bool active_tail_controls_keyframe =
        active_it != active_tail_anchors_.end() &&
        keyframe_id.local_kf_id >
            active_it->second.reference_keyframe_id.local_kf_id;
    const auto reference_id = active_tail_controls_keyframe
        ? std::optional<RawKeyFrameId>(
              active_it->second.reference_keyframe_id)
        : FindLatestAcceptedReference(keyframe_id);
    if (reference_id)
    {
        const auto reference_it = accepted_keyframe_anchors_.find(*reference_id);
        if (reference_it == accepted_keyframe_anchors_.end() ||
            !ProjectWorldPoseFromReference(
                *reference_id,
                reference_it->second.accepted_world_T_kf,
                keyframe_id,
                raw_db,
                world_T_kf,
                &result.reason))
        {
            result.status = GlobalPoseNewKeyFrameStatus::MissingRawKeyFrame;
            return result;
        }
    }

    const Eigen::Matrix4d correction_T_kf =
        world_T_kf * raw_world_T_kf.inverse();
    if (active_tail_controls_keyframe)
    {
        const std::string derived_source =
            active_it->second.source + "_DERIVED_TAIL";
        optimized_keyframes_[keyframe_id] =
            OptimizedKeyFramePose{
                world_T_kf,
                correction_T_kf,
                derived_source,
                true,
                false};
        keyframe_world_poses_[keyframe_id] =
            KeyFrameWorldPose{
                world_T_kf,
                derived_source,
                true,
                true,
                false};
        derived_tail_keyframes_[keyframe_id] =
            DerivedTailKeyFrame{
                active_it->second.reference_keyframe_id,
                derived_source,
                active_it->second.source_task_id};
        result.status = GlobalPoseNewKeyFrameStatus::CorrectionInherited;
        result.source = derived_source;
        result.reason = "active_tail_projection_applied";
        return result;
    }

    accepted_keyframe_anchors_[keyframe_id] =
        AcceptedKeyFrameAnchor{world_T_kf, source, 0};
    keyframe_world_poses_[keyframe_id] =
        KeyFrameWorldPose{world_T_kf, source, false};
    result.status = GlobalPoseNewKeyFrameStatus::PoseSet;
    result.reason = reference_id
        ? "accepted_reference_projection_applied"
        : "anchor_pose_applied";
    return result;
}

GlobalPoseOptimizedResult GlobalPoseStore::SetOptimizedKeyFramePose(
    const RawKeyFrameId& keyframe_id,
    const Eigen::Matrix4d& optimized_world_T_kf,
    const RawMapDatabase& raw_db,
    const std::string& source)
{
    return SetServerControlledKeyFramePose(
        keyframe_id,
        optimized_world_T_kf,
        raw_db,
        source,
        false,
        false);
}

GlobalPoseOptimizedResult GlobalPoseStore::SetPropagatedKeyFramePose(
    const RawKeyFrameId& keyframe_id,
    const Eigen::Matrix4d& propagated_world_T_kf,
    const RawMapDatabase& raw_db,
    const std::string& source)
{
    return SetServerControlledKeyFramePose(
        keyframe_id,
        propagated_world_T_kf,
        raw_db,
        source,
        true,
        false);
}

std::optional<Eigen::Matrix4d> GlobalPoseStore::GetKeyFrameServerCorrection(
    const RawKeyFrameId& keyframe_id) const
{
    const auto it = optimized_keyframes_.find(keyframe_id);
    if (it == optimized_keyframes_.end())
    {
        return std::nullopt;
    }
    return it->second.correction_T_kf;
}

std::optional<Eigen::Matrix4d> GlobalPoseStore::GetSubmapLastServerCorrection(
    const RawSubmapId& submap_id) const
{
    const auto it = submap_last_corrections_.find(submap_id);
    if (it == submap_last_corrections_.end())
    {
        return std::nullopt;
    }
    return it->second.correction_T_latest;
}

GlobalPoseOptimizedResult GlobalPoseStore::SetSubmapLastServerCorrectionFromKeyFrame(
    const RawKeyFrameId& keyframe_id,
    const std::string& source)
{
    // 1K: tras escribir una optimizacion, los KFs futuros deben heredar la
    // correccion del extremo mas nuevo ya optimizado/propagado, no volver a la
    // transformada del anchor fiducial anterior. Si 1L rechaza, el backup
    // restaura esta entrada.
    GlobalPoseOptimizedResult result;
    result.submap_id = SubmapOf(keyframe_id);

    const auto optimized_it = optimized_keyframes_.find(keyframe_id);
    if (optimized_it == optimized_keyframes_.end())
    {
        result.reason = "optimized_keyframe_missing";
        return result;
    }
    if (!IsFiniteTransform(optimized_it->second.correction_T_kf))
    {
        result.reason = "correction_T_kf_not_finite";
        return result;
    }

    submap_last_corrections_[result.submap_id] =
        SubmapCorrection{optimized_it->second.correction_T_kf, source, true, keyframe_id};

    result.success = true;
    result.correction_T_latest = optimized_it->second.correction_T_kf;
    result.reason = "last_correction_set_from_keyframe";
    return result;
}

GlobalPoseOptimizedResult GlobalPoseStore::SetDerivedTailKeyFramePose(
    const RawKeyFrameId& keyframe_id,
    const Eigen::Matrix4d& world_T_kf,
    const RawKeyFrameId& reference_keyframe_id,
    const RawMapDatabase& raw_db,
    uint64_t source_task_id,
    const std::string& source)
{
    GlobalPoseOptimizedResult result;
    result.submap_id = SubmapOf(keyframe_id);
    if (!(SubmapOf(reference_keyframe_id) == result.submap_id))
    {
        result.reason = "derived_reference_submap_mismatch";
        return result;
    }
    if (reference_keyframe_id.local_kf_id >= keyframe_id.local_kf_id)
    {
        result.reason = "derived_reference_not_older";
        return result;
    }
    if (accepted_keyframe_anchors_.find(reference_keyframe_id) ==
        accepted_keyframe_anchors_.end())
    {
        result.reason = "derived_reference_not_accepted";
        return result;
    }

    result = SetServerControlledKeyFramePose(
        keyframe_id,
        world_T_kf,
        raw_db,
        source,
        true,
        true);
    if (!result.success)
    {
        return result;
    }

    accepted_keyframe_anchors_.erase(keyframe_id);
    derived_tail_keyframes_[keyframe_id] =
        DerivedTailKeyFrame{
            reference_keyframe_id,
            source,
            source_task_id};
    result.reason = "derived_tail_pose_registered";
    return result;
}

bool GlobalPoseStore::SetActiveTailAnchor(
    const RawKeyFrameId& reference_keyframe_id,
    uint64_t source_task_id,
    const std::string& source,
    bool server_optimized,
    std::string* reason)
{
    const auto world_it = keyframe_world_poses_.find(reference_keyframe_id);
    if (world_it == keyframe_world_poses_.end())
    {
        SetReason(reason, "reference_world_pose_missing");
        return false;
    }
    if (!IsFiniteTransform(world_it->second.world_T_kf))
    {
        SetReason(reason, "reference_world_pose_not_finite");
        return false;
    }

    accepted_keyframe_anchors_[reference_keyframe_id] =
        AcceptedKeyFrameAnchor{
            world_it->second.world_T_kf,
            source,
            source_task_id};
    derived_tail_keyframes_.erase(reference_keyframe_id);
    active_tail_anchors_[SubmapOf(reference_keyframe_id)] =
        ActiveTailAnchor{
            reference_keyframe_id,
            world_it->second.world_T_kf,
            source,
            source_task_id,
            server_optimized};
    SetReason(reason, "active_tail_anchor_set");
    return true;
}

std::optional<ActiveTailAnchorInfo> GlobalPoseStore::GetActiveTailAnchor(
    const RawSubmapId& submap_id) const
{
    const auto it = active_tail_anchors_.find(submap_id);
    if (it == active_tail_anchors_.end())
    {
        return std::nullopt;
    }
    return ActiveTailAnchorInfo{
        submap_id,
        it->second.reference_keyframe_id,
        it->second.accepted_world_T_reference,
        it->second.source,
        it->second.source_task_id,
        it->second.server_optimized};
}

bool GlobalPoseStore::ProjectWorldPoseFromReference(
    const RawKeyFrameId& reference_keyframe_id,
    const Eigen::Matrix4d& accepted_world_T_reference,
    const RawKeyFrameId& keyframe_id,
    const RawMapDatabase& raw_db,
    Eigen::Matrix4d& world_T_kf,
    std::string* reason) const
{
    if (!(SubmapOf(reference_keyframe_id) == SubmapOf(keyframe_id)))
    {
        SetReason(reason, "reference_submap_mismatch");
        return false;
    }
    if (reference_keyframe_id.local_kf_id >= keyframe_id.local_kf_id)
    {
        SetReason(reason, "reference_not_older_than_keyframe");
        return false;
    }
    if (!IsFiniteTransform(accepted_world_T_reference))
    {
        SetReason(reason, "accepted_reference_pose_not_finite");
        return false;
    }

    const auto* reference = raw_db.GetKeyFrame(reference_keyframe_id);
    const auto* keyframe = raw_db.GetKeyFrame(keyframe_id);
    if (!reference || !keyframe)
    {
        SetReason(reason, "raw_reference_or_keyframe_missing");
        return false;
    }

    Eigen::Matrix4d raw_local_T_reference = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d raw_local_T_kf = Eigen::Matrix4d::Identity();
    if (!PoseMsgToMatrix(reference->pose, raw_local_T_reference) ||
        !PoseMsgToMatrix(keyframe->pose, raw_local_T_kf))
    {
        SetReason(reason, "raw_reference_or_keyframe_pose_invalid");
        return false;
    }

    world_T_kf =
        accepted_world_T_reference *
        raw_local_T_reference.inverse() *
        raw_local_T_kf;
    if (!IsFiniteTransform(world_T_kf))
    {
        SetReason(reason, "projected_world_pose_not_finite");
        return false;
    }
    SetReason(reason, "ok");
    return true;
}

ApplyBackupResult GlobalPoseStore::CreateApplyBackup(
    uint64_t task_id,
    const std::vector<RawKeyFrameId>& affected_keyframes)
{
    // 1K: el backup previo al apply debe poder restaurar tanto valores como
    // ausencias anteriores. Por eso guarda flags `had_*` en vez de asumir que
    // todos los KFs ya estaban optimizados.
    ApplyBackupResult result;
    result.task_id = task_id;
    if (affected_keyframes.empty())
    {
        result.reason = "no_affected_keyframes";
        return result;
    }

    ApplyBackup backup;
    backup.task_id = task_id;
    std::set<RawKeyFrameId> unique_keyframes;
    std::set<RawSubmapId> unique_submaps;

    for (const auto& keyframe_id : affected_keyframes)
    {
        if (!unique_keyframes.insert(keyframe_id).second)
        {
            continue;
        }

        ApplyBackupKeyFrameState state;
        state.keyframe_id = keyframe_id;

        const auto world_it = keyframe_world_poses_.find(keyframe_id);
        if (world_it != keyframe_world_poses_.end())
        {
            state.had_world_pose = true;
            state.world_T_kf = world_it->second.world_T_kf;
            state.world_source = world_it->second.source;
            state.world_optimized = world_it->second.optimized;
            state.world_propagated = world_it->second.propagated;
            state.world_rebased = world_it->second.rebased;
        }

        const auto optimized_it = optimized_keyframes_.find(keyframe_id);
        if (optimized_it != optimized_keyframes_.end())
        {
            state.had_optimized_pose = true;
            state.optimized_world_T_kf = optimized_it->second.optimized_world_T_kf;
            state.correction_T_kf = optimized_it->second.correction_T_kf;
            state.optimized_source = optimized_it->second.source;
            state.optimized_propagated = optimized_it->second.propagated;
            state.optimized_rebased = optimized_it->second.rebased;
        }

        const auto hard_it = hard_fiducial_keyframes_.find(keyframe_id);
        if (hard_it != hard_fiducial_keyframes_.end())
        {
            state.was_hard_fiducial = true;
            state.hard_fiducial_source = hard_it->second;
        }

        const auto accepted_it = accepted_keyframe_anchors_.find(keyframe_id);
        if (accepted_it != accepted_keyframe_anchors_.end())
        {
            state.was_accepted_anchor = true;
            state.accepted_world_T_kf =
                accepted_it->second.accepted_world_T_kf;
            state.accepted_anchor_source = accepted_it->second.source;
            state.accepted_anchor_task_id = accepted_it->second.source_task_id;
        }

        const auto derived_it = derived_tail_keyframes_.find(keyframe_id);
        if (derived_it != derived_tail_keyframes_.end())
        {
            state.was_derived_tail = true;
            state.derived_reference_keyframe_id =
                derived_it->second.reference_keyframe_id;
            state.derived_tail_source = derived_it->second.source;
            state.derived_tail_task_id = derived_it->second.source_task_id;
        }

        backup.keyframes.push_back(state);
        unique_submaps.insert(SubmapOf(keyframe_id));
    }

    for (const auto& submap_id : unique_submaps)
    {
        ApplyBackupSubmapCorrectionState state;
        state.submap_id = submap_id;
        const auto correction_it = submap_last_corrections_.find(submap_id);
        if (correction_it != submap_last_corrections_.end())
        {
            state.had_correction = true;
            state.correction_T_latest = correction_it->second.correction_T_latest;
            state.source = correction_it->second.source;
            state.had_from_keyframe = correction_it->second.has_from_keyframe;
            state.from_keyframe = correction_it->second.from_keyframe;
        }
        backup.submap_corrections.push_back(state);

        ApplyBackupActiveTailAnchorState tail_state;
        tail_state.submap_id = submap_id;
        const auto tail_it = active_tail_anchors_.find(submap_id);
        if (tail_it != active_tail_anchors_.end())
        {
            tail_state.had_active_tail_anchor = true;
            tail_state.reference_keyframe_id =
                tail_it->second.reference_keyframe_id;
            tail_state.accepted_world_T_reference =
                tail_it->second.accepted_world_T_reference;
            tail_state.source = tail_it->second.source;
            tail_state.source_task_id = tail_it->second.source_task_id;
            tail_state.server_optimized = tail_it->second.server_optimized;
        }
        backup.active_tail_anchors.push_back(tail_state);
    }

    result.affected_keyframes = backup.keyframes.size();
    result.affected_submaps = backup.submap_corrections.size();
    pending_apply_backups_[task_id] = backup;
    result.success = true;
    result.reason = "backup_created";
    return result;
}

RollbackResult GlobalPoseStore::RestoreApplyBackup(uint64_t task_id)
{
    // 1L: restaurar no recalcula poses desde raw ni reoptimiza; solo vuelve al
    // snapshot exacto tomado por 1K antes del apply para evitar efectos
    // laterales.
    RollbackResult result;
    result.task_id = task_id;
    const auto backup_it = pending_apply_backups_.find(task_id);
    if (backup_it == pending_apply_backups_.end())
    {
        result.reason = "backup_missing";
        return result;
    }

    for (const auto& state : backup_it->second.keyframes)
    {
        if (state.had_world_pose)
        {
            keyframe_world_poses_[state.keyframe_id] =
                KeyFrameWorldPose{
                    state.world_T_kf,
                    state.world_source,
                    state.world_optimized,
                    state.world_propagated,
                    state.world_rebased};
            ++result.restored_world_poses;
        }
        else
        {
            result.removed_world_poses += keyframe_world_poses_.erase(state.keyframe_id);
        }

        if (state.had_optimized_pose)
        {
            optimized_keyframes_[state.keyframe_id] =
                OptimizedKeyFramePose{
                    state.optimized_world_T_kf,
                    state.correction_T_kf,
                    state.optimized_source,
                    state.optimized_propagated,
                    state.optimized_rebased};
            ++result.restored_optimized_poses;
        }
        else
        {
            result.removed_optimized_poses += optimized_keyframes_.erase(state.keyframe_id);
        }

        if (state.was_hard_fiducial)
        {
            hard_fiducial_keyframes_[state.keyframe_id] = state.hard_fiducial_source;
        }
        else
        {
            hard_fiducial_keyframes_.erase(state.keyframe_id);
        }

        if (state.was_accepted_anchor)
        {
            accepted_keyframe_anchors_[state.keyframe_id] =
                AcceptedKeyFrameAnchor{
                    state.accepted_world_T_kf,
                    state.accepted_anchor_source,
                    state.accepted_anchor_task_id};
            ++result.restored_accepted_anchors;
        }
        else
        {
            result.removed_accepted_anchors +=
                accepted_keyframe_anchors_.erase(state.keyframe_id);
        }

        if (state.was_derived_tail)
        {
            derived_tail_keyframes_[state.keyframe_id] =
                DerivedTailKeyFrame{
                    state.derived_reference_keyframe_id,
                    state.derived_tail_source,
                    state.derived_tail_task_id};
            ++result.restored_derived_tail_keyframes;
        }
        else
        {
            result.removed_derived_tail_keyframes +=
                derived_tail_keyframes_.erase(state.keyframe_id);
        }
    }

    for (const auto& state : backup_it->second.submap_corrections)
    {
        if (state.had_correction)
        {
            submap_last_corrections_[state.submap_id] =
                SubmapCorrection{
                    state.correction_T_latest,
                    state.source,
                    state.had_from_keyframe,
                    state.from_keyframe};
            ++result.restored_submap_corrections;
        }
        else
        {
            result.removed_submap_corrections +=
                submap_last_corrections_.erase(state.submap_id);
        }
    }

    for (const auto& state : backup_it->second.active_tail_anchors)
    {
        if (state.had_active_tail_anchor)
        {
            active_tail_anchors_[state.submap_id] =
                ActiveTailAnchor{
                    state.reference_keyframe_id,
                    state.accepted_world_T_reference,
                    state.source,
                    state.source_task_id,
                    state.server_optimized};
            ++result.restored_active_tail_anchors;
        }
        else
        {
            result.removed_active_tail_anchors +=
                active_tail_anchors_.erase(state.submap_id);
        }
    }

    pending_apply_backups_.erase(backup_it);
    result.success = true;
    result.reason = "rollback_restored";
    return result;
}

bool GlobalPoseStore::ConfirmApply(uint64_t task_id)
{
    // 1L: confirmar un apply solo descarta el backup. No cambia poses; la
    // validacion ya decidió que el estado actual puede conservarse.
    return pending_apply_backups_.erase(task_id) > 0;
}

GlobalPoseOptimizedResult GlobalPoseStore::SetServerControlledKeyFramePose(
    const RawKeyFrameId& keyframe_id,
    const Eigen::Matrix4d& world_T_kf,
    const RawMapDatabase& raw_db,
    const std::string& source,
    bool propagated,
    bool rebased)
{
    // 1K: registra una pose decidida por el servidor y deriva la correccion
    // respecto a `world_T_local * local_T_kf`. RawMapDatabase sigue siendo
    // solo lectura; la marca propagated/rebased permite validar el origen.
    GlobalPoseOptimizedResult result;
    result.submap_id = SubmapOf(keyframe_id);

    if (!IsFiniteTransform(world_T_kf))
    {
        result.reason = "world_T_kf_not_finite";
        return result;
    }

    Eigen::Matrix4d raw_world_T_kf = Eigen::Matrix4d::Identity();
    if (!ComputeRawWorldPose(keyframe_id, raw_db, raw_world_T_kf, &result.reason))
    {
        return result;
    }

    const Eigen::Matrix4d correction_T_kf =
        world_T_kf * raw_world_T_kf.inverse();
    if (!IsFiniteTransform(correction_T_kf))
    {
        result.reason = "correction_T_kf_not_finite";
        return result;
    }

    optimized_keyframes_[keyframe_id] =
        OptimizedKeyFramePose{world_T_kf, correction_T_kf, source, propagated, rebased};
    keyframe_world_poses_[keyframe_id] =
        KeyFrameWorldPose{world_T_kf, source, true, propagated, rebased};
    accepted_keyframe_anchors_[keyframe_id] =
        AcceptedKeyFrameAnchor{world_T_kf, source, 0};
    derived_tail_keyframes_.erase(keyframe_id);
    // 1K: un parcial reduce una deuda fiducial grande, pero aun no es una
    // correccion global segura para todo el submapa. No debe alimentar la
    // correccion heredable que GlobalMapBuilder usa como fallback; 1L decidira
    // si se conserva, se reintenta o se restaura.
    if (!IsPartialOptimizationSource(source))
    {
        submap_last_corrections_[result.submap_id] =
            SubmapCorrection{correction_T_kf, source, true, keyframe_id};
    }

    result.success = true;
    result.correction_T_latest = correction_T_kf;
    result.reason = propagated ? "propagated_pose_registered" : "optimized_pose_registered";
    return result;
}

PoseStoreReconcileResult GlobalPoseStore::ReconcileAfterRawIngestResult(
    const RawInsertResult& ingest_result,
    const RawMapDatabase& raw_db,
    const std::string& source)
{
    // 1K: las poses aceptadas son autoridad world e inmutables ante cambios
    // raw. La cola derivada, en cambio, se vuelve a proyectar desde su ancla
    // aceptada para conservar coherencia cuando ORB-SLAM3 cambia de gauge.
    PoseStoreReconcileResult result;
    result.raw_pose_changed = ingest_result.raw_pose_changed_keyframes.size();
    std::set<RawSubmapId> changed_submaps;
    std::set<RawKeyFrameId> reconciled_derived;

    for (const auto& raw_change : ingest_result.raw_pose_changed_keyframes)
    {
        changed_submaps.insert(SubmapOf(raw_change.keyframe_id));
        PoseStoreReconcileEvent event;
        event.keyframe_id = raw_change.keyframe_id;
        event.raw_delta_translation_m = raw_change.delta_translation_m;
        event.raw_delta_yaw_rad = raw_change.delta_yaw_rad;

        if (derived_tail_keyframes_.find(raw_change.keyframe_id) !=
            derived_tail_keyframes_.end())
        {
            std::string reason;
            if (!RecomputeDerivedTailPose(
                    raw_change.keyframe_id,
                    raw_db,
                    source + "_DERIVED_TAIL",
                    &reason))
            {
                ++result.failed;
                event.action = "failed";
                event.reason = reason;
            }
            else
            {
                ++result.optimized_kept;
                result.optimized_submaps_affected.insert(
                    SubmapOf(raw_change.keyframe_id));
                event.action = "rebase_derived_tail";
                event.reason = "derived_tail_reprojected_from_accepted_anchor";
                reconciled_derived.insert(raw_change.keyframe_id);
            }
            result.events.push_back(event);
            continue;
        }

        const auto pose_it = keyframe_world_poses_.find(raw_change.keyframe_id);
        if (pose_it == keyframe_world_poses_.end())
        {
            ++result.no_world_pose;
            event.action = "no_world_pose";
            event.reason = "keyframe_without_global_pose";
            result.events.push_back(event);
            continue;
        }

        Eigen::Matrix4d raw_world_T_kf = Eigen::Matrix4d::Identity();
        std::string reason;
        if (!ComputeRawWorldPose(raw_change.keyframe_id, raw_db, raw_world_T_kf, &reason))
        {
            ++result.failed;
            event.action = "failed";
            event.reason = reason;
            result.events.push_back(event);
            continue;
        }

        const auto accepted_it =
            accepted_keyframe_anchors_.find(raw_change.keyframe_id);
        if (accepted_it != accepted_keyframe_anchors_.end())
        {
            const Eigen::Matrix4d accepted_world_T_kf =
                accepted_it->second.accepted_world_T_kf;
            const Eigen::Matrix4d correction_T_kf =
                accepted_world_T_kf * raw_world_T_kf.inverse();
            if (!IsFiniteTransform(correction_T_kf))
            {
                ++result.failed;
                event.action = "failed";
                event.reason = "correction_T_kf_not_finite";
                result.events.push_back(event);
                continue;
            }

            const auto opt_it = optimized_keyframes_.find(raw_change.keyframe_id);
            if (opt_it != optimized_keyframes_.end())
            {
                opt_it->second.optimized_world_T_kf = accepted_world_T_kf;
                opt_it->second.correction_T_kf = correction_T_kf;
                opt_it->second.source = source;
                keyframe_world_poses_[raw_change.keyframe_id] =
                    KeyFrameWorldPose{
                        accepted_world_T_kf,
                        source,
                        true,
                        opt_it->second.propagated,
                        opt_it->second.rebased};
                event.action = "keep_optimized";
                event.reason = "server_accepted_pose_preferred";
            }
            else
            {
                const bool corrected = !IsNearIdentity(correction_T_kf);
                keyframe_world_poses_[raw_change.keyframe_id] =
                    KeyFrameWorldPose{
                        accepted_world_T_kf,
                        source,
                        corrected,
                        corrected,
                        false};
                if (corrected)
                {
                    optimized_keyframes_[raw_change.keyframe_id] =
                        OptimizedKeyFramePose{
                            accepted_world_T_kf,
                            correction_T_kf,
                            source,
                            true,
                            false};
                }
                event.action = "keep_accepted";
                event.reason = "accepted_world_pose_immutable";
            }
            result.optimized_submaps_affected.insert(
                SubmapOf(raw_change.keyframe_id));
            ++result.optimized_kept;
            result.events.push_back(event);
            continue;
        }

        Eigen::Matrix4d world_T_kf = raw_world_T_kf;
        const auto reference_id =
            FindLatestAcceptedReference(raw_change.keyframe_id);
        if (reference_id)
        {
            const auto reference_it =
                accepted_keyframe_anchors_.find(*reference_id);
            if (reference_it == accepted_keyframe_anchors_.end() ||
                !ProjectWorldPoseFromReference(
                    *reference_id,
                    reference_it->second.accepted_world_T_kf,
                    raw_change.keyframe_id,
                    raw_db,
                    world_T_kf,
                    &reason))
            {
                ++result.failed;
                event.action = "failed";
                event.reason = reason;
                result.events.push_back(event);
                continue;
            }
        }

        keyframe_world_poses_[raw_change.keyframe_id] =
            KeyFrameWorldPose{world_T_kf, source, false, false, false};
        accepted_keyframe_anchors_[raw_change.keyframe_id] =
            AcceptedKeyFrameAnchor{world_T_kf, source, 0};
        ++result.anchor_rebased;
        event.action = reference_id ? "rebase_accepted_reference" : "rebase_anchor";
        event.reason = reference_id
            ? "legacy_pose_migrated_to_accepted_reference"
            : "anchor_derived_pose_recomputed";
        result.events.push_back(event);
    }

    // Si cambia el raw de la referencia, la relacion de toda la cola cambia
    // aunque ORB-SLAM3 no marque cada KF derivado como modificado.
    for (const auto& [keyframe_id, _] : derived_tail_keyframes_)
    {
        if (changed_submaps.find(SubmapOf(keyframe_id)) == changed_submaps.end() ||
            reconciled_derived.find(keyframe_id) != reconciled_derived.end())
        {
            continue;
        }

        PoseStoreReconcileEvent event;
        event.keyframe_id = keyframe_id;
        std::string reason;
        if (!RecomputeDerivedTailPose(
                keyframe_id,
                raw_db,
                source + "_DERIVED_TAIL",
                &reason))
        {
            ++result.failed;
            event.action = "failed";
            event.reason = reason;
        }
        else
        {
            ++result.optimized_kept;
            result.optimized_submaps_affected.insert(SubmapOf(keyframe_id));
            event.action = "rebase_derived_tail";
            event.reason = "derived_tail_reprojected_from_accepted_anchor";
        }
        result.events.push_back(event);
    }

    return result;
}

GlobalPoseStoreStats GlobalPoseStore::GetPoseStoreStats() const
{
    uint64_t propagated = 0;
    uint64_t rebased = 0;
    for (const auto& [_, pose] : optimized_keyframes_)
    {
        if (pose.propagated)
        {
            ++propagated;
        }
        if (pose.rebased)
        {
            ++rebased;
        }
    }

    return GlobalPoseStoreStats{
        static_cast<uint64_t>(submap_anchors_.size()),
        static_cast<uint64_t>(keyframe_world_poses_.size()),
        static_cast<uint64_t>(optimized_keyframes_.size()),
        propagated,
        rebased,
        static_cast<uint64_t>(submap_last_corrections_.size()),
        static_cast<uint64_t>(hard_fiducial_keyframes_.size()),
        static_cast<uint64_t>(accepted_keyframe_anchors_.size()),
        static_cast<uint64_t>(active_tail_anchors_.size()),
        static_cast<uint64_t>(derived_tail_keyframes_.size())};
}

GlobalSubmapPoseStats GlobalPoseStore::GetSubmapPoseStats(
    const RawSubmapId& submap_id) const
{
    GlobalSubmapPoseStats stats;
    stats.submap_id = submap_id;

    const auto anchor_it = submap_anchors_.find(submap_id);
    if (anchor_it != submap_anchors_.end())
    {
        stats.has_anchor = true;
        stats.anchor_source = anchor_it->second.source;
    }

    const auto correction_it = submap_last_corrections_.find(submap_id);
    if (correction_it != submap_last_corrections_.end())
    {
        stats.has_last_correction = true;
        stats.correction_source = correction_it->second.source;
    }

    const auto active_tail_it = active_tail_anchors_.find(submap_id);
    if (active_tail_it != active_tail_anchors_.end())
    {
        stats.has_active_tail_anchor = true;
        stats.active_tail_anchor_keyframe =
            active_tail_it->second.reference_keyframe_id;
        stats.active_tail_anchor_source = active_tail_it->second.source;
    }

    for (const auto& [keyframe_id, _] : keyframe_world_poses_)
    {
        if (keyframe_id.drone_id == submap_id.drone_id &&
            keyframe_id.map_epoch == submap_id.map_epoch)
        {
            ++stats.keyframe_world_poses;
        }
    }

    for (const auto& [keyframe_id, _] : accepted_keyframe_anchors_)
    {
        if (keyframe_id.drone_id == submap_id.drone_id &&
            keyframe_id.map_epoch == submap_id.map_epoch)
        {
            ++stats.accepted_keyframe_anchors;
        }
    }

    return stats;
}

bool GlobalPoseStore::ComputeRawWorldPose(const RawKeyFrameId& keyframe_id,
                                          const RawMapDatabase& raw_db,
                                          Eigen::Matrix4d& raw_world_T_kf,
                                          std::string* reason) const
{
    const RawSubmapId submap_id = SubmapOf(keyframe_id);
    const auto anchor_it = submap_anchors_.find(submap_id);
    if (anchor_it == submap_anchors_.end())
    {
        SetReason(reason, "submap_without_anchor");
        return false;
    }

    const auto* keyframe = raw_db.GetKeyFrame(keyframe_id);
    if (!keyframe)
    {
        SetReason(reason, "raw_keyframe_missing");
        return false;
    }

    Eigen::Matrix4d local_T_kf = Eigen::Matrix4d::Identity();
    if (!PoseMsgToMatrix(keyframe->pose, local_T_kf))
    {
        SetReason(reason, "local_T_kf_invalid");
        return false;
    }

    raw_world_T_kf = anchor_it->second.world_T_local * local_T_kf;
    if (!IsFiniteTransform(raw_world_T_kf))
    {
        SetReason(reason, "raw_world_T_kf_not_finite");
        return false;
    }

    SetReason(reason, "ok");
    return true;
}

std::optional<RawKeyFrameId> GlobalPoseStore::FindLatestAcceptedReference(
    const RawKeyFrameId& keyframe_id) const
{
    std::optional<RawKeyFrameId> latest;
    for (const auto& [candidate_id, _] : accepted_keyframe_anchors_)
    {
        if (candidate_id.drone_id != keyframe_id.drone_id ||
            candidate_id.map_epoch != keyframe_id.map_epoch ||
            candidate_id.local_kf_id >= keyframe_id.local_kf_id)
        {
            continue;
        }
        if (!latest ||
            candidate_id.local_kf_id > latest->local_kf_id)
        {
            latest = candidate_id;
        }
    }
    return latest;
}

bool GlobalPoseStore::RecomputeDerivedTailPose(
    const RawKeyFrameId& keyframe_id,
    const RawMapDatabase& raw_db,
    const std::string& source,
    std::string* reason)
{
    const auto derived_it = derived_tail_keyframes_.find(keyframe_id);
    if (derived_it == derived_tail_keyframes_.end())
    {
        SetReason(reason, "derived_tail_keyframe_missing");
        return false;
    }
    const auto accepted_it =
        accepted_keyframe_anchors_.find(
            derived_it->second.reference_keyframe_id);
    if (accepted_it == accepted_keyframe_anchors_.end())
    {
        SetReason(reason, "derived_reference_not_accepted");
        return false;
    }

    Eigen::Matrix4d world_T_kf = Eigen::Matrix4d::Identity();
    if (!ProjectWorldPoseFromReference(
            derived_it->second.reference_keyframe_id,
            accepted_it->second.accepted_world_T_kf,
            keyframe_id,
            raw_db,
            world_T_kf,
            reason))
    {
        return false;
    }

    Eigen::Matrix4d raw_world_T_kf = Eigen::Matrix4d::Identity();
    if (!ComputeRawWorldPose(keyframe_id, raw_db, raw_world_T_kf, reason))
    {
        return false;
    }
    const Eigen::Matrix4d correction_T_kf =
        world_T_kf * raw_world_T_kf.inverse();
    if (!IsFiniteTransform(correction_T_kf))
    {
        SetReason(reason, "derived_correction_not_finite");
        return false;
    }

    optimized_keyframes_[keyframe_id] =
        OptimizedKeyFramePose{
            world_T_kf,
            correction_T_kf,
            source,
            true,
            true};
    keyframe_world_poses_[keyframe_id] =
        KeyFrameWorldPose{
            world_T_kf,
            source,
            true,
            true,
            true};
    derived_it->second.source = source;
    SetReason(reason, "derived_tail_recomputed");
    return true;
}

}  // namespace orbslam3_multi
