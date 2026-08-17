#include "orbslam3_multi/raw_map_database.hpp"

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <set>
#include <sstream>

namespace orbslam3_multi
{
namespace
{
using OrbMap = orbslam3_msgs::msg::OrbMap;

constexpr uint32_t kRecordMagic = 0x31434252U;  // "RBC1" little-endian marker.
constexpr uint32_t kRecordVersion = 2U;

// F1C: helpers binarios minimos para el formato `.record`.
// Solo escriben/leen tipos triviales; los mensajes ROS 2 se serializan aparte
// para conservar exactamente el `OrbMap` recibido en el journal raw.
template <typename T>
bool WritePod(std::ostream& out, const T& value)
{
    out.write(reinterpret_cast<const char*>(&value), sizeof(T));
    return static_cast<bool>(out);
}

template <typename T>
bool ReadPod(std::istream& in, T& value)
{
    in.read(reinterpret_cast<char*>(&value), sizeof(T));
    return static_cast<bool>(in);
}

bool WriteString(std::ostream& out, const std::string& value)
{
    const uint64_t size = value.size();
    if (!WritePod(out, size))
    {
        return false;
    }
    out.write(value.data(), static_cast<std::streamsize>(size));
    return static_cast<bool>(out);
}

bool ReadString(std::istream& in, std::string& value)
{
    uint64_t size = 0;
    if (!ReadPod(in, size))
    {
        return false;
    }
    value.resize(static_cast<size_t>(size));
    if (size == 0)
    {
        return true;
    }
    in.read(value.data(), static_cast<std::streamsize>(size));
    return static_cast<bool>(in);
}

bool WriteFiducialObservation(std::ostream& out, const RecordedFiducialObservation& obs)
{
    return WritePod(out, obs.arrival_id) &&
           WritePod(out, obs.drone_id) &&
           WritePod(out, obs.map_epoch) &&
           WritePod(out, obs.local_keyframe_id) &&
           WritePod(out, obs.global_keyframe_id) &&
           WritePod(out, obs.fiducial_id) &&
           WritePod(out, obs.world_x) &&
           WritePod(out, obs.world_y) &&
           WritePod(out, obs.world_z) &&
           WritePod(out, obs.world_qx) &&
           WritePod(out, obs.world_qy) &&
           WritePod(out, obs.world_qz) &&
           WritePod(out, obs.world_qw) &&
           WritePod(out, obs.keyframe_stamp_sec) &&
           WritePod(out, obs.gt_stamp_sec) &&
           WritePod(out, obs.association_dt_sec) &&
           WritePod(out, obs.distance_to_fiducial_m) &&
           WriteString(out, obs.source);
}

bool ReadFiducialObservation(std::istream& in, RecordedFiducialObservation& obs)
{
    return ReadPod(in, obs.arrival_id) &&
           ReadPod(in, obs.drone_id) &&
           ReadPod(in, obs.map_epoch) &&
           ReadPod(in, obs.local_keyframe_id) &&
           ReadPod(in, obs.global_keyframe_id) &&
           ReadPod(in, obs.fiducial_id) &&
           ReadPod(in, obs.world_x) &&
           ReadPod(in, obs.world_y) &&
           ReadPod(in, obs.world_z) &&
           ReadPod(in, obs.world_qx) &&
           ReadPod(in, obs.world_qy) &&
           ReadPod(in, obs.world_qz) &&
           ReadPod(in, obs.world_qw) &&
           ReadPod(in, obs.keyframe_stamp_sec) &&
           ReadPod(in, obs.gt_stamp_sec) &&
           ReadPod(in, obs.association_dt_sec) &&
           ReadPod(in, obs.distance_to_fiducial_m) &&
           ReadString(in, obs.source);
}

// F1C: centraliza la escritura opcional de errores para que las rutas de
// guardado/carga puedan usarse tanto con diagnostico detallado como sin el.
void SetError(std::string* error_message, const std::string& value)
{
    if (error_message)
    {
        *error_message = value;
    }
}

double QuaternionYaw(const geometry_msgs::msg::Quaternion& q)
{
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

double NormalizeAngle(double angle)
{
    constexpr double kPi = 3.14159265358979323846;
    while (angle > kPi)
    {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi)
    {
        angle += 2.0 * kPi;
    }
    return angle;
}

RawPoseChange CompareKeyFramePose(const RawKeyFrameId& id,
                                  const orbslam3_msgs::msg::OrbKeyFrame& old_kf,
                                  const orbslam3_msgs::msg::OrbKeyFrame& new_kf)
{
    RawPoseChange change;
    change.keyframe_id = id;
    const double dx = new_kf.pose.position.x - old_kf.pose.position.x;
    const double dy = new_kf.pose.position.y - old_kf.pose.position.y;
    const double dz = new_kf.pose.position.z - old_kf.pose.position.z;
    change.delta_translation_m = std::sqrt(dx * dx + dy * dy + dz * dz);
    change.delta_yaw_rad =
        std::abs(NormalizeAngle(QuaternionYaw(new_kf.pose.orientation) -
                                QuaternionYaw(old_kf.pose.orientation)));
    change.large_change =
        change.delta_translation_m > 0.05 || change.delta_yaw_rad > 0.01;
    return change;
}

bool KeyFramePoseEqual(
    const orbslam3_msgs::msg::OrbKeyFrame& lhs,
    const orbslam3_msgs::msg::OrbKeyFrame& rhs)
{
    return lhs.pose == rhs.pose;
}

bool KeyFrameAssociationsEqual(
    const orbslam3_msgs::msg::OrbKeyFrame& lhs,
    const orbslam3_msgs::msg::OrbKeyFrame& rhs)
{
    return lhs.mappoint_ids == rhs.mappoint_ids;
}

bool KeyFrameConnectivityEqual(
    const orbslam3_msgs::msg::OrbKeyFrame& lhs,
    const orbslam3_msgs::msg::OrbKeyFrame& rhs)
{
    return lhs.connected_keyframe_ids == rhs.connected_keyframe_ids &&
           lhs.connected_keyframe_weights == rhs.connected_keyframe_weights &&
           lhs.parent_keyframe_id == rhs.parent_keyframe_id &&
           lhs.child_keyframe_ids == rhs.child_keyframe_ids &&
           lhs.loop_edge_keyframe_ids == rhs.loop_edge_keyframe_ids;
}

bool KeyFrameAppearanceEqual(
    const orbslam3_msgs::msg::OrbKeyFrame& lhs,
    const orbslam3_msgs::msg::OrbKeyFrame& rhs)
{
    return lhs.keypoints == rhs.keypoints &&
           lhs.bow_word_ids == rhs.bow_word_ids &&
           lhs.bow_word_values == rhs.bow_word_values &&
           lhs.feat_node_ids == rhs.feat_node_ids &&
           lhs.feat_node_start_indices == rhs.feat_node_start_indices &&
           lhs.feat_node_sizes == rhs.feat_node_sizes &&
           lhs.feat_indices == rhs.feat_indices &&
           lhs.is_bad == rhs.is_bad;
}

bool MapPointGeometryEqual(
    const orbslam3_msgs::msg::OrbMapPoint& lhs,
    const orbslam3_msgs::msg::OrbMapPoint& rhs)
{
    return lhs.position == rhs.position &&
           lhs.descriptor == rhs.descriptor &&
           lhs.is_bad == rhs.is_bad &&
           lhs.normal_vector == rhs.normal_vector &&
           lhs.min_distance == rhs.min_distance &&
           lhs.max_distance == rhs.max_distance;
}

bool MapPointAssociationsEqual(
    const orbslam3_msgs::msg::OrbMapPoint& lhs,
    const orbslam3_msgs::msg::OrbMapPoint& rhs)
{
    return lhs.reference_keyframe_id == rhs.reference_keyframe_id &&
           lhs.observations == rhs.observations;
}

bool MapPointMetadataEqual(
    const orbslam3_msgs::msg::OrbMapPoint& lhs,
    const orbslam3_msgs::msg::OrbMapPoint& rhs)
{
    return lhs.observations_count == rhs.observations_count &&
           lhs.found_ratio == rhs.found_ratio;
}

std::set<uint64_t> ObservingKeyFrameIds(
    const orbslam3_msgs::msg::OrbMapPoint& mappoint)
{
    std::set<uint64_t> result;
    result.insert(mappoint.reference_keyframe_id);
    for (const auto& observation : mappoint.observations)
    {
        result.insert(observation.keyframe_id);
    }
    return result;
}

}  // namespace

std::string ToString(const RawSubmapId& id)
{
    // F1C: la identidad raw siempre es `(drone_id, map_epoch)`, no solo dron.
    // Este texto aparece en logs/diagnostico y evita confundir submapas tras resets.
    std::ostringstream oss;
    oss << "drone=" << id.drone_id << ":epoch=" << id.map_epoch;
    return oss.str();
}

const char* ToString(RawJournalEntryKind kind)
{
    // F1C: nombres estables para logs y documentacion del journal raw.
    switch (kind)
    {
        case RawJournalEntryKind::Delta:
            return "delta";
        case RawJournalEntryKind::FullSnapshot:
            return "full_snapshot";
    }
    return "unknown";
}

RawInsertResult RawMapDatabase::InsertDelta(uint64_t arrival_id, const OrbMap& delta)
{
    // F1C: entrada incremental llegada desde el servidor. Mantiene semantica
    // de delta y delega la aplicacion comun en `InsertMap`.
    return InsertMap(arrival_id, RawJournalEntryKind::Delta, delta);
}

RawInsertResult RawMapDatabase::InsertFullSnapshot(uint64_t arrival_id, const OrbMap& snapshot)
{
    // F1C: snapshot completo usado para inicializar/reemplazar el estado raw
    // de un submapa antes de seguir aplicando deltas posteriores.
    return InsertMap(arrival_id, RawJournalEntryKind::FullSnapshot, snapshot);
}

RawInsertResult RawMapDatabase::InsertMap(uint64_t arrival_id,
                                          RawJournalEntryKind kind,
                                          const OrbMap& map)
{
    // F1C: aplica un `OrbMap` raw al estado agregado y lo conserva en journal.
    // El estado indexado permite consultas rapidas; el journal permite replay
    // posterior sin inventar datos ni depender de Gazebo.
    const RawSubmapId submap_id{map.drone_id, map.map_epoch};
    const bool new_submap = submaps_.find(submap_id) == submaps_.end();
    RawInsertResult result;
    result.new_submap = new_submap;
    result.submap_id = submap_id;
    result.kind = kind;
    result.arrival_id = arrival_id;

    // F1C: `operator[]` crea el submapa si no existe. La clave compuesta evita
    // mezclar mapas locales distintos del mismo dron tras cambios de epoch.
    RawSubmap& submap = submaps_[submap_id];
    if (new_submap)
    {
        submap.id = submap_id;
        result.has_material_changes = true;
        ++material_revision_;
    }

    // F1G: aunque el wrapper entregue un snapshot completo, esta fase usa una
    // reconciliacion conservadora. Insertamos/actualizamos lo recibido y solo
    // marcamos como bad lo que ORB-SLAM3 declara explicitamente; no borramos
    // elementos ausentes para evitar perder raw util por una resincronizacion.
    if (kind == RawJournalEntryKind::FullSnapshot)
    {
        ++submap.full_snapshot_count;
    }
    else
    {
        ++submap.delta_count;
    }

    // F1C: metadata de camara y secuencia se actualiza con el ultimo mensaje
    // recibido para ese submapa; no se transforma aqui a coordenadas globales.
    submap.drone_name = map.drone_name;
    submap.map_frame = map.map_frame;
    submap.last_map_sequence = map.map_sequence;
    submap.last_arrival_id = arrival_id;
    submap.fx = map.fx;
    submap.fy = map.fy;
    submap.cx = map.cx;
    submap.cy = map.cy;
    submap.bf = map.bf;
    submap.image_width = map.image_width;
    submap.image_height = map.image_height;

    // F1C: los deltas se aplican por ID local dentro del submapa. No se crea
    // ningun ID global plano ni se transforma la pose local del KeyFrame.
    for (const auto& keyframe : map.keyframes)
    {
        const auto previous = submap.keyframes.find(keyframe.id);
        const RawKeyFrameId keyframe_id{
            map.drone_id,
            map.map_epoch,
            keyframe.id};
        result.touched_keyframe_ids.push_back(keyframe_id);
        RawKeyFrameMapPointDelta mappoint_delta;
        mappoint_delta.keyframe_id = keyframe_id;
        if (previous == submap.keyframes.end())
        {
            ++result.new_keyframes;
            result.has_material_changes = true;
            result.new_keyframe_ids.push_back(keyframe_id);
            result.covisibility_changed_keyframe_ids.push_back(
                keyframe_id);
            mappoint_delta.new_keyframe = true;
            mappoint_delta.added_mappoint_ids.reserve(
                keyframe.mappoint_ids.size());
            for (const auto local_mappoint_id : keyframe.mappoint_ids)
            {
                mappoint_delta.added_mappoint_ids.push_back(
                    RawMapPointId{
                        map.drone_id,
                        map.map_epoch,
                        local_mappoint_id});
            }
            RawKeyFrameRevision revision;
            revision.material = ++material_revision_;
            revision.pose = revision.material;
            revision.associations = revision.material;
            revision.appearance = revision.material;
            revision.geometry = revision.material;
            revision.metadata = revision.material;
            submap.keyframe_revisions[keyframe.id] = revision;
            result.appearance_changed_keyframe_ids.push_back(keyframe_id);
            result.geometry_changed_keyframe_ids.push_back(keyframe_id);
            result.has_loop_material_changes = true;
        }
        else
        {
            ++result.received_existing_keyframes;
            const bool pose_changed =
                !KeyFramePoseEqual(previous->second, keyframe);
            const bool associations_changed =
                !KeyFrameAssociationsEqual(previous->second, keyframe);
            const bool connectivity_changed =
                !KeyFrameConnectivityEqual(previous->second, keyframe);
            const bool appearance_changed =
                !KeyFrameAppearanceEqual(previous->second, keyframe);
            const bool content_changed = previous->second != keyframe;
            if (content_changed)
            {
                ++result.updated_keyframes;
                result.updated_keyframe_ids.push_back(keyframe_id);
                result.has_material_changes = true;
                auto& revision = submap.keyframe_revisions[keyframe.id];
                revision.material = ++material_revision_;
                if (pose_changed)
                {
                    revision.pose = revision.material;
                    revision.geometry = revision.material;
                    result.geometry_changed_keyframe_ids.push_back(keyframe_id);
                    result.has_loop_material_changes = true;
                }
                if (associations_changed)
                {
                    revision.associations = revision.material;
                    revision.geometry = revision.material;
                    result.geometry_changed_keyframe_ids.push_back(keyframe_id);
                    result.has_loop_material_changes = true;
                }
                if (connectivity_changed)
                {
                    revision.geometry = revision.material;
                    result.geometry_changed_keyframe_ids.push_back(keyframe_id);
                    result.has_loop_material_changes = true;
                }
                if (appearance_changed)
                {
                    revision.appearance = revision.material;
                    revision.geometry = revision.material;
                    result.appearance_changed_keyframe_ids.push_back(keyframe_id);
                    result.geometry_changed_keyframe_ids.push_back(keyframe_id);
                    result.has_loop_material_changes = true;
                }
                if (!pose_changed && !associations_changed &&
                    !connectivity_changed && !appearance_changed)
                {
                    revision.metadata = revision.material;
                    result.has_metadata_changes = true;
                }
            }
            else
            {
                ++result.unchanged_keyframes;
            }
            if (previous->second.connected_keyframe_ids !=
                    keyframe.connected_keyframe_ids ||
                previous->second.connected_keyframe_weights !=
                    keyframe.connected_keyframe_weights)
            {
                result.covisibility_changed_keyframe_ids.push_back(
                    keyframe_id);
            }
            const std::set<uint64_t> previous_mappoint_ids(
                previous->second.mappoint_ids.begin(),
                previous->second.mappoint_ids.end());
            for (const auto local_mappoint_id : keyframe.mappoint_ids)
            {
                if (previous_mappoint_ids.find(local_mappoint_id) ==
                    previous_mappoint_ids.end())
                {
                    mappoint_delta.added_mappoint_ids.push_back(
                        RawMapPointId{
                            map.drone_id,
                            map.map_epoch,
                            local_mappoint_id});
                }
            }
            if (pose_changed)
            {
                const RawPoseChange change =
                    CompareKeyFramePose(keyframe_id, previous->second, keyframe);
                if (change.delta_translation_m > 1e-6 || change.delta_yaw_rad > 1e-6)
                {
                    result.raw_pose_changed_keyframes.push_back(change);
                    if (change.large_change)
                    {
                        ++result.large_raw_pose_changed_keyframes;
                    }
                }
            }
        }
        if (!mappoint_delta.added_mappoint_ids.empty())
        {
            result.added_keyframe_mappoint_associations +=
                mappoint_delta.added_mappoint_ids.size();
            result.keyframe_mappoint_deltas.push_back(
                std::move(mappoint_delta));
        }
        else
        {
            ++result.keyframes_without_mappoint_changes;
        }
        if (keyframe.is_bad)
        {
            ++result.bad_keyframes;
        }
        if (previous == submap.keyframes.end() || previous->second != keyframe)
        {
            submap.keyframes[keyframe.id] = keyframe;
        }
    }

    // F1C: MapPoints raw se actualizan por su ID local. Si en el futuro se
    // fusionan landmarks, esa fusion vivira fuera de RawMapDatabase.
    for (const auto& mappoint : map.mappoints)
    {
        const auto previous = submap.mappoints.find(mappoint.id);
        const RawMapPointId mappoint_id{
            map.drone_id,
            map.map_epoch,
            mappoint.id};
        bool geometry_changed = false;
        bool associations_changed = false;
        bool metadata_changed = false;
        std::set<uint64_t> affected_keyframes;
        if (previous == submap.mappoints.end())
        {
            ++result.new_mappoints;
            result.new_mappoint_ids.push_back(mappoint_id);
            result.has_material_changes = true;
            result.has_loop_material_changes = true;
            geometry_changed = true;
            associations_changed = true;
            affected_keyframes = ObservingKeyFrameIds(mappoint);
        }
        else
        {
            ++result.received_existing_mappoints;
            if (previous->second != mappoint)
            {
                ++result.updated_mappoints;
                result.updated_mappoint_ids.push_back(mappoint_id);
                result.has_material_changes = true;
                geometry_changed =
                    !MapPointGeometryEqual(previous->second, mappoint);
                associations_changed =
                    !MapPointAssociationsEqual(previous->second, mappoint);
                metadata_changed =
                    !MapPointMetadataEqual(previous->second, mappoint);
                affected_keyframes = ObservingKeyFrameIds(previous->second);
                const auto current_observers = ObservingKeyFrameIds(mappoint);
                affected_keyframes.insert(
                    current_observers.begin(), current_observers.end());
            }
            else
            {
                ++result.unchanged_mappoints;
            }
        }
        if (mappoint.is_bad)
        {
            ++result.bad_mappoints;
        }
        if (previous == submap.mappoints.end() || previous->second != mappoint)
        {
            submap.mappoints[mappoint.id] = mappoint;
        }
        if (geometry_changed || associations_changed)
        {
            const uint64_t revision_id = ++material_revision_;
            result.geometry_changed_mappoint_ids.push_back(mappoint_id);
            result.has_loop_material_changes = true;
            for (const auto local_kf_id : affected_keyframes)
            {
                const auto revision_it =
                    submap.keyframe_revisions.find(local_kf_id);
                if (revision_it == submap.keyframe_revisions.end())
                {
                    continue;
                }
                revision_it->second.material = revision_id;
                revision_it->second.geometry = revision_id;
                if (associations_changed)
                {
                    revision_it->second.associations = revision_id;
                }
                result.geometry_changed_keyframe_ids.push_back(
                    RawKeyFrameId{
                        map.drone_id, map.map_epoch, local_kf_id});
            }
        }
        if (metadata_changed)
        {
            const uint64_t revision_id = ++material_revision_;
            result.metadata_changed_mappoint_ids.push_back(mappoint_id);
            result.has_metadata_changes = true;
            for (const auto local_kf_id : affected_keyframes)
            {
                const auto revision_it =
                    submap.keyframe_revisions.find(local_kf_id);
                if (revision_it == submap.keyframe_revisions.end())
                {
                    continue;
                }
                revision_it->second.material = revision_id;
                revision_it->second.metadata = revision_id;
            }
        }
    }

    std::sort(
        result.appearance_changed_keyframe_ids.begin(),
        result.appearance_changed_keyframe_ids.end());
    result.appearance_changed_keyframe_ids.erase(
        std::unique(
            result.appearance_changed_keyframe_ids.begin(),
            result.appearance_changed_keyframe_ids.end()),
        result.appearance_changed_keyframe_ids.end());
    std::sort(
        result.geometry_changed_keyframe_ids.begin(),
        result.geometry_changed_keyframe_ids.end());
    result.geometry_changed_keyframe_ids.erase(
        std::unique(
            result.geometry_changed_keyframe_ids.begin(),
            result.geometry_changed_keyframe_ids.end()),
        result.geometry_changed_keyframe_ids.end());

    // F1C: el journal guarda el mensaje completo en orden de llegada para poder
    // persistirlo y reproducirlo mas tarde como si volvieran a llegar deltas.
    journal_.push_back(RawJournalEntry{arrival_id, kind, map});

    result.material_revision = material_revision_;
    result.stats = ComputeStatsLocked();
    return result;
}

bool RawMapDatabase::HasSubmap(const RawSubmapId& id) const
{
    // F1C: consulta pura; no crea entradas nuevas en la base raw.
    return submaps_.find(id) != submaps_.end();
}

bool RawMapDatabase::HasKeyFrame(const RawKeyFrameId& id) const
{
    // F1C: valida el KF dentro de su submapa exacto para no colisionar IDs
    // locales iguales procedentes de drones o epochs distintos.
    const auto* submap = GetSubmap(RawSubmapId{id.drone_id, id.map_epoch});
    return submap && submap->keyframes.find(id.local_kf_id) != submap->keyframes.end();
}

bool RawMapDatabase::HasMapPoint(const RawMapPointId& id) const
{
    // F1C: igual que los KFs, los MPs raw conservan su ID local y solo son
    // unicos al combinarlos con `(drone_id, map_epoch)`.
    const auto* submap = GetSubmap(RawSubmapId{id.drone_id, id.map_epoch});
    return submap && submap->mappoints.find(id.local_mp_id) != submap->mappoints.end();
}

const RawSubmap* RawMapDatabase::GetSubmap(const RawSubmapId& id) const
{
    // F1C: devuelve un puntero de solo lectura al estado agregado del submapa.
    // `nullptr` distingue "no existe" sin crear estado accidentalmente.
    const auto it = submaps_.find(id);
    if (it == submaps_.end())
    {
        return nullptr;
    }
    return &it->second;
}

const orbslam3_msgs::msg::OrbKeyFrame* RawMapDatabase::GetKeyFrame(const RawKeyFrameId& id) const
{
    // F1D: consulta minima para que GlobalPoseStore derive poses globales desde
    // la pose local raw sin copiar ni modificar el KeyFrame almacenado.
    const auto* submap = GetSubmap(RawSubmapId{id.drone_id, id.map_epoch});
    if (!submap)
    {
        return nullptr;
    }

    const auto it = submap->keyframes.find(id.local_kf_id);
    if (it == submap->keyframes.end())
    {
        return nullptr;
    }
    return &it->second;
}

const orbslam3_msgs::msg::OrbMapPoint* RawMapDatabase::GetMapPoint(const RawMapPointId& id) const
{
    // F1D: simetrica a GetKeyFrame; mantiene acceso raw de solo lectura para
    // futuras capas sin abrir la puerta a mutaciones desde fuera de la DB.
    const auto* submap = GetSubmap(RawSubmapId{id.drone_id, id.map_epoch});
    if (!submap)
    {
        return nullptr;
    }

    const auto it = submap->mappoints.find(id.local_mp_id);
    if (it == submap->mappoints.end())
    {
        return nullptr;
    }
    return &it->second;
}

RawKeyFrameRevision RawMapDatabase::GetKeyFrameRevision(
    const RawKeyFrameId& id) const
{
    const auto* submap = GetSubmap(RawSubmapId{id.drone_id, id.map_epoch});
    if (!submap)
    {
        return {};
    }
    const auto it = submap->keyframe_revisions.find(id.local_kf_id);
    if (it == submap->keyframe_revisions.end())
    {
        return {};
    }
    return it->second;
}

uint64_t RawMapDatabase::GetMaterialRevision() const
{
    return material_revision_;
}

std::vector<RawSubmapId> RawMapDatabase::GetSubmapIds() const
{
    std::vector<RawSubmapId> ids;
    ids.reserve(submaps_.size());
    // F1C: se enumeran las claves compuestas vigentes; el orden lo determina
    // `std::map` y es suficiente para diagnostico/replay determinista.
    for (const auto& [id, _] : submaps_)
    {
        ids.push_back(id);
    }
    return ids;
}

std::vector<RawKeyFrameId> RawMapDatabase::GetKeyFrameIdsForSubmap(const RawSubmapId& id) const
{
    std::vector<RawKeyFrameId> ids;
    const auto* submap = GetSubmap(id);
    if (!submap)
    {
        return ids;
    }

    ids.reserve(submap->keyframes.size());
    // F1C: reconstruye IDs completos desde las claves locales almacenadas en
    // el submapa para que el llamador no pierda identidad de dron/epoch.
    for (const auto& [local_id, _] : submap->keyframes)
    {
        ids.push_back(RawKeyFrameId{id.drone_id, id.map_epoch, local_id});
    }
    return ids;
}

std::vector<RawMapPointId> RawMapDatabase::GetMapPointIdsForSubmap(const RawSubmapId& id) const
{
    std::vector<RawMapPointId> ids;
    const auto* submap = GetSubmap(id);
    if (!submap)
    {
        return ids;
    }

    ids.reserve(submap->mappoints.size());
    // F1C: misma estrategia que KFs; los IDs locales solo son seguros dentro
    // del submapa al que pertenecen.
    for (const auto& [local_id, _] : submap->mappoints)
    {
        ids.push_back(RawMapPointId{id.drone_id, id.map_epoch, local_id});
    }
    return ids;
}

RawMapDatabase RawMapDatabase::CreateStateSnapshot() const
{
    RawMapDatabase snapshot;
    snapshot.submaps_ = submaps_;
    snapshot.material_revision_ = material_revision_;
    return snapshot;
}

RawMapDatabase RawMapDatabase::CreateSubsetSnapshot(
    const std::set<RawKeyFrameId>& geometry_keyframes) const
{
    RawMapDatabase snapshot;
    snapshot.material_revision_ = material_revision_;
    for (const auto& [submap_id, source] : submaps_)
    {
        RawSubmap target;
        target.id = source.id;
        target.drone_name = source.drone_name;
        target.map_frame = source.map_frame;
        target.last_map_sequence = source.last_map_sequence;
        target.last_arrival_id = source.last_arrival_id;
        target.delta_count = source.delta_count;
        target.full_snapshot_count = source.full_snapshot_count;
        target.fx = source.fx;
        target.fy = source.fy;
        target.cx = source.cx;
        target.cy = source.cy;
        target.bf = source.bf;
        target.image_width = source.image_width;
        target.image_height = source.image_height;
        target.keyframe_revisions = source.keyframe_revisions;

        std::set<uint64_t> geometry_mappoint_ids;
        for (const auto& [local_kf_id, source_kf] : source.keyframes)
        {
            const RawKeyFrameId id{
                submap_id.drone_id, submap_id.map_epoch, local_kf_id};
            if (geometry_keyframes.find(id) != geometry_keyframes.end())
            {
                target.keyframes.emplace(local_kf_id, source_kf);
                geometry_mappoint_ids.insert(
                    source_kf.mappoint_ids.begin(),
                    source_kf.mappoint_ids.end());
                continue;
            }

            orbslam3_msgs::msg::OrbKeyFrame visual;
            visual.id = source_kf.id;
            visual.stamp = source_kf.stamp;
            visual.pose = source_kf.pose;
            visual.camera_id = source_kf.camera_id;
            visual.frame_id = source_kf.frame_id;
            visual.mappoint_ids = source_kf.mappoint_ids;
            visual.is_bad = source_kf.is_bad;
            visual.bow_word_ids = source_kf.bow_word_ids;
            visual.bow_word_values = source_kf.bow_word_values;
            target.keyframes.emplace(local_kf_id, std::move(visual));
        }
        for (const auto local_mp_id : geometry_mappoint_ids)
        {
            const auto point_it = source.mappoints.find(local_mp_id);
            if (point_it != source.mappoints.end())
            {
                target.mappoints.emplace(local_mp_id, point_it->second);
            }
        }
        snapshot.submaps_.emplace(submap_id, std::move(target));
    }
    return snapshot;
}

RawDatabaseStats RawMapDatabase::GetDatabaseStats() const
{
    // F1C: estadisticas derivadas del estado actual y del journal; no se cachean
    // para evitar desincronizacion durante cargas/replays.
    return ComputeStatsLocked();
}

std::vector<RawJournalEntry> RawMapDatabase::GetJournalCopy() const
{
    // F1C: copia segura para consumidores que necesiten inspeccionar o iterar
    // sin quedar acoplados a la vida interna de la base.
    return journal_;
}

const std::vector<RawJournalEntry>& RawMapDatabase::Journal() const
{
    // F1C: acceso eficiente de solo lectura para rutas internas que no deben
    // mutar ni reordenar el journal.
    return journal_;
}

void RawMapDatabase::AddFiducialObservation(
    const RecordedFiducialObservation& observation)
{
    // F1E: el journal fiducial conserva eventos ya asociados a un `arrival_id`
    // para replay. No altera KFs/MPs raw ni decide anchors; eso vive fuera.
    fiducial_observation_journal_.push_back(observation);
}

std::vector<RecordedFiducialObservation>
RawMapDatabase::GetFiducialObservationJournalCopy() const
{
    // F1E: copia segura para que el servidor pueda construir un indice de replay
    // sin depender de la vida interna de RawMapDatabase.
    return fiducial_observation_journal_;
}

std::vector<RecordedFiducialObservation>
RawMapDatabase::GetFiducialObservationsForArrival(uint64_t arrival_id) const
{
    std::vector<RecordedFiducialObservation> observations;
    for (const auto& observation : fiducial_observation_journal_)
    {
        if (observation.arrival_id == arrival_id)
        {
            observations.push_back(observation);
        }
    }
    return observations;
}

void RawMapDatabase::Clear()
{
    // F1C: reset completo usado por carga/replay/pruebas; borra tanto el estado
    // agregado como la fuente historica que permite reconstruirlo.
    submaps_.clear();
    journal_.clear();
    fiducial_observation_journal_.clear();
    material_revision_ = 0;
}

RawDatabaseStats RawMapDatabase::ComputeStatsLocked() const
{
    // F1C: calcula contadores observables de la base raw. No valida geometria ni
    // coherencia SLAM; solo resume lo recibido y aplicado.
    RawDatabaseStats stats;
    stats.journal_entries = journal_.size();
    stats.fiducial_observations = fiducial_observation_journal_.size();
    stats.submaps = submaps_.size();

    // F1C: el journal conserva el historial de entradas, por eso de aqui salen
    // los contadores de deltas/snapshots y el ultimo `arrival_id` visto.
    for (const auto& entry : journal_)
    {
        if (entry.kind == RawJournalEntryKind::Delta)
        {
            ++stats.delta_entries;
        }
        else if (entry.kind == RawJournalEntryKind::FullSnapshot)
        {
            ++stats.full_snapshot_entries;
        }
        stats.last_arrival_id = entry.arrival_id;
    }

    // F1C: el estado agregado refleja el resultado final tras aplicar snapshots
    // y deltas; estos totales pueden ser menores que la suma historica.
    for (const auto& [_, submap] : submaps_)
    {
        stats.keyframes += submap.keyframes.size();
        stats.mappoints += submap.mappoints.size();
    }

    return stats;
}

bool RawMapDatabase::SaveToPath(const std::string& path, std::string* error_message) const
{
    namespace fs = std::filesystem;

    try
    {
        // F1C: el `.record` vive como snapshot del journal raw. Si el directorio
        // padre no existe, se crea aqui para que el servidor pueda persistirlo
        // sin pasos manuales previos.
        const fs::path output_path(path);
        if (output_path.has_parent_path())
        {
            fs::create_directories(output_path.parent_path());
        }

        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out)
        {
            SetError(error_message, "no se pudo abrir el archivo para escritura");
            return false;
        }

        // F1C: cabecera minima del formato propio: magic, version y numero de
        // entradas. Permite rechazar archivos que no sean rawdb antes de leerlos.
        if (!WritePod(out, kRecordMagic) || !WritePod(out, kRecordVersion))
        {
            SetError(error_message, "no se pudo escribir la cabecera");
            return false;
        }

        const uint64_t entry_count = journal_.size();
        if (!WritePod(out, entry_count))
        {
            SetError(error_message, "no se pudo escribir el numero de entradas");
            return false;
        }

        rclcpp::Serialization<OrbMap> serializer;

        // F1C: se serializa cada `OrbMap` completo con el serializador ROS 2.
        // Asi el replay no reconstruye contadores artificiales: reinyecta los
        // mismos mensajes que quedaron en el journal.
        for (const auto& entry : journal_)
        {
            rclcpp::SerializedMessage serialized;
            serializer.serialize_message(&entry.map, &serialized);
            const auto& rcl_message = serialized.get_rcl_serialized_message();

            const uint8_t kind = static_cast<uint8_t>(entry.kind);
            const uint64_t payload_size = rcl_message.buffer_length;
            // F1C: cada entrada guarda primero metadata pequena y despues el
            // payload ROS 2; asi la carga sabe cuanto leer sin parsear a ciegas.
            if (!WritePod(out, entry.arrival_id) ||
                !WritePod(out, kind) ||
                !WritePod(out, payload_size))
            {
                SetError(error_message, "no se pudo escribir metadata de entrada");
                return false;
            }

            out.write(reinterpret_cast<const char*>(rcl_message.buffer), payload_size);
            if (!out)
            {
                SetError(error_message, "no se pudo escribir payload serializado");
                return false;
            }
        }

        // F1E: version 2 del `.record`. Tras el journal raw se persisten las
        // observaciones fiduciales aceptadas, asociadas a `arrival_id`. El replay
        // las consumira sin consultar GT en vivo.
        const uint64_t fiducial_count = fiducial_observation_journal_.size();
        if (!WritePod(out, fiducial_count))
        {
            SetError(error_message, "no se pudo escribir el numero de observaciones fiduciales");
            return false;
        }

        for (const auto& observation : fiducial_observation_journal_)
        {
            if (!WriteFiducialObservation(out, observation))
            {
                SetError(error_message, "no se pudo escribir una observacion fiducial");
                return false;
            }
        }
    }
    catch (const std::exception& ex)
    {
        SetError(error_message, ex.what());
        return false;
    }

    return true;
}

bool RawMapDatabase::LoadFromPath(const std::string& path, std::string* error_message)
{
    // F1C: carga un `.record` persistido. La base actual solo se reemplaza al
    // final, cuando todas las entradas se han deserializado y aplicado bien.
    std::ifstream in(path, std::ios::binary);
    if (!in)
    {
        SetError(error_message, "no se pudo abrir el archivo para lectura");
        return false;
    }

    uint32_t magic = 0;
    uint32_t version = 0;
    uint64_t entry_count = 0;
    if (!ReadPod(in, magic) || !ReadPod(in, version) || !ReadPod(in, entry_count))
    {
        SetError(error_message, "cabecera incompleta");
        return false;
    }
    // F1C: magic/version protegen contra abrir archivos de otro formato o una
    // version futura que este codigo no sabe interpretar.
    if (magic != kRecordMagic || (version != 1U && version != kRecordVersion))
    {
        SetError(error_message, "formato rawdb no reconocido");
        return false;
    }

    RawMapDatabase loaded;
    rclcpp::Serialization<OrbMap> serializer;

    // F1C: se reconstruye una base temporal reinyectando cada entrada del
    // journal en orden. Esto reproduce el mismo camino que los deltas en vivo.
    for (uint64_t i = 0; i < entry_count; ++i)
    {
        uint64_t arrival_id = 0;
        uint8_t kind_raw = 0;
        uint64_t payload_size = 0;
        if (!ReadPod(in, arrival_id) || !ReadPod(in, kind_raw) || !ReadPod(in, payload_size))
        {
            SetError(error_message, "metadata de entrada incompleta");
            return false;
        }

        rclcpp::SerializedMessage serialized(payload_size);
        auto& rcl_message = serialized.get_rcl_serialized_message();
        in.read(reinterpret_cast<char*>(rcl_message.buffer), payload_size);
        if (!in)
        {
            SetError(error_message, "payload de entrada incompleto");
            return false;
        }
        rcl_message.buffer_length = payload_size;

        OrbMap map;
        serializer.deserialize_message(&serialized, &map);

        const auto kind = static_cast<RawJournalEntryKind>(kind_raw);
        // F1C: se conserva la semantica original de cada entrada. Los snapshots
        // sustituyen el submapa y los deltas actualizan IDs locales existentes.
        if (kind == RawJournalEntryKind::FullSnapshot)
        {
            loaded.InsertFullSnapshot(arrival_id, map);
        }
        else
        {
            loaded.InsertDelta(arrival_id, map);
        }
    }

    if (version >= 2U)
    {
        uint64_t fiducial_count = 0;
        if (!ReadPod(in, fiducial_count))
        {
            SetError(error_message, "numero de observaciones fiduciales incompleto");
            return false;
        }

        // F1E: carga el journal fiducial despues de reconstruir el raw. No se
        // aplica aqui para mantener RawMapDatabase como almacenamiento; el
        // servidor decide cuando reinyectarlo durante replay.
        for (uint64_t i = 0; i < fiducial_count; ++i)
        {
            RecordedFiducialObservation observation;
            if (!ReadFiducialObservation(in, observation))
            {
                SetError(error_message, "observacion fiducial incompleta");
                return false;
            }
            loaded.AddFiducialObservation(observation);
        }
    }

    *this = std::move(loaded);
    return true;
}

}  // namespace orbslam3_multi
