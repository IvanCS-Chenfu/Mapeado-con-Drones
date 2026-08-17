# `GlobalMapBuilder`

## Rol

Vista materializada incremental del mapa sparse publicable. Mantiene una cache
de poses world de KFs, slots reutilizables de MPs y los indices inversos
necesarios para recalcular solo IDs afectados.

## Estado interno

```text
keyframe_world_cache_             RawKeyFrameId -> GlobalKeyFrameView
keyframe_projection_cache_        KF -> world_T_kf * inverse(local_T_kf)
sparse_point_slots_               slots estables/reutilizables
point_slot_by_id_                 RawMapPointId -> slot
keyframe_to_mappoints_            KF asociado -> MPs
dirty_keyframes_/dirty_mappoints_ IDs pendientes
removed_mappoints_                retiradas pendientes
deferred_unanchored_submaps_      una marca acotada por submapa sin anchor
fused_track_cache_                FusedTrackId -> representante publicable
dirty_fused_tracks_               tracks pendientes de recalculo
hidden_raw_members_               miembros raw sustituidos por un track
```

## Reglas geometricas

- Solo existe un KF publico si raw lo conserva activo y el pose store aporta
  pose world activa.
- Cada MP intenta mantener su observador asociado previo; si deja de ser
  valido, usa primero el reference KF y despues observaciones ordenadas.
- La posicion usa `world_T_kf * inverse(local_T_kf) * p_local`.
- Sin observador world valido, el MP se omite. `fallback_submap_points` debe ser
  siempre cero.
- Un KF movido ensucia solo sus MPs asociados; un cambio material de score
  actualiza el valor sin reconstruir toda la nube. Cambios de inputs ORB que
  producen el mismo score no llegan al builder.
- Un commit secundario llama solo a `MarkPoseChanges()` con los KFs realmente
  movidos. No ejecuta `Update()`: el siguiente `PrimaryInput` expande los MPs
  asociados mediante `keyframe_to_mappoints_`, recalcula y publica la revision
  coherente.
- Un commit 3P llama a `MarkFusionChanges()` y `MarkScoreChanges()`: oculta
  miembros raw del track, conserva un unico representante ponderado y ensucia
  tracks afectados. El siguiente `PrimaryInput` materializa el resultado.
- Si cambian pose, geometria raw o score de un miembro, los indices inversos
  ensucian su track sin recorrer todos los tracks. Un track retirado libera o
  reasigna sus miembros segun el `FusionChangeSet`.
- `MarkRawChanges()` usa las categorias granulares 3G: pose/asociacion/
  covisibilidad para KFs y geometria/asociacion/invalidation para MPs. Un cambio
  solo de score llega por `MarkScoreChanges()`.
- Los dirty notificados por un full snapshot pueden acumularse sin ejecutar
  `Update()`; el siguiente delta normal los consume junto con sus propios IDs.
- `Update()` no recorre KFs/MPs de un submapa sin anchor. Descarta esos IDs y
  conserva una sola marca por submapa; cuando aparece el primer anchor obtiene
  todos los IDs activos de `RawMapDatabase` y hace un backfill completo.
- Para IDs anclados, `Update()` obtiene un `RawBuilderSnapshot` ligero en batch
  y trabaja sobre el snapshot inmutable. No toma un lock raw ni copia mensajes
  ROS completos por MP/observador; un lote suplementario cubre IDs ensuciados
  dinamicamente al retirar un KF cacheado.
- Cada KF se valida como maximo una vez por `Update()`. La transformada de
  proyeccion se recalcula solo si cambia su pose raw o world; todos sus MPs
  reutilizan la matriz validada y evitan locks, conversiones e inversas.
- `Update()` incrementa `publication_revision` solo si cambia la vista publica
  y devuelve cloud/KFs de la misma revision.
- La telemetria distingue IDs diferidos y entidades recuperadas por backfill.

## Referencias

```text
orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp
  -> GlobalSparsePoint / GlobalKeyFrameView / GlobalMapBuildResult
  -> GlobalMapBuilder
  -> rg -n "GlobalMapBuildResult|class GlobalMapBuilder"

orbslam3_multi/src/global_map_builder.cpp
  -> MarkRawChanges / MarkPoseChanges / MarkScoreChanges / Update
  -> MarkFusionChanges / UpdateFusedTrack / PopulateOutput
  -> rg -n "GlobalMapBuilder::(Mark|Update|UpdateFusedTrack|PopulateOutput)"

orbslam3_multi/test/test_global_map_builder.cpp
  -> pre-anchor vacio, backfill, reproyeccion focal y ausencia de fallback
```

El builder no publica ROS, no ejecuta timers y no se despierta desde tareas
secundarias. El `PrimaryWorker` lo invoca una vez al final de cada tarea. La
telemetria publica `recalculated_tracks` y `fusion_revision`; no filtra por
score.
