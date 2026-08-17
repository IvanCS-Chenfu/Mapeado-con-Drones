# `RawMapDatabase`

## Rol

`RawMapDatabase` es la base raw creada en `3C` dentro de `orbslam3_multi`.

Guarda exactamente los `orbslam3_msgs::msg::OrbMap` que llegan desde los wrappers, organizados por:

```text
submapa = (drone_id, map_epoch)
```

No transforma poses a `world`, no fusiona landmarks, no calcula score y no optimiza.

Desde `3E`, además del journal raw guarda un journal separado de observaciones fiduciales ya asociadas a `arrival_id`. Ese journal permite reproducir anclajes fiduciales en replay sin consultar GT en vivo.

Desde `3G`, los full snapshots vuelven a formar parte de la ruta activa. `RawMapDatabase` tiene autoridad sobre los datos raw/locales de ORB-SLAM3: un snapshot puede actualizar KFs, MPs y poses locales, pero nunca escribe poses globales ni decide optimizaciones.

Desde `3K`, el apply de optimización se valida explícitamente como no mutación de raw. Las poses optimizadas y propagadas viven en `GlobalPoseStore`; `RawMapDatabase` sigue conservando la información local original o actualizada por ORB-SLAM3.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp
orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp
orbslam3_multi/src/raw_map_database.cpp
```

## Responsabilidades

- `InsertDelta`: inserta o actualiza KFs/MPs de un delta y añade entrada al journal.
- `InsertFullSnapshot`: inserta/reconcilia un snapshot completo con politica `insert_update_no_absent_delete`: actualiza lo recibido, conserva ausentes por ahora y detecta cambios de pose local raw en KFs existentes.
- `GetDatabaseStats`: devuelve contadores de journal, deltas, snapshots, submapas, KFs y MPs.
- `GetJournalCopy`: entrega una copia ordenada del journal para replay.
- `GetKeyFrame`: consulta constante añadida en `3D` para que `GlobalPoseStore` derive poses globales desde la pose local raw sin mutarla.
- `GetMapPoint`: consulta constante añadida en `3D` para mantener acceso raw simétrico de solo lectura.
- `AddFiducialObservation`: guarda una observación fiducial persistible asociada a un `arrival_id`.
- `GetFiducialObservationJournalCopy`: devuelve el journal fiducial completo.
- `GetFiducialObservationsForArrival`: consulta observaciones fiduciales de una entrada concreta para replay.
- `SaveToPath`: guarda el journal serializado en disco, incluyendo deltas, full snapshots y observaciones fiduciales.
- `LoadFromPath`: carga el journal, reconstruye estado raw y permite reinyectar deltas y full snapshots por `arrival_id`.
- `RawInsertResult::new_keyframe_ids`: lista los KFs realmente nuevos de una
  llegada para que consumidores como `LoopDetector` no reconsulten KFs antiguos
  de full snapshots.
- Desde la revisión incremental de `3P`, `RawInsertResult` separa KFs tocados,
  cambios de covisibilidad, KFs sin cambios de asociaciones y deltas exactos
  `RawKeyFrameMapPointDelta`. Un snapshot de un KF conocido expone únicamente
  los IDs de MapPoints añadidos, sin convertirlos en un KF nuevo.

- Desde el rediseño de rendimiento del 2026-08-04, `RawInsertResult` separa
  elementos nuevos, actualizados y sin cambios; expone sus IDs,
  `has_material_changes` y `material_revision`. Un snapshot materialmente
  identico puede conservarse en el journal y ser no-op para geometria, score y
  reconstruccion de nube.
- `RawKeyFrameRevision` separa `pose`, `associations`, `appearance`, `geometry`
  y `metadata`. La igualdad geométrica excluye metadata y la igualdad BoW usa
  solo `appearance`.
- Un cambio geométrico o de asociaciones de un `OrbMapPoint` invalida solo los
  KFs que lo observaban antes o lo observan ahora. `observations_count` y
  `found_ratio` avanzan únicamente `metadata`, sin crear trabajo de loops.
- `CreateSubsetSnapshot` materializa KFs completos y MapPoints únicamente para
  el conjunto geométrico seleccionado. Del resto conserva solo la vista visual
  necesaria para BoW, sin copiar journal ni toda la base de MapPoints.
- `CreateStateSnapshot` copia únicamente el estado material vigente de
  submapas y su `material_revision`. No copia el journal raw ni el journal
  fiducial, por lo que publicación y optimización pueden capturar una vista
  inmutable sin duplicar el historial bajo `live_state_mutex_`.
- Desde `3N`, los KFs raw conservan `bow_word_ids` y `bow_word_values` recibidos
  en `orbslam3_msgs::msg::OrbKeyFrame`; `LoopDetector` los lee de forma
  constante para búsqueda visual. `RawMapDatabase` no calcula ni normaliza BoW.
- Desde `3O`, `SubcloudLoopVerifier` lee de forma constante KFs, MapPoints,
  observaciones, descriptores y covisibilidad nativa para construir subnubes de
  verificación. La base raw no se modifica y no registra loops confirmados.

## Tipos principales

- `RawSubmapId`: `drone_id` + `map_epoch`.
- `RawKeyFrameId`: `drone_id` + `map_epoch` + `local_kf_id`.
- `RawMapPointId`: `drone_id` + `map_epoch` + `local_mp_id`.
- `RawJournalEntry`: `arrival_id`, tipo de entrada y `OrbMap` original.
- `RecordedFiducialObservation`: observación fiducial persistida con pose `world_T_kf`, `fiducial_id`, timestamps, distancia y fuente.
- `RawPoseChange`: cambio de pose local detectado en un KF existente durante un full snapshot.
- `RawInsertResult`: resumen de ingesta con submapa, tipo, `arrival_id`,
  KFs/MPs nuevos, actualizados o sin cambios, cambios de pose raw,
  `touched_keyframe_ids`, `covisibility_changed_keyframe_ids`,
  `keyframe_mappoint_deltas`, `keyframes_without_mappoint_changes` y número de
  asociaciones añadidas, junto con revision material e IDs diferenciales.
- `RawKeyFrameRevision`: revisiones independientes de pose, asociaciones,
  apariencia, geometría observada y metadata usadas para validar cada etapa.

Desde `3D`, `RawKeyFrameId` y `RawMapPointId` tienen operadores de orden para poder usarse como claves de mapas ligeros externos, por ejemplo en `GlobalPoseStore`.

## Logs relacionados

Los logs los emite `orbslam3_server/src/global_map_server.cpp` al usar esta clase:

```text
[F1C-RAWDB-INIT]
[F1C-RAWDB-DELTA-RX]
[F1C-RAWDB-INSERT-DELTA]
[F1C-RAWDB-INSERT-FULL]
[F1C-RAWDB-NEW-SUBMAP]
[F1C-RAWDB-STATS]
[F1C-RAWDB-SAVE]
[F1C-REPLAY-LOAD]
[F1C-REPLAY-DELTA]
[F1C-REPLAY-DONE]
[F1E-FID-JOURNAL-SAVE]
[F1E-FID-REPLAY-OBS]
[F1G-RAWDB-INSERT-FULL]
[F1G-SNAPSHOT-RECONCILE]
[F1G-RAW-POSE-CHANGED]
[F1G-REPLAY-FULL-SNAPSHOT]
[F1K-RAWDB-NOT-MODIFIED]
[F1N-LOOP-NEW-KF-DISPATCH]
```

## Validación 3C

El 2026-07-06 se validó con:

- `284` deltas grabados desde Gazebo;
- `3` submapas raw;
- `142` KFs;
- `17884` MPs;
- dataset `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` de aproximadamente `272M`;
- replay completo sin Gazebo con `[F1C-REPLAY-DONE] entries=284 journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`.

## Validación 3E

El 2026-07-08 se validó `.record` versión 2 con:

- simulación Gazebo `prueba_1` con `SIM-DONE prueba=1 success=true`;
- nuevo `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` tras borrar el anterior;
- `[F1C-RAWDB-SAVE] reason=shutdown ... journal=364 deltas=364 full=0 fiducial_observations=60 submaps=3 kfs=175 mps=21095`;
- replay `prueba_2` con `[F1C-REPLAY-LOAD] ... fiducial_observations=60`;
- `60` eventos `[F1E-FID-REPLAY-OBS]`;
- `[F1C-REPLAY-DONE] entries=364 journal=364 deltas=364 full=0 fiducial_observations=60 submaps=3 kfs=175 mps=21095`.

## Validación 3G

El 2026-07-08 se validó la ruta de full snapshots:

- simulación live `prueba_1` con `SIM-DONE prueba=1 success=true`;
- `8` eventos `[F1G-RAWDB-INSERT-FULL]` en live;
- snapshots con KFs nuevos y actualizados, por ejemplo `new_kfs=3 updated_kfs=24` y `raw_pose_changed=23`;
- record nuevo `codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record` con `368` entradas, `356` deltas, `12` full snapshots, `74` observaciones fiduciales, `4` submapas, `225` KFs y `26165` MPs;
- replay `prueba_2` con `SIM-DONE prueba=2 success=true`;
- `12` eventos `[F1G-REPLAY-FULL-SNAPSHOT]` y `[F1G-RAWDB-INSERT-FULL]`;
- `[F1C-REPLAY-DONE] entries=368 journal=368 deltas=356 full=12 fiducial_observations=74 submaps=4 kfs=225 mps=26165`.

## Restricciones

- La identidad nunca debe reducirse a `drone_id`.
- El journal conserva la secuencia por `arrival_id`; no debe reordenarse por `map_sequence`.
- La base raw no debe borrar datos por fusión ni aplicar correcciones globales.
- En `3G`, un full snapshot no debe borrar ausentes hasta que una subfase posterior defina una política segura para `bad`/ausentes.
- Las fases posteriores deben usar `GlobalPoseStore` para poses globales, no mutar las poses locales raw guardadas aquí.
- Las consultas `GetKeyFrame` y `GetMapPoint` devuelven punteros constantes; ningún consumidor debe usarlas para modificar estado raw.
- La lectura BoW de `3N` es de solo lectura; no convierte `RawMapDatabase` en
  detector, índice global ni fuente de loops confirmados.
- La lectura de subnubes de `3O` también es de solo lectura: no puede borrar,
  fusionar, reponderar ni transformar los datos raw guardados.
- El journal fiducial no convierte a `RawMapDatabase` en fuente de poses globales; solo conserva evidencia para que `FiducialAnchorManager` pueda reproducir el anclaje.
- En `3K`, cualquier apply que altere contadores o poses raw debe considerarse fallo. La evidencia esperada es `[F1K-RAWDB-NOT-MODIFIED] ... ok=true`.

## Validación de revisiones 3P

El build del 2026-08-04 y `test_covisibility_database` validan que metadata de
un MP no cambia la revisión geométrica y que una posición modificada invalida
solo su KF observador. En `prueba_62`, `81` llegadas solo metadata crean cero
trabajo de loop y no reaparece la invalidación global de `prueba_61`.

## Validación 3K

El 2026-07-10 se verificó:

- `prueba_3.log`: `5` applies con `[F1K-RAWDB-NOT-MODIFIED] ... ok=true`;
- `prueba_2.log`: `3` applies con `[F1K-RAWDB-NOT-MODIFIED] ... ok=true`;
- no aparecieron `RAWDB-POSE-OVERWRITE-BY-OPT` ni `raw_db_modified=true`.
