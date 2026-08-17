# `GlobalPoseStore`

## Rol

`GlobalPoseStore` es la capa ligera creada en `3D` dentro de `orbslam3_multi`.

Su responsabilidad es responder:

```text
¿Que pose global tiene este KeyFrame segun el estado global actual?
```

No guarda datos ORB grandes. No guarda descriptores, BoW, keypoints, observaciones ni MapPoints completos. Cuando necesita una pose local o un listado de KFs consulta `RawMapDatabase`.

Desde `3E`, también conserva qué KeyFrames fueron marcados como fiduciales reales/hard fiducials por `FiducialAnchorManager`. Esa marca sirve para que optimizaciones futuras no traten esos KFs como variables normales.

Desde el hotfix de `3F`, también conserva la extrínseca fija `body_T_camera` usada para convertir observaciones fiduciales simuladas desde la pose world del cuerpo del dron a la pose world de la cámara/KF:

```text
world_T_camera = world_T_body * body_T_camera
```

Esto evita anclar submapas usando directamente el frame del cuerpo como si fuera el frame de cámara.

Desde `3G`, también puede reconciliar cambios raw procedentes de full snapshots
y, desde la revisión de `3K` del 2026-07-24, de deltas que modifiquen poses.
Conserva las poses aceptadas por el servidor y reproyecta solo la cola derivada.

Desde `3J`, `GlobalPoseStore` actua como entrada de solo lectura para `OptimizationManager::RunDryRun`. El dry-run copia poses a memoria temporal y el servidor comprueba por estadisticas que `GlobalPoseStore` no cambia durante la prueba.

Desde `3K`, `GlobalPoseStore` es la unica capa persistente donde el servidor aplica poses optimizadas o propagadas. `RawMapDatabase` conserva la pose local raw; `GlobalPoseStore` conserva la decision global del servidor.

Desde `3O`, `SubcloudLoopVerifier` consulta poses globales de KFs para construir
subnubes en frame `world` y calcular error geométrico entre candidato y query.
Esa ruta es estrictamente de solo lectura: `3O` no registra poses optimizadas,
no aplica propagación, no crea backups y no modifica correcciones.

La protección transaccional de apply se desarrolló durante `3L`, pero pertenece
conceptualmente a `3K`: `GlobalPoseStore` guarda el estado global aplicado por
el servidor y puede restaurar backups si una validación post-apply rechaza una
optimización. `3L` solo diagnostica/valida ese estado.

Actualización de diseño tras las pruebas fiduciales largas: el rebase rígido de
cola con `T_last_after * inverse(T_last_before)` resolvió una parte del problema,
pero no es suficiente cuando ORB-SLAM3 cambia poses raw ya aceptadas por el
servidor. La política vigente implementada es que las poses world aceptadas en
`GlobalPoseStore` son autoritativas e inamovibles salvo nueva optimización o
rollback del servidor. Los KFs posteriores se deben extender desde el último KF
global aceptado anterior:

```text
T_world_new = T_world_ref_accepted * inverse(T_raw_ref_current) * T_raw_new_current
```

`submap_last_correction` queda como compatibilidad temporal/deuda técnica: no
debe ser la fuente de autoridad para KFs futuros en rutas fiduciales largas.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp
orbslam3_multi/src/global_pose_store.cpp
```

## Estado inicial

`GlobalPoseStore` empieza vacío aunque `RawMapDatabase` ya tenga KFs y MPs.

Esto es intencionado: sin un anchor de submapa no existe una transformación válida:

```text
world_T_local
```

Por tanto no debe inventar:

```text
world_T_kf = world_T_local * local_T_kf
```

En `3D` solo se pobla mediante un modo debug sintético de replay. El primer anclaje real se espera en `3E` mediante `FiducialAnchorManager`.

## Responsabilidades

- Guardar anchors de submapa por `(drone_id, map_epoch)`.
- Crear poses `world_T_kf` para KFs existentes al anclar un submapa.
- Consultar si un submapa tiene anchor.
- Consultar si un KF tiene pose world.
- Registrar KFs nuevos de submapas ya anclados.
- Registrar poses optimizadas ya calculadas por una prueba o fase futura.
- Registrar poses propagadas tras optimización del servidor.
- Guardar una lista ordenada de KFs con pose world aceptada por el servidor
  (`accepted_keyframe_anchors`) y el `active_tail_anchor` de cada submapa.
- Mantener `derived_tail_keyframes` separado: tienen pose world publicable, pero
  pueden reproyectarse si cambia la relación raw con su ancla aceptada.
- Desde `3K`, tras un apply validable, registrar el target/último KF aplicado
  como nuevo `active_tail_anchor` si la optimización fue aceptada. Si el error
  fiducial era bajo y no hubo optimización, no se cambia ese anchor activo.
- Si un KF posterior ya está en el full snapshot o cambia su pose raw tras el
  apply, `ReconcileAfterRawIngestResult` debe conservar las poses world
  aceptadas y recalcular solo KFs derivados de cola usando el último anchor
  aceptado anterior y la relación raw actual.
- Los KFs posteriores ya existentes al cerrar un apply los recorre
  `OptimizationManager` en el mismo submapa. Si alguno está en `RawMapDatabase`
  pero todavía no tiene pose world, se registra primero con
  `RegisterNewKeyFrameIfAnchored` usando `active_tail_anchor`; después queda
  registrado como `derived_tail`, no como pose aceptada.
- Marcar y consultar KFs hard fiducial.
- Guardar y aplicar `body_T_camera` para convertir poses de cuerpo en poses de cámara antes de derivar anchors fiduciales.
- Reconciliar cambios de pose local raw detectados en full snapshots sin ceder la autoridad sobre poses globales optimizadas.
- Crear backups de apply para `3K` y restaurarlos/confirmarlos cuando `3L`
  decide rollback o commit.
- Exponer estadísticas ligeras para logs.

## Métodos principales

| Método | Responsabilidad |
|---|---|
| `AnchorSubmap` | Recibe `world_T_local`, consulta KFs en `RawMapDatabase` y crea poses `world_T_kf` para los KFs existentes. |
| `HasSubmapAnchor` | Indica si un submapa ya tiene anchor. |
| `HasWorldPose` | Indica si un KF ya tiene pose world registrada. |
| `GetWorldPose` | Devuelve la pose world existente; no inventa poses para KFs sin anchor. |
| `GetSubmapWorldTransform` | Devuelve `world_T_local` de un submapa anclado para consumidores de solo lectura como `GlobalMapBuilder`. |
| `RegisterNewKeyFrameIfAnchored` | Registra un KF nuevo desde `active_tail_anchor` y lo clasifica como `derived_tail`; si no hay ancla activa usa el último KF aceptado compatible. |
| `SetOptimizedKeyFramePose` | Registra una pose optimizada ya calculada y su corrección por KF. |
| `SetPropagatedKeyFramePose` | Registra una pose propagada tras optimización y su corrección por KF. |
| `SetDerivedTailKeyFramePose` | Registra/referencia un KF posterior como cola recalculable desde un KF aceptado. |
| `SetActiveTailAnchor` | Convierte la referencia aplicada en autoridad aceptada para KFs posteriores. |
| `GetActiveTailAnchor` | Consulta referencia, pose aceptada, tarea y fuente del ancla activa. |
| `ProjectWorldPoseFromReference` | Aplica la relación relativa raw actual entre referencia aceptada y KF derivado. |
| `GetKeyFrameServerCorrection` | Devuelve la corrección del servidor asociada a un KF. Desde `3K`, `GlobalMapBuilder` usa esta consulta como flag de cobertura por KF para decidir si un MapPoint debe proyectarse desde una pose corregida. |
| `GetSubmapLastServerCorrection` | Legacy/compatibilidad temporal. No debe usarse como veto global para MapPoints ni como autoridad de KFs futuros en trayectorias fiduciales largas. |
| `SetSubmapLastServerCorrectionFromKeyFrame` | Legacy/compatibilidad temporal. Debe sustituirse por registro explícito de `active_tail_anchor`. |
| `CreateApplyBackup` | Guarda poses, anclas aceptadas/activas y pertenencia `derived_tail` antes del apply de `3K`. |
| `RestoreApplyBackup` | Restaura el backup de un `task_id` tras `REJECT_ROLLBACK`. |
| `ConfirmApply` | Descarta el backup de un `task_id` tras `ACCEPT` o `PARTIAL_KEEP_FOR_NEXT_PASS`. |
| `MarkHardFiducialKeyFrame` | Marca un KF como fiducial real/hard fiducial tras una observación aceptada. |
| `IsHardFiducialKeyFrame` | Consulta si un KF está protegido como hard fiducial. |
| `ConfigureBodyCameraTransform` | Configura `body_T_camera` desde parámetros del servidor/launch. |
| `GetBodyCameraTransform` | Devuelve la matriz `body_T_camera` vigente para debug y logs. |
| `TransformBodyPoseToCameraPose` | Calcula `world_T_camera = world_T_body * body_T_camera`. |
| `GetBodyCameraTransformConfig` | Devuelve la configuración de extrínseca activa. |
| `ReconcileAfterRawIngestResult` | Conserva KFs `accepted`, actualiza su corrección raw y reproyecta solo `derived_tail` desde su referencia aceptada. |
| `GetPoseStoreStats` | Devuelve también contadores `accepted_keyframe_anchors`, `active_tail_anchors` y `derived_tail_keyframes`. |
| `GetSubmapPoseStats` | Resume estado global de un submapa concreto. |

## Extrínseca `body_T_camera`

La configuración activa por defecto replica los valores del launch legacy:

```text
body_T_camera_x=0.10
body_T_camera_y=0.03
body_T_camera_z=0.03
body_T_camera_roll_deg=0.0
body_T_camera_pitch_deg=-90.0
body_T_camera_yaw_deg=90.0
use_camera_optical_frame_convention=true
```

Con `use_camera_optical_frame_convention=true`, la rotación usa la convención óptica que ya estaba en `global_pose_corrector` legacy. Los valores RPY quedan documentados y parametrizados, pero la convención óptica es la que se aplica en la validación del hotfix `3F`.

## Política obsoleta y política vigente

La política de 3D para KFs futuros tras optimización es deliberadamente simple:

```text
world_T_kf = correction_T_latest * (world_T_local * local_T_kf)
```

La corrección heredada solo se aplica dentro del mismo submapa. Esa política fue
útil para validar la separación `RawMapDatabase`/`GlobalPoseStore`, pero queda
obsoleta para optimización fiducial larga porque ORB-SLAM3 puede cambiar las
poses raw de KFs ya aceptados.

La política vigente implementada es:

```text
T_world_new = T_world_ref_accepted * inverse(T_raw_ref_current) * T_raw_new_current
```

`ref` es el último KF anterior con pose world aceptada por el servidor. Para una
optimización fiducial aceptada, el `active_tail_anchor` pasa al target o al
último KF aplicado de la ventana. Si no hubo optimización porque el error fue
bajo, no cambia el anchor activo. No se edita ORB-SLAM3 por ahora; sus poses raw
son entrada local mutable, no autoridad world.

## Logs relacionados

Los logs los emite `orbslam3_server/src/global_map_server.cpp` al activar el modo debug de `3D`:

```text
[F1D-POSESTORE-INIT]
[F1D-POSESTORE-ANCHOR-REQUEST]
[F1D-POSESTORE-ANCHOR-SET]
[F1D-POSESTORE-ANCHOR-KF-POSE]
[F1D-POSESTORE-ANCHOR-SUMMARY]
[F1D-POSESTORE-OPT-POSE-SET]
[F1D-POSESTORE-CORRECTION-SET]
[F1D-POSESTORE-CORRECTION-STATS]
[F1D-POSESTORE-NEW-KF-POSE-SET]
[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT]
[F1D-POSESTORE-STATS]
[F1E-FID-KF-HARD]
[F1E-FID-STATS]
[F1F-BODY-CAMERA-CONFIG]
[F1F-BODY-CAMERA-APPLY]
[F1G-POSESTORE-REBASE-ANCHOR]
[F1G-POSESTORE-KEEP-OPTIMIZED]
[F1G-SNAPSHOT-AFFECTS-OPTIMIZED-KFS]
[F1G-POSESTORE-RECONCILE-SUMMARY]
[F1K-POSESTORE-OPTIMIZED-POSE-SET]
[F1K-POSESTORE-PROPAGATED-POSE-SET]
[F1K-POSESTORE-CORRECTION-SET]
[F1K-POSESTORE-LAST-CORRECTION-SET]
[F1K-POSESTORE-NEW-KF-INHERIT-CORRECTION]
[F1K-POSESTORE-KEEP-SERVER-OPTIMIZED]
[F1K-POSESTORE-KEEP-FUTURE-INHERIT]
[F1K-POSESTORE-RECOMPUTE-CORRECTION-AFTER-RAW-CHANGE]
[F1L-POSESTORE-BACKUP-CREATED]
[F1L-POSESTORE-COMMIT-CONFIRMED]
[F1L-POSESTORE-ROLLBACK]
```

## Validación 3D

El 2026-07-08 se validó con replay sin Gazebo de:

```text
codex/archivos_auxiliares/repeticiones/rawdb_prueba_1.record
```

Evidencia principal:

- `BUILD-EXIT-CODE 0` en `orbslam3_multi`;
- `BUILD-EXIT-CODE 0` en `orbslam3_server`;
- `SIM-DONE prueba=1 success=true`;
- `[F1C-REPLAY-DONE] entries=284 journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884`;
- `[F1D-POSESTORE-INIT] anchors=0 world_poses=0 optimized_kfs=0 corrections=0`;
- `[F1D-POSESTORE-ANCHOR-SUMMARY] ... anchored_kfs=1`;
- `[F1D-POSESTORE-OPT-POSE-SET]`;
- `[F1D-POSESTORE-CORRECTION-SET]`;
- `4` eventos `[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT]`;
- sin `ERROR`, `FATAL`, `Segmentation fault`, `Killed`, `std::bad_alloc` ni `No space left` durante la prueba.

## Validación 3E

El 2026-07-08 se validó el uso real con fiducial simulado:

- Gazebo live `prueba_1` generó observaciones fiduciales asociadas a KFs;
- replay `prueba_2` cargó `fiducial_observations=60` desde `.record` versión 2;
- `FiducialAnchorManager` aceptó `60` observaciones y creó `2` anchors;
- `GlobalPoseStore` terminó con `hard_fiducial_kfs=60` en `[F1E-FID-STATS]`;
- no se usó el modo debug `pose_store_debug_*` para validar `3E`.

## Validación hotfix 3F `body_T_camera`

El 2026-07-08 se corrigió la ruta de anclaje usada por `/global_sparse_cloud`:

- el `.record` conserva la pose world del cuerpo del dron;
- `FiducialAnchorManager` pide a `GlobalPoseStore` transformar esa pose a cámara;
- `world_T_local` se calcula desde `world_T_camera`, no desde `world_T_body`;
- el replay lento publicó `22394` puntos en `/global_sparse_cloud` tras aplicar la extrínseca;
- el dataset diferencial se conservó temporalmente como `codex/archivos_auxiliares/repeticiones/prueba_diff_anclaje.record`; al iniciar `3G` se borró a petición del usuario para liberar espacio y sustituirlo por un record mejor con full snapshots.

## Validación 3G

El 2026-07-08 se validó reconciliación tras full snapshots:

- live `prueba_1` terminó con `SIM-DONE prueba=1 success=true`;
- `8` eventos `[F1G-POSESTORE-RECONCILE-SUMMARY]`;
- `48` eventos `[F1G-POSESTORE-REBASE-ANCHOR]`, demostrando recalculo de poses derivadas de anchor;
- `1` evento `[F1G-POSESTORE-KEEP-OPTIMIZED]`, demostrando que una pose optimizada debug se conserva ante cambio raw;
- replay `prueba_2` terminó con `SIM-DONE prueba=2 success=true`;
- replay reprodujo `12` full snapshots y volvió a emitir `12` resúmenes de reconciliación;
- no hubo `failed` en los resúmenes relevantes de reconciliación observados.

## Restricciones

- No modifica `RawMapDatabase`.
- No usa ground truth.
- No detecta fiduciales.
- No ejecuta optimización real.
- No publica nube global.
- No convierte anchors debug de `3D` en estado real de mapa.
- En `3E`, las marcas hard fiducial sí representan observaciones fiduciales reales simuladas; no deben moverse en optimizaciones futuras salvo fase explícita.
- En `3F`, `GetSubmapWorldTransform` permite publicar puntos en `world`, pero no autoriza a modificar anchors ni poses desde `GlobalMapBuilder`.
- `body_T_camera` solo corrige la relación cuerpo/cámara de observaciones fiduciales; no debe usarse como sustituto de optimización, fusión ni corrección de deriva entre submapas.
- En `3G`, un full snapshot no es una optimización: si cambia `local_T_kf`, los KFs anclados se recalculan y los KFs optimizados mantienen `optimized_world_T_kf`.
- En `3J`, `OptimizationManager` no debe llamar a `SetOptimizedKeyFramePose`; esa frontera queda para `3K`.
- En `3K`, `SetOptimizedKeyFramePose` y `SetPropagatedKeyFramePose` solo representan autoridad del servidor. No deben escribir en `RawMapDatabase`.
- En `3K`, un backup debe existir antes de aplicar un candidato que vaya a validarse post-apply.
- En `3K`, rollback restaura solo estado de `GlobalPoseStore`; no debe reescribir poses raw ni entries del journal.
- En `3K`, `GetSubmapLastServerCorrection` queda como dato legacy y no gobierna
  KFs futuros ni toda la nube. Los KFs nuevos/derivados se extienden desde
  `active_tail_anchor`; la publicación de MapPoints decide por
  `GetKeyFrameServerCorrection` del KF de referencia/observador.

## Validación 3K

El 2026-07-10 se validó:

- `prueba_3.log`: `17` `[F1K-POSESTORE-OPTIMIZED-POSE-SET]` y `40` `[F1K-POSESTORE-PROPAGATED-POSE-SET]`;
- `prueba_2.log`: `23` `[F1K-POSESTORE-OPTIMIZED-POSE-SET]` y `54` `[F1K-POSESTORE-PROPAGATED-POSE-SET]`;
- `prueba_3.log`: `309` `[F1K-POSESTORE-NEW-KF-INHERIT-CORRECTION]`;
- todos los applies terminaron con `[F1K-RAWDB-NOT-MODIFIED] ... ok=true`;
- no se observaron hard fiducials movidos.

El 2026-07-23 se revalidó la semántica de publicación por cobertura de KFs
corregidos:

- build `orbslam3_multi orbslam3_server`: `BUILD-EXIT-CODE 0`;
- live `prueba_31` con la prueba típica corta fiducial 2 -> 1:
  `SCENARIO-RUNNER-DONE success=true`, `SIM-DONE success=true`,
  `SIM-EXIT-CODE 0`;
- `task_id=1`: `before_t=21.289176 -> after_t=0`, `optimized_kfs=26`,
  `propagated_kfs=69`, `raw_db_modified=false`;
- `task_id=2`: `before_t=0.444713 -> after_t=0`, `optimized_kfs=27`,
  `propagated_kfs=73`, `raw_db_modified=false`;
- en ambos applies `invalid_pose_skipped` queda `0 -> 0` y
  `server_corrected_missing_keyframe_skipped=0`.

El 2026-07-28 se cerró la revalidación de autoridad con `prueba_41`:

- dos applies live en un único `map_epoch`;
- las poses world `accepted` y `server_optimized` se conservan frente a
  cambios raw locales de ORB-SLAM3;
- `derived_tail` se reproyecta desde `active_tail_anchor`, con cambio world
  máximo observado de `0.089053 m`;
- ambos applies registran `raw_db_modified=false`;
- el usuario confirma en RViz2 que ambas ventanas, la cola posterior y los
  MapPoints se ven perfectamente y no regresan a poses raw antiguas;
- la autoridad y aplicación de `3K` quedan `CONSEGUIDAS`.

## Validación 3L

El 2026-07-10 se validó backup/commit/rollback:

- `prueba_1`: `8` backups y validaciones, `7` commits `ACCEPT` y `1` commit `PARTIAL_KEEP_FOR_NEXT_PASS`;
- `prueba_2`: `8` backups y validaciones, `4` commits `ACCEPT`, `1` commit `PARTIAL_KEEP_FOR_NEXT_PASS` y `3` rollbacks;
- `prueba_3`: `4` backups y validaciones, `3` commits `ACCEPT` y `1` commit `PARTIAL_KEEP_FOR_NEXT_PASS`;
- el rollback forzado de `prueba_2` mostro:
  ```text
  [F1L-POSESTORE-ROLLBACK] ok=true restored_world=80 removed_world=0 restored_optimized=0 removed_optimized=80 restored_submap_corrections=0 removed_submap_corrections=1
  ```
- no aparecieron `ROLLBACK_FAILED`, `RAWDB-POSE-OVERWRITE-BY-OPT` ni hard fiducials aceptados como movidos.

La validación final de `3L` corresponde a `prueba_41`: los marcadores técnicos,
los HTML y la inspección RViz2 del usuario son coherentes. `3L` queda
`CONSEGUIDA` el 2026-07-28.
