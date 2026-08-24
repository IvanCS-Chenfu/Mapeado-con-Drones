# Subfase 4H — Primer anchor visual del submapa sin Ground Truth

<!-- ACUERDOS_CIERRE_F2_2026_08_24_START -->
## Transición del feed GT al anchor visual en system_architecture

> **Vigencia:** acuerdo cerrado el 2026-08-24. Este bloque prevalece sobre cualquier
> frase anterior incompatible del mismo documento. No borra ni reescribe evidencia
> histórica; distingue siempre entre estado actual, deuda conocida y arquitectura objetivo.

Al demostrar el primer anchor visual, `system_architecture` debe dejar de presentar
GT→Servidor como camino funcional de fiducial. La ruta visual wrapper→Servidor pasa a
ser la vigente. GT puede seguir apareciendo únicamente como métrica/debug externo si
realmente existe. No borrar prematuramente código legacy antes de la validación prevista
en 4K.
<!-- ACUERDOS_CIERRE_F2_2026_08_24_END -->

## Estado

```text
sin hacer
```

## Dependencia

`4G`.

## Objetivo técnico

Sustituir la entrada GT del anchor fiducial por una restricción absoluta de cámara derivada exclusivamente de la observación visual de tag y de la pose global conocida del cubo. El KF que vio el tag debe ser exactamente el KF usado para construir `world_T_local`.

La lógica existente de autoridad de `GlobalPoseStore`, hard fiducials y raw map debe conservarse.

## Comportamiento esperado

Para una observación válida del KF:

```text
world_T_camera_target  <- servidor, desde tag/cubo
local_T_camera_kf      <- RawMapDatabase / OrbKeyFrame
world_T_local          = world_T_camera_target * inverse(local_T_camera_kf)
```

Si el submapa no tiene anchor, se crea el primer anchor. Si ya está anclado, la observación se deriva a la lógica de revisit de 4I.

## Contexto obligatorio a leer


```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4.md
```


Además:

```text
subfases/subfase_4G.md
src/orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp
src/orbslam3_multi/src/fiducial_anchor_manager.cpp
src/orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp
src/orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp
src/orbslam3_server/src/global_map_server.cpp
```

## Diagnóstico de partida

`orbslam3_multi::FiducialObservation` actual está moldeado por la simulación GT:

```text
world_T_body_fiducial
gt_stamp_sec
association_dt_sec
distance_to_fiducial_m
source
```

`FiducialAnchorManager` convierte después la pose body-GT a cámara usando extrínsecos. En la ruta visual, el servidor ya puede obtener directamente `world_T_camera`; volver a aplicar `body_T_camera` sería incorrecto.

## Archivos permitidos a modificar

```text
src/orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp
src/orbslam3_multi/src/fiducial_anchor_manager.cpp
src/orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp
src/orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp
src/orbslam3_multi/src/raw_map_database.cpp
src/orbslam3_server/src/global_map_server.cpp
src/orbslam3_server/include/orbslam3_server/global_map_server.hpp
codex/contexto/paquetes/orbslam3_multi/
codex/contexto/paquetes/orbslam3_server/
```

## Archivos prohibidos

```text
ORB_SLAM3/
ORB_SLAM3_ROS2/
src/simulacion_dron/src/plugins/plugin_sensor_groundtrurh.cpp
legacy orbslam3_multi/
```

## Funciones, clases o nodos que hay que localizar

```text
orbslam3_multi::FiducialObservation
FiducialAnchorManager::RegisterFiducialObservation
FiducialAnchorResult
GlobalPoseStore anchor API
RawMapDatabase KF lookup
HandleFiducialObservation
ToRecordedObservation / raw journal equivalente
```

## Cambios requeridos

1. Rediseñar o añadir una observación backend normalizada que reciba un objetivo absoluto de cámara y no una pose GT de body.
2. Campos funcionales mínimos: `arrival_id`, `drone_id`, `map_epoch`, `local_keyframe_id`, `global_keyframe_id` si aplica, `object_id`, `tag_id` representativo, `world_T_camera_target`, `keyframe_stamp_sec`, calidad/error y `source=visual_fiducial`.
3. Eliminar de la ruta visual la necesidad de `gt_stamp_sec`, `association_dt_sec` y `world_T_body_fiducial`.
4. Dentro de `RegisterFiducialObservation`, obtener la pose local cruda del KF exacto y calcular `world_T_local` directamente desde `world_T_camera_target`.
5. No aplicar `body_T_camera` a una observación que ya está en cámara.
6. Conservar la lógica de primer anchor, hard fiducial y autoridad de `GlobalPoseStore` salvo cambios mínimos necesarios por el nuevo tipo.
7. Si el mismo KF proporciona varias caras del mismo cubo, 4G debe haber elegido/normalizado un candidato lógico antes de llegar al manager.
8. Si el mismo KF proporciona varios cubos coherentes, usar un candidato canónico para crear el primer anchor y utilizar los demás como comprobaciones del mismo `world_T_camera`; no crear varios anchors contradictorios.
9. Registrar qué objeto/tag creó el anchor y qué observaciones adicionales lo validaron.
10. Mantener journaling raw/auditable sin insertar pose optimizada dentro de `RawMapDatabase`.
11. Añadir logs `FID-VIS-ANCHOR-CANDIDATE`, `FID-VIS-ANCHOR-COMMIT`, `FID-VIS-ANCHOR-REJECT`.
12. Desactivar la creación GT de anchor durante las pruebas de Fase 4 mediante parámetro/configuración explícita, sin borrar todavía el legacy si se necesita para regresión.

## Cambios prohibidos

- No consultar GT para completar campos faltantes.
- No aplicar extrínseco body-camera dos veces.
- No modificar pose local ORB cruda en `RawMapDatabase`.
- No tratar el anchor como loop closure.
- No marcar un KF como hard fiducial si la observación fue rechazada.
- No aceptar dos anchors incompatibles del mismo KF.
- No borrar rollback/validaciones existentes de optimización.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server orbslam3_msgs
```

Añadir wrapper solo si el contrato ROS de 4E cambia durante la ejecución; ese cambio debe estar justificado.

## Pruebas Gazebo requeridas

### Prueba 1 — Primer anchor con un tag

Arrancar un submapa sin anchor y hacer que un KF observe un tag válido. Debe aparecer un commit de anchor cuyo origen sea `visual_fiducial`, sin marcador GT de creación funcional.

### Prueba 2 — Dos caras del mismo cubo en KF de anchor

El KF observa dos tags del mismo cubo. Debe crearse un solo anchor y registrarse la coherencia/selección del candidato.

### Prueba 3 — Dos cubos en KF de anchor

El KF observa al menos un tag de dos cubos diferentes y coherentes. Debe existir un solo `world_T_local`; ambas restricciones deben ser compatibles dentro del umbral acordado.

### Prueba 4 — GT desconectado de la lógica

Deshabilitar `fiducial_sim_enabled`/ruta equivalente y confirmar que el anchor visual sigue ocurriendo. GT puede seguir publicándose únicamente para métricas externas.

## Patrones de reducción de logs

```text
FID-VIS-ANCHOR|FID-SYNC|FID-OBJECT|GlobalPoseStore|hard_fiducial|world_T_local|visual_fiducial|F1E-FID-CANDIDATE-GT|ERROR|FATAL|Segmentation fault|Killed
```

La presencia de logs GT de sensor no es fallo por sí misma; sí lo es que una ruta GT sea la fuente del anchor.

## Criterio de éxito

1. `orbslam3_multi` y servidor compilan.
2. Un KF visual crea `world_T_local` sin pose GT de dron.
3. El KF usado es el mismo identificado por 4C–4F.
4. `GlobalPoseStore` recibe el anchor y `RawMapDatabase` permanece crudo.
5. No se aplica body-camera a una pose ya expresada en cámara.
6. Varias observaciones del mismo KF no crean anchors incompatibles.
7. El anchor funciona con la ruta GT funcional deshabilitada.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: anchor depende de GT, usa otro KF, invierte transformaciones o corrompe raw poses.
- `PARCIAL`: anchor visual funciona con un tag pero falla multi-cara/multi-cubo o hard-fiducial.
- `BLOQUEADA`: convención local_T_camera del wrapper no puede verificarse con la información disponible.

## Riesgos

- `Tcw` vs `Twc`;
- `camera` vs `camera_optical_frame`;
- double extrinsic body-camera;
- seleccionar un candidato de cubo incorrecto;
- mezclar primera visita y revisit en el mismo KF.

## Documentación a actualizar

```text
codex/contexto/paquetes/orbslam3_multi/
codex/contexto/paquetes/orbslam3_server/
codex/contexto/01_ESTADO_ACTUAL.md si cambia el estado real
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4H.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4H_RESUMEN.md
```
