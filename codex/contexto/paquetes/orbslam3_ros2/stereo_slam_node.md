# Script `StereoSlamNode` del wrapper ORB-SLAM3 estéreo

## Rol

`StereoSlamNode` es el nodo que conecta cámaras estéreo ROS 2 con ORB-SLAM3. Además de ejecutar tracking local, exporta al servidor global la información de mapa necesaria para construir un mapa sparse multi-dron.

## Flujo interno

```text
camera/left + camera/right
  ↓ sincronización ApproximateTime
GrabStereo
  ↓ cv_bridge + rectificación opcional
ORB_SLAM3::System::TrackStereo
  ↓
pose local Twc + mapa ORB interno
  ↓
PublishLocalPose / PublishOrbMapDelta / get_full_map
```

## Constructor `StereoSlamNode(...)`

Responsabilidades:

1. Declara parámetros ROS:
   - `drone_id`;
   - `drone_name`;
   - `local_map_frame`;
   - `delta_publish_period_frames`.

2. Carga parámetros de cámara desde el YAML ORB-SLAM3:
   - `Camera.fx`, `Camera.fy`, `Camera.cx`, `Camera.cy`, `Camera.bf`;
   - `Camera.width`, `Camera.height`;
   - fallback a `LEFT.K`, `LEFT.width`, `LEFT.height` si hace falta.

3. Crea publishers:
   - `orbslam/orb_map_delta`;
   - `orbslam/pose_local`.

4. Crea servicio:
   - `orbslam/get_full_map`.

5. Configura rectificación si `rectify=true`.

6. Crea subscribers sincronizados:
   - `camera/left`;
   - `camera/right`.

## `GrabStereo(msgLeft, msgRight)`

Callback principal.

Hace:

1. Convierte imágenes ROS a `cv::Mat` con `cv_bridge`.
2. Rectifica si `doRectify` está activo.
3. Ejecuta:

```cpp
m_SLAM->TrackStereo(...)
```

4. Actualiza epoch/mapa activo con `UpdateMapEpochFromCurrentMap()`.
5. Si tracking está OK, publica pose local con `PublishLocalPose()`.
6. Incrementa `frame_counter_`.
7. Publica delta cada `delta_publish_period_frames_`.
8. Si detecta nuevo epoch, fuerza publicación inmediata.

## `PublishLocalPose(stamp, Tcw)`

Convierte `Tcw` a `Twc` y publica `geometry_msgs/PoseStamped` en `orbslam/pose_local`.

Convención:
- `Tcw`: cámara respecto al mundo local ORB.
- `Twc`: pose de cámara en mapa local ORB.

El `frame_id` publicado es `local_map_frame_`, normalmente algo como `dron_1_orb_map`.

## `PublishOrbMapDelta()`

Construye un `OrbMap` incremental con:

```cpp
BuildOrbMap(false, true)
```

Si no hay cambios en KFs ni MPs, no publica. Si hay cambios, publica en
`orbslam/orb_map_delta`. El delta no representa una secuencia cronologica de
KFs exclusivamente nuevos: tambien vuelve a incluir KFs existentes cuando
cambia su hash por pose, asociaciones u otro estado exportado. Por ello un KF
con ID menor puede aparecer en un delta posterior a otro con ID mayor.

Un dron quieto también puede producir deltas no vacíos. `HashMapPoint()` incluye
estadísticas mutables de tracking como `found_ratio`, observaciones y normal;
ORB-SLAM3 puede actualizarlas al seguir viendo el mismo mapa aunque no cree KFs
ni MPs. El intento periódico sin entidades se descarta antes del publisher, por
lo que un flujo wrapper-servidor visible siempre contiene al menos una entidad,
pero no implica geometría nueva. Esta distinción debe conservarse al extender
el flujo principal: cambios estadísticos raw no deben activar automáticamente
todos los consumidores derivados.

## `GetFullMapServiceCallback(...)`

Devuelve un snapshot completo:

```cpp
BuildOrbMap(true, true)
```

Uso principal:
- el servidor puede recuperar el mapa completo si perdió deltas o si entra un epoch nuevo.
- la reconciliacion puede incorporar en bloque KFs antiguos que no llegaron en
  deltas previos; el consumidor no debe interpretar el orden de llegada como
  orden temporal completo.

## `BuildOrbMap(full_snapshot, update_cache)`

Función central de exportación.

Hace:

1. Asegura que el epoch/mapa activo está actualizado.
2. Crea `orbslam3_msgs/msg/OrbMap`.
3. Rellena metadatos:
   - `drone_id`;
   - `drone_name`;
   - `map_frame`;
   - `map_sequence`;
   - `map_epoch`;
   - cámara estéreo.
4. Recorre `m_SLAM->GetAllMapPoints()`.
5. Filtra puntos que no pertenecen al mapa activo.
6. Para snapshot completo envía todos los puntos válidos.
7. Para delta envía solo puntos nuevos/cambiados o marcados bad.
8. Recorre `m_SLAM->GetAllKeyFrames()` con la misma lógica.
9. Actualiza caches de hashes si `update_cache=true`.

## `FillMapPointMsg(pMP, mp_msg)`

Exporta un `ORB_SLAM3::MapPoint` a `OrbMapPoint`.

Campos rellenados:
- ID local;
- posición local ORB;
- descriptor representativo;
- `is_bad=false`;
- número de observaciones;
- `found_ratio`;
- normal;
- distancias invariantes;
- KF de referencia;
- lista de observaciones KF-feature.

Importancia:
- El servidor usa estos datos para fused landmarks, subcloud matching, score y covisibilidad.

## `FillKeyFrameMsg(pKF, kf_msg)`

Exporta un `ORB_SLAM3::KeyFrame` a `OrbKeyFrame`.

Campos rellenados:
- ID local;
- pose `Twc`;
- timestamp;
- keypoints;
- descriptores;
- `u_right`;
- depth;
- IDs de MapPoints asociados por feature;
- covisibilidad;
- BoW vector;
- FeatureVector;
- parent/children;
- loop edges locales.

Importancia:
- Es la base para `GlobalKeyFrameDatabase`, `GlobalORBMatcher`, loops y optimización global/local.

## `HashMapPoint(pMP)`

Calcula un hash de estado del MapPoint.

Incluye:
- ID;
- posición cuantizada;
- descriptor;
- observaciones;
- found ratio;
- normal;
- distancias invariantes;
- KF de referencia;
- lista de observaciones.

Uso:
- decidir si el MapPoint debe publicarse en un delta.

## `HashKeyFrame(pKF)`

Calcula un hash de estado del KeyFrame.

Incluye:
- ID;
- pose cuantizada;
- asociaciones a MapPoints;
- covisibilidad;
- BoW;
- FeatureVector;
- stereo info;
- parent/children;
- loop edges locales.

Uso:
- decidir si el KeyFrame cambió suficientemente para ser reenviado.

## `UpdateMapEpochFromCurrentMap()`

Detecta si ORB-SLAM3 cambió de mapa/submapa.

Mecanismos:
- compara puntero del mapa activo;
- calcula firma por keyframes: número, mínimo ID, máximo ID;
- detecta resets donde el puntero no cambia pero el primer KF salta.

Si detecta cambio:
- incrementa `map_epoch_`;
- limpia caches de hashes;
- resetea `map_sequence_`;
- fuerza publicación inmediata desde `GrabStereo()`.

## `GetCurrentMapPointerFromKeyFrames()`

Busca el KF válido más reciente y devuelve su `GetMap()`.

Uso:
- inferir mapa activo cuando no hay API directa suficiente.

## `KeyFrameBelongsToMap(...)` y `MapPointBelongsToMap(...)`

Filtran KFs/MPs para no mezclar datos de mapas internos distintos de ORB-SLAM3.

Esto es fundamental para no contaminar un epoch con datos de otro.

## `LoadCameraInfoFromSettings(...)`

Carga intrínsecos y baseline desde YAML.

Primero intenta formato ORB-SLAM3:

```text
Camera.fx
Camera.fy
Camera.cx
Camera.cy
Camera.bf
Camera.width
Camera.height
```

Luego fallback a `LEFT.K`, `LEFT.width`, `LEFT.height`.

## Riesgos al modificar

- No romper identidad `(drone_id, map_epoch, local_id)`.
- No cambiar convención de pose sin actualizar servidor y corrector.
- No quitar BoW/FeatureVector: son necesarios para matching global.
- No publicar `MapPoints` de mapas inactivos.
- No usar score global aquí.

## Perfil de recursos 3G

El wrapper no cambia su contrato de deltas/snapshots, pero el launch multi-dron
lo alimenta con un perfil medido:

```text
imagen estereo: 480x360 a 20 Hz
features: 900
vocabulario: ORBvoc_L5.txt en multi-dron
allocator: MALLOC_ARENA_MAX=2
```

La calibracion se valida al arrancar con `[CALIB0-WRAPPER-INIT]` y debe mostrar
`camera_valid=true`, ancho/alto 480x360 y baseline estimada 0.057 m. El
vocabulario L5 no cambia el mensaje ROS, pero reduce la memoria DBoW2 privada
por proceso. El vocabulario L6 completo sigue disponible mediante
`orb_vocabulary_path` y debe utilizarse como referencia en futuras pruebas de
relocalizacion/loop de maxima fidelidad.

## Build recomendado si se modifica

El fuente completo esta disponible en `orbslam3_ros2/`, aunque el nombre ROS
declarado en `package.xml` es `orbslam3`. Tras modificarlo, compilar con:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3
```

Compilar tambien los consumidores si cambia algun contrato de mensajes.

La recuperacion de 2026-08-05 se valido con build aislado de `orbslam3` con
codigo 0 y comprobando que el binario `stereo` contiene `BuildOrbMap`,
`PublishOrbMapDelta`, `GetFullMapServiceCallback`, `HashMapPoint`,
`HashKeyFrame` y `UpdateMapEpochFromCurrentMap`.
