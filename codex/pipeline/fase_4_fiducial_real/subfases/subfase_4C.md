# Subfase 4C — Asociación exacta entre imagen y KeyFrame

## Estado

```text
sin hacer
```

## Dependencia

Puede desarrollarse en paralelo con `4A/4B`. Debe completarse antes de `4D`.

## Objetivo técnico

Garantizar que el wrapper sepa de forma determinista si la pareja estéreo que acaba de entregar a `ORB_SLAM3::System::TrackStereo()` ha originado un nuevo KeyFrame y, en caso afirmativo, cuál es su `keyframe_id`, `frame_id` y `timestamp`.

ORB-SLAM3 solo expone el evento de creación. No debe contener dependencias AprilTag/OpenCV aruco nuevas ni realizar detección fiducial. El wrapper debe conservar la imagen izquierda exacta que ORB-SLAM3 usó para construir ese KF.

## Comportamiento esperado

```text
frame normal -> TrackStereo -> no evento -> no procesamiento fiducial
frame KF     -> TrackStereo -> evento exacto -> wrapper conserva imagen de ese mismo frame
```

No se acepta como solución principal comparar deltas, esperar a `GetAllKeyFrames()` ni elegir el KF temporalmente más próximo.

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


Además, leer los códigos reales vigentes antes de editar:

```text
ORB_SLAM3/include/System.h
ORB_SLAM3/src/System.cc
ORB_SLAM3/include/Tracking.h
ORB_SLAM3/src/Tracking.cc
wrapper stereo-slam-node.hpp
wrapper stereo-slam-node.cpp
```

La ruta exacta del wrapper puede variar según cómo esté montado `ORB_SLAM3_ROS2`; `planificador_fase` debe localizarla estáticamente y no asumir una copia distinta del repositorio actual del usuario.

## Diagnóstico de partida

En el código actual:

- `System::TrackStereo()` llama síncronamente a `mpTracker->GrabImageStereo(...)`.
- `Tracking::GrabImageStereo()` crea `mCurrentFrame` con la imagen/timestamp y ejecuta `Track()`.
- `Track()` llama a `CreateNewKeyFrame()` cuando `NeedNewKeyFrame()` lo decide.
- `CreateNewKeyFrame()` construye `KeyFrame(mCurrentFrame, ...)` y actualiza `mpLastKeyFrame` antes de terminar.
- `StereoInitialization()` también crea el KF inicial desde `mCurrentFrame`.

Por tanto, la relación exacta existe dentro de Tracking, pero `System`/wrapper no exponen todavía un contrato explícito de “esta llamada creó este KF”.

El wrapper actual también llama dos veces seguidas a `UpdateMapEpochFromCurrentMap()` en el flujo mostrado; si la primera consume el cambio de epoch y su retorno se ignora, la publicación inmediata puede perderse. Esta subfase debe revisar y corregir esa duplicidad porque la identidad de observación incluye `map_epoch`.

## Archivos permitidos a modificar

Rutas de ORB-SLAM3 explícitamente autorizadas por el objetivo de esta subfase:

```text
ORB_SLAM3/include/System.h
ORB_SLAM3/src/System.cc
ORB_SLAM3/include/Tracking.h
ORB_SLAM3/src/Tracking.cc
ORB_SLAM3_ROS2/src/stereo/stereo-slam-node.hpp
ORB_SLAM3_ROS2/src/stereo/stereo-slam-node.cpp
documentación del wrapper/ORB-SLAM3 incluida en codex, si existe
```

Si las rutas locales tienen otro prefijo, localizar la copia que realmente se compila.

## Archivos prohibidos

```text
src/orbslam3_server/
src/orbslam3_multi/
src/orbslam3_msgs/
src/simulacion_dron/src/fiducials/      # salvo pruebas de integración ya creadas en 4B
src/dron_individual/
legacy ORB-SLAM3 no compilado
build/
install/
log/
```

## Funciones, clases o nodos que hay que localizar

```text
ORB_SLAM3::System::TrackStereo
ORB_SLAM3::Tracking::GrabImageStereo
ORB_SLAM3::Tracking::Track
ORB_SLAM3::Tracking::NeedNewKeyFrame
ORB_SLAM3::Tracking::CreateNewKeyFrame
ORB_SLAM3::Tracking::StereoInitialization
ORB_SLAM3::Tracking::Reset
ORB_SLAM3::Tracking::ResetActiveMap
StereoSlamNode::GrabStereo
StereoSlamNode::UpdateMapEpochFromCurrentMap
```

## Cambios requeridos

1. Definir una estructura pequeña de evento de creación de KF con, como mínimo, `created`, `keyframe_id`, `frame_id` y `timestamp`.
2. Limpiar/resetear el evento al comenzar a procesar un nuevo frame para impedir que un KF anterior se reutilice por error.
3. Registrar el evento inmediatamente después de crear el KF normal en `CreateNewKeyFrame()`.
4. Registrar también el KF inicial dentro de `StereoInitialization()`.
5. Limpiar el estado del evento en `Reset`, `ResetActiveMap` y cualquier ruta equivalente que invalide el mapa activo.
6. Exponer desde `System` una función de consulta/consumo de valor, evitando entregar al wrapper un puntero interno que pueda ser culled posteriormente.
7. Hacer que el evento de una llamada solo pueda consumirse una vez.
8. En el wrapper, conservar la imagen izquierda correspondiente a esa llamada a `TrackStereo()` hasta conocer el resultado del evento.
9. Garantizar una única cadena de preprocesado geométrico. La imagen que se entregue al futuro detector debe ser exactamente la que llega a `Tracking::GrabImageStereo` tras rectificación/resize.
10. Revisar la coexistencia de `doRectify` del wrapper con `settings_->needToRectify()/needToResize()` de `System::TrackStereo` y evitar doble rectificación/redimensionado.
11. Corregir la doble llamada a `UpdateMapEpochFromCurrentMap()` si sigue presente: calcular `epoch_changed` una sola vez y reutilizar el resultado.
12. Añadir logs técnicos sin imagen, por ejemplo `KF-EVENT-CREATED`, `KF-EVENT-NONE`, con ID/frame/timestamp/epoch.
13. No almacenar imágenes indefinidamente: la imagen solo vive durante el callback o en una cola acotada si el diseño final la necesita.

## Cambios prohibidos

- No añadir detector AprilTag a ORB-SLAM3.
- No modificar la política `NeedNewKeyFrame()` para forzar KFs cuando aparece un tag.
- No adjuntar imágenes a `OrbKeyFrame`/`OrbMap`.
- No usar `GetAllKeyFrames()` antes/después como asociación funcional.
- No usar “timestamp más cercano” como sustituto del evento exacto.
- No modificar LocalMapping/LoopClosing salvo que una búsqueda demuestre una necesidad imprescindible y el usuario la autorice.
- No introducir un thread detector en esta subfase.

## Paquetes a compilar

La librería ORB-SLAM3 debe recompilarse con su procedimiento real y después el wrapper ROS 2. Si el wrapper está integrado en un workspace con script seleccionado, usar el comando equivalente documentado.

Para paquetes ROS afectados, como mínimo:

```bash
./codex/herramientas/build_selected_packages.sh <paquete_wrapper_real>
```

No inventar el nombre del paquete si no coincide con `ORB_SLAM3_ROS2`; localizarlo primero.

## Pruebas Gazebo requeridas

### Prueba 1 — Evento exacto en tracking normal

Ejecutar una trayectoria que genere varios KFs. Por cada callback de imagen:

- `KF-EVENT-NONE` o un único `KF-EVENT-CREATED`;
- ningún evento duplicado en el frame siguiente;
- `event.timestamp` coincide con el timestamp de la imagen que originó el KF;
- `event.frame_id` coincide con el `mnFrameId` exportado por ese KF cuando aparezca después en `OrbMap`.

### Prueba 2 — KF inicial

Reiniciar el mapa y verificar que el primer KF generado por `StereoInitialization()` produce evento y queda asociado al frame inicial correcto.

### Prueba 3 — Reset / map_epoch

Provocar o usar una ruta de reset existente. Verificar que no aparece un evento stale del mapa anterior y que el wrapper etiqueta el siguiente KF con el epoch vigente.

## Patrones de reducción de logs

```text
KF-EVENT|WRAPPER-EPOCH|PIPE0-WRAPPER|TrackStereo|frame_id|keyframe_id|timestamp|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

1. ORB-SLAM3 y wrapper compilan.
2. Cada KF normal e inicial produce exactamente un evento de creación.
3. Un frame sin KF no produce un evento reutilizado.
4. La identidad del evento coincide con el KF que luego exporta el mapa.
5. La imagen conservada por wrapper es la misma geometría que recibió Tracking.
6. Reset/epoch no mezclan eventos entre submapas.
7. No se ha añadido ninguna lógica fiducial dentro de ORB-SLAM3.
8. La latencia de tracking no cambia materialmente por el evento de metadatos.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: eventos ambiguos, duplicados, stale, asociados a otro frame o pérdida de epoch.
- `PARCIAL`: evento exacto funciona en tracking normal pero falla en inicialización/reset.
- `BLOQUEADA`: la copia real de ORB-SLAM3 que se compila no puede identificarse o el wrapper enlaza contra otra biblioteca distinta.

## Riesgos

- doble rectificación entre wrapper y `System`;
- consultar un puntero a KF que LocalMapping pueda eliminar;
- evento stale tras reset;
- confundir `Frame::mnId` con `KeyFrame::mnId`;
- actualizar `map_epoch` después de publicar una observación futura.

## Documentación a actualizar

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/paquetes/<paquete_wrapper_real>/
documentación de la librería ORB-SLAM3 si está gestionada en codex
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4C.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4C_RESUMEN.md
```
