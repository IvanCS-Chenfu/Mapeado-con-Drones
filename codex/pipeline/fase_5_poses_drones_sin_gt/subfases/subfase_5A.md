# Subfase 5A — Investigación técnica y cierre de contratos de la Fase 5

## Estado

```text
sin hacer
```

## Objetivo técnico

Estudiar físicamente el código y la documentación vigentes después de las Fases 1–4 para convertir los contratos preliminares `5B`–`5I` en especificaciones ejecutables sin suposiciones sobre paths, topics, mensajes, ownership, frames, frecuencias o APIs de ORB-SLAM3.

Esta subfase es de investigación y documentación. No debe cambiar el comportamiento funcional de la pose, el control, el mapa o los mensajes. Su salida principal es una revisión precisa de los MD posteriores.

Al terminar deben quedar resueltas, mediante lectura estática y una línea base de ejecución, al menos estas cuestiones:

1. cuál es la estructura real Dron / Servidor / Simulación posterior a Fase 2;
2. dónde reside y cómo se versiona `orbslam3_msgs` en cada grupo;
3. qué nodo/wrapper publica `orbslam/pose_local`, con qué frame, timestamp y frecuencia;
4. cómo se detecta de forma explícita `TRACKING_OK` / `LOST` y cómo se comunica `map_epoch` al dron;
5. si puede exponerse el `reference_keyframe_id` real de ORB-SLAM3 desde el wrapper sin modificar de forma problemática `ORB_SLAM3`;
6. cómo se obtienen `L_T_KF` y `W_T_KF` en el backend actual y qué API de `GlobalPoseStore` es la autoridad;
7. si los publishers de `MapCorrection` / `CorrectedKeyFrameArray` están activos en el código/launch vigente o son infraestructura histórica no conectada;
8. qué campos faltan en los mensajes actuales para identificar de forma inequívoca `(drone_id, map_epoch, keyframe_id, revision)`;
9. dónde debe vivir el estimador embarcado para que no dependa del paquete Servidor;
10. qué extrínseca cámara↔cuerpo y convenciones `Tcw/Twc` se usan realmente;
11. cuáles son las suscripciones GT funcionales actuales de `gen_tray` y `control_calcular_fuerzas`;
12. qué semántica exacta implementan `absoluto_x/y/z/yaw=false` para Pol3, VelTrap y elipse;
13. qué frecuencia real tiene `pose_local`, qué jitter tiene y qué delay base existe antes de introducir el estimador;
14. cómo reutilizar `simulacion_dron/src/graficar/` sin mezclar drones ni magnitudes;
15. qué mecanismo de simulación permite provocar de forma reproducible pérdida de anchor, pérdida de tracking ORB y recuperación;
16. qué parte del `global_pose_corrector` histórico puede reutilizarse conceptualmente y qué parte debe moverse/reimplementarse para respetar el despliegue Dron/Servidor.

La subfase debe editar `subfase_5B.md` … `subfase_5I.md` con los nombres reales encontrados, eliminar placeholders técnicos y dejar cada prueba preparada para ejecución posterior. No debe ejecutar funcionalmente 5B–5I.

## Contexto obligatorio a leer

Primero, en el orden de `AGENTS.md`:

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/PLANTILLA_SUBFASE_EJECUTABLE.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5.md
```

Después:

```text
codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md
codex/pipeline/fase_3_sparse_global/pipeline_fase_3.md
```

Localizar y leer el pipeline vigente de Fase 4 — Fiducial Real. No asumir una ruta si todavía no existe en el workspace recibido.

Documentación de paquetes que debe consultarse antes de código:

```text
codex/contexto/paquetes/orbslam3_ros2/00_summary.md
codex/contexto/paquetes/orbslam3_ros2/stereo_slam_node.md
codex/contexto/paquetes/orbslam3_msgs/00_summary.md
codex/contexto/paquetes/orbslam3_multi/00_summary.md
codex/contexto/paquetes/orbslam3_multi/raw_map_database.md
codex/contexto/paquetes/orbslam3_multi/global_pose_store.md
codex/contexto/paquetes/orbslam3_server/00_summary.md
codex/contexto/paquetes/orbslam3_server/global_map_server.md
codex/contexto/paquetes/orbslam3_server/global_pose_corrector.md
codex/contexto/paquetes/orbslam3_server/launches.md
codex/contexto/paquetes/dron_individual/00_summary.md
codex/contexto/paquetes/dron_individual/control.md
codex/contexto/paquetes/dron_individual/trayectorias.md
codex/contexto/paquetes/simulacion_dron/00_summary.md
codex/contexto/paquetes/simulacion_dron/graficas_y_gui.md
codex/contexto/paquetes/simulacion_dron/launches.md
codex/contexto/paquetes/simulacion_dron/scenario_runner_node.md
```

ADRs relevantes:

```text
codex/contexto/decisiones/ADR_0001_identidad_submapa.md
codex/contexto/decisiones/ADR_0002_gt_solo_fiducial_y_debug.md
codex/contexto/decisiones/ADR_0005_wrapper_y_mensajes_estables.md
```

Si el historial de Fase 5 está vacío, no inventarlo. Si existen ejecuciones reales posteriores, leer primero índices/resúmenes.

## Diagnóstico de partida

El baseline entregado antes de esta fase muestra varias piezas útiles pero no una cadena activa completa de pose para control:

- el wrapper documentado publica `orbslam/pose_local` como `geometry_msgs/PoseStamped` y solo lo hace cuando tracking está OK;
- `OrbMap` ya transporta `drone_id`, `map_sequence` y `map_epoch`;
- `CorrectedKeyFrameArray` histórico contiene `drone_id` y KFs corregidos, pero no identifica explícitamente `map_epoch` ni revisión de pose;
- `MapCorrection` histórico contiene `map_sequence`, pero tampoco `map_epoch` explícito;
- existe `global_pose_corrector.cpp` con modo `CORRECTED_KEYFRAME_RELATIVE`, fallback rígido, raw/smoothed y extrínseca cuerpo-cámara;
- el launch activo del servidor entregado indica expresamente que no lanza correctores de pose porque todavía no publica `MapCorrection` ni `CorrectedKeyFrameArray`;
- `gen_tray` y `control_calcular_fuerzas` se suscriben actualmente a `sensor/GT/pose` y `sensor/GT/vel`;
- `control_calcular_fuerzas` usa pose, velocidad lineal y velocidad angular actual; la aceleración actual no entra en el cálculo funcional;
- `simulacion_dron/src/graficar/` ya dispone del patrón `numeric_array` + `labels_array` + `graficar.py` y de adaptadores GT/TrayAction.

Estos puntos son únicamente una línea base del código entregado. 5A debe verificar el workspace real posterior a Fases 2–4 y corregir los MD si el estado cambió.

Marcadores históricos que pueden aparecer y no deben confundirse con la nueva numeración de Fase 5:

```text
[POSE4A-*]
[POSE4B-*]
[CALIB0-*]
```

No renombrar masivamente markers históricos solo por numeración.

## Archivos permitidos a modificar

Solo documentación de planificación/contexto, salvo corrección mecánica indispensable:

```text
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5B.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5C.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5D.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5E.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5F.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5G.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5H.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5I.md
codex/contexto/paquetes/*/*.md                 # solo si la investigación demuestra que están desactualizados
codex/contexto/00_CONTEXTO_COMPACTACION.md
```

Puede crear notas temporales dentro de `codex/archivos_auxiliares/` si la política vigente lo permite, pero deben eliminarse o consolidarse al cierre.

## Archivos prohibidos

Durante 5A no modificar comportamiento funcional en:

```text
src/**
ORB_SLAM3/**
orbslam3_ros2/**
build/**
install/**
log/**
```

No cambiar mensajes, launch, YAML ni código para “facilitar” la investigación. Si falta instrumentación imprescindible para medir una propiedad, registrar el bloqueo y pedir autorización antes de introducirla.

## Funciones, clases o nodos que hay que localizar

No asumir paths físicos tras Fase 2; localizar primero el paquete real.

Baseline a buscar:

```text
StereoSlamNode
GrabStereo
PublishLocalPose
PublishOrbMapDelta
BuildOrbMap
UpdateMapEpochFromCurrentMap
FillKeyFrameMsg
HashKeyFrame

GlobalPoseStore::GetWorldPose
GlobalPoseStore
RawMapDatabase
RawKeyFrameId
RawSubmapId

GlobalMapServer
callback/handler que ingiere OrbMap delta
commit de optimización que actualiza GlobalPoseStore
publishers de correcciones si existen

GlobalPoseCorrector::LocalPoseCallback
GlobalPoseCorrector::MapCorrectionCallback
GlobalPoseCorrector::CorrectedKeyFramesCallback
InterpolateTransform
BuildBodyTCameraQuaternion

gen_tray / Clase_Servicio_Accion::execute
control_calcular_fuerzas / callback_pose / callback_vel / enviar_fuerzas

scenario_runner_node
graficar.py
graficar_GTvsTray
```

Topics/interfaces a resolver físicamente:

```text
orbslam/pose_local
orbslam/orb_map_delta
orbslam/get_full_map
map_correction                 # si sigue existiendo
corrected_keyframes            # si sigue existiendo
pose_global_corrected*         # si sigue existiendo
sensor/GT/pose
sensor/GT/vel
sensor/GT/acc
AccionTrayectoria
AccionTrayectoria/_action/feedback
```

Búsquedas mínimas recomendadas:

```bash
rg -n "pose_local|reference.*keyframe|tracking.*state|map_epoch|CorrectedKeyFrame|MapCorrection" . --hidden --glob '!build' --glob '!install' --glob '!log'
rg -n "sensor/GT/(pose|vel|acc)|absoluto_[xy]|absoluto_z|absoluto_yaw" . --hidden --glob '!build' --glob '!install' --glob '!log'
rg -n "GlobalPoseStore|GetWorldPose|optimized.*pose|map_revision|pose_revision" . --hidden --glob '!build' --glob '!install' --glob '!log'
```

## Cambios requeridos

1. Crear una tabla real de ownership por bloque:
   - wrapper/local tracking en Dron;
   - cálculo global/backend en Servidor;
   - métricas/GT/inyección de fallos en Simulación;
   - interfaces en la ubicación que haya fijado Fase 2.
2. Verificar y documentar las transformaciones exactas:
   - qué significa la pose publicada por ORB;
   - `Tcw` vs `Twc`;
   - `local_map_frame`;
   - extrínseca `body_T_camera` o `camera_T_body`;
   - frame esperado por el controlador para pose y twist.
3. Determinar si existe una señal explícita de tracking. Si no existe, fijar en 5B el cambio mínimo de wrapper/interfaz para no inferir `LOST` únicamente por timeout si ORB puede exponer estado real.
4. Investigar `reference_keyframe_id`:
   - comprobar si el wrapper puede obtenerlo de una API pública/estable de ORB-SLAM3;
   - preferir modificación del wrapper;
   - no modificar `ORB_SLAM3` core salvo necesidad demostrada y autorización posterior;
   - si no es viable, dejar 5C/5E con fallback de KFs cercanos.
5. Verificar cómo se materializa `map_epoch` en pose local. Si `PoseStamped` no basta, fijar contrato exacto para evitar aplicar correcciones de un epoch anterior.
6. Verificar el estado real de `global_pose_corrector` y de los publishers de corrección. Decidir qué lógica se reutiliza y dónde se ubica el estimador embarcado después de Fase 2.
7. Definir en 5C el punto exacto del backend donde obtener para cada KF:

```text
L_T_KF
W_T_KF
C_KF = W_T_KF * inverse(L_T_KF)
```

8. Definir en 5D el contrato exacto con `drone_id`, `map_epoch`, revisión y timestamp. Si hay que cambiar `orbslam3_msgs`, enumerar todas las copias/consumidores que Fase 2 obliga a mantener sincronizados.
9. Medir línea base de `orbslam/pose_local`: frecuencia, jitter, header stamp, receive time y comportamiento cuando ORB pierde tracking.
10. Verificar la semántica actual de `TrayAction` para cada tipo de trayectoria cuando `absoluto_*=false`, incluida la elipse. Si hay inconsistencias entre Pol3/VelTrap/elipse, escribirlas en 5B; no corregirlas en 5A.
11. Verificar que el controlador actual no necesita aceleración medida; documentar qué componentes de `TwistStamped` consume realmente.
12. Definir cómo provocar de forma reproducible:
   - submapa no anclado con ORB válido;
   - pérdida de pose global con ORB válido;
   - pérdida de tracking ORB;
   - recuperación de tracking;
   sin utilizar GT para decidir online el estado.
13. Inspeccionar la infraestructura de gráficas y fijar nombres/namespace para 5F/5G evitando `/numeric_array` global compartido entre drones si eso mezcla datos.
14. Editar 5B–5I sustituyendo cualquier formulación provisional por archivos, funciones, topics, parámetros y comandos reales encontrados.
15. Al terminar, dejar en `00_CONTEXTO_COMPACTACION.md`:

```text
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: PENDIENTE
Dudas abiertas: ninguna
Siguiente accion exacta: preparar/ejecutar 5B tras confirmacion del usuario
```

si y solo si no apareció una duda funcional material.

## Cambios prohibidos

- No implementar todavía el estimador de pose.
- No activar ni rediseñar correcciones servidor→dron.
- No sustituir GT en el control.
- No introducir un filtro de velocidad.
- No implementar recuperación ciega.
- No elegir por cuenta propia suavizado raw/smoothed.
- No cambiar `ORB_SLAM3` para obtener el reference KF sin demostrar primero que el wrapper no puede hacerlo.
- No dar por activo un topic porque exista un `.msg`, una clase histórica o un launch antiguo.
- No convertir markers históricos `[POSE4*]` en evidencia de la nueva Fase 5.
- No escribir resultados de línea base dentro de los `subfase_*.md`; la ejecución real irá a historial.

## Paquetes a compilar

Como 5A no modifica código, el build es una comprobación de línea base, no una prueba de cambios.

Después de resolver la estructura real post-Fase 2, compilar de forma aislada los paquetes que realmente participarán. Baseline orientativo del workspace entregado:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs
./codex/herramientas/build_selected_packages.sh orbslam3
./codex/herramientas/build_selected_packages.sh orbslam3_multi
./codex/herramientas/build_selected_packages.sh orbslam3_server
./codex/herramientas/build_selected_packages.sh dron_individual
./codex/herramientas/build_selected_packages.sh simulacion_dron
```

Si Fase 2 exige builds separados por grupo, usar sus scripts/overlays y actualizar 5B–5I con los comandos correctos. No forzar este listado si ya no coincide con la arquitectura real.

## Pruebas Gazebo requeridas

### Prueba 1 — Línea base de pose local y tracking

Objetivo: observar sin modificar código la publicación local de ORB y fijar frecuencia/frames/comportamiento de pérdida.

Secuencia:

1. arrancar una simulación corta con un dron y ORB-SLAM3;
2. esperar tracking estable;
3. medir `orbslam/pose_local` durante un intervalo suficiente;
4. registrar `header.frame_id`, stamps y frecuencia;
5. provocar de forma controlada una escena sin landmarks según el mecanismo real encontrado;
6. comprobar qué hace el wrapper durante `LOST`;
7. recuperar textura/zona conocida y observar relocalización.

Comando: 5A debe reemplazar el launch/escenario por el vigente. Baseline orientativo:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase5_5A_pose_local_baseline \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

No usar GT para determinar el estado de tracking; GT puede registrarse solo como referencia externa.

### Prueba 2 — Inventario ROS de correcciones y control

Con el sistema completo de Fases 3–4 arrancado:

1. comprobar si existen publishers/subscribers de corrección global por dron;
2. comprobar si existe un nodo corrector activo;
3. comprobar las suscripciones GT de `gen_tray` y `control_calcular_fuerzas`;
4. comprobar namespaces reales y tipo de cada topic relevante.

No declarar que un topic es funcional solo porque aparezca en `ros2 topic list`; verificar publisher y flujo de mensajes.

## Patrones de reducción de logs

### Prueba 1

```text
orbslam|TRACKING|LOST|RELOCAL|pose_local|map_epoch|reference|KeyFrame|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
map_correction|corrected_keyframes|pose_global|sensor/GT/pose|sensor/GT/vel|AccionTrayectoria|GlobalPoseStore|ERROR|FATAL|Segmentation fault|Killed
```

Si los markers reales son distintos, 5A debe registrar los nombres encontrados en los MD posteriores. No leer logs completos.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. la estructura física Dron/Servidor/Simulación queda identificada;
2. se conoce el source of truth/copia vigente de interfaces;
3. `pose_local`, tracking state y `map_epoch` tienen un contrato verificable o se ha definido el cambio mínimo exacto para conseguirlo;
4. se ha resuelto si `reference_keyframe_id` es viable; si no, el fallback queda definido;
5. se conocen las APIs reales para obtener `L_T_KF` y `W_T_KF`;
6. se conoce el estado real de los publishers/correctores globales;
7. se han medido frecuencia/jitter de `pose_local` y comportamiento de `LOST` en baseline;
8. se han verificado las entradas GT actuales del control y la semántica de `absoluto_*`;
9. 5B–5I se han actualizado con paths/topics/clases/comandos reales y no contienen supuestos técnicos evitables;
10. no se ha cambiado comportamiento funcional;
11. el build baseline aplicable termina correctamente o cualquier fallo preexistente queda documentado sin atribuirlo a 5A;
12. el historial real de 5A y documentación corregida se actualizan al ejecutar la subfase.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: se modificó funcionalmente el sistema, se inventaron APIs/paths o los MD posteriores siguen dependiendo de supuestos que podían resolverse leyendo el código.
- `PARCIAL`: gran parte del código se entiende, pero falta acceso a wrapper/ORB, Fase 4 o estructura post-Fase 2 y por ello no pueden cerrarse uno o más contratos.
- `BLOQUEADA`: falta físicamente un componente imprescindible —por ejemplo wrapper completo o Fase 4— y no existe evidencia suficiente para especificar sin inventar.

Si aparece una decisión funcional no acordada, no elegir una alternativa: suspender autorización y preguntar al usuario.

## Documentación a actualizar

En ejecución real:

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5B.md
...
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5I.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5A.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5A_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/INDEX.md
```

Actualizar además cualquier `codex/contexto/paquetes/<paquete>/` cuya documentación se demuestre desactualizada. Esta entrega inicial no crea esos archivos de historial porque 5A todavía no se ha ejecutado.
