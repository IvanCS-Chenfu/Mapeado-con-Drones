# Subfase 5E — Estimador embarcado de pose local-global a frecuencia ORB

<!-- ACUERDOS_CIERRE_F2_2026_08_24_START -->
## Estimador embarcado y representación arquitectónica

> **Vigencia:** acuerdo cerrado el 2026-08-24. Este bloque prevalece sobre cualquier
> frase anterior incompatible del mismo documento. No borra ni reescribe evidencia
> histórica; distingue siempre entre estado actual, deuda conocida y arquitectura objetivo.

Tras elegir físicamente dónde vive el estimador, actualizar metadata y flujos
locales sin inventar un paquete/nodo separado si no existe. El grafo principal sigue
siendo por paquetes. La actividad live debe observar interfaces reales y permanecer
apagada cuando el debug está off.
<!-- ACUERDOS_CIERRE_F2_2026_08_24_END -->

## Estado

```text
sin hacer
```

## Objetivo técnico

Implementar en el lado Dron un estimador ligero que produzca una pose utilizable a la frecuencia de `orbslam/pose_local`, combinando cada pose local nueva con la última corrección global por KF válida recibida en 5D.

La prioridad es doble:

1. mantener mínima la carga computacional embarcada;
2. no introducir dependencia de la latencia de red en cada ciclo de pose/control.

El cálculo principal, cuando existe una corrección válida para el KF de referencia seleccionado, es:

```text
W_T_camera(t) = C_KF * L_T_camera(t)
```

y posteriormente:

```text
W_T_body(t) = W_T_camera(t) * camera_T_body
```

según la convención/extrínseca cerrada en 5A.

Cuando no existe pose global válida pero ORB tiene tracking, el estimador debe seguir publicando estado local válido. No debe desaparecer la fuente de pose del dron por no estar anclado.

La salida global debe actualizarse por cada nueva `pose_local`, no solo cuando llega una nueva corrección del servidor.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5A.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5B.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5C.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5D.md
```

Leer los historiales/resúmenes reales de 5A–5D.

Documentación mínima:

```text
codex/contexto/paquetes/orbslam3_ros2/stereo_slam_node.md
codex/contexto/paquetes/orbslam3_server/global_pose_corrector.md   # referencia histórica de matemática/suavizado
codex/contexto/paquetes/dron_individual/control.md
codex/contexto/paquetes/dron_individual/launches.md
codex/contexto/paquetes/orbslam3_msgs/00_summary.md
```

5A debe haber fijado el paquete exacto del estimador embarcado. No ejecutar si el nodo sigue colocado en un paquete Servidor que no se instalaría en el dron real.

## Diagnóstico de partida

Existe una implementación histórica `global_pose_corrector` que ya demuestra varias ideas útiles:

- recibe `orbslam/pose_local`;
- puede usar `CorrectedKeyFrameArray`;
- selecciona un KF corregido cercano;
- aplica:

```text
T_world_current = T_world_kf_corrected * inv(T_local_kf) * T_local_current
```

- dispone de fallback rígido;
- publica raw y smoothed;
- convierte cámara→cuerpo con extrínsecos.

Sin embargo, ese nodo pertenece al paquete `orbslam3_server` del baseline y el launch activo entregado no lo lanza. La arquitectura objetivo de Fase 2 exige que el cálculo por frame que alimenta al control viva en el lado Dron.

5C/5D deben haber trasladado al servidor el cálculo pesado/global y haber dejado al dron un cache de `C_KF`. Por ello 5E no necesita una base global completa, MapPoints, BoW, RANSAC ni optimizador.

5B ya debe haber definido los estados `GLOBAL_VALID`, `LOCAL_ONLY`, `LOCALIZATION_LOST` y la congelación del frame de cada trayectoria.

## Archivos permitidos a modificar

5A debe sustituir paths concretos. Baseline probable si el estimador se integra en `dron_individual`:

```text
src/dron_individual/src/*pose*                  # nuevo componente ligero o ubicación fijada por 5A
src/dron_individual/include/*pose*
src/dron_individual/CMakeLists.txt
src/dron_individual/package.xml
src/dron_individual/launch/generar_dron.launch.py
src/dron_individual/config/*

orbslam3_ros2/...                               # solo si 5A decidió que el estimador pertenece al wrapper
orbslam3_msgs/msg/*                             # solo ajustes ya acordados en 5D

src/simulacion_dron/launch/*
src/simulacion_dron/src/control_tray/scenario_runner_node.cpp

codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/orbslam3_ros2/
codex/contexto/paquetes/orbslam3_msgs/
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_5_poses_drones_sin_gt/
```

No duplicar la misma lógica simultáneamente en `orbslam3_server` y Dron salvo transición temporal explícita para comparación.

## Archivos prohibidos

```text
ORB_SLAM3/**
src/orbslam3_multi/**
src/orbslam3_server/src/global_map_server.cpp    # 5C/5D ya deben proveer contrato
src/dron_individual/src/control_tray/control_calcular_fuerzas.cpp  # sustitución final en 5H
src/simulacion_dron/src/graficar/**              # 5F/5G
build/**
install/**
log/**
```

No implementar matching/optimización global en el dron.

## Funciones, clases o nodos que hay que localizar

5A debe haber fijado nombres definitivos. Baseline/referencias:

```text
StereoSlamNode::PublishLocalPose
<tracking status callback/topic>
<correction cache callback/topic de 5D>

GlobalPoseCorrector::LocalPoseCallback           # solo referencia histórica
GlobalPoseCorrector::PoseMsgToTf2
GlobalPoseCorrector::TransformMsgToTf2
GlobalPoseCorrector::BuildBodyTCameraQuaternion

<nodo estimador embarcado definido en 5A>
```

Entradas mínimas:

```text
orbslam/pose_local
tracking/map_epoch status
correcciones C_KF versionadas de 5D
reference_keyframe_id si está disponible
```

Salidas mínimas equivalentes a:

```text
pose local del cuerpo válida
pose global raw del cuerpo cuando exista
estado de validez/modo
revision/epoch usados
```

Los nombres exactos se fijan en 5A/5D. Evitar crear varios topics redundantes si una interfaz unificada puede expresar estado y frame sin ambigüedad.

## Cambios requeridos

1. Crear el estimador en un paquete que se instale en el dron real y no dependa de Gazebo, GT ni `orbslam3_server`.

2. Mantener siempre la pose local como producto propio mientras ORB tenga tracking:

```text
L_T_camera(t)
L_T_body(t)
```

   La conversión cámara→cuerpo debe usar exactamente la extrínseca validada en 5A, no parámetros duplicados con valores distintos.

3. Mantener un cache de correcciones recibido en 5D, indexado por:

```text
(drone_id, map_epoch, keyframe_id)
```

   y con revisión/edad.

4. Al recibir cada `pose_local`, seleccionar la corrección con esta prioridad:

```text
A. correction(reference_keyframe_id exacto)
B. KF corregido cercano/relevante del mismo epoch
C. fallback rígido válido, solo si 5A/5C lo mantienen
D. sin corrección global -> LOCAL_ONLY
```

5. Si se usa `reference_keyframe_id`, validar que pertenece al mismo `map_epoch`. Si no existe aún en cache, no bloquear ni hacer request síncrono por frame: usar fallback disponible o permanecer local.

6. Calcular la pose global raw:

```text
W_T_camera = C_KF * L_T_camera
W_T_body   = W_T_camera * camera_T_body
```

   o la composición equivalente fijada por la convención de 5A.

7. Publicar una nueva estimación por cada pose local recibida. No esperar a que llegue una corrección nueva.

8. Conservar el `header.stamp` de la observación local que originó la pose estimada. No sustituirlo por `now()` como timestamp físico de pose. Puede registrarse `receive_time` separado para medir delay.

9. Publicar/registrar el modo usado por muestra:

```text
LOCAL_ONLY
GLOBAL_REFERENCE_KF
GLOBAL_NEAREST_KF
GLOBAL_RIGID_FALLBACK
```

   Los nombres exactos pueden variar, pero deben permitir correlacionar errores de 5F con la estrategia activa.

10. Si cambia `map_epoch`:
   - invalidar la selección/corrección activa del epoch anterior;
   - resetear cualquier referencia temporal necesaria;
   - seguir publicando pose local del nuevo epoch;
   - no publicar global hasta disponer de relación válida para el nuevo epoch.

11. Si se pierde pose global pero ORB sigue válido:
   - pasar a `LOCAL_ONLY` sin interrumpir la frecuencia local;
   - notificar el cambio a la máquina de estados de 5B;
   - no continuar etiquetando la última global como válida.

12. Si ORB pierde tracking:
   - dejar de actualizar pose válida;
   - activar `LOCALIZATION_LOST` de 5B;
   - no extrapolar indefinidamente una pose y publicarla como fiable.

13. No aplicar todavía suavizado final. 5E debe publicar una salida raw medible; 5F añadirá/comparará el camino suavizado de forma experimental.

14. Instrumentar frecuencia y delay interno por muestra, de manera throttled/agregada:

```text
input_pose_count
output_pose_count
input_hz
output_hz
processing_us/ms
mode counts
cache hit/miss
reference hit/nearest/fallback counts
```

15. Añadir markers equivalentes a:

```text
[F5E-POSE] mode=... epoch=... ref_kf=... correction_revision=...
[F5E-CACHE-MISS] reason=reference_not_received|epoch_mismatch|stale
[F5E-MODE] old=... new=...
[F5E-STATS] input_hz=... output_hz=... p50_ms=... p95_ms=...
```

16. Mantener el cálculo por pose acotado. No recorrer un conjunto enorme de KFs por frame si 5D puede mantener un cache pequeño/índice espacial. Si se usa búsqueda cercana, 5A/5E deben justificar su complejidad y medirla.

17. Preparar una salida de pose raw global separable de la local para 5F, sin que el controlador final tenga que decidir todavía cuál consumir; la integración final será 5H.

18. No utilizar GT en ningún cálculo, selección, reset, validez o fallback.

## Cambios prohibidos

- No enviar cada `pose_local` al servidor para que este responda con una pose.
- No esperar `service.call()`/action por frame.
- No cargar `RawMapDatabase`, BoW, descriptors o nube global en el dron.
- No modificar el mapa interno de ORB con `C_KF`.
- No usar una corrección de otro epoch.
- No publicar pose global válida si solo existe pose local.
- No ocultar cache miss reutilizando una corrección incompatible.
- No suavizar todavía la salida principal como decisión definitiva.
- No derivar velocidad definitiva en esta subfase; 5G.
- No sustituir aún todas las suscripciones GT del control; 5H.

## Paquetes a compilar

5A debe fijar el paquete del estimador. Baseline si vive en `dron_individual`:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs dron_individual simulacion_dron
```

Añadir `orbslam3` solo si se modificó wrapper/reference KF y `orbslam3_server` solo si 5D necesita ajuste mecánico de contrato.

## Pruebas Gazebo requeridas

### Prueba 1 — Frecuencia local sin anchor

1. arrancar un dron con ORB tracking válido y sin anchor;
2. medir frecuencia de `orbslam/pose_local`;
3. medir frecuencia de la salida local del estimador;
4. comprobar `LOCAL_ONLY`;
5. comprobar que no se publica una global falsa.

La relación `output/input` debe ser próxima a 1 salvo pérdidas explícitamente justificadas; 5A debe haber fijado cómo cuantificarla con la frecuencia real del sistema.

### Prueba 2 — Aparición del primer anchor

1. comenzar en `LOCAL_ONLY`;
2. adquirir fiducial/anchor;
3. recibir correcciones de 5D;
4. comprobar que el estimador pasa a global sin dejar de procesar cada `pose_local`;
5. comprobar que la trayectoria local activa, si existe, sigue usando local según 5B.

### Prueba 3 — Reference KF exacto / fallback

Si existe `reference_keyframe_id`:

- comprobar hits de corrección exacta;
- forzar temporalmente un miss de cache y observar fallback/no bloqueo.

Si no existe:

- comprobar selección cercana/relevante;
- registrar distancia al KF seleccionado y modo.

### Prueba 4 — Corrección cambia durante vuelo

1. volar con pose global raw activa;
2. provocar una optimización que cambie `C_KF`;
3. comprobar que la siguiente `pose_local` usa la revisión nueva;
4. medir salto raw, pero no corregirlo todavía con suavizado.

### Prueba 5 — Pérdida de servidor/correcciones

1. con cache válido, interrumpir 5D temporalmente;
2. comprobar que ORB y el estimador siguen a frecuencia local usando la última corrección permitida por política;
3. si la corrección cruza el límite de validez acordado, pasar a `LOCAL_ONLY` sin dejar de publicar local;
4. reanudar servidor y recuperar global.

### Prueba 6 — Cambio de `map_epoch`

Provocar reset/relocalización que cree epoch nuevo:

- no usar `C_KF` anterior;
- local sigue disponible si ORB tracking está OK;
- global reaparece solo tras relación válida del nuevo submapa.

### Prueba 7 — Prueba larga de frecuencia y carga

Dos drones alrededor del edificio/casa con servidor y optimizaciones activas.

Medir por dron:

```text
pose_local input count/hz
estimated pose output count/hz
processing p50/p95
cache hit ratio
reference/nearest/fallback ratio
mode transitions
```

No usar todavía GT como criterio de exactitud; eso se hace en 5F. Sí puede registrarse externamente para que la siguiente subfase reutilice el escenario.

## Patrones de reducción de logs

```text
F5E-POSE|F5E-CACHE-MISS|F5E-MODE|F5E-STATS|reference|nearest|fallback|map_epoch|correction_revision|TRACKING|LOST|ERROR|FATAL|Segmentation fault|Killed
```

Para la prueba larga, generar sublogs por dron y por `F5E-STATS` si hace falta.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. el estimador vive en el lado Dron y no depende de Gazebo/GT/servidor por frame;
2. cada `pose_local` válida produce una salida local válida con frecuencia equivalente salvo pérdidas justificadas;
3. cuando hay corrección global válida, cada nueva pose local produce una pose global raw usando la última revisión cacheada;
4. el cálculo por frame no necesita round-trip;
5. reference KF exacto se usa si está disponible; el fallback funciona sin mezclar epoch;
6. un cache miss no bloquea ORB/control;
7. cambio de epoch invalida correcciones anteriores;
8. pérdida de global mantiene `LOCAL_ONLY`; pérdida de ORB entra en `LOCALIZATION_LOST`;
9. la extrínseca cámara↔cuerpo se aplica de forma coherente;
10. frecuencia, processing delay y ratios de modo quedan medidos;
11. no se usa GT funcionalmente;
12. build y pruebas, incluida la larga, terminan sin errores graves no explicados;
13. historial y docs quedan actualizados.

La precisión absoluta final no se declara en 5E; se cuantifica en 5F.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: la salida solo cambia cuando llega el servidor, el dron hace consultas síncronas por frame, se mezclan epochs, se usa GT o el estimador no puede mantener la frecuencia ORB.
- `PARCIAL`: la arquitectura funciona pero existe pérdida material de frecuencia, processing delay alto o dependencia excesiva del fallback cercano.
- `BLOQUEADA`: falta una interfaz de tracking/reference/corrección definida en 5A–5D y no puede resolverse sin cambiar el contrato funcional.

## Documentación a actualizar

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/paquetes/dron_individual/        # o paquete real del estimador
codex/contexto/paquetes/orbslam3_ros2/          # si se modifica wrapper
codex/contexto/paquetes/orbslam3_msgs/          # si cambia contrato
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/INDEX.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5E.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5E_RESUMEN.md
```
