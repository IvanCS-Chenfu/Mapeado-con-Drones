# Detalle largo importado - Subfase 4C

Este archivo conserva el detalle del contrato revisado importado desde `Fase_4_completa_4A_4I_muy_detallada.zip`. El contrato ejecutable corto esta en `../subfase_4C.md`.

# Subfase 4C — Evento exacto de creación de KeyFrame e imagen asociada

## Estado

```text
CONSEGUIDA
Preparacion: cerrada
Acuerdo cerrado: si
Autorizacion funcional: concedida y consumida
```

## Condición previa extraordinaria y reconciliación con Fase 2

Existe una implementación parcial/prototipo en el `main` inspeccionado: ORB-SLAM3 expone información del último KF y el wrapper compara el estado antes y después de `TrackStereo()`. El snapshot documental de referencia para esta revisión es `main@4424a586330ca0e54814824fae26bad9daed8232`.

Esa implementación es útil y no debe borrarse sin entenderla, pero **no se considera cierre de 4C**. El contrato definitivo sigue siendo el evento explícito, consumible y one-shot acordado en esta conversación y ya reflejado por la intención del MD oficial de 4C.

La preparación conversada está cerrada. No se modificará código, launch ni
configuración hasta recibir autorización funcional explícita. Al comenzar se
revalidarán `AGENTS.md`, las copias compiladas, las herramientas de build y
`system_architecture`.

## Dependencia

Puede prepararse conceptualmente en paralelo con 4A/4B, pero debe estar cerrada antes de conectar el detector de 4D.

## Objetivo técnico

Garantizar un contrato inequívoco:

> después de procesar una pareja estéreo concreta, el wrapper sabe si **esa misma llamada** a `TrackStereo()` creó un KeyFrame y, si lo creó, conoce exactamente su `keyframe_id`, `source_frame_id` y `timestamp`.

El wrapper debe conservar la imagen izquierda exacta correspondiente a esa llamada para que 4D pueda analizarla.

No se cambia cuándo ORB-SLAM3 decide crear un KF. Solo se añade un recibo/evento de metadatos.

## Problema del mecanismo parcial actual

La aproximación existente es conceptualmente:

```text
antes de TrackStereo:
    último KF = A

TrackStereo(frame N)

después de TrackStereo:
    último KF = B

si A != B:
    inferir que frame N creó B
```

En el flujo estéreo normal puede funcionar porque `TrackStereo()` procesa el frame síncronamente y `Tracking` actualiza el último KF cuando crea uno.

Sin embargo, el contrato definitivo debe ser más fuerte:

```text
Tracking crea el KF
      ↓
Tracking registra un evento de valor
      ↓
System permite consumir ese evento
      ↓
wrapper recibe exactamente el evento de esa llamada
```

Así no se deduce la creación mirando un estado global antes/después: ORB-SLAM3 declara explícitamente la creación en el punto donde ocurre.

## Decisiones cerradas

1. ORB-SLAM3 no detectará fiduciales.
2. No se modifica `NeedNewKeyFrame()`.
3. No se fuerza un KF al ver un tag.
4. No se guardan imágenes dentro de `KeyFrame`, `OrbKeyFrame` ni `OrbMap`.
5. Se añade un evento pequeño de metadatos.
6. El evento se registra exactamente cuando se crea el KF.
7. Se cubre el KF inicial de `StereoInitialization()`.
8. Se cubren KFs normales de `CreateNewKeyFrame()`.
9. El evento se limpia al comenzar el procesamiento de un frame nuevo.
10. El evento se invalida en reset/reset del mapa activo.
11. El wrapper obtiene una copia de valor, nunca un `KeyFrame*` que pueda ser eliminado posteriormente.
12. Un evento solo puede consumirse una vez.
13. Un frame sin KF produce un resultado explícito de "no creado" y no reutiliza el evento anterior.
14. La imagen exacta queda en el wrapper, no en `KeyFrame` ni en el mapa ORB.
15. 4C no crea todavía el worker de detección; solo deja preparado el dato exacto que 4D encolará.
16. `System` es la autoridad del preprocesado geométrico final y rectifica o
    redimensiona una sola vez antes de Tracking y detección.
17. El recibo asocia la imagen con `K`, distorsión, dimensiones y estado de
    rectificación efectivos.
18. La calibración pertenece a Dron; el servidor fiducial no la distribuye.
19. Una guarda impide rectificación simultánea en wrapper y `System`.

## Estructura de evento propuesta

Nombre orientativo:

```cpp
struct KeyFrameCreationEvent
{
    bool created = false;
    uint64_t keyframe_id = 0;
    uint64_t source_frame_id = 0;
    double timestamp = 0.0;
};
```

Los nombres finales conservarán el estilo de la copia ORB-SLAM3 vigente.

No debe transportar:

```text
imagen
MapPoint
pose global
GT
object_id
tag_id
```

## API acordada

La semántica es de recibo/consumo, no de consulta permanente. La extensión de
`TrackStereo()` mantiene compatibilidad mediante salidas opcionales y solo
clona la imagen efectiva cuando esa llamada crea un KF.

Ejemplo conceptual:

```cpp
TrackStereo(..., StereoTrackingReceipt* receipt)
```

El recibo contiene por valor:

```text
KeyFrameCreationEvent
image_left_effective
K_effective
distortion_effective
image_width / image_height
is_rectified
```

`System` consume internamente el evento de `Tracking` dentro de esa llamada. No
expone `KeyFrame*` ni conserva la imagen para una consulta posterior.

Una segunda llamada sin procesar un frame nuevo debe devolver:

```text
created=false
```

No se acepta como contrato final un getter permanente que siga devolviendo el mismo KF una y otra vez.

## Ciclo de vida del evento

### Inicio de un frame

Antes de procesar la nueva imagen:

```text
clear event
```

Esto impide que el KF anterior quede stale.

### StereoInitialization

Cuando se crea el KF inicial desde `mCurrentFrame`:

```text
event.created         = true
event.keyframe_id     = pKF->mnId
event.source_frame_id = mCurrentFrame.mnId
event.timestamp       = mCurrentFrame.mTimeStamp
```

Los nombres de campos reales se verificarán en la versión final.

### CreateNewKeyFrame

Inmediatamente después de crear/registrar el nuevo KF, guardar los mismos datos del frame que lo originó.

### Reset / ResetActiveMap

Cualquier operación que invalide el contexto del mapa debe limpiar el evento.

Nunca puede ocurrir:

```text
reset mapa A
consume evento viejo de mapa A
etiquetarlo como mapa B
```

## Thread-safety

El evento contiene valores escalares y no punteros internos.

La implementación debe revisar si las rutas de reset pueden ejecutarse desde un contexto distinto al tracking. Si existe concurrencia real, la lectura/escritura del evento se protegerá con el mecanismo mínimo necesario (`mutex`, estado atómico apropiado o el patrón ya utilizado por ORB-SLAM3).

No se introducirá una arquitectura compleja si la revisión demuestra que todas las operaciones implicadas están serializadas.

## Flujo del wrapper

Objetivo final dentro de `StereoSlamNode::GrabStereo`:

```text
1. recibir left/right sincronizadas
2. convertir a cv::Mat
3. aplicar una única cadena geométrica de preprocesado
4. TrackStereo(...) aplica el preprocesado final una sola vez
5. recibir `StereoTrackingReceipt`
6. validar evento, imagen y geometría efectiva
7. actualizar/determinar map_epoch una sola vez
8. si no hay KF -> continuar flujo normal
9. si hay KF -> disponer de:
       image_left_exacta
       keyframe_id
       source_frame_id
       timestamp
       map_epoch
10. 4C solo registra/verifica; 4D conectará el enqueue del worker
```

## `map_epoch`

La implementación parcial inspeccionada ya corrigió un problema previo donde `UpdateMapEpochFromCurrentMap()` podía llamarse dos veces.

La versión definitiva debe mantener esta propiedad:

```text
epoch_changed = UpdateMapEpochFromCurrentMap();  // una sola evaluación funcional
```

y reutilizar el resultado.

La identidad que 4D/4E acabarán usando debe capturar el epoch asociado al KF en el momento correcto.

## Imagen izquierda exacta

Este punto es crítico.

No basta con tener una imagen con el mismo timestamp. Debe ser la misma geometría de píxeles sobre la que ORB-SLAM3 creó ese frame/KF.

Se debe inspeccionar:

```text
wrapper doRectify
System::TrackStereo internal rectify
System::TrackStereo internal resize
Settings::needToRectify
Settings::needToResize
```

### Regla

Debe existir **una sola cadena de transformaciones geométricas**.

Flujo acordado:

```text
mensaje ROS -> System::TrackStereo -> rectificacion/resize unico
            -> misma imagen efectiva para Tracking y detector
            -> K/distorsion/dimensiones efectivas en el recibo
```

Si el modo legacy `doRectify` del wrapper y `System::Settings` solicitan ambos
rectificación, el arranque falla con diagnóstico claro en lugar de corregir dos
veces.

Una imagen rectificada usa `K` rectificada y distorsión efectiva cero. Una
imagen no rectificada conserva su `K` y coeficientes reales para PnP. Cualquier
`resize` escala `fx`, `fy`, `cx` y `cy` de forma coherente.

No se permitirá una detección con `K` de una geometría y píxeles de otra.

## Relación `frame_id` vs `keyframe_id`

No deben confundirse:

```text
Frame::mnId    != KeyFrame::mnId
```

Por eso el evento transporta ambos conceptos:

```text
keyframe_id
source_frame_id
```

`source_frame_id` es especialmente útil para comprobar posteriormente que el `OrbKeyFrame` publicado corresponde al frame que generó el evento.

## Impacto en `system_architecture`

4C introduce/refina una interfaz C++ entre ORB-SLAM3 y el wrapper (`KeyFrameCreationEvent`/API de consumo). El `AGENTS.md` vigente exige revisar el visualizador cuando una subfase cambia interfaces. Por tanto, al ejecutar 4C se debe comprobar si la relación ORB-SLAM3 → wrapper está representada en la capa `build/API` de `system_architecture` y, si lo está, actualizar su metadata para reflejar el evento/API real.

No debe iluminarse una nueva arista runtime ROS por este cambio: el evento es una relación interna de biblioteca/API, no un topic/service. No se añade telemetría específica salvo que el visualizador ya tenga un mecanismo explícito para APIs internas; en cualquier caso, debug `false` implica coste cero de observabilidad específica.

## Archivos previstos a modificar

Con el layout vigente:

```text
dron/ORB_SLAM3/include/System.h
dron/ORB_SLAM3/src/System.cc
dron/ORB_SLAM3/include/Tracking.h
dron/ORB_SLAM3/src/Tracking.cc
dron/orbslam3_ros2/src/stereo/stereo-slam-node.hpp
dron/orbslam3_ros2/src/stereo/stereo-slam-node.cpp
codex/contexto/paquetes/orbslam3_ros2/...
codex/contexto/03_ARQUITECTURA_ACTUAL.md / metadata de system_architecture si aplica
codex/contexto de ORB_SLAM3 si existe
metadata `build/API` de system_architecture si representa esta relación
codex/pipeline/fase_4_fiducial_real/...
```

Estas rutas son las actuales, no una autorización anticipada.

## Áreas prohibidas

```text
servidor/orbslam3_server/
servidor/orbslam3_multi/
dron/orbslam3_msgs/ para observaciones 4E
simulacion/simulacion_dron/ salvo escenario de prueba ya existente
LocalMapping/LoopClosing salvo necesidad demostrada y nueva autorización
```

No meter `opencv_aruco` dentro de ORB-SLAM3.

## Cambios requeridos

1. Auditar la implementación `GetLastKeyFrameInfo()` existente.
2. Reemplazar el contrato funcional before/after por un evento explícito consumible.
3. Mantener compatibilidad mínima con el comportamiento actual durante la transición si resulta útil para comparar pruebas.
4. Limpiar el evento al comenzar cada frame.
5. Registrar evento en `StereoInitialization()`.
6. Registrar evento en `CreateNewKeyFrame()`.
7. Limpiarlo en resets relevantes.
8. Exponer desde `System` un valor, no un puntero.
9. Hacer consumo one-shot.
10. Capturar `keyframe_id`, `source_frame_id`, `timestamp` exactos.
11. Conservar la imagen izquierda exacta en el wrapper durante la llamada.
12. Revisar rectificación/resize.
13. Mantener `map_epoch` coherente y sin doble actualización.
14. Añadir logs técnicos acotados.
15. No añadir todavía detección fiducial.
16. Revisar/actualizar la metadata `build/API` de `system_architecture` si el nuevo contrato ORB-SLAM3→wrapper está modelado allí.

## Logs recomendados

```text
[KF-EVENT-CREATED]
[KF-EVENT-NONE]
[KF-EVENT-CONSUMED]
[KF-EVENT-RESET]
[KF-EVENT-MISMATCH]
[WRAPPER-EPOCH]
```

Para un evento creado, log mínimo:

```text
drone_id
epoch
keyframe_id
source_frame_id
event_timestamp
input_timestamp
delta_timestamp
```

No imprimir imágenes ni descriptores.

## Pruebas requeridas

### Prueba 1 — Frames sin KF

Para múltiples frames consecutivos sin creación:

```text
exactamente un resultado NONE por frame auditado
ningún CREATED stale
```

### Prueba 2 — KF inicial

Tras inicialización estéreo:

```text
1 evento CREATED
source_frame_id = frame que inicializó
keyframe_id = KF inicial real
timestamp = timestamp del frame
```

### Prueba 3 — KFs normales

Una trayectoria que cree varios KFs debe producir un evento por cada creación y ninguno duplicado.

### Prueba 4 — Consumo one-shot

Tras consumir un evento:

```text
segunda consulta -> created=false
```

### Prueba 5 — Correspondencia con OrbMap

Cuando el KF aparezca después en el delta/snapshot:

```text
local_keyframe_id coincide
source_frame_id coincide
timestamp coincide dentro de precisión esperada
```

### Prueba 6 — Reset / nuevo mapa

Provocar una ruta real de reset existente y comprobar:

- evento anterior eliminado;
- `map_epoch` cambia según el contrato vigente;
- primer KF posterior pertenece al nuevo epoch;
- no hay mezcla de identidad.

### Prueba 7 — Preprocesado de imagen

Registrar dimensiones y configuración geométrica y demostrar que la imagen reservada para 4D es la misma geometría que recibe Tracking.

### Prueba 8 — Rendimiento

El evento solo manipula unos pocos escalares. La latencia de `TrackStereo` no debe cambiar materialmente por esta subfase.

### Prueba 9 — Metadata `build/API`

Si `system_architecture` modela la relación interna ORB-SLAM3→wrapper, comprobar que la metadata describe la API/evento vigente y que no se ha creado una arista runtime ROS inexistente. Cualquier instrumentación específica debe quedar dormida con debug desactivado.

## Paquetes/build

ORB-SLAM3 debe recompilarse con el procedimiento real vigente y después el wrapper ROS 2.

No se inventará el comando final antes de revisar Fase 2. Se utilizarán los scripts selectivos vigentes y se evitarán rebuilds globales innecesarios.

## Criterio de éxito

4C está realizada solo si:

1. cada KF inicial/normal produce exactamente un evento;
2. cada frame sin KF produce ausencia de evento;
3. el evento es one-shot;
4. no hay eventos stale tras reset;
5. `keyframe_id`, `source_frame_id` y timestamp son correctos;
6. la identidad coincide con el KF exportado posteriormente;
7. la imagen izquierda conservada es geométricamente la imagen del frame que originó el KF;
8. el epoch es coherente;
9. no se ha cambiado la política de creación de KFs;
10. no se ha introducido detección dentro de ORB-SLAM3;
11. no se observa regresión de tracking;
12. `system_architecture` no queda describiendo una API antigua si modela esta relación;
13. build, pruebas y logs quedan documentados.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: asociación ambigua, evento duplicado/stale, timestamp incorrecto, frame equivocado o imagen con otra geometría.
- `PARCIAL`: funciona en KFs normales pero falla inicialización/reset/one-shot.
- `BLOQUEADA`: tras Fase 2 se descubre que la copia de ORB-SLAM3 modificada no es la que enlaza el wrapper y debe localizarse primero la librería real.

## Riesgos

- mantener simultáneamente el getter antiguo y el evento nuevo con semánticas contradictorias;
- limpiar el evento demasiado tarde;
- no cubrir StereoInitialization;
- reset concurrente;
- doble rectificación;
- resize interno no detectado;
- confundir `Frame::mnId` y `KeyFrame::mnId`;
- actualizar epoch después de capturar la identidad equivocada.

## Documentación a actualizar al ejecutar

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/paquetes/orbslam3_ros2/...
codex/contexto de ORB_SLAM3 si está gestionado
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4C.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4C_RESUMEN.md
```
