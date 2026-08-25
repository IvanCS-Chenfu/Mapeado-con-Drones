# Detalle largo importado - Subfase 4D

Este archivo conserva el detalle del contrato revisado importado desde `Fase_4_completa_4A_4I_muy_detallada.zip`. El contrato ejecutable corto esta en `../subfase_4D.md`.

# Subfase 4D — Configuración remota, worker fiducial y estimación `camera_T_tag`

## Estado

```text
PARCIAL
Preparacion: cerrada
Acuerdo cerrado: si
Autorizacion funcional: concedida y consumida
```

## Acuerdo conversacional cerrado

- detector `APRILTAG_36H11`, refinamiento `SUBPIX` y solver `IPPE_SQUARE`;
- `max_reprojection_error_px=3.0` invalida observaciones visuales;
- retry `1 s`, timeout `2 s`, cola `4`, `drop oldest / keep newest`;
- rango fiducial `1-5 m` no filtra en Dron: las observaciones geométricamente
  válidas llegan a Servidor y 4G decide su uso;
- `quality_score=clamp(1-error_px/max_error_px,0,1)` inicialmente, sin perder
  error, área ni métricas IPPE originales;
- debug HighGUI opcional, no bloqueante y definido más adelante;
- prueba integral con trayectoria típica revisada, Gazebo/RViz2 y grafo de
  arquitectura activo, seguida de smoke corto con el grafo apagado.

## Condición previa extraordinaria y reconciliación con Fase 2

Esta subfase contiene decisiones que difieren del MD 4D antiguo y del sandbox original:

- la configuración del detector **sí se obtiene mediante un servicio del servidor**;
- se necesita una interfaz ROS mínima de configuración antes de 4E;
- la detección **no debe ejecutarse síncronamente dentro de `GrabStereo()`**;
- habrá un worker separado y una cola acotada;
- los KFs anteriores a tener configuración no se procesan.

La actualización de ADR 0009 realizada durante el cierre de Fase 2 confirma expresamente este patrón: un valor controlado por Servidor y consumido en Dron debe obtenerse mediante cliente Dron al arrancar → servicio de configuración Servidor → valor local en Dron. Por tanto, el servicio acordado para 4D ya no es solo una preferencia de diseño: es la materialización en Fase 4 del contrato futuro previsto por ADR 0009.

El snapshot documental usado inicialmente para reconciliar este MD fue
`main@4424a586330ca0e54814824fae26bad9daed8232`. Fase 2 y la preparación
conversada están cerradas; no se implementará hasta recibir autorización
funcional explícita. Al comenzar se revalidarán `AGENTS.md`, las dos copias de
`orbslam3_msgs`, sus guardas, `system_architecture` y los builds por grupo.

## Dependencias

```text
4A — contrato de objetos/tags y size_m
4B — objetos visuales reales en Gazebo
4C — evento exacto de KeyFrame + imagen izquierda exacta
```

## Objetivo técnico

Añadir al wrapper un subsistema visual desacoplado del tracking que:

1. obtenga del servidor la configuración mínima necesaria para visión;
2. espere/reintente sin bloquear ORB-SLAM3 si el servicio aún no está disponible;
3. empiece a procesar únicamente KFs creados después de disponer de configuración;
4. reciba del flujo principal trabajos `{identidad KF + imagen izquierda clonada}`;
5. procese esos trabajos en un único worker;
6. detecte `0..N` AprilTags `36h11`;
7. use el `size_m` específico de cada `tag_id`;
8. calcule la mejor `camera_T_tag` físicamente válida disponible mediante PnP/IPPE;
9. produzca métricas de calidad y coste;
10. nunca bloquee el flujo principal del wrapper, publicación de pose o envío de deltas.

4D termina con resultados internos de detección. El contrato ROS que enviará observaciones por KF al servidor se define en 4E.

## Arquitectura cerrada

```text
                         ARRANQUE

orbslam3_server
    |
    +--> fiducial_config_server   (nodo independiente)
              |
              +--> servicio GetFiducialConfig
                         |
                         v
                  wrapper de dron
                         |
                  worker fiducial
                         |
             WAIT_CONFIG / retry
                         |
                 configuración RAM


                         EJECUCIÓN

left/right camera
      |
      v
StereoSlamNode::GrabStereo
      |
      +--> TrackStereo(...) --------------------------+
      |                                               |
      |                                               +--> pose/deltas normales
      |
      +--> ConsumeKeyFrameCreationEvent()
                 |
                 +--> no KF -> nada de visión
                 |
                 +--> KF y config NO disponible -> no procesar ese KF
                 |
                 +--> KF y config READY
                           |
                           +--> clone(image_left_exacta)
                           +--> capturar identidad exacta
                           +--> enqueue no bloqueante
                                      |
                                      v
                               COLA ACOTADA
                                      |
                                      v
                               WORKER ÚNICO
                                      |
                               AprilTag 36h11
                                      |
                               tag_id detectado
                                      |
                               lookup tag_id -> size_m
                                      |
                               IPPE/PnP candidatos
                                      |
                               seleccionar mejor pose válida
                                      |
                               camera_T_tag
                               reprojection_error_px
                               tag_area_px2
                               tiempos
```

## Separación de responsabilidades

### El wrapper SÍ conoce

```text
familia del detector
parámetros del detector
tag_id -> size_m
intrínsecos de cámara
identidad exacta del KF
imagen izquierda exacta
```

### El wrapper NO conoce

```text
object_id
cara física a la que pertenece el tag
world_T_object
object_T_tag
world_T_tag
world_T_camera
anchor/revisit
GT
```

La agrupación de caras del mismo objeto pertenece a 4G en el servidor.

## Configuración mediante servicio — decisión cerrada

No habrá un YAML local del wrapper con copia de `tag_id -> size_m`.

Al arrancar, el subsistema fiducial del wrapper obtiene la configuración del servidor mediante un servicio ROS.

El sandbox propuso y se mantiene como baseline conceptual:

```text
servicio: /global_mapping/get_fiducial_config
```

El nombre exacto se revalidará contra las convenciones finales del repositorio, pero cualquier cambio deberá conservar la misma semántica.

## Nodo `fiducial_config_server`

Será un nodo independiente del servidor global.

Responsabilidades:

1. cargar el `fiducial_objects.yaml` correspondiente al modo de despliegue;
2. validar el esquema relevante;
3. extraer solo los parámetros necesarios por visión;
4. crear el servicio;
5. devolver la misma configuración a todos los drones;
6. usar `drone_id/drone_name` únicamente para trazabilidad/log;
7. no ejecutar anchor ni optimización;
8. no cambiar configuración dinámicamente durante una ejecución.

No se integrará esta responsabilidad dentro de `global_map_server.cpp` salvo que el layout final de Fase 2 demuestre una razón fuerte para ello y se vuelva a discutir.

## ADR 0009: ownership, authority y distribución Server→Dron

Se conserva ADR 0009 tal como queda tras Fase 2. 4D debe implementar explícitamente los tres conceptos:

```text
semantic ownership
authority/control
deployment source/profile
```

### Datos físicos de la instalación

`fiducial_objects.yaml` contiene la información física/global necesaria para conocer qué tags existen y su tamaño. Su perfil concreto depende del deployment.

En Simulación, `simulacion_dron` usa su fuente local de deployment para crear Gazebo. Cuando el deployment Gazebo arranca el `fiducial_config_server`, debe seleccionar/pasar ese mismo perfil mediante el mecanismo admitido por Fase 2, sin que un nodo Dron abra YAML de otro grupo.

En Servidor standalone/real existe el perfil correspondiente al despliegue real; puede representar `fiducials disabled/not configured` hasta que el operador declare objetos. Si ambos archivos representan el mismo perfil, las guardas deben exigir la igualdad declarada. Si representan perfiles distintos, la divergencia debe estar documentada como decisión de deployment.

### Parámetros del detector

`corner_refinement`, `pose_solver`, `max_reprojection_error_px` y otros parámetros funcionales de detección son **controlados por Servidor**. El Dron no mantiene una copia YAML. El `fiducial_config_server` los carga desde configuración local del Servidor/deployment y los entrega mediante servicio junto con `family` y `tag_id -> size_m`.

El nombre/archivo exacto de esa política puede decidirse al implementar tras inspeccionar los perfiles vigentes, pero no se duplicará semánticamente dentro del mismo perfil.

### Parámetros puramente gráficos

`surface_offset_m`, material o resolución de textura son de Simulación y no se envían al Dron mediante `GetFiducialConfig`.

### Prohibición de YAML cross-group en Dron

El wrapper no puede leer:

```text
servidor/.../fiducial_objects.yaml
simulacion/.../fiducial_objects.yaml
```

ni rutas fuente equivalentes. Solo conoce la respuesta del servicio. Esto preserva Dron como caja negra de despliegue.

## Interfaz ROS mínima de configuración

4D necesita adelantar únicamente las interfaces relacionadas con **configuración**. Esto no sustituye 4E, que seguirá siendo responsable de los mensajes de observaciones por KF.

### Mensaje propuesto

```text
FiducialTagConfig.msg
```

Contenido:

```text
uint32 tag_id
float64 size_m
```

### Servicio propuesto

```text
GetFiducialConfig.srv
```

Petición:

```text
uint32 drone_id
string drone_name
```

Respuesta mínima:

```text
bool success
string message
uint32 schema_version
string family
string corner_refinement
string pose_solver
float64 max_reprojection_error_px
orbslam3_msgs/FiducialTagConfig[] tags
```

El servicio NO devuelve:

```text
object_id
world_T_object
object_T_tag
nombre de cara
pose global del dron
GT
```

## Configuración vacía

Si el perfil contiene cero tags habilitados, no se considerará un error fatal.

Estado semántico:

```text
fiducials disabled/not configured
```

Una respuesta válida puede tener:

```text
success = true
tags = []
message = "fiducials disabled/not configured"
```

El wrapper pasa a estado `DISABLED` para la parte fiducial y ORB-SLAM3 continúa normalmente.

Como la configuración es inmutable durante la ejecución, para cargar tags nuevos se reinicia el proceso/despliegue.

## Configuración inmutable

Una vez que el wrapper recibe una configuración válida:

```text
config_version fija
family fija
tag_id -> size_m fijo
parámetros detector fijos
```

No se implementarán actualizaciones hot-reload en Fase 4.

Si el operador cambia el YAML, debe reiniciar para que todos los componentes trabajen con el mismo contrato.

## Estado de espera y reintentos

El worker fiducial empieza sin configuración.

Estados conceptuales:

```text
WAIT_SERVICE
REQUEST_CONFIG
READY
DISABLED
```

### WAIT_SERVICE

El worker no toca la cola de detección y reintenta encontrar/consultar el
servicio cada `1 s` por defecto.

El intervalo es configurable, con default acordado:

```text
1 s
```

El timeout de petición es configurable y parte de `2 s`.

### REQUEST_CONFIG

Cada petición tiene timeout finito.

Nunca debe ocurrir el fallo del prototipo en el que una petición queda marcada como "in flight" para siempre si el servidor desaparece.

Si falla/expira:

```text
log reducido
esperar intervalo
volver a WAIT_SERVICE
```

### READY

Configuración validada y con al menos un tag. Se permite encolar KFs futuros.

### DISABLED

El servidor respondió correctamente pero no existen tags configurados. No es un fallo del SLAM.

## KFs anteriores a la configuración — decisión cerrada

No se almacenan imágenes de KFs antiguos esperando a que llegue el servicio.

Mientras el detector no esté `READY`:

```text
KF creado -> no se clona imagen -> no se procesa fiducial
```

Esto evita:

- buffer especial de startup;
- crecimiento de memoria;
- reprocesado tardío;
- mezcla de epoch/configuración.

La detección empieza exclusivamente con KFs creados después de quedar `READY`.

## Worker único por dron

Habrá exactamente un worker fiducial por wrapper/dron en el baseline.

Motivos:

- limita consumo CPU;
- mantiene orden de trabajos;
- simplifica shutdown;
- evita detecciones concurrentes sobre OpenCV sin necesidad;
- facilita medir latencia y backlog.

No se crea un pool de threads en 4D.

## Trabajo encolado

Cuando 4C confirma un KF y la configuración está `READY`, el callback principal crea un trabajo de valor.

Contenido mínimo:

```text
drone_id
map_epoch
keyframe_id
source_frame_id
keyframe_timestamp
camera_frame_id / frame óptico si procede
cv::Mat image_left_clone
```

### `cv::Mat::clone()` obligatorio

No basta con copiar el header de `cv::Mat`.

La imagen puede compartir memoria con `sensor_msgs::Image`/`cv_bridge`; al terminar el callback esa memoria puede dejar de ser válida.

Por tanto:

```text
worker_job.image = imLeftForTracking.clone()
```

solo cuando se crea un KF y la configuración está lista.

No se clonan imágenes de frames normales.

## Captura de identidad antes de soltar el callback

El trabajo guarda `map_epoch`, timestamp e IDs en el momento de encolado.

El worker NO debe consultar después:

```text
map_epoch_ actual
último KF actual
último timestamp actual
```

porque mientras procesa un trabajo el wrapper puede haber avanzado, reiniciado mapa o creado otros KFs.

## Cola acotada

Capacidad acordada y configurable:

```text
4 KFs
```

Un ajuste futuro se hará con mediciones y no durante esta ejecución sin volver
a documentar la decisión.

La capacidad debe ser explícita y nunca ilimitada.

### Política cuando se llena — decisión cerrada

```text
drop oldest
keep newest
```

Es decir, si llega un nuevo KF y la cola está llena:

1. eliminar el trabajo más antiguo pendiente;
2. registrar `FID-WORKER-DROP` con la identidad del trabajo descartado;
3. insertar el KF nuevo;
4. no bloquear `GrabStereo()` esperando hueco.

La política prioriza información visual reciente y garantiza memoria acotada.

## Shutdown

El destructor/cierre del wrapper debe:

1. indicar al worker que termine;
2. despertar cualquier `condition_variable`;
3. decidir explícitamente si se drena o se descarta la cola al cerrar;
4. hacer `join()` del thread;
5. no dejar callbacks accediendo a miembros ya destruidos.

Para un cierre normal del nodo, se prioriza shutdown limpio sobre terminar detecciones pendientes que ya no podrán publicarse.

## `FiducialDetector` como componente separado

La clase del sandbox es una base útil y debe mantenerse conceptualmente separada de `StereoSlamNode`.

Interfaz orientativa:

```text
Configure(...)
IsConfigured()
Detect(image)
```

Entrada de configuración:

```text
family
corner_refinement
pose_solver
max_reprojection_error_px
vector<tag_id,size_m>
K de cámara
distorsión efectiva
```

Salida interna por tag:

```text
tag_id
R_camera_tag
t_camera_tag_m
reprojection_error_px
tag_area_px2
```

Salida de la imagen completa:

```text
raw_candidates
observations[]
rejections[]
detect_ms
pose_ms
total_ms
```

## Detección AprilTag

Baseline:

```text
OpenCV aruco
DICT_APRILTAG_36h11
imagen izquierda exacta del KF
```

No se analiza la imagen derecha para cerrar 4D.

El detector debe localizar todas las marcas visibles, no detenerse en la primera.

Resultado válido por KF:

```text
0 tags -> vector vacío
1 tag  -> una observación
N tags -> N observaciones independientes
```

Dos tags del mismo objeto siguen siendo dos observaciones distintas en el wrapper.

## IDs desconocidos

OpenCV puede detectar un `tag_id` perteneciente al diccionario pero no presente en la configuración recibida.

En ese caso:

```text
rejection = unknown_tag_id
```

No se intenta PnP con un tamaño inventado y no se bloquea el procesamiento del resto de tags.

## Tamaños distintos por tag

Esta es una condición funcional importante.

El detector no puede usar un único `tag_size_m` global.

Debe hacer:

```text
detect tag_id
    ↓
lookup size_m específico
    ↓
BuildSquareObjectPoints(size_m)
    ↓
PnP
```

Aunque el escenario baseline use 0.30 m en los 15 tags, una prueba específica de 4D utilizará tamaños distintos.

## Puntos 3D y frame `tag`

Con lado `L = size_m`, los cuatro puntos deben representar un cuadrado centrado en el origen del tag:

```text
(-L/2, +L/2, 0)
(+L/2, +L/2, 0)
(+L/2, -L/2, 0)
(-L/2, -L/2, 0)
```

El orden debe comprobarse contra la documentación/API de la versión concreta de OpenCV para `SOLVEPNP_IPPE_SQUARE`.

Frame:

```text
+X_tag = derecha visual
+Y_tag = arriba visual
+Z_tag = normal exterior
```

## Elección de la mejor solución PnP — decisión de calidad

No se copiará obligatoriamente el `solvePnP()` simple del prototipo.

Para una marca planar IPPE puede producir soluciones candidatas. El objetivo es aprovechar la API que permita inspeccionar la información disponible, por ejemplo `solvePnPGeneric` con `IPPE_SQUARE`, y seleccionar la mejor solución físicamente válida.

Criterios mínimos:

1. solución numéricamente finita;
2. `t_camera_tag.z > 0`;
3. puntos del cuadrado delante de la cámara;
4. matriz de rotación ortonormal;
5. determinante cercano a +1;
6. reproyección finita;
7. menor error de reproyección entre candidatos físicamente válidos.

La política de ambigüedad planar, coherencia multi-tag y falsos positivos se
endurece en las capas propietarias posteriores, especialmente 4G y la prueba
integral 4H. 4D debe obtener la mejor información disponible, no esconder
candidatos malos ni usar GT para elegir.

## Reprojection error

Después de PnP:

1. reproyectar los cuatro puntos 3D;
2. comparar con las esquinas detectadas;
3. calcular un RMS en píxeles claramente documentado.

Ejemplo:

```text
reprojection_error_px = sqrt(sum((u-u')² + (v-v')²) / 4)
```

El umbral inicial acordado es `3.0 px`. Superarlo crea un rechazo trazable y no
una observación funcional; puede revisarse en una fase futura con mediciones.

## `tag_area_px2`

Se conservará una métrica geométrica sencilla del área proyectada del cuadrilátero detectado.

Sirve para:

- diagnóstico de distancia/tamaño aparente;
- futuras políticas de calidad;
- analizar detecciones muy pequeñas.

No se convierte todavía en un `confidence` ambiguo.

## Intrínsecos y distorsión

El `K` utilizado en PnP debe corresponder exactamente a la imagen que recibe el worker. 4C entrega conjuntamente en su recibo la imagen izquierda efectiva,
`K`, distorsión, dimensiones y estado de rectificación efectivos.

Se verificará:

```text
fx, fy, cx, cy
width, height
rectificación aplicada
resize aplicado
distorsión efectiva
```

Si la imagen ya está rectificada, no se debe volver a aplicar una distorsión original incompatible.

4C debe haber garantizado previamente una única geometría de imagen.

La calibración es propiedad de Dron y procede del perfil local de cámara/ORB.
`GetFiducialConfig` no transporta intrínsecos ni distorsión. `System` aplica una
única rectificación/resize antes de entregar la misma imagen a Tracking y al
detector; una guarda rechaza doble rectificación wrapper+`System`.

## Debug visual fiducial

Parámetros:

```text
debug_fiducial_visualization = false
debug_fiducial_display_seconds = 5.0
```

Cuando está activo, cada wrapper crea un thread visual independiente del worker
de detección y una única ventana con nombre por dron. El worker entrega una
copia anotable del resultado y continúa sin esperar a HighGUI.

Política:

1. dibujar el cuadrilátero de esquinas real, no un bounding box aproximado;
2. mostrar todos los tags decodificados del KF en la misma imagen;
3. usar verde para observaciones válidas;
4. usar rojo y texto de motivo para IDs desconocidos, PnP inválido o rechazo
   por calidad/reproyección;
5. excluir candidatos OpenCV no decodificados porque carecen de `tag_id`;
6. mostrar `tag_id` junto al ROI;
7. mantener la imagen `debug_fiducial_display_seconds` de reloj real;
8. si llega otro KF durante ese intervalo, reemplazar por el más reciente y
   reiniciar el contador;
9. al cerrar, despertar y unir el thread y destruir únicamente su ventana;
10. si HighGUI/display falla, registrar una vez, desactivar este debug y mantener
    SLAM, worker y resultados funcionales.

Con el flag apagado no se crea thread visual, ventana, imagen anotada ni trabajo
específico de HighGUI. Los tags rechazados son solo diagnóstico interno; 4E
publicará exclusivamente observaciones visualmente válidas.

## No bloquear el flujo principal

Regla central de 4D:

```text
GrabStereo no espera Detect()
GrabStereo no espera PnP
GrabStereo no espera a que haya hueco en la cola
GrabStereo no espera indefinidamente al servicio de configuración
```

La única carga añadida al callback cuando hay KF y config `READY` debe ser aproximadamente:

```text
capturar metadatos
clone imagen izquierda
enqueue corto
return al flujo normal
```

La detección y PnP viven en el worker.

## Relación con deltas

El envío de `OrbMap`/deltas y la publicación de pose siguen siendo funciones prioritarias del wrapper.

Una carga alta del detector puede provocar `FID-WORKER-DROP`, pero no debe detener:

```text
tracking
pose local
map delta
servicio de mapa
```

Esto es una decisión explícita de degradación segura.

## 4D no publica todavía el contrato final

El worker produce un resultado interno asociado a la identidad del KF.

No se debe inventar un topic provisional por cada tag.

4E definirá después:

```text
FiducialTagObservation
FiducialKeyFrameObservations
```

y el batch por KF.

Por tanto, 4D se valida mediante logs/tests internos y posteriormente 4E conecta la publicación.

## `orbslam3_msgs` en 4D

Como el servicio de configuración cruza Servidor ↔ Dron, 4D necesita añadir:

```text
FiducialTagConfig.msg
GetFiducialConfig.srv
```

Esto modifica la separación antigua que reservaba todo `orbslam3_msgs` para 4E.

Después de Fase 2 se debe verificar cuántas copias físicas de `orbslam3_msgs` existen. La política vigente de 2G permite exactamente dos copias:

```text
dron/orbslam3_msgs
servidor/orbslam3_msgs
```

Servidor es la copia canónica para la guarda. La sincronización de 4D debe cubrir al menos:

- `FiducialTagConfig.msg`;
- `GetFiducialConfig.srv`;
- todos los demás `.msg/.srv/.action` existentes;
- `rosidl_generate_interfaces`/CMake relevante;
- `package.xml`, versión y auxiliares que afecten al contrato.

Se debe rechazar una tercera copia y cualquier divergencia no documentada. No se añadirán todavía los mensajes de observación de 4E en esta subfase.

## Impacto obligatorio en `system_architecture`

4D cambia simultáneamente una interfaz ROS, una relación cross-group Server→Dron, configuración distribuida y el deployment de Simulación. Por las reglas vigentes de Fase 2, `system_architecture` debe actualizarse en la misma subfase.

Debe reflejar declarativamente, usando **paquetes como nodos principales**:

```text
orbslam3_server / Servidor
    proporciona GetFiducialConfig

orbslam3 (wrapper en dron/orbslam3_ros2) / Dron
    actúa como cliente

orbslam3_msgs
    declara FiducialTagConfig + GetFiducialConfig
```

Metadata mínima:

- nombre del service;
- tipo de service;
- dirección lógica Server→Dron para la respuesta/configuración;
- petición con identidad/trazabilidad del dron;
- datos transportados (`family`, política detector, `tag_id -> size_m`);
- ausencia explícita de `object_id/world_T_object/object_T_tag/GT`;
- relación de deployment Simulación que selecciona el perfil correcto;
- capa `build/API` actualizada para `orbslam3_msgs`;
- capa `config/replica` coherente con ADR 0009.

La arista runtime solo debe iluminarse con evidencia directa de una petición/respuesta real. La telemetría específica debe ser ligera y **completamente dormida** cuando `debug_system_architecture_web`/debug maestro esté desactivado: sin observer, construcción, serialización ni publicación de eventos específicos. No se debe iluminar todavía el topic de observaciones wrapper→Servidor; ese contrato nace en 4E/4F.

## Archivos previstos a modificar

Rutas vigentes orientativas, que se confirmarán inmediatamente antes de editar:

```text
# Dron / interfaces
dron/orbslam3_msgs/msg/FiducialTagConfig.msg
dron/orbslam3_msgs/srv/GetFiducialConfig.srv
dron/orbslam3_msgs/CMakeLists.txt
dron/orbslam3_msgs/package.xml

# Servidor / copia de interfaces si sigue existiendo
servidor/orbslam3_msgs/msg/FiducialTagConfig.msg
servidor/orbslam3_msgs/srv/GetFiducialConfig.srv
servidor/orbslam3_msgs/CMakeLists.txt
servidor/orbslam3_msgs/package.xml

# Servidor de configuración
servidor/orbslam3_server/scripts/fiducial_config_server.py
servidor/orbslam3_server/config/fiducial_objects.yaml
servidor/orbslam3_server/CMakeLists.txt
servidor/orbslam3_server/package.xml
servidor/orbslam3_server/launch/...

# Wrapper
dron/orbslam3_ros2/src/stereo/fiducial_detector.hpp
dron/orbslam3_ros2/src/stereo/fiducial_detector.cpp
dron/orbslam3_ros2/src/stereo/stereo-slam-node.hpp
dron/orbslam3_ros2/src/stereo/stereo-slam-node.cpp
dron/orbslam3_ros2/CMakeLists.txt
dron/orbslam3_ros2/package.xml

# Simulación para pasar perfil y debug
simulacion/simulacion_dron/launch/multi_dron.launch.py
simulacion/simulacion_dron/config/debug.yaml

# Documentación/tests/arquitectura
codex/contexto/04_TOPICS_SERVICES_ACTIONS.md
codex/contexto/03_ARQUITECTURA_ACTUAL.md
codex/contexto/paquetes/orbslam3_msgs/...
codex/contexto/paquetes/orbslam3_ros2/...
codex/contexto/paquetes/orbslam3_server/...
metadata/tests/telemetría de system_architecture
codex/pipeline/fase_4_fiducial_real/...
```

No sobrescribir versiones actuales con snapshots del ZIP.

## Áreas prohibidas en 4D

```text
dron/ORB_SLAM3/src/Tracking.cc para detección visual
dron/ORB_SLAM3/src/System.cc para AprilTag
servidor/orbslam3_multi/ para agrupar objetos
servidor/orbslam3_server/src/global_map_server.cpp para anchor visual todavía
OrbMap.msg para transportar tags
sensor_msgs/Image hacia el servidor
GT como criterio runtime
```

## Cambios requeridos

### Servicio/configuración

1. Crear el contrato `FiducialTagConfig + GetFiducialConfig`.
2. Modificar de forma idéntica las dos copias permitidas de `orbslam3_msgs`, manteniendo Servidor como referencia canónica de la guarda.
3. Extender/ejecutar la guarda de interfaces para `.msg/.srv/.action`, CMake y `package.xml`; rechazar una tercera copia.
4. Crear `fiducial_config_server` como nodo separado.
5. Cargar el perfil `fiducial_objects.yaml` apropiado al deployment y la política detector controlada por Servidor.
6. Extraer/enviar únicamente la información que el Dron necesita: `family`, parámetros detector y `tag_id -> size_m`.
7. No enviar parámetros puramente gráficos de Gazebo.
8. Permitir lista vacía como `disabled/not configured`.
9. Devolver la misma configuración funcional a todos los drones.
10. No implementar hot reload.
11. No permitir que el wrapper lea YAML cross-group.
12. Actualizar `system_architecture` (runtime, build/API, config/replica y deployment) y sus guardas/metadata.
13. Garantizar coste cero de la telemetría específica con debug de arquitectura apagado.

### Worker

14. Crear un único worker por wrapper.
15. Mantener estado de configuración.
16. Reintentar el servicio cada `1 s` por defecto sin bloquear el callback principal.
17. Añadir timeout de petición y recuperación de `in_flight`.
18. No procesar KFs anteriores a `READY`.
19. Crear cola acotada inicialmente a 4.
20. Aplicar `drop oldest` si se llena.
21. Clonar la imagen al encolar.
22. Capturar todos los IDs/epoch/timestamp en el job.
23. Implementar shutdown limpio.

### Detector

24. Mantener `FiducialDetector` separado.
25. Usar `APRILTAG_36H11`.
26. Detectar todos los tags de la imagen.
27. Rechazar IDs desconocidos sin inventar tamaño.
28. Usar `size_m` específico del ID.
29. Usar IPPE Square y examinar candidatos si la API lo permite.
30. Elegir la mejor solución físicamente válida sin GT.
31. Calcular reprojection RMS.
32. Calcular área proyectada.
33. Medir `detect_ms`, `pose_ms`, `total_ms`.
34. Capturar excepciones para que el worker sobreviva y el SLAM continúe.

### Debug visual

35. Declarar el flag y duración acordados y propagarlos hasta cada wrapper.
36. Crear el thread/ventana únicamente con debug activo.
37. Reemplazar por el KF más reciente sin bloquear el worker.
38. Dibujar todos los tags decodificados con color, `tag_id` y motivo acordados.
39. Aislar fallo headless y destruir la ventana con shutdown limpio.

## Logs recomendados

### Configuración

```text
[FID-CONFIG-SERVER-READY]
[FID-CONFIG-SERVED]
[FID-CONFIG-WAIT]
[FID-CONFIG-REQUEST]
[FID-CONFIG-TIMEOUT]
[FID-CONFIG-READY]
[FID-CONFIG-DISABLED]
[FID-CONFIG-ERROR]
```

### Cola/worker

```text
[FID-WORKER-START]
[FID-WORKER-ENQUEUE]
[FID-WORKER-DROP]
[FID-WORKER-PROCESS]
[FID-WORKER-STOP]
```

### Detector

```text
[FID-DETECT]
[FID-POSE]
[FID-REJECT]
```

Los logs deben incluir identidad del KF cuando corresponda, pero no matrices completas salvo debug explícito.

## Pruebas requeridas

### Prueba 1 — Servicio no disponible

Arrancar wrapper sin `fiducial_config_server`.

Esperado:

```text
worker WAIT_CONFIG
reintentos periódicos
TrackStereo continúa
pose continúa
deltas continúan
sin acumulación de imágenes
```

Después arrancar el servicio y comprobar transición a `READY`.

### Prueba 2 — Petición que expira

Simular fallo/ausencia de respuesta.

Esperado:

```text
timeout
in_flight se libera
nuevo intento futuro
sin bloqueo
```

### Prueba 3 — Configuración vacía

Servidor con `objects: []` o cero tags habilitados.

Esperado:

```text
FID-CONFIG-DISABLED
sin detección
SLAM normal
no error fatal
```

### Prueba 4 — Misma configuración para dos drones

Solicitar desde dos wrappers con distinto `drone_id/drone_name`.

Esperado:

```text
lista de tags idéntica
parámetros detector idénticos
trazabilidad distinta en log
```

### Prueba 5 — KF antes de READY

Crear KFs mientras el servicio no está disponible.

Esperado:

```text
no clone
no queue
no reprocesado posterior
```

### Prueba 6 — Tag frontal

Con el escenario 4B, conseguir un KF con un tag frontal y grande.

Esperado:

```text
tag_id correcto
camera_T_tag finita
z > 0
reprojection_error_px finito
area > 0
```

### Prueba 7 — Múltiples tags en un KF

Ver dos caras o dos objetos en la misma imagen.

Esperado:

```text
N observaciones independientes
sin agrupación por object_id
```

### Prueba 8 — Sin tag

KF sin fiduciales visibles.

Esperado:

```text
observations.size() = 0
SLAM/deltas sin cambios
```

### Prueba 9 — ID desconocido

Mostrar un AprilTag 36h11 válido que no aparezca en configuración.

Esperado:

```text
unknown_tag_id
sin PnP inventado
otros tags válidos siguen procesándose
```

### Prueba 10 — Tamaños distintos

Crear una configuración de prueba con al menos dos tags de tamaños físicos distintos, por ejemplo:

```text
tag A -> 0.20 m
tag B -> 0.30 m
```

La geometría en Gazebo debe usar esos tamaños y el servicio debe devolverlos.

El detector debe demostrar mediante logs/tests que selecciona el `size_m` por ID y no un global.

### Prueba 11 — Cola llena

Forzar una condición donde el detector sea más lento que la generación de KFs o reducir temporalmente la capacidad.

Esperado:

```text
queue nunca supera capacidad
se elimina el más antiguo
FID-WORKER-DROP
GrabStereo no espera
```

### Prueba 12 — Flujo de deltas durante carga

Mientras el worker detecta, comprobar que siguen apareciendo los logs/topics de mapa/pose con comportamiento comparable al baseline.

### Prueba 13 — IPPE con candidatos

Usar una pose frontal y otra oblicua. Verificar que se analizan las soluciones disponibles y se elige una pose físicamente válida con mejor reproyección.

No usar GT para escoger la solución runtime. Fase 4 retirará también el uso de
GT fiducial como métrica cuando la cadena visual quede validada en 4H.

### Prueba 14 — Cámara/intrínsecos

Verificar que `K`, dimensiones y distorsión corresponden exactamente a `image_left_clone`.

### Prueba 15 — Guardas de `orbslam3_msgs`

Ejecutar la guarda vigente tras añadir las interfaces. Debe demostrar:

```text
2 copias permitidas
contenido de interfaces idéntico
CMake/package coherentes
0 tercera copia
```

Una divergencia temporal controlada en una copia de test debe ser detectada.

### Prueba 16 — `system_architecture`

Con debug de arquitectura activo:

- metadata del service correcta;
- paquete Servidor y paquete Dron correctamente relacionados;
- una solicitud/respuesta real produce evidencia runtime sin inventar tráfico;
- no aparece todavía la arista de observaciones 4E/4F.

Con todos los flags de `system_architecture` desactivados:

```text
sin observer específico
sin eventos fiduciales de arquitectura
sin serialización/publicación específica
SLAM/config service funcionales independientemente del visualizador
```

### Prueba 17 — Coste

Recoger una muestra suficiente de:

```text
detect_ms
pose_ms
total_ms
queue_depth
drops
```

No fijar todavía un umbral absoluto arbitrario. El criterio es que el tracking/deltas no queden bloqueados y que las métricas permitan ajustar capacidad/parámetros.

## Build previsto

Después de Fase 2 habrá que compilar por los grupos/paquetes realmente afectados.

Como mínimo, conceptualmente:

```text
orbslam3_msgs del dron
orbslam3_msgs del servidor
orbslam3_server
wrapper orbslam3_ros2
simulacion_dron si cambia launch
```

Se usarán los scripts selectivos de Fase 2 y no se reutilizarán comandos antiguos sin verificar.

## Criterio de éxito

4D está realizada solo si:

1. el servicio de configuración funciona;
2. el nodo de configuración es independiente del global map server;
3. dos drones reciben la misma configuración;
4. lista vacía produce `disabled/not configured`, no fallo del SLAM;
5. ausencia del servicio produce reintentos sin bloqueo;
6. una petición colgada tiene timeout y se recupera;
7. los KFs anteriores a READY no se guardan;
8. solo los KFs exactos de 4C pueden encolarse;
9. la imagen se clona antes de salir del callback;
10. la cola es acotada;
11. al llenarse se descarta el trabajo más antiguo;
12. existe un solo worker por dron;
13. `GrabStereo` no ejecuta el detector/PnP de forma síncrona;
14. se detectan `0..N` tags;
15. `size_m` se resuelve por `tag_id`;
16. se obtiene `camera_T_tag` finita y físicamente válida;
17. se calcula reproyección y área;
18. se mide coste;
19. no existe `object_id` ni pose global en el wrapper;
20. no se usa GT runtime;
21. deltas/pose continúan aunque la visión falle o se retrase;
22. el Dron no lee YAML de Servidor/Simulación y la distribución cumple ADR 0009;
23. las dos copias de `orbslam3_msgs` permanecen idénticas y las guardas pasan;
24. `system_architecture` refleja el nuevo service/relación/configuración y su debug-off tiene coste específico cero;
25. no se publica todavía un contrato ad-hoc distinto del que definirá 4E;
26. build, pruebas y logs reales quedan documentados.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: detector bloquea tracking, usa tamaño incorrecto, pierde identidad de KF, cola crece sin límite, servicio puede bloquear indefinidamente o las poses tienen convención incorrecta.
- `PARCIAL`: detección funciona pero falta robustez de servicio/cola/multi-tag/tamaños distintos.
- `BLOQUEADA`: Fase 2 cambia el contrato de mensajes o la estructura del wrapper/servidor de modo que sea necesario revisar esta arquitectura antes de implementar.

## Riesgos

- llamar a `spin_until_future_complete` de una forma incompatible con el executor del wrapper;
- dejar una petición `in_flight` para siempre;
- empezar a clonar KFs antes de tener configuración;
- guardar un `cv::Mat` sin `clone()`;
- leer `map_epoch_` tarde desde el worker;
- cola sin límite;
- hacer `Detect()` dentro de `GrabStereo()` por comodidad;
- usar un único tamaño global;
- mezclar ID visual con `object_id`;
- usar intrínsecos de otra resolución;
- aplicar distorsión original sobre imagen rectificada;
- tomar la primera solución IPPE sin evaluar las demás;
- introducir GT para desambiguar;
- modificar `OrbMap` para transportar observaciones;
- copiar snapshots antiguos de launch/CMake del ZIP y perder cambios de Fase 2/3;
- divergir las dos copias de `orbslam3_msgs`;
- representar en `system_architecture` una arista que todavía no existe o dejar telemetría activa con debug apagado;
- mezclar `surface_offset_m` u otros detalles Gazebo en la configuración enviada al Dron.

## Documentación a actualizar al ejecutar

```text
codex/contexto/04_TOPICS_SERVICES_ACTIONS.md
codex/contexto/paquetes/orbslam3_msgs/...
codex/contexto/paquetes/orbslam3_ros2/...
codex/contexto/paquetes/orbslam3_server/...
codex/contexto/paquetes/simulacion_dron/... si cambia launch
codex/contexto/03_ARQUITECTURA_ACTUAL.md
metadata/tests/telemetría de system_architecture
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4D.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4D_RESUMEN.md
```

## Nota para enlazar con 4E

Al cerrar 4D deben existir resultados internos con identidad completa del KF y `0..N` observaciones.

4E únicamente deberá convertir ese resultado a un batch ROS estable, sin volver a detectar imágenes ni buscar KFs por timestamp.


---

## Decisiones downstream cerradas posteriormente en esta conversación

Las conversaciones posteriores de 4E–4I añaden las siguientes aclaraciones a 4D. No cambian su arquitectura básica, pero deben considerarse parte del contrato definitivo:

1. El resultado interno de 4D debe incluir, para cada tag válido, un `quality_score` junto con métricas originales que permitan explicar/recalcular esa calidad. Como mínimo se conservarán `reprojection_error_px` y `tag_area_px2`; si se usa una métrica de ambigüedad IPPE u otra métrica geométrica, también debe conservarse explícitamente.
2. `quality_score` no puede ser un número opaco. Su fórmula exacta se fijará/ajustará con mediciones, pero deberá estar documentada y ser determinista para una misma detección/configuración.
3. La distancia 1–5 m **no es un filtro de detección del wrapper**. Si una marca es detectada y su PnP es geométricamente válido, 4D conserva la observación aunque luego el servidor decida que no es apta para anchor. Esto permite reutilizar detecciones lejanas en fases futuras de tareas.
4. El servicio de 4D **no enviará** `object_id`, `object_T_tag`, cara, centro del cubo ni pose global del objeto. El dron no necesita esa semántica y no debe conocerla.
5. 4E publicará todos los tags visualmente válidos del KF en un único batch no vacío; 4D no debe preagruparlos ni elegir qué fiducial lógico utilizará finalmente el backend.
6. Si un KF contiene tags de dos o más fiduciales lógicos distintos, 4D no decide entre ellos. Esa selección pertenece a 4G en Servidor.
7. El objetivo sigue siendo no sobrecargar el flujo principal: estas métricas se calculan en el worker y no justifican bloquear `GrabStereo()`.

Estas decisiones son especialmente importantes para conservar la frontera:

```text
Dron/wrapper = percepción geométrica de tags
Servidor      = semántica tag→fiducial, zona segura, fusión y selección funcional
```
