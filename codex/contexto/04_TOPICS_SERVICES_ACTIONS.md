# 04 — Topics, servicios y acciones

## Aclaraciones de interfaces que debe reflejar system_architecture

Interfaces actuales que deben representarse sin inferencias:
- cámaras de Simulación → `orbslam3` por los topics estéreo remapeados;
- `sensor/GT/pose` y `sensor/GT/vel` → `dron_individual` mientras Fase 5 no los retire;
- GT → Servidor únicamente como soporte provisional del fiducial simulado;
- `OrbMap` delta → Servidor;
- `GetOrbMap`: la solicitud va Servidor→wrapper y la respuesta wrapper→Servidor; no
  mezclar service y delta en una única dirección simplificada;
- control Dron→Simulación: topics `motor/*` realmente consumidos por el plugin, no los
  topics internos `control/tray/*`;
- `AccionTrayectoria`: Simulación/runner → `dron_individual`;
- backpressure, nube global, keyframes y observabilidad del Servidor → Simulación cuando
  corresponda.

`/global_mapping/flow_events` es telemetría de `pipeline_flow`, no un bus universal para
inferir cualquier arista de `system_architecture`. Los eventos desconocidos no se
mapean a `server_backend_internal`.

La telemetría arquitectónica futura será ligera y específica, sin payloads de imagen,
nube o mapa.


## Clasificación de estado

- **ACTIVA AHORA**: cámaras→`orbslam3`, GT→`dron_individual` provisional, GT→Servidor fiducial provisional, `OrbMap` delta, `GetOrbMap`, motores Dron→Simulación, `AccionTrayectoria`, backpressure, nube/keyframes y telemetría que el código realmente publique.
- **LEGACY/INFRAESTRUCTURA NO ACTIVA**: contratos históricos no lanzados actualmente.
- **FUTURA/RESERVADA**: interfaces de corrección/pose que Fase 5 reactive o rediseñe.

`system_architecture` solo dibuja como runtime activo la primera categoría.

## Aviso

Este documento resume las conexiones conocidas. Si Codex modifica launches, namespaces o nombres de nodos/topics, debe actualizarlo.

Los nombres pueden ser relativos al namespace de cada dron. Revisar siempre los launch actuales antes de hacer cambios importantes.

## Convenciones de namespace

Los drones suelen usar namespaces tipo:

```text
/dron_1
/dron_2
...
```

## Telemetria de arquitectura

| Topic | Tipo | Descripcion |
|---|---|---|
| `/system_architecture/activity` | `std_msgs/msg/String` | Eventos JSON ligeros y muestreados para aristas runtime conocidas. Solo existe con web y telemetria activas. |
| `/global_mapping/flow_events` | `std_msgs/msg/String` | Eventos internos de `pipeline_flow`; su productor se desactiva si la web esta apagada. |

Los eventos arquitectonicos incluyen `edge_id`, `interface`, `interface_kind`,
`source`, `drone_id` y timestamp. No transportan imagenes, nubes ni mapas.

La acción de trayectoria por dron sigue el patrón:

```text
/dron_X/AccionTrayectoria
```

## Wrapper ORB-SLAM3 por dron

### Entradas

| Topic relativo | Tipo | Descripción |
|---|---|---|
| `camera/left` | `sensor_msgs/msg/Image` | Imagen izquierda estéreo, normalmente remapeada desde la cámara izquierda del dron. |
| `camera/right` | `sensor_msgs/msg/Image` | Imagen derecha estéreo, normalmente remapeada desde la cámara derecha del dron. |

Ejemplo conceptual con namespace:

```text
/dron_1/camera/left
/dron_1/camera/right
```

En la simulación real estos topics pueden estar remapeados desde nombres tipo sensores Gazebo.

### Salidas

| Topic relativo | Tipo | Descripción |
|---|---|---|
| `orbslam/pose_local` | `geometry_msgs/msg/PoseStamped` | Pose local estimada por ORB-SLAM3. |
| `orbslam/orb_map_delta` | `orbslam3_msgs/msg/OrbMap` | Deltas del mapa ORB-SLAM3 local. |

Ejemplo:

```text
/dron_1/orbslam/pose_local
/dron_1/orbslam/orb_map_delta
```

### Servicio

| Servicio relativo | Tipo | Descripción |
|---|---|---|
| `orbslam/get_full_map` | `orbslam3_msgs/srv/GetOrbMap` | Snapshot completo del mapa local. |

Ejemplo:

```text
/dron_1/orbslam/get_full_map
```

## Servidor global

Estado activo desde 3C: `global_map_server` consume los deltas de ambos wrappers
y publica backpressure/telemetría. No crea todavía salidas espaciales para
RViz2. Las interfaces de poses, snapshots, fiduciales y mapa global de las
tablas siguen reservadas para sus subfases propietarias.

### Entradas principales

| Entrada | Tipo | Descripción |
|---|---|---|
| `/dron_X/orbslam/orb_map_delta` | `orbslam3_msgs/msg/OrbMap` | Deltas de mapa local por dron. |
| `/dron_X/orbslam/pose_local` | `geometry_msgs/msg/PoseStamped` | No consumido por el servidor mínimo de `3C`; reservado para fases posteriores. |
| `/dron_X/...GT...` | variable | GT de Gazebo consumido solo para fiducial simulado/debug; no se usa para loops, mapa global ni pose final. |

### Servicios usados

| Servicio | Tipo | Descripción |
|---|---|---|
| `/dron_X/orbslam/get_full_map` | `orbslam3_msgs/srv/GetOrbMap` | Usado desde `3G` para pedir full snapshots y reconciliar KFs/MPs; solo su efecto material se guarda como delta normalizado. |

### Salidas principales

| Salida | Tipo | Descripción |
|---|---|---|
| `/global_mapping/backpressure_active` | `std_msgs/msg/Bool` | Activa en 3C; reliable/transient-local, histéresis por pendientes high=8/low=2. |
| `/global_mapping/flow_events` | `std_msgs/msg/String` | Activa en 3C; metadatos no bloqueantes para el grafo web. |
| `/global_keyframes` | `visualization_msgs/msg/MarkerArray` | Activo en 3F; frustums `LINE_LIST`, color por submapa, IDs estables y `DELETE` para retiradas; reliable/transient-local depth 1. |
| `/global_sparse_cloud` | `sensor_msgs/msg/PointCloud2` | Activo en 3F, frame `world`; campos `x,y,z,score,rgb,drone_id,map_epoch_low,map_epoch_high`; reliable/transient-local depth 1. |
| `/dron_X/map_correction` | `orbslam3_msgs/msg/MapCorrection` | FUTURA/RESERVADA en el runtime actual; Fase 5 decide si se reutiliza. |
| `/dron_X/corrected_keyframes` | `orbslam3_msgs/msg/CorrectedKeyFrameArray` | FUTURA/RESERVADA en el runtime actual; Fase 5 decide si se reutiliza. |
| estado fiducial | `orbslam3_msgs/msg/FiducialLockStatus` | Estado de fiduciales/anclaje. |

## Corrector de pose global retirado

El ejecutable antiguo `global_pose_corrector` fue eliminado en 3T: no se
compilaba, instalaba ni publicaba estas interfaces. Si se recupera esta
capacidad deberá implementarse sobre el backend vigente.

### Entradas

| Topic | Tipo | Descripción |
|---|---|---|
| `/dron_X/orbslam/pose_local` | `geometry_msgs/msg/PoseStamped` | Pose local de ORB-SLAM3. |
| `/dron_X/map_correction` | `orbslam3_msgs/msg/MapCorrection` | Transformación/corrección desde servidor. |
| `/dron_X/corrected_keyframes` | `orbslam3_msgs/msg/CorrectedKeyFrameArray` | KeyFrames corregidos. |

### Salidas

Publica poses globales corregidas de cámara/cuerpo del dron. Los nombres exactos deben comprobarse en código/launch.

## Acción de movimiento de drones

La acción está definida en `dron_individual`:

```text
dron_individual/action/TrayAction
```

Nombre de acción usado por la GUI y por el nodo automático:

```text
AccionTrayectoria
```

Con namespace:

```text
/dron_1/AccionTrayectoria
/dron_2/AccionTrayectoria
```

### Goal

```text
uint8 tipo_trayectoria
geometry_msgs/PoseStamped target_pose
float32 tx
float32 ty
float32 tz
float32 tyaw
bool absoluto_x
bool absoluto_y
bool absoluto_z
bool absoluto_yaw
```

### Result

```text
bool success
float32 t_total
```

### Feedback

```text
float32 t_act
std_msgs/Float64MultiArray x
std_msgs/Float64MultiArray y
std_msgs/Float64MultiArray z
std_msgs/Float64MultiArray yaw
```

## Nodo automático de escenarios

El nodo automático `scenario_runner_node` de `simulacion_dron` lee YAMLs tipo:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

y envía goals a `/dron_X/AccionTrayectoria`, esperando `success=true` antes de continuar según el modo del YAML.

En `3C`, `tray_prueba_4.yaml` se usa también con el mismo nodo para mantener vivo el servidor durante el replay; no envía movimiento real, solo espera.

## Logs útiles de movimiento automático

El nodo de escenario debe emitir logs con prefijo:

```text
SCENARIO-RUNNER
```

para poder filtrarlos en `reduce_simulation_log.sh`.

## Acción de trayectoria usada por pruebas automáticas

Cada dron expone la acción:

```text
/<dron_namespace>/AccionTrayectoria
```

Ejemplos:

```text
/dron_1/AccionTrayectoria
/dron_2/AccionTrayectoria
```

Tipo de acción:

```text
dron_individual/action/TrayAction
```

La GUI de control y el nodo automático scenario_runner_node deben usar esta misma acción.

El launch principal de simulación es:

```txt
ros2 launch simulacion_dron multi_dron.launch.py
```
