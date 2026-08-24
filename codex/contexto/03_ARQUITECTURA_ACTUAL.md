# 03 — Arquitectura actual

## Distribucion fisica vigente

Desde Fase 2, los paquetes activos se distribuyen asi:

```text
dron:       ORB_SLAM3, dron_individual, lib_tray, orbslam3, orbslam3_msgs
servidor:   orbslam3_multi, orbslam3_server, orbslam3_msgs canonico
simulacion: simulacion_dron
```

Dron y Servidor compilan sin descubrir el otro grupo. Simulacion integra ambos
prefijos. La comunicacion funcional sigue siendo ROS 2 directa; las dos
instalaciones de `orbslam3_msgs` son replicas exactas.

La observabilidad se divide en `pipeline_flow`, para el pipeline interno, y
`system_architecture`, para paquetes e interfaces. Ninguna gobierna el sistema.

## Resumen de Fase 3

El objetivo es construir un mapa sparse global multi-dron a partir de mapas
locales ORB-SLAM3 ejecutados en cada dron.

3B congeló temporalmente el runtime anterior y 3T lo retiró después de validar
su sustitución. El runtime actual ya integra la ruta completa 3C-3R:
`orbslam3_server` consume deltas mediante una FIFO y un worker principal único,
mientras `orbslam3_multi` mantiene autoridades separadas para raw, poses,
score, fusión y vista pública.

## Flujo de alto nivel

```text
Gazebo
  -> drones simulados
  -> cámaras estéreo
  -> wrapper ORB-SLAM3 por dron
  -> OrbMap delta / pose_local
  -> GlobalMapServer -> PrimaryQueue -> PrimaryWorker -> RawMapDatabase

Ruta derivada vigente:
  raw -> poses/anchors -> score/fusion -> builder -> loops/optimizacion
```

## `orbslam3_server`

En el estado activo validado de 3C, `orbslam3_server` declara subscriptions
`OrbMap`, telemetría de flujo y backpressure. Los callbacks solo validan,
encolan y retornan; `PrimaryWorker` delega el commit raw en `orbslam3_multi`.

En fases posteriores debe crecer de forma controlada para:

- declarar parámetros;
- suscribirse a `orbslam/orb_map_delta`;
- pedir `orbslam/get_full_map`;
- recibir `orbslam/pose_local`;
- leer GT solo para fiducial simulado/debug;
- convertir mensajes ROS a estructuras internas;
- llamar a `orbslam3_multi`;
- publicar nube global, `MapCorrection`, `CorrectedKeyFrameArray`, estado fiducial y topics debug.

No debe volver a concentrar de forma permanente:

- base de datos raw;
- BoW pesado;
- subnubes/RANSAC;
- decisión de loops;
- fusión y score;
- construcción de grafos;
- optimización;
- rollback.

## `orbslam3_multi`

Actualmente contiene `RawMapDatabase` y sus tipos/tests de 3C. Debe seguir
creciendo como backend algorítmico reutilizable, sin poseer ROS topics.

Componentes objetivo:

| Componente | Responsabilidad |
|---|---|
| `RawMapDatabase` | Guardar datos ORB-SLAM3 crudos, deltas, snapshots y replay. |
| `GlobalPoseStore` | Guardar anchors, poses globales, poses optimizadas, propagadas y correcciones heredables. |
| `FiducialAnchorManager` | Anclar submapas en primera visita y crear tareas por error en visitas posteriores. |
| `LoopDetector` | Generar candidatos BoW amplios sin confirmar loops. |
| `SubcloudLoopVerifier` | Verificar candidatos con subnubes, matching ORB, reducción espacial y RANSAC. |
| `LoopDecisionManager` | Decidir `REJECT`, `HOLD`, fusión u optimización por loop. |
| `FusionManager` / `FusedLandmarkManager` | Crear y actualizar tracks fused sin borrar raw. |
| `LandmarkScoreManager` | Centralizar score raw y fused mediante eventos semánticos. |
| `PoseGraphBuilder` | Construir grafos temporales para fiduciales y loops. |
| `OptimizationManager` | Dry-run, apply útil, validación, rollback e integración con `GlobalPoseStore`. |
| `GlobalMapBuilder` | Construir nube sparse global publicable desde poses globales y fused tracks. |

## Separación de datos

### Datos raw

`RawMapDatabase` conserva:

- submapas por `(drone_id, map_epoch)`;
- KeyFrames;
- MapPoints;
- poses locales;
- BoW/FeatureVector;
- descriptores;
- observaciones;
- covisibilidad;
- spanning tree;
- loop edges locales;
- `arrival_id`;
- deltas/full snapshots para replay.

Regla: no se modifica por optimización ni por fusión.

### Estado global

`GlobalPoseStore` conserva:

- `world_T_local`;
- anchors;
- estado de submapa;
- KFs realmente fiduciales/hard-fixed;
- poses globales optimizadas;
- poses propagadas;
- KFs rebasados;
- correcciones heredables.

Regla: si existe pose optimizada para un KF, esa pose prevalece frente a full snapshots posteriores. Si no existe, se deriva desde `world_T_local * local_T_kf`.

### Datos publicables

`GlobalMapBuilder` usa:

```text
RawMapDatabase + GlobalPoseStore + FusedLandmarkManager + LandmarkScoreManager
```

Debe evitar publicar duplicados raw cuando un punto pertenece a un fused track publicable.

## Fiduciales

El servidor detecta fiducial simulado con GT y entrega la observación al backend. El backend decide:

- primera visita: ancla submapa;
- visita posterior: mide error absoluto;
- error alto: crea `FiducialOptimizationTask`.

El fiducial no es un loop.

## Loops

El flujo objetivo es:

```text
LoopDetector
  -> candidatos BoW
SubcloudLoopVerifier
  -> query_subcloud / candidate_subcloud / matching / RANSAC
LoopDecisionManager
  -> reject / hold / fusion / loop optimization task
```

BoW no confirma loops. La confirmación exige evidencia geométrica.

## Optimización

`PoseGraphBuilder` construye grafos temporales. `OptimizationManager` ejecuta dry-run y solo aplica si es útil. El apply escribe en `GlobalPoseStore`, nunca en `RawMapDatabase`.

Después del apply se recalcula el error real. Si falla, rollback restaura poses, correcciones, scores y fused tracks afectados.

## Simulación

El launch oficial es:

```bash
ros2 launch simulacion_dron multi_dron.launch.py
```

`run_simulation.sh` usa YAMLs:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_X.yaml
```

Los logs completos se guardan como `prueba_X.log` y los reducidos como `prueba_X.reduced.log`.

En `3C`, la simulacion validada usa `tray_prueba_1.yaml` para generar deltas reales y `tray_prueba_4.yaml` para replay sin Gazebo. No se espera nube global ni RViz2 util.

## Relación con fases futuras

Fase 3 debe producir:

- mapa sparse global fused;
- submapas coherentes;
- anchors;
- poses corregidas;
- corrected keyframes;
- landmarks globales con score;
- observabilidad suficiente para depurar.

Esto alimenta la separación física de paquetes de Fase 2 y las fases posteriores de fiducial real, pose global sin GT y nube densa.
