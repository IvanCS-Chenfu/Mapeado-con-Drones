# 03 — Arquitectura actual y objetivo de Fase 3

<!-- ACUERDOS_CIERRE_F2_2026_08_24_START -->
## Lectura correcta de la arquitectura actual y sus deudas programadas

> **Vigencia:** acuerdo cerrado el 2026-08-24. Este bloque prevalece sobre cualquier
> frase anterior incompatible del mismo documento. No borra ni reescribe evidencia
> histórica; distingue siempre entre estado actual, deuda conocida y arquitectura objetivo.

La arquitectura documentada debe separar **lo que existe hoy** de lo previsto en Fases
4 y 5.

Estado funcional actual relevante:
- Simulación produce cámaras estéreo para `orbslam3`.
- `dron_individual` todavía consume `sensor/GT/pose` y `sensor/GT/vel` para trayectoria
  y control. Es una dependencia provisional conocida, no la arquitectura final.
- el wrapper `orbslam3` intercambia el mapa ORB con Servidor;
- Servidor mantiene el fiducial simulado basado en GT como mecanismo provisional;
- Dron publica comandos por motor hacia los plugins de Simulación;
- `pipeline_flow` observa el pipeline interno de mapa;
- `system_architecture` debe representar paquetes y comunicaciones reales vigentes.

Fase 4 sustituirá la ruta funcional del fiducial GT por observaciones visuales asociadas
al KF exacto. Fase 5 retirará GT del control y fijará que la comunicación cross-group
Servidor↔Dron pase por `orbslam3`; `dron_individual` no abrirá una conexión directa con
Servidor.

No representar como activa una arista futura solo porque esté planificada.
<!-- ACUERDOS_CIERRE_F2_2026_08_24_END -->

## Resumen

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
