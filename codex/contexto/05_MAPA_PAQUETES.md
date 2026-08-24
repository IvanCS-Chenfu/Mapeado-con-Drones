# 05 — Mapa de paquetes

## Propósito

Índice rápido de paquetes y responsabilidades. La documentación detallada vive en:

```text
codex/contexto/paquetes/<paquete>/00_summary.md
```

Si se modifica un paquete, actualizar también su documentación.

Nota: para buscar un paquete concreto sin abrir muchos archivos, usar:

```bash
python3 codex/herramientas/find_context.py <query>
```

## Paquetes principales

| Paquete | Grupo | Estado | Rol actual |
|---|---|---|---|
| `orbslam3_msgs` | Dron y Servidor | estable/replicado | Contrato ROS 2; Servidor es canónico y Dron réplica exacta. |
| `orbslam3_ros2` | Dron | estable | Wrapper estéreo ORB-SLAM3. Publica pose local, `OrbMap` delta y `GetOrbMap`. |
| `orbslam3_multi` | Servidor | activo | Backend raw/poses/score/fusión/optimización/builder. |
| `orbslam3_server` | Servidor | activo | Workers, replay/backpressure y publicación global. |
| `dron_individual` | Dron | activo | Control por dron y acción `AccionTrayectoria`. |
| `simulacion_dron` | Simulación | activo | Gazebo, integración, escenarios, RViz2 y dos visualizadores web. |
| `lib_tray` | Dron | activo | Generación de trayectorias. |
| `ORB_SLAM3` | Dron | externo/modificación local justificada | Frontend visual local, tracking, KFs, MapPoints, BoW y covisibilidad. |

## Rutas fisicas vigentes

```text
src/dron/ORB_SLAM3
src/dron/dron_individual
src/dron/lib_tray
src/dron/orbslam3_ros2
src/dron/orbslam3_msgs

src/servidor/orbslam3_multi
src/servidor/orbslam3_server
src/servidor/orbslam3_msgs

src/simulacion/simulacion_dron
```

Los comandos de build seleccionan un grupo con `--group` y no realizan un
descubrimiento global de `src/`, porque existen dos paquetes llamados
`orbslam3_msgs`.

## Separación conceptual de Fase 3

### Servidor ROS 2

```text
orbslam3_server
```

Debe tender a:

- recibir y publicar ROS 2;
- convertir mensajes;
- leer GT solo para fiducial simulado/debug;
- delegar lógica en `orbslam3_multi`.

### Backend algorítmico

```text
orbslam3_multi
```

Debe tender a contener:

- `RawMapDatabase`;
- `SparseGlobalBackend`;
- `GlobalPoseStore`;
- `FiducialAnchorManager`;
- `LoopDetector`;
- `SubcloudLoopVerifier`;
- `LoopDecisionManager`;
- `FusionManager` / `FusedLandmarkManager`;
- `LandmarkScoreManager`;
- `PoseGraphBuilder`;
- `OptimizationManager`;
- `GlobalMapBuilder`.

### Dron/wrapper

```text
orbslam3_ros2
orbslam3_msgs
dron_individual
lib_tray
```

No rediseñar durante Fase 1 salvo necesidad justificada por una subfase.

### Simulación

```text
simulacion_dron
```

Proporciona Gazebo, GT permitido para fiduciales/debug y el launch oficial:

```bash
ros2 launch simulacion_dron multi_dron.launch.py
```

## Paquetes legacy o de bajo interés

| Paquete | Regla |
|---|---|
| `mi_tfg` | No usar salvo petición explícita. |

`ORB_SLAM3_MULTI/` fue retirado completamente en la correccion final de 3T.

Más detalles:

```text
codex/contexto/paquetes/paquetes_legacy.md
```

## Separacion completada

Fase 2 materializo la separacion. Las fases posteriores deben conservar estos
grupos, actualizar `system_architecture` cuando cambien interfaces y ejecutar
la guarda arquitectonica antes del cierre.
