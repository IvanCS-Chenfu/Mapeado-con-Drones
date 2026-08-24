# 05 — Mapa de paquetes

## Grupos actuales

### Dron

| Paquete | Rol |
|---|---|
| `dron_individual` | Trayectoria, control y mezcla a motores. |
| `lib_tray` | Generadores de trayectoria. |
| `ORB_SLAM3` | Frontend visual local. |
| `orbslam3` / fuente `orbslam3_ros2` | Wrapper ROS 2: cámaras, pose local, mapa delta y snapshot. |
| `orbslam3_msgs` | Réplica del contrato ROS; Servidor es canónico. |

### Servidor

| Paquete | Rol |
|---|---|
| `orbslam3_msgs` | Copia canónica del contrato ROS 2. |
| `orbslam3_multi` | Backend raw/global, loops, fusión, score y optimización. |
| `orbslam3_server` | Adaptador/orquestador ROS 2, colas, publicación y replay. |

### Simulación

| Paquete | Rol |
|---|---|
| `simulacion_dron` | Gazebo, escenarios, plugins, launch de integración, RViz y visualizadores web. |

`mi_tfg` permanece legacy y fuera de los builds activos.

## Relación con system_architecture

`system_architecture` usa paquetes como nodos principales. Ejecutables, nodos ROS, librerías, YAML y responsabilidad aparecen como metadata/tooltip/panel.

La lista de paquetes no se valida con números mágicos. La policy, el descubrimiento real y el grafo deben coincidir. Si una fase futura añade/elimina/mueve un paquete, esa misma subfase actualiza mapa/policy, grafo, guardas/tests y documentación.

## Fronteras

- Dron no depende de paquetes de Servidor/Simulación.
- Servidor no depende de paquetes de Dron/Simulación.
- Simulación integra explícitamente Dron y Servidor.
- En Fase 5 la única frontera Servidor↔Dron permitida termina en `orbslam3`; no hay conexión directa a `dron_individual`.
