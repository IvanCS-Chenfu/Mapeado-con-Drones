# Paquetes legacy

## Paquetes legacy o de pruebas anteriores

| Paquete | Estado | Regla |
|---|---|---|
| `mi_tfg` | legacy/bajo interés | No modificar salvo petición explícita. |
| `ORB_SLAM3_MULTI` | legacy/bajo interés | No modificar salvo petición explícita. |

El pipeline activo usa:

- `dron_individual`
- `lib_tray`
- `simulacion_dron`
- `orbslam3_msgs`
- `orbslam3_ros2`
- `orbslam3_multi`
- `orbslam3_server`
- `ORB_SLAM3` como librería externa/base

## Regla para Codex

No extender estos paquetes salvo que una fase activa lo exija explícitamente.
Las rutas legacy internas de Fase 3 ya fueron retiradas en 3T; su evidencia
cronológica permanece únicamente en los historiales y en Git.
