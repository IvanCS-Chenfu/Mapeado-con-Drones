# Subfase 1E — Sensores simulados: Ground Truth y cámaras

## Estado

```text
realizado
```

## Dependencias

```text
1B — links del cuerpo y cámaras
1C — modelo insertado y namespaces
```

## Objetivo técnico

Integrar en el Xacro:

1. el plugin propio `plugin_sensor_groundtrurh`, que publica pose, velocidad y aceleración del link `cuerpo`;
2. el plugin estándar `libgazebo_ros_camera.so`, que publica imágenes y `camera_info` para configuración mono o estéreo.

Comprobar los topics de GT y visualizar las imágenes en RViz2. El GT se acepta en la Fase 1 como estado para el control, pero deberá eliminarse como entrada funcional en la **Fase 5**.

## Documentos de detalle

```text
subfase_1E_especificacion.md
subfase_1E_testing.md
```

Ambos forman parte del contrato de `1E`; no son subfases nuevas.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/pipeline/fase_1_control_dron/pipeline_fase_1_RESUMEN.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1B.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1C.md
codex/contexto/paquetes/simulacion_dron/00_summary.md
codex/contexto/paquetes/dron_individual/00_summary.md
```

## Diagnóstico de partida

El código de referencia ya contiene:

```text
src/simulacion_dron/src/plugins/plugin_sensor_groundtrurh.cpp
src/simulacion_dron/urdf/dron_plugins.xacro
src/dron_individual/config/hardware.yaml
```

El plugin GT usa tiempo de simulación, `WorldPose`, `WorldLinearVel` y `WorldAngularVel`; deriva aceleraciones numéricamente. Las cámaras se crean mediante macros Xacro y el plugin estándar de Gazebo ROS.

## Alcance

Incluye:

- configuración física de cámaras;
- mono frente a estéreo;
- frecuencia y visualización;
- topics `image_raw` y `camera_info`;
- GT de pose, velocidad y aceleración;
- frames y timestamps;
- aislamiento multi-dron;
- visualización en RViz2.

No incluye:

- ORB-SLAM3;
- calibración real;
- estimación de pose sin GT;
- nube de profundidad o nube densa;
- sustitución de GT en el control.

## Archivos permitidos a modificar

```text
src/simulacion_dron/src/plugins/plugin_sensor_groundtrurh.cpp
src/simulacion_dron/urdf/dron_plugins.xacro
src/dron_individual/config/hardware.yaml
src/simulacion_dron/rviz/*.rviz
src/simulacion_dron/CMakeLists.txt
src/simulacion_dron/package.xml
codex/contexto/paquetes/simulacion_dron/
codex/contexto/paquetes/dron_individual/
```

## Archivos prohibidos

```text
src/dron_individual/src/control_tray/
src/lib_tray/
src/orbslam3_multi/
src/orbslam3_server/
ORB_SLAM3/
orbslam3_ros2/
```

## Cambios requeridos

Los cambios completos, interfaces y validaciones se definen en `subfase_1E_especificacion.md`. Las pruebas, logs y criterios se definen en `subfase_1E_testing.md`.

## Cambios prohibidos

- No usar GT para validar online el mapa sparse.
- No presentar GT como sensor disponible en hardware real.
- No eliminar GT todavía del control; esa migración pertenece a Fase 5.
- No escribir un plugin de cámara propio si `libgazebo_ros_camera.so` cubre el contrato.
- No mezclar imágenes de drones distintos fuera de namespace.
- No afirmar estéreo calibrado solo porque existen dos topics.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh simulacion_dron dron_individual
```

## Criterio de éxito resumido

`1E` queda conseguida cuando:

1. GT publica pose, velocidad y aceleración con timestamps de simulación y frame documentado;
2. la configuración mono publica una cámara;
3. la configuración estéreo publica izquierda y derecha;
4. `image_raw` y `camera_info` son visibles en RViz2;
5. dos drones mantienen topics separados;
6. todos los parámetros proceden de YAML/Xacro;
7. queda explícito que GT se retirará del control en Fase 5.

## Documentación a actualizar al ejecutar

```text
codex/contexto/paquetes/simulacion_dron/
codex/contexto/paquetes/dron_individual/
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1E.md
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1E_RESUMEN.md
```
