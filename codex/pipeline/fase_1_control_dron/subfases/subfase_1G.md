# Subfase 1G — Generación y control de trayectoria del dron

## Estado

```text
realizado
```

## Dependencias

```text
1D — actuación por motor
1E — pose y velocidad GT disponibles
1F — lib_tray validada
```

## Objetivo técnico

Crear el control completo en lazo cerrado para un cuadricóptero de cuatro motores:

```text
TrayAction / gen_tray
  -> feedback temporal
  -> control_calcular_fuerzas
  -> control/tray/fuerza + control/tray/torque
  -> aplicar_fuerzas_dron
  -> cuatro topics motor/*
  -> plugin_actuar_motores
  -> Gazebo
```

El estado actual se toma de GT. Esta dependencia es aceptada en Fase 1 y deberá sustituirse en Fase 5.

## Documentos de detalle

```text
subfase_1G_especificacion.md
subfase_1G_testing.md
```

Ambos pertenecen a `1G`.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/pipeline/fase_1_control_dron/pipeline_fase_1_RESUMEN.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1D.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1E.md
codex/pipeline/fase_1_control_dron/subfases/subfase_1F.md
codex/contexto/paquetes/dron_individual/00_summary.md
codex/contexto/paquetes/lib_tray/00_summary.md
codex/contexto/paquetes/simulacion_dron/00_summary.md
```

## Diagnóstico de partida

La implementación de referencia contiene:

```text
src/dron_individual/action/TrayAction.action
src/dron_individual/src/control_tray/gen_tray.cpp
src/dron_individual/src/control_tray/control_calcular_fuerzas.cpp
src/dron_individual/src/control_tray/aplicar_fuerzas_dron.cpp
src/dron_individual/config/tray_dron.yaml
src/dron_individual/launch/generar_dron.launch.py
```

- `gen_tray` ofrece `AccionTrayectoria`, espera una muestra nueva de GT de pose/velocidad y publica feedback a 30 Hz.
- `control_calcular_fuerzas` se ejecuta cada 20 ms y publica esfuerzo total.
- `aplicar_fuerzas_dron` se ejecuta cada 20 ms y resuelve una matriz 4×4.
- El mixer solo implementa cuatro motores.
- No existen saturaciones físicas explícitas en el baseline.

## Alcance

Incluye:

- contrato de `TrayAction`;
- pol3, veltrap y elipse;
- objetivos absolutos/relativos;
- feedback de posición y derivadas;
- controlador geométrico;
- mixer de cuatro motores;
- launch por namespace;
- cancelación y sustitución de goals;
- pruebas de vuelo simulado.

No incluye:

- control de 6/8 motores;
- pose sin GT;
- autopiloto real;
- planificación multi-dron o anticolisión;
- GUI operacional de Fase 7.

## Archivos permitidos a modificar

```text
src/dron_individual/action/TrayAction.action
src/dron_individual/src/control_tray/gen_tray.cpp
src/dron_individual/src/control_tray/control_calcular_fuerzas.cpp
src/dron_individual/src/control_tray/aplicar_fuerzas_dron.cpp
src/dron_individual/config/tray_dron.yaml
src/dron_individual/config/hardware.yaml
src/dron_individual/launch/generar_dron.launch.py
src/dron_individual/CMakeLists.txt
src/dron_individual/package.xml
src/lib_tray/
src/simulacion_dron/src/control_tray/scenario_runner_node.cpp
codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/lib_tray/
codex/contexto/paquetes/simulacion_dron/
```

## Archivos prohibidos

```text
src/orbslam3_multi/
src/orbslam3_server/
src/orbslam3_msgs/
ORB_SLAM3/
orbslam3_ros2/
```

## Cambios requeridos

La especificación de interfaces, YAML y comportamiento se encuentra en `subfase_1G_especificacion.md`. Las pruebas y criterios completos se encuentran en `subfase_1G_testing.md`.

## Cambios prohibidos

- No sustituir GT en esta fase.
- No usar GT fuera del control simulado y sus métricas.
- No afirmar control funcional para seis u ocho motores.
- No cambiar ecuaciones o semántica de `TrayAction` sin acuerdo funcional.
- No corregir estabilidad aumentando fuerzas o ganancias sin prueba controlada.
- No lanzar un nuevo thread por goal sin preservar seguridad de cancelación/vida del nodo.
- No aceptar NaN/Inf o tiempos inválidos silenciosamente.

## Criterio de éxito resumido

El cuadricóptero debe ejecutar hover y trayectorias básicas con goals válidos, cancelación y reemplazo controlados, configuración física coherente y ausencia de fallos graves. Las pruebas obligatorias se detallan en `subfase_1G_testing.md`.

## Documentación a actualizar al ejecutar

```text
codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/lib_tray/
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1G.md
codex/pipeline/fase_1_control_dron/historial/por_subfase/historial_1G_RESUMEN.md
```
