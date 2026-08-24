# Resultado final de Fase 2

## Conclusión

```text
Fase 2: CONSEGUIDA
Fecha: 2026-08-24
Revisión visual humana de prueba 200: confirmada correcta por el usuario
Cierre documental y visual: completado
```

## Entrega

- grupos físicos `dron/`, `servidor/` y `simulacion/` con 5, 3 y 1 paquetes;
- `orbslam3_msgs` canónico en Servidor y réplica exacta en Dron;
- `build/install/log` separados por grupo y build de un paquete por invocación;
- YAML con ownership de ADR 0009, réplicas parciales, `deployment profile`
  `global_map` controlado y reloj explícito;
- ORBvoc completo preparado fuera de `src/` e instalado por `dron_individual`;
- `pipeline_flow` con generación de eventos desactivable antes de serializar;
- `system_architecture` separado, declarativo, estático y live sobre su topic propio;
- guarda `check_workspace_architecture.py` para estructura, interfaces, dependencias,
  configuración, rutas, visualizadores y documentación.

## Validación

- build limpio: 9/9 invocaciones, un paquete por invocación;
- CTest: `lib_tray` 4/4, `orbslam3_multi` 9/9,
  `orbslam3_server` 10/10 y `simulacion_dron` 9/9;
- `dron_individual`: archivos tocados correctos y rebuild correcto; su suite global
  conserva deuda legacy en `flake8`, `pep257` y C++ históricos no modificados;
- prueba 199: 5/5 pasos, 4/4 goals, debug específico inactivo y guarda de recursos
  no disparada;
- visualizadores aislados: HTTP/health correctos, modo estático sin telemetría y
  modo live con evento ROS real aceptado;
- prueba 200: 14/14 pasos, 20/20 goals, ambos bridges y RViz2 activos,
  `success=true`, `SIM-EXIT-CODE=0` y guarda no disparada.
- cierre visual: rebuild aislado de `simulacion_dron`, CTest 9/9, guarda 15/15
  y capturas 1440x900 y 820x1000 inspeccionadas sin solapes.

## Incidencias conservadas

Después de `SIM-DONE` en la prueba 200, `gui_tray_multi` emitió un traceback de
shutdown de `rclpy` y Gazebo terminó con el `exit 255` conocido. No afectaron al
escenario. Ambos bridges web, RViz2, wrappers y servidor cerraron limpiamente;
el antiguo `ValueError` de `system_architecture_bridge` no se reprodujo.

## Evidencia

```text
codex/archivos_auxiliares/logs/prueba_199.reduced.log
codex/archivos_auxiliares/logs/prueba_199.resources.summary
codex/archivos_auxiliares/logs/prueba_200.reduced.log
codex/archivos_auxiliares/logs/prueba_200.resources.summary
codex/archivos_auxiliares/system_architecture_layout_desktop.png
codex/archivos_auxiliares/system_architecture_layout_narrow.png
codex/pipeline/fase_2_separacion_paquetes/historial/INDEX.md
```
