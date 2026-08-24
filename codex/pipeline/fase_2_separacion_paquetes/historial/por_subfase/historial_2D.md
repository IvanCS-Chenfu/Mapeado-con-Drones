# Historial 2D

## 2026-08-24 - Prueba 199, smoke debug-off

- YAML: `smoke_debug_desactivado.yaml`;
- resultado: 5/5 pasos, 4/4 goals, `success=true` y exit 0;
- evidencia negativa: RViz/web 0 MiB y sin marcadores de debug específico;
- recursos: guarda no disparada;
- conclusión: `CONSEGUIDA`.

## 2026-08-24 - Prueba 200, vuelta oficial

- YAML: `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
- resultado: 14/14 pasos, 20/20 goals, `success=true` y exit 0;
- debug: RViz2, ambos bridges, navegadores, telemetría y logs F3 activos;
- recursos: guarda no disparada, mínimo 3873.8 MiB disponibles;
- cleanup: traceback de `gui_tray_multi` y Gazebo 255 después de `SIM-DONE`;
- revisión humana: el usuario confirmó el 2026-08-24 que la simulación y sus
  vistas salieron correctamente;
- conclusión revisada: `CONSEGUIDA`.
