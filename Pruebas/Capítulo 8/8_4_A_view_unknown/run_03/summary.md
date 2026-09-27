# Prueba 8.4-A - VIEW_UNKNOWN

Resultado: **CONSEGUIDA**.

- Wrapper y escenario: PASS (`success=true`). D1 llegó con GT al fiducial 2 y el handoff autónomo general quedó desactivado.
- El escenario invocó `/mission/test_view_unknown_right`. El servidor registró `yaw_before=90.00 deg`, `yaw_target=0.00 deg`; `task_manager` ejecutó `LOOK_AND_CAPTURE` correlacionado `autonomous:test_view_unknown_right:1:1`.
- Giro medido por NavigationState: `90.0000 -> -0.0009 deg`, delta `-90.0009 deg`.
- El resultado local fue `look_completed` con una observación depth. El servidor clasificó la integración como `kind=vista_unknown`.
- Fuentes depth exactas del comando: `free=1`, `direct_free=0`, `occupied=0`. La continuación del workflow de prueba se suprimió tras materializarse, por lo que no se ejecutó un `VIEW_ADVANCE` que contaminase la evidencia.
- El coverage permaneció sin cambios y con progreso cero. En el VoxelMap global `FREE` aumentó de `6073` a `9331`; el total `OCCUPIED` también cambió por materialización sparse concurrente, pero no procede de la captura depth: su contador de fuentes `occupied=0` lo descarta.

Artefactos: `raw/before_view_unknown.json`, `raw/after_view_unknown.json`, `processed/view_unknown_summary.json`, `processed/view_unknown_metrics.csv` y el log reducido `codex/archivos_auxiliares/logs/prueba_c8_4_a_view_unknown_run_03.reduced.log`.
