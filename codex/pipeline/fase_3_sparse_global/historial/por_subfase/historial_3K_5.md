# Historial 3K.5 - Controles fiduciales de cola pendiente

## 2026-07-28 - Corrección del tramo llegado durante el solver

### Diagnóstico

En `prueba_42`, el solver largo del dron antihorario cerró su grafo en un KF
fiducial mientras varios KFs nuevos aparecían durante la optimización. El apply
los proyectaba rígidamente desde el target original. Aunque la ventana del HTML
era correcta, unos pocos KFs posteriores podían quedar visualmente fuera de la
trayectoria correcta en RViz2.

### Implementación

- `OptimizationApplyKeyFrameRecord` conserva poses `before/after` para
  diagnóstico post-commit.
- `global_map_server` busca en el journal observaciones del mismo submapa y
  fiducial con `arrival_id > task.created_arrival_id` y KF posterior al target.
- `ApplyCandidateResult` usa cada observación como
  `PendingTailFiducialConstraint`.
- Los KFs entre controles reciben una corrección SE(3) interpolada por distancia
  acumulada de trayectoria.
- Los controles y KFs intermedios quedan aceptados; solo la cola posterior al
  último control permanece `derived_tail`.
- `active_tail_anchor` avanza al último control.
- El HTML 3D se reexporta después del apply e incluye el tramo post-target.

No se modifica `RawMapDatabase` ni se usa el almacén GT de diagnóstico. En
simulación, el journal fiducial contiene la medición absoluta simulada que ya
usa la ruta funcional.

### Build y test

```text
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
BUILD-EXIT-CODE 0
test_global_pose_store_tail_anchor: exit code 0
```

El test reproduce target `KF1 -> 10 m`, control pendiente `KF3 -> 12 m` y
comprueba `KF2 -> 11 m`, `KF4 -> 13 m`, un control, dos KFs refinados, un KF
derivado y `KF3` como anchor activo.

### Prueba live

```text
prueba_43
YAML: prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
SCENARIO-RUNNER-DONE success=true
SIM-DONE success=true
SIM-EXIT-CODE 0
```

En `task_id=2`, dron 2:

```text
graph_target_kf=243
solver=27201.861 ms
pending controls=KF245,KF246,KF247,KF250
refined_kfs=7
derived_kfs=1
active_ref_kf=250
```

El commit cubrió `KF244-251`. Los cuatro controles quedaron con
`error_t=0/error_yaw=0`; el target pasó `20.430510 -> 0 m`. La validación
aceptó el resultado con cero aristas fuertes o deformables rotas,
`raw_db_modified=false` e `invalid_pose_skipped=0`.

La nube se mantuvo 300 s con:

```text
points=64335
server_corrected_points=48014
server_corrected_missing_keyframe_skipped=0
bad_skipped=0
invalid_pose_skipped=0
```

### Artefactos

```text
codex/archivos_auxiliares/logs/prueba_43.log
codex/archivos_auxiliares/logs/prueba_43.reduced.log
codex/archivos_auxiliares/logs/prueba_43.index.md
codex/archivos_auxiliares/html/f3l_debug_animation_task_2.html
codex/archivos_auxiliares/repeticiones/f3i_window_task_2.tsv
codex/archivos_auxiliares/repeticiones/f3l_graph_task_2.tsv
```

El HTML de `task_id=2` contiene 95 KFs e incluye explícitamente `KF243-251`.
El dump original vuelve a pasar `test_opt_graph_offline`.

### Limitaciones

- El dron 1 volvió a perder tracking y su optimización ocurrió en `epoch=1`.
- Las dos ventanas fueron disjuntas; la espera/recheck por solape sigue sin
  evidencia live.
- Falta inspección del usuario en RViz2 de `prueba_43`.

### Conclusión

La corrección del tail llegado durante un solver largo queda validada por test
y logs live. `3K` continúa `PARCIAL/REABIERTA` por las limitaciones anteriores.
