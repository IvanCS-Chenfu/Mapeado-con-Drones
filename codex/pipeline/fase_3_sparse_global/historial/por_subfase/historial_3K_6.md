# Historial 3K.6 - KFs tardíos dentro de ventana

## 2026-07-28 - Reconciliación en commit y `prueba_44`

- objetivo intentado: evitar que un `live_full_snapshot` deje sin corregir KFs
  insertados durante el solver con IDs anteriores al target.
- archivos modificados: `optimization_result.hpp`,
  `optimization_manager.cpp`, `test_global_pose_store_tail_anchor.cpp` y
  `global_map_server.cpp`.
- implementación: el commit vuelve a consultar el intervalo del grafo, detecta
  KFs sin propuesta, interpola su corrección SE(3) entre controles vecinos y
  los registra como propagados aceptados. El backup del servidor incluye esos
  KFs y el HTML poscommit consume sus registros.
- paquetes compilados: `orbslam3_multi`, `orbslam3_server`.
- resultado de build: `BUILD-EXIT-CODE 0`.
- test local: `test_global_pose_store_tail_anchor`, exit code `0`; reproduce
  dos KFs tardíos entre `KF0` y `KF3` y comprueba poses `4 m`, `8 m`, `12 m`
  y cola derecha en `13 m`.
- prueba Gazebo: `prueba_44` con
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
  `SCENARIO-RUNNER-DONE success=true`, `SIM-DONE success=true`,
  `SIM-EXIT-CODE 0`.
- evidencia de carrera: la ventana `(drone=2, epoch=0)` capturó 86 KFs en
  `[55,155]`; después aparecieron `KF143-146` y `KF150-154`.
- evidencia de apply: `late_window_detected=9`,
  `late_window_refined=9`, `late_window_skipped=0`; los IDs aplicados coinciden
  exactamente con los nueve ausentes del dump inicial.
- evidencia poscommit: el HTML `f3l_debug_animation_task_1.html` contiene los
  95 KFs reales, incluidos los nueve tardíos. Cambios raw posteriores emiten
  `KEEP-SERVER-OPTIMIZED` y no `DERIVED-TAIL` para esos KFs.
- validación: target `20.133619 -> 0 m`, hard fiducial inmóvil, cero aristas
  rotas, apply `ACCEPT`, `raw_db_modified=false`.
- nube final tras 300 s: `62325` puntos, `53680` corregidos por servidor,
  `server_corrected_missing_keyframe_skipped=0`, `invalid_pose_skipped=0`.
- evidencia negativa o ausente: el dron 1 no produjo la optimización simétrica;
  no se ejercitó espera por solape y falta inspección RViz2 del usuario. El
  `gazebo exit code 255` aparece únicamente durante cleanup posterior a
  `SIM-DONE success=true`.
- conclusión: corrección de KFs tardíos internos `CONSEGUIDA` por test,
  logs y HTML; subfase `3K` global `PARCIAL/REABIERTA`.
- siguiente paso recomendado: inspeccionar `prueba_44` en RViz2 y mantener
  pendiente la prueba específica de ventanas solapadas.

## Artefactos

```text
codex/archivos_auxiliares/logs/prueba_44.log
codex/archivos_auxiliares/logs/prueba_44.reduced.log
codex/archivos_auxiliares/logs/prueba_44.index.md
codex/archivos_auxiliares/html/f3l_debug_animation_task_1.html
codex/archivos_auxiliares/repeticiones/f3i_window_task_1.tsv
codex/archivos_auxiliares/repeticiones/f3l_graph_task_1.tsv
```
