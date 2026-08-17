# Historial 3L.8 - Validación de `prueba_44`

## 2026-07-28 - KFs tardíos internos

`prueba_44` acepta la única tarea fiducial creada con error real final nulo,
hard fiducial inmóvil, cero aristas rotas y mapa global válido. Los nueve KFs
omitidos del dump inicial aparecen corregidos en el HTML poscommit y conservan
autoridad `server_optimized` frente a cambios raw posteriores.

La nube permanece estable durante 300 s con `62325` puntos,
`invalid_pose_skipped=0` y
`server_corrected_missing_keyframe_skipped=0`. La validación técnica queda
`CONSEGUIDA`; la subfase sigue `PARCIAL/REABIERTA` hasta la inspección RViz2
del usuario y la continuidad multi-epoch. El detalle completo está en
`historial_3K_6.md`.
