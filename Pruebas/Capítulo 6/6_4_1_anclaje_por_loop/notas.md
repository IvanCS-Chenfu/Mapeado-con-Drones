# Prueba 6.4.1 - Anclaje de un submapa mediante loop

## Configuracion

- D1, a `z=1.0 m`, se ancla inicialmente mediante el fiducial sur y recorre
  hasta `(-10,0)`.
- D2, a `z=1.3 m`, empieza en la fachada oeste fuera del rango de los
  fiduciales y recorre `(-10,-10) -> (-10,0) -> (-10,10) -> (0,10)`.
- D1 completa su mapa y espera antes de que D2 entre en la region compartida.
  D2 tambien espera tras esa region para hacer observable el anclaje por loop
  antes de dirigirse al fiducial norte.
- Fuente GT, Gazebo y `multidron_gui`; sin RViz2, Fase 6 ni protocolo de
  perdida ORB.

## Instrumentacion y artefactos

- `[F3O-RANSAC]` y `[F3O-LOOP-DONE]` deben mostrar la geometria y la decision
  de loop que ancla D2 mediante la autoridad de D1.
- Los `[C6-KF-POSE]` de D2 deben aparecer tras ese anclaje, demostrando la
  materializacion retrospectiva de su submapa en `W`.
- `datos_brutos/ejecucion.log` conserva la ejecucion completa y
  `datos_brutos/eventos_marcadores_reducidos.log` conserva la reduccion por
  familias de marcadores.
- `datos_procesados/timeline_autoridad_d2.csv` y
  `datos_procesados/keyframes_globales.csv` son las series reproducibles.
- Las figuras `timeline_autoridad_global_d2` y
  `mapa_xy_despues_anclaje_loop` se generan con
  `scripts/procesar_c6_4_1_anclaje_por_loop.py`.

## Resultado

La ejecucion termino correctamente: `scenario_runner=0` y
`[SIM-DONE] ... success=true`.

1. D2 no tenia autoridad global al inicio. A los `123.991 s` se produjo
   `[F3O-LOOP-DONE]` para `query=(2,0,9)` con `decision=anchor_proposed`,
   `anchors=1`, `bow=7`, una region y una geometria. El soporte fue aceptado
   con `4/4` evidencias independientes.
2. A los `125.290 s` aparecio la primera telemetria global de D2; por tanto,
   sus KeyFrames previos se materializaron retrospectivamente tras el anclaje
   por loop. El CSV conserva 88 publicaciones de KFs de D2.
3. D2 observo el fiducial norte a los `229.723 s`. A los `229.932 s` se
   produjo `[F3K-ATOMIC-COMMIT]` con `commit=75`, `revision=75` y
   `hard_added=1`, consistente con la incorporacion de la restriccion
   absoluta.

No se emitio `[F3O-FID-LOOP-REANCHOR]` en esta ejecucion. Por ello la prueba
demuestra el anclaje por loop, la materializacion retrospectiva y el commit
fiducial posterior, pero no permite afirmar que ese camino use el marcador
explicito de reanclaje. Se conserva esta discrepancia sin alterar el algoritmo.
