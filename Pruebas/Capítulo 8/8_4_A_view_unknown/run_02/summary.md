# Prueba 8.4-A - VIEW_UNKNOWN

Resultado: **NO CONSEGUIDA**.

- Wrapper y escenario PASS; D1 llegó con GT al fiducial 2, el handoff se abrió y `TRACKING_RISK` permaneció desactivado.
- El primer `LOOK_AND_CAPTURE` correlacionado fue `autonomous:point:map_section_level_0_AB:2:2`. Terminó `look_completed`, devolvió una observación depth y el servidor la clasificó como `vista_unknown`.
- La fuente `VIEW_UNKNOWN` no tuvo claims de coverage; el coverage conservó progreso cero en el snapshot temporal inmediato.
- El giro medido fue de `90,00°` a `56,93°`: `-33,07°` a la derecha. No cumple el giro aproximado de `-90°` acordado para 8.4-A.
- Durante la mirada continuó materializándose evidencia sparse, por lo que el total de OCCUPIED del VoxelMap no permite atribuir un delta global exclusivamente a la fuente depth.

La toma prueba que el flujo real `LOOK_AND_CAPTURE -> VIEW_UNKNOWN -> depth` funciona, pero no es evidencia final de la maniobra geométrica requerida.
