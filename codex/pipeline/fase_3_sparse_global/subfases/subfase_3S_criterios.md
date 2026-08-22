# Subfase 3S - Criterios

## Exito

1. `LandmarkScoreManager` es la unica autoridad numerica.
2. La formula raw acordada usa base ORB, distancia, aislamiento y `+0.04` por
   inlier, con factores acotados y recuperables.
3. Con baseline actual `0.06 m`, el factor de distancia es neutro entre 1 y
   5 m; por debajo de 1 m y por encima de 5 m cae cuadraticamente hasta sus
   minimos `0.05` y `0.25`, respectivamente.
4. Raw no anclado conserva factores neutros; cambios ORB/pose/posicion se
   propagan incrementalmente.
5. Fused score es exactamente `clamp(media(raw) + 0.04 * N, 0, 1)` y se
   actualiza tras altas, cambios, merges y bajas.
6. Una penalizacion cercana raw puede diluirse en la media fused y no impone
   cap permanente al track.
7. Visibilidad sparse produce diagnostico y cero penalizaciones numericas.
8. Patches rejected/stale dejan cero score visible y la evidencia es
   idempotente.
9. `ScoreChangeSet` y dirty sets no pierden IDs ni fuerzan snapshots completos.
10. Builder publica todos los puntos; RViz2 colorea rojo-amarillo-verde por
   score.
11. No existe GT, cola/worker nuevo ni bloqueo de publicacion.
12. Build, tests y simulacion pasan con evidencia reducida y recursos estables.
13. Grafo web y RViz2 validan puntos cercanos degradados, paredes hasta 5 m sin
    penalizacion de distancia y refuerzo correcto por revisitas.

## Parcial

- La implementacion y pruebas automaticas pasan, pero no aparece suficiente
  ruido/fusion natural para valorar una regla; o
- falta la confirmacion visual del usuario.

## Fallo

- score modificado fuera del manager;
- formula fused distinta de la acordada;
- penalizacion numerica sparse por oclusion en 3S;
- barrido completo por llegada, snapshot completo para publicar o worker nuevo;
- punto ocultado por score en builder;
- cambio visible procedente de tarea rejected/stale;
- NaN, carrera, crash, uso de GT o bloqueo de ingesta/publicacion.

## Documentacion de cierre

Actualizar docs vigentes de `landmark_score_manager`,
`fused_landmark_manager`, `sparse_global_backend`, `global_map_builder` y
`global_map_server`, ademas de historial 3S, resumen, indice, estado y ultima
sesion. Cada ejecucion conserva su evidencia y conclusion propia.
