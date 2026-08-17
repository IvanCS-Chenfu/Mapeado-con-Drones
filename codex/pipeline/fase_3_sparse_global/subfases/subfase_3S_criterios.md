# Subfase 3S - Criterios

## Estado vigente

```text
REHACER
```

## Criterios añadidos por la reimplementación

- barridos completos de score por llegada: `0`;
- score de patches rechazados/stale: `0`;
- publicaciones bloqueadas esperando score secundario: `0`;
- IDs dirty de score perdidos: `0`;
- prioridades/colas nuevas creadas solo para score: `0`.

## Exito

1. `LandmarkScoreManager` es la unica autoridad numerica.
2. El flujo principal actualiza solo MPs raw afectados por `ChangeSet`.
3. La `LoopTask` compromete score fused junto a la fusion.
4. Tareas rechazadas/stale no dejan score.
5. No existe cola/worker de score.
6. Snapshots son coherentes y commits breves.
7. `GlobalMapBuilder` lee sin modificar ni bloquear.
8. La ausencia de resultado secundario no impide publicar score base.
9. No se usa GT.
10. Build, tests y simulacion pasan con evidencia reducida.
11. Un outlier sin contradiccion visible fiable no recibe penalizacion.
12. Repetir evidencia/revision no vuelve a incrementar o reducir score.
13. `GlobalMapBuilder` publica todos los puntos con independencia del score.

## Parcial

Si score raw funciona pero no se obtiene fusion real, o la formula fused queda
provisional aunque el ownership/transaccion sean correctos.

## Fallo

- score modificado fuera del manager;
- worker o cola propios;
- score visible de tarea rechazada;
- fusion y score publicados en revisiones incompatibles;
- puntos ocultados por umbral de score dentro de `GlobalMapBuilder`;
- recalculo completo bajo lock;
- publicacion bloqueada esperando score secundario;
- crash, carrera, NaN o uso de GT.

## Documentacion

Tras implementar, actualizar `landmark_score_manager.md`,
`fused_landmark_manager.md`, `global_map_builder.md`, `global_map_server.md` e
historial `3S` sin borrar intentos anteriores.

El vertice de score, sus patches y el efecto posterior en RViz2 deben quedar
confirmados; pendiente visual implica `PARCIAL`.
