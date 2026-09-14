# Subfase 3R - Especificacion de scoring

## Ownership

`LandmarkScoreManager` posee bases ORB, factores geometricos, evidencias raw,
scores finales, scores fused y revisiones. Fusion, backend, servidor y builder
solo aportan entradas estructuradas o consumen resultados.

## Score raw

```text
base_score_orb = clamp(
  0.55 * min(observations_count / 8, 1)
  + 0.35 * clamp(found_ratio, 0, 1)
  + 0.10 * descriptor_valid,
  0, 1)

score_raw = clamp(
  base_score_orb * factor_distancia * factor_aislamiento
  + positive_adjustment,
  0, 1)
```

En 3R `positive_adjustment` crece `+0.04` por inlier de fusion confirmado e
idempotente. No se generan ajustes negativos por visibilidad sparse.

El factor de distancia usa tres tramos. Con `d` como distancia al KF observador:

```text
near_limit = 1.0 m
far_limit = max(near_limit, 83.333333 * baseline)

d < near_limit:
  factor_distancia = max(0.05, (d / near_limit)^2)
near_limit <= d <= far_limit:
  factor_distancia = 1
d > far_limit:
  factor_distancia = max(0.25, (far_limit / d)^2)
```

`baseline = bf / fx`; si no esta disponible, `far_limit` usa fallback de 5 m.
Con el baseline actual de `0.06 m`, la banda neutra es exactamente 1-5 m. El
limite cercano permanece metrico porque representa plausibilidad fisica del
entorno; el lejano escala con el alcance estereo.

El factor de aislamiento usa vecinos globales dentro de un radio configurable.
Solo se aplica a puntos con madurez ORB minima, evitando penalizar
inmediatamente el frente de crecimiento del mapa. Se recalcula en las celdas
espaciales afectadas cuando un punto aparece, se mueve o desaparece.

Una evolucion futura, aun no implementada, revisara esta penalizacion para
reducir de forma mas marcada el score de MapPoints persistentemente aislados y
para incorporar evidencia geometrica de KeyFrames cercanos de otro dron. Debe
ser una entrada incremental, idempotente y recuperable al cambiar geometria o
vecindad; no pertenece a `task_server`, no usa GT y no altera la formula 3R
vigente hasta que exista contrato y prueba especificos.

Esa evolucion se coordinara con 6D: el score final de cada MapPoint seguira
siendo una salida de este manager, pero no sera por si solo una decision de
ocupacion. `VoxelMapWorker` agregara reversiblemente los scores de identidades
contenidas en cada voxel y comparara esa evidencia espacial con un umbral de
ocupacion propio. La suma no podra duplicarse por republicaciones y debera
revertirse por move/delete/cambio de score. Los filtros transitorios de 6D se
retiraran en esa migracion conjunta, no antes.

Un raw no anclado usa factores `1`. Si ORB-SLAM3 cambia calidad o posicion, o
si cambia el anchor/pose de su submapa, el score se vuelve a calcular y puede
bajar o recuperarse.

## Score fused

```text
score_fused = clamp(sum(score_raw_i) / N + 0.04 * N, 0, 1)
```

La media incluye todos los miembros del track. `N >= 2` para un track fusionado
normal, de modo que la primera fusion añade `0.08`. El orden de insercion no
cambia el resultado. Cualquier cambio material de un miembro ensucia y
recalcula solo su track. La penalizacion cercana queda en el raw afectado y
puede diluirse al incorporar raw miembros buenos; no existe cap ni castigo
permanente sobre el fused track.

## Visibilidad y oclusion

Proyecciones, misses y contradicciones sparse pueden contarse y mostrarse como
diagnostico. No crean deltas numericos en 3R porque una profundidad menor puede
ser ruido foreground y no evidencia de mejor landmark. La politica de oclusion
queda aplazada a Fase 8, al contrastar sparse con nube densa.

## Invariantes

- `ScoreChangeSet` solo contiene salidas materialmente modificadas.
- La misma evidencia no se aplica dos veces.
- Patches rejected/stale dejan cero cambios visibles.
- El builder no escribe score ni filtra geometria por score.
- No hay GT, barridos completos por delta ni cola propia.
- Estadisticas y visualizacion quedan fuera de locks de commit prolongados.
