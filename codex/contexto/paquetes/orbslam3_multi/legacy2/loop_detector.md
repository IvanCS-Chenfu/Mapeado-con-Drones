# `LoopDetector`

## Estado

Implementación BoW inicial de `3N` añadida el 2026-07-20. La subfase queda
`CONSEGUIDA` el 2026-07-21: el detector procesa KFs nuevos, compara BoW real,
genera candidatos en live/replay y salta pares ya confirmados por
`CovisibilityDatabase`. El 2026-07-28 se corrigió el contrato de cercanía: KFs
cercanos del mismo submapa no se descartan por `min_kf_gap_same_submap` si no
tienen arista confirmada; se devuelven como candidatos BoW etiquetados para que
los consumidores posteriores decidan covisibilidad, error o fusion. Desde la
correccion de `prueba_75`, un candidato posterior a la query dentro del mismo
submapa se descarta: una tarea atrasada no puede mirar hacia su futuro.
El aborto live `stereo-8 exit code -6` quedó resuelto en ORB-SLAM3 y ya no
bloquea esta ruta.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/loop_candidate.hpp
orbslam3_multi/include/orbslam3_multi/loop_detector.hpp
orbslam3_multi/src/loop_detector.cpp
```

## API vigente

`ProcessNewKeyFrame` recibe un `RawKeyFrameId`, `RawMapDatabase`,
`GlobalPoseStore` opcional y `CovisibilityDatabase`. Comprueba la existencia del
KF, construye vectores BoW desde `OrbKeyFrame::bow_word_ids` y
`OrbKeyFrame::bow_word_values`, compara contra los KFs de todos los submapas de
`RawMapDatabase` y devuelve una lista corta de `LoopCandidate`.

La comparación es una similitud coseno esparsa con pesos acumulados por word id.
La búsqueda es amplia y después aplica filtros conservadores:

- query/candidato sin BoW válido;
- KF `bad` o con menos de `min_mappoints`;
- score bajo `min_bow_score`;
- candidato posterior a la query dentro del mismo submapa;
- par ya confirmado por `CovisibilityDatabase::HasConfirmedEdge`;
- límites `max_candidates` y `max_candidates_per_submap`.

`min_kf_gap_same_submap` ya no es un veto. Solo marca candidatos cercanos del
mismo submapa como `near_same_submap` y los contabiliza en
`near_same_submap_candidates`. La única razón de covisibilidad que impide enviar
un candidato a `3O` es que `CovisibilityDatabase` ya tenga una arista confirmada
para ese par.

La causalidad se aplica antes del score BoW:

```text
same_submap && candidate.local_kf_id > query.local_kf_id -> skip
```

`LoopCandidateResult::skipped_noncausal_same_submap` contabiliza esos KFs. El
KF posterior podra evaluar el par desde su propia `LoopTask`.

Los candidatos sin pose global no se descartan en `3N`; se loggean y se
penalizan suavemente en el ranking para que fases posteriores puedan decidir si
sirven para anclaje por loop. BoW nunca confirma un loop y este componente no
modifica poses, fusiona landmarks, crea tareas de optimización ni ejecuta RANSAC.

Desde `3O`, los candidatos devueltos por `LoopDetector` pasan al
`SubcloudLoopVerifier`. Esa verificación geométrica es un consumidor posterior:
`LoopDetector` sigue sin confirmar loops ni crear efectos laterales.

## Configuración

`LoopDetectorConfig` contiene:

```text
min_kf_gap_same_submap = 20
max_candidates = 10
max_candidates_per_submap = 3
min_mappoints = 15
min_bow_score = 0.01
```

El servidor expone estos valores como parámetros `loop_bow_*`.

## Salida

`LoopCandidate` contiene query/candidato, submapas, `bow_score`, `rank`,
banderas `same_drone`/`same_submap`, `kf_gap`, `near_same_submap`, pose
global/anclaje del candidato y número de MapPoints de query/candidato.

`LoopCandidateResult` resume si se procesó el KF, razón de salida, KFs
indexados/comparados, candidatos raw, candidatos tras filtro, skips por
covisibilidad confirmada, skips no causales, candidatos cercanos del mismo
submapa no descartados, BoW de query y eventos de filtro acotados para logs.

## Logs

La integración en `global_map_server` emite:

```text
[F1N-LOOP-DETECTOR-CONFIG]
[F1N-LOOP-NEW-KF-DISPATCH]
[F1N-SERVER-LOOP-DETECTOR-CALL]
[F1N-LOOP-KF-QUERY]
[F1N-LOOP-BOW-SEARCH]
[F1N-LOOP-CANDIDATE-FILTER]
[F1N-BOW-SKIP-CONFIRMED-COVIS]
[F1N-LOOP-CANDIDATE-SUMMARY]
[F1N-LOOP-NO-CANDIDATES]
[F1N-LOOP-BOW-CANDIDATES]
[F1N-LOOP-CANDIDATE]
[F1N-LOOP-CANDIDATE-RANK]
[F1N-SERVER-LOOP-CANDIDATES-RX]
```

`[F1N-LOOP-KF-QUERY]` y `[F1N-LOOP-CANDIDATE-SUMMARY]` incluyen
`skipped_noncausal_same_submap` y `near_same_submap_candidates`;
`[F1N-LOOP-CANDIDATE]` incluye
`near_same_submap=true/false`.

Estado posterior: `3O` ya construye subnubes y verifica geometría. Ningún
candidato BoW debe tratarse como loop confirmado hasta que una subfase posterior
inserte explícitamente la relación confirmada con evidencia geométrica.

En `3Q`, una geometria de error alto con `near_same_submap=true` se degrada a
`HOLD` y no construye un grafo de poses. Esa guarda pertenece al consumidor,
no a `LoopDetector`.
