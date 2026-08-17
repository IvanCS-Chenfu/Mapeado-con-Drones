# Contexto minimo actual

Precondicion: leer fisicamente `00_CONTEXTO_COMPACTACION.md` antes de este
archivo y reconciliarlo con la peticion mas reciente.

## Estado

```text
Fase: 3 - mapa sparse global multi-dron
3B-3O: CONSEGUIDAS
3P: CONSEGUIDA; cierre funcional y visual confirmado por el usuario
3Q: preparacion cerrada y documentada; implementacion pendiente de autorizacion
```

## Runtime

```text
wrapper -> PrimaryQueue -> PrimaryWorker -> SparseGlobalBackend -> builder/ROS

fiducial MAX -> SecondaryTaskQueue -> SecondaryWorker -> graph/solver/commit
ChangeSet MEDIA -> CovisibilityDatabase -> LoopTask BAJA
LoopTask -> BoW -> regiones -> subnubes/RANSAC -> decision
         -> anchor loop | fusion tracks/covis/score | evidencia pendiente 3Q
```

- una tarea principal y una secundaria pueden calcular en paralelo;
- un unico worker secundario no preemptivo prioriza MAX > MEDIA > BAJA;
- las tareas se revalidan al dequeue y los loops se coalescen por huella
  semantica, conservando validacion geometrica exacta separada;
- `RawMapDatabase` y BoW original permanecen crudos;
- 3P continua el error bajo en la misma `LoopTask`, con tracks transitivos,
  score/visibilidad, covisibilidad server y commit incremental con rollback;
- visibilidad recorre toda evidencia elegible sin corte temporal; stale o
  rollback termina y encola una BAJA fresca deduplicada para el mismo KF;
- el siguiente principal materializa miembros ocultos y un representante por
  track; el secundario no publica;
- un anchor loop es blando y sigue al padre hasta un fiducial hard propio;
- el builder consume KFs dirty en el siguiente `PrimaryInput`.

## Contrato 3Q acordado

- optimizador SE(3) comun para fiducial absoluto y loop relativo;
- subgrafo minimo con hard, tramos temporales, dependencias soft,
  loops/fusiones previos y covisibilidad confirmada;
- dos queries, inliers vivos, 30 % base ampliable y sin exclusion inter/intra;
- fusiones anteriores relativas soft; accept loop completo inicialmente;
- fusion 3P directa opcional, stale/rollback con BAJA fresca;
- una optimizacion loop conserva prioridad BAJA pero activa `stop_drones`
  desde branch begin hasta task end, incluida fusion posterior;
- matriz de tests/replay y diez topologias Gazebo naturales, cuatro visuales.

## Evidencia vigente

- build final 3/3; 53/53 tests C++ y 9/9 web;
- replay 152 `PARCIAL`: 384 secundarias pendientes por coste geometrico;
- replay 153 `CONSEGUIDA`: 806 procesadas, `pending=0`, cero hard;
- live 154: A ancla por fiducial; B/KF5 y B/KF7 confirman dos apoyos y
  `(2,0)` se ancla por loop sin fiducial;
- siguiente principal hace backfill de 9 KFs/1013 MPs;
- cierre live: anchors=2, hard=1, poses=248, active=222, guard inactivo.
- prueba 157: B se ancla contra A/KF72 y el commit fiducial propaga 78 KFs del
  hijo blando;
- prueba 156: reanchor hard post-loop de 32 KFs, tres commits fiduciales,
  1060 tareas para 486 poses (2.18/KF), `pending=0`, cero hard y guard inactivo;
- ORB genero siete submapas; cuatro quedaron anclados y tres diferidos al
  cierre. El usuario confirmo que RViz2 y el grafo web se veian correctamente.
- prueba 159: runner completo pero servidor abortado por track absorbido todavia
  presente en `touched_tracks`; intento `NO CONSEGUIDO` conservado.
- prueba 160 corregida: 56 commits de fusion, cinco stale, un rollback, 1116
  secundarias drenadas, cero hard y servidor limpio; builder consume tracks.
- tests 9/9 + 4/4 + 1/1, guardia de recursos inactiva. El usuario confirma que
  RViz2 y el grafo web se vieron muy bien.
- prueba 161: 27 intentos, ocho commits, 19 stale/rollback y 19 retries
  drenados; `56/56` regiones completas, cuatro optimizaciones fiduciales full,
  `pending=0`, cero hard y guard inactivo.
- sin presupuesto, el prepare aceptado sube a 633.852/1087.130 ms de
  media/maximo; la cola y el escenario aun terminan correctamente.

## Lectura siguiente

```text
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3Q_RESUMEN.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3Q.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3Q_especificacion.md
```

Los logs completos nunca se leen directamente; siempre se reducen primero.
