# Pipeline Fase 3 - resumen

Usar este archivo antes de abrir el pipeline o historiales largos.

## Estado vigente

```text
Fase 3: ACTUAL - REIMPLEMENTACION EN CURSO
3B-3L: CONSEGUIDAS tecnica y visualmente
3M-3O: CONSEGUIDAS tecnica y visualmente
3P: CONSEGUIDA; CIERRE FUNCIONAL Y VISUAL CONFIRMADO
3Q: PREPARACION CERRADA; IMPLEMENTACION PENDIENTE
```

## Runtime hasta 3P

```text
wrapper -> GlobalMapServer -> PrimaryQueue -> PrimaryWorker
        -> SparseGlobalBackend -> raw/poses/score/builder -> ROS

fiducial -> FiducialAnchorManager --opt_fid/MAX--> SecondaryTaskQueue
          -> SecondaryWorker -> PoseGraphBuilder -> OptimizationManager
          -> OptimizationValidator -> GlobalPoseStore atomic commit
          -> GlobalMapBuilder pose dirty (consumido por proximo PrimaryInput)

raw ChangeSet -> DatabaseUpdateTask MEDIA -> CovisibilityDatabase
              -> LoopTask BAJA -> BoW/regiones/subnubes/RANSAC/decision
              -> anchor loop | fusion tracks/covis/score | evidencia 3Q
```

El flujo principal no espera al solver. El mission gate detiene el envio del
siguiente goal mientras hay optimizacion activa, pero no cancela el actual. Una
tarea secundaria activa se completa sin preemption.

## Propiedad por subfase

| Subfase | Propiedad | Estado |
|---|---|---|
| 3B | congelacion, grafo base y apertura integrada | `CONSEGUIDA` |
| 3C | raw, FIFO/worker, replay y backpressure | `CONSEGUIDA` |
| 3D | backend y pose store | `CONSEGUIDA` |
| 3E | primer anchor fiducial | `CONSEGUIDA` |
| 3F | score, builder y publicacion | `CONSEGUIDA` |
| 3G | snapshots y rendimiento | `CONSEGUIDA` |
| 3H | revisit, tarea MAX, queue/worker, stale y mission gate | `CONSEGUIDA` |
| 3I | grafo temporal mono-submapa 30/20 | `CONSEGUIDA` |
| 3J | propuesta SE(3) privada | `CONSEGUIDA` |
| 3K | commit atomico, late-window, tail y dirty KFs | `CONSEGUIDA` |
| 3L | validacion/refinamiento/fallo duro | `CONSEGUIDA` |
| 3M | covisibilidad y DatabaseUpdateTask MEDIA | `CONSEGUIDA` |
| 3N | LoopTask BAJA, indice BoW y regiones | `CONSEGUIDA` |
| 3O | subnubes, RANSAC y anchor por loop | `CONSEGUIDA` |
| 3P | fusion transitiva, score geometrico/visibilidad y commit incremental | `CONSEGUIDA` |
| 3Q | optimizacion covisible comun loop/fiducial | `PREPARADA` |
| 3S-3U/3W | integracion y hardening restante | segun contrato |
| 3V-3X | regresion y cierre | pendientes |

## Contrato 3H-3L

- cada KF fiducial distinto fuera de umbral encola MAX; solo el duplicado
  exacto se deduplica;
- dequeue relee `GlobalPoseStore` y puede terminar `STALE`;
- primer KF observado reserva control; coherente acepta directo y target full
  lo acepta tras commit;
- ventana mono-submapa, intermedios inactivos omitidos y control hard fijo;
- target absoluto, aristas temporales SE(3), sin GT global ni covisibilidad;
- commit atomico de KFs compatibles y tail posterior;
- conflictos stale revalidan/reconstruyen de forma acotada;
- solo IDs movidos ensucian el builder;
- record v3 persiste visita y mantiene replay v1/v2 compatible.

## Runtime 3M-3O

- una `DatabaseUpdateTask` MEDIA por `ChangeSet` actualiza covisibilidad y
  despues encola una `LoopTask` BAJA por KF elegible;
- BoW original permanece raw y `LoopDetector` mantiene un indice derivado;
- una busqueda BoW agrupa candidatos por region y entrega inicialmente hasta
  tres seeds a geometria;
- KFs no anclados tambien ejecutan loops; query/candidate es simetrico;
- subnubes, matching y RANSAC conservan la base validada de `legacy2` sobre
  inputs acotados y privados;
- una fusion valida domina y suprime optimizacion de esa tarea, pero 3O no
  fusiona ni optimiza;
- anchors/optimizaciones/constraints requieren dos queries independientes,
  con baseline inicial `0.20 m` o `5 grados` y margen de ambiguedad dos;
- 3O si compromete atomicamente anchors por loop y componentes conectados,
  incluyendo KFs tardios y notificando dirty al builder;
- el hijo blando sigue al KF de apoyo si este se mueve; el primer fiducial
  directo reancla todo el hijo como hard y corta el seguimiento rigido;
- scheduling loop usa huella semantica y el dequeue/commit una revision
  geometrica exacta separada;
- los parametros anteriores son hipotesis ajustables con evidencia; raw, un
  worker, prioridades, atomicidad, no GT y fronteras 3O/3P/3Q son invariantes.

## Evidencia 3M-3O

- build final correcto; CTests funcionales 8/8 + 4/4;
- replay 152 `PARCIAL` por backlog; replay 153 drena 806 secundarias con cero
  hard y `max_active=1`;
- live 154: A crea el unico hard fiducial, B/KF5 espera segundo apoyo y B/KF7
  ancla `(2,0)` por loop; backfill posterior de 9 KFs/1013 MPs;
- cierre live: anchors=2, hard=1, poses=248, active=222, secundaria vacia.
- prueba 157: apoyo A/KF72 dentro de ventana y 78 KFs hijos propagados en el
  mismo commit fiducial;
- prueba 156: reanchor hard post-loop de 32 KFs, tres commits completos,
  1060 secundarias/486 poses, `pending=0`, `hard_failed=0`, `max_active=1`;
- la carga baja de 9.20 a 2.18 tareas por KF; el usuario confirma RViz2 y grafo
  web correctos. 3P/3Q validaran integralmente las ramas producidas por 3O.

## Runtime 3P

- `LoopPipeline` decide y despacha fusion en la misma `LoopTask` BAJA;
- se reutilizan todas las regiones compatibles de 3O, sin repetir RANSAC;
- `FusedLandmarkManager` mantiene union transitiva, ID estable, medoid,
  procedencia y representante ponderado;
- inlier `+0.04`; outlier solo penaliza tras visibilidad sparse simetrica
  fiable (`-0.01/-0.03`), recorriendo toda evidencia elegible sin corte por
  reloj;
- tracks, covisibilidad y score positivo usan patch/commit breve coherente;
- stale o rollback termina la tarea y encola una BAJA fresca para el mismo KF;
  el retry atraviesa solo el ledger completado y conserva deduplicacion;
- changesets exactos quedan dirty para el siguiente principal; el secundario no
  publica ni despierta al builder;
- builder publica todos los puntos independientemente del score;
- 3P no persiste evidencia de error alto. 3Q conservara los inliers en la
  `LoopTask` que obtiene el segundo apoyo; stale crea una BAJA fresca que
  recalcula geometria.

## Contrato preparado 3Q

- fiducial absoluto y loop relativo reutilizan builder/solver/validator/commit;
- ventana = subgrafo minimo con hard, temporal, soft, fusion/loop y covis;
- fusiones previas son constraints relativas soft con residual medido;
- controles base 30 % se amplian por endpoints covisibles obligatorios;
- no se distingue inter/intra dron/submapa para la decision geometrica;
- loop requiere dos queries y accept completo; fusion 3P directa es opcional;
- KFs tardios/tails siguen continuidad por submapa y paran ante hard;
- la `LoopTask` permanece BAJA/no preemptiva, pero `stop_drones` permanece
  activo desde el inicio de 3Q hasta task end, incluida fusion posterior;
- validacion: tests/replay, diez escenarios Gazebo naturales y cuatro revisiones
  RViz2/web representativas.

## Evidencia 3P

- prueba 159 conservada como `NO CONSEGUIDA`: el runner termino, pero el
  servidor aborto por un track absorbido que seguia en `touched_tracks`;
- la correccion valida referencias, elimina el track retirado, incorpora una
  regresion exacta y protege el worker con una barrera de excepciones;
- build 3/3 y tests `orbslam3_multi` 9/9, servidor 4/4, web 1/1;
- prueba 160: 62 intentos, 56 commits, cinco stale, un rollback, 1116 tareas
  secundarias drenadas, cero hard failure y servidor limpio;
- builder consume tracks en 228 de 383 publicaciones y la final recalcula 87;
- recursos correctos: MemAvailable minimo 6568.8 MiB, PSS servidor 229.7 MiB,
  PSI full 0 y guardia inactiva;
- el usuario confirma que RViz2 y el grafo web de la prueba 160 se vieron muy
  bien;
- prueba 161: 27 intentos, ocho commits, 19 stale/rollback y exactamente 19
  retries; `56/56` regiones completas, cuatro optimizaciones fiduciales full,
  1162 secundarias drenadas, cero hard y guard inactivo;
- prepare aceptado media/maximo `633.852/1087.130 ms`; el mayor coste no impide
  cierre ni drenaje. El usuario da por concluida 3P; queda solo un pulido de
  distribucion desktop del grafo web.

## Evidencia

- build final correcto, 53/53 C++ y 9/9 web;
- 142/143 `NO CONSEGUIDAS`, causas conservadas y corregidas;
- 144: 10 commits y cero fallos;
- 145 live: 79 observaciones, 44 tareas, 30 commits, 14 stale, cero hard,
  282 KFs movidos, recursos correctos;
- 146 replay v3: reproduce exactamente 44/30/14/0;
- live 145 revisada diagnostico colores casi indistinguibles, pulsos web de
  240 ms y KFs futuros bajo el anchor original; se conserva como evidencia del
  fallo anterior;
- los 30 commits fueron reales, pero parte de ellos corrigio repetidamente la
  discontinuidad creada despues de commits anteriores.
- continuidad corregida con record atomico en `GlobalPoseStore`; replay 149
  reduce a 7 tareas/3 commits/4 stale sin errores posteriores altos;
- live 148 falla por carrera de control KF149/KF150 y se conserva; replay 150
  valida la reserva del primer KF con cero hard;
- live 151 completa: 6 submapas, 11 tareas, 3 commits, 8 stale, cero hard,
  pending0 y backpressure liberado; el usuario confirmo el cierre visual.

## Referencias

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/pipeline/fase_3_sparse_global/historial/INDEX.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3M.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3N.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3O.md
```
