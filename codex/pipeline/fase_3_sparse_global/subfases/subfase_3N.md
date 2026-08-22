# Subfase 3N - LoopTask, indice BoW y regiones candidatas

## Estado

```text
CONSEGUIDA; IMPLEMENTADA Y VALIDADA EL 2026-08-15
```

Este contrato sustituye la politica anterior que impedia loops pre-anchor. Los
KFs no anclados deben participar para permitir el anclaje por loop de 3O.

## Objetivo

Crear una `LoopTask` BAJA por cada KF materialmente elegible y ejecutar una
unica busqueda BoW incremental que entregue regiones candidatas a 3O dentro del
mismo `task_id`.

```text
LoopTask BAJA
  -> revalidar query y revisiones
  -> insertar/actualizar query en LoopBoWIndex
  -> intentar rama rapida si tiene pose world, en 3O
  -> una busqueda BoW si la rama rapida no resuelve
  -> filtros y memoria canonica de pares
  -> agrupar candidatos por region
  -> hasta tres candidate seeds
  -> 3O en la misma LoopTask
```

BoW propone lugares; no confirma geometria, no ancla, no fusiona y no modifica
poses.

## Creacion de LoopTask

La clave causal es:

```text
(RawKeyFrameId, appearance_revision, geometry_revision, anchor_revision)
```

- una tarea es un ticket pequeño con IDs y revisiones;
- nunca contiene mapas, nubes o descriptores completos;
- un `ChangeSet` con covisibilidad se procesa primero mediante MEDIA y esta
  encola una BAJA por cada KF elegible;
- si no hace falta MEDIA, el servidor encola la BAJA directamente;
- un snapshot equivalente no crea otra tarea;
- cambios de metadata ajena a BoW/geometria/anchor no repiten el loop;
- al anclarse un submapa o componente, el cambio de `anchor_revision` habilita
  una nueva pasada coalescida de sus KFs.

Los KFs anclados y no anclados se admiten. La unica prohibicion es aplicar un
efecto world cuando ninguno de los dos lados proporciona autoridad suficiente;
3O puede conservar la relacion relativa para una cascada posterior.

## LoopTaskLedger

El ledger mantiene como minimo:

```text
QUEUED
ACTIVE
COMPLETED
STALE
DIRTY_AFTER_RUN
```

`DEFERRED` se usa para hipotesis o pares pendientes, no para impedir que un KF
no anclado ejecute BoW/RANSAC.

Reglas:

- una clave equivalente solo puede estar pendiente o activa una vez;
- una revision nueva sustituye trabajo pendiente obsoleto;
- si cambia durante `ACTIVE`, queda un unico `DIRTY_AFTER_RUN`;
- no existe retry inmediato infinito tras `STALE`;
- una nueva revision material o de anchor puede crear otra pasada;
- no hay limite destructivo de cola; el watermark aplica backpressure.

## Almacenamiento BoW

`RawMapDatabase` conserva el BoW original de cada KF y sigue siendo la
autoridad. No se copia a `CovisibilityDatabase`.

`LoopDetector` mantiene un `LoopBoWIndex` derivado, reconstruible y residente:

```text
word_id -> [(RawKeyFrameId, weight, appearance_revision), ...]
```

Al comenzar la `LoopTask`:

1. valida el BoW raw del query;
2. elimina postings de una revision de apariencia anterior;
3. inserta o actualiza los postings actuales;
4. consulta solo KFs que comparten palabras;
5. excluye la propia identidad;
6. calcula similitud coseno esparsa y ranking determinista.

No se crea una tarea de indexado. Si se pierde el indice, puede reconstruirse
desde raw. Deben existir APIs de test para comprobar upsert, invalidacion y
reconstruccion sin convertir el indice en autoridad.

## LoopPairAttemptDatabase

La memoria de pares es independiente de covisibilidad. Su clave canonica incluye
las revisiones materiales de ambos extremos.

Puede registrar:

```text
BOW_REJECTED
GEOMETRY_REJECTED
DEFERRED_AMBIGUOUS
PENDING_GEOMETRY
PENDING_LOOP_ANCHOR
ALREADY_RESOLVED
```

- un rechazo definitivo se reutiliza solo para las mismas revisiones;
- falta temporal de anchor, estado stale o ambiguedad no son rechazos
  definitivos;
- una revision material distinta habilita otro intento;
- una confirmacion positiva invalida rechazos anteriores del par;
- los snapshots repetidos y la direccion simetrica no repiten trabajo resuelto;
- se conserva un estado vigente por par/revisiones, sin limite arbitrario de
  pares y eliminando revisiones obsoletas.

## Busqueda y filtros

La busqueda BoW se ejecuta una vez por query. Se excluyen o etiquetan con motivo:

- misma identidad;
- KF bad, incompleto o sin BoW valido;
- pocos MapPoints observados;
- candidato posterior no causal dentro del mismo submapa;
- par definitivamente rechazado para las mismas revisiones;
- arista `SERVER_LOOP_GEOMETRIC` ya resuelta;
- score bajo el umbral configurable.

Una arista `ORBSLAM3_NATIVE` no implica fusion y no se descarta ciegamente:
sirve para agrupar KFs que representan la misma region y construir la ventana
de 3O. La cercania temporal `near_same_submap` se conserva como diagnostico, no
como veto absoluto.

Los candidatos sin pose world no se descartan por ese motivo. Query y candidate
son simetricos: si exactamente uno esta anclado, 3O puede anclar el otro.

## Agrupacion por region

Una busqueda puede devolver muchos KFs de la misma fachada. Antes de RANSAC:

1. ordenar por score BoW;
2. agrupar candidatos unidos por covisibilidad fuerte y vecindad temporal del
   mismo submapa;
3. elegir como seed el mejor score de cada region;
4. aplicar diversidad determinista;
5. entregar inicialmente hasta tres regiones a 3O.

El limite inicial es `max_candidate_regions = 3`. No significa tres busquedas
BoW ni tres tareas: es una busqueda y hasta tres verificaciones dentro de la
misma `LoopTask`.

Los parámetros históricos se usan como punto de partida para score mínimo,
MapPoints y ranking. El limite antiguo de diez KFs puede mantenerse como
preseleccion antes de agrupar, pero no obliga a ejecutar diez RANSAC.

## Tipos de salida

`LoopCandidateRegion` debe contener al menos:

```text
query_kf_id
candidate_seed_kf_id
candidate_region_id
member_kf_ids acotados
bow_score
rank
same_drone / same_submap
near_same_submap
query_has_world_pose
candidate_has_world_pose
dependency_revisions
```

`LoopCandidateResult` resume comparados, filtrados, regiones, motivos de rechazo
y seeds entregados. No se devuelve solo un booleano o un best candidate unico.

## Sincronizacion

- el detector consume vistas inmutables y acotadas;
- no mantiene locks de raw/covisibilidad durante una busqueda amplia;
- no posee cola, worker, callback ROS ni publishers;
- el resultado puede quedar stale y se revalida antes de efectos;
- el flujo principal debe seguir ingresando y publicando mientras una
  `LoopTask` ejecuta BoW o geometria.

## Cambios previstos

En `orbslam3_multi`:

- crear `LoopTask`, `LoopTaskLedger` y sus claves causales;
- reconstruir `LoopDetector` y su indice BoW incremental;
- crear tipos de candidatos agrupados por region;
- reconstruir `LoopPairAttemptDatabase` por revisiones;
- exponer todo mediante `SparseGlobalBackend`.

En `orbslam3_server`:

- sustituir el placeholder `Loop` por payload real;
- encolar loops tras MEDIA o directamente segun `ChangeSet`;
- ejecutar 3N y continuar a 3O con el mismo `task_id`;
- conservar un unico worker y la prioridad BAJA.

En el visualizador:

- añadir `LoopDetector`, `LoopBoWIndex` y aristas reales;
- mostrar query, revision, candidatos raw, regiones y seeds;
- mantener iluminado el flujo durante la etapa, sin replay visual tardio;
- no enviar BoW, descriptores ni nubes completas.

## Archivos probables

```text
orbslam3_multi/include/orbslam3_multi/loop_task.hpp
orbslam3_multi/include/orbslam3_multi/loop_task_ledger.hpp
orbslam3_multi/include/orbslam3_multi/loop_candidate.hpp
orbslam3_multi/include/orbslam3_multi/loop_detector.hpp
orbslam3_multi/src/loop_detector.cpp
orbslam3_multi/include/orbslam3_multi/loop_pair_attempt_database.hpp
orbslam3_multi/src/loop_pair_attempt_database.cpp
orbslam3_multi/test/test_loop_detector.cpp
orbslam3_multi/test/test_loop_task_ledger.cpp
orbslam3_multi/test/test_loop_pair_attempt_database.cpp
orbslam3_server/include/orbslam3_server/secondary_queue.hpp
orbslam3_server/src/global_map_server.cpp
```

## Pruebas

1. upsert, reemplazo e invalidacion del indice BoW;
2. reconstruccion equivalente desde `RawMapDatabase`;
3. ranking coseno determinista;
4. una busqueda produce varios candidatos sin repetir BoW;
5. candidatos covisibles se agrupan en una region;
6. se entregan como maximo tres regiones y el mejor seed de cada una;
7. una arista ORB no se trata como fusion confirmada;
8. una arista server resuelta evita trabajo repetido;
9. KFs no anclados generan `LoopTask` y candidatos;
10. snapshots equivalentes no generan tareas;
11. cambios materiales si generan una unica revision vigente;
12. un cambio durante `ACTIVE` produce como maximo un rerun;
13. la cache negativa acierta solo con revisiones equivalentes;
14. query/candidate simetricos no duplican un par canonico;
15. la ingesta principal progresa con BoW ralentizado;
16. lifecycle web corresponde a una sola `LoopTask`.

## Criterios de cierre

3N sera `CONSEGUIDA` si todos los KFs elegibles, anclados o no, pasan por una
unica tarea causal, el indice evita barridos globales, los snapshots no repiten
trabajo, las regiones se agrupan deterministicamente y no se producen efectos
de pose/fusion.

Sera `NO CONSEGUIDA` si se excluyen KFs por no tener anchor, se pierden tareas,
se procesa solo un best candidate sin diagnostico, se repiten snapshots
equivalentes, se guarda BoW en covisibilidad o se bloquea el flujo principal.

## Evidencia de cierre

- `test_loop_pipeline` valida indice derivado, regiones y causalidad junto al
  flujo 3O; `test_secondary_queue` valida prioridad y coalescencia;
- replay 153 procesa y drena 806 tareas secundarias con `max_active=1`;
- live 154 conserva trabajo de KFs no anclados y entrega las queries B/KF5 y
  B/KF7 como apoyos independientes de la misma hipotesis.

## Politica de adaptacion

La idea principal es una busqueda BoW incremental y hasta tres regiones. Tras
medir tests, replay y live se podran ajustar `max_candidate_regions`, score,
diversidad, agrupacion y representacion compacta del indice. Si tres regiones
son insuficientes o costosas, se documentara la evidencia y se acordara el
cambio. Permanecen invariantes una sola `LoopTask`, un solo worker, raw como
autoridad, ausencia de efectos en 3N y causalidad por revisiones.
