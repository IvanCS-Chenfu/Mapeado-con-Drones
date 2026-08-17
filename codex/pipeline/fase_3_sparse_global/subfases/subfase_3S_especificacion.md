# Subfase 3S - Especificacion de scoring

## Estado vigente

```text
REHACER INTEGRACION
```

La autoridad y formula vigentes se conservan. El score ORB base sigue en el
flujo principal para MPs del `ChangeSet`; solo calculos derivados opcionales
usan la `DatabaseUpdateTask` MEDIA existente. Toda modificacion devuelve un
`ScoreChangeSet` preciso para `3F`.

### Lo anterior que estaba mal

- actualización amplia dentro del callback principal;
- snapshots completos de score para publicación/loops;
- falta de dirty IDs/revisiones por entidad;
- dependencia implícita entre score secundario y publicación principal.

## Estado histórico anterior

Las secciones posteriores se conservan como contrato/evidencia de la
implementación anterior. Si contradicen el bloque REHACER de esta cabecera,
prevalece el contrato de reimplementación nuevo.

```text
sin hacer; contrato reorganizado y preparado
```

## Objetivo

Mantener un score unico y versionado para MPs raw y fused tracks sin convertir
el scoring en otro pipeline. Ninguna clase externa modifica valores numericos
directamente; solo emite eventos estructurados a `LandmarkScoreManager`.

## Dos fuentes de actualizacion

### Flujo principal

Tras cada `RawInsertResult/ChangeSet`, el servidor entrega solo MPs nuevos o
materialmente modificados:

```text
RAW_CREATED
RAW_QUALITY_UPDATED
RAW_MARKED_BAD
RAW_OBSERVATION_CHANGED
```

La actualizacion base debe ser barata y terminar antes de solicitar la revision
publicable. No recorre toda raw ni espera a loops.

### `LoopTask`

La rama `3P` prepara eventos como parte del candidato de fusion:

```text
FUSION_CONFIRMED
TRACK_CREATED
TRACK_EXTENDED
TRACK_MERGED
GEOMETRIC_REOBSERVATION_CONFIRMED
EXPECTED_VISIBLE_MISS
FOREGROUND_CONTRADICTION
```

Esos eventos se comprometen en la misma `DerivedStateRevision` que los fused
tracks. Una tarea rechazada/stale no deja score derivado.

Una optimizacion sin fusion no cambia score por el mero hecho de mover poses,
salvo una regla futura acordada expresamente.

Reglas fijadas por 3P:

- RANSAC rechazado, tarea stale o error alto sin optimizacion: ningun cambio;
- inlier aceptado: `+0.04` para cada uno de los dos MPs;
- outlier ambiguo/aislado: solo diagnostico;
- hard outlier: solo puede penalizar despues de una comprobacion de visibilidad
  sparse simetrica;
- miss esperado y visible: `-0.01` solo al MP responsable;
- contradiccion foreground fiable: `-0.03` solo al MP responsable;
- ocluido, fuera de imagen, detras o incierto: ningun cambio.

Los deltas y umbrales son configurables. Si se agota el presupuesto de
visibilidad de `2 ms` por region, se conservan positivos y se omiten negativos
incompletos.

## Estado poseido

```text
RawMapPointId -> RawLandmarkScore
FusedTrackId -> FusedLandmarkScore
base_score_orb
positive/negative evidence counters and adjustments
final_score
score_revision
event counters / diagnostico acotado
```

Un historial de eventos puede conservarse para debug/replay, pero no es el
mecanismo normal de rollback. La ruta normal prepara el estado candidato y lo
publica atomically al aceptar el commit.

## Politica

- rango y defaults configurables;
- score raw derivado de calidad ORB disponible, no GT;
- score fused calculado desde miembros, soporte y eventos confirmados;
- penalizaciones por unmatched solo con evidencia de visibilidad/expectativa;
- un candidato rechazado no penaliza ni premia;
- toda razon de cambio es trazable;
- la misma evidencia/revision es no-op; una revision materialmente nueva puede
  reforzar;
- cambios de metadata irrelevante son no-op.

## Publicacion

`GlobalMapBuilder` recibe score compatible con las revisiones raw y fusion.
Publica todos los fused tracks y raws no fusionados, sin umbral, y copia el
valor como atributo. Si el score derivado aun no esta disponible, usa el score
base acordado; nunca espera al worker. La GUI de Fase 7 aplicara el filtrado
visual configurable.

La publicacion no cambia scores y el visualizador incremental de `3B` solo
observa eventos; `3U` valida su comportamiento final.

## Archivos permitidos

```text
orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp
orbslam3_multi/src/landmark_score_manager.cpp
orbslam3_multi/include/orbslam3_multi/landmark_score_event.hpp
orbslam3_multi/include/orbslam3_multi/fused_landmark_manager.hpp
orbslam3_multi/src/fused_landmark_manager.cpp
orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp
orbslam3_multi/src/global_map_builder.cpp
orbslam3_server/src/global_map_server.cpp
```

## Prohibiciones

- No modificar score desde `GlobalMapBuilder`, servidor, fusion u optimizador
  mediante `score += X`.
- No crear worker/cola de score.
- No hacer rollback normal despues de exponer scores live.
- No usar GT.
- No reconstruir todos los scores por cada delta o fusion.
- No bloquear ingesta/publicacion por exportar CSV o estadisticas.
