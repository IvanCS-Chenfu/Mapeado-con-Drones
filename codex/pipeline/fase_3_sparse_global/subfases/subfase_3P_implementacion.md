# Subfase 3P - Implementacion acordada

## Estado

```text
AJUSTE POST-160 IMPLEMENTADO Y VALIDADO TECNICAMENTE
```

## Principio de integracion

`LoopPipeline` conserva la decision y produce una `FusionRequest` tipada.
`SparseGlobalBackend` la entrega al servicio de fusion especializado sin salir
de la `LoopTask`. No se crea `LoopDecisionManager`, `FusionTask`, otra cola ni
otro worker.

La implementacion puede adaptar nombres a las clases activas, pero debe
mantener este ownership y actualizar sus MD de paquete.

## Tipos requeridos

Crear o activar equivalentes a:

```text
FusedTrackId
FusedLandmarkTrack
FusionRequest
FusionPatch
FusionChangeSet
FusionRollbackPatch / inverse patch
ScoreEvidence
ScorePatch
ScoreChangeSet
```

`FusionChangeSet` enumera tracks creados, actualizados y retirados, miembros
raw ocultados y raws liberados. `ScoreChangeSet` enumera exclusivamente raws o
tracks cuyo score cambio.

## Lectura acotada

Extender `RawMapDatabase` solo con una API batch read-only por IDs si no existe
una equivalente. Debe obtener MPs, descriptores, observaciones y flags
necesarios sin copiar la base completa.

Las vistas de pose, fusion, covisibilidad y score tambien se limitan a KFs,
MPs, tracks y revisiones implicados. Los calculos costosos usan estructuras
privadas despues de liberar locks.

## Preparacion paso a paso

1. Revalidar que RANSAC fue aceptado y que la decision sigue siendo fusion.
2. Reunir todas las regiones compatibles; no detenerse en la primera.
3. Canonizar pares `(min(raw_a, raw_b), max(raw_a, raw_b))` y eliminar
   duplicados de la misma evidencia/revision.
4. Clasificar cada match por inlier, ambiguo, hard outlier o aislado.
5. Validar IDs activos, finitud, descriptor, uno-a-uno y revisiones.
6. Resolver en O(1) `SAME_RAW_ID` y `ALREADY_FUSED`.
7. Simular create/add/merge sobre estado privado y aplicar guard de dispersion.
8. Calcular medoid, posicion ponderada, procedencia y estado del track.
9. Crear refuerzos positivos de inliers.
10. Ejecutar visibilidad simetrica completa para hard outliers sobre inputs
    estructuralmente acotados.
11. Preparar arista `SERVER_LOOP_GEOMETRIC`, score fused y changesets.
12. Validar soporte final y producir `COMMIT`, `NO_OP`, `REJECT` o `STALE`.

No se repiten BoW, region building, descriptor matching ni RANSAC en esta rama.

## Nucleo de `FusedLandmarkManager`

Reimplementar conservando del enfoque validado previamente:

- indice raw a track O(1);
- union transitiva;
- ID estable;
- descriptor medoid;
- procedencia raw completa;
- representante y conteos exactos.

No conservar:

- mutacion live pareja por pareja;
- copias de bases completas;
- `live_state_mutex_` durante el algoritmo;
- publicacion o reconstruccion desde fusion;
- revisiones parciales o rollback implicito.

La union de tracks se calcula en un patch. El menor ID existente sobrevive y
el changeset retira los IDs absorbidos. Una incompatibilidad local rechaza solo
el par; se mantiene el resto si conserva soporte suficiente.

## Visibilidad y score eficiente

El modelo RANSAC aceptado entrega inliers y outliers sin una segunda busqueda.
Cada inlier crea dos eventos positivos de `+0.04`, sujetos a idempotencia.

Para negativos se construye un z-buffer sparse temporal por direccion. Solo se
proyectan puntos de las subnubes acotadas de 3O y se recorren completamente los
hard outliers/evidencias elegibles. Los eventos permitidos son:

```text
EXPECTED_VISIBLE_MISS        -0.01
FOREGROUND_CONTRADICTION     -0.03
```

Puntos ocluidos, fuera de imagen, detras de camara, ambiguos o sin expectativa
fiable no cambian. No existe corte por tiempo; la carga se limita mediante
`max_regions`, subnubes de unas 320 muestras, cola/coalescencia y backpressure.

Telemetria minima:

```text
visibility_regions_started/completed
projected_points_by_direction
visibility_time_avg/max
positive/negative/noop evidence counts
```

## Commit coordinado

Preparar todos los nuevos valores e inverse patches fuera de locks. Bajo un
coordinador breve como `state_commit_mutex_`:

1. revalidar revisiones raw, pose, fusion, covisibilidad y score relevantes;
2. aplicar el lote de tracks;
3. aplicar la covisibilidad geometrica;
4. aplicar score fused/positivo;
5. publicar una revision derivada coherente;
6. devolver `FusionChangeSet` y `ScoreChangeSet` exactos;
7. liberar el coordinador.

Si un paso falla antes de exponer la revision, aplicar el inverse patch y no
dejar estado parcial. No existe objetivo/timeout de 5 o 20 ms ni warning nuevo;
se conservan tiempos de telemetria para analizar pruebas.

Ante `fusion_dependencies_changed_before_commit`, query stale, fallo de revision
de fusion/covisibilidad/score o rollback:

1. terminar el intento actual sin estado parcial;
2. completar su lifecycle y liberar su identidad en la cola;
3. crear una `LoopTask` para el mismo KF con revisiones actuales;
4. encolarla al final del carril BAJA con un `task_id` nuevo;
5. repetir el pipeline completo, incluida visibilidad y scores.

No hay limite fijo. La deduplicacion impide mas de un retry equivalente y
`CreateLoopTasks()` cancela naturalmente el retry de un KF inexistente/inactivo/
`bad`. Un retry nunca conserva el coordinador entre intentos.

## Integracion con builder y servidor

Tras commit, el backend acumula los changesets en dirty sets exactos:

```text
tracks created/updated/retired
raw members hidden/released
raw/fused scores changed
```

El worker secundario no llama a `GlobalMapBuilder`, no solicita publicacion y
no despierta al principal. En el siguiente `PrimaryInput`, el builder:

- consulta tracks y scores actuales por ID;
- elimina slots de miembros raw ocultados;
- publica un representante por track;
- conserva todos los raws no fusionados;
- actualiza solo score si XYZ no cambio;
- publica todos los puntos sin umbral de score.

## Error alto y relacion con 3Q

En el runtime 3P, `LoopPipeline` termina una tarea de error alto despues de 3O:
no optimiza, no fusiona ni cambia score.

El acuerdo 3Q decide conservar la evidencia acotada en memoria dentro de la
`LoopTask` que obtiene el segundo apoyo. Tras `ACCEPT`, prepara esta fusion con
las poses candidatas y los mismos inliers, sin repetir BoW/RANSAC ni reencolar.
Si una revision raw invalida la evidencia, el intento termina stale y una BAJA
fresca recalcula la geometria completa. Una fusion omitida no revierte una
optimizacion valida.

## Visualizador y logs

Usar marcadores coherentes con el prefijo activo de Fase 3, incluyendo:

```text
F3P-PREPARE-BEGIN/END
F3P-PAIR-RESULT
F3P-VISIBILITY
F3P-COMMIT-BEGIN/END
F3P-TRACK-UNION
F3P-SCORE-COMMIT
F3P-TASK-RESULT
```

Los eventos contienen IDs, revisiones, conteos, motivos y tiempos, nunca nubes
ni listas masivas. El lifecycle visual de la misma tarea permanece activo de
decision a commit/finalizacion.

## Adaptabilidad durante ejecucion

Los umbrales iniciales son hipotesis configurables. Si tests o simulacion
demuestran falsos merges, coste excesivo o evidencia insuficiente, se ajustara
la implementacion dentro de estos invariantes. Una alternativa que cambie
ownership, atomicidad, scoring negativo o fronteras de fase exige suspender la
autorizacion y acordarla con el usuario.
