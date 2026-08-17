# Subfase 3F - Contrato del flujo principal y publicacion incremental

## Estado

```text
CONSEGUIDA; implementado y validado tecnica y visualmente
Contrato subordinado a subfase_3F_*.md
```

La evidencia legacy de publicaciones se conserva en el historial, pero no
valida la arquitectura anterior: su captura global bajo mutex llego a superar
27 segundos.

## Propiedad funcional

La publicacion forma parte del final serial de cada `PrimaryTask` con cambios
publicables. No existe publication worker, request global, timer de
reconciliacion pesado ni ACK de RViz2.

```text
PrimaryTask
  -> commit raw
  -> score base
  -> poses/anchor
  -> drenar dirty sets pendientes
  -> GlobalMapBuilder actualiza caches
  -> serializar cloud+KFs coherentes
  -> publish ROS
  -> task COMPLETED
```

El flujo principal sigue avanzando aunque el worker secundario este vacio,
saturado o ejecutando una tarea lenta. No espera BoW, matching, RANSAC, fusion,
solver, HTML, navegador ni confirmacion visual.

## Cambio material y skip

Una llegada puede activar callbacks o aristas de ingesta sin producir una
revision publica. Solo se construye/publica cuando los change sets alteran:

- KFs o MPs visibles;
- anchor/pose world de un KF visible;
- score de un MP visible;
- en el futuro, membresia o representante fusionado visible;
- eliminacion/invalidez de una entidad visible.

Con dirty sets vacios o cambios solo de metadata no publica:

```text
[F3F-BUILDER-SKIP] reason=no_public_dirty
```

## Primer anchor

El primer anchor se procesa dentro de la misma `PrimaryTask` que lo detecta.
Marca todo el submapa como dirty, hace backfill y publica antes de terminar esa
tarea. No espera un delta posterior ni un worker auxiliar.

## Commits secundarios futuros

Una tarea secundaria aceptada realiza un commit breve y atomico, y deja:

```text
IDs afectados + revision de autoridad
```

Esos IDs se acumulan en dirty sets thread-safe. El commit:

- no llama a `GlobalMapBuilder`;
- no serializa ROS;
- no publica;
- no despierta al `PrimaryWorker`;
- no espera a RViz2/web;
- puede terminar y dejar que empiece la siguiente tarea secundaria.

La siguiente entrada principal drena los IDs y publica una revision que incluye
los commits aceptados disponibles hasta su frontera de captura.

La reconciliacion por full snapshot de 3G adopta deliberadamente esta misma
semantica aunque el `SnapshotInput` se serialice en la cola principal: actualiza
solo las autoridades afectadas, acumula dirty y termina sin invocar el builder.
El siguiente `DeltaInput` normal drena esos IDs. Si no llega otro delta, la
revision publicada permanece sin cambios.

## Coherencia

`GlobalMapBuildResult` fija una combinacion coherente de:

```text
raw_revision
pose_revision
score_revision
fusion_revision opcional
publication_revision
timestamp
```

La nube y los KFs se derivan del mismo resultado. Una revision vieja nunca
reemplaza otra ya publicada. El builder consulta por ID las autoridades y no
compone snapshots completos de todas las bases.

## Backpressure

La construccion y serializacion pertenecen al coste natural de la cola
principal. Se conserva la histeresis de 3C:

```text
high watermark = 8
low watermark  = 2
```

El goal activo termina; mientras el flag siga activo no se envia el siguiente.
3F no introduce otra cola ni otra causa de bloqueo.

## Observabilidad

Cada revision publica comparte `flow_id` entre:

```text
RawMapDatabase -> LandmarkScoreManager
LandmarkScoreManager -> GlobalMapBuilder
RawMapDatabase -> GlobalMapBuilder
GlobalPoseStore -> GlobalMapBuilder
GlobalMapBuilder -> GlobalMapServer
GlobalMapServer -> RViz2 cloud
GlobalMapServer -> RViz2 keyframes
```

Las aristas solo se activan si ocurre su evento. El frontend web no inventa
actividad periodica ni participa en scheduling/backpressure.

## Pruebas deterministas minimas

1. Un delta no material termina sin build/publish.
2. El primer anchor publica el backfill en su propia tarea.
3. Un KF movido recoloca solo sus puntos en la siguiente tarea principal.
4. Un score modificado actualiza solo sus puntos en la siguiente tarea.
5. Un commit secundario puede terminar sin publicacion inmediata.
6. Nube y KFs comparten revision y timestamp.
7. Desconectar RViz2/web no altera colas ni bases.
8. `PrimaryWorker max_active=1` se conserva.

## Errores anteriores prohibidos

- `PublicationSnapshot` global compuesto bajo mutex.
- `RequestGlobalMapPublication` desde commits secundarios.
- timer que escanea bases para descubrir cambios.
- coalescing en un worker de publicacion separado.
- estado `AWAITING_VISUAL_ACK`.
- mutex live durante build, serializacion o publish.
