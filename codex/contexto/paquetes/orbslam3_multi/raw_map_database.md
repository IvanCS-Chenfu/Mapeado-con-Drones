# `RawMapDatabase`

## Rol

Autoridad exclusiva del estado ORB-SLAM3 crudo. La identidad de submapa es
siempre `(drone_id, map_epoch)` y ninguna optimización o pose world escribe en
esta base.

## Referencias

```text
orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp
  -> RawInsertResult / RecordedFiducialObservation
  -> RawCameraCalibration / RawFusionMapPointInput
  -> rg -n "RawInsertResult|RawCameraCalibration|RawFusionMapPointInput"

orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp
  -> RawMapDatabase
  -> rg -n "class RawMapDatabase|GetFusionMapPointInputs|GetCameraCalibration|GetActiveSubmapEntityIds|ReadRecordMetadata"

orbslam3_multi/src/raw_map_database.cpp
  -> InsertMap compartido / delta / full snapshot / record v3
  -> rg -n "RawMapDatabase::(InsertMap|InsertDelta|InsertFullSnapshot|AddFiducialObservation|SaveToPath|LoadRecord)"
```

## Contrato activo

- `InsertDelta()` exige `arrival_id` creciente y devuelve cambios precisos,
  incluidos `new_keyframe_ids` y cambios de pose.
- `InsertFullSnapshot()` compara el estado completo del submapa y separa KFs
  con cambios de pose, asociaciones o covisibilidad, y MPs con cambios de
  geometria, score, asociaciones o validez.
- Un KF/MP activo ausente del full snapshot se conserva pero pasa a
  inactivo/`is_bad`; no se borra su identidad ni su linaje.
- Un snapshot no-op no añade una entrada al journal. Uno material genera un
  delta normalizado con el mismo `arrival_id`, formado solo por entidades
  modificadas e invalidaciones, para mantener el record delta-only.
- El delta normalizado se construye desde un mensaje vacio que copia solo los
  metadatos de `OrbMap` y añade las entidades materiales. No se copia primero
  el snapshot completo; los deltas normales tampoco crean un segundo mensaje
  temporal.
- La clasificacion de una entidad existente compara por referencia const el
  valor raw almacenado y calcula todos los flags/diffs antes del commit. No
  realiza una copia profunda previa de cada KF o MP.
- Los conjuntos de IDs recibidos solo existen para `full_snapshot`, donde son
  `unordered_set` reservados al tamaño de entrada. Un delta normal no asigna ni
  rellena estructuras de pertenencia que nunca va a consultar.
- El diff de asociaciones de un KF construye una sola pareja de indices hash y
  obtiene añadidos/retirados en ambas direcciones. La salida se ordena y
  deduplica para conservar resultados deterministas sin cuatro arboles
  temporales.
- Conserva KFs, MPs, asociaciones, covisibilidad y revisiones. El journal raw
  tiene tres modos fijados antes del primer delta: residente para tests/guardado
  manual, deshabilitado sin record, o stream incremental v3 en live.
- `GetLoopSemanticRevision()` separa cuatro autoridades: revision raw
  diagnostica, apariencia BoW, geometria semantica de scheduling y
  `validation_revision` exacta. La geometria semantica solo cambia al cruzar
  celdas de pose de 0.50 m/0.10 en cuaternion, estados de madurez respecto a
  `min_query_mappoints` o presencia de covisibilidad fuerte. La validacion
  exacta conserva pose, IDs, posiciones, descriptores y pesos para impedir
  usar un calculo obsoleto en el commit.
- No conoce anchors ni llama a `GlobalPoseStore`.
- Expone KFs concretos y snapshots de pose por submapa para que el backend
  coordine el anclaje sin reinterpretar deltas en el servidor.
- `GetActiveSubmapEntityIds()` devuelve solo IDs activos de KFs y MPs bajo un
  unico lock. El builder la usa una vez al producirse el primer anchor para un
  backfill completo, incluidos MPs no enumerados por un KF.
- `GetMapPointScoreInputs()` extrae en batch, bajo un unico lock, solo contador,
  ratio, validez del descriptor e `is_bad`; no copia geometria ni observaciones.
- `GetFusionMapPointInputs()` extrae en un unico batch acotado la geometria,
  descriptor, KF de referencia, observadores y revisiones que 3P necesita para
  preparar tracks fuera de locks live.
- `GetCameraCalibration()` conserva la calibracion pinhole raw por submapa para
  la evaluacion sparse simetrica de visibilidad; no genera un depth map ni
  altera el mensaje ORB recibido.
- `GetBuilderSnapshot()` expande asociaciones de los KFs dirty y devuelve bajo
  un unico lock solo poses/validez de KFs y geometria/referencia/IDs observadores
  de MPs. Excluye keypoints, BoW, feature vectors, descriptores y metadatos que
  el builder no consume.
- `AddFiducialObservation()` conserva observaciones ya normalizadas asociadas a
  un KF y `arrival_id`; no almacena muestras GT del ring.
- El stream incremental serializa cada delta normalizado una sola vez en
  `<path>.in_progress`, mantiene `resident_entries=0`, añade al cierre el
  pequeño journal fiducial, actualiza el contador y reemplaza el destino por
  `rename`. Un record previo permanece intacto hasta finalizar correctamente.
- El record version 3 añade `fiducial_visit_id` a cada observacion y mantiene el
  payload raw delta-only. `LoadRecord()` acepta versiones 1, 2 y 3; en v1/v2 el
  campo se inicializa a cero y el servidor lo infiere temporalmente.
- `ReadRecordMetadata()` hace una primera pasada que salta payloads y carga las
  observaciones fiduciales pequeñas. `StreamRecordEntries()` reabre el archivo
  y deserializa una sola entrada cada vez; `LoadRecord()` conserva su API
  histórica construyendo el vector sobre esas primitivas.
- Sin grabacion, el servidor usa modo deshabilitado: conserva contadores
  lógicos para telemetría pero no retiene `shared_ptr<OrbMap>` históricos.

`test/test_raw_map_database.cpp` cubre diez casos, incluidos diff selectivo de
full snapshot, invalidacion por ausencia, preservacion de todos los metadatos y
equivalencia del delta normalizado al reproducirlo, ademas de orden determinista
del diff de asociaciones con entradas no ordenadas, stream v3 sin retencion,
  reemplazo final del record previo, compatibilidad v1/v2/v3,
  `fiducial_visit_id` y journal deshabilitado.
`test/test_loop_pipeline.cpp` verifica ademas que refinamientos internos de MPs
no cambian la huella de scheduling, aunque si cambian la validacion exacta, y
que una nube al pasar a estado denso vuelve a habilitar trabajo.
