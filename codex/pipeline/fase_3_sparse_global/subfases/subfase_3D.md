# Subfase 3D — backend incremental y `GlobalPoseStore` ligero

## Estado vigente
```text
PARCIAL - IMPLEMENTADA Y VALIDADA TECNICAMENTE; PENDIENTE VALIDACION VISUAL
```
La capa de poses globales se ha rehecho sobre el flujo principal de 3C mediante
`SparseGlobalBackend` y un `GlobalPoseStore` vacio hasta que exista un anchor,
sin publicar resultados espaciales en ROS 2. Las pruebas automáticas, replay y
live cumplen; falta incorporar la observación del usuario en RViz2 y la web.

## Contrato acordado para la reimplementacion
### Propiedad y coordinacion
- `SparseGlobalBackend`, en `orbslam3_multi`, posee y coordina
  `RawMapDatabase` y `GlobalPoseStore`.
- `GlobalMapServer` conserva subscriptions, `PrimaryQueue`, `PrimaryWorker`,
  backpressure, replay y telemetria; no decide anchors ni transforma poses.
- `RawMapDatabase` sigue siendo autoridad exclusiva del estado ORB-SLAM3 raw y
  no conoce anchors ni llama directamente a `GlobalPoseStore`.
- `GlobalPoseStore` es la unica autoridad sobre anchors y poses world; raw no
  duplica el listado de submapas anclados.
- No hay mensajes ROS entre ambas bases: el backend mueve resultados ligeros
  dentro del mismo proceso y evita dependencias circulares.

```text
PrimaryWorker -> SparseGlobalBackend::InsertDelta -> RawMapDatabase
              -> GlobalPoseStore::ApplyRawPoseChanges, si corresponde
```

### Resultado raw preciso y rama condicional

`RawMapDatabase::InsertDelta()` debe distinguir, ademas de sus cambios raw:

- KFs nuevos;
- KFs cuya pose local realmente ha cambiado;
- KFs invalidados por ORB-SLAM3 mediante `is_bad=true`;
- actualizaciones exclusivas de asociaciones o covisibilidad.

El resultado para poses transporta solo ID compuesto, revision raw, pose local
y tipo de cambio. No copia descriptores, BoW, keypoints, observaciones,
MapPoints ni el mensaje original completo.

Los cambios exclusivos de MapPoints, asociaciones o covisibilidad terminan tras
el commit raw, sin llamar a `GlobalPoseStore` ni iluminar su arista. KFs nuevos,
cambios reales de pose local e invalidaciones sí extienden la misma
`PrimaryTask` hasta poses. No se crea otro worker.

### Anchors y poses

- `GlobalPoseStore` empieza completamente vacio aunque raw ya contenga datos.
- Un cambio de KF de un submapa sin anchor devuelve estado `UNANCHORED` y no
  crea placeholders ni poses provisionales.
- En 3D solo se permite un anchor sintetico explicito para tests y replay. El
  primer anchor real procedente de fiduciales pertenece a 3E.
- Al anclar, el backend obtiene de raw una captura acotada y versionada de todos
  los KFs del submapa y la entrega con `world_T_local` a un commit atomico.
- Para un KF derivado se calcula inicialmente
  `world_T_kf = world_T_local * local_T_kf`.

- Una pose derivada puede reconciliarse si cambia su pose raw. Una pose global
  aceptada por una fuente de mayor autoridad no puede degradarse mediante una
  actualizacion raw silenciosa.
- Un KF `is_bad` conserva revision, pose y procedencia, pero queda
  `active=false`; futuras consultas activas y publicaciones deben excluirlo.

Cada pose guarda `pose_revision`, `source_kind`, `source_commit_id`,
`source_task_id`, `parent_commit_id` y `base_raw_revision`. Cada lote devuelve
un `PoseChangeSet` con IDs, revisiones, cambios y omisiones.

3D implementa commits atomicos y versionados. Los backups y el rollback de una
optimizacion rechazada pertenecen a 3K/3L; no deben adelantarse aqui.

### Grafo web y RViz2

- Añadir el vertice `GlobalPoseStore`.
- Añadir la arista `RawMapDatabase -> GlobalPoseStore`.
- Activarla solo ante KFs nuevos, poses locales modificadas o invalidaciones.
- Mostrar resultado, cantidades, revisiones y estado `UNANCHORED` sin enviar
  payloads ORB pesados al visualizador.
- RViz2 debe permanecer vacio durante toda 3D: no se crean publishers,
  marcadores, nubes ni poses globales ROS.

### Validacion acordada

- Tests deterministas para store vacio, rama sin cambios de pose, submapa no
  anclado, anchor sintetico, altas, cambios, `is_bad`, autoridad y atomicidad.
- Replay de `rawdb_prueba_85.record` por la misma cola y worker de 3C, con un
  anchor sintetico explicito para verificar poses world reproducibles.
- Simulacion live con el recorrido usado en 3C y sin anchor sintetico: el store
  permanece sin poses, el grafo muestra la rama condicional y RViz2 no muestra
  ningun resultado global.
- Compilar los tres paquetes y verificar que estadisticas y journal raw no
  cambian por introducir la capa de poses.

### Fuera de alcance

No incluir fiduciales reales, `GlobalMapBuilder`, snapshots completos,
publicacion espacial, worker secundario, loops, solver, optimizacion, backups o
rollback. Tampoco copiar en bloque el `GlobalPoseStore` de la baseline anterior.

### Trabajo anterior que se conserva como referencia

- separación entre raw local y poses globales;
- anchors, poses world y la distincion entre pose derivada y aceptada;
- reconciliación que ya distingue `keep_optimized`, `keep_accepted` y
  `derived_tail`.

### Implementacion anterior incorrecta que no debe repetirse

- copiar todo `GlobalPoseStore` como snapshot de publicación o loop;
- representar autoridad solo mediante flags insuficientes para saber qué
  commit sustituyó a otro;
- permitir que una reconciliación raw pueda degradar una pose aceptada;
- escribir lotes parciales visibles o publicar una revisión antigua después de
  otra nueva;
- hacer que esta subfase mantenga la nube: `GlobalMapBuilder` pertenece a `3F`.

## Estado histórico anterior

Las secciones posteriores se conservan como contrato/evidencia de la
implementación anterior. Si contradicen el bloque REHACER de esta cabecera,
prevalece el contrato de reimplementación nuevo.

```text
realizado
```

---

Realizada el 2026-07-08 con conclusión `CONSEGUIDA`.

Evidencia principal:

- `orbslam3_multi` y `orbslam3_server` compilaron con `BUILD-EXIT-CODE 0`;
- replay `prueba_1` sin Gazebo con `SIM-DONE prueba=1 success=true`;
- `GlobalPoseStore` empezó vacío:
  ```text
  [F1D-POSESTORE-INIT] anchors=0 world_poses=0 optimized_kfs=0 corrections=0
  ```
- anchor sintético debug:
  ```text
  [F1D-POSESTORE-ANCHOR-SUMMARY]
  ```
- pose optimizada sintética y corrección:
  ```text
  [F1D-POSESTORE-OPT-POSE-SET]
  [F1D-POSESTORE-CORRECTION-SET]
  ```
- KFs nuevos posteriores heredaron la última corrección del mismo submapa:
  ```text
  [F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT]
  ```
- replay completo:
  ```text
  [F1C-REPLAY-DONE] entries=284 journal=284 deltas=284 full=0 submaps=3 kfs=142 mps=17884
  ```

El anchor de `3D` es sintético/debug. El primer anchor real queda para `3E`.

### Contrato runtime vigente de `GlobalPoseStore`

`GlobalPoseStore` es el nombre definitivo de la base conceptual
`kf_pose_db`. No se introduce una segunda clase que duplique poses globales.
Es la unica autoridad para:

- estado anclado/no anclado de `(drone_id, map_epoch)`;
- anchors fiduciales aceptados;
- pose world vigente de cada KF;
- poses optimizadas y propagadas;
- ultimo control/anchor activo desde el que extender KFs futuros;
- revisiones de pose, backup transaccional y rollback excepcional.

El servidor le entrega los KFs afectados por cada `ChangeSet`. Para un submapa
ya anclado, los KFs nuevos se registran inmediatamente mediante la referencia
global aceptada mas reciente, sin esperar BoW, fusion u optimizacion. Para un
submapa sin anchor, los IDs permanecen raw/no publicables; al recibir el primer
anchor, el servidor obtiene de `RawMapDatabase` la captura de todos sus KFs y
la entrega en un unico lote de anclaje.

Una tarea secundaria no modifica poses una a una durante el solver. Construye
un candidato privado y llama a una API de commit de lote que:

1. valida las revisiones capturadas;
2. escribe de forma atomica todas las poses aceptadas;
3. actualiza el ultimo control/campo de correccion para KFs posteriores;
4. incrementa una unica revision de pose;
5. devuelve un `PoseCommitResult` al servidor.

El commit es el final de la tarea. La publicacion posterior pertenece a `3F` y
no forma parte del ciclo de vida del worker. `GlobalPoseStore` debe exponer
snapshots inmutables/versionados para `GlobalMapBuilder` y tareas secundarias;
ningun consumidor mantiene un mutex de poses durante un calculo largo.

La propagacion inmediata de KFs futuros sustituye la necesidad de una
`PostOptimizationKeyFrameQueue` especial. `3R` queda absorbida por este
contrato y por la admision normal de `LoopTask`.

## Objetivo tecnico

Crear en `orbslam3_multi` una clase ligera llamada `GlobalPoseStore` encargada de guardar y consultar la pose global actual de los `KeyFrames` que ya pertenecen a submapas anclados o corregidos.

Esta subfase separa definitivamente dos capas:

```text
RawMapDatabase
    Datos crudos de ORB-SLAM3.
    Poses locales originales.
    KeyFrames, MapPoints, BoW, observaciones, covisibilidad, etc.

GlobalPoseStore
    Estado global ligero.
    Anchors de submapas.
    Pose world asociada a cada KeyFrame anclado/corregido.
    Poses aceptadas por el servidor, que ORB-SLAM3 raw no puede mover por si solo.
    Anchors de cola para extender KFs futuros desde el ultimo KF global aceptado.
```

`GlobalPoseStore` debe empezar vacía aunque `RawMapDatabase` ya tenga datos. Solo debe poblarse cuando una clase externa, ahora en modo prueba/controlado y en subfases posteriores mediante fiduciales u optimización, invoque métodos de anclaje o corrección. Una vez que una pose global queda aceptada en esta capa, la pose raw mutable de ORB-SLAM3 pasa a ser solo una coordenada local de referencia, no la autoridad para mover el mapa global.

La subfase debe ser verificable mediante build y logs. No se exige mapa global visible en RViz2 en esta subfase, aunque se permite publicar marcadores debug de poses si el servidor nuevo ya tiene infraestructura simple para ello.

---

## Contexto obligatorio a leer

- `AGENTS.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/pipeline/PIPELINE_MAESTRO.md`
- `codex/pipeline/fase_3_sparse_global/pipeline_fase_3.md`
- `codex/pipeline/fase_3_sparse_global/subfases/subfase_3A.md`
- `codex/pipeline/fase_3_sparse_global/subfases/subfase_3B.md`
- `codex/pipeline/fase_3_sparse_global/subfases/subfase_3C.md`
- historial reciente de Fase 3
- documentación de paquetes afectados:
  - `codex/contexto/paquetes/orbslam3_multi/`
  - `codex/contexto/paquetes/orbslam3_server/`
- ADRs relacionados con:
  - identidad `(drone_id, map_epoch)`;
  - separación servidor/backend;
  - fiduciales como observaciones absolutas;
  - no sobrescribir poses locales raw;
  - optimización local y futura propagación de correcciones.

---

## Diagnostico de partida

Tras la subfase 3C debe existir una `RawMapDatabase` en `orbslam3_multi` que reciba del servidor los datos crudos enviados por los wrappers ORB-SLAM3.

Esa base raw debe guardar información por:

```text
submapa = (drone_id, map_epoch)
```

pero todavía no debe existir una capa clara, ligera y separada que responda:

```text
¿Dónde está este KeyFrame en world según el sistema global actual?
```

El problema que se prepara para resolver, aunque no se optimiza todavía en esta subfase, es el siguiente:

1. Un submapa puede anclarse con una transformación rígida `world_T_local`.
2. Esa transformación permite calcular una pose global inicial para todos sus KFs:

   ```text
   world_T_kf = world_T_local * local_T_kf
   ```

3. Más adelante, una optimización del servidor podrá mover algunos KFs
   concretos y esas poses globales pasarán a ser autoritativas.
4. ORB-SLAM3 puede seguir modificando sus poses raw mediante optimizaciones
   internas o full snapshots, pero esos cambios no deben mover por sí solos las
   poses globales ya aceptadas por el servidor.
5. Si luego llegan KFs nuevos del mismo submapa, no deben heredar una
   transformación absoluta antigua del submapa. Deben extender la trayectoria
   desde el último KF global aceptado usando la relación raw actual:

   ```text
   T_world_new = T_world_ref_accepted * inverse(T_raw_ref_current) * T_raw_new_current
   ```

   donde `ref` es el último KF anterior con pose global aceptada.

En esta subfase no se implementa la optimización real, pero sí se deja preparada la estructura para que otras clases de `orbslam3_multi` puedan:

- anclar submapas;
- registrar poses optimizadas;
- consultar poses globales actuales;
- extender KFs nuevos desde el último KF global aceptado sin modificar
  `RawMapDatabase`.

---

## Archivos permitidos a modificar

### `orbslam3_multi`

Crear o modificar, según exista la estructura real del paquete:

- `orbslam3_multi/include/orbslam3_multi/global_pose_store.hpp`
- `orbslam3_multi/src/global_pose_store.cpp`
- `orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp`
- `orbslam3_multi/src/raw_map_database.cpp`
- `orbslam3_multi/include/orbslam3_multi/multi_drone_system.hpp`
- `orbslam3_multi/src/multi_drone_system.cpp`
- `orbslam3_multi/CMakeLists.txt`
- `orbslam3_multi/package.xml` solo si fuese estrictamente necesario

`raw_map_database.*` solo puede tocarse para añadir consultas mínimas que `GlobalPoseStore` necesite, por ejemplo:

- obtener IDs de KFs de un submapa;
- obtener pose local de un KF;
- obtener submapa de un KF;
- obtener parent/covisibilidad si ya existe y se usa para elegir corrección heredada.

### `orbslam3_server`

Modificar solo para integrar y probar `GlobalPoseStore` desde el servidor nuevo:

- `orbslam3_server/src/global_map_server.cpp`
- launch activo nuevo del servidor, si hace falta añadir parámetros debug/replay
- `orbslam3_server/CMakeLists.txt` solo si fuese necesario

### Archivos auxiliares y pruebas

- `codex/archivos_auxiliares/trayectorias/tray_prueba_1.yaml`
- dataset raw/replay generado en 3C, si existe:

  ```text
  codex/archivos_auxiliares/repeticiones/rawdb_record_*.json
  codex/archivos_auxiliares/repeticiones/rawdb_record_*.bin
  ```

- documentación relacionada en `codex/contexto/paquetes/`
- historial de la fase/subfase

---

## Archivos prohibidos

No modificar en esta subfase:

- `ORB_SLAM3/`
- `orbslam3_ros2/` wrapper
- `orbslam3_msgs/` mensajes y servicios
- lógica de detección real de fiduciales
- lógica real de loops/subnubes/RANSAC
- fusión de landmarks
- solver/optimización real
- publicación final de mapa global fused
- referencias historicas recuperables desde `1b96a7a`, salvo consulta

No modificar manualmente:

- `build/`
- `install/`
- `log/`

---

## Funciones, clases o nodos que hay que localizar

`planificador_fase` debe localizar antes de implementar:

- clase nueva o punto de integración existente:

  ```text
  GlobalPoseStore
  ```

- base raw creada en 3C:

  ```text
  RawMapDatabase
  ```

- fachada/backend si existe:

  ```text
  MultiDroneSystem
  ```

- servidor nuevo creado en 3B:

  ```text
  global_map_server.cpp
  ```

- punto del `PrimaryWorker` donde el `RawInsertResult` de 3C se entrega a la
  etapa de poses; el callback ROS nunca llama directamente a `RawMapDatabase`.
- mecanismo de replay creado en 3C, si existe.
- tipos de identificador ya definidos para:

  ```text
  drone_id
  map_epoch
  SubmapId
  GlobalKeyFrameId
  GlobalMapPointId
  ```

No inventar nombres si ya existen tipos equivalentes. Reutilizar los tipos existentes en `orbslam3_multi` siempre que no rompa claridad ni build.

---

## Cambios requeridos

### 1. Crear `GlobalPoseStore` como clase ligera

Intención:

Crear una clase en `orbslam3_multi` que guarde únicamente estado global ligero.

Debe guardar como mínimo:

```text
submap_anchors:
    submap_id -> world_T_local, source, metadata minima

keyframe_world_poses:
    kf_id -> world_T_kf, source

optimized_keyframes:
    kf_id -> optimized_world_T_kf, correction_T_kf, source

submap_last_correction:
    submap_id -> correction_T_latest
```

Actualización de diseño vigente:

`submap_last_correction` fue una simplificación inicial. Para trayectorias
largas con optimizaciones raw internas de ORB-SLAM3, la estructura correcta debe
representar un anchor de extensión o cadena aceptada:

```text
accepted_keyframe_anchors:
    submap_id -> ordered accepted KFs
    cada entrada: kf_id, accepted_world_T_kf, source, authority
active_tail_anchor:
    submap_id -> último KF desde el que se extienden KFs futuros
```

La pose raw puede cambiar; la pose world aceptada permanece bajo autoridad del
servidor.

Condición de seguridad:

No guardar en `GlobalPoseStore` datos grandes como descriptores, BoW, keypoints, MapPoints completos u observaciones. Si necesita esa información, debe consultarla en `RawMapDatabase`.

Logs requeridos:

```text
[F1D-POSESTORE-INIT]
[F1D-POSESTORE-STATS]
```

---

### 2. Añadir método para anclar un submapa

Método esperado, ajustando nombres a los tipos existentes:

```cpp
SetSubmapAnchor(submap_id, world_T_local, source)
```

o método equivalente:

```cpp
AnchorSubmap(submap_id, world_T_local, raw_db, source)
```

Comportamiento:

1. Recibir un `submap_id` y una transformación `world_T_local`.
2. Consultar a `RawMapDatabase` los IDs de todos los KFs existentes en ese submapa.
3. Para cada KF:

   ```text
   world_T_kf = world_T_local * local_T_kf
   ```

4. Guardar `world_T_kf` asociada al ID global del KF.
5. Marcar el submapa como anclado.

Condición de seguridad:

No escribir ni modificar la pose local original guardada en `RawMapDatabase`.

Logs requeridos:

```text
[F1D-POSESTORE-ANCHOR-REQUEST]
[F1D-POSESTORE-ANCHOR-SET]
[F1D-POSESTORE-ANCHOR-KF-POSE]
[F1D-POSESTORE-ANCHOR-SUMMARY]
```

Ejemplo de log:

```text
[F1D-POSESTORE-ANCHOR-SUMMARY] drone_id=1 epoch=0 source=DEBUG_TEST anchored_kfs=42 world_t=(2.000,0.000,0.000) yaw=1.570
```

---

### 3. Añadir consultas de pose global

Métodos esperados:

```cpp
HasSubmapAnchor(submap_id) const
HasWorldPose(kf_id) const
GetWorldPose(kf_id) const
GetPoseStoreStats() const
GetSubmapPoseStats(submap_id) const
```

Comportamiento:

- `HasWorldPose(kf_id)` debe ser `true` solo si el KF ya tiene pose global registrada.
- KFs de submapas no anclados no deben aparecer como publicables.
- `GetWorldPose(kf_id)` no debe inventar una pose global si no hay anchor o pose optimizada.

Logs requeridos:

```text
[F1D-POSESTORE-GET-POSE]
[F1D-POSESTORE-MISSING-POSE]
```

Los logs de consulta deben estar limitados o throttled para no saturar la simulación.

---

### 4. Registrar KFs nuevos de un submapa ya anclado

Método esperado:

```cpp
OnNewKeyFrameInserted(kf_id, raw_db)
```

o equivalente:

```cpp
RegisterNewKeyFrameIfAnchored(kf_id, raw_db)
```

Comportamiento:

1. Consultar en `RawMapDatabase` a qué submapa pertenece el KF.
2. Si el submapa no está anclado:

   ```text
   no crear pose global
   ```

3. Si el submapa está anclado y no hay ningún KF aceptado anterior más
   específico:

   ```text
   world_T_kf = world_T_local * local_T_kf
   ```

4. Si existe un KF global aceptado anterior dentro del mismo submapa:

   ```text
   T_raw_ref_current = world_T_local * local_T_ref_actual
   T_raw_kf_current  = world_T_local * local_T_kf_actual
   T_world_kf = T_world_ref_accepted * inverse(T_raw_ref_current) * T_raw_kf_current
   ```

Esta regla sustituye a la herencia global simple cuando haya KFs aceptados por
fiducial u optimización. Conserva coherencia local aunque ORB-SLAM3 haya
reoptimizado las poses raw entre `ref` y el KF nuevo.

Condición de seguridad:

La corrección heredada solo debe aplicarse dentro del mismo submapa.

Logs requeridos:

```text
[F1D-POSESTORE-NEW-KF-UNANCHORED]
[F1D-POSESTORE-NEW-KF-POSE-SET]
[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT]
```

Ejemplo:

```text
[F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT] drone_id=1 epoch=0 kf=57 correction_source=LAST_SUBMAP_CORRECTION dx=0.420 dy=-0.030 dyaw=0.080
```

---

### 5. Registrar poses optimizadas sin ejecutar optimización real

Método esperado:

```cpp
SetOptimizedKeyFramePose(kf_id, optimized_world_T_kf, raw_db, source)
```

Comportamiento:

1. Recibir la pose final optimizada en `world` de un KF.
2. Consultar `RawMapDatabase` para obtener:

   ```text
   local_T_kf
   submap_id
   ```

3. Obtener `world_T_local` del submapa.
4. Calcular la pose rígida base:

   ```text
   raw_world_T_kf = world_T_local * local_T_kf
   ```

5. Calcular la corrección:

   ```text
   correction_T_kf = optimized_world_T_kf * inverse(raw_world_T_kf)
   ```

6. Guardar:

   ```text
   optimized_world_T_kf
   correction_T_kf
   source
   ```

7. Registrar el KF como punto aceptado/autoritativo para que pueda servir como
   referencia de extensión de KFs posteriores:

   ```text
   accepted_keyframe_anchors[submap_id].push(kf_id, optimized_world_T_kf)
   ```

8. Solo si se mantiene compatibilidad temporal, actualizar
   `submap_last_correction`; esa corrección no debe ser la autoridad principal
   para KFs futuros.

Condición de seguridad:

En esta subfase la optimización real no existe. Este método se prueba con una corrección sintética/debug para comprobar que la estructura funciona.

Logs requeridos:

```text
[F1D-POSESTORE-OPT-POSE-SET]
[F1D-POSESTORE-CORRECTION-SET]
[F1D-POSESTORE-CORRECTION-STATS]
```

---

### 6. Integrar `GlobalPoseStore` con el servidor nuevo solo para prueba controlada

El servidor nuevo debe poder activar un modo debug de 3D para validar la clase con datos reales/replay de 3C.

Parámetros orientativos, adaptables a la estructura real del launch:

```text
pose_store_debug_enabled
pose_store_debug_anchor_after_deltas
pose_store_debug_anchor_drone_id
pose_store_debug_anchor_epoch
pose_store_debug_anchor_world_x
pose_store_debug_anchor_world_y
pose_store_debug_anchor_world_z
pose_store_debug_anchor_yaw
pose_store_debug_opt_enabled
pose_store_debug_opt_after_deltas
pose_store_debug_opt_kf_id
pose_store_debug_opt_dx
pose_store_debug_opt_dy
pose_store_debug_opt_dz
pose_store_debug_opt_dyaw
```

Si existen mecanismos mejores en el servidor nuevo, pueden usarse, pero deben quedar documentados.

Flujo de prueba esperado:

1. Cargar o recibir deltas en `RawMapDatabase`.
2. Tras cierto número de deltas o cuando exista al menos un submapa con KFs, invocar un anchor sintético:

   ```text
   SetSubmapAnchor(..., source=DEBUG_TEST)
   ```

3. Comprobar que se crean poses globales para los KFs existentes.
4. Aplicar una pose optimizada sintética a uno o varios KFs:

   ```text
   SetOptimizedKeyFramePose(..., source=DEBUG_TEST_OPT)
   ```

5. Insertar más deltas por replay.
6. Comprobar que los KFs nuevos del mismo submapa heredan la última corrección simple.

Logs requeridos del servidor:

```text
[F1D-SERVER-DEBUG-ANCHOR]
[F1D-SERVER-DEBUG-OPT]
[F1D-SERVER-POSESTORE-STATS]
```

---

### 7. Documentar la limitación de la corrección heredada simple

Debe quedar escrito en documentación e historial:

```text
La política original de 3D para KFs futuros tras optimización aplicaba la última
corrección conocida del mismo submapa. Esa política queda obsoleta para la ruta
fiducial actual: puede romper la trayectoria si ORB-SLAM3 optimiza poses raw y
los KFs nuevos se colocan con una corrección calculada sobre raw antiguo.

La política implementada desde la revisión `3K` del 2026-07-24 extiende cada
KF nuevo desde el `active_tail_anchor` aceptado y lo conserva como
`derived_tail` recalculable:

T_world_new = T_world_ref_accepted * inverse(T_raw_ref_current) * T_raw_new_current
```

---

## Cambios prohibidos

No hacer en esta subfase:

- no detectar fiduciales reales;
- no usar ground truth para construir mapa;
- no calcular `world_T_local` desde fiducial real;
- no implementar `FiducialAnchorManager`;
- no implementar loops por BoW/subnube;
- no implementar fusión de landmarks;
- no ejecutar ni migrar el solver real de optimización;
- no publicar mapa global fused final;
- no modificar la semántica raw de `RawMapDatabase`;
- no sobrescribir poses locales originales de ORB-SLAM3;
- no renombrar ni limpiar legacy masivamente;
- no dividir deltas de replay por KeyFrame si 3C definió replay por delta completo;
- no mover lógica al servidor que deba vivir en `orbslam3_multi`, salvo parámetros/debug de integración.

---

## Paquetes a compilar

Comando esperado:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
```

Si cambios reales en dependencias obligan a compilar más paquetes, `implementador_fase` puede añadirlos justificándolo en el historial.

Si falla el build:

```bash
./codex/herramientas/reduce_build_log.sh
```

Después, diagnosticar el primer error real y corregir lo mínimo.

---

## Pruebas requeridas

### Prueba 1 — Replay de RawMapDatabase con anchor y corrección sintéticos

Esta es la prueba principal de 3D.

Debe usar un dataset guardado en 3C. Si no existe ningún dataset compatible, Codex debe generar uno ejecutando una prueba corta de 3C antes de esta prueba, documentándolo en historial.

Dataset esperado, nombre orientativo:

```text
codex/archivos_auxiliares/repeticiones/rawdb_record_prueba_simple.json
```

o equivalente real generado por 3C.

Flujo esperado:

1. arrancar el servidor nuevo en modo replay 3C;
2. cargar el dataset raw con deltas ordenados por `arrival_id`;
3. crear una `RawMapDatabase` nueva vacía;
4. reinsertar deltas con timer;
5. tras varios deltas, ejecutar un anchor sintético sobre un submapa existente;
6. comprobar que `GlobalPoseStore` crea poses para KFs existentes del submapa;
7. aplicar una corrección/pose optimizada sintética sobre un KF;
8. continuar el replay;
9. comprobar que KFs nuevos del mismo submapa reciben pose global heredando la corrección simple;
10. terminar cuando se reproduzcan suficientes deltas para validar estadísticas.

Comando orientativo si el replay se integra en el launch normal:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 1 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Si el modo replay no necesita Gazebo, la subfase debe documentar el comando exacto usado para ejecutar el nodo/launch de replay y conservar logs en:

```text
codex/archivos_auxiliares/logs/prueba_1.log
codex/archivos_auxiliares/logs/prueba_1.reduced.log
```

Observación esperada en RViz2:

```text
No obligatoria en 3D.
Opcionalmente, si se implementan markers debug, visualizar poses globales de KFs anclados/corregidos.
```

### Prueba 2 — Simulación corta de compatibilidad, solo si no existe dataset 3C

Esta prueba solo es necesaria si no hay dataset raw/replay generado en 3C.

YAML:

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_2.yaml
```

Secuencia mínima:

1. esperar arranque;
2. mover ambos drones hacia el fiducial 2 a alturas distintas;
3. mover un dron ligeramente en X;
4. mover el segundo dron de forma similar;
5. esperar unos segundos;
6. terminar.

El objetivo de esta prueba no es validar fiduciales, sino generar datos raw suficientes para que la Prueba 1 pueda ejecutarse.

Comando:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba 2 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

---

## Patrones de reduccion de logs

### Prueba 1

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1C-REPLAY|F1D-SERVER|F1D-POSESTORE|F1D-POSESTORE-ANCHOR|F1D-POSESTORE-OPT|F1D-POSESTORE-CORRECTION|F1D-POSESTORE-NEW-KF|F1D-POSESTORE-STATS|ERROR|FATAL|Segmentation fault|Killed
```

### Prueba 2

```text
SCENARIO-RUNNER|GOAL|RESULT|success|F1B-ORBMAP-RX|F1C-RAWDB|F1C-RAWDB-SAVE|F1D-POSESTORE|ERROR|FATAL|Segmentation fault|Killed
```

La reducción genera `prueba_X.reduced.log` y conserva `prueba_X.log` completo.

Si el reducido no muestra datos suficientes, regenerarlo con patrones específicos antes de repetir Gazebo o concluir que el código no emitió el marcador. Nunca abrir el log completo.

---

## Criterio de exito

La subfase se considera `CONSEGUIDA` solo si:

1. el build devuelve `0`;
2. `GlobalPoseStore` existe en `orbslam3_multi` y compila;
3. el servidor nuevo puede crear/usarla sin depender del servidor antiguo;
4. `GlobalPoseStore` empieza vacía;
5. al anclar un submapa sintético/debug:
   - se consulta `RawMapDatabase`;
   - se obtienen los KFs del submapa;
   - se calcula y guarda una pose `world_T_kf` para cada KF existente;
6. `RawMapDatabase` no sobrescribe poses locales originales;
7. al insertar KFs nuevos de un submapa anclado:
   - reciben pose global;
8. al registrar una pose optimizada sintética:
   - se guarda la pose optimizada;
   - se calcula una corrección;
   - se actualiza la corrección simple del submapa;
9. al insertar KFs nuevos posteriores a esa corrección:
   - heredan la última corrección del mismo submapa;
10. aparecen los marcadores obligatorios:

    ```text
    F1D-POSESTORE-ANCHOR-SUMMARY
    F1D-POSESTORE-OPT-POSE-SET
    F1D-POSESTORE-CORRECTION-SET
    F1D-POSESTORE-NEW-KF-CORRECTION-INHERIT
    F1D-POSESTORE-STATS
    ```

11. no aparecen errores graves no explicados;
12. el historial queda actualizado.

No se exige que RViz2 muestre nube global en esta subfase.

---

## Criterio de fallo o parcial

La subfase debe marcarse como `NO CONSEGUIDA` si:

- no compila;
- `GlobalPoseStore` no se integra en `orbslam3_multi`;
- no se puede anclar un submapa usando datos de `RawMapDatabase`;
- no se generan poses globales para KFs existentes del submapa anclado;
- se modifican poses raw en `RawMapDatabase`;
- faltan marcadores obligatorios;
- aparece un error grave no explicado;
- se implementan fiduciales/loops/optimización real en esta subfase rompiendo el alcance.

La subfase puede marcarse como `PARCIAL` si:

- compila;
- `GlobalPoseStore` existe;
- el anclaje sintético crea poses;
- pero falla la corrección heredada para KFs nuevos o falta alguna estadística/log no crítica.

La subfase debe marcarse como `BLOQUEADA` solo si:

- no existe dataset replay 3C y Gazebo no puede ejecutarse para generarlo;
- la API real de `RawMapDatabase` no permite consultar KFs/poses locales y no puede ampliarse mínimamente;
- hay una dependencia externa no resuelta que impide validar la integración.

---

## Documentacion a actualizar

Actualizar obligatoriamente:

- historial de Fase 3/subfase 3D;
- `codex/contexto/01_ESTADO_ACTUAL.md` si cambia el estado real;
- `codex/contexto/paquetes/orbslam3_multi/orbslam3_multi.md`;
- crear o actualizar:

  ```text
  codex/contexto/paquetes/orbslam3_multi/global_pose_store.md
  ```

- documentación de `RawMapDatabase` si se añaden consultas nuevas:

  ```text
  codex/contexto/paquetes/orbslam3_multi/raw_map_database.md
  ```

- documentación de `orbslam3_server` si se añaden parámetros o modo debug/replay:

  ```text
  codex/contexto/paquetes/orbslam3_server/global_map_server.md
  codex/contexto/paquetes/orbslam3_server/launches.md
  ```

La documentación debe explicar claramente:

```text
GlobalPoseStore es una capa ligera.
No guarda datos ORB grandes.
Consulta RawMapDatabase cuando necesita KFs o poses locales.
Empieza vacía.
Se llena al anclar submapas o registrar poses optimizadas.
La política de corrección heredada de 3D es simple y temporal.
```

También debe quedar escrito que:

```text
El problema de KFs nuevos tras optimización se ha considerado desde 3D,
pero la política robusta basada en parent/covisibilidad/interpolación queda para subfases posteriores.
```

No marcar la subfase como `realizado` si no se cumple el criterio de éxito.

## Incremento Visual Obligatorio

Aplicar `../CONTRATO_VISUAL_INCREMENTAL.md`. Añadir `GlobalPoseStore` y la
transferencia acotada de altas/cambios raw hacia poses, con IDs y revisiones
reales. No dibujar anchors fiduciales, builder ni RViz2 antes de sus subfases.
