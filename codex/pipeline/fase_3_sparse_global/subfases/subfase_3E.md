# Subfase 3E - FiducialAnchorManager y primer anclaje simulado

## Estado

```text
CONSEGUIDA
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA
Dudas abiertas: ninguna
```

Este documento es el contrato ejecutable vigente. La evidencia de la
implementacion anterior se conserva en
`historial/por_subfase/historial_3E_RESUMEN.md` y `historial_3E.md`.

## Objetivo

Incorporar el primer anclaje absoluto de cada submapa mediante un fiducial
simulado con ground truth de Gazebo, manteniendo la arquitectura preparada para
sustituir esa fuente simulada por una deteccion visual real en Fase 4.

Al terminar 3E:

- cada dron podra anclar su submapa al observar por primera vez un fiducial;
- `GlobalPoseStore` contendra el anchor y las poses world de los KFs existentes;
- solo el KF que crea el anchor sera hard fiducial;
- las observaciones posteriores quedaran registradas para 3H, sin recalcular el
  anchor;
- el mismo flujo podra reproducirse desde un `.record` sin GT en directo;
- RViz2 seguira vacio porque `GlobalMapBuilder` pertenece a 3F.

## Punto de partida de 3D

- `GlobalMapServer` recibe deltas, los encola y ejecuta una sola `PrimaryTask`.
- `SparseGlobalBackend` coordina `RawMapDatabase` y `GlobalPoseStore`.
- `RawMapDatabase` conserva la autoridad sobre los datos ORB-SLAM3 crudos y
  devuelve un resultado de insercion con los KFs nuevos o modificados.
- `GlobalPoseStore` permanece vacio para submapas sin anchor y propaga cambios
  raw solo para submapas ya anclados.
- no existen `GlobalMapBuilder` ni publishers del mapa global.

## Lo anterior que no debe repetirse

- bloquear la recepcion de GT con un executor monohilo mientras trabaja el
  flujo principal;
- usar un buffer de miles de muestras o `vector.erase(begin())` para mantenerlo;
- proteger GT con el mutex general del estado live;
- volver a interpretar el delta en `GlobalMapServer` para extraer KFs que ya ha
  procesado `RawMapDatabase`;
- convertir `FiducialAnchorManager` en una clase dependiente de Gazebo, GT o la
  pose del cuerpo del dron;
- marcar como hard todos los KFs observados dentro del fiducial;
- promediar observaciones cercanas o recalcular repetidamente el primer anchor;
- crear loops, BoW, fusion u optimizacion para KFs pre-anchor;
- esperar una publicacion o un ACK de RViz2 que todavia no existe;
- usar el GT como pose final general de los KFs o del mapa.

## Arquitectura acordada

### 1. GroundTruthBuffer acotado

`GlobalMapServer` tendra una utilidad interna `GroundTruthBuffer`:

- un `deque` o ring de exactamente 50 muestras por dron;
- insercion y expulsion O(1);
- mutex propio, sin compartir el mutex de la cola principal;
- callback group independiente;
- recepcion capaz de avanzar mientras la `PrimaryTask` procesa un delta;
- topic por dron `/dron_X/sensor/GT/pose`;
- muestra ligera con dron, timestamp, pose GT y fiducial cuyo radio contiene al
  dron, si existe.

Fiduciales simulados configurados para esta subfase:

| ID | Posicion world | Radio |
|---|---|---|
| `1` | `(0, 9, 1)` | `2 m` |
| `2` | `(0, -9, 1)` | `2 m` |

El callback solo captura y clasifica muestras. No consulta bases, no crea
anchors y no ejecuta calculos pesados.

### 2. Asociacion temporal tras el commit raw

La misma `PrimaryTask` creada en 3C se amplia despues de insertar el delta en
raw. No se crea otra cola ni otro worker principal.

`RawMapDatabase` devuelve al backend las identidades de los KFs realmente
nuevos. Para cada KF nuevo, `SparseGlobalBackend`:

1. obtiene su timestamp y `local_T_camera` desde `RawMapDatabase`;
2. consulta un snapshot acotado del ring del dron;
3. selecciona la muestra valida mas cercana en tiempo;
4. exige `abs(gt_stamp - kf_stamp) <= 1.0 s`;
5. descarta la asociacion si la muestra no esta dentro de un fiducial;
6. registra exito o motivo de rechazo, incluido `nearest_dt`.

Un KF fuera de la historia disponible no provoca busquedas ilimitadas ni un
anchor inventado. Se registra como `gt_history_expired` o motivo equivalente.
Los KFs solamente modificados no generan una nueva observacion fiducial en 3E.

### 3. Adaptador simulado y contrato normalizado

La capa especifica de simulacion convierte:

```text
world_T_body_GT * body_T_camera = world_T_camera_target
```

Extrinseca aceptada para la camara:

```text
translation = (0.10, 0.03, 0.03)
RPY_deg     = (0, -90, 90)
optical convention = true
```

El adaptador construye una observacion normalizada que contiene como minimo:

- `arrival_id`;
- identidad exacta `RawKeyFrameId` (`drone_id`, `map_epoch`, KF local);
- timestamp del KF y diagnostico temporal;
- `world_T_camera_target`;
- identidad del fiducial;
- fuente (`simulated_gt` en 3E);
- calidad o diagnostico de la asociacion.

`FiducialAnchorManager` recibe exclusivamente esta observacion normalizada. Su
API no menciona GT, Gazebo, body pose ni topics ROS.

### 4. Primer anchor y GlobalPoseStore

`SparseGlobalBackend`, no `GlobalMapServer` ni `RawMapDatabase`, coordina el
anclaje. `FiducialAnchorManager` calcula:

```text
world_T_local = world_T_camera_target * inverse(local_T_camera)
```

Si `(drone_id, map_epoch)` todavia no esta anclado, `GlobalPoseStore` realiza un
commit atomico que:

- registra `world_T_local` y aumenta la revision del submapa;
- transforma e inserta todos los KFs raw existentes del submapa;
- marca como hard fiducial solamente el KF asociado que creo el anchor;
- deja disponibles esas poses para que los deltas posteriores sigan la ruta
  incremental ya creada en 3D.

En 3E basta distinguir `UNANCHORED` y `ANCHORED_PENDING_PUBLICATION`, derivados
de la existencia del anchor y su revision. Los estados de publicacion, backfill
y activacion pertenecen a 3F.

### 5. Observaciones posteriores

Si el submapa ya tiene anchor, una nueva observacion valida:

- se conserva en el journal con su KF exacto y diagnosticos;
- no se promedia con la primera;
- no cambia `world_T_local`;
- no crea otro hard fiducial;
- no inicia optimizacion.

3H reutilizara estas observaciones para calcular error de revisit y decidir si
corresponde una tarea fiducial prioritaria.

### 6. Record y replay

Se generara un dataset nuevo `rawdb_prueba_3e.record`; no se sobrescribe el de
3C. El formato guardara las observaciones fiduciales normalizadas vinculadas a
su `arrival_id`.

El replay:

- no se suscribe a GT live;
- reinyecta deltas y observaciones en el orden grabado;
- atraviesa el mismo `SparseGlobalBackend`, `FiducialAnchorManager` y
  `GlobalPoseStore`;
- debe reproducir las mismas asociaciones, anchors, KFs hard y poses globales
  dentro de la tolerancia numerica definida por los tests.

### 7. Preparacion para Fase 4

La deteccion visual real entregara la identidad exacta del KF y la transformada
camara-fiducial. La capa de Fase 4 la normalizara a `world_T_camera_target` y
reutilizara el manager, el backend, el pose store y el journal de 3E.

El camino visual real evitara el adaptador temporal GT, pero no exigira cambiar
la semantica ni la API de `FiducialAnchorManager`.

## Grafo web

- añadir el vertice `FiducialAnchorManager`;
- no añadir un vertice para `GroundTruthBuffer`;
- activar `GlobalMapServer -> FiducialAnchorManager` solo cuando una asociacion
  GT-KF valida sea entregada al manager;
- activar `FiducialAnchorManager -> GlobalPoseStore` al comprometer el primer
  anchor;
- conservar `RawMapDatabase -> GlobalPoseStore` para KFs posteriores de un
  submapa ya anclado;
- enviar solo metadatos ligeros y no bloquear el pipeline si se pierde
  telemetria visual.

## Archivos previstos

### `orbslam3_multi`

- tipos de observacion fiducial normalizada;
- `fiducial_anchor_manager.hpp/.cpp`;
- ampliaciones de `SparseGlobalBackend`, `RawMapDatabase` y `GlobalPoseStore`;
- journal record/replay y tests unitarios;
- `CMakeLists.txt` y documentacion activa del paquete.

### `orbslam3_server`

- `GroundTruthBuffer` y callbacks GT independientes;
- adaptador simulado body-camera;
- integracion con la `PrimaryTask` existente;
- parametros de fiduciales, extrinseca y `max_dt`;
- instrumentacion web, launch, tests y documentacion activa.

### `simulacion_dron`

- topologia/eventos nuevos del grafo web;
- trayectoria dedicada de prueba 3E si la existente no expresa exactamente el
  recorrido y las esperas acordadas.

## Fuera de alcance

- `GlobalMapBuilder`, publicaciones ROS globales y contenido en RViz2;
- snapshots periodicos de 3G;
- revisit, calculo de error y optimizacion fiducial de 3H;
- BoW, loops, RANSAC, fusion, score y worker secundario;
- modificar `ORB_SLAM3`, `orbslam3_ros2` u `orbslam3_msgs`;
- usar GT fuera del adaptador de fiducial simulado, debug o metricas externas.

## Validacion obligatoria

### Tests y build

- ring de 50 muestras por dron y expulsion correcta;
- concurrencia basica y snapshot sin retener el mutex durante el calculo;
- asociacion al GT mas cercano, limite `1.0 s` y rechazos diagnosticados;
- transformacion body-camera y formula de anchor;
- primer anchor atomico, poblado de KFs y un unico hard fiducial;
- observaciones posteriores registradas pero diferidas;
- equivalencia live/replay;
- build de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`.

### Prueba live

1. Abrir Gazebo, RViz2 y el grafo web.
2. Hacer que ambos drones lleguen al fiducial 2 y permanezcan alli el tiempo
   suficiente para producir al menos un KF asociable por dron.
3. Exigir que los dos submapas creen su primer anchor en el fiducial 2.
4. Verificar que `GlobalPoseStore` contiene poses globales de ambos submapas.
5. Verificar en el grafo actividad real
   `GlobalMapServer -> FiducialAnchorManager -> GlobalPoseStore`.
6. Mantener RViz2 vacio durante toda la prueba.
7. Guardar `rawdb_prueba_3e.record` y analizar exclusivamente logs reducidos.

### Prueba replay

Reproducir el dataset sin Gazebo ni GT live y comprobar que se obtienen las
mismas observaciones aceptadas, dos anchors, identidades hard y contenido de
`GlobalPoseStore` que en la ejecucion live.

## Marcadores minimos

```text
[F3E-GT-BUFFER]
[F3E-FID-KF-ASSOC]
[F3E-FID-ASSOC-REJECT]
[F3E-FID-OBS]
[F3E-FID-FIRST-ANCHOR]
[F3E-FID-KF-HARD]
[F3E-FID-JOURNAL-SAVE]
[F3E-FID-REPLAY-OBS]
[F3E-FID-STATS]
```

## Criterio de exito

3E solo queda `CONSEGUIDA` si:

- compilan los tres paquetes y pasan los tests focalizados;
- ambos drones crean un anchor independiente en el fiducial 2;
- `GlobalPoseStore` contiene KFs de los dos submapas anclados;
- solo el primer KF de cada submapa queda hard;
- las observaciones posteriores no mueven el anchor;
- el replay reproduce el resultado sin GT live;
- el grafo web muestra los flujos acordados;
- RViz2 permanece vacio y no existen publishers globales prematuros;
- los logs no muestran crash, deadlock, crecimiento no acotado ni uso de GT
  fuera de la ruta simulada.

Si solo se ancla un dron, faltan datos de uno de los submapas, el replay diverge
o aparece contenido global en RViz2, la subfase no puede declararse conseguida.

## Documentacion al cerrar

Actualizar el estado compacto, docs vigentes de los tres paquetes, historial y
resumen de 3E, indice de historial y ultima sesion. Cada intento live o replay
se registra por separado y nunca se elimina una prueba fallida anterior.
