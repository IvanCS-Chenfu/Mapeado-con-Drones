# Subfase 3H - Revisita fiducial y planificador secundario

## Estado

```text
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: PENDIENTE PARA EXTENSION POST-LOOP
Ejecucion: BASE VALIDADA; REANCHOR HARD POST-LOOP PENDIENTE
Dudas abiertas: ninguna
```

Este documento es el contrato vigente y ejecutado de 3H. La evidencia de
implementaciones anteriores se conserva en el historial de la subfase y en
`legacy2`; no forma parte del runtime activo.

## Sucesion acordada en 3Q

La base 3H permanece como regresion. Al implementar 3Q, el primer fiducial de
un hijo ya anclado soft deja de hacer siempre reanchor rigido: dentro de umbral
promociona hard sin mover; fuera de umbral crea MAX y usa el grafo covisible
comun antes de cortar la dependencia. Una optimizacion loop sigue siendo BAJA,
pero activa `stop_drones` desde branch begin hasta task end, incluida su fusion
directa. Esta extension pertenece a 3Q y no cambia la evidencia historica 3H.

## Objetivo

Completar la entrada de una revisita fiducial y crear desde esta subfase la
infraestructura secundaria minima que ejecutara la optimizacion completa de
3H-3L:

```text
FiducialAnchorManager --opt_fid/MAX--> SecondaryTaskQueue
                      --dequeue/start--> SecondaryWorker
```

Una observacion fiducial absoluta debe:

1. anclar el submapa si todavia no tiene pose world;
2. si solo tiene un anchor loop blando, sustituirlo por un anchor fiducial hard
   mediante reanchor absoluto de todo el submapa;
3. si ya posee autoridad fiducial propia, comparar el KF observado con su
   objetivo absoluto;
4. terminar sin optimizacion si esa revisita esta dentro de umbral;
5. encolar una `FiducialOptimizationTask` MAX si la revisita supera el umbral;
6. volver a validar el error al desencolar la tarea, porque otra optimizacion
   puede haber movido ya el KF.

El flujo principal no se detiene: sigue incorporando KFs y publicando el estado
coherente disponible. El bloqueo de mision impide enviar el siguiente goal,
pero no cancela el goal activo ni bloquea el ingest.

## Fuente de la pose absoluta

El servidor reutilizara el proceso ya implantado para el primer fiducial:

- obtiene el GT permitido del fiducial simulado;
- lo asocia al KF concreto que produjo la observacion;
- aplica la extrinseca conocida `body_T_camera` y normaliza el dato a
  `target_world_T_kf`;
- entrega a `FiducialAnchorManager` una observacion independiente de ROS y del
  formato original del simulador.

El GT solo representa la observacion absoluta simulada. No puede usarse para
ponderar aristas temporales, ajustar el solver, aceptar una propuesta por una
metrica global ni construir el mapa final.

## Identidad de visita

Cada entrada al radio de un fiducial recibe un `fiducial_visit_id` estable.
Debe persistirse junto a la observacion para que live y replay tomen las mismas
decisiones. El formato de record debe ser versionado y compatible con records
anteriores que no contengan el campo.

Reglas de control dentro de una visita:

- el primer KF observado de la visita reserva la candidatura a
  `last_accepted_control_kf`;
- si esta dentro de umbral se acepta como control inmediatamente;
- si requiere optimizacion, pasa a ser control cuando su resultado
  sea aceptado y comprometido;
- los KFs posteriores de la misma visita no pueden adelantarse ni desplazar el
  control reservado, aunque sean coherentes mientras la tarea calcula;
- cada KF posterior sigue calculando su propio error y puede originar una tarea
  si supera el umbral;
- una nueva visita, incluso al mismo fiducial, puede establecer un nuevo control.

De esta forma, una futura ventana puede ir desde un KF del fiducial 2 hasta otro
KF del mismo fiducial 2, siempre que pertenezcan a visitas distintas.

## Decision de `FiducialAnchorManager`

Para cada observacion se consulta el estado mas reciente de `GlobalPoseStore`.

### Submapa no anclado

- se ejecuta exclusivamente el flujo `first_anchor` ya validado en 3E;
- se crea el anchor y se registra el primer control aceptado;
- no se crea `opt_fid` para esa misma observacion.

### Submapa anclado solo por loop

- el anchor actual se considera blando y no equivale a autoridad fiducial;
- el primer fiducial observado directamente calcula `world_T_local` con el
  mismo procedimiento absoluto de `first_anchor`;
- todos los KFs actuales del submapa se reanclan rigidamente y en un unico
  commit versionado;
- el KF observador queda hard y pasa a `last_accepted_control_kf`, incluso si
  el error anterior ya estaba dentro de umbral;
- se corta la dependencia live respecto del submapa padre;
- la constraint loop se conserva como evidencia inactiva para una futura
  optimizacion con covisibilidad;
- esta observacion no crea `opt_fid`: funcionalmente es el primer anchor hard
  propio del submapa.

### Submapa con autoridad fiducial propia

- no se vuelve a ejecutar `first_anchor`;
- se obtiene `current_world_T_kf` de `GlobalPoseStore`;
- se calcula error de traslacion, rotacion completa y yaw respecto de
  `target_world_T_kf`;
- si todos los errores estan dentro de umbral, se registra el resultado
  coherente y termina el flujo;
- si cualquier error supera su umbral, se crea una tarea MAX ligera.

Valores iniciales, configurables y apoyados en el comportamiento validado de
`legacy2`:

```text
fiducial_translation_threshold_m: 0.35
fiducial_rotation_threshold_rad: 0.35
fiducial_yaw_threshold_rad: 0.25
```

## Tarea secundaria

`FiducialOptimizationTask` conserva solo datos estables y baratos:

```text
task_id
priority = MAX
submap_id = (drone_id, map_epoch)
keyframe_id
fiducial_id
fiducial_visit_id
target_world_T_kf
observation_revision / provenance
enqueue_sequence
```

La identidad logica para deduplicar una repeticion exacta es, como minimo,
`(submap_id, keyframe_id, fiducial_id)`. No se deduplican KFs distintos de una
misma visita. No se usa `arrival_id` como identidad del trabajo.

Cada KF distinto con error alto se encola, aunque ya haya otra optimizacion
fiducial pendiente o activa. Esto evita perder observaciones. Cuando llegue su
turno, la revalidacion barata eliminara como `STALE` las que ya hayan quedado
corregidas.

## `SecondaryTaskQueue`

3H crea una unica cola global, con tres carriles de prioridad:

| Prioridad | Tipo | Estado en 3H-3L |
|---|---|---|
| `MAX` | `FiducialOptimizationTask` | funcional |
| `HIGH` | `DatabaseUpdateTask` | reservada en 3H-3L; activada en 3M como MEDIA |
| `NORMAL` | `LoopTask` | reservada en 3H-3L; activada en 3N como BAJA |

La seleccion se hace por prioridad y conserva FIFO dentro de cada carril. Una
tarea activa nunca se interrumpe. Desde 3N, si hay un loop activo, se termina
todo su flujo antes de iniciar las tareas MAX pendientes.

La cola debe tener capacidad operativa muy alta o crecimiento controlado, sin
descartar trabajo unico. La proteccion real contra acumulacion es el
backpressure con histeresis, no un drop silencioso.

## `SecondaryWorker`

3H crea un unico worker persistente. Una tarea fiducial mantiene el mismo
`task_id` durante grafo, solver, validacion, commit y posibles pasadas internas
de refinamiento. Esas etapas no se reencolan como tareas independientes.

Antes de construir el grafo, el worker:

1. vuelve a leer la pose del KF en `GlobalPoseStore`;
2. recalcula los tres errores con el objetivo almacenado en la tarea;
3. termina con estado `STALE` si ya estan dentro de umbral;
4. captura las revisiones necesarias y continua hacia 3I si el error persiste.

Este camino solo corresponde a revisitas posteriores de un submapa con
autoridad fiducial propia. El primer fiducial posterior a un anchor loop se
resuelve mediante reanchor absoluto y no entra en el worker, 3I, 3J o 3L.

El worker calcula sobre snapshots privados. No publica ROS, no llama a
`GlobalMapBuilder`, no espera a RViz2/web y no mantiene locks live durante el
calculo.

## Backpressure y parada de drones

El flag de parada es el OR de:

- high watermark de la cola principal;
- `secondary_pending >= 64`;
- optimizacion fiducial activa;
- optimizacion por loop 3Q activa.

La liberacion requiere simultaneamente:

- ninguna optimizacion activa;
- `secondary_pending <= 16`;
- cola principal por debajo de su low watermark.

El goal actual termina. Mientras el flag siga activo no se envia el siguiente
goal. El flujo principal y los datos ya en vuelo continuan normalmente.

## Cambios a realizar

- ampliar la observacion fiducial y su record con `fiducial_visit_id`, objetivo
  absoluto normalizado y compatibilidad hacia atras;
- reutilizar en el servidor la asociacion GT-KF y `body_T_camera` de 3E;
- extender `FiducialAnchorManager` con las ramas first-anchor, reanchor hard
  post-loop y revisita, calculo de error, control por visita y emision de
  `opt_fid`;
- crear los tipos de tarea, estados y resultado secundarios;
- crear `SecondaryTaskQueue` y `SecondaryWorker` unicos;
- conectar prioridad, FIFO, deduplicacion exacta, revalidacion y estado
  `STALE`;
- integrar los nuevos motivos de backpressure y su histeresis;
- añadir telemetria ligera para observacion, decision, enqueue, dequeue,
  `STALE`, inicio/fin y parada de mision.

La localizacion exacta de nuevas clases se decidira al auditar el codigo activo.
El servidor sera orquestador; la logica geometrica y de estado permanecera en
`orbslam3_multi`, siguiendo los limites ya usados por el backend.

`RawMapDatabase` puede persistir el metadato crudo de la observacion/record, pero
ninguna optimizacion puede reescribir sus poses, KFs o MPs ORB-SLAM3.

## Contrato visual

La arista de observacion fiducial pulsa siempre. `first_anchor` solo pulsa para
un submapa sin pose world. El primer fiducial de un submapa anclado por loop
debe mostrar `first_hard_reanchor` hasta el commit directo en
`GlobalPoseStore`, sin pasar por la cola. En revisita con error alto debe verse:

```text
FiducialAnchorManager --opt_fid/MAX--> SecondaryTaskQueue
SecondaryTaskQueue --dequeue/start--> SecondaryWorker
```

Los eventos incluyen IDs, visita, prioridad, error, estado y tiempos, nunca
matrices o payloads pesados completos.

## Pruebas de 3H

- first-anchor no crea una optimizacion duplicada;
- el primer fiducial post-loop reancla todo el submapa, queda hard/control y no
  crea `opt_fid`, tanto con error alto como bajo;
- la constraint loop se conserva pero deja de propagar movimientos live;
- observacion coherente termina sin tarea y actualiza control solo con las
  reglas de visita acordadas;
- KFs distintos de una visita con error alto generan tareas distintas;
- repeticion exacta del mismo KF se deduplica;
- prioridad MAX y FIFO son deterministas;
- una tarea activa no se interrumpe;
- una tarea pendiente se convierte en `STALE` tras una correccion previa;
- record/replay conserva `fiducial_visit_id` y acepta records antiguos;
- el flujo principal sigue avanzando mientras el worker calcula;
- el flag de parada activa y libera con los umbrales acordados;
- el grafo web refleja el orden real.

## Criterios de cierre

3H queda conseguida solo si compila, supera tests C++, replay y prueba live
acordada, y si se observa que no se pierde trabajo ni se bloquea el flujo
principal. En la entrega conjunta 3H-3L, una tarea no `STALE` debe continuar por
3I, 3J, 3L y 3K hasta un commit valido o un fallo duro explicito.

## Fuera de alcance

- covisibilidad y ventanas multi-submapa;
- implementacion funcional de `DatabaseUpdateTask` y `LoopTask`;
- matching, RANSAC, fusion y optimizacion por loop;
- publicacion directa desde el worker secundario;
- modificacion de `ORB_SLAM3`, `orbslam3_ros2` u `orbslam3_msgs`.
