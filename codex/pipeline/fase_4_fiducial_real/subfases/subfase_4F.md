# Subfase 4F — Recepción en Servidor y sincronización exacta batch ↔ KeyFrame

## Estado

```text
CONSEGUIDA — implementada, compilada y validada el 2026-08-25
```

## Dependencia

`4E` completada. 4F no necesita todavía implementar semántica `tag→fiducial`; eso pertenece a 4G.

## Objetivo

Recibir el topic fiducial y asociar cada batch **solo** con el KF identificado por:

```text
(drone_id, map_epoch, local_keyframe_id)
```

El orden de llegada entre `orb_map_delta` y el batch visual no puede cambiar el resultado.

## Casos obligatorios

```text
A) delta/KF llega primero
   batch después -> match inmediato

B) batch llega primero
   KF ausente -> pending
   delta llega -> resolver exactamente ese pending

C) batch llega y KF nunca aparece
   -> permanece pending hasta ser emparejado o expulsado por el FIFO del dron

D) reentrega idéntica
   -> duplicate, no procesar dos veces

E) misma identidad con contenido diferente
   -> conflict, no tratarlo como duplicado benigno
```

Nunca buscar el KF anterior/posterior ni “más cercano” por timestamp.

## Identidad y timestamp

La clave funcional es la identidad exacta. `header.stamp` actúa solo como
verificación de integridad:

- debe concordar exactamente con el KF crudo tras una conversión temporal común;
- un mismatch no autoriza reasignación;
- un epoch incoherente se rechaza.

## Estructura pending

Se implementa como un sidecar interno de `RawMapDatabase`: acompaña a la base
raw, pero no modifica `OrbMap`, `OrbKeyFrame` ni sus semánticas. La base no llama
a `FiducialManager`.

Conceptualmente:

```text
PendingKey = {drone_id, map_epoch, local_keyframe_id}
PendingValue = {batch}
pending_by_key = unordered_map<PendingKey, PendingValue>
pending_fifo_per_drone = deque<PendingKey>
```

Debe ser:

- protegida reutilizando el mutex interno ya existente en `RawMapDatabase`;
- lookup O(1) por clave exacta;
- acotada y configurable por dron mediante
  `fiducial_pending_capacity_per_drone`, default `10`;
- sin TTL;
- FIFO por dron: al insertar el undécimo pending se expulsa el más antiguo;
- liberada inmediatamente al emparejar el batch;
- observable mediante contadores;
- sin imágenes.

Una capacidad pequeña es deliberada. Se medirá cuántos batches se expulsan y
se ampliará el parámetro si la simulación demuestra pérdidas frecuentes.

## Concurrencia

El callback de deltas y el callback fiducial pueden ejecutarse concurrentemente.
La comprobación, inserción y extracción se realizan bajo el mutex existente para
evitar carreras donde:

1. callback A comprueba “KF no existe”;
2. callback B inserta el KF;
3. callback A mete pending y nadie vuelve a resolverlo.

No se introduce un sistema adicional de locks. La construcción del resultado,
telemetría y handoff en `GlobalMapServer` se realizan fuera del mutex. Bajo el
lock no se ejecutan PnP, matrices ni semántica fiducial.

## Integración con commits raw

- si el batch llega después, `RawMapDatabase` consulta el KF ya comprometido y
  devuelve un match inmediato;
- si llega antes, se guarda pending y se resuelve al comprometer el KF;
- delta y full snapshot consultan únicamente `RawInsertResult.new_keyframe_ids`;
- un KF actualizado o repetido no reactiva un batch ya consumido;
- `RawMapDatabase` conserva internamente `keyframe_first_arrival_id` para cada
  KF, sin añadir campos a `OrbKeyFrame`;
- el match sincronizado conserva ese `arrival_id` exacto del primer commit raw.

## Duplicados y conflictos

### Duplicado idéntico

Misma clave e igual contenido semántico -> ignorar segundo procesamiento y aumentar contador.

### Conflicto

Misma clave, pero cambia algún tag/pose/calidad de forma no explicable por serialización -> `FID-SYNC-CONFLICT`.

Esto es anómalo porque 4D/4E deben producir un resultado único por KF. No elegir
uno silenciosamente. Un digest por clave exacta ya consumida basta para
distinguir ambos casos: su presencia sustituye cualquier flag separado
`fiducial_batch_consumed`. Otro KF que observe el mismo tag es una clave distinta
y sigue el flujo normal.

## Salida de 4F

4F produce algo conceptualmente como:

```text
SynchronizedFiducialBatch {
  exact_raw_keyframe
 batch_metadata
  observations[]
  raw_first_arrival_id
}
```

El resultado se devuelve mediante `RawInsertResult` o el resultado de envío del
batch. `GlobalMapServer` hace el handoff fuera del lock. En 4F ese handoff solo
alimenta logs, contadores y telemetría; 4G añadirá `FiducialManager` y la
interpretación semántica.

No calcula todavía:

```text
object_id
object_T_tag
world_T_camera
zona segura
fusión
anchor
```

## GT legacy

La ruta GT de Fase 3 puede seguir físicamente presente durante transición/regresión, pero en pruebas visuales debe estar separada y no mezclarse con el batch nuevo.

No borrar legacy antes de que 4H pruebe sustitución completa.

## Grafos web

Aquí nace la arista runtime real:

```text
wrapper/orbslam3  --fiducial_keyframe_observations-->  orbslam3_server
```

Debe tener evidencia live directa y metadatos ligeros `{drone_id, epoch, kf, tag_count}`. No usar `flow_events`/GT para fingir actividad.

El bloque 4E+4F debe representar esta arista tanto en `system_architecture`
como en `pipeline_flow`, respetando el gating independiente de ambos debug.

## Archivos probables

```text
servidor/orbslam3_server/include/.../global_map_server.hpp
servidor/orbslam3_server/src/global_map_server.cpp
servidor/orbslam3_server/launch/...
servidor/orbslam3_multi/include/.../raw_map_database.hpp
servidor/orbslam3_multi/src/.../raw_map_database.cpp
metadata system_architecture
```

## Pruebas obligatorias

1. delta antes que batch;
2. batch antes que delta;
3. KF inexistente y expulsión FIFO al superar capacidad 10 en su dron;
4. batch duplicado idéntico;
5. batch conflictivo con misma identidad;
6. epoch viejo/futuro incoherente;
7. dos drones con mismo `local_keyframe_id` pero claves completas distintas;
8. carrera artificial callback delta/batch;
9. pending acotado bajo inyección de IDs inexistentes.
10. full snapshot y update no reactivan un KF consumido;
11. asociación exacta de `keyframe_first_arrival_id` en ambos órdenes;
12. trayectoria típica completa con Gazebo, RViz2, `system_architecture` y
    `pipeline_flow` activos, con ventanas fiduciales desactivadas.

## Logs

```text
FID-SYNC-MATCHED
FID-SYNC-PENDING
FID-SYNC-EVICTED
FID-SYNC-DUPLICATE
FID-SYNC-CONFLICT
FID-SYNC-REJECT
```

Siempre incluir clave completa; no matrices. Exponer además:

```text
pending_current
pending_peak
evicted
matched_immediate
matched_from_pending
duplicate
conflict
rejected
```

## Criterio de éxito

- ambos órdenes de llegada llevan al mismo KF exacto;
- ninguna asociación temporal aproximada;
- pending acotado por dron, configurable, FIFO y sin carreras conocidas;
- duplicados no duplican restricciones;
- conflictos son visibles;
- updates y snapshots no reactivan batches consumidos;
- el handoff ocurre fuera del mutex y conserva el primer `arrival_id` raw;
- 4F todavía no interpreta objetos;
- la arista runtime queda correctamente reflejada en `system_architecture` y
  `pipeline_flow`.
