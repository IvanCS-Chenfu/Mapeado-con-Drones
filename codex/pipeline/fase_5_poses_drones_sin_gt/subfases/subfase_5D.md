# Subfase 5D — Consulta asíncrona de KF y push de revisiones

## Estado

```text
CONSEGUIDA; servicio/push dirigido validados en prueba 230
```

## Objetivo

Implementar el transporte ROS 2 ligero entre Servidor y `orbslam3` del Dron:

1. consulta inicial de `W_T_KF` al cambiar el reference KF;
2. revisiones posteriores por push cuando cambie esa pose;
3. procesamiento no bloqueante respecto a `TrackStereo`;
4. resolución pendiente reutilizando el patrón real final de Fase 4.

Frontera obligatoria:

```text
Servidor <-> orbslam3
```

No hay comunicación directa Servidor↔`dron_individual`.

## Contexto y ámbitos

Leer 5A-5C, docs de `orbslam3_msgs`, `orbslam3_server`, `orbslam3_ros2`, la
decisión de Fase 2 sobre interfaz canónica/réplica y la implementación real de
Fase 4 para consumidores que piden un KF todavía no disponible.

Ámbitos probables:

```text
servidor/orbslam3_msgs/**   # copia canónica si la ADR vigente lo mantiene
dron/orbslam3_msgs/**       # réplica byte a byte
servidor/orbslam3_server/**
dron/orbslam3_ros2/**
launch/config mínimos, tests y docs
```

## Contrato del transporte

Servicio conceptual:

```text
request:  drone_id, map_epoch, keyframe_id
response: status, identidad, W_T_KF, pose_revision
```

Flujo:

```text
Knew -> async_send_request -> ORB continúa -> callback valida respuesta
```

Si el KF todavía no existe, la petición queda pendiente y se resuelve cuando
se materializa. No introducir polling periódico sin detenerse y debatirlo.

Semántica pending acordada:

```text
servicio -> respuesta PENDING inmediata
Servidor -> conserva solo el interés de la reference KF activa por dron
nuevo reference KF -> sustituye el interés anterior
materialización/revisión -> push dirigido al wrapper de ese dron
reference anterior reaparece -> nueva consulta
```

No se mantiene una FIFO ilimitada de referencias. El push es unitario,
`reliable` y namespaced por dron; no publica arrays globales a todos los drones.

Una revisión `N+1` del KF conocido se publica automáticamente. Se rechazan
epoch distinto, revisión antigua y respuesta que ya no corresponda al estado
activo. Clave completa:

```text
(drone_id, map_epoch, keyframe_id, pose_revision)
```

No introducir heartbeat de 1 Hz en 5D. La hipótesis de Fase 5 es ausencia de
pérdida de mensajes y reinicio inesperado del Servidor.

El wrapper puede registrar o cachear una respuesta tardía, pero nunca aplicarla
al estado activo si ya cambió reference KF. Una consulta posterior siempre
revalida identidad y revisión con el Servidor.

## Pruebas acordables

1. KF disponible: request/response/revisión aceptada sin degradar ORB;
2. KF pendiente: pending→materialización→respuesta;
3. optimización posterior: push de revisión `N+1`;
4. revisión obsoleta y epoch incorrecto descartados.

Patrones iniciales:

```text
F5D|KF_REQUEST|KF_PENDING|KF_RESPONSE|KF_REVISION_PUSH|STALE_REVISION|EPOCH_MISMATCH|REFERENCE_KF|ERROR|FATAL
```

## Criterio de éxito

Servicio asíncrono, pending resuelto, push versionado, copias de mensajes
idénticas, frontera correcta y builds/tests acordados correctos.
