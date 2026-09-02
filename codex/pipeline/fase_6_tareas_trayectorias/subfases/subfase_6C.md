# Subfase 6C - Contratos ROS, registro de drones y lifecycle

## Estado

```text
sin hacer
```

## Dependencia

6A y semantica 6B.

## Objetivo tecnico

Crear `mission_msgs`, el handshake de drones y el ciclo de vida compartido
entre `task_server` y `task_manager`, sin implementar aun planning.

## Contrato vigente

`mission_msgs` es canonico en Servidor y replica exacta en Dron. No se usa
`orbslam3_msgs` como contenedor generico de tareas.

El registro se implementa como servicio idempotente. Transmite una vez
`drone_id`, bounding box/dimensiones, `vehicle_profile`, `protocol_version`,
`trajectory_generator_id/version` y un `capability_mask` tipado. Como minimo
se reservan capacidades para sparse, stereo depth, camera pitch, fiducial
visual y dense futura. La respuesta congela frame de mision, `voxel_size`,
perfiles y versiones. Un dron no registrado no puede recibir reserva valida.
El margen extra pertenece al servidor y no viaja en cada plan.

El contrato debe representar registro, asignacion/estado, `TrajectoryPlan`,
ack/reject, replanning, depth emergency, `TRACKING_RISK`, STOP, recovery y
finalizacion. Distingue `task_id`, `plan_id/revision`, `trajectory_id`,
`map_epoch`, `map_revision` y revision de alineamiento.

Lifecycle minimo: `PENDING`, `ASSIGNED`, `RUNNING`, `PAUSED`, `COMPLETED`,
`FAILED`, `CANCELLED` y `BLOCKED/WAITING` cuando corresponda. Los nombres ROS
exactos se eligen por semantica real tras auditar interfaces F5.

Distribucion ROS acordada:

- servicio para `RegisterDrone` y respuesta inmediata;
- topics `reliable+transient_local` para snapshots de mision, geometria,
  registro y estado que deba recuperar un observador tardio;
- action de ejecucion de `TrajectoryPlan` preparada para bloques posteriores;
- topics reliable para STOP, riesgo y otras excepciones asincronas.

En 6C solo funcionan registro, snapshots y lifecycle sintetico. La action se
compila y prueba por round-trip, pero no recibe ningun plan real.

## Cambios requeridos

1. Diseñar los mensajes, servicio, action y eventos minimos con la distribucion
   ROS acordada, sin negociacion redundante.
2. Incluir payload regional de `MAP_SECTION`, no A-B-C ni ruta precalculada.
3. Definir `TrajectoryPlan` en W reproducible y rechazos de alineamiento/start state/generador.
4. Implementar `DroneRegistry`, `capability_mask`, versiones y handshake
   idempotente validado.
5. Definir reportes de STOP, riesgo visual, retreat, pausa y reanudacion.
6. Mantener copias Server/Dron byte a byte iguales y actualizar guardas F2.
7. Exponer estado agregado/eventos para GUI F7 y grafo sin convertirlos en control.

## Limites

No implementar allocator, planner o ejecucion. No transmitir GT, samples densos
ni dimensiones repetidas. No fijar valores experimentales sin prueba.

## Pruebas

- Build/round-trip de campos no triviales y transiciones validas/invalidas.
- Registro duplicado, dron desconocido, capability/version y perfil
  incompatibles.
- Guarda exacta de replicas `mission_msgs`.
- Integracion ROS estacionaria con dos drones, GUI+Gazebo y ambos grafos
  mostrando registro/lifecycle sintetico, sin enviar trayectorias reales. GT
  puede sostener el control simulado, pero no aparece en `mission_msgs`.

## Criterio de exito

Contratos minimos, versionados y reproducibles; registro estable; `PAUSED` y
motivos excepcionales explicitos; replicas identicas y ninguna contaminacion de
`orbslam3_msgs`.
