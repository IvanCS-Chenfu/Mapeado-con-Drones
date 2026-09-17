# Subfase 6A - Contrato de workflows y colas

## Estado

`PARCIAL`.

La base aislada esta implementada: `WorkflowScheduler` aporta las cinco colas,
FIFO, reencolado justo, deduplicacion de workflows/resultados y generaciones
de orden normal por dron invalidadas por STOP. Queda migrar los productores y
consumidores reales en las subfases siguientes.

## Objetivo

Definir la infraestructura comun de la nueva Fase 6. El servidor deja de
recorrer periodicamente todas las tareas para decidir el siguiente paso. Cada
hecho relevante crea un trabajo finito en una cola FIFO y cada worker consume
un trabajo, produce su continuacion y se desentiende del dron.

## Contrato funcional

Todo trabajo debe llevar `drone_id`, `task_id`, `workflow_id`, `command_id`,
`map_epoch` y la revision de mapa de entrada cuando corresponda. Los IDs hacen
idempotentes los comandos, resultados y continuaciones tardias.

```text
TaskAssignment -> PointSelection -> TrajectoryPlanning
                                      -> ActiveTrajectoryMonitor
DepthIntegration -> EvidenceDatabase -> VoxelMapBuilder
                                      -> continuation queue
```

Un dron conserva una unica orden normal activa. STOP es una excepcion local y
asincrona: invalida la generacion de la orden normal y acumula causas si dos
riesgos llegan a la vez.

## Cambios requeridos

- Introducir tipos de trabajo y estados de workflow por dron.
- Implementar FIFO justa: un reintento vuelve al final y no impide otros IDs.
- Reconocer como duplicado cualquier `command_id` o resultado ya procesado.
- Preparar la sustitucion de `RunFacadeWorker` como barrido global de tareas;
  su eliminacion ocurre cuando sus transiciones ya usen las colas nuevas.
- Separar callback groups de recepcion ligera, planificacion, materializacion
  voxel y monitor de trayectoria.

## Exclusiones

- No migrar todavia el pipeline de Fase 4/5 ni los mandos externos legacy.
- No calcular D*, depth ni voxelizacion dentro de un callback de servicio.

## Validacion y exito

Tests de orden FIFO, duplicados, resultado tardio y generacion invalidada. Un
trabajo lento de D1 no puede impedir desencolar D2. Ningun worker queda
esperando movimiento, giro, depth o respuesta de otro nodo.
