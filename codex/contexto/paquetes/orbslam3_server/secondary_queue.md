# `SecondaryTaskQueue`

## Rol

Scheduler global de trabajos secundarios con un unico consumidor persistente.

## Contrato

| Carril | Trabajo | Estado actual |
|---|---|---|
| MAX | `FiducialOptimizationTask` | funcional |
| HIGH/MEDIA | `DatabaseUpdateTask` | funcional |
| NORMAL/BAJA | `LoopTask` | funcional |

- siempre elige el carril no vacio de mayor prioridad;
- mantiene FIFO por `enqueue_sequence` dentro del carril;
- no interrumpe la tarea activa;
- fiducial deduplica solo su identidad exacta pendiente o activa;
- loop deduplica la misma revision semantica y conserva un ledger acotado de
  revisiones ya completadas. `raw_revision` y `validation_revision` no crean
  por si solas otra tarea; siguen disponibles para diagnostico y seguridad. Una
  revision semantica nueva sustituye
  una pendiente antigua; si la identidad esta activa conserva como maximo un
  rerun pendiente con la revision mas reciente;
- `PushLoop(task, true)` es la via explicita de retry 3P: omite solo el ledger
  de revision ya completada. Sigue respetando deduplicacion pendiente/activa y
  coalescencia, por lo que nunca crea dos equivalentes simultaneas;
- una MEDIA lleva un `DatabaseUpdateTask` real y al terminar genera las BAJAS
  por KF indicadas en su payload;
- `Complete()` libera la identidad y despierta a productores/consumidor;
- `PendingStats()` separa `critical` de `maintenance`: fiducial, MEDIA y loops
  `Full` son criticos; `FusionRefresh` es mantenimiento. El total sigue
  observable, pero el mission gate usa watermarks 64/16 sobre pendientes
  criticos para que un backlog post-opt de fusion no detenga la trayectoria;
- no impone limite destructivo y conserva un unico worker no preemptivo.

## Referencias

```text
orbslam3_server/include/orbslam3_server/secondary_queue.hpp
  -> SecondaryTask / SecondaryTaskQueue
  -> rg -n "enum class SecondaryTaskPriority|class SecondaryTaskQueue"
orbslam3_server/test/test_secondary_queue.cpp
  -> prioridad, no preemption, FIFO, coalescencia causal y retry explicito que
     atraviesa solo el ledger completado, precedencia `Full` y conteos
     `critical/maintenance`
```
