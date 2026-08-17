# `PrimaryQueue`

## Rol

FIFO thread-safe que define el orden único del flujo principal desde 3C.

```text
orbslam3_server/include/orbslam3_server/primary_queue.hpp
PrimaryQueue / PrimaryInput / PrimaryInputKind / BackpressureHysteresis
rg -n "PrimaryInputKind|class PrimaryQueue|PushLive|PushReplay|MarkReady|WaitPop|class Backpressure"
```

## Semantica

- `PushLive()` asigna IDs monótonos; `PushReplay()` conserva IDs grabados.
- `PrimaryInput::source` distingue `live/replay`; `kind` distingue
  `delta/full_snapshot`. Ambos tipos comparten exactamente la misma FIFO.
- Una entrada se almacena inicialmente como no preparada.
- `MarkReady(arrival_id)` se llama después de emitir la telemetría de enqueue;
  el worker no puede extraerla antes. Esto mantiene el orden visual y lógico.
- `WaitPop()` usa `condition_variable`, sin polling ni lock durante el commit.
- `WaitUntilPendingBelow(limit)` permite al feeder replay esperar capacidad con
  la misma `condition_variable`; cada `pop` despierta al productor bloqueado.
- `Close()` deja de aceptar entrada, libera lo ya aceptado y permite drenaje.
- No descarta ni agrupa mensajes.

`BackpressureHysteresis` activa en `pending >= high` y libera en
`pending <= low`. El test `test/test_primary_queue.cpp` cubre productores
concurrentes, IDs replay, mezcla FIFO delta/snapshot, espera de capacidad,
liberación previa a dequeue e histéresis 8/2.
