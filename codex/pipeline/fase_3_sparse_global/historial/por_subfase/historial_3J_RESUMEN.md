# Historial 3J - resumen

## Estado vigente

`CONSEGUIDA`: solver privado reimplementado y validado. La live 145 confirma
30 propuestas convergentes con target a cero; la discontinuidad posterior se
origina al insertar KFs futuros en 3K, no en el solver.

## Estado actual

- Calcula una propuesta SE(3) privada; no muta bases ni publica.
- Fija el primer control, lleva target a observacion absoluta y conserva
  vecindades rigidas de extremos.
- Distribuye la correccion suavemente entre controles.
- Con fraccion 1.0 converge en una pasada; con fraccion menor puede devolver un
  parcial finito para refinamiento de la misma tarea.
- No usa GT global, covisibilidad ni variantes experimentales de `legacy2`.

## Evidencia

- Tests full/partial correctos.
- Replay 144 y los 30 commits live/replay v3 registran `converged` y error
  target `(0,0,0)`.

Detalle: `historial_3J.md`.
