# Historial 7C

## 2026-09-02 - Bloque 1

`GuiDataModel` rechaza estados anteriores por `map_epoch`, `sample_sequence` y
`pose_revision`. El bridge registra tiempo monotono de recepcion y, tras el
timeout configurable, conserva la ultima pose y marca el dron perdido.

CTest cubre update anterior rechazado y stale sin borrar pose. En 377 se cargan
tres fiduciales y las suscripciones de ambos drones.

Conclusion: `CONSEGUIDA`.
