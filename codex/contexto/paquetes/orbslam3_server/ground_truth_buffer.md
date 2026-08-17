# `GroundTruthBuffer`

## Rol

Buffer temporal acotado para el adaptador fiducial simulado de 3E. No es una
base de mapa y no forma parte de la API de `FiducialAnchorManager`.

## Referencia

```text
orbslam3_server/include/orbslam3_server/ground_truth_buffer.hpp
  -> GroundTruthSample / GroundTruthMatch / GroundTruthBuffer
  -> rg -n "struct GroundTruth|class GroundTruthBuffer|FindNearest"
```

## Contrato

- `deque` de exactamente 50 muestras máximas por dron;
- inserción y expulsión O(1), con mutex propio;
- `Snapshot()` copia el tramo acotado y libera el mutex antes del cálculo;
- `FindNearest()` elige la muestra temporal más próxima y exige `max_dt`;
- distingue `no_gt_samples`, `invalid_keyframe_stamp`, `gt_history_expired`,
  `no_gt_within_threshold` y `outside_fiducial_radius`.

El servidor usa un callback group independiente y un executor multihilo para
que el ring siga avanzando mientras trabaja el flujo principal. El test
`test/test_ground_truth_buffer.cpp` cubre capacidad exacta, asociación y
rechazos temporales/espaciales.
