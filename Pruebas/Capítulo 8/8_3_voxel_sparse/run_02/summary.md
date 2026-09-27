# Prueba 8.3 - mapa voxel sparse sin depth

Resultado: **PARCIAL**.

- Wrapper y escenario PASS; handoff autonomo omitido y depth desactivado.
- El capturador temporal corrigio el conteo final de keyframes, pero B se tomo antes de recibir el primer delta de KF: `map_revision=2`, `FREE=21`, `OCCUPIED=0`, `keyframes=0`.
- La dinamica es valida, pero B no representa aun la materializacion sparse historica exigida.

## Serie temporal disponible

La serie usa exclusivamente los snapshots A/B/C de `run_02`, sin interpolar muestras no registradas:

| Snapshot | Tiempo desde A (s) | KFs con evidencia | FREE | OCCUPIED |
|---|---:|---:|---:|---:|
| A | 0.000 | 0 | 0 | 0 |
| B | 0.120 | 0 | 21 | 0 |
| C | 50.955 | 46 | 11568 | 598 |

- Figura convencional: `figures/voxel_free_occupied_temporal.png` (tambien se conserva el SVG).
- Datos: `processed/voxel_counts_temporal.csv`.
- `OCCUPIED` corresponde a MapPoints sparse cualificados. `FREE` es el agregado publicado por `VoxelMap` de rayos sparse/RANSAC y volumen fisico libre asociado al movimiento/KFs; la interfaz no separa ambos subtotalizados.
