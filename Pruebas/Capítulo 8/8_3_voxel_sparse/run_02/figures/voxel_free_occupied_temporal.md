# Serie temporal de voxeles

Fuente exclusiva: `run_02`, snapshots A/B/C. La figura muestra muestras publicadas, sin interpolar valores no medidos.

- `OCCUPIED`: voxeles materializados desde MapPoints sparse cualificados.
- `FREE`: el mensaje `VoxelMap` agrupa rayos sparse/RANSAC y volumen fisico libre asociado al movimiento y los KFs; no expone ambos subtotalizados.
- B se tomo antes del primer delta de KF y se conserva como evidencia temporal, no como validacion completa de la condicion B de 8.3.
