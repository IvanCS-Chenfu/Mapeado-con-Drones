# task_lib - resumen vigente

## Responsabilidad

`task_lib` concentra la logica pura que usa `task_server` para geometria de
mision, mapa voxel reversible, cobertura de fachada, planificacion D* y
reservas. No ejecuta ROS ni decide el lifecycle de una tarea.

## Mapa voxel y autoridades

Archivo principal: `servidor/task_lib/src/voxel_map.cpp`.

`ReversibleVoxelMap` conserva por separado:

- `OCCUPIED` respaldado por MapPoints ORB cualificados;
- `FREE` confirmado por el volumen fisico recorrido por un dron;
- `FREE` depth reversible, identificado por `source_id`.

La ocupacion sparse domina sobre cualquier contribucion depth. Sustituir o
retirar una observacion depth solo modifica sus propias celdas y no deja
endpoints ocupados. El FREE atravesado conserva autoridad independiente.

Simbolos utiles:

- `ReversibleVoxelMap::Snapshot`
- `ReversibleVoxelMap::ReplaceDepthFreeCells`
- `ReversibleVoxelMap::UpdateSparseOccupancy`
- `ReversibleVoxelMap::MarkTraversedFree`

## Barrido de fachada

Archivos:

- `servidor/task_lib/include/task_lib/facade_coverage.hpp`
- `servidor/task_lib/src/facade_coverage.cpp`

La geometria AB/BC/CD/DA se proyecta sobre una linea de fachada orientada.
`FacadeCoverage` conserva la union normalizada de intervalos recorridos y
calcula el ratio respecto al intervalo total. No existe un enum voxel
`COVERAGE`: la cobertura es una magnitud 1D independiente del estado
`FREE/OCCUPIED/UNKNOWN`.

`SelectFacadeCandidate` busca un punto dentro del volumen duro con costes
suaves y bilaterales alrededor de:

- distancia preferida a pared;
- desplazamiento preferido desde la pose actual;
- altura media del nivel.

Los pesos son configurables. El selector no aplica clearance ni reservas; esas
restricciones pertenecen al mapa navegable y a D*.

`LongestConnectedFreePrefix` recorta un corredor inspeccionado al mayor prefijo
conectado completamente FREE. El destino original solo se conserva si todo su
volumen barrido es conocido y libre.

La implementacion anterior `SurfaceCoverageAnalyzer` y los portales de ramas
se retiraron del runtime. Las ramas siguen siendo un requisito aplazado y no un
modo alternativo oculto.

## D* y reservas

Archivos:

- `servidor/task_lib/src/dstar_lite_3d.cpp`
- `servidor/task_lib/src/reservation_table.cpp`

`DStarLite3D` mantiene el grafo incremental y admite perfiles que exigen
`require_known_free=true`. Para el barrido de fachada, el destino y el corredor
proceden de la inspeccion depth y D* no usa UNKNOWN como atajo.

Las reservas son una capa temporal distinta de la ocupacion persistente. Se
tratan como bloqueadas para otros drones durante la planificacion, pero no
cambian el estado voxel base.

## Pruebas

Tests relevantes:

- `servidor/task_lib/test/test_facade_coverage.cpp`
- `servidor/task_lib/test/test_voxel_map.cpp`
- `servidor/task_lib/test/test_dstar_lite_3d.cpp`
- `servidor/task_lib/test/test_reservation_table.cpp`

Validacion vigente: build correcto y CTest `9/9`.
