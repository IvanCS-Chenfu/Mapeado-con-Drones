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

## Coverage U de fachada

Archivos:

- `servidor/task_lib/include/task_lib/facade_coverage.hpp`
- `servidor/task_lib/src/facade_coverage.cpp`

`BuildFacadeCoveragePlan` construye las tres caras de una U, abiertas por la
cara mas cercana al centro del ROI, a un offset configurable. Las secciones se
almacenan en orden continuo de U, de modo que sus vecinos pueden cruzar una
esquina pero nunca cerrar la cara abierta. `FacadeCoverageSectionForPoint`
asigna una celda espacial a su rebanada U. No existe un enum voxel `COVERAGE`:
la cobertura es una capa derivada, reversible y separada de
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

`DStarLitePlanner` mantiene el grafo incremental y admite perfiles que pueden
exigir `require_known_free=true`. El barrido de fachada usa ese perfil estricto:
D* solo atraviesa FREE y nunca UNKNOWN, OCCUPIED o RESERVED. La misma identidad
de perfil debe conservarse en la simplificacion de la ruta, la cola, la reserva
y cualquier replanificacion.

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

## Score continuo depth

`ReversibleVoxelMap` conserva `sparse_score=sum(scores)/N` por identidades
MapPoint y declara `OCCUPIED` sobre `0.4`. Las fuentes
`ReplaceDirectDepthFreeCells` y `ReplaceDepthOccupiedCells` resuelven una
observacion frontal depth a `FREE=0` en rayo y `OCCUPIED=1` en endpoint; al
retirarlas reaparece la media sparse. `SelectFacadeCoverageCandidate` elige
una seccion U pendiente y una pose fisica de coste. Si el destino o el corredor
no son FREE, `task_server` inspecciona el objetivo visual y despues solo puede
despachar toda la ruta FREE o su prefijo FREE continuo. Nunca devuelve el
centro visual como pose fisica de vuelo.
