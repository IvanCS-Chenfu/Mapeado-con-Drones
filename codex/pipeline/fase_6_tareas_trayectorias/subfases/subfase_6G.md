# Subfase 6G - VoxelMapBuilder, deltas y activacion de coverage

## Estado

`PARCIAL`. El materializador incremental ya deriva claims U reversibles en el
mismo tick; falta validar en una optimizacion real la retirada visual de claims
tras mover o borrar una fuente KF.

## Objetivo

Ser el unico escritor del mapa mundial derivado de `EvidenceDatabase` y aplicar
solo deltas: fuentes nuevas, tombstones, cambios de pose KF y revisiones. No
recalcula el mapa completo ni ejecuta logica de tareas.

## Contrato

En cada tick coalescido, inicialmente hasta 1 Hz, `VoxelMapBuilder` retira la
contribucion anterior de las fuentes afectadas y reproyecta solo sus celdas.
Publica `map_revision`, el delta de GUI/D* y
`sources_applied(workflow_id, source_ids, map_revision)`. Un cambio que toca
un corredor reservado despierta de forma prioritaria al monitor para STOP.

La misma transaccion que hace visible el delta deriva los claims de coverage U
asociados a fuentes depth. Cada claim queda indexado por `source_id`, KF y
seccion; una seccion esta activa mientras conserve al menos un claim. Se
recalcula solo para fuentes afectadas por alta, tombstone o cambio de pose KF:

- un `depth_occupied` frontal de `vista_pared` o `VIEW_ADVANCE` reclama su
  seccion espacial y `depth_coverage_neighbor_sections=2` secciones contiguas
  a cada lado en el orden de la U. El vecindario puede cruzar una esquina, pero
  nunca cierra la U por su cara abierta;
- cada voxel depth directo `OCCUPIED=1` reclama adicionalmente la seccion U
  volumetrica que contiene. Una sola captura puede, por tanto, activar varias
  secciones no vecinas si su evidencia ocupa varias rebanadas;
- un resultado `vista_pared` valido sin impacto hasta 4 m, con FREE fiable
  hasta esa distancia en una esquina sin fachada, activa su seccion como
  `SIN_FACHADA`;
- evidencia incompleta o `TRACKING_RISK` no cambia ninguna seccion.

La activacion es idempotente, queda en coste `10` y se publica al GUI como
estado de cobertura. Al retirar o mover una fuente, el builder elimina sus
claims y una seccion vuelve a coste `0` si no conserva otro. No es una capa de
ocupacion y no se entrega a D* como restriccion: 6H consume el coste para
escoger secciones, mientras D* consume solo el mapa navegable y reservas.

## Exclusiones

No calcula RANSAC/depth, no selecciona objetivos, no controla el dron y no
convierte por si mismo un score medio sparse en coverage. Mantiene la
procedencia KF/local para poder retirar o reproyectar evidencia sin reconstruir
todo el mundo.

## Validacion y exito

Mover una pose KF o borrar una fuente modifica solo sus celdas y claims. Un
impacto depth `1` activa las secciones U espaciales y su vecindario acordado;
una esquina `SIN_FACHADA` valida las activa igual. La revision que desbloquea
una continuacion es la misma que contiene los cambios de voxel y coverage.
