# Ultima sesion

## Fase 6 - Claims U reversibles, prueba 786

Se sustituyo la activacion irreversible de coverage U por claims reversibles
por `source_id` y KF. El materializador ahora informa fuentes OCCUPIED
proyectadas y retiradas; al cambiar una fuente se recalculan sus secciones y
una tombstone retira sus claims. Un impacto frontal reclama su seccion, dos
vecinas continuas y las rebanadas espaciales alcanzadas. La U se almacena en
orden continuo y `SIN_FACHADA` queda vinculado a una fuente FREE.

El default de `extra_obstacle_clearance_voxels` pasa a `1`; D1 usa inflacion
`(3,3,2)` y reserva `(2,2,1)`. Builds de `task_lib`, `task_server` y
`simulacion_dron` correctos; CTest `task_lib` y `task_server`, 9/9.

La prueba 786 con D1/GT/depth durante 180 s cerró `SIM-DONE success=true`.
Dos fuentes `VIEW_ADVANCE` materializaron 14 y 13 claims, dejando 18 secciones
activas; GUI publicó reservas. Los tres STOP tuvieron causa trazada: dos
`unresolved_navigation_change` sobre UNKNOWN y uno
`raw_occupied_static_clearance`.

Estado: 6G/6H/6I siguen `PARCIAL`. Falta observar en Gazebo la retirada de
claims tras una optimización/tombstone, el relevo `TO_FINISH`, la progresión
de fachada sostenida y decidir la política de cambios navegables UNKNOWN.
