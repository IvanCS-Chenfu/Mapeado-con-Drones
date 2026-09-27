# Prueba 8.4-B - Vista frontal y coverage

## Conclusión vigente

**CONSEGUIDA en `run_14`.** D1 ejecutó `MOVE_AND_CAPTURE` a `(4,-9,1,90 grados)` dentro del workflow `test_view_wall_fixed:1` y terminó con `depth=1`. La captura de fachada escribió `FREE=1`, `DIRECT_FREE=1` y `OCCUPIED=1`; tras materializarse `depth_occupied:2108:1:0:33`, se activaron 13 de las 192 secciones U (`6,77 %` de progreso derivado). Esta es también la evidencia principal de [8.2-B](../8_2_B_coverage/notas.md).

El claim se registró después de la fuente depth, no por el desplazamiento. Los modos de navegación empleados para aislar esta ejecución fueron experimentales y se limitaron a `run_14`; los defaults del sistema permanecen intactos.

## Historial de ejecuciones

| Run | Resultado | Uso |
|---|---|---|
| `run_01` | NO CONSEGUIDA | El handoff, la asignacion y la tarea `map_section_level_0_AB` llegaron a `RUNNING`, pero `phase6_execute_facade_sweeps=false` impidio el worker legacy. No hubo `MOVE_AND_CAPTURE`, depth ni claims. |
| `run_02` | INTERRUMPIDA | Al activar `phase6_execute_facade_sweeps`, el worker legacy uso `inspect_facade` y eligio un candidato propio cerca de `(-0.62,-8.88,1.12)`. No corresponde al destino pedido y se detuvo sin usarla como evidencia. |
| `run_03` | INVALIDA | El launch recibio el perfil con una ruta relativa, no pudo abrirlo y termino antes de iniciar Gazebo o nodos ROS. Se repetira sin cambios funcionales usando la ruta absoluta. |
| `run_04` | INTERRUMPIDA | Gazebo y el escenario ya habian arrancado con la ruta absoluta, pero se detuvo por peticion del usuario antes de la evidencia para cerrar todos los nodos ROS. No se usa como resultado. |
| `run_05` | NO CONSEGUIDA | D1 se anclo correctamente. El Trigger recibio la llamada, pero la implementacion aun consultaba `SelectFacadeTaskCandidate` y rechazo `test_view_wall_fixed_section_unavailable`; nunca armo ni despacho el movimiento. Corregido: el candidato se crea directamente desde el objetivo fijo y su seccion U. |
| `run_06` | NO CONSEGUIDA | El Trigger fijo a (-2,8,1,90) se armo, pero el planificador rechazo la celda como goal_occupied_or_inflated. |
| `run_07-run_09` | NO CONSEGUIDA | Las repeticiones a (-6,-9,1,90) tambien fueron rechazadas antes del despacho. |
| `run_10` | PARCIAL | Con margen extra cero se despacho la ruta, pero una actualizacion del corredor la sustituyo por STOP antes de completar el desplazamiento. |
| `run_11-run_12` | NO CONSEGUIDA | La supresion experimental de STOP estaba activa, pero el objetivo se rechazo antes de crear trayectoria. |
| `run_13` | DIAGNOSTICA | Confirmo que (-6,-9,1) era UNKNOWN, no OCCUPIED; el modo estricto require_known_free lo bloqueaba. |
| `run_14` | CONSEGUIDA | D1 se anclo con GT y el workflow de prueba despacho MOVE_AND_CAPTURE a (4,-9,1,90). La captura genero FREE=1, DIRECT_FREE=1 y OCCUPIED=1; DEPTH_OCCUPIED reclamo 13 secciones U. Esta es la evidencia vigente. |

La repeticion ancla D1 con GT en `(0,-10,1,90 grados)` y llama al Trigger de prueba. En la evidencia vigente, el servidor planifica y ejecuta `MOVE_AND_CAPTURE` a `(4,-9,1,90 grados)`; no hay un segundo movimiento GT ni seleccion automatica alternativa. Los modos experimentales usados solo en run_14 permiten UNKNOWN, eliminan el margen extra y suprimen cambios de corredor para ese workflow.
