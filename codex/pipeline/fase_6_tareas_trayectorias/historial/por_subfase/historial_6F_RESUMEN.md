# Resumen - 6F

Estado agregado: PARCIAL.

La confirmacion local previa se conserva como antecedente. El bloque nuevo
anade `DepthIntegrationWorker` desacoplado: `ReportAutonomousResult` persiste
y encola, el worker escribe fuentes locales tipadas y `VoxelMapBuilder` libera
la continuacion solo al aplicar sus IDs. `vista_pared` puede crear OCCUPIED
directo; `vista_unknown` solo FREE. Build correcto y CTest 9/9, incluidas
pruebas de pose tardia y repeticion. Falta conectar runtime 6D y validar en
Gazebo; `InspectFacade` legacy sigue aislado.

Actualización 778/779: `VIEW_ADVANCE` cubre el prefijo FREE de D* y el avance
FREE de respaldo tras `vista_unknown`. Ambos capturan depth, escriben fuentes
y esperan `sources_applied` antes de reseleccionar. La 779 valida terminales
`depth=1` y fuentes `kind=view_advance`; la 778 se conserva como el fallo que
reveló el antiguo `depth=0`. `VIEW_ADVANCE` no activa coverage U. Quedan STOP
repetidos y el despeje insuficiente de destinos UNKNOWN. Estado: PARCIAL.
