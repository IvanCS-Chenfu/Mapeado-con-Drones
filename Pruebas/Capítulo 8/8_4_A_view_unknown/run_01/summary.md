# Prueba 8.4-A - VIEW_UNKNOWN

Resultado: **PARCIAL**.

El wrapper y el escenario terminaron correctamente. D1 ejecutó un `LOOK_AND_CAPTURE` real, terminó con `look_completed`, capturó depth y el servidor escribió/aplicó una fuente `kind=vista_unknown`. No hubo coverage asociado a ese `command_id`.

El capturador original fijó el snapshot posterior al primer crecimiento FREE tras solicitar UNKNOWN, antes de la materialización depth correlacionada. Después, el runtime emitió un `VIEW_ADVANCE` distinto que sí activó coverage. Esta ejecución se conserva como evidencia del flujo, pero no se usa para concluir la prueba.
