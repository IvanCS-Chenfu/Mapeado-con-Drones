# Prueba 8.4-A - VIEW_UNKNOWN (repeticion visual)

Resultado: **CONSEGUIDA**.

- Wrapper y escenario: PASS (`success=true`), con GUI de Gazebo, GUI de mision y GUI global del servidor activas durante la ventana de observacion.
- El Trigger despacho `LOOK_AND_CAPTURE` correlacionado `autonomous:test_view_unknown_right:1:1`; el giro medido fue `-90.0039 deg`.
- La captura se clasifico como `vista_unknown`; fuentes del comando: `free=1`, `direct_free=0`, `occupied=0`.
- El coverage permanecio sin cambios. La continuidad posterior de la prueba se suprimio al materializar las fuentes.

Esta repeticion confirma visualmente la evidencia tecnica vigente de `run_03`; no sustituye sus artefactos como referencia primaria.
