# Prueba 8.4-A - Resolucion de VIEW_UNKNOWN

## Conclusión vigente

La evidencia final es `run_03`: **CONSEGUIDA**. La captura `LOOK_AND_CAPTURE` se realizó a 90 grados a la derecha y fue procesada como `VIEW_UNKNOWN`, creando solo una fuente FREE de depth, sin endpoint OCCUPIED frontal ni coverage.

## Historial de ejecuciones

| Run | Resultado | Uso |
|---|---|---|
| `run_01` | PARCIAL | El flujo funcionó, pero el snapshot posterior no quedó correlacionado con la materialización y la autonomía posterior añadió un `VIEW_ADVANCE`. |
| `run_02` | NO CONSEGUIDA | La captura fue `VIEW_UNKNOWN`, pero el selector dinámico solo giró `-33.07 deg`. |
| `run_03` | CONSEGUIDA | Modo de prueba determinista: giro `-90.0009 deg`; fuentes depth `free=1`, `direct_free=0`, `occupied=0`; coverage sin cambio. |
| `run_04` | INTERRUMPIDA | Se detuvo a petición del usuario al no haberse iniciado la GUI del servidor; no se usa como evidencia. |
| `run_05` | CONSEGUIDA | Repetición visual con Gazebo, GUI de misión y GUI global: giro `-90.0039 deg`; fuentes depth `free=1`, `direct_free=0`, `occupied=0`; coverage sin cambio. |

El incremento agregado de OCCUPIED en `run_03` no se usa para atribuir evidencia depth, porque el mapa sparse se actualiza concurrentemente. La prueba se valida por los contadores por fuente del comando correlacionado.
