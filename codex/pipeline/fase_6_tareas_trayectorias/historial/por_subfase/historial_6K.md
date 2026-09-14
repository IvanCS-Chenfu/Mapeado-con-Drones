# Historial 6K

## Prueba 682 - 2026-09-12

La FIFO pura aceptó D1 y D2, pero tras STOP D1 se colocó detrás de D2 en espera
por reserva. El HOLD de D1 impedía a D2 avanzar y D1 no podía reemplazarlo.
Resultado: `SIM-DONE` correcto, veredicto funcional parcial por interbloqueo.

## Prueba 683 - 2026-09-12

La reanudación del dueño de HOLD se insertó al frente. Los marcadores
`F6K-SUBTASK-READY priority=true`, el replan de D1 y el release posterior
demuestran la salida del interbloqueo; D2 fue despachado y más tarde coexistió
con una ruta ACTIVE de D1. Resultado: lifecycle FIFO/HOLD conseguido dentro del
alcance actual.

## Prueba 686 - 2026-09-12

La cola serializo decisiones y commits, pero no las ejecuciones: tras el
conflicto inicial de D1, ambos drones llegaron a ejecutar rutas `ACTIVE` de
forma simultanea en dos intervalos. Los STOP de D2 no bloquearon su runtime:
al acabar la action de parada, el servidor redujo su reserva a HOLD local y
despacho nuevas subtareas. `SIM-DONE success=true`, sin guard de recursos.
Resultado: se confirma que `WAITING` se reserva para falta real de alternativa,
y que los runtimes por `drone_id` pueden progresar en paralelo una vez
confirmadas sus reservas.
