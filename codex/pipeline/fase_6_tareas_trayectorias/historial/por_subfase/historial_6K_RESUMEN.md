# Resumen histórico 6K

## Estado vigente

`PARCIAL`. Cada subtarea se identifica por dron en `ExecutionRuntime`; ya no
hay estado global ni selección exclusiva de D1. La cola FIFO serializa la
decisión/commit y permite ejecutar en paralelo las rutas ya confirmadas.

## Evidencia

- Build y CTest `task_server`: 7/7.
- 682 encontró el caso HOLD/FIFO que bloqueaba el progreso.
- 683 valida la excepción mínima: STOP reencola al dueño de HOLD al frente;
  después D1 libera, D2 recibe su subtarea y ambos llegan a ACTIVE.
- 686 confirma que la cola solo serializa decisión/commit: D1 y D2 ejecutaron
  rutas `ACTIVE` en paralelo, y los STOP liberaron MOVING para dejar HOLD local.

## Pendiente

No están implementadas prioridades generales P1/P2, anti-starvation, ni la
reparación de prefijos/sufijos. Los STOP repetidos de cobertura pertenecen a la
política de corredor/ocupación de 6I y no se reinterpretan como fallo de FIFO.
La revision manual 688 observo que las primeras rutas pueden arrancar de una en
una por conflicto de reservas cerca del fiducial; cuando hay corredores
compatibles, ambos runtimes vuelven a ejecutar en paralelo. Se conserva la
serializacion decision/commit y se difiere la busqueda anticipada de alternativas.
