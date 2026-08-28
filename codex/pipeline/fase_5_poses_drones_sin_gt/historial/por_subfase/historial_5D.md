# Historial de subfase 5D

## 2026-08-27 - Servicio y push dirigido

- objetivo intentado: resolver PENDING y revisiones sin polling ni arrays.
- archivos modificados: interfaces Dron/Servidor y `global_map_server.cpp`.
- build/tests: ambas copias de mensajes correctas; servidor 12/12 tras una correccion de formato.
- simulacion: 230 confirma servicio, reemplazo de referencia y push por commits primary/secondary.
- evidencia negativa: build inicial del wrapper fallo por lambda generica ROS Iron; corregido con `SharedFuture` explicito.
- conclusion: CONSEGUIDA.
