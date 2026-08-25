# Historial 4B

## 2026-08-24 - Prueba 201, trayectoria completa

- objetivo intentado: validar spawn visual y trayectoria segura del bloque 4A+4B;
- prueba Gazebo: `tray_prueba_201.yaml`, Gazebo GUI y RViz2 activos, ambos grafos web desactivados;
- patrones de reduccion: spawn/texturas/SDF, scenario runner, procesos GUI/web y errores;
- evidencia positiva: 15 texturas verificadas, 3/3 objetos creados, readiness recibido, 5 movimientos y 10/10 goals con `success=true`;
- evidencia de debug: RViz2 arranco y cerro limpio; no arrancaron bridges web, navegadores ni telemetria arquitectonica;
- evidencia negativa: el spawner salio 1 por doble `rclpy.shutdown()` despues de `SIM-DONE`; Gazebo salio 255 durante cleanup;
- interpretacion: la ruta funcional y la trayectoria pasan; el defecto del spawner era exclusivamente de cierre;
- revision visual posterior: el usuario confirma que la prueba fue perfecta en
  Gazebo; no observo defectos de texturas, caras, z-fighting ni colisiones;
- interpretacion revisada: spawn y visualizacion quedan conseguidos. La unica
  carencia fue que los puntos elegidos para esta trayectoria no representan la
  prueba tipica que el usuario quiere conservar;
- conclusion revisada: CONSEGUIDA; la ruta posterior es una mejora de escenario
  cuya ejecucion se aplaza por decision explicita del usuario.

## 2026-08-24 - Prueba 202, regresion de cierre

- objetivo intentado: confirmar readiness y el mismo apagado sin repetir la trayectoria larga;
- prueba Gazebo: espera `/fiducial_spawn_ready`, estabiliza 2 s y finaliza sin mover drones;
- evidencia positiva: scenario y herramienta exit 0, spawner terminado limpiamente, sin traceback ni `FID-SPAWN-ERROR`, RViz2 limpio y web desactivado;
- evidencia negativa: exit 255 conocido de Gazebo tras `SIGINT`;
- conclusion: CONSEGUIDA para spawn/cierre automatizado; la revision visual de
  201 tambien queda confirmada correcta;
- siguiente paso recomendado revisado: preparar la trayectoria tipica y aplazar
  su ejecucion al bloque 4C+4D, segun la decision posterior del usuario.

## 2026-08-24 - Ajuste estatico de la trayectoria tipica

- objetivo: conservar el rodeo por las aristas del cuadrado ±10 y detener cada
  lado en su punto medio `(0,±10)` o `(±10,0)`;
- archivos: YAML auxiliar y escenario instalado de `simulacion_dron`;
- cambios: todos los antiguos objetivos ±9 pasan a ±10; se anaden los puntos
  medios laterales de subida y bajada; se conserva readiness antes de mover;
- verificacion: ambas copias son identicas y `test_fiducial_contract.py` pasa 6/6;
- simulacion: no ejecutada por peticion explicita del usuario; se aplaza a 4C+4D;
- conclusion: CONSEGUIDA. La evidencia 201/202 cierra 4B y la ruta revisada
  queda preparada para la siguiente prueba integral.

## 2026-08-25 - Prueba 212, yaw relativo y bloqueo previo

- objetivo intentado: revisar la trayectoria tipica con seis giros relativos alrededor de `±180°`, Gazebo/RViz2 y ventanas fiduciales de 5 s;
- cambios previos: dron 2 usa `+90/0/+90` relativo y dron 1 `-90/0/-90`; copias identicas, contratos 19/19 y build `simulacion_dron` correcto;
- evidencia positiva: primer movimiento al fiducial 2 completo 2/2 goals; visualizadores mostraron tags y cerraron por timeout; recursos estables;
- evidencia negativa: un `LoopTask` de dos submapas fue rechazado en commit por `commit_pose_store_hard_constraint_violation` y se clasifico como hard failure;
- efecto: `blocking_failure=true` mantuvo backpressure activo; el paso 5 empezo, pero no envio goals. Codex interrumpio la prueba tras 548 s;
- interpretacion: ninguno de los seis yaw modificados llego a ejecutarse; no es evidencia negativa del cambio de trayectoria ni del visualizador;
- conclusion: NO CONSEGUIDA como repeticion visual; bloqueada por una salvaguarda loop previa al tramo modificado;
- siguiente paso recomendado: acordar si se corrige primero el fallo duro o se hace una prueba visual que ignore explicitamente el mission gate.

## 2026-08-25 - Prueba 213, repeticion sin fallo bloqueante

- cambio previo: eliminado por decision del usuario el latch
  `secondary_blocking_failure_`; los fallos secundarios conservan log/contador,
  pero no mantienen el backpressure despues de terminar la tarea;
- validacion: build `orbslam3_server` 1/1 y suite 12/12 targets, 61 tests, cero
  errores/fallos (9 skipped); el contrato dedicado impide reintroducir el latch;
- escenario: trayectoria tipica completa con Gazebo/RViz2, grafos web apagados
  y visualizadores fiduciales a 5 s; exit 0, `SIM-DONE success=true` y 17/17 pasos;
- goals: 22 enviados y 22 resultados `success=true`; el paso 5 y todos los
  tramos posteriores se ejecutaron;
- backpressure: las optimizaciones rechazadas terminaron con
  `optimization_active=false`, publicaron `active=false` y permitieron nuevos
  goals; no hubo ningun `F3L-HARD-FAILURE` real;
- visualizador: 2 READY, 74 publicaciones, 74 SHOW y cierres por timeout; no
  murieron wrappers, visualizadores ni RViz2;
- recursos: guarda inactiva, minimo disponible 4996.2 MiB y servidor RSS maximo
  269.0 MiB; Gazebo 255 aparece solo en cleanup posterior al exito;
- valoracion visual posterior: el usuario da 4A-4F por concluidas, pero observa
  derivas que las optimizaciones loop debian corregir y no corrigieron;
- conclusion revisada: CONSEGUIDA para las subfases de Fase 4. La prueba 213
  queda `A REVISAR DE NUEVO EN 3Q`; no valida la calidad de correccion loop.
