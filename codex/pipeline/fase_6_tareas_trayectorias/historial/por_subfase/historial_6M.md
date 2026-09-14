# Historial 6M

## 2026-09-13 - Cadena de orientacion yaw/pitch

- objetivo intentado: transportar yaw y `camera_pitch` desde el plan del
  servidor hasta la accion y el joint fisico sin controlador paralelo.
- archivos modificados: `TrajectoryPlan`, `TrayAction`, `task_manager`,
  `gen_tray`, GUI de trayectoria y politica inicial de `task_server`.
- builds/tests: `dron_individual` y `task_manager` correctos; CTest de
  `dron_individual` 8/8 correcto.
- prueba Gazebo/F7: 693 termina `success=true`; para `reorient_1_2`, el
  contrato enviado/recibido fue `pitch=-0.35 rad` y el joint recibio
  `-20.054 grados`; el terminal fue correcto.
- conclusion inicial: PARCIAL. Faltaba evaluar la seleccion de vista sobre
  paredes, suelo, techo, pasillos y los limites fisicos del joint.

## 2026-09-13 - Cierre de alcance con prueba 696

- la prueba 696 activa la ejecucion de coverage tras el anclaje GT y fuerza
  evidencia ORB pobre controlada;
- para `reorient_1_2`, `task_manager` envio yaw `1.573948 rad` y pitch
  `-0.35 rad`; `gen_tray` recibio el contrato y publico el joint a
  `-20.054 grados`;
- el terminal normal de la reorientacion reencolo la subtarea, por lo que yaw,
  pitch y lifecycle comparten la cadena normal de trayectoria;
- conclusion agregada: CONSEGUIDA con la politica inicial. La calibracion de
  geometria compleja se conserva como mejora futura, junto con depth 6N.

## 2026-09-13 - Limpieza de politica ordinaria

- se retiro de `BuildExecutablePlan` la eleccion automatica de yaw/pitch hacia
  pared, avance o distancia preferida; las rutas D* ordinarias conservan
  exclusivamente XYZ y valores de orientacion de entrada/neutros;
- se conserva el transporte fisico por waypoint para la unica excepcion activa:
  `REORIENT_FOR_TRACKING` de 6L;
- 698/699 volvieron a mostrar pitch neutro en rutas ordinarias. La evidencia
  real que active el paso visual de +/-25 grados sigue pendiente en 6L;
- conclusion: CONSEGUIDA como transporte; no existe politica ordinaria activa.

## 2026-09-13 - Orientacion ordinaria persistente y pruebas 700/701

- se detecto que un `TrajectoryWaypoint` D* sin orientacion explicita llevaba
  yaw cero al ejecutor. `task_server` conserva ahora yaw/pitch por
  `(drone_id,map_epoch)`, inicializa desde la navegacion autorizada y copia el
  valor a todos los waypoints ordinarios;
- el terminal normal de `visual_risk_reorient` es el unico que sustituye esa
  orientacion. Una muestra temporalmente no elegible no la borra; solo cambia
  al cambiar realmente el epoch;
- 700 completo tres tramos X+ y 701 tres ascensos Z+ con yaw aproximadamente
  `90 grados` y `camera_pitch=0`. Los STOPs de corredor de 701 se estabilizaron
  antes del siguiente tramo. No hubo reorientacion visual que medir en estas
  dos pruebas.
