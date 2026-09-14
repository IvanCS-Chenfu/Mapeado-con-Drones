# Historial 6N

## 2026-09-14 - Migracion al barrido de fachada

- objetivo intentado: sustituir la exploracion reiterada de destinos `UNKNOWN`
  por inspeccion depth bajo demanda y barrido lateral conocido `FREE`.
- archivos modificados: contratos `mission_msgs`, procesamiento depth de
  `orbslam3_ros2`, `InspectFacade` en `task_manager`, runtime de fachada en
  `task_server`, utilidades de `task_lib`, GUI F7, launch y documentacion.
- resultado de build: correcto en los paquetes dirigidos; CTest vigente de
  `task_server` 7/7, `orbslam3_server` 13/13, `task_lib` 9/9 y
  `multidron_gui_lib` 9/9.
- evidencia positiva: contratos, tests unitarios y cadena estatica compilan.
- evidencia ausente: faltaba una ejecucion integrada completa del barrido.
- conclusion: PARCIAL.

## 2026-09-14 - Prueba 734

- objetivo intentado: D1/GT hasta fiducial 2 y barrido automatico con una GUI.
- prueba Gazebo: no alcanzo ningun paso porque el runner recibio una ruta YAML
  relativa invalida desde la raiz del workspace.
- conclusion: NO CONSEGUIDA por error mecanico del escenario, sin evidencia
  funcional sobre 6N.

## 2026-09-14 - Pruebas 735 y 736

- objetivo intentado: repetir la validacion integrada con YAML absoluto.
- evidencia negativa: al anclarse D1, `InspectFacade` envio una reorientacion al
  mismo `gen_tray` unos instantes antes de terminar la llegada externa y
  sustituyo ese goal. Aumentar el periodo del worker no elimino la carrera.
- correccion: `gen_tray` publica `control/trajectory_active`; `task_manager`
  aplaza la inspeccion con `drone_busy`, y `task_server` no consume intentos por
  ese aplazamiento.
- builds/tests: `dron_individual`, `task_manager` y `task_server` correctos;
  CTest `task_server` 7/7.
- conclusion: NO CONSEGUIDAS como pruebas integradas; diagnostico y correccion
  implementados.

## 2026-09-14 - Prueba 737

- prueba Gazebo/F7: ejecucion terminal correcta tras 431 s, con una sola GUI y
  sin activar la guarda de recursos.
- evidencia positiva: la llegada externa termino `success=true`; la asignacion
  automatica solo comenzo despues. Queda validada la frontera comun de
  trayectoria fisica.
- evidencia negativa: una interrupcion fiducial permitio asignar DA antes de
  cerrar AB y el cierre anterior retiro la entrada `ready` nueva. Ademas, una
  captura depth vacia termino `success=false` con motivo incorrecto `ok`.
- correccion: la interrupcion fiducial bloquea nuevas asignaciones hasta su
  cierre y `InspectFacade` conserva el motivo real de cada etapa.
- conclusion: PARCIAL; guarda validada, barrido completo no validado.

## 2026-09-14 - Prueba 738

- prueba Gazebo/F7: interrumpida voluntariamente por el usuario tras 213 s; el
  helper cerro todos los procesos y no hubo presion de memoria.
- evidencia positiva: el diagnostico corregido aparecio como
  `target_capture_failed:exact_frame_not_buffered`.
- evidencia negativa: D2 entro en la cola antes que D1 aunque el escenario solo
  desplazaba D1. `ready_drones_` se consume solo por su frente, por lo que D2
  bloqueo el progreso de D1. No aparecieron plan D*, trayectoria de fachada ni
  incremento de coverage antes de la parada.
- conclusion: PARCIAL por interrupcion y bloqueo de planificacion entre drones.
- siguiente paso recomendado: arbitraje independiente por dron, retencion del
  frame exacto hasta `CaptureDepth`, reduccion del ruido
  `invalid_stereo_receipt` y nueva prueba D1 realmente aislada hasta validar
  inspeccion, FREE, D*, reserva, movimiento, coverage y `TO_FINISH`.
