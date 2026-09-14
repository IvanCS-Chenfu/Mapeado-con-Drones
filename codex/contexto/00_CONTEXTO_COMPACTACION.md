# Contexto de compactacion

## Trabajo activo

Peticion vigente: continuar la migracion desde la exploracion volumetrica de
destinos `UNKNOWN` al barrido lateral de fachadas con las correcciones y pruebas
acordadas.

Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA
Dudas abiertas: ninguna
Trabajo activo: si

## Contrato vigente

- El servidor selecciona destinos de barrido sobre la fachada de cada
  `MAP_SECTION`. AB/BC/CD/DA se tratan con la misma geometria rotada y sus caras
  laterales delimitan los extremos del intervalo.
- Los costes son suaves y bilaterales alrededor de tres preferencias:
  distancia a pared `2.5 m`, desplazamiento `2.0 m` y altura media del nivel,
  inicialmente con pesos iguales.
- Antes de moverse, el dron ejecuta `InspectFacade`: captura depth de la
  fachada, gira hacia el corredor, realiza una segunda captura y restaura la
  orientacion inicial. Si el corredor completo no queda `FREE`, puede ejecutar
  solo un prefijo conectado completamente `FREE` de al menos `1 m` y repetir la
  inspeccion desde alli.
- Depth persistente aporta exclusivamente evidencia `FREE` reversible. Los
  MapPoints ORB cualificados son la unica fuente `OCCUPIED`; depth nunca puede
  borrar esa autoridad y el paso fisico del dron conserva mayor autoridad FREE.
- D* planifica solo sobre corredor conocido `FREE`; se conservan reservas,
  STOP, Pol3Waypoints, colas y runtime por dron y trayectoria activa en GUI.
- El movimiento mantiene yaw/pitch de la primera captura de fachada. El
  tracking risk durante la mirada temporal usa solo el frame exacto que activa
  la persistencia y despues restaura la orientacion de fachada.
- Coverage es la union 1D de intervalos recorridos fisicamente mientras la
  orientacion es valida. Umbral de finalizacion `0.99`. Una tarea parcial al
  alcanzar un extremo pasa a `TO_FINISH` y vuelve a competir por cercania al
  intervalo pendiente.
- Tras tres inspecciones fallidas, la tarea pasa a `TO_FINISH`, se desasigna y
  el dron vuelve a la cola general.
- Un fiducial primary nuevo para `(drone_id, map_epoch, object_id)` interrumpe
  el barrido, libera la ejecucion y deja actuar al pipeline fiducial existente;
  no se modifica el optimizador de Fase 3.
- Al seleccionar una tarjeta de tarea, GUI F7 muestra el subROI y la linea de
  coverage con el mismo color. Las ramas quedan documentadas y aplazadas.

## Estado implementado

- Contratos de subfase `6H`, `6I`, `6L`, `6M`, `6N`, `6O`, `6P` y `7I`
  migrados al nuevo modelo.
- Ambas replicas de `mission_msgs` incorporan `FacadeCoverageInterval`,
  `TaskState.TO_FINISH`, `CaptureDepth`, `InspectFacade`,
  `FiducialPrimaryObservation` y el producto depth enriquecido; son identicas y
  compilan.
- `task_lib` implementa cobertura de fachada, seleccion de candidato, prefijo
  FREE, autoridad voxel por fuente, D* y reservas. CTest: 9/9.
- `orbslam3` conserva un buffer estereo acotado, captura depth bajo demanda por
  frame exacto, muestreo uniforme y normal depth; ya no publica depth
  automaticamente por cada KF.
- `task_manager` implementa `InspectFacade`, giro temporal, segundo depth o
  frame exacto de tracking risk, STOP local y restauracion de yaw/pitch.
- `task_server` implementa el runtime de fachada, integracion depth FREE,
  relleno conservador de huecos de una celda, D* `require_known_free`,
  intervals/`TO_FINISH` e interrupcion fiducial primary.
- `orbslam3_server` publica el evento primary antes del backend, sin cambiar el
  optimizador.
- `multidron_gui_lib` propaga intervalos y dibuja la linea al seleccionar la
  tarea. CTest: 9/9.
- Launch y grafos web exponen solo el flujo vigente de fachada. Builds finales
  de `task_manager`, `orbslam3_server`, `task_server`, `multidron_gui_lib` y
  `simulacion_dron`: correctos. CTest `task_server`: 7/7;
  `orbslam3_server`: 13/13. Contratos web dirigidos: 29/29.

## Validacion pendiente

Prueba acordada 1: Gazebo y una unica GUI F7, D1 con control GT hasta fiducial
2 `(0,-10,Z)`, asignacion automatica de `MAP_SECTION` y barrido hasta una cara
lateral. Debe validar inspeccion bajo demanda, corredor/prefijo FREE, D*,
reserva, orientacion mantenida, coverage lineal y `TO_FINISH`.

Prueba acordada 2: mirada controlada hacia una zona visualmente pobre. Debe
validar que `TRACKING_RISK` emplea el ultimo frame exacto como segunda captura,
realiza la accion local prevista y restaura la orientacion de fachada.

El CTest global de `simulacion_dron` tuvo fallos externos o previos al cambio:
escenario fiducial auxiliar divergente de la copia canonica, entorno
`matplotlib` que exige `numpy>=1.23` frente a `1.21.5`, y formato acumulado de
`scenario_runner_node.cpp`. El fallo `flake8` del launch ya fue corregido. No
se deben ocultar ni atribuir esos tres fallos a la migracion.

## Checkpoint de reanudacion

Reanudacion tras compactacion (2026-09-14): este archivo se relee fisicamente
y se reconcilia con la orden vigente `Sigue`. Se elimina la acumulacion de
checkpoints historicos que habia convertido esta memoria breve en un archivo de
miles de lineas; la evidencia cronologica permanece en historiales de subfase.

Las fichas compactas de `mission_msgs`, `task_lib`, `task_manager`,
`task_server`, `multidron_gui_lib`, `orbslam3_ros2`, `orbslam3_server` y los
launches ya reflejan el runtime de fachada. `git diff --check` es correcto y
las replicas `mission_msgs` son identicas.

Prueba preparada: `734`, YAML
`codex/archivos_auxiliares/trayectorias/tray_prueba_734.yaml`. Launch:
`simulacion_dron multi_dron.launch.py` con D1/GT, Gazebo, una unica GUI F7,
sin GUI legacy ni RViz, barrido de fachada e inspeccion/evidencia depth
activados, riesgo visual activo y STOP depth desactivado para aislar el flujo
normal. `multidron_gui_start_delay_sec=20`, cero reintentos de Gazebo, timeout
de escenario 520 s y espera final 10 s.

Resultado 734: `run_simulation.sh` y `scenario_runner_node` terminaron con
codigo 1 tras unos 32 s. El helper limpio launch y ambas GUI; la guarda de
recursos no se activo. Log completo preservado en
`codex/archivos_auxiliares/logs/prueba_734.log`, que no se leera directamente.
Este intento no constituye evidencia sobre el barrido porque fallo durante el
arranque del escenario.

Diagnostico reducido 734: el runner no pudo abrir la ruta relativa
`codex/archivos_auxiliares/trayectorias/tray_prueba_734.yaml` porque se ejecuta
desde la raiz del workspace. No se alcanzo ningun step. La correccion es solo
mecanica: prueba nueva `735`, YAML equivalente y argumento `--yaml` absoluto.

Prueba preparada: `735`, mismo launch, flags, timeout y criterios de 734.

Resultado 735: el YAML absoluto se cargo, pero `scenario_runner_node` termino
con codigo 250 antes de la espera larga. `run_simulation.sh` completo la
limpieza, incluida terminacion forzada del grupo launch; la guarda de recursos
no se activo. Log completo preservado en
`codex/archivos_auxiliares/logs/prueba_735.log`, que no se leera directamente.

Diagnostico reducido 735: no hubo `TRACKING_RISK` ni `STOP`. El goal manual de
D1 comenzo correctamente y, unos 0.2 s antes de terminar, el anclaje hizo que
`task_server` asignase `map_section_level_0_AB` y lanzase `InspectFacade`. La
reorientacion local uso el mismo `gen_tray`, sustituyo el goal externo aun
activo y este termino abortado. Es una carrera de orquestacion de la llegada
manual de prueba, no evidencia contra el barrido de fachada.

Reanudacion vigente (2026-09-14): tras la nueva compactacion se ha releido
fisicamente este archivo y se ha reconciliado con la ultima orden `Sigue`.
La autorizacion y el contrato no cambian.

Prueba preparada: `736`, mismo movimiento y espera que 735. Para aislar la
carrera externa se usara `phase6_facade_worker_period_sec:=2.0`; tras completar
la llegada, el planificador automatico conserva todo el comportamiento
acordado. Antes de lanzarla se comprobara que no queda ningun proceso ROS,
Gazebo o GUI de pruebas anteriores.

Resultado 736: el runner volvio a terminar con codigo 250 antes de alcanzar la
espera larga, pese al periodo de worker de 2 s. `run_simulation.sh` termino su
limpieza y la guarda de recursos no se activo. El intento demuestra que
espaciar el timer no resuelve de forma determinista la carrera; no se debe
seguir ajustando el escenario por temporizacion.

Correccion implementada tras 736: `gen_tray` publica
`control/trajectory_active` transient-local para todos los goals;
`task_manager` rechaza temporalmente `InspectFacade` con `drone_busy` si la
trayectoria fisica sigue activa, y `task_server` aplaza ese resultado sin
consumir los tres intentos de inspeccion. Se actualizaron los contratos 6I/6N
y la documentacion vigente de los tres paquetes.

Siguiente accion exacta: compilar `dron_individual`, `task_manager` y
`task_server`; corregir solo errores mecanicos, ejecutar sus tests dirigidos y
repetir la validacion integrada como prueba nueva.

Build preparado: comando
`./codex/herramientas/build_selected_packages.sh dron_individual task_manager task_server`
para validar la nueva frontera comun de actividad fisica.

Resultado build inicial: codigo 2, sin iniciar `colcon`; la herramienta exige
un unico grupo por invocacion. Es un error mecanico de sintaxis, no de codigo.
Se repetira en dos builds: grupo `dron` para `dron_individual task_manager` y
grupo `servidor` para `task_server`.

Segundo intento de build: codigo 2, tambien sin iniciar `colcon`; la
herramienta acepta exactamente un paquete por invocacion. Se ejecutaran tres
builds pequenos independientes con su grupo correspondiente.

Build `dron_individual`: codigo 0, correcto. Log completo de build conservado
por la herramienta en `codex/archivos_auxiliares/colcon_build.log`. Siguiente:
compilar `task_manager`.

Build `task_manager`: codigo 0, correcto. Log completo renovado en
`codex/archivos_auxiliares/colcon_build.log`. Siguiente: compilar
`task_server`.

Build `task_server`: codigo 0, correcto; solo conserva avisos previos de APIs
QoS deprecadas. Los tres paquetes modificados compilan. Siguiente: ejecutar
CTest dirigido en `task_server` y comprobar formato/diff antes de la nueva
simulacion.

Primer intento CTest `task_server`: sin resultado tecnico; la revision
automatica del permiso caduco antes de completar la llamada. Se repetira una
vez y no se clasificara este intento como fallo del paquete.

CTest `task_server` repetido: codigo 0, 7/7 tests correctos. Siguiente:
`git diff --check`, preflight de procesos y prueba integrada 737.

Preflight 737: `git diff --check` correcto y no quedan procesos Gazebo, GUI,
runner ni nodos ROS de pruebas anteriores. La prueba usara el periodo normal
del worker para validar que la guarda depende de
`control/trajectory_active`, no de una demora artificial.

Resultado terminal 737: `scenario_runner_node` y `run_simulation.sh`
terminaron con codigo 0 tras 431 s. La llegada externa completo y se mantuvo la
ventana automatica de 360 s; no se activo la guarda de recursos. Pico del grupo:
2208.5 MiB RSS y 604.5 % CPU. Log completo preservado en
`codex/archivos_auxiliares/logs/prueba_737.log`, que no se leera directamente.

Diagnostico reducido 737: la guarda fisica queda validada; el goal externo
termino `success=true` y la primera asignacion se produjo despues. El barrido
no alcanzo D*: AB fue interrumpida correctamente por una primary nueva del
mismo fiducial en `map_epoch=1`; antes de finalizar esa inspeccion se asigno DA
y el cierre de AB retiro por error la entrada `ready` que ya pertenecia a DA.
Ademas, una segunda captura depth vacia produjo `success=false` pero la
restauracion posterior sobrescribio el motivo con `ok`.

Correccion preparada: bloquear asignaciones mientras exista
`fiducial_interrupt_task_by_drone_` y conservar motivos independientes para
giro, captura objetivo y restauracion. Son correcciones de secuenciacion y
diagnostico que mantienen el contrato acordado.

Correcciones aplicadas y documentadas: la interrupcion fiducial es ahora una
barrera de asignacion hasta su cierre, y `InspectFacade` conserva el motivo
terminal de cada etapa. Siguiente build: `task_manager` (grupo dron) y
`task_server` (grupo servidor), uno por invocacion.

Build `task_manager` tras 737: codigo 0, correcto. Siguiente: build
`task_server`.

Build `task_server` tras 737: codigo 0, correcto; solo avisos QoS previos.
Siguiente: CTest dirigido 7/7 esperado y nueva prueba integrada 738.

CTest tras 737: codigo 0, 7/7. `git diff --check` correcto y preflight sin
procesos residuales. Prueba 738 preparada con 180 s de actividad automatica
para validar secuenciacion tras interrupcion, tres reintentos diagnosticos y
aparicion de un plan D* sin repetir aun otra ventana de seis minutos.

Resultado terminal 738: detenida por peticion explicita del usuario mediante
SIGINT tras 213 s totales y unos 156 muestreos. El helper ejecuto su limpieza,
cerro GUI/Gazebo/nodos y la guarda de recursos no se activo. La prueba es
parcial por interrupcion voluntaria, no un fallo tecnico. Log completo
preservado en `codex/archivos_auxiliares/logs/prueba_738.log`; se reducira solo
para informar del progreso observado hasta la parada.

Diagnostico reducido 738: la correccion de motivos queda validada porque la
inspeccion fallo explicitamente con
`target_capture_failed:exact_frame_not_buffered`. D2 se considero elegible y
entro en la cola antes que D1, aunque el escenario solo desplazaba D1. La cola
global `ready_drones_`, consumida exclusivamente por su frente, produjo bloqueo
entre drones: D2 retuvo el turno y D1 no avanzo. Antes de la parada no aparecio
ningun plan D*, trayectoria de fachada ni incremento de coverage. No se valida
todavia el flujo funcional completo.

Pendientes tecnicos vigentes: convertir la cola de inspeccion en arbitraje
independiente por dron o eliminar el bloqueo por frente global; garantizar que
el frame exacto de `TRACKING_RISK` permanezca disponible hasta `CaptureDepth`;
revisar y limitar el ruido `invalid_stereo_receipt`; aislar de verdad D1 en la
prueba; y repetir hasta validar inspeccion doble, evidencia FREE, D*, reserva,
ejecucion, coverage y `TO_FINISH`.

Checkpoint de preparacion (2026-09-14): se ha releido fisicamente este archivo
y se ha reconciliado con la peticion de continuar. Se propone sustituir el
consumo exclusivo del frente global por secuenciacion independiente por dron,
conservar el frame exacto hasta que `CaptureDepth` lo consuma y limitar el ruido
diagnostico sin alterar la autoridad voxel. No quedan procesos activos.

Acuerdo de reanudacion: sustituir el bloqueo por frente global por secuenciacion
independiente por dron; retener el frame exacto hasta que `CaptureDepth` lo
consuma; limitar el ruido `invalid_stereo_receipt`; y validar primero con una
simulacion que lance exclusivamente D1. Si el flujo monodron se valida, realizar
una segunda prueba breve con D1 y D2 para comprobar concurrencia.

Prueba acordada: una GUI F7 y Gazebo, solo D1 con GT hasta fiducial 2
`(0,-10,Z)`, seguido del runtime automatico hasta observar inspeccion doble,
evidencia FREE, D*, reserva, ejecucion y aumento de coverage. Segunda prueba
condicionada: dos drones para verificar ausencia de bloqueo mutuo.

Autorizacion funcional: CONCEDIDA
Trabajo activo: si
Siguiente accion exacta: leer las fichas de `task_server`, `task_manager`,
`orbslam3_ros2` y `simulacion_dron`; localizar los simbolos de cola, captura y
numero de drones; implementar las correcciones acordadas.
