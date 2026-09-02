# Contexto de compactacion

## Preparacion bloque 3 de Fases 6/7

Peticion vigente: ejecutar el bloque 3 funcional, formado por 6A-6C.
Preparacion: `CERRADA`. Acuerdo cerrado: `si`. Autorizacion funcional:
`CONCEDIDA`. Prueba acordada: integracion estacionaria de dos drones con
Gazebo, GUI F7, grafo F6 y pausa manual, mas unitarios, guardas y builds; GT
permitido solo como fuente de simulacion/control actual y nunca como entrada
funcional de F6. Dudas abiertas: ninguna.

Decisiones confirmadas: `mission_flow` sera un grafo dedicado y
`system_architecture` se ampliara tambien. Cada subROI ocupa exactamente el
50 % del slice: longitud completa de su lado y profundidad hasta el centro.
Cada par adyacente solapa en un cuarto del slice, equivalente a la mitad de
cada subROI; los pares opuestos solo comparten la frontera central. No existe
ratio de solape configurable. Canon confirmado: `A=(xmin,ymin)`,
`B=(xmax,ymin)`, `C=(xmax,ymax)`, `D=(xmin,ymax)`. La histeresis es
`[hx,hy,hz]` en metros no negativos y expande min/max en ambos sentidos para
derivar `hard_flight_volume`.

Tambien quedan aceptados: `RegisterDrone` idempotente por servicio; snapshots
`reliable+transient_local`; action futura para ejecucion de planes; eventos
reliable para excepciones; handshake extensible con dimensiones, perfil,
version de protocolo/generador y `capability_mask`; vista 2D de niveles/subROIs
en `mission_flow`; y la convencion A-B-C-D indicada. Desde 6B se publica
geometria real no asignada y se prepara en GUI una `MissionRegionLayer`: el
panel derecho muestra `Regiones de mision` hasta 6E, cuando las tareas reales
se integraran en las tarjetas del dron. La seleccion es unica: otra tarea u
objeto la sustituye y clicar vacio la limpia. El prisma seleccionado usa
relleno translucido, contorno y etiqueta `Nivel N - lado`; no se inventan
assignments runtime.

Objetivo preparado: crear esqueletos compilables `task_server`, `task_lib`,
`task_manager`, `task_manager_lib` y replicas exactas de `mission_msgs`; parser
puro del YAML de mision y `hard_flight_volume`; niveles/cuatro subROIs y modelo
de ownership; registro idempotente, lifecycle y contratos versionados; grafo
web incremental desde 6A. Exclusiones: voxel map, allocator, frontiers, planner,
reservas, ejecucion, movimiento y GT funcional. No se ha modificado codigo,
launch, YAML ni configuracion ni se ha compilado/simulado. Siguiente accion
exacta: auditar patrones reales de paquetes, launch, GUI, grafos, interfaces y
guardas; despues registrar archivos criticos antes de implementar. Plan:
Servidor (`mission_msgs`, `task_lib`, `task_server`), Dron (replica
`mission_msgs`, `task_manager_lib`, `task_manager`), GUI/`mission_flow`/
`system_architecture`/launch, tests, builds por grupos, integracion con puerta
manual y cierre documental.

Reanudacion de ejecucion tras compactacion: ya se creo el contrato canonico
`servidor/mission_msgs` con geometria, registro, estado de tareas, plan,
evento de seguridad, servicio `RegisterDrone` y action futura
`ExecuteTrajectory`; se copio mecanicamente la replica exacta en
`dron/mission_msgs`. Tambien se creo `servidor/task_lib` con parser/validacion
del YAML, revision determinista, `hard_flight_volume`, niveles y las cuatro
mitades solapadas, junto con tests unitarios iniciales. Todavia no se ha
compilado ni probado este codigo. Siguiente accion exacta: completar
`task_server`, registrar el hito de servidor y continuar con los componentes
del dron, GUI y grafos antes de los builds.

Hito ROS del bloque 3 implementado, aun sin build: `task_server` carga y
publica geometria real sin asignaciones, snapshots transient-local de registro
y lifecycle vacio, servicio idempotente y telemetria web opcional;
`task_manager` genera el handshake versionado de cada dron y reintenta el
registro sin consumir GT. `task_manager_lib` y el registro del servidor tienen
unitarios de perfiles validos/incompatibles. `BaseSubRoi`/`TaskState` ya
reservan ownership base/rama y los tests geometricos cubren areas, solape,
resto y multiplo exacto. Siguiente accion exacta: integrar geometria en la GUI,
crear `mission_flow`, ampliar `system_architecture` y conectar el launch.

Actualizacion documental solicitada completada: `subfase_6A.md` fija
`mission_flow`, `system_architecture`, histeresis vectorial y prueba con GT solo
para control; `subfase_6B.md` fija las cuatro mitades exactas, snapshot de
geometria y `MissionRegionLayer`; `subfase_6C.md` fija transporte ROS,
handshake/capabilities y limites de ejecucion; `subfase_7I.md` enlaza esa
seleccion con tareas reales futuras sin declarar 7I conseguida. `git diff
--check` correcto. No se modifico codigo/configuracion ni se ejecuto build o
simulacion. La autorizacion funcional sigue `PENDIENTE`.

Auditoria ZIP: las subfases actuales no son una transcripcion fiel. Mezclan
decisiones posteriores (eliminacion de `flight_bounds`, derivacion de
`hard_flight_volume` y workers) con el plan antiguo de 4/8 puntos, A-B-C, ownership en
`orbslam3_server` y contratos en `orbslam3_msgs`. El ZIP reestructurado fija
6A-6O, paquetes independientes `task_server/task_lib`,
`task_manager/task_manager_lib`, interfaces `mission_msgs`, elimina
`tasks_per_level` y define cuatro subROIs solapadas por nivel. Ademas, en el ZIP
6D es mapa voxel y 6F es base de `task_manager`; el pipeline actual desplaza
esas responsabilidades y las expande artificialmente hasta 6T. El complemento
post-ZIP prevalece ante contradicciones y elimina tambien `flight_bounds`.

Siguiente accion exacta: explicar el diagnostico al usuario y, si lo autoriza,
reescribir antes de cualquier bloque `pipeline_fase_6`, su resumen y las
subfases 6A-6O desde ambos ZIP; retirar/archivar 6P-6T solo tras acordar como
preservar sus ideas validas sin mantener una secuencia falsa.

Autorizacion documental concedida por `Haz eso`. Reconciliados fisicamente los
19 archivos del ZIP reestructurado y los 18 del complemento. Mapa definitivo:
6A arquitectura/config; 6B subROIs/ownership; 6C `mission_msgs`/registro;
6D voxel reversible; 6E gestor/asignador; 6F `task_manager`; 6G D* Lite; 6H
frontiers; 6I trayectoria reproducible; 6J reservas; 6K replanning; 6L riesgo
visual; 6M yaw/pitch; 6N especiales; 6O integracion. El contenido valido de
los actuales 6P-6T se absorbe en 6J/6K/6N/6O antes de retirarlos. No se modifica
codigo ni se ejecutan builds/simulaciones. Siguiente accion exacta: reescribir
resumen y pipeline maestro de F6 con esta secuencia.

Resumen y pipeline F6 reescritos; contratos 6A-6C sustituidos. Ya fijan paquetes
separados, YAML sin `flight_bounds/tasks_per_level`, cuatro subROIs regionales,
ownership 3D y `mission_msgs`/handshake. Siguiente accion exacta: sustituir
6D-6F (voxel, gestor/asignador y base `task_manager`).

Contratos 6D-6O sustituidos y secuencia documental 6A-6O ya reconstruida desde
los dos ZIP, con precedencia del complemento post-ZIP. No se ha modificado
codigo ni ejecutado build o simulacion. Siguiente accion exacta: retirar los
archivos obsoletos 6P-6T, corregir referencias vigentes y validar la coherencia
del conjunto documental.

Archivos 6P-6T retirados tras absorber sus conceptos validos en 6J/6K/6N/6O.
Contexto vigente, pipeline maestro, ADR y dependencias futuras de Fases 7/8 ya
usan la arquitectura `task_server`/`task_manager`/`mission_msgs` y el volumen
duro derivado sin `flight_bounds`. Siguiente accion exacta: ejecutar controles
de referencias, estructura Markdown y diff; corregir solo defectos documentales
mecanicos y cerrar la auditoria.

Cierre documental: `CONSEGUIDO`. Existen exactamente 15 contratos 6A-6O, todos
en estado `sin hacer` y con secciones de dependencia, objetivo, pruebas y
criterio de exito. No quedan referencias activas a 6P-6T, placeholders ni
propietarios legacy en los pipelines F6/F7/F8; fences equilibrados y
`git diff --check` correcto. No se modifico codigo ni se ejecuto build,
simulacion, Gazebo o GUI. Siguiente accion exacta: presentar el resultado y,
en una peticion posterior, proponer de nuevo los bloques funcionales a partir
de 6A-6O y las dependencias pendientes de Fase 7.

## Preparacion bloque 2 de Fase 7 (`7E`, `7F`, `7H`)

Peticion vigente: ejecutar completo el segundo bloque funcional tras cerrar su
preparacion.

Preparacion: `CERRADA`. Acuerdo cerrado: `si`. Autorizacion funcional:
`CONSUMIDA`. Prueba acordada: unitarios/sinteticos, rendimiento y GUI+Gazebo
real sin RViz, con puerta manual para que el usuario pruebe toggles, score,
camara y picking antes de liberar el escenario. Dudas abiertas: ninguna.

Alcance propuesto: `7E` sparse/score, `7F` drones+KFs+fiduciales y `7H`
picking+inspector. `7G` se excluye porque requiere trayectoria vigente real de
Fase 6. Auditoria inicial: el bloque 1 ya dejo VBOs, toggles, gradiente/filtro,
geometrias, opacidad stale y picking CPU preliminar; falta convertirlos en
contratos robustos, logs agregados, tests y validacion real. El servidor aun
publica `score`, `rgb`, `drone_id` y `map_epoch`; la GUI ignora RGB y deriva
color de score. No se ha modificado codigo, launch ni configuracion.

Siguiente accion exacta: completar auditoria puntual de score, identidad,
fuentes de KFs/fiduciales y tests existentes antes de editar codigo.

Decisiones confirmadas por el usuario: retirar completamente el field `rgb`
temporal tras validar equivalencia; labels permanentes solo para drones y
fiduciales, KFs bajo seleccion; picking por distancia en pantalla y desempate
por profundidad; pausa manual final aceptada. La pausa se implementara con un
`wait_for_bool` auxiliar de test y se liberara solo cuando el usuario confirme,
evitando matar la simulacion o cerrar la GUI a mitad de revision. La prueba
real usara una ruta dirigida corta.

Plan autorizado: (1) auditar contratos reales de score/identidad y fuentes
7F; (2) formalizar layers, logs, labels y seleccion estable/eficiente; (3)
validar que GUI deriva color de score y retirar `rgb` temporal del servidor;
(4) tests sinteticos y rendimiento; (5) builds/CTest; (6) ruta corta de dos
drones con GUI+Gazebo, sin RViz, detenida en puerta manual hasta confirmacion
del usuario; (7) analizar y documentar. Archivos criticos:
`servidor/multidron_gui_lib/`, `servidor/orbslam3_server/src/global_map_server.cpp`,
tests/docs de ambos paquetes y YAML auxiliar de prueba. Paquetes previstos:
`multidron_gui_lib`, `multidron_gui`, `orbslam3_server`, `simulacion_dron`.

Bloque de implementacion inicial completado: politica pura de score/picking y
tests (incluido 100k candidatos), slider+valor sincronizados, labels D/F,
seleccion por identidad estable y limpieza al desaparecer, logs agregados
7E/7F/7H y field `local_mp_id_low/high`. El servidor retiro `ScoreRgb` y
`rgb`, conserva score y publica identidad completa en `point_step=36`; contrato
pytest dedicado anadido. No se ha compilado. Siguiente accion exacta: ejecutar
formato/contratos estaticos, corregir errores mecanicos y despues compilar
`multidron_gui_lib`.

Contrato sparse: 1/1; formato y diff correctos tras una correccion mecanica de
sangria. Build `multidron_gui_lib`: codigo 0 en 26.7 s. Siguiente accion exacta:
ejecutar CTest completo con entorno Servidor cargado y corregir solo fallos
mecanicos antes de compilar el servidor.

CTest `multidron_gui_lib`: 9/9 passed. Build `orbslam3_server`: codigo 0 en
39.6 s. Siguiente accion exacta: ejecutar CTest completo del servidor con el
entorno correcto; despues compilar ejecutable GUI y simulacion.

CTest `orbslam3_server`: 13/13 passed. Ruta auxiliar 378 creada: dos drones,
paso por fiduciales sur/norte, mapa lateral y puerta transient-local
`/codex/f7_manual_review_done` con timeout 1800 s. Build `multidron_gui`: codigo
0 en 2.05 s. Siguiente accion exacta: rebuild rapido de la biblioteca tras
activar score/filtro en el smoke sintetico, abrirlo, inspeccionar captura y
borrarla antes de la simulacion real.

Rebuild smoke intento 1 no iniciado: codigo 1 al no poder crear
`/home/chenfu/Gazebo/log/servidor/build_*` por sandbox read-only; colcon no
compilo ningun fuente. Siguiente accion exacta: repetir el mismo helper con
escalado de filesystem, sin cambios funcionales.

Rebuild smoke repetido con escalado: codigo 0 en 3.08 s. Smoke sintetico con
entorno Snap saneado: Qt/OpenGL AMD RENOIR Mesa 4.6, color por score y filtro
activos con umbral 0.35; captura 1500x900 inspeccionada, puntos verdes/amarillos
visibles, puntos rojos bajo umbral ausentes y toolbar/layout sin solapes. La
captura es transitoria y se elimina antes de la prueba real. Siguiente accion
exacta: recompilar `multidron_gui` y `simulacion_dron` por separado, ejecutar
verificaciones finales automaticas y preparar la prueba 378.

Captura temporal eliminada. Builds finales: `multidron_gui` codigo 0 en 0.17 s
y `simulacion_dron` codigo 0 en 0.56 s. Verificaciones: `git diff --check`
correcto, contrato sparse 1/1 y CTest `multidron_gui_lib` 9/9. Un primer CTest
no ejecuto casos porque el sandbox impidio escribir `LastTest.log`; la
repeticion escalada es la ejecucion valida. Prueba 378 preparada: GUI F7 y
Gazebo activos, RViz y GUI legacy inactivos, dos drones sobre ruta corta y
puerta `/codex/f7_manual_review_done`. Siguiente accion exacta: iniciar 378,
confirmar mediante log reducido que alcanza la puerta y pedir la revision
manual del usuario sin liberar el escenario.

Prueba 378 abortada por indicacion del usuario antes de la puerta manual: la
ruta lateral/norte no era adecuada. El proceso quedo completamente cerrado con
exit 130 por SIGINT solicitado; este intento se conserva como abortado, no como
fallo funcional. Correccion acordada: `tray_prueba_378R.yaml`, un unico goal
simultaneo a `(0, -10, Z)` con `yaw_deg=90.0`, conservando Z=1.0 para dron 1 y
Z=1.3 para dron 2, espera de integracion y la misma puerta manual. Siguiente
accion exacta: validar el YAML auxiliar e iniciar 378R con el mismo launch.

Prueba 378R activa y detenida correctamente en la puerta manual. Ambos goals
simultaneos terminaron; la GUI recibe sparse real (pico observado 5544 puntos,
score 0.0206..1.0, identidad `drone_epoch_local_mp`) y 53 KFs antes de la
revision. Gazebo y GUI permanecen abiertos; RViz sigue desactivado. Siguiente
accion exacta: esperar la revision visual/interactiva del usuario y no publicar
`/codex/f7_manual_review_done` hasta su confirmacion.

Revision manual confirmada por el usuario. Evidencia 378R: picking real de
MapPoints, KFs y fiduciales y limpieza al clicar vacio; la GUI fue cerrada por
el usuario durante la revision. La puerta expiro despues con
`SCENARIO-RUNNER-READY-TIMEOUT`, por lo que 378R conserva exit 1 de harness sin
convertirlo en fallo funcional. Se prepara 378RR, misma ruta y datos, para
publicar la puerta inmediatamente y verificar cierre automatico limpio sin
repetir la inspeccion manual. Siguiente accion exacta: validar e iniciar 378RR.

Cierre 378RR: runner codigo 0, `SIM-DONE success=true`, 64 muestras de
recursos, minimo 4305.3 MiB, PSI memoria 0, guarda inactiva, RViz RSS/PSS 0 y
`GUI-SHUTDOWN` limpio. Documentacion de paquetes, contratos 7E/7F/7H,
historiales, resumen F7, contexto minimo y ultima sesion sincronizados. Bloque
2: `CONSEGUIDO`. Preparacion `CERRADA`, acuerdo `si`, autorizacion `CONSUMIDA`,
dudas abiertas ninguna. No hay GUI, Gazebo ni simulacion activa.

## Preparacion bloque 1 de Fase 7 (`7A`-`7D`)

Peticion vigente: ejecutar como una sola unidad funcional el bloque 1 acordado
tras completar su preparacion conversada.

Preparacion: `CERRADA`. Acuerdo cerrado: `si`. Autorizacion funcional:
`CONCEDIDA`. Prueba acordada: tests unitarios/Qt/ROS con datos sinteticos,
arranque GUI sin publishers, backend/Gazebo sin GUI y simulacion corta de dos
drones con GUI propia + Gazebo, sin RViz2, incluyendo cierre y reapertura de la
GUI. Dudas abiertas: ninguna.

Alcance preparatorio: `7A`-`7D` como un unico bloque. No se ha modificado codigo,
launch, YAML ni configuracion; no se ha compilado ni simulado. Auditoria real:
`multidron_gui_lib` y `multidron_gui` ya existen y contienen una base Qt5/Qt6,
executor ROS separado, snapshots, layout y renderer OpenGL con VBOs. Pendientes
principales: inventario contractual/documentacion de paquete, shutdown ROS->Qt,
rechazo de updates obsoletos, timeout stale, formulario accesible, arquitectura
`RenderLayer`, pruebas de bridge/layout/render e integracion reproducible con
Gazebo. Las capas adelantadas de `7E`-`7H` se preservaran, pero no se declararan
validadas dentro del bloque 1.

Decisiones confirmadas: timeout stale configurable con default `1.0 s`;
formulario `drone/tipo/x/y/z/yaw` visible desde este bloque y envio deshabilitado
hasta Fase 6; `multi_dron.launch.py` con `launch_multidron_gui=true` y
`launch_rviz=false` por defecto, conservando modo headless; prueba integrada
corta con dos drones; migracion estructural de todas las capas existentes a
`RenderLayer` sin adelantar el cierre funcional de `7E`-`7H`. Se permiten datos
sinteticos de prueba para poblar la GUI: los generadores deterministas pueden
quedar bajo `test/`, pero no se crean publishers runtime ficticios ni se
conservan mapas/datos temporales tras la prueba.

Checkpoint: contratos `7A`-`7D`, pipeline F7, codigo de ambos paquetes y patron
web de Fase 3 revisados fisicamente. El grafo web incremental de Fase 6 comenzara
en el bloque 3 y crecera en los bloques posteriores; no forma parte del bloque 1.
Plan autorizado: (1) lifecycle/shutdown y modelo stale/revisiones; (2) layout y
formulario visible deshabilitado; (3) `RenderLayer` y escena sintetica de test;
(4) integracion en launch y `system_architecture`; (5) tests; (6) builds
pequenos; (7) pruebas standalone y GUI+Gazebo, reduccion de logs y cierre
documental. Archivos criticos: `servidor/multidron_gui_lib/`,
`servidor/multidron_gui/`, `simulacion/simulacion_dron/launch/multi_dron.launch.py`
y topologia/tests de `system_architecture`. Paquetes previstos:
`multidron_gui_lib`, `multidron_gui`, `simulacion_dron`.

Bloque de cambios base completado: lifecycle Qt/ROS bidireccional, rechazo de
estados de navegacion obsoletos, timeout stale configurable de `1.0 s` que
conserva la ultima pose, formulario F6 visible y deshabilitado fuera del scroll,
y abstraccion `RenderLayer` aplicada a las seis capas existentes. El launch
global declara `launch_multidron_gui=true`, `launch_rviz=false` y conserva modo
headless; la GUI recibe numero/namespaces de drones, fiduciales y timeout. Se
anadieron tests de modelo, layer, layout con 20 tarjetas y un smoke sintetico
solo bajo `BUILD_TESTING`, sin publishers runtime. No se ha compilado aun.

Siguiente accion exacta: sincronizar `system_architecture`, ejecutar contratos
estaticos y corregir cualquier error mecanico antes de los builds pequenos.

Topologia sincronizada: `multidron_gui` y `multidron_gui_lib` constan en
politica, grafo, layout y metadata, con dependencia build y despliegue desde
`simulacion_dron`; no se inventaron edges runtime sin telemetria real. Sintaxis
de ambos launch y `git diff --check` correctos. Primer intento pytest no ejecuto
tests por una ruta GUI inexistente; repeticion dirigida del contrato de
arquitectura: `11 passed`. Siguiente accion exacta: compilar primero
`multidron_gui_lib`, corregir el primer error real si aparece y despues compilar
`multidron_gui` y `simulacion_dron` por separado.

Build `multidron_gui_lib` intento 1: codigo 0, paquete terminado en 34.0 s.
Unico warning propio: referencia `QString` ligada a temporales en el bucle de
campos del formulario. Correccion mecanica aplicada usando copia explicita;
siguiente accion exacta: recompilar `multidron_gui_lib` y ejecutar sus CTests.

Rebuild `multidron_gui_lib`: codigo 0 en 7.93 s y sin warning propio. CTest
intento 1: 3/8; cuatro GTests no arrancaron por `LD_LIBRARY_PATH` incompleto al
invocar CTest sin source de `install/servidor`, y `uncrustify` marco formato en
tres archivos; cppcheck, lint_cmake y xmllint pasaron. Reformat mecanico exacto
aplicado. Siguiente accion exacta: rebuild y repetir CTest con el entorno ROS y
Servidor cargado, sin interpretar los cuatro errores de loader como fallos
funcionales.

Rebuild post-formato: codigo 0 en 10.7 s. CTest intento 2 con entorno correcto:
`8/8 passed`, incluidos cuatro GTests (modelo stale/reordenacion, fiduciales,
`RenderLayer`, layout de 20 drones) y cuatro linters. Siguiente accion exacta:
compilar `multidron_gui` y despues `simulacion_dron`, cada uno en su invocacion.

Build `multidron_gui`: codigo 0 en 6.50 s. Build `simulacion_dron`: codigo 0 en
8.88 s. Los tres paquetes del bloque compilan. Siguiente accion exacta: ejecutar
smoke GUI sintetico con captura temporal, inspeccionarla y borrarla; despues
probar GUI standalone sin publishers.

Smoke sintetico intento 1 invalido por contaminacion Snap/VS Code
(`libpthread.so.0` de core20); no abrio Qt. Repeticion con entorno saneado:
codigo 0, OpenGL AMD RENOIR/Mesa 4.6, captura 1500x900 inspeccionada con MP,
KFs, fiducial, 20 tarjetas y formulario fijo sin solapes; captura temporal
borrada. Standalone sin publishers: subscripciones y OpenGL correctos; SIGINT
produjo `[GUI-SHUTDOWN] qt_event_loop_finished=true`. Pruebas 376/377
preparadas con dos drones y la misma trayectoria GT corta. 376: Gazebo/backend,
`launch_multidron_gui=false`, interfaces legacy apagadas, timeout 180 s y
post-wait 5 s. Siguiente accion exacta: ejecutar 376 y registrar el resultado
bruto antes de reducir su log.

Prueba 376 terminada: runner codigo 1 y `SIM-EXIT-CODE=1`; duracion 44 s,
37 muestras, minimo 4540.7 MiB, PSI memoria 0 y guarda inactiva. Log completo
conservado en `codex/archivos_auxiliares/logs/prueba_376.log` sin leer. No se
asume causa funcional. Siguiente accion exacta: reducir por runner, goals,
launch GUI/RViz, Gazebo/backend y errores; diagnosticar solo el reducido antes
de decidir una correccion o repeticion.

Diagnostico reducido 376: Gazebo, backend y ambos drones arrancaron; RViz RSS
0.0 MiB y no hubo GUI F7. El runner completo pasos 1-2 y rechazo antes de enviar
goals `mode: parallel`: modos validos `sequential, simultaneous`. Es un error
mecanico exclusivo del YAML auxiliar, no del bloque funcional. 376 queda
intacta como intento fallido; `tray_prueba_376R.yaml` usa `simultaneous` y 377
se corrigio antes de su primera ejecucion. Prueba 376R: mismo launch headless,
timeout 180 s y post-wait 5 s. Siguiente accion exacta: ejecutar 376R y registrar
su resultado bruto antes de reducir.

Prueba 376R terminada: runner codigo 0, `SIM-DONE success=true`, exit 0,
duracion 61 s, 50 muestras, minimo 4528.9 MiB, PSI memoria maximo 0.36 y guarda
inactiva. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_376R.log` sin leer. Siguiente accion
exacta: reducir por pasos/goals, backend, ausencia de GUI/RViz y errores graves;
cerrar la conclusion headless antes de lanzar 377.

Analisis 376R: pasos 1-4 completos, dos goals enviados en modo `simultaneous`,
dos inicios atomicos `success=true source=4`, backend finalizo limpio,
`SIM-DONE success=true` y RViz RSS/PSS 0.0 MiB. Trazas F5 de arranque
`DYNAMIC_BASE_NOT_READY` son ajenas a F7 y no afectaron control GT. Conclusion
headless: `CONSEGUIDA`. El smoke previo probo que Qt falla bajo variables Snap;
se aplica al Node `multidron_gui` el mismo entorno saneado ya usado por RViz.
Siguiente accion exacta: rebuild pequeno de `simulacion_dron`, despues ejecutar
377 con GUI F7 y Gazebo, sin RViz ni GUI legacy.

Rebuild `simulacion_dron` post-saneado: codigo 0 en 0.56 s. Prueba 377:
`tray_prueba_377.yaml`, `multi_dron.launch.py`, Gazebo GUI y GUI F7 activas,
`launch_mission_gui=false`, `launch_rviz=false`, dos drones, timeout 180 s,
post-wait 8 s y monitor de recursos. Criterio: runner completo, marcadores
`GUI-BOOT/ROS-READY/OPENGL`, recepcion de datos, ausencia de proceso RViz,
shutdown limpio y sin error fatal GUI. Siguiente accion exacta: ejecutar 377 y
registrar el resultado bruto antes de reducir.

Prueba 377 terminada: runner codigo 0, `SIM-DONE success=true`, exit 0,
duracion 63 s, 49 muestras, minimo 4383.2 MiB, PSI memoria 0 y guarda inactiva;
RViz RSS/PSS 0.0 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_377.log` sin leer. Siguiente accion
exacta: reducir por marcadores GUI precisos, pasos/goals, shutdown y errores de
proceso; analizar solo el reducido y despues validar cierre/reapertura.

Analisis 377: GUI F7 arranco como proceso independiente con Qt 5.15.3 y OpenGL
AMD/Mesa 4.6; suscribio dos `NavigationState`, cargo tres objetos fiduciales y
quedo lista para sparse/KFs. El runner completo 4/4 pasos y envio ambos goals;
`SIM-DONE success=true`. En cleanup, GUI emitio `GUI-SHUTDOWN` y termino limpia;
Gazebo/backend tambien. El spawner fiducial reporto contexto invalido solo tras
el shutdown global, ajeno a F7. Conclusion integrada provisional:
`CONSEGUIDA`; RViz permanecio ausente. Siguiente accion exacta: reabrir GUI
standalone y cerrarla otra vez por SIGINT para completar el ciclo de reapertura,
despues ejecutar verificaciones finales y documentar 7A-7D.

Reapertura standalone completada: Qt/OpenGL arranco de nuevo y SIGINT produjo
shutdown limpio. Documentacion de paquete creada para `multidron_gui` y
`multidron_gui_lib`; resumen de `simulacion_dron`, estados 7A-7D e historiales
cronologicos sincronizados, incluido 376 fallido y 376R corregido. Siguiente
accion exacta: ejecutar verificaciones finales de contratos/diff, revisar el
estado agregado de Fase 7 y cerrar `00_CONTEXTO_COMPACTACION.md` sin trabajo
activo.

Cierre: verificaciones finales correctas (`git diff --check`, sintaxis launch,
arquitectura 11/11); estados agregados, contexto minimo y ultima sesion
sincronizados. Bloque 1 de Fase 7 (`7A`-`7D`): `CONSEGUIDO`. Preparacion:
`CERRADA`; acuerdo: `si`; autorizacion funcional: `CONSUMIDA`; dudas abiertas:
ninguna. No hay build, GUI ni simulacion activa. Siguiente punto de entrada:
preparar el bloque 2 de Fase 7 y determinar el tramo de Fase 6 que lo desbloquea.

## Cierre Fase 1J

Objetivo: restaurar el vuelo GT previo al joint, validar topologia fija y movil,
y ejecutar llegada GT -> autoridad ORB/fallback -> barrido `+70/-70/+70 deg`.

Preparacion: `CERRADA`. Acuerdo cerrado: `si`. Autorizacion funcional:
`CONCEDIDA`. Prueba acordada: completada. Dudas abiertas: ninguna.

Cambios vigentes:

- `camera_pitch_enabled=false` crea `stereo_pitch_joint` fixed y no carga
  `plugin_camera_pitch`; `true` conserva revolute y servo;
- cada camara y `stereo_rig` usan masa positiva `1e-5 kg`, total `3e-5 kg`;
- `scenario_runner_node` admite `wait_for_navigation_pose` con tolerancias,
  hold y timeout sobre `NavigationState`;
- `tray_prueba_367.yaml` exige gate real tras el primer goal GT;
- los goals conservan `navigation_source: None|GT|ORB`, donde `None` hereda
  `phase5_navigation_source` y ORB puede usar el fallback configurado.

Diagnostico causal: 370 fallo incluso con joint fijo, pose final
`(0.002421,-6.979690,0.813943)`, error `3.026036 m` y torque `~0.0392 Nm`.
Los `0.04 kg` anteriores a `0.10 m` producian precisamente
`0.04*9.81*0.10=0.03924 Nm`, no incluidos en el modelo de control. No se
cambiaron ganancias, mixer ni ley geometrica.

Builds: el primer build del gate fallo por include Eigen ausente; correccion
mecanica aplicada. Builds posteriores de `simulacion_dron`: codigo 0.

Pruebas:

- 370: `NO CONSEGUIDA`, demuestra la regresion de masa;
- 370R: `INVALIDA DE INFRAESTRUCTURA`, ruta YAML relativa, sin goal;
- 370R2: `CONSEGUIDA`, joint fixed, error `0.015278 m`, yaw `0.000131 deg`,
  torque `~0.000025 Nm`;
- 371: `CONSEGUIDA`, joint/servo activos neutral, error `0.016051 m`, torque
  `~0.000029 Nm`;
- 372: `CONSEGUIDA PARA 1J`, gate GT `0.015747 m`; segundo goal inicia con ORB
  real `source=1`, anchored; barrido `69.007/-69.013/69.035 deg`; error de
  posicion maximo `0.077636 m`, sin NaN ni inestabilidad.

En 372 ORB pierde tracking durante el primer cambio grande de pitch y conmuta a
`GT_FALLBACK`, segun la politica acordada. Esto queda como entrada visual para
Fase 5/6 y no como fallo fisico de 1J.

Conclusion: Fase 1J `CONSEGUIDA`. Documentacion de paquete, historial, resumen,
indice, contexto minimo y ultima sesion sincronizados. No hay simulacion activa.

## Preparacion regresion Fases 3/4 post-1J

Peticion: ejecutar la prueba tipica larga de rodeo con GT durante toda
la trayectoria y pitch neutral para comprobar que los cambios 1J no rompieron
Fases 3/4. Preparacion: `CERRADA`. Acuerdo cerrado: `si`. Autorizacion
funcional: `CONCEDIDA`. Prueba acordada: variante GT estricta del rodeo largo.
Dudas abiertas: ninguna.

Hallazgo: el YAML canonico contiene llamadas legacy `activate_orb_shadow` y
esperas `orb_authority_confirmed`, por lo que el launch global GT no garantiza
por si solo autoridad GT continua. Propuesta: no modificar el canonico; crear
una variante auxiliar con la misma geometria/tiempos, `navigation_source: GT`
en todos los goals y sin llamadas/esperas de autoridad ORB. ORB permanece en
sombra para KFs, mapa y fiduciales; GT solo gobierna navegacion. Launch con
`camera_pitch_enabled=true`, sin comandos pitch. Validar runner completo,
fuente 4 en todos los goals, tracking/KFs/mapa sparse/loops de F3, detecciones,
anchors/tareas/revisits de fiduciales 1 y 2 de F4, y ausencia de no finitos o
errores graves. Autorizacion explicita recibida. No se modifica codigo ni el
YAML canonico y no procede build: se crea solo una variante auxiliar mecanica,
se valida su contrato y se ejecuta con `phase5_navigation_source=gt`, pitch
activo neutral y recording deshabilitado. Variante
`tray_prueba_373.yaml` creada: conserva 22/22 goals, todos declaran
`navigation_source: GT`, no contiene `activate_orb_shadow`,
`orb_authority_confirmed`, `set_trajectory_active` ni pasos pitch; YAML canonico
intacto. No procede build. Siguiente accion exacta: ejecutar prueba 373 con
timeout largo y conservar el log completo sin leer para su reduccion posterior.

Checkpoint prueba 373 terminada antes de analizar: scenario runner codigo 0,
`SIM-DONE success=true`, exit 0, duracion 537 s, 395 muestras, minimo 4170.0
MiB, memory PSI 0 y guard inactivo. Log completo conservado sin leer. Siguiente
accion exacta: reducir por runner/fuentes, tracking/KFs/mapa/loops F3,
fiduciales/anchors/tareas/revisits F4, pitch y errores graves; emitir conclusion
separada para cada fase antes de documentar.

Diagnostico 373: 22/22 goals `success=true` y 22/22 inicios atomicos
`source=4`; joint cargado neutral y cero `1J-PITCH-COMMAND`. F3 local produce
520 `KF-EVENT-CREATED` y 658 `PIPE0-WRAPPER-DELTA-PUB`; al final hay deltas con
~944 MapPoints en drone 1 y ~1393 en drone 2. El backend rechaza seis tareas de
loop con `[F3L-HARD-FAILURE] reason=loop_submap_interval_too_small
action=continue`; no hay crash ni no finitos. F4 visual procesa 517 KFs,
publica 77 batches y acepta 88 tags; ambos drones tienen KFs con tags validos
(34 y 43). No aparecen marcadores positivos suficientes de anchor/revisit/task
commit del backend para certificar F4 completa. Conclusion prueba 373:
`PARCIAL`. Demuestra que 1J no rompio trayectoria GT, mapa sparse local ni
deteccion fiducial, pero no permite afirmar perfeccion end-to-end de F3/F4.
Siguiente accion exacta: documentar la ejecucion y presentar el resultado sin
aplicar correcciones no acordadas.

Cierre documental 373: historial de pruebas tipicas, resumen y ultima sesion
sincronizados. Preparacion `CERRADA`, acuerdo `si`, autorizacion consumida,
dudas abiertas ninguna. No hay simulacion ni trabajo activo. Siguiente punto:
debatir los rechazos loop y una validacion backend F4 solo si el usuario lo
solicita.

Revision conversada de 373: el usuario aclara que desde `marker_id=368`, al
final del rodeo, debieron ejecutarse varias optimizaciones por loop para
corregir el cierre y no se hicieron. Los seis
`loop_submap_interval_too_small` dejan de interpretarse solo como rechazos
protectores inocuos y pasan a ser evidencia de una deuda concreta de 3Q.
Historial 373, resumen de pruebas tipicas, resumen/indice 3Q y ultima sesion
sincronizados. Decision: apuntar para revisar en 3Q; no investigar, modificar ni
repetir ahora. No hay trabajo activo.

## Preparacion 1K limpieza y cierre de Fase 1

Peticion: crear una subfase de limpieza de Fase 1, retirar codigo y artefactos
sin uso demostrado, introducir un flag global del simulador para silenciar su
telemetria, comprobar regresion y cerrar con commit y push. Preparacion:
`CERRADA`. Acuerdo cerrado: `si`. Autorizacion funcional: `CONCEDIDA`.
Prueba acordada: tests estaticos/unitarios, build de `dron_individual` y
`simulacion_dron`, simulacion corta GT con movimiento/yaw/pitch en
`debug_fase_1=false`, repeticion con `debug_fase_1=true`; rodeo largo solo si
aparece una regresion. Dudas abiertas: ninguna.

Auditoria inicial: no existe `debug_fase_1`; los logs relevantes estan
repartidos entre control de trayectoria/fuerzas y plugins de simulacion, y
`scenario_runner` mezcla diagnostico con marcadores de resultado. Se propone
preservar siempre errores y resultados de prueba. `src/vision/control_dron.cpp`
es experimental, pero aun se compila e instala; no aparece referenciado por
launches o configuracion en el primer barrido y solo se retirara tras verificar
su falta de uso. El resumen de Fase 1 sigue marcando 1J como pendiente y debe
sincronizarse como parte del cierre documental. No se ha modificado codigo,
launch ni configuracion y no hay simulacion activa.

Acuerdo: `debug_fase_1=false` por defecto silenciara telemetria informativa y
periodica de Fase 1, conservando advertencias importantes, errores y marcadores
de resultado; no gobernara logs F3/F4/F5. La limpieza se limitara a elementos
sin uso demostrado. El commit de cierre incluira cambios intencionales
pendientes de 1J, 1K y documentacion 373, excluyendo metricas, logs, caches y
artefactos generados; despues se enviara a `origin/main`. Plan: auditar
referencias e inventario de logs; crear contrato 1K; implementar limpieza y
flag; añadir pruebas de contrato; compilar; ejecutar las dos simulaciones;
reducir y analizar logs; documentar; revisar diff; commit y push. Archivos
criticos: CMake/launch/config de ambos paquetes, nodos de control y plugins F1,
pruebas y documentacion de Fase 1/paquetes. Siguiente accion exacta: completar
la auditoria de referencias y definir los cambios minimos antes de editar.

Bloque de implementacion 1K completado: retirados `control_dron`, el `clock`
duplicado de `dron_individual` y los prototipos no instalados ni referenciados
de `dron_individual/src/vision/`; no se ha tocado `mi_tfg` ni codigo vigente de
ORB/nube. `debug_fase_1=false` ya se declara en `debug.yaml`, se propaga por
`multi_dron.launch.py` y `generar_dron.launch.py`, fija nivel `warn/info` en
los cuatro nodos de vuelo y llega mediante generador/Xacro a motores, GT y
pitch. Los plugins conservan errores y warnings de seguridad y condicionan su
telemetria `INFO`. Contrato `subfase_1K.md` e historial inicial creados. Tests
rapidos de launch/config/arquitectura: 24 passed; `git diff --check` correcto.
Siguiente accion exacta: compilar `dron_individual` y `simulacion_dron` con la
herramienta del proyecto y registrar el resultado antes de diagnosticar.

Build 1K intento 1 no iniciado: codigo 2 porque la herramienta exige un unico
`--group` por invocacion; no compilo ningun paquete ni genero un diagnostico de
codigo. Correccion mecanica: ejecutar primero
`--group dron dron_individual` y despues `--group simulacion simulacion_dron`.
Siguiente accion exacta: build pequeno de `dron_individual`.

Build 1K `dron_individual`: codigo 0, 1/1 paquete terminado en 5.76 s. Log de
build gestionado por `build_selected_packages.sh`; unico aviso externo por una
ruta Drake inexistente en `CMAKE_PREFIX_PATH`, sin efecto en el paquete.
Siguiente accion exacta: compilar `simulacion_dron` con grupo `simulacion`.

Build 1K `simulacion_dron`: codigo 0, 1/1 paquete terminado en 52.9 s; mismo
aviso externo de Drake, sin fallos. Ambos builds acordados estan correctos.
Siguiente accion exacta: seleccionar una trayectoria auxiliar GT corta que
ejercite movimiento, yaw y pitch, preparar dos ejecuciones identicas variando
solo `debug_fase_1`, y registrar la primera antes de lanzarla.

Pruebas 374/375 preparadas: YAMLs equivalentes con llegada GT a
`(0,-10,1)`, yaw 90 grados, gate real y pitch `+30/-30/0`. Prueba 374 usara
`debug_fase_1=false`, GUI desactivadas, dos drones configurados pero solo el
primero comandado, `phase5_navigation_source=gt`
y timeout 240 s. Criterio: runner completo, goal y gate correctos, tres pasos
pitch correctos, ausencia de errores graves y ausencia de `INFO` F1 (en
especial `1J-PITCH-*` informativos). Siguiente accion exacta: ejecutar 374 y
registrar su resultado bruto antes de reducir el log.

Prueba 374 terminada: scenario code 0, `SIM-DONE success=true`, exit 0,
duracion 63 s, 50 muestras, minimo 4807.4 MiB, PSI memoria 0 y guard inactivo.
Log completo conservado en `codex/archivos_auxiliares/logs/prueba_374.log` sin
leer. Siguiente accion exacta: reducirlo por runner/goals/gate/pitch,
telemetria F1 y errores graves; analizar solo el reducido antes de preparar
375.

CTest `dron_individual` intento 1: 7/8 passed; GTests, cppcheck, flake8,
lint_cmake, pep257 y xmllint correctos. Solo `uncrustify` fallo por formato en
dos fragmentos pendientes de 1J (`navigation_goal_policy.hpp` y
`test_navigation_state_mux.cpp`). Correccion mecanica aplicada exactamente
segun el diff del linter. Siguiente accion exacta: repetir CTest de
`dron_individual` y despues ejecutar CTest de `simulacion_dron`.

CTest `dron_individual` intento 2: 8/8 passed, incluidos 2 GTests y 6 linters.
Siguiente accion exacta: ejecutar CTest completo de `simulacion_dron`.

CTest `simulacion_dron`: 9/12 passed. Pasan los contratos de pipeline,
configuracion global y arquitectura (incluido 1K) y los seis linters. Fallos
ajenos/preexistentes: `fiducial_contract` exige igualdad entre el canonico y
una copia auxiliar historica ya divergente; `pose_metrics_contract` carga
matplotlib de usuario que exige numpy>=1.23 frente a 1.21.5; y
`global_drone_pose_visualizer_contract` no encuentra `orbslam3_msgs` en el
entorno aislado de CTest. No se corrigen porque alterarian escenarios o entorno
fuera de 1K. Validacion dirigida: 24/24; CTest dron: 8/8; builds 2/2;
simulaciones 374/375: ambas correctas. Conclusion tecnica 1K: `CONSEGUIDA`.
Siguiente accion exacta: terminar documentacion, revisar diff y seleccionar
solo cambios intencionales para el commit conjunto, excluyendo artefactos.

Cierre documental 1K sincronizado: contrato 1J reconstruido, contrato/historial
1K, resumen/indice F1, contexto minimo, ultima sesion y docs de ambos paquetes
describen el estado vigente. `git diff --check` correcto. Se incluiran todos
los cambios tracked intencionales pendientes de 1J/1K/373, el plugin pitch,
historiales y YAMLs de evidencia 359-375 presentes; se excluyen metricas,
logs, `mplconfig`, caches y `pytest-of-chenfu`. Siguiente accion exacta: stage
selectivo, revisar lista/diff staged, crear commit de cierre y enviarlo a
`origin/main`.

Commit local de cierre creado inicialmente como `2648e32`, 94 archivos,
3185 inserciones y 13309 eliminaciones; artefactos acordados excluidos. Se
añade este checkpoint mediante amend mecanico antes del push. Siguiente accion
exacta: incorporar el checkpoint al commit, verificar HEAD y hacer push a
`origin/main`.

Cierre final: commit funcional/documental `28099b9` publicado correctamente;
`origin/main` avanzo de `82dd004` a `28099b9`. Subfase 1K `CONSEGUIDA`,
preparacion cerrada, autorizacion consumida y dudas abiertas ninguna. No hay
build ni simulacion activa. Solo permanecen sin seguimiento metricas, logs y
caches excluidos deliberadamente. Siguiente punto de entrada: preparar el
ciclo iterativo de Fases 6/7.

Analisis 374: 7/7 pasos completados; goal GT `success=true` en 12 s; gate
source 4 con error de posicion 0.005084 m y yaw 0.007652 grados; los tres pasos
pitch `+30/-30/0` terminaron. Cero coincidencias de `1J-PITCH`, mensajes de
suscripcion de motores, publicacion GT o generacion de trayectoria: la
telemetria F1 queda silenciada. No hubo `FATAL`, `NONFINITE` ni NaN. El wrapper
ORB emitio trazas F5 de validez clasificadas como `ERROR` durante su arranque;
son ajenas al flag F1 y no afectaron el control GT ni el resultado. Conclusion
374: `CONSEGUIDA`. Prueba 375: mismo YAML funcional, `debug_fase_1=true`, GUI
desactivadas, timeout 240 s; debe repetir resultado y mostrar telemetria F1.
Siguiente accion exacta: ejecutar 375 y registrar el resultado antes de reducir.

Prueba 375 terminada: scenario code 0, `SIM-DONE success=true`, exit 0,
duracion 61 s, 49 muestras, minimo 4811.8 MiB, PSI memoria 0 y guard inactivo.
Log completo conservado en `codex/archivos_auxiliares/logs/prueba_375.log` sin
leer. Siguiente accion exacta: reducir por los mismos patrones funcionales y
de telemetria F1, analizar el reducido y comparar con 374.
