# 00 - Contexto de compactacion

## Checkpoint vigente 2026-08-28 - cierre Git solicitado

```text
Estado: Fase 5H PARCIAL; prueba 256 NO CONSEGUIDA y diagnosticada.
Peticion activa: crear un commit del estado actual y subirlo a GitHub.
Rama/remoto: `main`, `origin` apunta a
`https://github.com/IvanCS-Chenfu/Mapeado-con-Drones.git`; HEAD inicial
`ff1e80a`, alineado con `origin/main` antes del commit.
Alcance del commit: todos los cambios actuales de codigo, configuracion, tests,
trayectorias y documentacion, incluido este checkpoint.
Exclusiones de staging: `codex/archivos_auxiliares/metricas/` (57 MB de
artefactos generados) y `codex/archivos_auxiliares/logs/prueba_252.index.md`
(indice generado de log). No se eliminan del workspace.
Validacion previa: `git diff --cached --check` correcto; 106 archivos
preparados. Builds y pruebas vigentes quedan documentados en los historiales.
Commit local vigente: `a67e450`, mensaje
`Integra poses de control ORB para la fase 5`.
Push intentado a `origin/main`, pero bloqueado por la revision de permisos: se
requiere confirmacion explicita del repositorio externo concreto
`https://github.com/IvanCS-Chenfu/Mapeado-con-Drones.git` y de la rama `main`.
Confirmacion explicita recibida del usuario para ese repositorio y rama.
Trabajo activo: incluir este checkpoint mediante amend y ejecutar el push
autorizado.
Siguiente accion exacta: `git commit --amend --no-edit` y despues
`git push origin main`.
```

## Checkpoint vigente 2026-08-28 - diagnostico prueba 256 tras compactacion

```text
Estado: Fase 5H PARCIAL; prueba 256 NO CONSEGUIDA.
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: SUSPENDIDA
Prueba acordada: prueba 256 ya ejecutada con la trayectoria tipica y
Gazebo/RViz2 visibles.
Dudas abiertas: determinar la causa exacta del colapso al entrar en ORB antes
de proponer otra correccion funcional.
Reanudacion tras compactacion: checkpoint fisico releido y reconciliado con la
ultima observacion del usuario: `En cuanto se ha cambiado a ORB ha colapsado el
dron.` No se modificara codigo ni se repetira la simulacion durante este
diagnostico.
Evidencia ya fijada: el cambio GT->ORB tuvo continuidad de pose exacta y el
handoff angular empezo con error cero; drone2 perdio tracking unos 4.6 s despues
de entrar en ORB. Inmediatamente antes, una innovacion angular local de
0.125261 rad fue aceptada y corregida gradualmente (`orientation_rejected=false`,
`limited=true`) tras aceptar reference KF 26; despues se aceptaron rapidamente
los KF 28 y 31 y llego la perdida de tracking.
Trabajo activo: no; diagnostico y documentacion de 256 cerrados.
Diagnostico cerrado: no hubo optimizacion global en el intervalo critico ni
salto del mux. Ambos handoffs empezaron con pose continua, error angular cero y
torque pequeno. El nuevo predictor acepto para drone2 una innovacion aislada de
`0.125261 rad` y publico un paso de `0.119002 rad`; despues acepto rapidamente
los reference KF 28 y 31 y perdio tracking `0.793 s` mas tarde. La politica de
corregir toda innovacion angular menor que `0.35 rad` de forma gradual sigue
siendo demasiado permisiva para control: suaviza el salto, pero aun inyecta una
actitud visual falsa antes de comprobar persistencia temporal. El goal absoluto
se transformo sin discontinuidad y sus diferencias de cota corresponden al
`W_T_O` inclinado vigente, no demuestran por si solas otro fallo inicial.
Resultado: prueba 256 `NO CONSEGUIDA`; Fase 5H agregada `PARCIAL`. Una futura
correccion necesita nuevo acuerdo: cuarentena/probation temporal separada para
innovaciones angulares moderadas, manteniendo prediccion breve sin invalidar
inmediatamente toda la pose ORB.
Documentacion sincronizada: entrada cronologica y resumen 5H, indice, resumen
de Fase 5, contexto minimo, estado actual, ultima sesion y docs vigentes de
`orbslam3_ros2`. `git diff --check` correcto. No se modifico codigo tras la
prueba ni se ejecuto otra simulacion.
Siguiente accion exacta: explicar al usuario el diagnostico y debatir la
probation angular temporal; no editar hasta cerrar acuerdo y recibir nueva
autorizacion.
```

## Checkpoint vigente 2026-08-28 - revision visual prueba 255

```text
Estado: Fase 5H PARCIAL; alternativas de correccion local ORB en debate.
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA
Prueba acordada: build y CTest focales, seguidos de una nueva repeticion de
`prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` con Gazebo/RViz2 visibles.
Dudas abiertas: ninguna.
Observacion del usuario: al pasar a ORB los drones funcionan bien brevemente;
despues se vuelven locos, se estrellan o pierden, posiblemente al optimizar KFs
o por muchos cambios de reference KF.
Reanudacion tras compactacion: checkpoint fisico releido y reconciliado con la
peticion mas reciente. Arquitectura confirmada: el optimizador modifica W y el
control consume O; por contrato W no mueve O.
Resultado: 11 optimizaciones globales; solo el primer episodio coincide con una
activa. Los otros empiezan 8.5 y 47 s despues de la ultima. El codigo confirma
que `ApplyAuthoritativeGlobalPose()` solo modifica W/revision, nunca O.
Diagnostico: los tres episodios empiezan tras pocos segundos en ORB y estan
precedidos por churn de reference KF, timeout de probation o innovacion angular
local de `0.081-0.214 rad`; despues llega la perdida real de tracking. Los 10
timeouts demuestran churn. Los outliers tambien aparecen con referencia activa,
por lo que el cambio de KF no es la unica causa local.
Documentacion sincronizada: entrada 255, resumen/indice 5H, contexto minimo,
estado, ultima sesion y doc vigente del wrapper. No se modifico codigo.
Trabajo activo: no.
Alternativas debatidas: un low-pass aplicado solo al cambio de KF es incompleto
y puede introducir retardo peligroso, sobre todo angular. Recomendacion:
estimador continuo en SE(3) con prediccion, gate duro y correccion gradual
limitada fisicamente para toda medida, pose/velocidad coherentes y fallback
temprano ante churn. Mejorar ademas la probation: hoy exige varios frames del
mismo candidato y puede expirar ante KFs distintos pero geometricamente
coherentes. Alternativa arquitectonica mas fuerte: integrar incrementos de
`local_t_camera` para O y reservar reference KF/Tcr para autoridad W, siempre
con gate ante BA/relocalizacion. IMU para actitud es la solucion realista a
largo plazo, pero amplia el alcance.
Decisiones nuevas del usuario: no aceptar fallback temprano como solucion; ORB
debe sostener el control y perderse muy poco. Se acepta el estimador SE(3)
continuo y probation geometrica independiente del ID. No se utilizara IMU. El
fallback GT se conserva solo como proteccion temporal ya acordada, no como
criterio de funcionamiento. La opcion `local_t_camera` necesita explicacion.
Valoracion: no conviene sustituir ahora Tcr por integracion primaria de
`local_t_camera`, porque BA/relocalizacion podria interpretarse como movimiento.
Puede usarse como contraste secundario para validar incrementos Tcr sin hacer
depender O de sus correcciones de mapa.
Acuerdo funcional propuesto tras la orden `aplica esas modificaciones`: estado
SE(3) continuo con gate duro y correccion residual gradual limitada para toda
medida; pose y velocidades derivadas del mismo estado; probation geometrica
independiente del ID de reference KF; `local_t_camera` solo como contraste
secundario. Alcance: wrapper `orbslam3_ros2`, parametros, marcadores y tests.
Exclusiones: GT/mux/control/ganancias, ORB-SLAM3 core, optimizador global, IMU y
YAML funcional. Riesgos: demasiado filtrado puede retrasar actitud y un gate
permisivo puede aceptar deriva; se conservaran limites configurables y fallback
solo ante perdida real/inconsistencia persistente. Prueba propuesta: build y
CTest focales, despues nueva repeticion de
`prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` con Gazebo/RViz2 visibles.
Criterios: completar 17/17 pasos y 22/22 goals, ORB sostenido casi toda la
trayectoria, sin movimientos alocados/choques, sin churn que fuerce fallback
habitual y con pose/velocidad coherentes; revision visual final del usuario.
Confirmacion final recibida del usuario. Plan: revisar diff y tests vigentes;
implementar estado SE(3) continuo/correccion residual y probation geometrica
multi-KF usando `local_t_camera` solo como contraste; actualizar parametros,
marcadores, tests y docs del wrapper; compilar `orbslam3`, ejecutar CTest focal,
preparar y lanzar la siguiente prueba libre con el YAML acordado; reducir logs,
diagnosticar y documentar. Trabajo activo: implementacion y validacion.
Siguiente accion exacta: inspeccionar el diff vigente y los simbolos/tests
minimos de `NavigationStateEstimator` y `OrbPosePredictor` antes de editar.
Archivos criticos revisados: `navigation-state-estimator.hpp/.cpp`,
`stereo-slam-node.cpp` y `test_navigation_state_estimator.cpp`. Diseno minimo:
probation conserva una cadena O geometrica aunque cambie el ID candidato;
`local_t_camera` solo enlaza/coteja el frame de cambio y Tcr gobierna dentro de
cada referencia. El predictor sigue una pose objetivo SE(3) con correccion
residual, velocidad y aceleracion acotadas; la pose publicada se integra desde
esas mismas velocidades. Innovaciones angulares moderadas se corrigen
gradualmente y solo las imposibles se rechazan. Siguiente accion exacta: editar
estos cuatro archivos, ampliar regresiones y sincronizar doc del wrapper antes
del build.
Bloque funcional aplicado: predictor SE(3) con `position_alpha=0.55`,
`orientation_alpha=0.70`, innovacion maxima `0.30 m/0.35 rad`, velocidades
`1.5 m/s` y `1.5 rad/s`, aceleraciones `4 m/s2` y `12 rad/s2`; la pose se
integra desde las velocidades publicadas. El gate acumula tres incrementos
plausibles aunque cambie el ID candidato, usa incremento local solo para
enlazar el cambio y conserva timeout de seis para inconsistencia persistente.
Regresiones ampliadas para correccion angular gradual, churn coherente
multi-KF y timeout inconsistente. Docs del wrapper y contrato 5H sincronizados;
estilo focal limpio y `git diff --check` correcto. No se tocaron exclusiones.
Preparacion build: compilar solo `orbslam3` del grupo Dron mediante
`./codex/herramientas/build_selected_packages.sh --group dron orbslam3`; el
log completo se conservara sin lectura directa. Siguiente accion exacta:
ejecutar build, registrar su salida y solo despues lanzar CTest focal.
Build SE(3)/multi-KF: exit 0, `orbslam3` 1/1 correcto en 50.1 s. Los avisos son
legacy de ORB-SLAM3/Eigen/cv_bridge; no hay error nuevo. Log completo conservado
en `codex/archivos_auxiliares/colcon_build.log` y no leido directamente.
Siguiente accion exacta: ejecutar CTest focal
`test_navigation_state_estimator` sobre el binario recien compilado; registrar
sus GTests antes de corregir o preparar simulacion.
CTest focal intento 1: 14/15 GTests correctos. Falla solo
`ReturningActiveReferenceCancelsCandidate`: un paso nominal exactamente en el
limite `0.10 m` queda ligeramente por encima por redondeo float y permanece
pending. La cadena multi-KF, timeout inconsistente y los cinco tests del
predictor pasan. Correccion mecanica: aplicar epsilon `1e-6` a comparaciones de
umbral del gate, rebuild `orbslam3` y repetir el mismo CTest; no cambia alcance
ni politica acordada.
Rebuild tras epsilon: exit 0, `orbslam3` 1/1 correcto en 6.83 s, sin error.
Log completo conservado y no leido. Siguiente accion exacta: repetir CTest
`test_navigation_state_estimator`; si pasa 15/15, validar estilo/diff y preparar
la siguiente prueba libre con el YAML tipico acordado.
Preparacion prueba 256: identificador libre, sin simulacion activa y
`git diff --check` correcto. YAML absoluto sin modificar:
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`.
Launch: `ros2 launch simulacion_dron multi_dron.launch.py
launch_gazebo_gui:=true launch_mission_gui:=false
debug_sparse_global_rviz:=true phase5_global_pose_rviz_enabled:=true
gt_fallback_enabled:=true debug_fase3_logs_terminal:=true`; startup 15 s,
timeout 1200 s, post 35 s, un reintento Gazebo y sin guarda de recursos.
Patrones de reduccion: `F5H-REFERENCE-GATE`, filtro/rechazo/predictor SE(3),
fuente/continuidad/handoff, tracking/reference KF, F3Q optimizacion,
scenario/SIM y errores graves. Criterio: 17/17 pasos, 22/22 goals, ORB
sostenido sin movimiento alocado ni fallback por churn normal. Siguiente
accion exacta: lanzar 256; al terminar o ser interrumpida, registrar salida y
log completo antes de reducirlo.
Prueba 256 finalizada con fallo: scenario exit 1 y runner/launch `[SIM-EXIT-CODE]
1`; cleanup ejecutado. Duracion aproximada 80 s, sin guarda de recursos.
Observacion visual inmediata del usuario: en cuanto la fuente cambio a ORB el
dron colapso. Resultado preliminar `NO CONSEGUIDA`; contradice el comportamiento
brevemente estable de 255 y apunta al nuevo predictor/handoff inicial, no a
churn acumulado. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_256.log` y no se leera directamente.
Autorizacion funcional: SUSPENDIDA para nuevas correcciones. Trabajo activo:
diagnostico y documentacion de 256. Siguiente accion exacta: reducir por
handoff/fuente/predictor/filtro/reference/tracking/control/scenario y analizar
el primer estado ORB antes del colapso.
CTest focal final: target 1/1 correcto, 15 GTests y cero fallos en 0.07 s.
Validacion estatica vigente: ambos builds correctos, estilo focal limpio y las
regresiones nuevas pasan. Siguiente accion exacta: comprobar identificador de
prueba libre, ausencia de simulacion activa y `git diff --check`; preparar copia
del YAML tipico y registrar launch/timeout/patrones antes de lanzar Gazebo/RViz2.
```

## Checkpoint vigente 2026-08-28 - prueba 253 fallida

```text
Estado: Fase 5H PARCIAL; validacion dinamica 253 interrumpida por divergencia.
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: SUSPENDIDA
Prueba acordada: vuelta tipica 253 con Gazebo/RViz2; ejecutada e interrumpida.
Dudas abiertas: ninguna para el diagnostico; la siguiente correccion requiere acuerdo.
Observacion del usuario: cuando RViz indico fuente GT, ambos drones se volvieron
locos. La mejora no queda validada.
Reanudacion tras compactacion: checkpoint fisico releido y reconciliado con la
peticion mas reciente. No se autorizan nuevos cambios funcionales hasta cerrar
un nuevo acuerdo con el usuario.
Resultado operativo 253: interrupcion solicitada, runner cerrado y cleanup
correcto; sesion exit 130 por Ctrl-C, launch termino con `[SIM-EXIT-CODE] 0`,
sin guarda de recursos activada. Duracion observada 103 s.
Diagnostico 253: no hubo transicion ORB->GT; ambos drones permanecieron en
`GT_FALLBACK` desde startup. El predictor uniforme filtro tambien la actitud GT.
Su limite de innovacion angular de 0.08 rad atraso la orientacion consumida por
el lazo de torque durante la inclinacion del dron y creo realimentacion
inestable. Se observaron innovaciones angulares de unos 3 rad, 1096/1612 medidas
limitadas en drone1, 863/1244 en drone2 e innovacion lineal maxima de 40.9 m en
drone1. Conclusion 253: `NO CONSEGUIDA`; 5H agregado `PARCIAL`.
Documentacion sincronizada: historial largo/resumen/indice 5H, resumen de Fase
5, estado actual, ultima sesion, contrato 5H y docs de `dron_individual`.
Decision nueva del usuario: GT debe permanecer completamente intacto; pose y
velocidad GT no pasan por filtro/predictor. El predictor se limita a ORB.
Revision de errores adicionales: ninguno independiente demostrado en 253. Las
perdidas/epochs ORB son posteriores a la inestabilidad fisica; los errores de
scenario, insercion URDF y exit 255 de Gazebo aparecen durante el Ctrl-C. ORB no
llego a gobernar, por lo que la estabilidad del predictor angular bajo ORB
queda sin evaluar, no confirmada como error.
Preparacion correccion post-253: EN_DEBATE. Acuerdo cerrado: no.
Autorizacion funcional: SUSPENDIDA. Trabajo activo: no.
Duda abierta: decidir si bajo ORB se predice tambien la orientacion o se evita
el mismo retardo angular separando actitud de traslacion/velocidad.
Siguiente accion exacta: explicar esta unica incertidumbre al usuario; no editar
codigo hasta cerrar el acuerdo y recibir autorizacion.

Correccion arquitectonica indicada por el usuario: predictor y filtros no deben
pertenecer a `dron_individual`. Revision confirmada: actualmente
`orbslam3_ros2/stereo` publica la medida ORB, remapeada a
`orbslam/navigation_state_orb`, y `dron_individual/navigation_state_mux` contiene
`PoseStatePredictor`, filtra/predice y publica el estado final. No cumple la
frontera solicitada y explica que GT quedara afectado. Arquitectura objetivo en
debate: estimacion/filtro/prediccion ORB dentro del nodo/capa estimadora de
`orbslam3_ros2`; el mux de `dron_individual` solo selecciona ORB ya corregido o
GT intacto y conserva la politica temporal Fase 5. No tocar ORB-SLAM3 core.
Preparacion: EN_DEBATE. Acuerdo cerrado: no. Autorizacion funcional: SUSPENDIDA.
Trabajo activo: no. Siguiente accion exacta: confirmar con el usuario esta
frontera y cerrar como tratar orientacion/velocidad ORB antes de editar.

Detalle mecanico confirmado: el mux actual solo recibe `sensor/GT/pose`; la
velocidad GT publicada se deriva en el predictor. Para cumplir GT intacto debe
suscribirse temporalmente tambien a `sensor/GT/vel` y reenviar pose/velocidad
exactas por `NavigationState`, sin filtrado. `orbslam3_ros2` ya contiene
`NavigationStateEstimator`; ahi debe vivir el predictor ORB y la publicacion
corregida a 50 Hz. Doc `dron_individual/control.md` sincronizada. Duda funcional
restante: bajo ORB, elegir entre mover tambien el filtro angular alpha-beta
actual o evitar su retardo usando orientacion ORB corregida/propagada sin ese
low-pass. Preparacion: EN_DEBATE; no hay autorizacion para editar codigo.

Acuerdo correccion post-253 cerrado: predictor/filtros salen de
`dron_individual` y pasan a `orbslam3_ros2/NavigationStateEstimator`; se filtran
traslacion y velocidades ORB, se propaga orientacion a 50 Hz con velocidad
angular estimada pero sin low-pass alpha-beta angular sobre la pose. GT se
reenvia desde `sensor/GT/pose` y `sensor/GT/vel` sin filtro ni prediccion. El mux
solo conserva seleccion, lock y alineacion rigida temporal Fase 5. Si no mejora,
se revisara en una nueva iteracion. No tocar ORB-SLAM3 core, ganancias,
optimizador ni YAML. Prueba: unitarios focales, builds seleccionados y repeticion
completa de la trayectoria tipica con Gazebo/RViz2. Criterios: tramo GT estable,
ORB publicado a 50 Hz sin escalones graves, fallback GT exacto, sin divergencia,
goals/pasos completos y revision visual del usuario. Riesgo aceptado: la
prediccion angular ORB puede requerir otro ajuste tras observarla.
Preparacion: CERRADA. Acuerdo cerrado: si. Autorizacion funcional: CONCEDIDA.
Dudas abiertas: ninguna. Trabajo activo: implementacion, build, tests,
simulacion, reduccion y documentacion. Plan: mover predictor/config/tests a
`orbslam3_ros2`; simplificar mux y anadir GT velocity exacta; sincronizar launch
y docs; compilar/testear ambos paquetes; ejecutar nueva prueba tipica posterior
a 253 y analizarla. Siguiente accion exacta: localizar CMake/tests y simbolos
minimos de publicacion/estado antes de editar.
Archivos criticos localizados: `orbslam3_ros2/navigation-state-estimator.hpp/.cpp`
para predictor ORB traslacional y velocidad angular sin low-pass de actitud;
`stereo-slam-node.hpp/.cpp` para cache, timer 50 Hz, parametros y coherencia O/W;
`test_navigation_state_estimator.cpp` para regresiones. En `dron_individual`,
`navigation_state_mux.hpp/.cpp` pierde todo predictor/timer y recibe
`geometry_msgs/TwistStamped` GT exacto; `test_navigation_state_mux.cpp` conserva
solo policy/alineacion y verifica rotacion de vectores entre frames. No hacen
falta cambios de mensajes, ORB-SLAM3 core, YAML ni ganancias. El wrapper usa
executor monohilo, por lo que callback de imagen y timer comparten estado sin
concurrencia interna adicional. Siguiente accion exacta: aplicar este bloque
minimo y sincronizar docs antes del primer build.
Bloque funcional post-253 aplicado: `OrbPosePredictor` vive en
`orbslam3_ros2`, filtra traslacion/velocidades, acepta orientacion ORB medida y
la propaga entre frames; `StereoSlamNode` publica a 50 Hz y mantiene O/W
coherentes. `dron_individual` ya no contiene predictor, parametros ni timer;
recibe GT pose+vel exactas y solo rota vectores al O continuo. Tests del
predictor se trasladaron al wrapper y el mux prueba la rotacion de velocidad.
Docs de ambos paquetes y contrato 5H sincronizados; `git diff --check` correcto.
Siguiente accion exacta: compilar aisladamente `orbslam3` y
`dron_individual`; registrar el resultado antes de corregir o ejecutar tests.
Build post-253 `orbslam3`: exit 0, 1/1 paquete correcto en 51.1 s. Los avisos
mostrados son legacy de ORB-SLAM3/Eigen/cv_bridge; no hay error nuevo. Log
completo conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido
directamente. Siguiente accion exacta: compilar aisladamente `dron_individual`;
registrar resultado antes de ejecutar CTest.
Build post-253 `dron_individual`: exit 0, 1/1 paquete correcto en 30.7 s, sin
errores. Log completo conservado y no leido directamente. Ambos paquetes
compilan con la nueva frontera. Siguiente accion exacta: ejecutar CTests focales
`test_navigation_state_estimator`, `test_navigation_state_mux` y
`test_navigation_goal_policy`; registrar resultados antes de estilo o
simulacion.
CTest focal intento 1: no ejecutado, ambos comandos exit 8 porque el sandbox
impidio escribir `Testing/Temporary/LastTest.log` bajo `build/dron`; no es fallo
de tests. Correccion operativa: repetir los mismos CTests con permiso de escritura
en build, sin cambiar codigo ni alcance. Siguiente accion exacta: ejecutar ambos
CTest focales con escalado y registrar sus resultados reales.
CTest focal intento 2: la solicitud de escalado fue rechazada automaticamente
por limite operativo de uso/aprobaciones antes de crear el proceso; tampoco es
un resultado de test. No se intentan workarounds ni se lanza simulacion sin esta
puerta. Builds siguen correctos. Autorizacion funcional permanece CONCEDIDA,
pero ejecucion operativa detenida a la espera de aprobacion explicita del usuario
para ejecutar fuera del sandbox:
`ctest --test-dir /home/chenfu/Gazebo/build/dron/orbslam3 -R
test_navigation_state_estimator --output-on-failure` y el CTest focal de
`dron_individual`. Trabajo activo: validacion pendiente. Siguiente accion exacta:
tras esa aprobacion, repetir CTests; si pasan, estilo y simulacion 254.
CTest focal intento 3 autorizado: `test_navigation_state_estimator` 1/1
correcto en 0.09 s. CTest `dron_individual`: `test_navigation_goal_policy` y
`test_navigation_state_mux` 2/2 correctos en 0.18 s. Tras los tests se detecta
una duplicacion mecanica de cadencia: callback ORB llamaba al publisher ademas
del timer, sumando hasta 20 Hz a los 50 Hz. Retirada esa llamada inmediata; el
timer queda como unico publicador del estado ORB corregido. No cambia filtro ni
politica acordados. Siguiente accion exacta: validar uncrustify focal, rebuild
`orbslam3` por esta ultima linea y repetir su GTest antes de preparar 254.
Estilo post-253: primer focal detecto divergencia en cinco archivos del wrapper;
`stereo-slam-node.*` es legacy y no se reformatea en bloque. Los tres archivos
acotados estimator/header/test se reformatearon mecanicamente. Focal final sobre
los seis archivos propios del estimador/mux: limpio; `git diff --check`
correcto. Siguiente accion exacta: rebuild `orbslam3` por formateo y retirada de
publicacion inmediata; despues repetir GTest estimator y, si pasa, registrar
simulacion 254.
Rebuild final `orbslam3`: exit 0, 1/1 correcto en 50.1 s; solo avisos legacy.
Log completo conservado y no leido. Siguiente accion exacta: repetir CTest
`test_navigation_state_estimator` sobre este binario final; si pasa, verificar
diff y registrar comando completo de simulacion 254 antes de lanzarla.
CTest estimator final: 1/1 correcto en 0.09 s. Validacion focal agregada: tres
contratos correctos, estilo propio limpio y builds finales correctos.
Siguiente accion exacta: comprobar que 254 esta libre y no hay simulacion
activa; registrar YAML, launch, timeout y patrones ORB/GT definitivos antes de
ejecutar con Gazebo/RViz2 visibles.
Reanudacion 2026-08-28: checkpoint fisico releido y reconciliado con la orden
mas reciente `Ejecutalos`. Prueba 254 libre, sin simulacion activa y
`git diff --check` limpio. Se ejecutara la trayectoria
`prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` con Gazebo y RViz2 visibles,
`gt_fallback_enabled:=true`, timeout de 1200 s y espera final de 35 s. Patrones
de reduccion acordados: marcadores F5H de predictor/filtro ORB, continuidad,
fuente de pose, handoff y diagnostico; tracking F5B, scenario runner y errores
de proceso. Trabajo activo: simulacion 254, reduccion, diagnostico y
documentacion. Siguiente accion exacta: lanzar la prueba 254; al terminar,
registrar el resultado operativo antes de reducir su log completo.
Prueba 254 interrumpida por peticion del usuario tras observar inestabilidad:
el inicio fue correcto y los drones se volvieron locos al girar cuando no
utilizaban GT, es decir, bajo ORB segun la observacion visual. Runner cerrado
limpiamente mediante Ctrl-C: sesion exit 130, launch `[SIM-EXIT-CODE] 0`, 346 s
de monitorizacion y guarda de recursos no activada. Resultado preliminar:
`NO CONSEGUIDA`; la nueva arquitectura evita alterar GT, pero la orientacion o
el estado angular ORB durante giros sigue sin ser apto para control. No se
autoriza todavia una nueva correccion funcional. Trabajo activo: reducir 254 y
diagnosticar la causa exacta antes de proponer cambios. Siguiente accion exacta:
generar y leer solo reducidos tematicos de fuente, predictor ORB, tracking,
handoff, giros y errores; sincronizar la conclusion viva de la prueba 254.
Diagnostico 254 cerrado: no hay salto de cambio de frame como causa primaria;
los handoffs `GT -> ORB` registran `translation_jump_m=0` y
`rotation_jump_rad=0`, y arrancan con `er=ew=0`. Bajo ORB, la implementacion
acepta cada orientacion medida completa, pero limita solo la velocidad angular
derivada. En los giros aparecen pasos de orientacion de hasta aproximadamente
0.279 rad en drone1 y 0.273 rad en drone2 por frame; la pose incorpora el salto
completo mientras `angular_velocity` queda acotada a 1.5 rad/s. Esa
incoherencia pose/velocidad excita el lazo angular, precede las perdidas de
tracking y explica la divergencia visual. Drone2 pierde tracking unos 7.6 s
despues de entrar en ORB y ya reaparece en el siguiente goal con GT fisicamente
desplazado a `(30.68,-35.84,32.11)`. Drone1 tambien alterna perdidas tras pasos
angulares/innovaciones grandes. El escenario completo no termina: llega a
13/17 pasos y el 14 se cancela por el Ctrl-C solicitado; los unicos ERROR son
la interrupcion controlada. Conclusion 254: `NO CONSEGUIDA`; Fase 5H sigue
`PARCIAL`. Correccion funcional propuesta, aun no autorizada: limitar/suavizar
la innovacion angular ORB en `OrbPosePredictor` y derivar de esa misma
orientacion corregida una velocidad coherente; GT permanece intacto.
Autorizacion funcional: SUSPENDIDA. Trabajo activo: documentacion de 254 y
explicacion al usuario. Siguiente accion exacta: sincronizar historial 5H,
resumen, indice, estado y ultima sesion; despues pedir acuerdo antes de editar
el filtro angular ORB o repetir la simulacion.
Documentacion 254 sincronizada: historial largo/resumen/indice 5H, contrato
5H, resumen de Fase 5, contexto minimo, estado actual, ultima sesion y doc de
`orbslam3_ros2`. `git diff --check` correcto y no queda simulacion activa.
Conclusion final de la ejecucion: prueba 254 `NO CONSEGUIDA`; Fase 5H
`PARCIAL`. Preparacion de la siguiente correccion: EN_DEBATE. Acuerdo cerrado:
no. Autorizacion funcional: SUSPENDIDA. Prueba siguiente: pendiente de acuerdo,
previsiblemente repetir la misma trayectoria tipica con Gazebo/RViz2. Duda
abierta: politica exacta para limitar/suavizar innovacion angular ORB y mantener
pose/velocidad coherentes. Trabajo activo: no. Siguiente accion exacta: explicar
el diagnostico y acordar la correccion antes de editar codigo o ejecutar otra
prueba.
Revision conversada de 254: los nueve eventos registrados con
`rotation_step_rad > 0.08` no ocurren en el mismo callback del cambio de
reference KF. Los eventos `[F5B-REFERENCE-KF]` conservan paso rotacional cero;
los picos aparecen `0.042-0.250 s` despues, con esa referencia ya activa. Hay
correlacion con intervalos de conmutacion rapida entre KFs, pero no evidencia de
un salto geometrico directo producido por el reanclaje. Historial largo y
resumen 5H sincronizados. Trabajo activo: no; autorizacion funcional suspendida.
Debate posterior 254: el usuario propone no adoptar inmediatamente un nuevo
reference KF si provoca cambios bruscos de pose/velocidad, manteniendo la
referencia actual y esperando otro candidato. Valoracion pendiente de acuerdo:
la intuicion de histeresis es buena, pero el primer frame del nuevo KF siempre
tiene paso cero por construccion y `StereoTrackingReceipt` deja de aportar
`Tcr` respecto al KF anterior. Por ello no conviene forzar ORB-SLAM3 ni mantener
indefinidamente la referencia vieja. Propuesta recomendada en el wrapper:
separar referencia reportada/candidata de referencia aceptada, poner las
medidas posteriores al cambio en probation, publicar mientras tanto la
prediccion, exigir varios frames plausibles y aplicar un gate de innovacion
continuo aun dentro del mismo KF. Si no aparece candidato estable antes de un
timeout, declarar ORB no consumible y usar `GT_FALLBACK` en Fase 5. Debounce de
autoridad W hasta aceptar referencia; epoch nuevo y tracking perdido invalidan
sin probation. Preparacion: EN_DEBATE; acuerdo cerrado: no; autorizacion
funcional: SUSPENDIDA. Dudas abiertas: confirmar politica safety-first,
duracion/frames de probation y timeout antes de editar o probar.
Acuerdo post-254 cerrado y ejecucion autorizada por el usuario. Objetivo:
incorporar en `orbslam3_ros2` probation/histeresis para reference KF y un gate
continuo de innovacion ORB, publicando prediccion durante candidaturas y
rechazando medidas dinamicamente inverosimiles. Politica safety-first: si no se
obtiene referencia/medida estable antes del timeout, estado ORB no consumible y
`GT_FALLBACK` durante la trayectoria conforme a la politica Fase 5. Pose y
velocidad angular deben derivar del mismo estado corregido. Alcance:
`NavigationStateEstimator`, `OrbPosePredictor`, integracion minima en
`StereoSlamNode`, parametros/marcadores, tests y docs. Exclusiones: GT,
`dron_individual`, ganancias, ORB-SLAM3 core, optimizador y YAML de trayectoria.
Configuracion inicial conservadora y ajustable: primer frame solo inicializa,
varios frames plausibles confirman; probation/timeout breves y gate permanente
para outliers posteriores. Prueba acordada: unitarios focales, builds
seleccionados y misma trayectoria tipica con Gazebo/RViz2. Criterios: builds y
tests correctos, GT estable, referencias aceptadas sin churn dañino, pose y
velocidad ORB coherentes, sin divergencia y progreso completo sujeto a revision
visual. Riesgo aceptado: un gate conservador puede aumentar fallback GT y
requerir ajuste. Preparacion: CERRADA. Acuerdo cerrado: si. Autorizacion
funcional: CONCEDIDA. Dudas abiertas: ninguna. Trabajo activo: implementacion,
validacion y documentacion. Siguiente accion exacta: revisar headers/tests y
diff vigente, fijar la maquina de estados minima y editar solo el wrapper.
Archivos criticos revisados. Diseno minimo: `NavigationStateEstimator` conserva
referencia activa y candidata, no cuenta el primer frame, confirma tras varios
incrementos plausibles, cancela al volver la activa y expira churn prolongado.
El resultado distingue medida aceptada de pose solo predicha. Durante probation
`StereoSlamNode` no realimenta el predictor con una pose congelada y difiere la
autoridad W hasta aceptar referencia. `OrbPosePredictor` compara orientacion
medida con predicha, rechaza el outlier completo, conserva estado pose/omega
coherente y declara unhealthy tras rechazos consecutivos. Primera referencia,
epoch y recovery conservan semantica existente; GT y `dron_individual` no se
tocan. Tests cubriran candidato estable, churn/timeout, retorno a referencia
activa, outlier dentro del mismo KF y coherencia angular. Siguiente accion
exacta: editar estimator/header/tests e integracion/parametros del stereo node.
Bloque post-254 implementado solo en `orbslam3_ros2`. `NavigationStateEstimator`
mantiene referencia activa/candidata, exige tres frames plausibles tras el
frame de inicializacion, limita probation a seis frames, cancela al volver la
activa y difiere request/metadata W hasta aceptar. `StereoSlamNode` publica
prediccion durante probation y emite `[F5H-REFERENCE-GATE]` para
pending/rejected/accepted/timeout. `OrbPosePredictor` rechaza innovacion angular
mayor de 0.08 rad o velocidad implicita mayor de 1.5 rad/s, conserva pose/omega
coherentes y vuelve ORB no consumible tras tres rechazos consecutivos; medidas
buenas usan alpha angular 1.0 sin retardo. Tests nuevos cubren rechazo,
churn/timeout, retorno a activa y outlier angular coherente; tests previos se
adaptaron a probation. Formato focal limpio. No se tocaron GT,
`dron_individual`, ORB-SLAM3 core, ganancias ni YAML. Siguiente accion exacta:
sincronizar docs actuales del wrapper, ejecutar `git diff --check` y compilar
aisladamente `orbslam3`.
Preparacion build post-254: compilar solo paquete `orbslam3` del grupo Dron con
`./codex/herramientas/build_selected_packages.sh --group dron orbslam3`, porque
todos los cambios funcionales pertenecen al wrapper y sus tests. Log completo
en `codex/archivos_auxiliares/colcon_build.log`, no se leera directamente.
Siguiente accion exacta: ejecutar build; registrar exit y paquete antes de CTest.
Build post-254 gate: exit 0, `orbslam3` 1/1 correcto en 49.7 s. Avisos legacy de
ORB-SLAM3/Eigen/cv_bridge, sin error nuevo. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido directamente. Siguiente
accion exacta: ejecutar CTest focal `test_navigation_state_estimator`; si
falla, usar solo su salida focal para corregir antes de simular.
CTest gate intento 1: 12/13 GTests correctos; falla solo
`ReferenceChurnTimesOutInsteadOfSwitching` porque la rama timeout marca
`reference_gate_timed_out=true` pero hereda mecanicamente `local_valid=true`
asignado al inicio de `Update`. No es una duda funcional. Correccion exacta:
poner `local_valid=false` y `continuity_valid=false` en esa rama, rebuild y
repetir el mismo CTest antes de cualquier simulacion.
Rebuild mecanico gate: exit 0, `orbslam3` 1/1 correcto en 8.79 s, sin error.
Log completo conservado y no leido. Siguiente accion exacta: repetir CTest
`test_navigation_state_estimator`; si pasa, validar estilo/diff y preparar una
prueba nueva con el mismo YAML tipico y Gazebo/RViz2.
CTest gate intento 2: `test_navigation_state_estimator` 1/1 target correcto,
13 GTests y cero fallos en 0.17 s. Validacion focal funcional superada.
Siguiente accion exacta: comprobar uncrustify focal, `git diff --check`, que la
prueba 255 este libre y que no exista simulacion activa; registrar comando y
patrones antes de lanzarla.
Preparacion prueba 255: estilo focal y `git diff --check` correctos, identificador
libre y sin simulacion activa. Se repetira
`prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` con Gazebo/RViz2 visibles,
`gt_fallback_enabled:=true`, startup 15 s, timeout 1200 s y espera final 35 s.
Launch identico a 254. Patrones de reduccion: `F5H-REFERENCE-GATE`,
`F5H-ORB-STATE-REJECTED`, predictor/filtro ORB, fuente/continuidad/handoff,
tracking/reference KF, scenario/SIM y errores graves. Criterios: observar si el
gate evita divergencia en giros, cuantificar aceptaciones/rechazos/timeouts y
fallback, y completar 17/17 pasos/22/22 goals sin fallo propio de Fase 5.
Siguiente accion exacta: lanzar 255; al terminar o interrumpirse, registrar el
resultado operativo antes de reducir el log completo.
Prueba 255 interrumpida por peticion del usuario. Runner exit 130 por Ctrl-C,
launch `[SIM-EXIT-CODE] 0`, 211 s de duracion monitorizada y guarda de recursos
no activada. No consta todavia una observacion visual causal del usuario; no se
presupone exito ni fallo dinamico. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_255.log` y no se leera directamente.
Siguiente accion exacta: reducir por gate/ref/predictor/fuente/scenario/errores,
analizar evidencia objetiva y documentar esta ejecucion separada.
Diagnostico objetivo 255: ejecucion llega a 7/17 pasos completos y 8 resultados
de goal antes de cancelar el paso 8 por Ctrl-C. El gate registra 108 referencias
aceptadas, cero `event=rejected` por incremento candidato y 10 timeouts por
churn; el predictor registra siete limitaciones y tres eventos unhealthy por
rechazos angulares consecutivos. En los tramos 5, 6 y 7 ambos drones entran en
ORB con salto cero; ante innovaciones angulares de 0.081-0.214 rad, la medida se
rechaza y/o expira probation, y la fuente vuelve a GT con salto cero. No vuelve
a aparecer publicada la pareja incoherente de 0.27 rad frente a omega limitada
observada en 254. Contrapartida: ORB gobierna pocos segundos y el gate provoca
fallback frecuente; no queda validada una vuelta completa ni calidad ORB
sostenida. Los ERROR finales de scenario son la interrupcion controlada; los
ERROR F5H son decisiones safety-first, no caidas de proceso. Conclusion
preliminar 255: `PARCIAL`; mecanismo de proteccion demostrado, ajuste/resultado
visual pendiente de la observacion del usuario. Autorizacion funcional:
SUSPENDIDA hasta conversar esa revision. Siguiente accion exacta: documentar la
entrada 255 y preguntar que observo el usuario antes de ajustar thresholds,
probation o repetir.
Documentacion 255 sincronizada: historial largo/resumen/indice 5H, contrato,
pipeline Fase 5, contexto minimo, estado actual, ultima sesion y docs del
wrapper. `git diff --check` correcto y no queda simulacion activa. Conclusion
vigente: prueba 255 `PARCIAL`, Fase 5H `PARCIAL`; proteccion objetiva demostrada
pero calidad visual y exceso de fallback pendientes. Preparacion de otra
iteracion: NO_INICIADA. Acuerdo cerrado: no. Autorizacion funcional:
SUSPENDIDA. Duda abierta: que observo el usuario y si la parada se debio a
movimiento anomalo, exceso de GT o simplemente revision suficiente. Trabajo
activo: no. Siguiente accion exacta: recibir esa observacion y revisar la misma
entrada 255 antes de proponer cualquier ajuste.
```

## Checkpoint vigente 2026-08-26

```text
Estado: Fase 4 CONSEGUIDA Y CERRADA con alcance 4A-4H; Fase 3 reabierta en 3Q.
Preparacion 4G+4H: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA y consumida
Prueba acordada: trayectoria tipica con Gazebo/RViz2 y ventanas 5 s, seguida
de smoke web separado.
Dudas abiertas: ninguna.
Resultado: builds correctos; CTest Servidor 150/150 y Simulacion 85/85 sin
fallos; prueba 216 correcta sin ruta GT fiducial, con 52/52 primary y cobertura
de los tres objetos; smoke 217 correcto con ambos grafos live.
Decision de cierre: el usuario aplaza 4I como regresion opcional futura; no
condiciona ni reabre el cierre de Fase 4.
Subfase actual: 3Q, para diagnosticar y corregir las optimizaciones que dejaron
derivas visibles en la prueba 213.
Preparacion 3Q: CERRADA
Acuerdo cerrado 3Q: si
Autorizacion funcional 3Q: CONCEDIDA el 2026-08-26
Prueba acordada 3Q: regresiones deterministas de los casos 194 y 213, tests y
builds seleccionados de `orbslam3_multi`, `orbslam3_server` y
`simulacion_dron`, y repeticion de la trayectoria tipica equivalente a la 213
con Gazebo, RViz2 y logs F3Q; la revision visual final corresponde al usuario.
Dudas abiertas 3Q: ninguna.
Clarificacion conversada 3Q: los fiduciales hard no se mueven; el corredor
hard-hard conserva para cada KF entre el primer y ultimo hard una pose world de
referencia y una tolerancia local modulada por su posicion en el tramo. El
validador actual rechaza si el exceso final sobre esa tolerancia supera al
exceso inicial en mas de `1e-8`, por lo que un exceso nuevo de `0.000416 m`
cancela toda la propuesta aunque los hard sigan exactamente fijos. Pendiente
explicar al usuario el grafo completo y cerrar despues la politica correctiva.
Propuesta funcional del usuario 3Q: construir la ventana por segmentos
temporales delimitados por el hard anterior y, si existe, el siguiente;
expandir a otros submapas solo mediante covisibilidad/fusiones confirmadas cuyos
extremos pertenezcan a esos segmentos; aplicar igual expansion a optimizaciones
fiduciales; condensar como priors un consenso mayoritario de al menos tres
submapas para que absorba la correccion el lado query; exigir secuencias de
loops coherentes al lado no anclado, continuidad espacial reforzada tras
perdida y apoyo creciente con riesgo/distancia. Valoracion provisional: buena
direccion, pero la mayoria debe medir fuentes independientes y cobertura
distribuida, y la distancia no debe decidir sola porque la pose world puede
estar precisamente afectada por deriva.
Confirmaciones posteriores 3Q: la rama 3Q parte de RANSAC valido con residual
bajo y discrepancia world alta; la segmentacion sera simetrica para query y
candidate; se acepta expansion solo por relaciones server confirmadas,
consenso/scaffold independiente y secuencias query-candidate coherentes. El
solver debe mantener todos los hard inmoviles y acercar las nubes segun la
transformacion RANSAC, permitiendo reparto de correccion entre ambos lados de
acuerdo con aristas y autoridad. Continuidad tras perdida y proteccion de zonas
repetidas se conservan. El usuario propone reducir el umbral de fusion.
Dato runtime verificado: hoy `loop_fusion_translation_threshold_m=0.35` y
`loop_fusion_rotation_threshold_rad=0.25` (14.3 grados); el mismo par clasifica
fusion directa y fija el residual final exigido por el validator. Cambiarlo sin
separar ambos usos puede aumentar simultaneamente solves y rechazos.
Auditoria preparatoria detallada 3Q: viable sin tocar ORB-SLAM3 ni mensajes.
`LoopPipeline` hoy acumula solo pose/id query y no comprueba progresion del
candidate; el apoyo adaptativo debe conservar endpoints canonicos de ambos
lados y una secuencia coherente. Regla propuesta configurable: 2 apoyos sin
riesgo, 4 con una senal y 6 con dos o mas; senales simples: asimetria protegida
o perdida reciente, ambiguedad competidora y correccion grande. El cierre por
submapa completo de `SparseGlobalBackend` y `BuildLoop()` debe sustituirse por
intervalos delimitados por hard consecutivos, siguiendo solo aristas server
incidentes; varios intervalos del mismo submapa se combinan en un unico batch
atomico. El fiducial multi-submapa deja de usar un loop sintetico y reutiliza
esa seleccion real. El consenso de tres segmentos independientes no fija
candidate: aporta `PriorLoop` y cobertura, y el solver reparte movimiento;
solo hard queda inmovil. Hallazgo adicional: `information_weight` participa en
el coste pero no en `RelaxEdge`, por lo que debe normalizarse por familia y
afectar la relajacion para que autoridad/soporte gobiernen de verdad. Corredor:
referencias por parejas de hard consecutivos y deadband configurable inicial
`0.02 m / 0.00872665 rad`, sin tolerancia para hard. Fusion directa propuesta
`0.20 m / 0.12 rad`; separar parametro de residual final. Regresiones nuevas:
194 ambiguo/asimetrico queda en HOLD con apoyo insuficiente y ventana acotada;
213 acepta exceso numerico y rechaza exceso material; segmentacion loop y
fiducial, consenso no hard, pesos efectivos y commit multi-intervalo.
Clarificacion de histeresis 3Q: el usuario acepta la opcion recomendada y
pregunta por el objetivo cero. El error de pose `CurrentLoop` debe minimizarse
respecto a la transformacion RANSAC hacia cero; las poses de camara y los pares
de puntos no tienen por que coincidir exactamente por distinto punto de vista y
ruido. El solver actual corta segun fracciones del umbral de aceptacion, por lo
que se propone un target de convergencia propio cercano a cero. Una propuesta
entre el limite de fusion y el maximo seguro puede comprometer una mejora sin
fusionar landmarks; debe conservar la constraint relativa aceptada como
`PriorLoop` para una futura refinacion. Solo se fusionan puntos tras opt si el
error de pose queda dentro de `0.20 m / 0.12 rad` y pasa dispersion/evidencia.
Acuerdo final de umbrales 3Q: objetivo matematico del `CurrentLoop` cero;
convergencia practica propia `0.05 m / 0.03 rad`; fusion directa o post-opt de
landmarks solo hasta `0.20 m / 0.12 rad`; aceptacion maxima de una correccion
segura sin fusion hasta `0.25 m / 0.15 rad`. Entre los dos ultimos limites se
comprometen las poses si pasan las guardas estructurales, se conserva la
constraint como `PriorLoop` y no se fusionan landmarks. Por encima del maximo
seguro, o ante movimiento de hard/infraccion estructural material, se rechaza.
Trabajo activo: ejecucion completa de 3Q autorizada sobre el estado vigente
posterior a Fase 4, usando fiduciales reales y la trayectoria tipica actual.
Plan activo 3Q: localizar simbolos desde docs de paquete; implementar ventana
segmentada/expansion covisible/apoyo adaptativo; separar convergencia, fusion y
commit seguro; hacer efectivos los pesos y la deadband de corredor; retirar
logica obsoleta; ampliar regresiones; compilar, ejecutar suites y repetir la
trayectoria equivalente a 213 con Gazebo/RViz2 y logs reducidos.
Contexto 3Q leido: contrato 3Q, especificacion, implementacion, testing,
criterios, historial 3Q resumen y fragmentos 194/213 del historial largo; docs
de `orbslam3_multi` y `orbslam3_server` localizan `LoopPipeline`,
`PoseGraphBuilder`, `OptimizationValidator` y `SparseGlobalBackend`.
Preparacion Fase 5: CERRADA para ejecutar primero 5A como auditoria y
reconciliacion documental de `Fase_5_preparada_para_Codex.zip`.
Acuerdo cerrado Fase 5: si
Autorizacion funcional Fase 5/5A: CONCEDIDA y consumida el 2026-08-25 para el
alcance exclusivamente documental acordado.
Prueba acordada Fase 5/5A: revision de coherencia documental y
`git diff --check`; sin build, test ni simulacion porque 5A no modificara codigo,
launch, YAML funcional, configuracion ni mensajes.
Dudas abiertas Fase 5/5A: ninguna.
Acuerdo Fase 5/5A: actualizar el pipeline, resumen y contratos 5A-5H para
absorber las decisiones del zip; conservar 5I como nota/stub absorbido en 5H;
crear o actualizar el indice/historial documental sin inventar ejecuciones;
preparar 5B como primera subfase funcional. Se puede avanzar documentalmente
en paralelo con 3Q, pero antes de backend fuerte en 5C/5D se verificara el
cierre final de 3Q y el HEAD vigente.
Contexto Fase 5 leido: README/resumen/pipeline propuestos del zip, subfases
propuestas 5A-5H y nota 5I absorbida; pipeline/resumen/5A vigentes del repo;
docs de `orbslam3_ros2`, `orbslam3_msgs`, `orbslam3_multi`,
`orbslam3_server`, `dron_individual`, `simulacion_dron` y ADR 0002 GT.
Observacion Fase 5: HEAD actual `a44b8b8` difiere del SHA auditado del zip
`1d585059`; el zip cambia decisiones funcionales respecto a los MD vigentes:
`O_T_B` continuo para control, `W_T_B` global corregible aparte, usar ref-KF
real + `Tcr`, rechazar goals absolutos sin `W_T_O`, no imponer smoothing,
usar `GT_FALLBACK` temporal solo en F5 y absorber la antigua 5I en 5H. ADR
0002 debe reconciliarse antes de implementar ese fallback.
Decision GT_FALLBACK confirmada por el usuario: mantenerlo durante toda Fase 5
y hasta que Fase 6 aporte tareas reales de recuperacion. Si ORB-SLAM3 se pierde,
el dron debe poder terminar trayectorias largas como el rodeo del edificio sin
quedar bloqueado. Es una fuente temporal de continuidad de control, visible y
parametrizable; no alimenta el mapa, anchors, optimizacion ni la pose global
final, y su retirada queda como condicion explicita de Fase 6.
Plan ejecutado Fase 5/5A: reconciliar `pipeline_fase_5.md`, su resumen y los
contratos 5A-5H con el zip y el acuerdo conversado; dejar 5I como stub absorbido
en 5H; crear indice e historial de 5A; verificar referencias, estados y
`git diff --check`. No modificar codigo, launch, YAML, configuracion ni mensajes;
no ejecutar build, test o simulacion.
Resultado Fase 5/5A: CONSEGUIDA documentalmente. Pipeline, resumen y contratos
reconciliados; 5I absorbida en 5H; historial creado. Verificacion correcta:
referencias presentes, cero clausulas obsoletas buscadas, contratos menores de
250 lineas, bloques Markdown equilibrados y `git diff --check` correcto.
Trabajo activo Fase 5: no. 5B no esta preparada ni autorizada.
Preparacion por bloques Fase 5B-5H: EN_DEBATE.
Estructura de bloques Fase 5 confirmada por el usuario: B1=`5B`,
B2=`5C+5D+5E+5F`, B3=`5G+5H`.
Acuerdo cerrado bloques Fase 5: no; falta cerrar el contrato ejecutable de B1.
Autorizacion funcional bloques Fase 5: PENDIENTE.
Propuesta de bloques: B1=`5B`; B2=`5C+5D+5E+5F` con checkpoints internos y
puerta humana tras metricas 5F; B3=`5G+5H` con reconciliacion previa de ADR
0002, prueba dirigida de `GT_FALLBACK` y vuelta final multi-dron.
Pruebas propuestas: tests deterministas por capa, builds aislados solo tras
cada bloque coherente y una simulacion integrada por bloque; B2 espera el
cierre de 3Q y termina presentando metricas para aceptacion del usuario; B3
incluye perdida dirigida y trayectoria tipica completa.
Dudas abiertas bloques Fase 5: la division ya esta confirmada; cerrar pruebas y
criterios dentro de la preparacion de cada bloque. El mecanismo reproducible de
perdida se propone iniciar en 5B y reutilizar en B3.
Estado de dependencia 3Q verificado: cierre publicado en HEAD `ff1e80a`; 3Q
queda `A REVISAR` solo si reaparece el outlier aislado de la prueba 220 y no
mantiene trabajo activo. Ya no bloquea el inicio de 5B.
Preparacion bloque 1/5B: CERRADA.
Acuerdo cerrado 5B: si.
Autorizacion funcional 5B: CONCEDIDA el 2026-08-26.
Objetivo propuesto 5B: muestra ORB coherente por frame, pose `O_T_B` continua
dentro de un `map_epoch`, estado de localizacion y politica de goals; no
migrar aun trayectoria/control desde GT, no backend global, velocidad ni
fallback.
Hallazgos preparatorios 5B: `StereoTrackingReceipt` no expone hoy tracking,
reference KF ni `Tcr`; obtenerlos coherentemente requiere ampliar de forma
aditiva `System.h/System.cc`. `gen_tray::handle_goal()` solo valida tipo y
captura pose/vel GT despues de aceptar; no existe snapshot ni gate global.
Prueba propuesta 5B: tests deterministas de recibo/transformaciones/estado y
politica de goals; builds aislados; una simulacion integrada multi-dron con
trayectoria relativa, cambios de reference KF/Local BA, absoluto esperado como
rechazado y, al final, llegada al fiducial, anclaje confirmado y giro relativo
de yaw de 180 grados respecto a la pose normal de la trayectoria. La camara
queda orientada hacia la zona sin texturas para provocar la perdida completa y
observar `RECENTLY_LOST`/`LOST` real. GT solo como control legacy y metrica
externa, nunca para construir `O` o decidir tracking.
Confirmaciones 5B: aceptados el cambio minimo y aditivo en
`dron/ORB_SLAM3`, una interfaz ROS coherente y ampliable para estado
local/global futuro, y el limite de continuidad solo intra-epoch hasta que 5G
implemente fallback. La perdida se provocara tras llegar al fiducial y quedar
anclado, mediante un giro real de 180 grados hacia una zona sin texturas; no se
anadira blackout visual artificial.
Prueba acordada 5B: tests deterministas, builds aislados y una simulacion
integrada multi-dron con continuidad/ref-KF/Local BA, rechazo absoluto,
aceptacion relativa y secuencia fiducial-anclaje-giro de 180 grados para
observar la perdida por estado ORB real.
Dudas abiertas 5B: ninguna.
Trabajo activo 5B: ejecucion completa autorizada sobre HEAD `ff1e80a` y los
cambios documentales preparatorios vigentes.
Plan activo 5B: localizar simbolos exactos; implementar recibo ORB coherente,
interfaz ROS y continuidad `O_T_B` intra-epoch; implementar gate y snapshot de
goals sin migrar todavia el control GT; anadir tests deterministas y compilar;
ejecutar la simulacion fiducial-anclaje-giro relativo de 180 grados; reducir y
analizar logs; actualizar documentacion e historial.
Siguiente accion exacta 5B: inspeccionar mensajes, CMake, clases y tests
existentes en los paquetes afectados antes del primer cambio funcional.
Archivos criticos localizados 5B: `dron/ORB_SLAM3/include/System.h`,
`dron/ORB_SLAM3/src/System.cc`, `dron/orbslam3_msgs/msg`,
`dron/orbslam3_ros2/src/stereo/stereo-slam-node.hpp/.cpp`,
`dron/dron_individual/src/control_tray/gen_tray.cpp`, CMake/package manifests,
`generar_dron.launch.py`, `orbslam_use.launch.py` y escenario F5B nuevo.
Diseno mecanico 5B: mensaje ROS unico namespaced con validez local/global,
tracking, epoch, ref-KF, `Tcr` y `O_T_B`; continuidad fijando `O_T_Kref` por
reference KF y reanclandolo sin salto al cambiar de referencia; gate de goals
con muestra fresca y snapshot al aceptar. Los goals absolutos se rechazan en
5B porque global es invalida; la simulacion funcional usara goals relativos.
Siguiente accion exacta 5B: aplicar el cambio aditivo del recibo ORB, crear la
interfaz/helper de continuidad y conectar su publicacion en el wrapper.
Bloque funcional 1 completado 5B: `StereoTrackingReceipt` expone tracking,
ref-KF real y `Tcr`; creado `NavigationState.msg`; el wrapper publica una
muestra coherente con `O_T_B` y validez explicita, usando `body_T_camera`; el
estimador mantiene `O_T_Kref` intra-epoch, evita saltos al cambiar ref-KF y no
finge continuidad tras perdida. `gen_tray` consume el estado fresco, rechaza
absolutos sin global, acepta relativos validos y congela epoch/muestra al
aceptar. Aniadidos tests deterministas de continuidad y politica.
Build 5B preparado: compilar primero `orbslam3_msgs` con
`./codex/herramientas/build_selected_packages.sh --group dron orbslam3_msgs`
para validar la interfaz antes de ORB, wrapper y control. Log completo en
`codex/archivos_auxiliares/colcon_build.log`, que no se leera directamente.
Siguiente accion exacta 5B: ejecutar build aislado de `orbslam3_msgs` y
registrar inmediatamente su resultado.
Build 5B `orbslam3_msgs`: 1/1 correcto, exit 0, 45.6 s. Log completo
conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido.
Build core ORB 5B preparado: recompilar la biblioteca modificada con el metodo
documentado `cmake --build dron/ORB_SLAM3/build -j4`; despues compilar el
wrapper ROS contra el nuevo recibo.
Siguiente accion exacta 5B: ejecutar el build controlado de ORB-SLAM3 y
registrar su resultado antes de compilar `orbslam3`.
Build core ORB 5B intento 1: no iniciado, exit 1 inmediato porque
`dron/ORB_SLAM3/build` no existe en el checkout actual. Diagnostico mecanico:
las bibliotecas Thirdparty y `lib/libORB_SLAM3.so` existen, pero se limpio la
cache CMake del core. No hay error de compilacion ni cambio funcional.
Siguiente accion exacta 5B: regenerar la cache con
`cmake -S dron/ORB_SLAM3 -B dron/ORB_SLAM3/build -DCMAKE_BUILD_TYPE=Release`
y, si configura, repetir `cmake --build dron/ORB_SLAM3/build -j4`.
Reanudacion 5B tras interrupcion: la configuracion CMake termino con exit 0 y
el build completo avanzo sin errores visibles, solo warnings heredados, pero
la sesion PTY `39147` ya no existe y no dejo codigo de salida verificable.
Siguiente accion exacta 5B: relanzar incrementalmente
`cmake --build dron/ORB_SLAM3/build -j4`; CMake reutilizara los objetos ya
compilados y el nuevo resultado sera la evidencia autoritativa.
Build core ORB 5B intento 2: avanzo incrementalmente sobre `Tracking`,
`LoopClosing`, `Frame` y `Optimizer`, sin errores visibles, pero una segunda
interrupcion conversacional cerro la sesion `10953` antes del codigo de salida.
Siguiente accion exacta 5B: relanzar el build incremental con `-j2` para
reducir presion durante las unidades pesadas y registrar su salida final.
Build core ORB 5B intento 3: la compilacion completa avanzo hasta los ejemplos;
se interrumpio voluntariamente con exit 130 porque la biblioteca ya estaba
enlazada y los ejecutables de ejemplo no pertenecen a 5B. Validacion
autoritaria posterior: `cmake --build dron/ORB_SLAM3/build --target ORB_SLAM3
-j2`, target `ORB_SLAM3` 100%, exit 0.
Build wrapper 5B preparado: ejecutar
`./codex/herramientas/build_selected_packages.sh --group dron orbslam3` para
validar recibo, mensaje, estimador y publicacion ROS. Log completo en
`codex/archivos_auxiliares/colcon_build.log`, no se leera directamente.
Siguiente accion exacta 5B: compilar `orbslam3` y registrar inmediatamente el
resultado; reducir el log solo si falla.
Build 5B `orbslam3`: 1/1 correcto, exit 0, 2 min 23 s. Hubo stderr solo por
warnings heredados de ORB-SLAM3/Eigen y `cv_bridge`; no se leyo el log completo
ni fue necesario reducirlo porque el build paso.
Build control 5B preparado: ejecutar
`./codex/herramientas/build_selected_packages.sh --group dron dron_individual`
para validar dependencia del mensaje, gate, snapshot y launch/calibracion.
Siguiente accion exacta 5B: compilar `dron_individual`, registrar el resultado
y reducir solo ante exit distinto de cero.
Build 5B `dron_individual`: 1/1 correcto, exit 0, 21.4 s. Log completo en
`codex/archivos_auxiliares/colcon_build.log`, no leido.
Tests 5B preparados: ejecutar CTest en `build/dron/orbslam3` y
`build/dron/dron_individual`, incluyendo `test_navigation_state_estimator` y
`test_navigation_goal_policy` junto con regresiones existentes.
Siguiente accion exacta 5B: ejecutar primero CTest de `orbslam3`, registrar el
resultado y luego CTest de `dron_individual`.
CTest 5B `orbslam3` intento 1: no ejecuto pruebas; el sandbox devolvio
`Read-only file system` al crear `build/dron/orbslam3/Testing/Temporary`.
Fallo exclusivamente operativo, sin resultado funcional.
Siguiente accion exacta 5B: repetir el mismo CTest con permiso escalado para
escribir sus artefactos normales bajo `build/`.
CTest 5B `orbslam3` intento 2: 2/2 correctos, exit 0; pasan detector fiducial
y `test_navigation_state_estimator` en 0.16 s.
Siguiente accion exacta 5B: ejecutar CTest de `dron_individual` con el mismo
permiso operativo y registrar politica de goals y linters.
CTest 5B `dron_individual`: 4/7 correctos; pasa
`test_navigation_goal_policy`, `cppcheck`, `lint_cmake` y `xmllint`. Fallan
`flake8`, `pep257` y `uncrustify` por deuda legacy masiva del paquete; la salida
muestra numerosos Python y cuatro C++ no relacionados, mientras el test nuevo
no tiene divergencia. Falta aislar si `gen_tray.cpp` aporta alguna divergencia
nueva antes de clasificar el resultado.
Siguiente accion exacta 5B: ejecutar los tests funcionales por nombre y
`ament_uncrustify` solo sobre `gen_tray.cpp`; corregir exclusivamente formato
introducido por 5B si aparece.
Validacion aislada 5B: `test_navigation_goal_policy` 1/1 correcto; formato de
`gen_tray.cpp` corregido en una unica indentacion introducida por 5B y
`ament_uncrustify` pasa en codigo/test nuevos. Los fallos globales de flake8,
pep257 y otros cuatro C++ son deuda legacy no modificada.
Instrumentacion de prueba 5B completada: `scenario_runner_node` admite
`expect_rejected`; creados el escenario instalado `f5b_fiducial_giro_180.yaml`
y `tray_prueba_221.yaml`. Secuencia: rechazo absoluto esperado, avance relativo
de ambos drones al fiducial 2, espera de anclaje y giro relativo de 180 grados
en 4 s hacia zona sin textura, seguido de observacion de perdida.
Build simulacion 5B preparado: ejecutar
`./codex/herramientas/build_selected_packages.sh --group simulacion simulacion_dron`
para instalar escenario y validar `scenario_runner_node`.
Siguiente accion exacta 5B: compilar `simulacion_dron`, registrar resultado y
ejecutar sus tests contractuales antes de Gazebo.
Build 5B `simulacion_dron`: 1/1 correcto, exit 0, 9.82 s. Log completo en
`codex/archivos_auxiliares/colcon_build.log`, no leido.
Siguiente accion exacta 5B: ejecutar CTest de `build/simulacion/simulacion_dron`
con permiso operativo, registrar regresiones y corregir solo fallos de 5B.
CTest 5B `simulacion_dron`: 10/10 correctos, exit 0; pasan cuatro contratos y
seis linters en 6.34 s.
Sincronizacion de interfaz 5B: detectada y completada la copia espejo de
`NavigationState.msg` en `servidor/orbslam3_msgs`; `cmp` confirma igualdad
exacta con Dron. Falta compilar esa copia para evitar sombreado runtime segun
el orden de overlays.
Build espejo 5B preparado: ejecutar
`./codex/herramientas/build_selected_packages.sh --group servidor orbslam3_msgs`.
Siguiente accion exacta 5B: compilar la interfaz espejo del Servidor y, si
pasa, preparar checkpoint y lanzar prueba 221.
Build espejo 5B `servidor/orbslam3_msgs`: 1/1 correcto, exit 0, 42.9 s. Log
completo en `codex/archivos_auxiliares/colcon_build.log`, no leido.
Simulacion 5B preparada: prueba 221, YAML
`codex/archivos_auxiliares/trayectorias/tray_prueba_221.yaml`, launch
`ros2 launch simulacion_dron multi_dron.launch.py`, startup 2 s, timeout 300 s,
post-scenario 10 s, un reintento Gazebo y monitor de recursos. Patrones de
reduccion: `F5B|SCENARIO-RUNNER|FID|HARD|ANCHOR|RECENTLY_LOST|LOST|ERROR|FATAL`.
Siguiente accion exacta 5B: ejecutar prueba 221 con `run_simulation.sh`,
registrar exit/success/ruta del log antes de reducir o analizar.
Prueba 221 intento 1: ejecucion real fallida, exit 1, `success=false` porque
`scenario_runner_node` termino con codigo 1. Gazebo arranco y la guarda de
recursos no se activo; min MemAvailable 6975.5 MiB, max ORB PSS 955.7 MiB.
Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_221.log` y no leido.
Siguiente accion exacta 5B: reducir prueba 221 con patrones F5B/scenario/
fiducial/tracking/error, leer solo el reducido y diagnosticar el primer fallo
real antes de modificar o repetir.
Diagnostico prueba 221: el reducido confirma dos causas operativas. El spawner
del fiducial 1 agoto su cola de Gazebo y `/fiducial_spawn_ready` no llego a
publicarse; el escenario expiro en el primer paso. Ademas ambos ORB quedaron
en `NOT_INITIALIZED` al permanecer inmoviles con una vista inicial inadecuada.
No hubo error de enlace, mensaje ni launch de 5B y la guarda de recursos no se
activo.
Correccion mecanica de prueba 5B preparada: `generador_URDF` admite un override
de spawn desactivado por defecto y `multi_dron.launch.py` lo expone para situar
los drones cerca del fiducial con yaw 90 grados solo en esta prueba. El avance
relativo se acorta a 0.8 m manteniendo anclaje, giro real de 180 grados y todos
los criterios acordados; no se debilita el gate ni se introduce GT en `O`.
Build correctivo 5B preparado: compilar `simulacion_dron` con
`./codex/herramientas/build_selected_packages.sh --group simulacion simulacion_dron`
para instalar el launch/escenario corregidos antes de repetir CTest y Gazebo.
Siguiente accion exacta 5B: verificar formato/diff del cambio correctivo y
ejecutar el build seleccionado de `simulacion_dron`.
Verificacion previa correctiva 5B: `ament_uncrustify` pasa en los dos C++
afectados, el launch compila con Python y `git diff --check` es correcto.
Build correctivo 5B `simulacion_dron`: 1/1 correcto, exit 0, 4.71 s. Log
completo conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido.
Siguiente accion exacta 5B: ejecutar CTest completo de
`build/simulacion/simulacion_dron`; si pasa, preparar la prueba 222 con el
override de spawn habilitado.
CTest correctivo 5B no iniciado: la plataforma rechazo la ejecucion con permiso
operativo al alcanzar el limite de uso del entorno, con reapertura indicada
para 2026-08-27 00:30. No es un fallo de codigo ni de test. El build correctivo
permanece correcto y no se declara 5B terminada sin CTest y simulacion.
Reintento autorizado CTest correctivo 5B: 10/10 correctos, exit 0, 7.06 s;
el rechazo anterior era externo al codigo y no se reprodujo.
Trabajo activo 5B: pendiente exclusivamente de validacion operativa y cierre
documental; implementacion y correccion mecanica completas.
Simulacion 5B prueba 222 preparada: YAML independiente
`codex/archivos_auxiliares/trayectorias/tray_prueba_222.yaml`, launch
`ros2 launch simulacion_dron multi_dron.launch.py
dron_spawn_override_enabled:=true dron_spawn_y:=-10.8
dron_spawn_yaw_deg:=90.0`, startup 2 s, timeout 300 s, post 15 s, un reintento
Gazebo y monitor de recursos.
Siguiente accion exacta 5B: crear `tray_prueba_222.yaml`, ejecutar prueba 222,
registrar su resultado y reducir el log antes de analizarlo.
Prueba 222: ejecucion real correcta a nivel de escenario, exit 0,
`success=true`; 78 muestras de recursos, minimo MemAvailable 6308.4 MiB,
max ORB PSS 998.6 MiB y `guard_triggered=false`. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_222.log` y no leido.
Siguiente accion exacta 5B: reducir prueba 222 con patrones de escenario,
gate, tracking, ref-KF, continuidad, fiducial/anclaje y errores; leer solo el
reducido y contrastar todos los criterios antes del cierre documental.
Analisis prueba 222: escenario 7/7 correcto; ambos ORB parten en tracking 2,
los cambios de reference KF conservan paso cero, el absoluto se rechaza por
`global_valid=false`, los relativos se aceptan con snapshot de epoch/muestra y
los dos giros terminan. Tras girar ambos pasan 2->3 con `local_valid=false` y
`continuity_valid=false`, y despues 3->0->1 sin continuidad ficticia. Hay
observaciones fiduciales validas de ambos drones y no aparecen ERROR/FATAL.
Limitacion de evidencia 222: `fase3_logs_terminal=false` oculta en el servidor
los marcadores `[F3E-FID-FIRST-ANCHOR]`/hard, por lo que la observacion valida
no demuestra por si sola el commit de anclaje acordado. La prueba queda
correcta salvo esa confirmacion observable y no sustituye el fallo 221.
Simulacion 5B prueba 223 preparada: repetir exactamente el YAML 222 y el mismo
spawn, anadiendo solo `fase3_logs_terminal:=true` al launch para observar el
commit fiducial hard. Startup 2 s, timeout 300 s, post 15 s, un reintento Gazebo
y monitor de recursos.
Siguiente accion exacta 5B: ejecutar prueba 223 con el YAML 222, registrar el
resultado, reducir por F3E/F5B/escenario/error y comprobar el anclaje antes del
giro y la perdida posterior.
Prueba 223: ejecucion real fallida a nivel de escenario, exit 1 porque
`scenario_runner_node` termino con codigo 1. La guarda no se activo; minimo
MemAvailable 6200.8 MiB y max ORB PSS 959.3 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_223.log` y no leido.
Siguiente accion exacta 5B: reducir prueba 223 con marcadores F3E/F5B,
escenario, tracking, fiducial y errores; diagnosticar el primer fallo real antes
de corregir o repetir.
Diagnostico prueba 223: `fiducial_object_2` entro en la cola de spawn pero el
servicio de Gazebo agoto el timeout; el spawner murio, nunca publico ready y el
escenario fallo en el paso 1 tras 90 s. Ambos ORB si alcanzaron tracking 2.
Fallo transitorio de infraestructura equivalente al 221, sin evidencia contra
la implementacion ni contra los criterios funcionales.
Simulacion 5B prueba 224 preparada: repeticion identica de 223, sin cambios de
codigo, YAML, comportamiento ni criterio, para obtener el marcador de anchor
con `fase3_logs_terminal:=true`. Mismos limites y monitor de recursos.
Siguiente accion exacta 5B: ejecutar prueba 224, registrar el resultado y
reducir el log antes de analizarlo.
Prueba 224: ejecucion real correcta a nivel de escenario, exit 0,
`success=true`; 78 muestras, minimo MemAvailable 6619.6 MiB, max ORB PSS
998.1 MiB y `guard_triggered=false`. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_224.log` y no leido.
Siguiente accion exacta 5B: reducir prueba 224 por F3E/F5B/escenario/error,
leer solo el reducido y decidir el cierre contra todos los criterios.
Analisis prueba 224: repite correctamente gate, snapshots, cambios ref-KF con
paso cero, giro y perdida 2->3->0->1 en ambos drones, sin continuidad ficticia.
No aparece telemetria F3 porque el launch declara
`debug_fase3_logs_terminal`, mientras la prueba uso por error mecanico
`fase3_logs_terminal`; ese argumento no modifico el nivel `error` del servidor.
Simulacion 5B prueba 225 preparada: repeticion final identica con el nombre
correcto `debug_fase3_logs_terminal:=true`; no cambia codigo, trayectoria,
comportamiento ni criterios.
Siguiente accion exacta 5B: ejecutar prueba 225, registrar y reducir; exigir
anchor hard previo al giro junto con toda la evidencia F5B ya observada.
Prueba 225: ejecucion real correcta a nivel de escenario, exit 0,
`success=true`; 79 muestras, minimo MemAvailable 6629.6 MiB, max ORB PSS
1000.8 MiB y `guard_triggered=false`. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_225.log` y no leido.
Siguiente accion exacta 5B: reducir prueba 225 por F3E/F5B/escenario/error,
leer solo el reducido, registrar la conclusion y cerrar documentacion.
Analisis prueba 225: ambos drones inicializan tracking 2 y sus submapas `(1,0)`
y `(2,0)` reciben `[F3E-FID-FIRST-ANCHOR] status=applied` seguido de
`[F3E-FID-KF-HARD] hard=true` antes de los goals. El absoluto se rechaza por
global invalida; relativos y giros se aceptan con snapshots congelados. Todos
los cambios de ref-KF observados mantienen paso cero y continuidad valida. Tras
el giro ambos pasan 2->3 con local/continuidad invalidas y despues 3->0->1 sin
fabricar continuidad. Escenario 7/7 correcto. El unico ERROR es el exit 255
conocido de Gazebo durante cleanup posterior a `success=true`.
Conclusion tecnica 5B: CONSEGUIDA. Builds e interfaces correctos, tests
funcionales correctos, simulacion integrada final 225 satisface todos los
criterios. Los intentos 221 y 223 fallidos y las pruebas 222/224 correctas pero
con observabilidad incompleta se conservan como entradas independientes.
Siguiente accion exacta 5B: actualizar docs de paquetes, historial 5B y su
resumen, indice/estado/pipeline/ultima sesion; ejecutar verificaciones finales
de formato, referencias y `git diff --check`.
Cierre documental 5B completado: actualizados contrato, pipeline de Fase 5,
PIPELINE_MAESTRO, historial largo/resumen/indice, contexto minimo, estado,
ultima sesion y docs de ORB_SLAM3, mensajes, wrapper, control y Simulacion.
Preparacion 5B: CERRADA.
Acuerdo cerrado 5B: si.
Autorizacion funcional 5B: CONCEDIDA y consumida.
Resultado final 5B: CONSEGUIDA en prueba 225.
Trabajo activo 5B: no.
Dudas abiertas 5B: ninguna.
Siguiente accion exacta Fase 5: ninguna sin nueva preparacion; el siguiente
bloque acordado es 5C+5D+5E+5F.
Verificacion final 5B: `git diff --check` correcto; mensajes Dron/Servidor y
escenario instalado/auxiliar idénticos; historial, tests y headers presentes;
busqueda de estados obsoletos 5B sin coincidencias. No queda trabajo activo.
Preparacion bloque 2/5C+5D+5E+5F: EN_DEBATE.
Acuerdo cerrado bloque 2: no.
Autorizacion funcional bloque 2: PENDIENTE.
Prueba propuesta bloque 2: tests deterministas por capa y una simulacion
integrada de dos drones con control GT legacy, anchors/loops reales, cambios de
reference KF y revisiones globales; 5F presenta metricas O/W frente a GT y se
detiene en puerta humana antes de 5G.
Hallazgos preparatorios bloque 2: HEAD `ff1e80a` contiene el cierre 3Q y el
working tree conserva 5B completa; la dependencia esta satisfecha sin revertir
ni separar esos cambios. `SparseGlobalBackend::GetGlobalPose()` y
`GlobalPoseRecord::pose_revision` ya proporcionan la autoridad de 5C, por lo
que no se creara otra base. 5D debe anadir servicio inicial y push dirigido al
wrapper; pending se puede resolver por interes activo y eventos de commit/raw,
sin polling. 5E extiende `NavigationState` con estado global/revision y compone
W por frame sin alterar O. 5F requiere correlacion temporal, alineado GT->O por
epoch y comparacion W->GT directa, sin realimentar GT ni integrar control.
Dudas abiertas bloque 2: semantica de provisional frente a goals absolutos;
politica exacta de pending al cambiar de reference KF; formato de entrega
cuantitativa/visual de 5F.
Siguiente accion exacta bloque 2: explicar propuesta, riesgos y pruebas al
usuario; cerrar dudas y esperar una autorizacion posterior antes de editar
codigo, interfaces, launch o configuracion.
Confirmaciones del usuario bloque 2: los goals absolutos permanecen rechazados
durante 5C-5F y 5H debera habilitarlos solo tras implementar world->O; pending
conserva exclusivamente la reference KF mas reciente por dron y una referencia
anterior se vuelve a solicitar si reaparece; 5F entregara metricas numericas y
graficas temporales O/W/GT por dron.
Semantica cerrada bloque 2: `W_T_B` provisional puede publicarse para
observabilidad, pero `global_valid=false` hasta recibir autoridad; estado
global explicito INVALID/PROVISIONAL/AUTHORITATIVE y `pose_revision` en
`NavigationState`; `pose_source=ORB` mientras ORB sea la fuente de control.
El action server mantendra un gate explicito de absolutos deshabilitado durante
este bloque para no ejecutar semantica GT/world antes de 5H.
Transporte cerrado bloque 2: servicio compartido de consulta inicial, respuesta
PENDING inmediata y push reliable dirigido por dron al materializarse o cambiar
la pose; sin polling ni heartbeat. Respuestas tardias/revisiones antiguas se
descartan y un nuevo reference KF sustituye el interes pending anterior.
Validacion cerrada bloque 2: tests por capas y una trayectoria integrada larga
de dos drones con control GT legacy, anchors/loops reales y metricas por
timestamp. Si no ocurre una revision global significativa natural, se permite
una prueba dirigida adicional sin GT funcional. 5F presenta numeros y graficas
y se detiene para aceptacion humana antes de 5G.
Preparacion bloque 2/5C+5D+5E+5F: CERRADA.
Acuerdo cerrado bloque 2: si.
Autorizacion funcional bloque 2: PENDIENTE.
Dudas abiertas bloque 2: ninguna.
Siguiente accion exacta bloque 2: esperar una orden posterior explicita para
ejecutar; entonces actualizar los contratos 5C-5F, localizar simbolos exactos,
implementar por checkpoints internos, compilar, probar y detenerse en la puerta
humana de 5F.
Actualizacion documental bloque 2 autorizada por el usuario: sincronizar ahora
los contratos 5C-5F, pipeline/resumen y contexto compacto con el acuerdo ya
cerrado; no modificar codigo, mensajes, launch, YAML ni configuracion y no
ejecutar build, tests o simulacion.
Siguiente accion exacta bloque 2: aplicar la reconciliacion documental y
verificar referencias, tamaños y `git diff --check`.
Actualizacion documental bloque 2 completada: contratos 5C-5F,
`pipeline_fase_5.md`, su resumen, `PIPELINE_MAESTRO.md`, contexto minimo,
estado actual resumido y ultima sesion sincronizados con el acuerdo cerrado.
Verificacion documental: `git diff --check` correcto; contratos 5C-5F por
debajo de 250 lineas, bloques Markdown equilibrados y busqueda de estados
obsoletos sin coincidencias. No se modifico codigo, mensajes, launch, YAML ni
configuracion; no hubo build, tests ni simulacion.
Preparacion bloque 2/5C+5D+5E+5F: CERRADA.
Acuerdo cerrado bloque 2: si.
Autorizacion funcional bloque 2: PENDIENTE.
Dudas abiertas bloque 2: ninguna.
Trabajo activo bloque 2: no.
Siguiente accion exacta bloque 2: esperar una orden explicita del usuario para
ejecutar 5C+5D+5E+5F sobre el HEAD vigente.
Autorizacion funcional bloque 2: CONCEDIDA el 2026-08-26 para ejecutar
5C+5D+5E+5F conforme al acuerdo cerrado.
Trabajo activo bloque 2: implementacion, builds, tests deterministas,
simulacion integrada larga de dos drones, analisis O/W/GT y cierre documental.
Plan activo bloque 2: localizar interfaces desde docs de paquete; implementar
consulta global y estados 5C; servicio/push dirigido y pending 5D; composicion
W y extension aditiva de `NavigationState` 5E; captura, metricas y graficas 5F;
compilar por paquetes, ejecutar suites y realizar la prueba integrada acordada.
Prueba acordada bloque 2: trayectoria larga de dos drones con control GT
legacy, anchors/loops reales, cambios de reference KF y revisiones globales;
comparacion temporal numerica y grafica O/W/GT. Se permite una prueba dirigida
adicional de revision si la trayectoria no produce una revision significativa.
Dudas abiertas bloque 2: ninguna.
Siguiente accion exacta bloque 2: consultar contexto minimo, contratos 5C-5F,
resumen de Fase 5, indice historico y docs de los paquetes afectados para
localizar simbolos y archivos criticos antes del primer cambio funcional.
Archivos criticos localizados bloque 2: API en
`servidor/orbslam3_multi/{include,src}/sparse_global_backend.*`; transporte en
`servidor/orbslam3_server/src/global_map_server.cpp`; interfaces canonica y
replica en `servidor/dron/orbslam3_msgs`; estimador y wrapper en
`dron/orbslam3_ros2/src/stereo/{navigation-state-estimator,stereo-slam-node}.*`;
metricas y launch en `simulacion/simulacion_dron`. Los commits principal y
secundario son los eventos que resolveran pending y revisiones, sin timer de
polling. El recolector 5F suscribira NavigationState y GT namespaced, alineara
GT->O una vez por epoch y generara JSON/CSV/PNG por timestamp.
Siguiente accion exacta bloque 2: aplicar el contrato de consulta 5C y los
mensajes/servicio 5D en ambas copias, junto con sus tests deterministas, antes
de conectar callbacks ROS o modificar el estimador.
Bloque funcional implementado 5C-5F: `QueryGlobalPose()` clasifica autoridad
sin duplicar store; mensajes/servicio canonico y replica exponen identidad,
status, revision y W_T_KF; el servidor conserva un interes activo por dron y
publica push reliable solo tras materializacion/revision; el wrapper descarta
respuestas tardias, solicita al cambiar ref-KF y el estimador publica W
provisional/autoritativa sin mover O. `NavigationState` incorpora estado global
y revision. 5F anade recolector O/W/GT por timestamp, alineacion fija por epoch,
JSON/CSV/PNG y escenario relativo largo de dos drones `tray_prueba_226.yaml`.
Validacion mecanica: `git diff --check` correcto, Python compila y YAML 226
parsea 12 pasos/14 goals. Intento directo `pytest`: no iniciado porque el
ejecutable no esta en PATH; se ejecutara por CTest tras instalar overlays.
Siguiente accion exacta bloque 2: revisar contratos de copias/interfaces y
compilar primero `orbslam3_msgs` canonico y replica, antes de backend, servidor,
wrapper y simulacion.
Build bloque 2 `servidor/orbslam3_msgs`: 1/1 correcto, exit 0, 49.9 s. Las
interfaces nuevas se generan en la copia canonica. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido.
Siguiente accion exacta bloque 2: compilar la replica
`dron/orbslam3_msgs`; despues validar backend contra la interfaz canonica.
Build bloque 2 `dron/orbslam3_msgs`: 1/1 correcto, exit 0, 50.5 s. Copias
`msg/` y `srv/` verificadas identicas antes del build. Log completo conservado
y no leido.
Siguiente accion exacta bloque 2: compilar `orbslam3_multi` para validar
`QueryGlobalPose()` y su regresion de estados 5C.
Build bloque 2 `orbslam3_multi`: 1/1 correcto, exit 0, 1 min 22 s. Log completo
conservado y no leido.
Siguiente accion exacta bloque 2: ejecutar CTest de `orbslam3_multi` con el
overlay Servidor para validar la regresion 5C y toda la suite backend antes de
compilar `orbslam3_server`.
CTest bloque 2 `orbslam3_multi`: 9/9 targets correctos, exit 0, 28.73 s;
incluye `test_sparse_global_backend` con AVAILABLE/PENDING/UNKNOWN/
INVALID_EPOCH y conserva todas las regresiones previas.
Siguiente accion exacta bloque 2: compilar `orbslam3_server` para validar el
servicio compartido, el interes activo por dron y el push event-driven.
Build bloque 2 `orbslam3_server`: 1/1 correcto, exit 0, 41.0 s. Servicio y push
compilan contra la interfaz canonica y el backend vigente. Log completo
conservado y no leido.
Siguiente accion exacta bloque 2: ejecutar CTest completo de
`orbslam3_server`; si pasa, compilar el wrapper Dron y su estimador 5E.
CTest bloque 2 `orbslam3_server` intento 1: 11/12 correctos, exit 8. Pasan los
seis targets funcionales y cinco linters; falla solo `uncrustify` por la
sangria de una linea en la declaracion `global_pose_service_`. Correccion
mecanica autorizada: ajustar esa sangria y repetir la suite completa.
Siguiente accion exacta bloque 2: aplicar la sangria indicada por uncrustify y
repetir CTest de `orbslam3_server`.
CTest bloque 2 `orbslam3_server` intento 2: 12/12 correctos, exit 0, 5.40 s;
la correccion fue exclusivamente de formato.
Siguiente accion exacta bloque 2: compilar `dron/orbslam3` para validar cliente,
push, descarte stale, composicion W y ampliacion de NavigationState.
Build bloque 2 `dron/orbslam3` intento 1: fallido, exit 2, 34.8 s. El compilador
no puede inferir la lambda generica del callback `async_send_request` en ROS 2
Iron. Log completo conservado y no se usara para diagnostico adicional.
Siguiente accion exacta bloque 2: ejecutar `reduce_build_log.sh`, confirmar el
primer error en el reducido y tipar explicitamente el `SharedFuture` del
callback sin cambiar semantica.
Diagnostico build wrapper intento 1 confirmado en log reducido: unica familia
de errores propia en `RequestGlobalPose()`, donde `rclcpp::function_traits` no
resuelve `operator()` de lambda generica. Correccion mecanica aplicada: callback
tipado como `Client<GetGlobalKeyFramePose>::SharedFuture`; sin cambio funcional.
Siguiente accion exacta bloque 2: repetir build aislado de `dron/orbslam3`.
Build bloque 2 `dron/orbslam3` intento 2: 1/1 correcto, exit 0, 35.6 s. Stderr
contiene solo warnings heredados de ORB-SLAM3/Eigen/cv_bridge. Log completo no
leido.
Siguiente accion exacta bloque 2: ejecutar CTest de `dron/orbslam3`, incluida
la regresion de autoridad/revision/provisional 5E.
CTest bloque 2 `dron/orbslam3`: 2/2 correctos, exit 0, 0.20 s. La suite valida
que una revision mueve W sin mover O, el cambio ref-KF pasa por PROVISIONAL,
una respuesta stale se descarta y un epoch nuevo invalida autoridad.
Siguiente accion exacta bloque 2: compilar `simulacion_dron` para instalar
recolector 5F, launch y test de metricas; despues ejecutar su CTest completo.
Build bloque 2 `simulacion_dron`: 1/1 correcto, exit 0, 9.88 s. Recolector,
launch y escenario quedan instalados. Log completo conservado y no leido.
Siguiente accion exacta bloque 2: ejecutar CTest completo de
`simulacion_dron`, incluido `pose_metrics_contract`.
CTest bloque 2 `simulacion_dron` intento 1: 10/11 correctos, exit 8, 10.62 s.
Pasa `pose_metrics_contract` y todos los contratos funcionales; falla solo
`flake8` E402 porque `MPLCONFIGDIR` se fija deliberadamente antes de importar
Matplotlib para evitar escritura en HOME. Correccion mecanica: anotar esos
imports con excepcion local `noqa: E402` y repetir CTest.
Siguiente accion exacta bloque 2: aplicar la excepcion local de lint y repetir
CTest completo de `simulacion_dron`.
CTest bloque 2 `simulacion_dron` intento 2: 11/11 correctos, exit 0, 9.63 s;
incluye metricas, contratos de configuracion/launch y todos los linters.
Siguiente accion exacta bloque 2: recompilar `dron_individual` contra el
NavigationState aditivo y repetir su test de politica para confirmar que goals
absolutos siguen deshabilitados durante 5C-5F.
Build bloque 2 `dron_individual`: 1/1 correcto, exit 0, 19.1 s. Log completo
conservado y no leido.
Siguiente accion exacta bloque 2: ejecutar solo
`test_navigation_goal_policy` (la suite completa conserva deuda legacy de
linters ya documentada) y despues preparar simulacion 226.
CTest enfocado bloque 2 `dron_individual`: `test_navigation_goal_policy` 1/1
correcto, exit 0, 0.10 s; los goals absolutos permanecen rechazados y los
relativos validos siguen aceptados.
Siguiente accion exacta bloque 2: preparar prueba 226 con escenario relativo
largo, spawn F5B, metricas 5F activas y logs F3/F5; registrar comando, timeout,
patrones y salida antes de ejecutar Gazebo.
Simulacion bloque 2 preparada: prueba 226, YAML
`codex/archivos_auxiliares/trayectorias/tray_prueba_226.yaml`, launch headless
`ros2 launch simulacion_dron multi_dron.launch.py launch_gazebo_gui:=false
launch_mission_gui:=false dron_spawn_override_enabled:=true
dron_spawn_y:=-10.8 dron_spawn_yaw_deg:=90.0
debug_fase3_logs_terminal:=true phase5_pose_metrics_enabled:=true
phase5_pose_metrics_output_dir:=/home/chenfu/Gazebo/src/codex/archivos_auxiliares/metricas/prueba_226`.
Startup 2 s, timeout 900 s, post 25 s, un reintento Gazebo y monitor de
recursos. Patrones de reduccion: `F5C|F5D|F5E|F5F|SCENARIO-RUNNER|F3E-FID|F3O|F3Q|ERROR|FATAL`.
Siguiente accion exacta bloque 2: ejecutar prueba 226 con
`run_simulation.sh`, registrar exit/success/log antes de reducir o analizar.
Prueba 226: ejecucion real fallida, exit 1, `success=false`; el scenario runner
termino con codigo 1 tras unos 33 s. Guarda de recursos inactiva, minimo
MemAvailable 6646.9 MiB y max ORB PSS 961.1 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_226.log` y no leido.
Siguiente accion exacta bloque 2: reducir prueba 226 por F5C-F5F, escenario,
launch, imports y errores; leer solo el reducido y diagnosticar el primer fallo
real antes de corregir o repetir.
Diagnostico prueba 226: dos fallos mecanicos independientes. El scenario runner
recibio una ruta YAML relativa a un cwd distinto y devolvio `bad file`; el nodo
5F murio al asignar `self.subscriptions`, propiedad reservada read-only de
rclpy. Evidencia positiva previa al cierre: ambos drones hicieron consulta
PENDING, recibieron anchor hard, push revision 1 y pasaron a AUTHORITATIVE con
`accepted=true`; backend y wrappers cerraron limpios.
Correccion mecanica preparada: renombrar la coleccion a
`input_subscriptions`, anadir test que construya el nodo y repetir como prueba
227 usando copia YAML propia/ruta absoluta, sin cambiar trayectoria,
comportamiento ni criterios.
Siguiente accion exacta bloque 2: aplicar el rename/test, recompilar y ejecutar
CTest de `simulacion_dron` antes de preparar prueba 227.
Reanudacion bloque 2 tras compactacion 2026-08-27: releido fisicamente este
checkpoint y reconciliado con la ultima orden `sigue`. El acuerdo permanece
cerrado, la autorizacion funcional sigue concedida y no hay dudas abiertas.
El rename a `input_subscriptions` y su test de construccion ya estan aplicados.
Siguiente accion exacta bloque 2: verificar mecanicamente el recolector 5F y
compilar `simulacion_dron`; registrar el resultado antes de ejecutar CTest.
Build correctivo bloque 2 `simulacion_dron`: 1/1 correcto, exit 0, 0.96 s.
El rename a `input_subscriptions` compila y queda instalado; log completo
conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido.
Siguiente accion exacta bloque 2: ejecutar CTest completo de
`build/simulacion/simulacion_dron`, incluido el test de construccion real del
nodo, y registrar su resultado antes de preparar la prueba 227.
CTest correctivo bloque 2 intento 1: no iniciado; la revision automatica del
permiso para escribir artefactos normales bajo `build/` agoto su plazo. No es
evidencia de fallo funcional ni se genero resultado de suite.
Siguiente accion exacta bloque 2: repetir una vez el mismo CTest con identico
alcance y registrar el resultado autoritativo.
CTest correctivo bloque 2 intento 2: 11/11 correctos, exit 0, 10.81 s;
`pose_metrics_contract` construye el nodo y valida cuatro suscripciones, y
tambien pasan todos los contratos y linters.
Siguiente accion exacta bloque 2: crear `tray_prueba_227.yaml` como copia
independiente de la trayectoria 226 y preparar la simulacion con ruta YAML
absoluta y salida de metricas propia.
Simulacion bloque 2 prueba 227 preparada: YAML independiente e identico a 226
en `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/tray_prueba_227.yaml`;
launch headless con spawn F5B, logs F3/F5 y metricas activas hacia
`codex/archivos_auxiliares/metricas/prueba_227`; startup 2 s, timeout 900 s,
post 25 s, un reintento Gazebo y monitor de recursos. Patrones de reduccion:
`F5C|F5D|F5E|F5F|SCENARIO-RUNNER|F3E-FID|F3O|F3Q|ERROR|FATAL`.
Siguiente accion exacta bloque 2: ejecutar prueba 227 con `run_simulation.sh`,
registrar exit, success y ruta del log antes de reducir o analizar.
Prueba 227: ejecucion real fallida, exit 1, `success=false`; el scenario runner
termino con codigo 1. La ruta YAML absoluta fue aceptada y la guarda de
recursos no se activo; minimo MemAvailable 6259.4 MiB y max ORB PSS 1030.0 MiB.
Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_227.log` y no leido.
Siguiente accion exacta bloque 2: reducir prueba 227 por marcadores F5C-F5F,
escenario, fiduciales, tracking y errores; leer solo el reducido y diagnosticar
el primer fallo real antes de modificar o repetir.
Diagnostico prueba 227: la ruta absoluta y el nodo 5F funcionan; ambos drones
anclan, reciben autoridad y se observan revisiones (incluida revision 2) y loop
fusionado. El dron 2 pierde tracking durante el tramo unico largo hacia la
esquina sur y el gate 5B rechaza el goal siguiente, comportamiento esperado
sin el `GT_FALLBACK` que corresponde a 5G. El recolector escribe reportes pero
en cleanup recibe `ExternalShutdownException` y llama `rclpy.shutdown()` por
segunda vez. Son dos limites mecanicos de prueba: cierre idempotente y ruta
demasiado expuesta para validar 5C-F antes de 5G.
Correccion mecanica preparada: hacer idempotente el cierre de rclpy y crear
prueba 228 larga por varios tramos cortos en zona texturizada, conservando dos
drones, anchors, loops, cambios de reference KF, revisiones y metricas O/W/GT.
No se habilita fallback ni se cambia ningun gate o criterio funcional.
Siguiente accion exacta bloque 2: inspeccionar el `main()` y test de metricas,
aplicar la correccion de cleanup y preparar el YAML 228 antes de rebuild/CTest.
Correccion prueba 228 completada: `main()` tolera shutdown externo sin segundo
`rclpy.shutdown()`, el report final evita rosout con contexto invalido y existe
regresion del lifecycle. `tray_prueba_228.yaml` define 12 pasos/16 goals
relativos por tramos cortos en la zona sur texturizada, sin fallback ni cambio
de gates. Python, YAML y `git diff --check` correctos.
Build prueba 228 preparado: recompilar `simulacion_dron` con
`./codex/herramientas/build_selected_packages.sh --group simulacion simulacion_dron`.
Siguiente accion exacta bloque 2: ejecutar el build y registrar su resultado
antes de CTest.
Build prueba 228 `simulacion_dron`: 1/1 correcto, exit 0, 0.97 s. Log completo
conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido.
Siguiente accion exacta bloque 2: ejecutar CTest completo de
`build/simulacion/simulacion_dron`, incluida la regresion de shutdown externo,
y registrar el resultado antes de preparar Gazebo 228.
CTest prueba 228 `simulacion_dron`: 11/11 correctos, exit 0, 11.11 s; pasan
`pose_metrics_contract`, todos los contratos y todos los linters.
Simulacion bloque 2 prueba 228 preparada: YAML absoluto
`/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/tray_prueba_228.yaml`,
mismo launch headless/spawn/logs, metricas hacia `metricas/prueba_228`, startup
2 s, timeout 900 s, post 25 s, un reintento Gazebo y monitor de recursos.
Patrones: `F5C|F5D|F5E|F5F|SCENARIO-RUNNER|F3E-FID|F3O|F3Q|ERROR|FATAL`.
Siguiente accion exacta bloque 2: ejecutar prueba 228, registrar resultado y
ruta del log antes de reducir o analizar.
Prueba 228: ejecucion real fallida, exit 1, `success=false`; el scenario runner
termino con codigo 1. Guarda de recursos inactiva, minimo MemAvailable
6291.3 MiB y max ORB PSS 1027.4 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_228.log` y no leido.
Siguiente accion exacta bloque 2: reducir prueba 228 por marcadores F5C-F5F,
escenario, tracking y errores; localizar el punto exacto y decidir una
validacion dirigida, sin otra variacion de ruta a ciegas.
Diagnostico prueba 228: ambos drones completan anclaje y primer desplazamiento;
el dron 2 pierde tracking `2->3->0->1` durante una separacion lateral de solo
3 m y el siguiente goal se rechaza por el gate vigente. Reducir mas la ruta no
resolveria el limite previo a 5G. El cierre evita ya el doble shutdown, pero
ROS Iron puede lanzar tambien `RCLError` desde el wait set cuando el contexto
queda invalido antes de `ExternalShutdownException`.
Validacion dirigida preparada: prueba 229 con arranque/anclaje y un unico tramo
largo ya aceptado por dron, seguido de observacion/reporte. Asi el control GT
legacy termina aun si ORB se pierde, sin fallback, polling ni relajacion del
gate, y conserva suficiente duracion para anchors, loops, ref-KF y revisiones.
Correccion lifecycle: propagar excepciones mientras `rclpy.ok()` y silenciar
solo la carrera de cierre cuando el contexto ya no es valido.
Siguiente accion exacta bloque 2: aplicar la correccion/test de lifecycle y
crear YAML 229; despues rebuild, CTest y simulacion.
Correccion/lanzamiento 229 completados: el recolector silencia cualquier
excepcion de `spin` solo si el contexto ya esta invalido y conserva propagacion
normal mientras ROS esta activo; nueva regresion cubre la carrera del wait set.
YAML 229 valido con 6 pasos/4 goals, incluido tramo lateral de 60 s ya
aceptado. Python y `git diff --check` correctos.
Build prueba 229 preparado: recompilar `simulacion_dron` y registrar resultado
antes de repetir CTest completo.
Siguiente accion exacta bloque 2: ejecutar build seleccionado de
`simulacion_dron`.
Build prueba 229 `simulacion_dron`: 1/1 correcto, exit 0, 1.04 s; log completo
conservado y no leido.
Siguiente accion exacta bloque 2: ejecutar CTest completo con la regresion de
wait set invalido y registrar resultado antes de preparar Gazebo 229.
CTest prueba 229 `simulacion_dron`: 11/11 correctos, exit 0, 10.27 s; pasan
metricas, lifecycle, contratos y linters.
Simulacion bloque 2 prueba 229 preparada: YAML absoluto
`/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/tray_prueba_229.yaml`,
launch headless con spawn/logs F3-F5, metricas hacia `metricas/prueba_229`,
startup 2 s, timeout 900 s, post 25 s, un reintento y monitor. Patrones:
`F5C|F5D|F5E|F5F|SCENARIO-RUNNER|F3E-FID|F3O|F3Q|ERROR|FATAL`.
Siguiente accion exacta bloque 2: ejecutar prueba 229 y registrar resultado
antes de reducir o analizar.
Prueba 229: ejecucion real correcta, exit 0, `success=true`; escenario completo
en una sesion de 171 s. Guarda inactiva, minimo MemAvailable 6094.9 MiB y max
ORB PSS 1021.6 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_229.log` y no leido.
Siguiente accion exacta bloque 2: reducir prueba 229 por marcadores F5C-F5F,
escenario, fiduciales, loops, tracking y errores; analizar solo el reducido y
despues inspeccionar JSON/CSV/PNG de `metricas/prueba_229`.
Analisis inicial prueba 229: escenario completo, dos anchors hard, consultas
PENDING/AVAILABLE, pushes primary/secondary, autoridad aceptada, revisiones
naturales hasta al menos 3, multiples loops fusionados y perdida real del dron
2 sin abortar el goal ya aceptado. No hay ERROR/FATAL/Traceback ni procesos
muertos; el lifecycle queda limpio. Se generaron JSON, dos CSV y dos PNG.
Hallazgo 5F: `latency_sec` es invalida (~1.787e9 s) por mezclar reloj wall con
stamp simulado. Hallazgo visual: W del dron 2 sigue GT en posicion hasta la
perdida (~0.154 m MAE), pero W del dron 1 diverge materialmente (~5.38 m MAE)
y las metricas angulares W son grandes; no se cierra la puerta humana hasta
separar error del recolector de error real de composicion/autoridad.
Siguiente accion exacta bloque 2: inspeccionar convenciones de timestamps y
pose en recolector/estimador/wrapper usando docs y fragmentos minimos, corregir
lo que pertenezca a 5C-F y repetir validacion si cambia semantica medida.
Conclusion prueba 229: escenario y transporte 5C-E CONSEGUIDOS, validacion 5F
PARCIAL. La prueba demuestra revisiones reales y cierre limpio, pero su latencia
no es util y W no cumple coherencia global para ambos drones; esta evidencia no
se borra ni se reclasifica como pase completo.
Correccion 5F aplicada: latencia relativa a un offset fijo reloj ROS/image
stamp por epoch, frecuencia y jitter explicitos en JSON, con regresion temporal.
No se altera W, O, backend ni control; la divergencia seguira siendo evidencia.
Python y `git diff --check` correctos.
Siguiente accion exacta bloque 2: rebuild y CTest de `simulacion_dron`; si
pasan, repetir el escenario dirigido como prueba 230 para obtener metricas
temporales validas y presentar la puerta humana con el resultado real de W.
Build metricas 230 `simulacion_dron`: 1/1 correcto, exit 0, 0.97 s; log
completo conservado y no leido.
Siguiente accion exacta bloque 2: ejecutar CTest completo y registrar resultado
antes de preparar la simulacion 230.
CTest metricas 230 `simulacion_dron`: 11/11 correctos, exit 0, 10.87 s.
Simulacion bloque 2 prueba 230 preparada: YAML independiente e identico a 229
en `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/tray_prueba_230.yaml`,
launch headless con salida `metricas/prueba_230`, startup 2 s, timeout 900 s,
post 25 s, un reintento y monitor. Patrones F5C-F5F, escenario, F3E/F3O,
tracking, ERROR y FATAL.
Siguiente accion exacta bloque 2: ejecutar prueba 230, registrar resultado y
reducir antes de analizar metricas finales.
Prueba 230: ejecucion real correcta, exit 0, `success=true`, duracion 171 s;
guarda inactiva, minimo MemAvailable 6061.1 MiB y max ORB PSS 1028.6 MiB.
Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_230.log` y no leido.
Siguiente accion exacta bloque 2: reducir prueba 230 por F5C-F5F, escenario,
F3E/F3O, tracking y errores; analizar solo el reducido y revisar JSON/PNG.
Analisis final prueba 230: escenario completo `success=true`, dos anchors hard,
consultas PENDING/AVAILABLE, pushes primary/secondary, loops fusionados y
revisiones naturales hasta al menos 4. El dron 2 pierde tracking `2->3->0->1`
despues de aceptar el goal largo; el escenario termina sin ERROR/FATAL ni
procesos muertos. JSON/CSV/PNG generados y revisados visualmente.
Metricas 230: frecuencia 19.162/19.148 Hz, jitter p95 0.001/0.003 s y latencia
relativa p95 0.480/0.240 s para drones 1/2. W posicion MAE 2.953/0.191 m y W
angular MAE 1.907/2.104 rad. La instrumentacion es valida, pero W no es
coherente para ambos drones y no supera la puerta de calidad.
Conclusion bloque 2: 5C CONSEGUIDA, 5D CONSEGUIDA, 5E CONSEGUIDA tecnicamente
y 5F PARCIAL. Puerta humana no aceptada; 5G no autorizado ni iniciado.
Cierre documental bloque 2: sincronizados contratos 5C-5F, pipeline/resumen,
PIPELINE_MAESTRO, contexto minimo, estado, ultima sesion, docs de paquetes e
historiales largos/resumen/indice 5C-5F. Intentos 226-230 conservados.
Preparacion bloque 2: CERRADA.
Acuerdo cerrado bloque 2: si.
Autorizacion funcional bloque 2: CONCEDIDA y consumida.
Trabajo activo bloque 2: no; queda analisis/revision humana de calidad W.
Dudas abiertas bloque 2: ninguna de implementacion; interpretacion por revision
y convencion angular pendientes.
Siguiente accion exacta Fase 5: revisar con el usuario la prueba 230 y separar
la evidencia por revision/optimizacion antes de proponer correcciones; no
iniciar 5G.
Revision conversada prueba 230: el usuario recuerda correctamente que existe
deriva y que el error GT->W se reduce mediante optimizaciones sucesivas, no
tiene que ser cero en cada muestra. El MAE publicado agrega todas las muestras
autoritativas de varias revisiones y no aisla ventanas post-optimizacion ni la
revision final; por tanto no demuestra por si solo incoherencia global. W
mejora posicion frente a O en ambos drones. El error angular cercano a 2 rad
queda como indicio a contrastar con convenciones body/camera/GT, no como fallo
atribuido ya al optimizador. Conclusion 5F permanece PARCIAL por analisis
incompleto y puerta humana pendiente. Siguiente accion: segmentar la evidencia
230 por revision/optimizacion y validar frames antes de proponer correcciones;
no iniciar 5G.
Preparacion ampliacion visual 5F: EN_DEBATE.
Acuerdo parcial visual 5F: mostrar en RViz2 un sistema de referencia XYZ
ubicado y orientado por `W_T_B` para cada dron, con etiqueta `drone_1` o
`drone_2`; usar el frame global de los KFs y actualizarlo con las revisiones.
Visible solo con `global_valid=true`; ocultar al perder autoridad. Sin GT como
entrada visual, TF `world -> body`, trail, smoothing ni cambios de control.
Acuerdo cerrado ampliacion visual 5F: no; falta preparar ownership tecnico,
prueba y criterios detallados con el usuario.
Autorizacion funcional ampliacion visual 5F: PENDIENTE.
Prueba acordada ampliacion visual 5F: pendiente de preparacion.
Dudas abiertas ampliacion visual 5F: ownership del publisher/etiqueta y prueba
integrada exacta.
Siguiente accion exacta ampliacion visual 5F: verificar la infraestructura
RViz vigente y presentar implementacion, prueba y criterios antes de editar
codigo, launch o configuracion.
Preparacion tecnica ampliacion visual 5F: RViz usa frame fijo `world` y ya
muestra `/global_keyframes` como `MarkerArray`. Ownership recomendado:
`simulacion_dron` incorpora un visualizador independiente que consume
`/dron_X/orbslam/navigation_state` y publica ejes X/Y/Z mas texto en un
`MarkerArray`; asi puede emitir `DELETE` al pasar `global_valid=false` y evita
dependencias RViz en el wrapper. La documentacion provisional de `PoseStamped`
debe reconciliarse con este transporte si el usuario acepta la propuesta.
Hallazgo de prueba: la trayectoria tipica de rodeo vigente usa goals absolutos,
que permanecen rechazados por acuerdo hasta 5H. La variante relativa 226 ya
mostro perdida de tracking del dron 2 y rechazo de goals posteriores antes del
fallback de 5G. Ejecutar la vuelta completa exige elegir explicitamente entre
adelantar fallback/semantica futura, crear una excepcion GT de prueba o aceptar
que la prueba visual pueda terminar al perder tracking. Autorizacion funcional
ampliacion visual 5F: PENDIENTE; no modificar ni simular hasta cerrar esta
decision con el usuario.
Alternativa de prueba recomendada: tras un posicionamiento/anclaje relativo,
enviar a cada dron un unico goal `GenTrayElipse` relativo alrededor del
edificio. El goal completo queda aceptado mientras ORB es valido y el control
GT legacy puede terminarlo aunque tracking se pierda, como en 230, sin fallback,
bypass absoluto ni semantica 5H. El generador actual solo recorre el sentido
positivo; dos sentidos exigirian ampliar `lib_tray`, fuera del alcance visual
minimo. Pendiente confirmar con el usuario vuelta en el mismo sentido a alturas
separadas o ampliar el generador.
Decision posterior del usuario: rechaza la alternativa eliptica y exige usar
exactamente
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
acepta el resto de la propuesta visual `MarkerArray`.
Auditoria del gate tras esa decision: `NavigationGoalPolicy` acepta hoy un
goal absoluto cuando `global_valid=true`, y su test lo exige, aunque 5E y el
acuerdo conversado ordenan mantener absolutos deshabilitados hasta 5H. No hay
todavia `W_T_O` ni conversion world->O; `gen_tray` ejecutaria el target absoluto
mediante GT legacy. La trayectoria exacta podria funcionar tras anclaje, pero
aprovecharia una discrepancia funcional no acordada. Autorizacion funcional:
SUSPENDIDA hasta decidir entre corregir el gate y no poder ejecutar ese YAML,
o habilitar un harness GT de simulacion explicito, default false y exclusivo de
esta prueba. Duda abierta unica: aceptar o rechazar ese harness temporal.
Confirmacion del usuario ampliacion visual 5F: acepta el harness GT temporal y
ordena ejecutar conforme a la propuesta.
Preparacion ampliacion visual 5F: CERRADA.
Acuerdo cerrado ampliacion visual 5F: si.
Autorizacion funcional ampliacion visual 5F: CONCEDIDA el 2026-08-27.
Prueba acordada ampliacion visual 5F: ejecutar exactamente
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`
con dos drones, RViz2, KFs globales y ejes/etiquetas W; observar movimiento y
recolocacion tras optimizaciones. Harness absoluto GT solo para simulacion,
default false, activado exclusivamente en esta prueba.
Dudas abiertas ampliacion visual 5F: ninguna.
Plan activo ampliacion visual 5F: corregir gate absoluto con parametro explicito
default false y tests; implementar visualizador `MarkerArray` independiente en
`simulacion_dron`, launch/config RViz y tests; compilar `dron_individual` y
`simulacion_dron`; ejecutar la vuelta exacta, reducir logs, preservar revision
visual humana y cerrar documentacion.
Trabajo activo ampliacion visual 5F: implementacion, build, tests, simulacion y
analisis autorizados. No implementar 5G ni 5H.
Siguiente accion exacta ampliacion visual 5F: localizar los puntos de launch y
tests y aplicar el gate parametrico junto al visualizador, sin modificar el
YAML solicitado.
Implementacion ampliacion visual 5F completada: `NavigationGoalPolicy` rechaza
absolutos por defecto y solo los acepta con harness explicito mas global valida;
el parametro se propaga desde `multi_dron` a `gen_tray`. Un nodo independiente
de `simulacion_dron` convierte `NavigationState.w_t_body` autoritativa en tres
flechas XYZ y texto por dron sobre `/global_drone_poses`, y emite DELETE al
invalidarse. RViz incorpora el display; launch y dependencias quedan conectados;
tests deterministas cubren gate, rotacion, colores, etiqueta y borrado. YAML de
rodeo solicitado sin cambios. Verificacion mecanica, Python y
`git diff --check` correctos.
Build ampliacion visual 5F preparado: compilar primero `dron_individual` para
validar gate/harness; despues ejecutar su test enfocado y compilar
`simulacion_dron`.
Siguiente accion exacta ampliacion visual 5F: ejecutar build seleccionado de
`dron_individual` y registrar el resultado antes de continuar.
Build ampliacion visual 5F `dron_individual`: 1/1 correcto, exit 0, 15.2 s.
Log completo conservado en `codex/archivos_auxiliares/colcon_build.log` y no
leido. Siguiente accion: ejecutar `test_navigation_goal_policy`; si pasa,
compilar `simulacion_dron`.
CTest enfocado ampliacion visual 5F `dron_individual`: 1/1 correcto, exit 0;
valida rechazo absoluto default, harness explicito y requisito de global
valida. Siguiente accion exacta: compilar `simulacion_dron`.
Build ampliacion visual 5F `simulacion_dron`: 1/1 correcto, exit 0, 8.70 s;
visualizador, launch, RViz y tests instalados. Log completo conservado y no
leido. Siguiente accion exacta: ejecutar CTest completo de `simulacion_dron`.
CTest ampliacion visual 5F `simulacion_dron` intento 1: 10/12 correctos, exit 8.
Los dos pytest que importan `NavigationState` no llegaron a coleccionarse porque
el comando CTest no tenia cargado el overlay Dron (`ModuleNotFoundError:
orbslam3_msgs`); tambien afecta al test de metricas previamente validado. No es
fallo funcional. Siguiente accion exacta: repetir la suite completa cargando
explicitamente ROS y los overlays Dron/Servidor/Simulacion.
CTest ampliacion visual 5F `simulacion_dron` intento 2: 12/12 correctos, exit
0, 9.65 s, incluidos visualizador, metricas, contratos y linters.
Precondicion de trayectoria: spawn normal aleatorio no garantiza autoridad
antes del primer goal absoluto. Se reutiliza el override F5B ya validado
(`x=-1/+1`, `y=-10.8`, yaw 90 grados) para iniciar frente al fiducial 2 y
permitir anclaje durante la espera inicial del YAML, sin modificarlo.
Simulacion ampliacion visual 5F preparada como prueba 231: YAML exacto
`/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
`multi_dron.launch.py` con Gazebo headless, RViz2, visualizador W, metricas,
logs F3/F5, harness absoluto legacy y spawn F5B; GUI de mision desactivada.
Startup 2 s, timeout 1200 s, post 35 s, un reintento Gazebo y monitor de
recursos. Patrones: `F5F-RVIZ|F5B-GOAL|F5E|F5D|SCENARIO-RUNNER|F3E-FID|F3O|F3Q|ERROR|FATAL`.
Siguiente accion exacta ampliacion visual 5F: ejecutar prueba 231, registrar
exit/success/log antes de reducir o analizar.
Prueba 231 ampliacion visual 5F: ejecucion real fallida antes de Gazebo, exit 1,
`success=false`; ambos intentos de launch terminaron anticipadamente. Guarda de
recursos inactiva, minimo MemAvailable 7479.1 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_231.log` y no leido.
Siguiente accion exacta ampliacion visual 5F: reducir 231 por launch, Python,
harness, visualizador, RViz y errores; diagnosticar el primer fallo real antes
de corregir o repetir.
Diagnostico prueba 231: fallo mecanico unico confirmado en reducido; el symlink
instalado de `global_drone_pose_visualizer.py` existe, pero el fuente nuevo no
tiene bit ejecutable y ROS 2 lo clasifica como ausente del libexec. Correccion
autorizada: marcar el script ejecutable, repetir build/CTest de
`simulacion_dron` y ejecutar una prueba nueva 232 con identico YAML/launch.
Siguiente accion exacta: aplicar permiso ejecutable y rebuild seleccionado.
Build correctivo prueba 232 `simulacion_dron`: 1/1 correcto, exit 0, 0.69 s;
el ejecutable instalado conserva ahora el permiso correcto. Siguiente accion:
repetir CTest completo antes de simular 232.
CTest correctivo prueba 232 `simulacion_dron`: 12/12 correctos, exit 0, 9.65 s.
`ros2 pkg executables` confirma `global_drone_pose_visualizer.py` disponible.
Simulacion 232 preparada con YAML, launch, spawn, timeout, post-wait, monitor y
patrones identicos a 231; solo cambia el numero de prueba para conservar el
fallo anterior. Siguiente accion exacta: ejecutar prueba 232 y registrar su
resultado antes de reducir.
Prueba 232 ampliacion visual 5F: ejecucion real iniciada con Gazebo/RViz, pero
scenario runner termino exit 1 y `success=false` tras unos 23 s; se mantuvo la
ventana 35 s y el cierre total duro 58 s. Guarda inactiva, minimo MemAvailable
5968.7 MiB, max ORB PSS 978.0 MiB y max RViz PSS 127.4 MiB. Log completo en
`codex/archivos_auxiliares/logs/prueba_232.log`, no leido.
Siguiente accion exacta: reducir 232 por F5F-RVIZ, gate, scenario, autoridad,
tracking y errores para identificar el primer rechazo antes de modificar.
Diagnostico prueba 232: visualizador listo; dron 2 detecta fiducial, recibe
revision 1, muestra ejes W y acepta el primer absoluto con harness. Dron 1
mantiene tracking local pero no obtiene anchor durante los 16 s y rechaza el
primer goal con `reject_global_invalid`, por lo que el scenario falla en paso
3. No hay fallo de visualizador ni gate. Prueba 233 preparada con el mismo YAML
y launch funcional, acercando spawn a `y=-10.0` y stagger 0 para que ambos
tengan toda la ventana inicial frente al fiducial; no se modifica trayectoria,
harness ni criterio. Siguiente accion: ejecutar 233 y registrar resultado.
Prueba 233 ampliacion visual 5F: ejecucion real fallida, scenario exit 1 y
`success=false`; mismo YAML exacto, spawn `y=-10.0` y stagger 0. Cierre en 61 s,
guarda inactiva, minimo MemAvailable 6009.5 MiB, max ORB PSS 960.1 MiB y RViz
PSS 118.1 MiB. Log completo en `logs/prueba_233.log`, no leido.
Siguiente accion exacta: reducir 233 por autoridad, fiducial, gate, visualizador
y scenario; confirmar si se repite la falta de global antes de decidir.
Diagnostico prueba 233: ambos drones tienen tracking local continuo, pero
ninguno detecta fiducial ni obtiene autoridad estando quieto; ambos rechazan el
paso 3 con `reject_global_invalid`. El primer goal del YAML es precisamente el
movimiento que permite llegar/anclarse, por lo que exigir global previa en el
harness hace imposible ejecutar exactamente esa trayectoria. Hallazgo mecanico
adicional: el visualizador sufre una carrera de shutdown al invalidarse ROS,
equivalente a la ya corregida en metricas.
Autorizacion funcional ampliacion visual 5F: SUSPENDIDA. Decision necesaria:
permitir que el harness GT, solo cuando su parametro explicitamente true,
acepte absolutos con estado local fresco/continuo aunque global aun sea
invalida; default false sigue rechazando y GT no entra en estimacion. Tras el
primer movimiento, ejes aparecen solo al anclarse como ya se acordo.
Siguiente accion exacta: esperar confirmacion del usuario; despues corregir
policy/test y lifecycle, rebuild/CTest y repetir el YAML exacto como nueva
prueba. No iniciar 5G/5H.
Decision del usuario tras prueba 233: para esta prueba no usar la pose relativa
ni `NavigationState` para gobernar el movimiento. Activar un modo legacy GT
explicito que acepte goals relativos y absolutos sin el gate 5B y genere/siga
la trayectoria solo desde `sensor/GT/pose` y `sensor/GT/vel`, como hacia el
control previo a 5B. El estimador sigue ejecutandose en paralelo sin recibir GT;
RViz muestra exclusivamente `W_T_B` y solo mientras `global_valid=true`.
Preparacion de la correccion 5F: CERRADA.
Acuerdo cerrado de la correccion 5F: si.
Autorizacion funcional de la correccion 5F: CONCEDIDA el 2026-08-27.
Prueba acordada: repetir exactamente
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`
con el modo legacy GT activo, RViz2, KFs globales y ejes estimados; no modificar
el YAML ni implementar navegacion 5G/5H.
Dudas abiertas: ninguna.
Trabajo activo: ajustar policy/test/launch, corregir lifecycle del visualizador,
rebuild/CTest y ejecutar la prueba 234 conservando 231-233.
Siguiente accion exacta: aplicar los cambios funcionales acordados en
`dron_individual` y la correccion mecanica de shutdown en `simulacion_dron`.
Cambios funcionales de la correccion 5F completados: el parametro pasa a
`use_legacy_gt_goal_policy_for_simulation`; cuando esta activo, la policy acepta
goals relativos/absolutos antes de consultar `NavigationState` y no congela
snapshot estimado. El modo normal conserva gate 5B y absolutos deshabilitados.
El visualizador maneja `ExternalShutdownException` y solo relanza excepciones
si el contexto ROS sigue valido. Launches, tests, contrato 5F y docs vigentes de
`dron_individual`/`simulacion_dron` sincronizados.
Siguiente accion exacta: compilar `dron_individual`, ejecutar su test enfocado,
compilar `simulacion_dron` y ejecutar CTest completo con overlays.
Build correccion GT 5F `dron_individual`: 1/1 correcto, exit 0, 15.5 s. Log
completo conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido.
Siguiente accion exacta: ejecutar `test_navigation_goal_policy`; si pasa,
compilar `simulacion_dron`.
CTest policy intento 1: no ejecutado, exit 8 porque el sandbox impidio escribir
`Testing/Temporary/LastTest.log` bajo `build/dron`; no es fallo del test.
Siguiente accion exacta: repetir el mismo CTest con permiso de escritura en el
directorio de build.
CTest policy intento 2: 1/1 correcto, exit 0; valida que el modo normal exige
estado local y mantiene absolutos deshabilitados, mientras el modo GT legacy
acepta relativos y absolutos aun sin `NavigationState`.
Siguiente accion exacta: compilar `simulacion_dron` para instalar launch y
visualizador corregidos.
Build correccion GT 5F `simulacion_dron`: 1/1 correcto, exit 0, 0.67 s. Log
completo conservado y no leido. Siguiente accion exacta: ejecutar CTest completo
con ROS y overlays Dron/Servidor/Simulacion cargados.
CTest correccion GT 5F `simulacion_dron`: 12/12 correctos, exit 0, 10.31 s;
incluye visualizador, metricas, contratos y linters.
Siguiente accion exacta: preparar prueba 234 con copia byte a byte del YAML
tipico solicitado, launch visual RViz2, modo GT legacy activo y spawn normal.
Prueba 234 preparada: YAML exacto
`/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`
pasado directamente con `--yaml`, sin copia ni modificacion. Launch
`multi_dron.launch.py` headless Gazebo, GUI de mision desactivada, RViz2 sparse
y ejes W activos, metricas en `metricas/prueba_234`, logs F3 y
`use_legacy_gt_goal_policy_for_simulation=true`; spawn normal y stagger default
8 s. Startup 2 s, timeout 1200 s, post 35 s, un reintento y monitor de recursos.
Patrones de reduccion: `F5F-RVIZ|F5B-GOAL|F5E|F5D|SCENARIO-RUNNER|F3E-FID|F3O|F3Q|ERROR|FATAL`.
Siguiente accion exacta: ejecutar prueba 234 y registrar exit/success/log antes
de reducir o analizar.
Prueba 234: ejecucion real correcta, scenario exit 0, `success=true` y runner
exit 0; duracion monitorizada 514 s incluida ventana post 35 s. Guarda inactiva,
minimo MemAvailable 5224.6 MiB, max ORB PSS 1321.5 MiB y max RViz PSS
144.3 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_234.log` y no leido.
Siguiente accion exacta: reducir 234 con los patrones acordados y analizar solo
el reducido junto con los artefactos de metricas.
Analisis y revisión humana prueba 234: 17/17 pasos y 22/22 goals absolutos
aceptados por política GT legacy, sin rechazos; ambos drones publican W y
revisiones. El usuario confirma que las poses estimadas siguen el camino de KFs
y se corrigen visualmente al optimizar. Los defectos observados en las
optimizaciones se atribuyen a Fase 3 y quedan fuera del alcance actual.
Defecto visual confirmado: parpadeo frecuente sin pérdida real. En cada cambio
de reference KF, `NavigationStateEstimator::Update()` conserva `W_T_B` continua
pero pasa de `AUTHORITATIVE` a `PROVISIONAL` y revisión 0 mientras consulta la
nueva autoridad. `global_valid` exige `AUTHORITATIVE`; el visualizador interpreta
el provisional como ausencia y emite DELETE, produciendo el parpadeo. Prueba 234
registra múltiples alternancias dentro del mismo epoch y tracking válido; no son
pérdidas ORB.
Preparacion correccion antiparpadeo 5F: EN_DEBATE.
Acuerdo cerrado: no.
Autorizacion funcional: PENDIENTE.
Propuesta: representar `PROVISIONAL` y `AUTHORITATIVE` siempre que local y
continuidad sean válidas; ocultar solo `INVALID`, pérdida real o epoch nuevo sin
anchor. No añadir smoothing ni retener una pose congelada tras pérdida.
Dudas abiertas: decidir si distinguir visualmente el estado provisional del
autoritativo o mantener los mismos ejes y dejar que solo su recolocación muestre
la llegada de autoridad.
Siguiente accion exacta: explicar el diagnóstico y cerrar con el usuario la
presentación provisional/autoritativa antes de editar código.
Confirmacion del usuario correccion antiparpadeo 5F: mostrar con igual apariencia
W `PROVISIONAL` y `AUTHORITATIVE`; ocultar solo `INVALID`, perdida real o epoch
sin anchor. Preparacion: CERRADA. Acuerdo cerrado: si. Autorizacion funcional:
CONCEDIDA el 2026-08-27. Prueba acordada: repetir exactamente la prueba 234 como
235, sin analizar resultados. Dudas abiertas: ninguna.
Siguiente accion exacta: editar visualizador/test/documentacion, compilar y
ejecutar CTest de `simulacion_dron`, y ejecutar prueba 235.
Correccion antiparpadeo aplicada: visibilidad con local/continuidad validas y W
distinta de `INVALID`; test provisional añadido. Build `simulacion_dron` 1/1
correcto, exit 0, 0.69 s. Siguiente accion: CTest completo.
CTest `simulacion_dron`: 12/12 correctos, exit 0, 9.76 s. Prueba 235 preparada
idéntica a 234: YAML típico exacto, control GT legacy, spawn normal, RViz2 y
ejes W; startup 2 s, timeout 1200 s, post 35 s y monitor. Siguiente accion:
ejecutar prueba 235 sin análisis posterior.
Prueba 235: exit 0, `success=true`, scenario exit 0, guarda inactiva y minimo
MemAvailable 5358.8 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_235.log` y no analizado por peticion del
usuario. Trabajo activo: no; revision visual corresponde al usuario.
Preparacion bloque 3 Fase 5G+5H: EN_DEBATE.
Acuerdo cerrado: no.
Autorizacion funcional: PENDIENTE.
Peticion nueva del usuario: retirar completamente GT del control, usar la pose
estimada para la misma trayectoria tipica y mostrar simultaneamente Gazebo GUI
y RViz2. Esto contradice la decision anterior de mantener `GT_FALLBACK` hasta
Fase 6 y requiere confirmacion explicita.
Diseño propuesto: 5G estima velocidad lineal/angular desde `O_T_B`; 5H migra
`gen_tray` y `control_calcular_fuerzas` a `NavigationState`, elimina sus
suscripciones GT, ejecuta relativos en O y convierte absolutos W->O al aceptar
con `W_T_O` autoritativa congelada. Sin smoothing, replanning ni cambios de
ganancias no justificados.
Riesgos abiertos: sin fallback una perdida ORB invalida pose/velocidad y puede
interrumpir la misión; el primer goal del YAML es absoluto y no puede aceptarse
antes del primer anchor. Bootstrap recomendado: spawn determinista cerca del
fiducial 2 y breve movimiento relativo en O para crear KFs/anclar, seguido del
YAML exacto. Debe decidirse tambien la reaccion de seguridad al perder estado y
si GT permanece solo como metrica externa o se desactiva incluso para metricas.
Prueba propuesta: unitarias por velocidad/frames/goals/perdida, builds aislados
y simulacion multi-dron con Gazebo GUI+RViz2, preambulo relativo de bootstrap y
despues YAML tipico exacto; registrar si termina o donde falla, sin atribuir a
Fase 5 errores ya conocidos del optimizador 3Q.
Dudas abiertas: retirada definitiva del fallback; bootstrap relativo previo;
reaccion al perder tracking; alcance de GT externo de metricas.
Siguiente accion exacta: cerrar estas cuatro decisiones con el usuario antes de
editar contratos, codigo, launch o configuracion.
Revision conversada bloque 3: el usuario mantiene `GT_FALLBACK`. GT entra al
perder tracking y sostiene la trayectoria hasta que el submapa actual se ancle
por fiducial o loop; fuera de pérdida no se usa. `gen_tray` y control consumen
una interfaz común estimada/fallback, no topics GT directos durante operación
normal. La reacción de seguridad sin GT deja de aplicar porque la misión
continúa mediante fallback.
Interpretacion propuesta de salida: abandonar fallback solo cuando coincidan
tracking ORB recuperado y autoridad global del `map_epoch` actual; un anchor sin
tracking o tracking recuperado sin anchor no bastan. El cambio a ORB debe
realinearse con O sin salto.
Bootstrap explicado: spawn determinista cerca del fiducial 2, breve barrido
relativo en O para crear KFs y obtener anchor, y después ejecutar sin modificar
el YAML típico cuyo primer goal es absoluto. No usa GT para control.
Preparacion bloque 3: EN_DEBATE. Acuerdo cerrado: no. Autorizacion: PENDIENTE.
Dudas abiertas: confirmar la condición conjunta ORB_OK+anchor para salir del
fallback; aceptar el bootstrap relativo; decidir timeout si nunca llega anchor.
Siguiente accion exacta: recibir estas confirmaciones antes de editar MDs o
iniciar implementación.
Acuerdo final conversado bloque 3: el fallback se usa tanto ante pérdida ORB
como al arrancar una tarea absoluta sin `W_T_O`/anchor. Control estimado solo
cuando coinciden ORB `OK` y epoch actual anclado; si falta cualquiera para la
tarea absoluta, `pose_source=GT_FALLBACK`. Si nunca llega anchor, GT continúa
hasta terminar la misión. GT queda prohibido fuera de ese modo y no alimenta
mapa, KFs, anchors, loops, optimización ni W.
No habrá bootstrap relativo ni modificación del YAML: se ejecuta directamente
la vuelta típica absoluta. No se exige transición sin salto; el salto se mide y
se acepta como deuda de Fase 6. La regresión final absoluta basta para validar
el bloque; movimiento relativo recibe tests deterministas, pero su primera
prueba integrada queda en Fase 6 y cualquier fallo atribuible se corrige
reabriendo 5H.
Prueba final acordada: dos drones, YAML exacto
`prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`, Gazebo GUI y RViz2 visibles,
modo GT legacy 234/235 desactivado, métricas de tiempo ORB/fallback y logs de
fuente. Se acepta que defectos de optimización 3Q afecten la ejecución sin
corregirlos dentro de este bloque.
Alcance: estimar velocidad desde O; integrar una interfaz común ORB/fallback;
retirar GT directo de `gen_tray` y `control_calcular_fuerzas`; goals absolutos
W->O cuando haya autoridad y continuidad GT explícita cuando no la haya;
actualizar ADR 0002, launch/config, arquitectura, tests y documentación.
Exclusiones: recovery/replanning de Fase 6, suavizado de transición, cambios de
ganancias para ocultar errores, correcciones del optimizador de Fase 3 y prueba
Gazebo relativa completa.
Preparacion bloque 3 Fase 5G+5H: CERRADA.
Acuerdo cerrado: si.
Autorizacion funcional: PENDIENTE.
Dudas abiertas: ninguna.
Siguiente accion exacta: esperar una orden posterior explícita para actualizar
los contratos acordados o ejecutar el bloque; no modificar código ni simular.
Autorizacion documental bloque 3: CONCEDIDA el 2026-08-27 para reconciliar
exclusivamente `subfase_5G.md` y `subfase_5H.md` con el acuerdo conversado. No
autoriza codigo, launch, configuracion, build ni simulacion.
Siguiente accion exacta: editar ambos contratos y verificar `git diff --check`.
Contratos 5G/5H reconciliados: fallback inicial absoluto, pérdida y epoch nuevo
con causas separadas; salida ORB `OK`+anchor; permanencia hasta fin si no ancla;
sin bootstrap ni requisito de transición suave; tests relativos deterministas;
vuelta YAML exacta con Gazebo GUI+RViz2; deudas de Fase 6 explícitas. Ambos MD
quedan cortos (133/148 líneas) y `git diff --check` es correcto.
Autorizacion documental: consumida. Autorizacion funcional: PENDIENTE.
Trabajo activo: no. Dudas abiertas: ninguna.
Siguiente accion exacta: esperar orden explícita de ejecución del bloque 5G+5H.
Autorizacion funcional bloque 3 Fase 5G+5H: CONCEDIDA el 2026-08-27 por orden
explícita del usuario. Preparacion CERRADA, acuerdo cerrado si, prueba definida
y dudas ninguna.
Plan activo: auditar interfaces; implementar velocidad/fallback común;
migrar `gen_tray` y `control_calcular_fuerzas`; actualizar launch/config/ADR y
arquitectura; builds y tests selectivos; ejecutar YAML exacto con Gazebo GUI y
RViz2; reducir logs y documentar 5G/5H.
Paquetes previstos: `orbslam3_ros2`, `dron_individual`, `simulacion_dron`; solo
ampliar `orbslam3_msgs` si la causa de fallback no puede representarse sin
cambiar contrato.
Siguiente accion exacta: localizar desde docs y símbolos actuales el productor
de `NavigationState`, callbacks GT, consumidores de control y tests antes del
primer cambio funcional.
Checkpoint de reanudacion bloque 3 Fase 5 2026-08-27: memoria fisica releida y
reconciliada con la orden vigente. Preparacion CERRADA, acuerdo cerrado si,
autorizacion funcional CONCEDIDA, prueba acordada definida y dudas ninguna.
Archivos criticos 5G+5H localizados: productor en
`dron/orbslam3_ros2/src/stereo/stereo-slam-node.cpp`; consumidores GT en
`dron/dron_individual/src/control_tray/gen_tray.cpp` y
`control_calcular_fuerzas.cpp`; lanzamiento en `generar_dron.launch.py` y
`simulacion_dron/launch/multi_dron.launch.py`; contrato existente en
`NavigationState.msg` y tests de politica en `dron_individual/test`.
Diseno mecanico cerrado: el wrapper se remapea a
`orbslam/navigation_state_orb`; un nuevo mux de `dron_individual` sera el unico
consumidor de GT y publicara `orbslam/navigation_state`. El mux enclava anchor
por epoch, distingue en logs startup absoluto, tracking perdido y epoch nuevo,
sale solo con ORB OK+anchor, alinea el frame local continuo al cambiar de fuente
y deriva velocidad lineal/angular de la pose final. `gen_tray` y control
consumiran exclusivamente el estado comun; goals absolutos se transformaran
W->O con la autoridad disponible y el fallback inicial conserva la convencion
O=W. No se amplian mensajes: `pose_source` aporta la metrica de fuente y la
causa detallada queda en logs. Siguiente accion exacta: implementar helper,
nodo mux y tests deterministas; despues migrar ambos consumidores y launches.
Bloque funcional implementado 5G+5H: creados helper, nodo
`navigation_state_mux` y cuatro regresiones de fuente/anchor/continuidad/
velocidad. El mux es el unico consumidor operativo GT, alinea el frame local al
cambiar ORB/fallback, publica velocidad derivada y metricas de fuente.
`gen_tray` ya consume pose/velocidad comunes, acepta absoluto con autoridad o
fallback y congela W->O; `control_calcular_fuerzas` consume exclusivamente el
estado comun con guarda de frescura. El wrapper se remapea al topic ORB crudo,
los launches arrancan el mux y se retiro el bypass legacy 234/235. Metricas 5F
ampliadas con conteos/ratio ORB-fallback. `git diff --check` correcto.
Build preparado: compilar `dron_individual` con
`./codex/herramientas/build_selected_packages.sh --group dron dron_individual`
para validar helper, nodos y tests contra las interfaces ya instaladas.
Siguiente accion exacta: ejecutar ese build y registrar inmediatamente el
resultado antes de diagnosticar o continuar.
Build 5G+5H `dron_individual`: 1/1 correcto, exit 0, 39.8 s. Log completo en
`codex/archivos_auxiliares/colcon_build.log`, no leido. Siguiente accion exacta:
ejecutar CTest de `dron_individual` para validar politica de goals y regresiones
del mux antes de compilar `simulacion_dron`.
CTest `dron_individual`: tests funcionales 2/2 correctos
(`test_navigation_goal_policy`, `test_navigation_state_mux`) y 3 linters
estructurales correctos; suite agregada 5/8 por deuda legacy global ya conocida
en 26 scripts de vision (flake8/pep257) y tres archivos C++ legacy
(uncrustify). Los cinco archivos C++ tocados y ambos tests nuevos constan sin
divergencia de estilo; no se modificara deuda ajena. Siguiente accion exacta:
compilar `simulacion_dron` para instalar el launch y las metricas de fuente, y
despues ejecutar su CTest.
Build 5G+5H `simulacion_dron`: 1/1 correcto, exit 0, 0.69 s. Log completo en
`codex/archivos_auxiliares/colcon_build.log`, no leido. Siguiente accion exacta:
ejecutar CTest completo de `simulacion_dron` y corregir solo fallos mecanicos
atribuibles al bloque.
CTest `simulacion_dron` intento 1: 9/12 correctos, exit 8. Dos tests Python no
se recolectaron porque la invocacion directa no cargo el prefijo Dron y no
encontro `orbslam3_msgs`; es un problema de entorno de prueba. El contrato de
arquitectura detecto un fallo real y mecanico: aun exige que `gen_tray` emita
la arista GT retirada, que ahora pertenece al nuevo mux. Siguiente accion
exacta: actualizar topologia/contrato de arquitectura para representar
GT->mux->estado comun y repetir CTest con los prefijos ROS cargados.
Correccion mecanica CTest aplicada: `sim_to_dron_gt` apunta a
`navigation_state_mux`, su metadata describe solo PoseStamped de fallback, el
mux emite la telemetria de esa arista y el contrato impide reintroducir GT en
`gen_tray`. `git diff --check` y `py_compile` de launches/metricas correctos.
Build preparado: recompilar `dron_individual` y `simulacion_dron`; despues
repetir CTest de Simulacion cargando `/opt/ros/iron`, Dron, Servidor y
Simulacion. Siguiente accion exacta: ejecutar ambos builds seleccionados en ese
orden y registrar cada resultado.
Rebuild mecanico `dron_individual`: 1/1 correcto, exit 0, 14.4 s. Siguiente
accion exacta: recompilar `simulacion_dron` para instalar topologia y contrato
actualizados.
Rebuild mecanico `simulacion_dron`: 1/1 correcto, exit 0, 0.68 s. Log completo
de ultimo build conservado y no leido. Siguiente accion exacta: repetir CTest
de `simulacion_dron` con los cuatro prefijos cargados para distinguir la
correccion real de los fallos previos de entorno.
CTest `simulacion_dron` intento 2 con overlays: 12/12 correctos, exit 0, 9.46 s;
quedan validados topologia, metricas, visualizador y linters. Prueba 236
preparada con el YAML exacto
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`
pasado directamente por `--yaml`: dos drones, `launch_gazebo_gui=true`, GUI de
mision desactivada, `debug_sparse_global_rviz=true`, ejes W activos, metricas
en `codex/archivos_auxiliares/metricas/prueba_236`, logs F3 visibles y sin
parametro/bypass GT legacy. Startup 2 s, timeout 1200 s, post 35 s, un reintento
y monitor de recursos. Patrones previstos: `F5H-POSE-SOURCE|F5H-SOURCE-METRICS|F5H-REPORT|F5H-GOAL|F5B-GOAL|F5E|F5D|SCENARIO-RUNNER|F3E-FID|F3O|F3Q|ERROR|FATAL`.
Siguiente accion exacta: ejecutar prueba 236 y registrar exit/success/log antes
de reducir o analizar.
Prueba 236 intento 1: terminada, exit 1; scenario exit 1 antes de comenzar la
trayectoria, duracion monitorizada 43 s, guarda inactiva y minimo MemAvailable
5903.2 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_236.log` y no leido. No es aun una
conclusion funcional. Siguiente accion exacta: reducir 236 con los patrones
acordados y marcadores de runner/launch para diagnosticar el primer error real
sin abrir el log completo.
Diagnostico reducido prueba 236: `SCENARIO-RUNNER-ERROR` no pudo cargar el YAML
porque se paso una ruta relativa desde un proceso cuyo cwd no es `src`; no se
envio ningun goal. Mux, Gazebo y RViz2 si arrancaron: ambos drones publicaron
`startup_unanchored_absolute` y metricas 100% fallback, coherente antes de
anclar. Correccion puramente mecanica: repetir como prueba 237 pasando el mismo
archivo exacto mediante ruta absoluta, sin cambiar codigo, YAML, launch,
criterio ni comportamiento. Siguiente accion exacta: ejecutar prueba 237 con
los mismos argumentos visuales/timeout y registrar el resultado antes de
reducir.
Prueba 237: ejecucion real terminada, scenario exit 0, `success=true`, runner
exit 0 y duracion monitorizada 491 s incluida ventana post 35 s. Gazebo GUI y
RViz2 se lanzaron; guarda inactiva, minimo MemAvailable 5623.0 MiB. Log completo
conservado en `codex/archivos_auxiliares/logs/prueba_237.log` y no leido.
Siguiente accion exacta: reducir 237 con los patrones acordados y analizar solo
el reducido junto con `metricas/prueba_237/summary.json`.
Analisis reducido prueba 237: 17/17 pasos y 22/22 goals absolutos aceptados y
finalizados, sin rechazos ni fallback no disponible. Se observaron las tres
causas acordadas y retornos a ORB: startup en ambos drones, tracking_lost en
ambos y new_epoch_unanchored en dron 2. La fuente ORB fue muy breve: dron 1
16/9164 muestras y fallback 9148 (99.8254%); dron 2 30/9006 y fallback 8976
(99.6669%). La mision continua queda validada, pero el fallback domina porque
ORB vuelve a RECENTLY_LOST poco despues de cada anchor. Las metricas W muestran
errores ya atribuidos a Fase 3 y no se corregiran aqui. Gazebo reporto exit 255
solo durante cleanup posterior a `SIM-DONE`, conocido y no funcional.
Huecos contractuales detectados antes del cierre: falta el interruptor explicito
de fallback default false fuera del perfil Fase 5 y faltan metricas externas de
velocidad. Siguiente accion exacta: añadir el parametro, activarlo en el perfil
multi-dron, ampliar `pose_metrics_node` con error de velocidad y sus tests;
despues recompilar y validar antes de decidir si hace falta repetir la vuelta.
Huecos contractuales corregidos: `navigation_state_mux` declara
`gt_fallback_enabled=false`; `generar_dron.launch.py` conserva ese default y el
perfil temporal `multi_dron.launch.py` lo activa explicitamente. Las metricas
externas emparejan tambien GT velocity y calculan error lineal/angular en world
tras alinear O. Contrato de launch y conteo de suscripciones actualizados;
`py_compile` y `git diff --check` correctos. Builds preparados:
`dron_individual` y `simulacion_dron` en sus grupos aislados. Siguiente accion
exacta: ejecutar ambos builds, CTest funcional/Simulacion y decidir la
regresion final sobre el codigo instalado.
Build final `dron_individual`: 1/1 correcto, exit 0, 12.3 s. Siguiente accion
exacta: compilar `simulacion_dron` con metricas/launch finales.
Build final `simulacion_dron`: 1/1 correcto, exit 0, 0.67 s. Log completo del
ultimo build conservado y no leido. Siguiente accion exacta: ejecutar los dos
GTest funcionales de `dron_individual` y CTest completo de `simulacion_dron`
con overlays.
GTest finales `dron_individual`: 2/2 correctos, exit 0, 0.15 s. Siguiente
accion exacta: ejecutar CTest completo de `simulacion_dron` con overlays.
CTest final `simulacion_dron`: 12/12 correctos, exit 0, 9.77 s. Aunque el camino
habilitado del mux no cambio respecto a 237, el interruptor y las metricas se
anadieron despues de esa ejecucion. Prueba final 238 preparada con el mismo YAML
absoluto exacto, Gazebo GUI+RViz2, `gt_fallback_enabled=true` explicito,
metricas en `metricas/prueba_238`, startup 2 s, timeout 1200 s, post 35 s y
monitor. Siguiente accion exacta: ejecutar 238 y registrar su resultado antes
de reducir.
Prueba 238 detenida por Codex tras observacion visual del usuario: al anclarse,
los dos drones divergieron en direcciones distintas y despues desaparecio toda
pose de dron en RViz2. La ejecucion queda `NO CONSEGUIDA`; el resultado visual
invalida cualquier aparente continuidad del runner. El proceso fue interrumpido
y limpio, duracion monitorizada 436 s, guarda inactiva, minimo MemAvailable
5476.1 MiB; no consta `SCENARIO-RUNNER-DONE` y el exit final del wrapper no se
interpreta como éxito funcional. Hipotesis principal propia de 5H: al salir de
fallback se alinea la pose de control, pero `gen_tray` mantiene un setpoint
absoluto congelado con una transformacion W->O distinta, creando un error de
control grande e incoherente por dron; la posterior perdida oculta W en RViz2.
Siguiente accion exacta: reducir 238 alrededor de source transitions, goals,
pose/velocidad y tracking; confirmar la causa sin abrir el log completo y
corregir la coherencia de frame antes de repetir.
Diagnostico reducido 238 confirmado: ambos aceptan el primer goal en fallback;
al recibir autoridad pasan brevemente a ORB y vuelven a tracking_lost. En esos
frames `gen_tray` cachea C_T_W calculada desde W estimada; la metrica final
muestra error angular W alto y distinto (dron 1 ~2.085 rad; dron 2 ~2.100 rad,
yaw especialmente distinto). Los goals siguientes, aceptados ya en fallback,
reutilizan esa transformacion y convierten world en direcciones incoherentes.
La desaparicion RViz2 coincide con tracking perdido/global invalida, no con un
fallo del visualizador. Correccion 5H aplicada: fallback publica GT directamente
en el frame world temporal (salto permitido) y `gen_tray` fuerza W->O identidad
en cada muestra fallback; ORB se alinea al world temporal al entrar. Regresion
determinista anadida para el reset del frame. Siguiente accion exacta: compilar
`dron_individual`, ejecutar sus GTest y repetir la trayectoria visual.
Build correccion frame 5H `dron_individual`: 1/1 correcto, exit 0, 25.6 s. Log
completo conservado y no leido. Siguiente accion exacta: ejecutar GTest de
politica/mux y, si pasan, preparar nueva prueba visual exacta.
GTest correccion frame 5H: 2/2 correctos, exit 0, 0.13 s. Prueba 239 preparada
con el YAML exacto absoluto, Gazebo GUI+RViz2, fallback explicito, metricas en
`metricas/prueba_239`, startup 2 s, timeout 1200 s, post 35 s y monitor. Punto
visual critico: primer anchor y comienzo del segundo goal; ya no debe aplicar
la C_T_W global erronea mientras la fuente sea fallback. Siguiente accion
exacta: ejecutar 239 y registrar resultado/observacion antes de analizar.
Prueba 239 detenida a los 165 s por segunda observacion visual humana: los
drones vuelven a perder el control al llegar/anclarse en fiducial 2, antes de
validar el resto de la vuelta. Conclusión de la prueba: `NO CONSEGUIDA`; la
correccion de target fallback no resolvio el transitorio. El usuario plantea
tres causas: pose estimada mala, controlador `dron_individual` mal configurado o
union GT/estimada incorrecta. El proceso se limpio, guarda inactiva y minimo
MemAvailable 5567.6 MiB. Siguiente accion exacta: reducir 239 alrededor del
primer anchor y comparar discontinuidad de pose, velocidad y fuente; no abrir
el log completo ni repetir hasta aislar la causa.
Diagnostico reducido 239: el control con fallback GT permanece estable y los
errores aparecen justo despues de cada entrada ORB. Dron 1 alcanza durante ORB
1.151 m/1.478 rad de error local y 4.75 m/s/8.17 rad/s de error de velocidad;
dron 2 alcanza 2.467 m/1.559 rad y 4.62 m/s/8.79 rad/s. Las entradas duran solo
decimas o pocos segundos y alternan repetidamente con fallback. Causa principal:
la union GT->estimada admite de inmediato tracking `OK` anclado aunque la
trayectoria ORB aun no sea temporalmente coherente; el controlador estable con
GT reacciona despues a esos saltos y velocidades, pero sus ganancias no son la
causa primaria. Correccion mecanica dentro del acuerdo: mantener fallback
mientras una ventana consecutiva compara el movimiento ORB alineado con GT y
solo conmutar tras superar la cualificacion; no cambiar control ni trayectoria.
Siguiente accion exacta: implementar la cualificacion determinista en
`navigation_state_mux`, ampliar su GTest, compilar `dron_individual` y repetir
la trayectoria visual acordada si las pruebas pasan.
Correccion de transicion 5H implementada: `OrbTransitionQualifier` mantiene GT
durante al menos 20 muestras y exige movimiento observado de 0.10 m o 0.10 rad,
con error ORB alineado frente al GT de fallback no mayor de 0.20 m/0.20 rad.
Una divergencia reinicia la ventana; cada perdida exige recualificar. Anadidos
tres GTest para movimiento coherente, divergencia y estacionario. Las ganancias
y el controlador no se modifican. Siguiente accion exacta: compilar
`dron_individual` con la herramienta seleccionada y registrar el resultado.
Build de cualificacion 5H `dron_individual`: 1/1 correcto, exit 0, 26.1 s. Log
completo conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido.
Siguiente accion exacta: ejecutar el CTest funcional `test_navigation_state_mux`
y registrar el resultado antes de preparar otra simulacion.
CTest de cualificacion 5H: `test_navigation_state_mux` 1/1 correcto, exit 0,
incluidas las tres regresiones nuevas. La prueba siguiente conserva exactamente
la trayectoria absoluta tipica, Gazebo GUI, RViz2, fallback Fase 5 y metricas;
se registrara como prueba 240 para no sobrescribir 239. Siguiente accion exacta:
recuperar los argumentos de ejecucion vigentes del artefacto reducido/script y
registrar el comando antes de lanzar la simulacion larga.
Prueba 240 preparada: YAML absoluto exacto
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`,
startup 2 s, timeout 1200 s, post 35 s, un reintento y monitor de recursos.
Launch: `multi_dron.launch.py` con Gazebo GUI, GUI de mision desactivada, RViz2
sparse/global activo, metricas F5 en `metricas/prueba_240`, logs F3 visibles y
`gt_fallback_enabled=true`. Criterio inmediato: no perder control al anclarse;
la revision visual corresponde al usuario. Siguiente accion exacta: ejecutar
prueba 240 con `run_simulation.sh` y registrar el resultado antes de reducir.
Prueba 240 finalizada: `scenario_runner_node` exit 1 y wrapper exit 1 tras 425 s;
por tanto `NO CONSEGUIDA` como trayectoria completa, sin atribuir aun la causa.
Gazebo permanecio activo, guarda de recursos no disparada y minimo MemAvailable
5247.3 MiB. Log completo en `codex/archivos_auxiliares/logs/prueba_240.log`, que
no se leera directamente; metricas en `metricas/prueba_240`. Siguiente accion
exacta: reducir 240 por runner/goals, cualificacion/fuentes, tracking, autoridad
y errores; distinguir timeout/rechazo de inestabilidad fisica antes de corregir.
Diagnostico 240: la proteccion permite superar el primer anclaje y completar
los pasos 1-15, con 20/20 goals terminados `success=true`; falla solo el primer
envio del paso 16. Dron 2 recibe en ese instante ORB local valido pero una
muestra `global_valid=false` transitoria del ref-KF, y `gen_tray` rechaza el
goal absoluto aunque conserva una transformacion C_T_W valida del mismo epoch.
El posterior abort de `gen_tray` dron 1 ocurre durante cleanup porque su goal
parejo ya habia sido aceptado; no causa el rechazo. Metricas: fallback 99.09 %
y 98.08 %, error O p95 0.150/0.158 m y 0.024/0.033 rad; queda una cola maxima
transitoria, pero ya no impide casi toda la mision. Correccion mecanica: permitir
goal absoluto con la C_T_W cacheada solo si pertenece al epoch actual; no usar
una transformacion de otro epoch ni cambiar la politica de fuentes. Siguiente
accion exacta: ampliar `NavigationGoalState`/`gen_tray` y su GTest, compilar y
repetir la prueba completa.
Correccion de goal final implementada: `NavigationGoalState` distingue
`absolute_frame_valid`; `gen_tray` marca la C_T_W cacheada con su `map_epoch` y
solo la admite durante huecos globales transitorios del mismo epoch. Anadida
regresion de aceptacion con frame cacheado; el rechazo sin pose/frame global se
conserva. Siguiente accion exacta: compilar `dron_individual` y registrar el
resultado antes de ejecutar sus GTest.
Build de frame cacheado 5H `dron_individual`: 1/1 correcto, exit 0, 12.2 s. Log
completo conservado y no leido. Siguiente accion exacta: ejecutar los GTest
`test_navigation_goal_policy` y `test_navigation_state_mux`, registrar el
resultado y preparar prueba 241 solo si pasan.
GTest de frame cacheado/transicion 5H: 2/2 correctos, exit 0, 0.12 s. Prueba
241 preparada identica a 240 salvo directorio de metricas/prueba: YAML absoluto
tipico, Gazebo GUI, RViz2, fallback true, startup 2 s, timeout 1200 s, post 35 s,
un reintento y monitor. Criterio: pasos 1-16 y todos los goals correctos, sin
perdida de control en los anclajes; revision visual del usuario. Siguiente
accion exacta: ejecutar prueba 241 y registrar resultado antes de reducir.
Prueba 241 interrumpida por Codex a peticion conversacional del usuario tras
456 s, exit 130 de la sesion interactiva aunque el cleanup del wrapper muestra
exit 0; no se considera una ejecucion completa ni sustituye 240. Guarda inactiva
y minimo MemAvailable 5185.6 MiB. Observacion humana: Gazebo se bloqueo y no
pudo evaluarse; RViz2 gusto bastante y el movimiento fue medianamente bueno,
pero persisten tramos con movimientos raros que el usuario asocia a cambios
GT->ORB. Ademas el TF/ejes de cada dron parpadea o desaparece y debe mostrar
siempre la pose exacta que entra al controlador. Propuesta funcional nueva del
usuario: no conmutar durante una trayectoria activa; mantener su fuente hasta
que termine el goal y, entre goals, pasar a ORB si sigue anclado/valido.
Autorizacion funcional 5G+5H: SUSPENDIDA por cambio material de politica de
conmutacion. Preparacion adicional: EN_DEBATE. Dudas abiertas: definir la
coordinacion de frontera de goal y el comportamiento si ORB se pierde durante
un goal iniciado con ORB; investigar 241 reducida y el visualizador antes de
cerrar acuerdo. Siguiente accion exacta: reducir 241 por goals/fuentes y revisar
interfaces actuales de `gen_tray`, mux y visualizador, sin modificar codigo.
Investigacion 241 completada: hay conmutaciones GT->ORB dentro de goals activos
(por ejemplo dron 2 a t=55.01 s antes de terminar el primer goal a t=56.08 s,
y nuevas entradas durante movimientos posteriores), seguidas a menudo por
`tracking_lost` 0.6-1.1 s despues. Dron 2 llega durante ORB a 1.557 m, 3.064 rad,
14.41 m/s y 13.48 rad/s de error; confirma la observacion de movimientos raros.
El cualificador previo reduce entradas malas pero no protege una estimacion que
diverge despues de tomar control. El visualizador actual no muestra la entrada
del controlador: usa `w_t_body` y borra los ejes si `global_status` es invalid;
el controlador usa siempre `o_t_body`, de ahi el parpadeo/discrepancia.
Propuesta a acordar: congelar fuente por goal; impedir GT->ORB mientras haya
trayectoria activa; permitir siempre ORB->GT inmediato por perdida o divergencia
de control; recualificar ORB en segundo plano y conmutar solo en frontera de
goal; visualizar `o_t_body` mientras el estado local/control sea valido, con
fuente visible y sin depender de global. Repetir despues la trayectoria con
Gazebo y RViz2, reduciendo carga visual no necesaria. Preparacion adicional:
EN_DEBATE; autorizacion SUSPENDIDA; dudas abiertas: confirmacion del usuario de
esta politica y de considerar divergencia ORB frente a GT como perdida temporal
de pose para el fallback Fase 5.
Acuerdo adicional 5G+5H cerrado: ignorar por completo el error GT-pose estimada
como criterio de fuente porque la deriva puede hacerlo grande sin implicar
perdida. No se implementara guarda de divergencia frente a GT. La fuente queda
congelada por goal: GT no pasa a ORB hasta terminar; ORB pasa inmediatamente a
GT solo cuando el estado ORB real deja de ser valido y permanece en GT el resto
del goal. Entre goals se aplica ORB si esta anclado, tracking OK y cualificado.
RViz2 mostrara `o_t_body` exacta que entra en `control_calcular_fuerzas`, sin
depender de `global_valid`, conservando ejes y etiqueta con fuente. Prueba
acordada: repetir YAML tipico completo con Gazebo GUI y RViz2 visibles.
Preparacion adicional: CERRADA. Acuerdo cerrado: si. Dudas abiertas: ninguna.
Autorizacion funcional 5G+5H: CONCEDIDA el 2026-08-27. Plan: documentar contratos
5G/5H; coordinar frontera de goal entre `gen_tray` y mux con handshake fiable;
modificar visualizador/tests; compilar `dron_individual` y `simulacion_dron`;
ejecutar suites y prueba visual 242. Siguiente accion exacta: actualizar los MD
de 5G/5H con el acuerdo antes del cambio funcional.
Contratos 5G/5H actualizados: fuente congelada por goal, ORB->GT inmediato por
tracking perdido, retorno solo en frontera, `o_t_body` exacta en RViz2 y error
GT-estimada solo como metrica externa sin autoridad. Implementacion aplicada:
`GoalSourceLock`; cualificacion basada solo en 20 muestras consecutivas de
tracking+anchor, sin comparar GT; servicio namespaced
`control/set_trajectory_active`; handshake de `gen_tray` al inicio/final con
espera de nueva muestra en frontera; C_T_W cacheada limitada al epoch; ejes
RViz2 desde `o_t_body`, persistentes ante muestras no consumibles y etiqueta
`[ORB]/[GT]`. Tests unitarios actualizados. Siguiente accion exacta: revisar
diff mecanico/compilabilidad y compilar primero `dron_individual`.
Build source-lock 5H `dron_individual`: 1/1 correcto, exit 0, 30.0 s. Log
completo conservado y no leido. Siguiente accion exacta: ejecutar los GTest de
goal policy y mux; si pasan, compilar `simulacion_dron` con el visualizador
actualizado.
GTest source-lock 5H: 2/2 correctos, exit 0, 0.13 s; cubren goal GT bloqueado,
perdida ORB con fallback retenido, cualificacion consecutiva y frame absoluto
cacheado. Siguiente accion exacta: compilar `simulacion_dron` para instalar el
visualizador/control-pose y su test actualizado.
Build control-pose RViz `simulacion_dron`: 1/1 correcto, exit 0, 0.79 s. Log
completo conservado y no leido. Siguiente accion exacta: ejecutar el test del
visualizador y despues una comprobacion de integracion corta del handshake antes
de preparar la simulacion larga 242.
CTest visualizador intento 1: no llego a ejecutar tests, exit 8, porque el
comando directo no cargo overlays y Python no encontro `orbslam3_msgs`; 0 items
coleccionados, no es fallo funcional. Siguiente accion exacta: repetir el mismo
CTest cargando los overlays Dron/Servidor/Simulacion documentados.
CTest visualizador intento 2 con overlays: 1/1 correcto, exit 0, test real 1.19 s.
Prueba 242 preparada con el YAML tipico absoluto exacto, Gazebo GUI y RViz2
control-pose visibles, fallback true, metricas en `metricas/prueba_242`, startup
2 s, timeout 1200 s, post 35 s y monitor. Se desactiva solo la terminal verbose
Fase 3 para reducir carga de interfaz; backend y logs de archivo siguen activos.
Criterios: handshake por cada goal, ninguna transicion GT->ORB entre lock/unlock,
ORB->GT permitido por perdida, 16 pasos completos y ejes persistentes desde
`o_t_body`; la revision Gazebo/RViz2 corresponde al usuario. Siguiente accion
exacta: ejecutar prueba 242 y registrar resultado antes de reducir.
Prueba 242 finalizada: scenario runner exit 1 y wrapper exit 1 tras 150 s;
`NO CONSEGUIDA`, fallo temprano aun sin atribuir. Gazebo/launch permanecieron
activos, guarda inactiva y minimo MemAvailable 5683.1 MiB. Log completo en
`logs/prueba_242.log`, no se leera directamente; metricas en `metricas/prueba_242`.
Siguiente accion exacta: reducir exclusivamente runner, handshake, lock, goals y
fuentes para localizar el primer fallo antes de tocar comportamiento o repetir.
Diagnostico reducido 242: el source lock funciona; goals 1-4 terminan y ORB se
cualifica durante GT sin conmutar, pasando a ORB solo tras `active=false`. El
paso 6 se rechaza porque el handshake de frontera considera suficiente la
primera muestra nueva, que por reset de velocidad tiene `velocity_valid=false`;
el goal llega 39 ms despues y `gen_tray` devuelve `reject_local_invalid` aunque
local/global/frame son validos. Correccion mecanica: al desbloquear esperar una
muestra posterior con local, continuidad y velocidad validas, no solo un numero
de secuencia distinto. El abort de shutdown es consecuencia del goal parejo ya
aceptado. Siguiente accion exacta: corregir el predicado de handshake, compilar,
repetir GTest y ejecutar una nueva prueba 243.
Predicado de frontera corregido: exige secuencia nueva y muestra local,
continua y con velocidad valida antes de devolver el resultado del goal. No se
cambia politica de fuente. Siguiente accion exacta: recompilar
`dron_individual` y registrar resultado.
Build handshake consumible `dron_individual`: 1/1 correcto, exit 0, 13.1 s.
Log completo conservado y no leido. Siguiente accion exacta: ejecutar los dos
GTest de navegacion y preparar 243 si pasan.
GTest handshake consumible: 2/2 correctos, exit 0, 0.16 s. Prueba 243 preparada
identica a 242: YAML tipico, Gazebo GUI, RViz2 `o_t_body`, fallback true,
metricas `prueba_243`, terminal F3 desactivada, startup 2 s, timeout 1200 s,
post 35 s y monitor. Siguiente accion exacta: ejecutar 243 y registrar resultado
antes de reducir.
Prueba 243 finalizada correctamente: scenario runner exit 0, `SIM-DONE
success=true`, wrapper exit 0, duracion 492 s. Guarda inactiva y minimo
MemAvailable 5572.5 MiB. Log completo en `logs/prueba_243.log`, no se leera
directamente; metricas en `metricas/prueba_243`. Resultado visual pendiente de
interpretacion final del usuario. Siguiente accion exacta: reducir runner,
handshake, lock, fuentes y RViz; verificar programaticamente que no hubo
GT->ORB dentro de goals y documentar evidencia antes del cierre.
Reanudacion tras compactacion 2026-08-27: checkpoint releido fisicamente y
reconciliado con la peticion `sigue`. La reduccion y auditoria de 243 ya estan
completadas: 17/17 pasos, 22/22 goals, 44/44 handshakes correctos, 0 cambios
GT->ORB con goal activo y 12 ORB->GT por perdida real, permitidos por contrato.
Los ejes RViz2 se publican desde la `o_t_body` consumida por el controlador y
conservan su ultima pose ante muestras temporalmente no consumibles. Builds y
tests seleccionados son correctos. Estado agregado 5G+5H: PARCIAL hasta recibir
la valoracion visual del usuario sobre Gazebo/RViz2 de 243; el error GT-pose
estimada queda excluido de criterios y decisiones. Trabajo activo: cierre
documental. Siguiente accion exacta: sincronizar estado minimo, pipeline y
ultima sesion, ejecutar `git diff --check` y comunicar el resultado tecnico sin
declarar CONSEGUIDA la revision visual.
Cierre operativo 2026-08-27: estado minimo, resumen actual, pipeline maestro,
pipeline Fase 5, ultima sesion, contratos, docs de paquete e historiales 5G/5H
sincronizados. `git diff --check` correcto. No queda build, test, simulacion ni
proceso activo. Resultado agregado 5G+5H: PARCIAL; implementacion y validacion
tecnica de 243 correctas, pendiente exclusivamente de la observacion visual del
usuario sobre Gazebo/RViz2. Autorizacion funcional consumida. Trabajo activo:
no; esperar esa valoracion antes de cerrar o abrir una correccion concreta.
Repeticion visual 244 autorizada 2026-08-27: mismo YAML
`prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`, launch
`simulacion_dron multi_dron.launch.py`, Gazebo GUI y RViz2 de pose de control
activos, `GT_FALLBACK` activo, logs F3 de terminal desactivados, startup 2 s,
timeout 1200 s y post 35 s. No se haran cambios ni analisis adicional.
Trabajo activo: simulacion 244. Siguiente accion exacta: ejecutar la prueba y
mantener las ventanas visibles para la revision del usuario.
Prueba 244: `scenario_runner_node` termino con exit 1 antes de iniciar la
trayectoria; wrapper exit 1, recursos correctos y guarda inactiva. Gazebo/RViz2
se cerraron durante cleanup. Repeticion mecanica 245 preparada con el mismo
YAML y launch, aumentando solo el arranque a 15 s. Siguiente accion exacta:
ejecutar 245 y dejar Gazebo/RViz2 visibles durante la trayectoria.
Prueba 245: exit 1 por ruta YAML relativa al directorio del workspace, no por
simulacion ni codigo (`bad file`). Correccion mecanica para 246: usar la ruta
absoluta del mismo YAML; resto identico. Siguiente accion exacta: ejecutar 246.
Prueba 246 interrumpida por peticion del usuario durante la trayectoria tras
observar que un cambio `GT -> ORB` vuelve loco al dron. El proceso se detuvo y
cleanup termino; recursos correctos y guarda inactiva. Esta observacion cambia
la interpretacion del bloque: 5G+5H siguen PARCIAL y la validacion visual falla
en la conmutacion. Trabajo activo: diagnostico dirigido de 246. Siguiente accion
exacta: reducir solo source lock, fronteras de goal, pose de control y comandos
alrededor de `GT -> ORB`, sin leer el log completo ni modificar codigo.
Diagnostico 246 cerrado: la fuente se desbloquea al terminar cada action goal,
pero `control_calcular_fuerzas` mantiene `feedback_activado=true` y conserva el
ultimo `x_des`. En la frontera compara esa consigna GT con `o_t_body` ORB; el
primer cambio ocurre inmediatamente tras `active=false` y ambos drones pierden
tracking unos 1.5 s despues. La unidad correcta de bloqueo es toda la mision
YAML, como pretendia el acuerdo del usuario: conservar fuente inicial hasta el
final; si ORB se pierde, pasar a GT y retenerlo el resto de la mision. 5G+5H:
PARCIAL, prueba 246 NO CONSEGUIDA visualmente. Historial y estado resumido
sincronizados. No se ha modificado codigo. Duda abierta: ninguna tecnica;
pendiente autorizacion explicita del usuario para implementar lock de mision,
compilar y repetir la prueba.
Preparacion correccion 5H: CERRADA. Acuerdo cerrado: si. Autorizacion
funcional: CONCEDIDA el 2026-08-27. Alcance revisado por el usuario: no ampliar
el lock a mision; aplicar la alternativa temporal minima que sincroniza el
nuevo goal con el cambio de frame. `control_calcular_fuerzas` detectara un
cambio de `pose_source` valido y sustituira la consigna retenida por hold en la
nueva `o_t_body` (posicion/yaw actuales, derivadas/feedforward cero) antes de
seguir controlando; el handshake vigente hara que el siguiente goal se genere
desde una muestra del mismo frame. No se usara blending ni error GT-estimada.
Prueba acordada: build y tests seleccionados de `dron_individual`, seguida de
la trayectoria tipica completa con Gazebo y RViz2. Criterio: sin impulso brusco
en `GT -> ORB`, goals correctos y observacion visual del usuario. Riesgo
aceptado: solucion temporal que se retirara en Fase 6. Dudas abiertas: ninguna.
Trabajo activo: implementacion. Siguiente accion exacta: modificar solo
`control_calcular_fuerzas.cpp` y su documentacion de paquete.
Cambio minimo aplicado: `control_calcular_fuerzas` guarda la ultima
`pose_source`; ante una transicion valida resetea la consigna retenida a la
`o_t_body` actual, yaw actual y feedforward/derivadas cero, manteniendo el
control de hover hasta el siguiente feedback. Marcador nuevo:
`[F5H-CONTROL-SOURCE-HOLD]`. Documentacion de paquete sincronizada. Build
preparado: `./codex/herramientas/build_selected_packages.sh --group dron
dron_individual`. Siguiente accion exacta: compilar `dron_individual`.
Build reset de consigna intento 1: fallo, exit 2, 0/1 paquetes, porque la
variable local `double x` del cuaternion oculta al miembro `Vector3d x` y
`x_des = x` intenta asignar un escalar. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log`, no leido directamente. Correccion
mecanica sin cambio funcional: usar `x_des = this->x`. Siguiente accion exacta:
aplicar esa correccion y repetir el mismo build.
Build reset de consigna intento 2: exit 0, `dron_individual` 1/1 correcto en
15.4 s. Log completo conservado y no leido. Siguiente accion exacta: ejecutar
los GTest seleccionados de navegacion/control disponibles y, si pasan, preparar
la simulacion visual 247 con el YAML tipico y marcador de hold.
CTest intento 1: no ejecuto tests; el sandbox impidio escribir
`Testing/Temporary/LastTest.log` bajo `build/dron`. No es fallo funcional.
Siguiente accion exacta: repetir el mismo CTest con permiso operativo.
CTest completo intento 2: 5/8; los GTest funcionales
`test_navigation_goal_policy` y `test_navigation_state_mux` pasan, y el archivo
tocado pasa `uncrustify`. Fallan `flake8`, `pep257` y `uncrustify` global por
deuda legacy en archivos ajenos ya conocida. Siguiente accion exacta: ejecutar
solo los dos GTest funcionales como evidencia seleccionada y preparar 247.
GTest seleccionados: 2/2 correctos, exit 0, 0.16 s. Prueba 247 preparada con
YAML absoluto `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`, Gazebo GUI,
RViz2 de pose de control, fallback activo, terminal F3 desactivada, startup
15 s, timeout 1200 s, post 35 s y monitor. Patrones de cierre: marcador
`F5H-CONTROL-SOURCE-HOLD`, fuentes, locks, goals y errores. Siguiente accion
exacta: ejecutar 247 y dejar las ventanas visibles para el usuario.
Prueba 247 interrumpida por peticion del usuario tras observar que persisten
saltos malos. Cleanup completo, recursos correctos y guarda inactiva. Resultado
visual: NO CONSEGUIDA; el reset de consigna no basta. Autorizacion funcional:
SUSPENDIDA para nuevos cambios hasta diagnosticar. Siguiente accion exacta:
reducir `F5H-CONTROL-SOURCE-HOLD`, fuentes, locks, goal snapshots, continuidad
ORB y errores, y contrastar el orden temporal sin leer el log completo.
Diagnostico 247: `[F5H-CONTROL-SOURCE-HOLD]` confirma que el reset ocurre. La
primera conmutacion sucede durante una espera de 8 s, sin goal siguiente, pero
los incrementos ORB crecen hasta `0.032 m / 0.145 rad` y ambos drones pierden
tracking 1.2-1.6 s despues. Otra conmutacion muestra `0.048 m / 0.063 rad` y
perdida en unos 1.2 s. Se descarta la consigna GT retenida y la conversion del
nuevo goal como causas principales. Hipotesis vigente: pose/velocidad/actitud
ORB cierran un lazo inestable con ganancias ajustadas para GT suave; hace falta
separar `ev`, error de actitud y fuerza/torque antes de elegir correccion.
Historial y estado sincronizados. No se aplicaran mas cambios sin nuevo acuerdo.
Revision conversada de 247: el usuario descarta tocar ORB o ajustar el
controlador porque ambos funcionan bien por separado. La causa aceptada es una
conmutacion incompleta: el reset fija `x_des=x_ORB` pero `x_dot_des=0`, por lo
que solo `ep` empieza en cero; `ev` no. Ademas GT->ORB ocurre al finalizar el
goal anterior, incluso durante waits, en lugar de ser atomico con el siguiente
goal. Politica propuesta: mantener GT entre goals; al llegar el siguiente,
capturar una muestra ORB comun, generar la trayectoria con `x0=x_ORB` y
`v0=v_ORB` (tambien yaw/rate coherentes), y activar ORB junto al primer setpoint
sin ciclo intermedio. No tocar ORB, filtros ni ganancias. Preparacion:
EN_DEBATE; autorizacion funcional: SUSPENDIDA; duda abierta: confirmacion final
de esta secuencia antes de implementarla.
Acuerdo atomico 5H cerrado: el usuario confirma la secuencia y autoriza
implementacion, build, tests y repeticion visual. Preparacion: CERRADA. Acuerdo
cerrado: si. Autorizacion funcional: CONCEDIDA el 2026-08-27. Dudas abiertas:
ninguna. Alcance: mantener la fuente bloqueada entre goals; al aceptar el
siguiente, abrir frontera, esperar muestra consumible, bloquear la fuente
elegida y congelar de esa misma muestra pose, velocidad y `C_T_W`; generar y
publicar `t=0` con `x0/v0/yaw0/yaw_rate0`; en el controlador, el hold de cambio
usa tambien la velocidad/rate actuales para `ep=ev=0`. No tocar ORB, filtros ni
ganancias. Documentar en Fase 6 la retirada de este handshake, source lock,
hold y `GT_FALLBACK` cuando las tareas/recovery reales sean autoridad. Prueba:
build/GTest `dron_individual` y vuelta tipica con Gazebo/RViz2. Siguiente accion
exacta: modificar `gen_tray`, ajustar el hold y actualizar contratos F5/F6.
Implementacion atomica aplicada: `gen_tray` retiene fuente al finalizar; al
inicio siguiente hace unlock+espera consumible+lock, captura un unico
`NavigationState` y `C_T_W`, y fuerza primer feedback `t=0`. Eliminado el
snapshot prematuro y estado pose/vel duplicado. El hold copia pose, velocidad,
yaw y yaw rate actuales. Contratos 5H, paquete y deuda de 6I/resumen Fase 6
actualizados para retirar todo el mecanismo temporal. `git diff --check`
correcto. Build preparado: `dron_individual` aislado. Siguiente accion exacta:
compilar y diagnosticar solo si falla.
Build atomico `dron_individual`: exit 0, 1/1 correcto, 26.0 s. Log completo
conservado y no leido. Siguiente accion exacta: ejecutar los dos GTest
funcionales de navegacion y preparar la prueba visual 248 si pasan.
GTest atomicos: 2/2 correctos, exit 0, 0.13 s. Prueba 248 preparada: YAML
tipico absoluto con ruta absoluta, Gazebo GUI, RViz2 control-pose, fallback
activo, logs F3 terminal desactivados, startup 15 s, timeout 1200 s, post 35 s
y monitor. Criterio programatico: `SOURCE-RETAINED` durante waits,
`ATOMIC-GOAL-START` por goal, primer feedback `t=0`, goals correctos; criterio
visual: ausencia de salto malo en GT->ORB. Siguiente accion exacta: ejecutar
248 y esperar observacion del usuario.
Prueba 248 interrumpida por peticion del usuario tras observar un nuevo error.
Cleanup completo, recursos correctos y guarda inactiva; no se interpreta como
success por interrupcion. Autorizacion funcional: SUSPENDIDA para cambios.
Siguiente accion exacta: reducir solo `ATOMIC-GOAL-START`, retencion, locks,
fuentes, goals, continuidad y primer ERROR/FATAL para localizar el fallo.
Reanudacion diagnostica 2026-08-28: checkpoint releido fisicamente y
reconciliado con la peticion vigente. La reduccion inicial confirma que el
arranque atomico recibe `x0/v0/yaw0/yaw_rate0` coherentes y pequenos, pero los
dos drones pierden tracking aproximadamente 1.5 s despues del primer cambio a
ORB y su pose GT ya es fisicamente anomala al siguiente goal. El fallo del paso
16 pertenece a la interrupcion manual y no es la causa. Trabajo activo: aislar
si el primer goal ORB convierte su objetivo world con una `C_T_W` autoritativa
del mismo epoch o con la identidad conservada durante `GT_FALLBACK`, sin editar
codigo ni repetir simulacion. Autorizacion funcional: SUSPENDIDA. Siguiente
accion exacta: regenerar un reducido tematico de 248 con autoridad global,
estado de pose, fuente y arranque atomico, y leer solo ese artefacto.
Diagnostico 248 cerrado: ambos wrappers tenian autoridad global autoritativa
del epoch 0 y revision 2 antes de la frontera. El mux paso a ORB y los arranques
atomicos llegaron 34-51 ms despues con pose coherente y velocidades menores de
`0.007 m/s`; por tanto no hubo identidad global heredada ni error inicial
traslacional apreciable. Dron 1 y dron 2 perdieron tracking 1.56-1.70 s despues
y sus poses GT del siguiente goal confirman divergencia fisica, incluido dron 2
a `z=11.37 m`. El error del paso 16 fue efecto de la interrupcion manual. El
handshake actual no sincroniza ni registra el lazo completo de actitud/fuerza
(`er`, `ew`, fuerza y torque), asi que la causa queda localizada en esa parte
rotacional/dinamica pero no atribuida a un termino sin inventar evidencia.
Estado 5G+5H: PARCIAL; prueba 248 NO CONSEGUIDA visualmente. Autorizacion
funcional: SUSPENDIDA. Trabajo activo: no. Siguiente accion exacta: acordar con
el usuario una instrumentacion temporal minima del primer `GT -> ORB` antes de
elegir cualquier otra correccion.
Preparacion correccion angular 5H: CERRADA. Acuerdo cerrado: si.
Autorizacion funcional: CONCEDIDA el 2026-08-28. Objetivo: completar el
handoff atomico `GT_FALLBACK -> ORB` con actitud y velocidad angular. Al primer
feedback de la nueva trayectoria se capturaran `R_act/w_b`, se impondran
`R_des=R_act` y `Omega_des=w_b`, y se interpolara en SO(3) durante `0.5 s`
hacia la referencia nominal. Se registraran de forma acotada `er`, `ew`, fuerza
y torque al inicio/mitad/final. Alcance: solo `control_calcular_fuerzas` y su
documentacion; no tocar ORB, ganancias ni `ORB -> GT`; marcar retirada en Fase
6. Prueba acordada: build y GTest seleccionados de `dron_individual`, seguida
de la trayectoria tipica completa con Gazebo y RViz2. Criterio: primer `er/ew`
practicamente cero, transicion sin picos bruscos y ausencia visual del salto.
Riesgo aceptado: handoff temporal de Fase 5. Dudas abiertas: ninguna. Trabajo
activo: implementacion. Siguiente accion exacta: localizar parametros y estado
del controlador y aplicar el cambio minimo antes del build aislado.
Handoff angular implementado: `control_calcular_fuerzas` arma la transicion solo
en `GT_FALLBACK -> ORB`, la activa con el primer feedback, fuerza el primer
ciclo a `R_des=R_act/Omega_des=w_b` e interpola durante `0.5 s` con smoothstep y
quaternion `slerp`. Una perdida/cualquier otra transicion cancela el estado
pendiente. Marcador acotado `[F5H-ANGULAR-HANDOFF]` en inicio/mitad/final con
`er/ew`, fuerza y torque. Docs del paquete y deuda de retirada Fase 6
sincronizadas. Build preparado: `./codex/herramientas/build_selected_packages.sh
--group dron dron_individual`. Siguiente accion exacta: comprobar diff mecanico
y compilar `dron_individual`; reducir el log solo si falla.
Build handoff angular intento 1: no compilo, exit 1 antes de procesar paquetes,
porque el sandbox impidio crear `/home/chenfu/Gazebo/log/dron/build_*`. No es un
fallo de codigo. `git diff --check` previo correcto. Correccion mecanica:
repetir el mismo comando con el permiso operativo preaprobado. Siguiente accion
exacta: ejecutar de nuevo el build aislado fuera del sandbox y registrar su
resultado antes de tests.
Build handoff angular intento 2: exit 0, `dron_individual` 1/1 correcto en
14.5 s. Log completo conservado en `codex/archivos_auxiliares/colcon_build.log`
y no leido. Siguiente accion exacta: ejecutar los GTest seleccionados
`test_navigation_goal_policy` y `test_navigation_state_mux` y el check de estilo
del archivo tocado; preparar simulacion 249 solo si pasan.
Tests handoff angular: GTest funcionales 2/2 correctos (`goal_policy` y
`navigation_state_mux`). CTest agregado devuelve exit 8 solo porque
`uncrustify` global conserva divergencias legacy en cinco archivos ajenos; el
propio resultado confirma `No code style divergence` para
`control_calcular_fuerzas.cpp`. Prueba 249 preparada con el YAML absoluto
`prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`, Gazebo GUI, RViz2 de pose de
control, fallback activo, startup 15 s, timeout 1200 s, post 35 s y monitor.
Patrones: handoff angular, arranque atomico, fuentes, tracking, goals y errores.
Criterio: inicio `er/ew` cercano a cero, tres etapas finitas sin pico, mision
completa y revision visual sin salto. Siguiente accion exacta: ejecutar 249 y
registrar el resultado antes de reducir el log.
Prueba 249 interrumpida por peticion del usuario tras 274 s. Sesion exit 130 por
SIGINT manual; cleanup completo, launch detenido, guarda de recursos inactiva y
minimo MemAvailable 5789.7 MiB. El wrapper de cleanup informa
`SIM-EXIT-CODE 0`, pero la trayectoria no se considera completada ni validada.
Log completo conservado en `codex/archivos_auxiliares/logs/prueba_249.log` y no
leido. Trabajo activo: no. Autorizacion funcional consumida. Siguiente accion
exacta: esperar la observacion visual del usuario antes de reducir o interpretar
249.
Diagnostico reducido 249: el handoff angular queda validado mecanicamente.
Inicio en ambos drones con `alpha=0`, `er=ew=0`, fuerza de hover
`13.68-13.79 N` y torque cero; al final `er<0.024`, `ew<0.035` y
`torque<0.0033 Nm`. Aun asi pierden tracking 1.7-2.2 s despues. Tras el goal,
dron 2 pasa de `z=1.31` a `z=11.37 m` cuando el target world cambiaba X de 0 a
+10 m, y dron 1 con X=-10 termina en el suelo. Interpretacion revisada: el
desplazamiento X se esta proyectando sobre Z al aplicar la rotacion de `C_T_W`.
No es un impulso del handoff; lo mas probable es una convencion de orientacion o
body frame incompatible entre `O_T_B` y `W_T_B` (o la direccion de su
composicion). Estado 5H: PARCIAL/249 NO CONSEGUIDA. Autorizacion funcional:
SUSPENDIDA. Trabajo activo: no. Siguiente accion exacta: acordar instrumentar
target world/control y orientaciones O/W en una unica frontera antes de cambiar
la formula.
Preparacion diagnostico de frame 5H: CERRADA. Acuerdo cerrado: si.
Autorizacion funcional: CONCEDIDA el 2026-08-28. Alcance: observabilidad pura,
sin cambiar formulas, control, ORB ni politica de fuentes. `gen_tray` registrara
una vez por goal absoluto ORB el target world/control, `O_T_B`, `W_T_B`,
`C_T_W` y las imagenes de los ejes world en control. Prueba acordada: build y
GTest seleccionados de `dron_individual`, seguido de la trayectoria tipica en
modo diagnostico solo hasta obtener la primera frontera `GT -> ORB`. Criterio:
marcador completo y finito que permita decidir si falla la direccion de
composicion o la convencion body/camera. Riesgo: ninguno funcional; solo log
acotado. Dudas abiertas: ninguna. Trabajo activo: instrumentacion. Siguiente
accion exacta: editar `gen_tray.cpp` y el MD del controlador, revisar diff y
compilar el paquete aislado.
Instrumentacion de frame aplicada: `gen_tray` conserva `world_target` antes de
convertirlo y emite `[F5H-ABSOLUTE-FRAME-DIAG] part=poses/target_axes` solo para
goals XYZ absolutos iniciados con ORB. Incluye autoridad, `O_T_B`, `W_T_B`,
`C_T_W`, target world/control y columnas de rotacion world->control con el mismo
epoch/sample. No cambia ningun dato usado por la trayectoria. MD del paquete
sincronizado y `git diff --check` correcto. Build preparado:
`./codex/herramientas/build_selected_packages.sh --group dron dron_individual`.
Siguiente accion exacta: compilar `dron_individual` fuera del sandbox y registrar
el resultado antes de tests.
Build diagnostico frame: exit 0, `dron_individual` 1/1 correcto en 12.0 s. Log
completo conservado y no leido. Siguiente accion exacta: ejecutar GTest
`test_navigation_goal_policy` y `test_navigation_state_mux`; si pasan, preparar
prueba diagnostica 250 con timeout corto suficiente para alcanzar el step 5.
GTest diagnostico frame: 2/2 correctos, exit 0. Creado
`trayectorias/tray_prueba_250.yaml`: reproduce llegada y anclaje, envia los
targets world X=+/-10 con duracion 40 s y timeout de action 1 s para capturar el
marcador antes de movimiento material; la cancelacion/fallo final es esperada y
no es criterio funcional. Prueba 250 preparada sin Gazebo GUI, RViz2 ni metricas,
fallback activo, startup 15 s, timeout 180 s, post 5 s y monitor. Criterio:
cuatro lineas DIAG (poses+target_axes por dron) finitas y emparejadas. Siguiente
accion exacta: ejecutar 250, registrar salida y reducir exclusivamente el
marcador y la secuencia de frontera.
Prueba diagnostica 250 finalizada con scenario/wrapper exit 1 esperado por el
timeout intencional de 1 s del segundo goal; duracion 91 s, cleanup correcto,
guarda inactiva y minimo MemAvailable 6227.6 MiB. No es una validacion funcional
de trayectoria. Log completo en `logs/prueba_250.log`, que no se leera.
Siguiente accion exacta: reducir 250 por `F5H-ABSOLUTE-FRAME-DIAG`, arranque
atomico, fuente y resultado del step; comprobar que existen poses/axes de ambos
drones y calcular la causa exclusivamente desde ese reducido.
Reduccion downstream 250: cuatro lineas completas y finitas. En ambos drones
`world_x_in_control=(~0.014,~0,~0.9999)`: X world se convierte casi exactamente
en Z control. Targets resultantes: dron 1 `(-0.255,-9.887,-8.920)` y dron 2
`(0.019,-9.956,11.232)`, explicando suelo/subida. El handoff vuelve a iniciar
con `er=ew=0`. Para distinguir autoridad global incorrecta de conversion
camera/body se amplia observabilidad pura en el wrapper, autorizada por la
peticion de diagnostico completo y el `sigue` posterior. Nuevo marcador
`[F5H-WRAPPER-FRAME-DIAG]` registra inputs `O_T_C/W_T_C/B_T_C` y outputs
`O_T_B/W_T_B/Tcr`, solo con autoridad y throttle 1 s. Docs del wrapper
sincronizadas. Siguiente accion exacta: revisar estilo/diff, compilar paquete
ROS `orbslam3`, ejecutar su test de estimator y repetir captura corta como 251.
Build wrapper frame diag: exit 0, `orbslam3` 1/1 correcto en 29.1 s. Solo emite
warnings legacy de cv_bridge/ORB-SLAM3/Eigen; no hay error del cambio. Log
completo conservado y no leido. Siguiente accion exacta: ejecutar el CTest
seleccionado `test_navigation_state_estimator`; si pasa, preparar 251 identica a
250 con el nuevo wrapper instalado.
CTest wrapper frame diag: `test_navigation_state_estimator` 1/1 correcto, exit
0. Prueba 251 preparada con el mismo `tray_prueba_250.yaml`, sin GUI/RViz2,
fallback activo, startup 15 s, timeout 180 s, post 5 s y monitor. Criterio:
capturar pares upstream inputs/outputs de ambos drones mas los cuatro marcadores
downstream antes del timeout intencional. Siguiente accion exacta: ejecutar 251,
registrar resultado y reducir solo ambos marcadores de frame y la frontera.
Prueba diagnostica 251 finalizada: scenario/wrapper exit 1 esperado por timeout
intencional del step 5, 91 s, cleanup correcto, guarda inactiva y minimo
MemAvailable 6408.6 MiB. Log completo en `logs/prueba_251.log`, no se leera.
Siguiente accion exacta: reducir por `F5H-WRAPPER-FRAME-DIAG`,
`F5H-ABSOLUTE-FRAME-DIAG`, autoridad y frontera; emparejar samples y cerrar el
diagnostico sin modificar formulas.
Checkpoint final diagnostico 251: peticion `sigue` completada. X world llega
casi como Z control y el handoff arranca con error nulo. El contraste optico
demuestra que `W_T_C≈(-0.707,0,0,0.707)` es correcto para cuerpo yaw 90; la
rotacion YAML cargada como `B_T_C=(0.5,-0.5,0.5,0.5)` es en realidad su inversa
`C_T_B`. El wrapper la invierte otra vez y produce el `W_T_B` con ejes
permutados que explica exactamente suelo/subida. PnP, `FaceTransform` y el
anchor fiducial quedan descartados como origen. `use_camera_optical_frame_convention`
no se consume y la alineacion ORB-GT del mux ocultaba el defecto en O. Builds
previos correctos: `dron_individual` 1/1, `orbslam3` 1/1; tests seleccionados
3/3. Pruebas 250/251 diagnosticas, timeout intencional y cleanup correcto.
Documentacion de paquetes, contrato, historial, resumen, indice, estado y
ultima sesion sincronizados; `git diff --check` correcto. Preparacion: CERRADA.
Acuerdo cerrado: si. Autorizacion funcional: SUSPENDIDA tras consumir el
diagnostico. Prueba acordada: 251. Dudas abiertas: ninguna. Trabajo activo: no.
Reanudacion correccion 5H 2026-08-28: el usuario autoriza explicitamente
aplicar el diagnostico 251 y realizar la prueba. Preparacion: CERRADA. Acuerdo
cerrado: si. Autorizacion funcional: CONCEDIDA. Alcance: cambiar las copias de
calibracion efectivas a un `B_T_C` optico `RPY=(-90,0,-90)`, sin tocar formulas,
`C_T_W`, ganancias, optimizador, handoff ni politica de fuentes. Prueba acordada:
compilar consumidores y ejecutar sin modificar
`prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` con Gazebo y RViz2 visibles.
Criterio: `W_T_B` corporal coherente, ejes world/control sin permuta X->Z,
movimiento finito y vuelta completa o evidencia reducida del primer fallo real.
Riesgo aceptado: la vuelta puede revelar otro defecto despues de retirar esta
causa. Dudas abiertas: ninguna. Trabajo activo: correccion 5H. Cambios aplicados:
las tres copias `dron_individual/config/calibration.yaml`,
`simulacion_dron/config/calibration_dron.yaml` y
`orbslam3_server/config/calibration_dron.yaml` usan ya
`roll=-90,pitch=0,yaw=-90`; traslacion, flag y formulas intactos. Docs de
`dron_individual`, wrapper, Simulacion/modelo y Servidor sincronizadas.
`git diff --check` correcto y las tres rotaciones coinciden. Build preparado,
un paquete por invocacion: `dron_individual`, `orbslam3_server` y
`simulacion_dron`, para instalar configuracion y ejecutar sus guardas. Siguiente
accion exacta: compilar primero `dron_individual`; registrar resultado antes de
continuar con Servidor.
Build correccion extrinseca Dron: exit 0, `dron_individual` 1/1 correcto en
1.53 s. Log completo conservado en `codex/archivos_auxiliares/colcon_build.log`
y no leido. La nueva `calibration.yaml` queda instalada. Siguiente accion
exacta: compilar `orbslam3_server` en grupo Servidor para instalar/verificar su
replica; registrar el resultado antes de compilar Simulacion.
Build correccion extrinseca Servidor: exit 0, `orbslam3_server` 1/1 correcto en
18.3 s. Log completo conservado y no leido; replica instalada. Siguiente accion
exacta: compilar `simulacion_dron` en grupo Simulacion y registrar resultado;
si pasa, ejecutar guardas/tests focales de replicas antes de preparar 252.
Build correccion extrinseca Simulacion: exit 0, `simulacion_dron` 1/1 correcto
en 0.69 s. Los tres builds pasan e instalan sus copias. Siguiente accion exacta:
localizar y ejecutar los tests focales que validan replicas/configuracion; si no
existe una guarda especifica, comprobar igualdad estructural de las tres copias
y ejecutar los contratos de configuracion ya existentes antes de preparar la
simulacion 252.
Test focal de replicas: `global_map_config_contract` 1/1 correcto en 0.93 s;
confirma igualdad estructural Dron/Servidor/Simulacion. Prueba 252 se preparara
con el YAML oficial absoluto sin modificar, Gazebo GUI y RViz2 de pose de
control, fallback activo, startup 15 s, timeout 1200 s, post 35 s y monitor.
Patrones: diagnosticos wrapper/target/ejes, handoff, fuente, tracking, goals,
scenario, optimizacion y errores. Siguiente accion exacta: consultar opciones
del runner, registrar el comando definitivo y ejecutar 252.
Prueba 252 preparada definitivamente: YAML absoluto
`/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
launch `ros2 launch simulacion_dron multi_dron.launch.py
launch_gazebo_gui:=true launch_mission_gui:=false
debug_sparse_global_rviz:=true phase5_global_pose_rviz_enabled:=true
gt_fallback_enabled:=true debug_fase3_logs_terminal:=true`; startup 15 s,
timeout 1200 s, post 35 s, un reintento Gazebo y monitor de recursos. No se
activan metricas GT ni bypass legacy. Siguiente accion exacta: ejecutar 252 y,
al terminar o ser interrumpida por observacion del usuario, registrar exit,
success, cleanup y log completo antes de reducirlo.
Prueba 252 finalizada: scenario exit 0, `success=true`, wrapper exit 0 y cleanup
correcto tras 505 s. Monitor: 383 muestras, minimo MemAvailable 5624.0 MiB,
max ORB PSS 1366.1 MiB, memory PSI full 0 y guarda inactiva. Gazebo/RViz2
permanecieron visibles. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_252.log` y no se leera. Siguiente accion
exacta: reducir 252 por diagnosticos de frame/ejes, goals, fuentes, tracking,
scenario, optimizacion y errores; comprobar desaparicion X->Z y 17/17 pasos
antes de concluir o documentar.
Revision visual del usuario sobre 252: mejora muy grande y desaparicion del
fallo anterior, pero observa (1) movimiento a tirones frente a GT, (2)
movimientos puntuales alocados en giros/momentos raros y (3) una discrepancia
ocasional pose mostrada/KFs que podria coincidir con etiqueta GT y deriva ORB.
La prueba conserva `success=true`, pero 5H no se cierra aun. El reducido general
tiene 2187 lineas por telemetria F3; no basta como lectura eficiente. Siguiente
accion exacta: consultar exclusivamente `prueba_252.reduced.log` con subpatrones
de scenario, fuentes, tracking, continuidad, handoff, frame y errores; contar
transiciones y localizar sus timestamps respecto a goals/giros, sin abrir el
log completo ni modificar codigo.
Reanudacion diagnostico 252 tras compactacion: checkpoint fisico releido y
reconciliado con la ultima peticion. Evidencia reducida preliminar: fallback GT
en 68.2 % de muestras de drone1 y 76.1 % de drone2; 4/5 activaciones ORB;
perdidas `RECENTLY_LOST` 8/14 y `LOST` 3/3; saltos ORB hasta 0.208 m; varios
goals atomicos nacen con velocidades estimadas incompatibles con reposo (p. ej.
drone2 `v_z=-0.983 m/s` y drone1 velocidad total ~1.13 m/s). Preparacion:
CERRADA. Acuerdo cerrado: si. Autorizacion funcional: CONCEDIDA solo para el
diagnostico/documentacion de 252; no hay autorizacion para nuevos cambios de
control. Prueba acordada: analizar 252 ya ejecutada. Dudas abiertas: ninguna.
Trabajo activo: diagnostico visual 252. Siguiente accion exacta: confirmar en
la documentacion y el rango minimo de codigo del mux/control la frecuencia, el
calculo de derivadas y cualquier filtrado; despues correlacionar los eventos
reducidos con los giros y documentar la conclusion viva de la prueba.
Diagnostico 252 confirmado: `navigation_state_mux` publica al ritmo del estado
ORB (camara 20 Hz) y `PoseVelocityEstimator::Update` calcula velocidad lineal y
angular por diferencia finita de dos poses consecutivas, sin filtro, rechazo de
outliers ni interpolacion; el PD corre a 50 Hz y reutiliza la ultima muestra.
Esto amplifica el ruido ORB y explica el movimiento a tirones. Los eventos
alocados coinciden con tracking 2->3 y `ORB -> GT_FALLBACK` dentro de goals. En
esa rama `ContinuousSourcePose::ResetToSource` sustituye O por GT; el hold del
controlador pone error cero solo transitoriamente y el siguiente feedback del
goal, aun congelado en el O anterior, lo sobrescribe. En inicios ORB se observan
`v0` espurias y el handoff llega a `er_norm=0.72`, `ew_norm=0.50` en 0.5 s. La
discrepancia ejes/KFs no demuestra un KF incorrecto: los ejes muestran
`NavigationState.o_t_body` de control (a menudo `[GT]`) y los KFs usan W global
optimizado. Las optimizaciones pueden mover KFs visualmente, pero no reescriben
la trayectoria O congelada. Conclusion provisional de 252: `PARCIAL`; vuelta
17/17 y 22/22 completa, extrinseca corregida, pero suavidad y transicion de
marco aun no cumplen calidad visual. Siguiente accion exacta: actualizar la
entrada viva de 252, resumen 5H, indice/estado y docs de control con esta causa;
no modificar codigo funcional ni repetir simulacion.
Cierre diagnostico 252: historial largo y resumen 5H, indice, contrato 5H,
resumen de Fase 5, estado actual, contexto minimo, ultima sesion y docs de
`dron_individual`/wrapper sincronizados. El reducido 252 se regenero incluyendo
fuente, tracking, continuidad, arranques, hold, handoff, scenario y errores; el
log completo permanece sin leer. `git diff --check` correcto. Conclusion:
`PARCIAL`. Preparacion: CERRADA. Acuerdo cerrado: si. Autorizacion funcional:
SUSPENDIDA; el diagnostico autorizado esta consumido y cualquier cambio de
filtrado o transicion requiere nuevo acuerdo. Prueba acordada: 252 ya ejecutada
y revisada. Dudas abiertas: ninguna para el diagnostico. Trabajo activo: no.
Siguiente accion exacta: explicar al usuario las tres causas y, solo si lo pide,
preparar conversacionalmente la correccion minima antes de tocar codigo.
Preparacion correccion dinamica 5H tras 252: el usuario pide implementar las
dos medidas conversadas. Objetivo propuesto: (1) estimador/predictor permanente
que reciba ORB a 20 Hz, filtre pose/velocidad y publique estado propagado a
50 Hz para el controlador; (2) durante una perdida dentro de goal, alinear GT
con el O activo mediante continuidad en vez de `ResetToSource`, conservar GT
bloqueado hasta la frontera y permitir ORB solo en la trayectoria siguiente si
esta anclado. Los comentarios `TODO FASE 6` se limitaran a fallback, alineacion
GT, locks y handoff temporales; el predictor 50 Hz no se marcara para retirar
porque sigue siendo necesario sin GT. No se tocaran ganancias, ORB-SLAM3,
optimizador ni trayectoria YAML. Prueba propuesta: unitarios deterministas de
prediccion/ruido/outlier/rotacion y transicion ORB->GT, builds focales y nueva
vuelta tipica completa con Gazebo/RViz2, comparando suavidad, tracking, fuentes
y goals contra 252. Criterio: 17/17 y 22/22, sin salto numerico al entrar GT,
sin `v0` espurias equivalentes y mejora visual humana; los fallos F3 quedan
fuera. Riesgos: filtrado excesivo puede introducir retardo y la estimacion a
50 Hz no crea informacion visual nueva. Preparacion: EN_DEBATE. Acuerdo cerrado:
no. Autorizacion funcional: PENDIENTE. Prueba acordada: pendiente de confirmar
la repeticion de la vuelta 252. Dudas abiertas: confirmar que solo el codigo GT
temporal llevara marcas de retirada y que se repite la prueba completa. Trabajo
activo: preparacion 5H; no se ha modificado codigo/configuracion ni ejecutado
build/test/simulacion. Siguiente accion exacta: obtener esa confirmacion antes
de implementar.
Autorizacion correccion dinamica 5H: el usuario confirma la distincion de deuda
GT y la repeticion completa. Preparacion: CERRADA. Acuerdo cerrado: si.
Autorizacion funcional: CONCEDIDA. Prueba acordada: unitarios focales, builds y
vuelta tipica completa posterior a 252 con Gazebo/RViz2. Dudas abiertas:
ninguna. Plan: localizar contratos/tests/parametros actuales; implementar
estimador-predictor 50 Hz permanente con limites y tests; sustituir solo la
entrada GT en goal activo por alineacion continua y marcar deuda Fase 6;
compilar `dron_individual` y consumidores necesarios; ejecutar pruebas focales;
registrar y lanzar la nueva simulacion; reducir, analizar y documentar. Trabajo
activo: correccion dinamica 5H. Siguiente accion exacta: inspeccionar los tests,
CMake, launch/config y rangos minimos del mux para fijar archivos criticos antes
de editar.
Archivos criticos correccion dinamica 5H: header
`dron_individual/navigation_state_mux.hpp` para `ContinuousSourcePose` y nuevo
predictor SE(3); nodo `navigation_state_mux.cpp` para separar medicion 20 Hz de
publicacion 50 Hz y transportar temporalmente la pose world GT con
`global_valid=false`; `gen_tray.cpp` para obtener `O_T_W` explicita durante
fallback; `control_calcular_fuerzas.cpp` y `GoalSourceLock` solo para comentarios
de retirada Fase 6; `test_navigation_state_mux.cpp` para continuidad,
prediccion, ruido/outlier y rotacion. No hace falta cambiar mensajes ni YAML de
trayectoria. Parametros del predictor tendran defaults declarados en el nodo.
La prueba 252 demuestra que `success` del action no valida llegada, por lo que
los logs nuevos incluiran cadencia/prediccion e innovacion limitada ademas de
las fuentes existentes. Siguiente accion exacta: editar header, nodo, gen_tray,
control y tests en un bloque minimo; despues actualizar docs antes del build.
Bloque funcional correccion dinamica aplicado: `PoseStatePredictor` alpha-beta
limita innovacion lineal/angular, velocidad y extrapolacion; el mux actualiza
mediciones al ritmo ORB y publica por timer a 50 Hz con
`[F5H-STATE-PREDICTOR]/[F5H-STATE-FILTER]`. `ContinuousSourcePose::Update`
reemplaza por completo `ResetToSource`; durante fallback `w_t_body` transporta
la pose GT solo con `global_valid=false`, y `gen_tray` compone O_T_W para goals
absolutos. Tests cubren continuidad del goal, movimiento GT posterior,
prediccion 50 Hz, outlier y velocidad angular. Comentarios `TODO FASE 6` quedan
solo en lock/suscripcion/alineacion/transporte/hold GT; predictor permanente sin
marca de retirada. Docs de `dron_individual` y contrato 5H sincronizados;
`git diff --check` correcto. Siguiente accion exacta: compilar aisladamente
`dron_individual`; registrar resultado antes de tests o correcciones.
Build correccion dinamica 5H: exit 0, `dron_individual` 1/1 correcto en 43.5 s
(43.9 s total). Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido. El predictor, mux,
`gen_tray`, control y tests compilan. Siguiente accion exacta: ejecutar CTest
focal `test_navigation_state_mux` desde el build aislado Dron; registrar el
resultado antes de ampliar tests o preparar simulacion.
CTest focal predictor/mux: `test_navigation_state_mux` 1/1 correcto en 0.09 s.
Valida continuidad ORB->GT, desplazamiento GT posterior, prediccion lineal a
intervalos de 20 ms, limitacion de outlier y propagacion angular. Siguiente
accion exacta: ejecutar conjuntamente los contratos focales de estado y policy
de goals; si pasan, verificar estilo/diff y preparar la simulacion siguiente a
252 con nuevos patrones de predictor y continuidad.
Contratos focales conjuntos: `test_navigation_goal_policy` y
`test_navigation_state_mux` 2/2 correctos en 0.16 s. La policy de aceptacion y
el nuevo estado suave son compatibles. Siguiente accion exacta: listar los
tests de estilo disponibles, ejecutar el focal que cubra los archivos tocados y
`git diff --check`; despues registrar el comando definitivo de simulacion 253.
Uncrustify global intento 1: CTest falla. Detecta cinco archivos; tres son
legacy no tocados (`navigation_goal_policy.hpp`, `clock.cpp`,
`control_dron.cpp`). En archivos del alcance senala una indentacion previa en
`gen_tray.cpp` y tres formatos mecanicos en `navigation_state_mux.cpp`; header,
control y test nuevos pasan. Correccion siguiente puramente mecanica: aplicar
solo esos cuatro cambios de estilo en archivos del alcance, rebuild corto y
validar focalmente los archivos tocados sin reformatear legacy.
Correccion mecanica de estilo aplicada solo a `navigation_state_mux.cpp` y una
indentacion de `gen_tray.cpp`. `ament_uncrustify` focal sobre header, mux,
gen_tray, control y test: 5/5 sin divergencias. Los tres fallos legacy globales
permanecen sin tocar. Siguiente accion exacta: rebuild aislado de
`dron_individual` porque dos fuentes cambiaron desde el build; si pasa, repetir
los 2 GTests focales y preparar 253.
Rebuild correccion dinamica 5H: exit 0, `dron_individual` 1/1 correcto en
25.8 s (26.1 s total). Log completo conservado y no leido. Siguiente accion
exacta: repetir `test_navigation_goal_policy` y `test_navigation_state_mux`
sobre este build final; si pasan, cerrar bloque de validacion focal y registrar
la simulacion 253 antes de lanzarla.
Validacion focal final: GTests `test_navigation_goal_policy` y
`test_navigation_state_mux` 2/2 correctos en 0.18 s; uncrustify focal 5/5 y
`git diff --check` sin errores. Build/test quedan cerrados. Siguiente accion
exacta: consultar workflow/opciones del runner, registrar YAML/launch/timeout y
patrones definitivos de 253, y solo entonces ejecutar la vuelta con Gazebo y
RViz2 visibles.
Preparacion 253 detecta una carencia de evidencia, no funcional: no existe un
marcador numerico del salto exacto del mux al cambiar fuente. Se anadira
`[F5H-SOURCE-CONTINUITY]` con traslacion/rotacion entre la ultima salida medida
y la primera alineada; no cambia politica, filtro ni control. Correccion
mecanica/observacional dentro del acuerdo. Siguiente accion exacta: editar solo
el nodo y doc de control, rebuild/test focal corto y despues registrar 253.
Marcador `[F5H-SOURCE-CONTINUITY]` aplicado y documentado; registra salto
lineal/angular en toda conmutacion real. Uncrustify focal del nodo correcto.
Siguiente accion exacta: rebuild final `dron_individual` y GTest del mux; no se
modificara mas funcionalidad antes de 253 salvo error mecanico.
Build final con marcador: exit 0, `dron_individual` 1/1 correcto en 15.3 s
(15.6 s total); log completo conservado y no leido. Siguiente accion exacta:
ejecutar `test_navigation_state_mux`; si pasa, registrar configuracion completa
de 253 y lanzar la simulacion.
GTest final del mux: 1/1 correcto en 0.09 s; `git diff --check` correcto y el
numero 253 esta libre. Prueba 253 preparada con YAML absoluto sin modificar
`/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
launch `ros2 launch simulacion_dron multi_dron.launch.py
launch_gazebo_gui:=true launch_mission_gui:=false
debug_sparse_global_rviz:=true phase5_global_pose_rviz_enabled:=true
gt_fallback_enabled:=true debug_fase3_logs_terminal:=true`; startup 15 s,
timeout 1200 s, post 35 s, un reintento Gazebo y monitor de recursos. Patrones:
`F5H-STATE-PREDICTOR|F5H-STATE-FILTER|F5H-SOURCE-CONTINUITY|F5H-POSE-SOURCE|F5H-CONTROL-SOURCE-HOLD|F5H-ATOMIC-GOAL-START|F5H-ANGULAR-HANDOFF|F5B-TRACKING|F5B-O-CONTINUITY|F5H-ABSOLUTE-FRAME-DIAG|SCENARIO-RUNNER|SIM-|ERROR|FATAL`.
Siguiente accion exacta: ejecutar 253 y, al finalizar o ser interrumpida por el
usuario, registrar exit/success/cleanup/log antes de reducir o analizar.
Verificacion final bloque 2: `git diff --check` correcto; mensajes y servicios
Dron/Servidor identicos; contratos 5C-5F e historiales/resumen presentes y
coherentes; busqueda de estados obsoletos sin coincidencias. Los artefactos de
la prueba 230 quedan como evidencia vigente. No queda ejecucion activa.
Cambios completados Fase 5/5A: reconciliados pipeline, resumen y contratos
5A-5H; 5I convertida en stub absorbido; creados `historial/INDEX.md`,
`historial_5A.md` y su resumen; sincronizados `PIPELINE_MAESTRO.md`, contexto
minimo, estado actual resumido y ultima sesion. No se modifico codigo, launch,
YAML funcional, configuracion ni mensajes; no hubo build, test o simulacion.
Siguiente accion exacta 3Q: leer los resúmenes y MDs vigentes de los componentes
afectados, localizar los simbolos exactos y registrar los archivos criticos
antes de editar codigo o configuracion.
Archivos criticos localizados 3Q: `loop_pipeline.hpp/.cpp`,
`pose_graph_problem.hpp`, `pose_graph_builder.hpp/.cpp`,
`optimization_manager.hpp/.cpp`, `optimization_validator.hpp/.cpp`,
`sparse_global_backend.hpp/.cpp`, `global_pose_store.hpp/.cpp`,
`global_map_server.cpp`, las dos copias `config/global_map/loop_fusion.yaml` y
tests de `orbslam3_multi`/Servidor/Simulacion. Hallazgos confirmados: expansion
transitiva en `BuildLoop`, relajacion sin peso efectivo, convergencia ligada al
umbral de fusion, validator sin banda separada de commit y corredor antiguo
`5 m / 20 grados` con comparacion de exceso casi exacta.
Siguiente accion exacta 3Q tras localizacion: inspeccionar solo las estructuras
y funciones senaladas, cerrar el diseno mecanico de tipos/parametros y aplicar
el primer bloque de implementacion y regresiones sin tocar ORB-SLAM3 ni msgs.
Bloque 1 implementado 3Q: umbral de fusion reducido a `0.20 m / 0.12 rad`;
convergencia separada `0.05 m / 0.03 rad`; commit seguro hasta
`0.25 m / 0.15 rad`; apoyo adaptativo 2/4/6 con progresion query-candidate,
ambiguedad, anclaje/perdida y correccion grande; `information_weight` afecta
la relajacion normalizado por familia; corredor reemplaza `5 m / 20 grados` y
comparacion `1e-8` por deadband `0.02 m / 0.00872665 rad`; commit seguro fuera
de fusion no fusiona landmarks. YAML Servidor/Simulacion sincronizados y
regresion numerica `0.000416 m` anadida. Aun pendientes ventana segmentada,
persistencia explicita de `PriorLoop`, limpieza final y validacion.
Siguiente accion exacta 3Q: compilar `orbslam3_multi` para detectar errores
mecanicos del bloque 1 antes de abordar la seleccion segmentada.
Build 3Q bloque 1: `orbslam3_multi` 1/1 correcto, exit 0, 58.8 s. Log completo
conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido.
Siguiente accion exacta 3Q: implementar la seleccion por intervalos temporales
hard delimitados y expansion solo por aristas server incidentes en esos
intervalos; adaptar commit atomico y tests antes del siguiente build.
Bloque 2 implementado 3Q: `BuildLoop()` parte solo de endpoints CurrentLoop,
calcula intervalos temporales entre hard consecutivos y expande iterativamente
solo cuando el extremo de una arista `ServerLoopGeometric` o dependencia soft
ya cae dentro de un intervalo seleccionado. Intervalos disjuntos del mismo
submapa no crean aristas temporales ni propagacion a traves del hueco. Cada
submapa conserva un unico batch con lista explicita de intervalos y el commit
solo actualiza KFs dentro de ellos. Eliminado el fixed artificial cuando no hay
hard; el gauge relativo se reparte entre lados movibles.
Siguiente accion exacta 3Q: rebuild aislado de `orbslam3_multi`; si compila,
ejecutar su CTest para localizar regresiones contractuales antes de completar
la ruta fiducial comun y persistencia PriorLoop.
Build 3Q bloque 2: `orbslam3_multi` 1/1 correcto, exit 0, 42.8 s. Log completo
conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido.
Siguiente accion exacta 3Q: ejecutar CTest de
`/home/chenfu/Gazebo/build/servidor/orbslam3_multi`; registrar el resultado
antes de corregir tests o continuar con el bloque funcional restante.
CTest `orbslam3_multi` intento 1: no ejecuto funcionalmente ninguna suite;
0/9 porque el entorno no encontraba
`liborbslam3_msgs__rosidl_generator_c.so` (exit 8). No es evidencia de fallo
del codigo. Correccion operativa mecanica: repetir el mismo CTest tras cargar
`/opt/ros/iron/setup.bash` y `/home/chenfu/Gazebo/install/servidor/setup.bash`.
CTest `orbslam3_multi` intento 2: 8/9 targets correctos; 7/12 casos de
`test_loop_pipeline` pasan y cinco contratos antiguos fallan. Causa: siguen
esperando aceptacion fija tras dos apoyos y rechazo previo `5 m / 20 grados`,
ambos comportamientos sustituidos deliberadamente por apoyo adaptativo 2/4/6.
`test_fiducial_optimization`, incluida la deadband numerica, pasa completo.
Siguiente accion exacta 3Q: reemplazar los cinco tests obsoletos por regresiones
de apoyo adaptativo/continuidad y ventana acotada, sin restaurar el guard viejo;
despues rebuild y CTest.
Tests 3Q actualizados: los casos de anclaje, error alto y perdida reciente
aportan suficientes KFs independientes para los defaults 2/4/6; el antiguo
test de rechazo previo por distancia ahora exige HOLD sin solver y soporte
adaptativo; el asimetrico exige evidencia solo tras reunir apoyo. No se ha
restaurado el guard `5 m / 20 grados`.
Siguiente accion exacta 3Q: rebuild `orbslam3_multi` y repetir CTest con overlay.
Build 3Q tests adaptativos: `orbslam3_multi` 1/1 correcto, exit 0, 4.89 s.
Log completo conservado y no leido. Siguiente accion exacta: CTest completo de
`orbslam3_multi` con overlays ROS/Servidor cargados.
CTest `orbslam3_multi` intento 3: 8/9 targets; el nuevo HOLD adaptativo pasa,
pero cuatro escenarios de pipeline aun no alcanzan/cierran su flujo esperado.
La suite de grafo/validator permanece correcta. Diagnostico siguiente: revisar
la geometria sintetica de progresion candidate y la persistencia de PriorLoop;
no relajar los defaults ni restaurar expectativas obsoletas sin comprobar la
causa exacta.
Diagnostico/correccion intento 3: el fixture usaba BoW identico en todos los
KFs, de modo que candidate quedaba artificialmente fijo aunque query avanzase;
ahora cada pareja temporal comparte una palabra distintiva. Hallazgo funcional
real: el `ServerLoopGeometric` aceptado solo se persistia durante fusion 3P;
ahora todo commit loop de poses aplica inmediatamente su constraint seleccionada
como `PriorLoop`, tambien en la banda segura sin fusion. Si esa persistencia
falla se informa error explicito y nunca se intenta fusionar.
Siguiente accion exacta 3Q: rebuild y CTest completo de `orbslam3_multi`.
Build 3Q regresion segmentada: `orbslam3_multi` 1/1 correcto, exit 0, 9.43 s;
log completo conservado y no leido. Siguiente accion: CTest completo con overlay.
CTest `orbslam3_multi` intento 5: 9/9 targets correctos, 100 %, incluida
regresion de intervalo hard `[4,7]`, deadband numerica, apoyo adaptativo,
continuidad de perdida, pesos y persistencia de PriorLoop. Siguiente accion:
Checkpoint de reanudacion tras compactacion 2026-08-26: la peticion mas
reciente es continuar la ejecucion 3Q ya autorizada; alcance, umbrales, prueba
y dudas permanecen sin cambios. Tras el CTest 9/9 se eliminaron campos y
razones obsoletas (`hard_corridor_alpha`, `protected_region_rejected` y
`waiting_second_independent_query`), pero esa limpieza aun no se ha recompilado.
Siguiente accion exacta 3Q: repetir busquedas estaticas, `git diff --check` y
comparacion de YAML; despues rebuild y CTest de `orbslam3_multi` antes de
completar la ruta fiducial comun, consenso, builds de integracion y simulacion.
Auditoria estatica posterior a limpieza 3Q: no quedan referencias no
documentales a los simbolos y parametros obsoletos buscados; `git diff --check`
correcto y las copias Servidor/Simulacion de `loop_fusion.yaml` son identicas.
Siguiente accion exacta 3Q: rebuild seleccionado de `orbslam3_multi`.
Build 3Q posterior a limpieza: `orbslam3_multi` 1/1 correcto, exit 0, 1 min
2 s. Log completo conservado por la herramienta y no leido. Siguiente accion
exacta 3Q: repetir CTest completo de `orbslam3_multi` con overlays cargados.
CTest 3Q posterior a limpieza: primer lanzamiento dentro del sandbox no fue
funcional porque CTest no pudo escribir `LastTest.log` en `build/`; repetido
con permiso operativo, 9/9 targets correctos, 100 %, 30.40 s. Siguiente accion
exacta 3Q: eliminar la adaptacion de optimizacion fiducial mediante loop
sintetico y reutilizar la seleccion segmentada mediante una entrada comun real;
anadir la regresion correspondiente antes de integrar Servidor.
Ruta fiducial comun 3Q implementada: `PoseGraphBuilder::BuildSegmented()` es la
seleccion compartida; `BuildLoop()` la alimenta con constraints RANSAC
`CurrentLoop` y `BuildExpandedFiducial()` con el target fiducial real, sin
`LoopTaskComputation` ni geometria sintetica. La expansion solo incorpora
`PriorLoop` server/dependencias confirmadas, mantiene intervalos explicitos y
solo hard queda fixed. Regresion multi-submapa reforzada para exigir problema
`FiducialAbsolute`, ventanas segmentadas y cero `loop_edges`. Siguiente accion
exacta 3Q: compilar `orbslam3_multi` y ejecutar su CTest completo.
Build 3Q selector fiducial comun: `orbslam3_multi` 1/1 correcto, exit 0,
39.3 s. Siguiente accion exacta 3Q: CTest completo con overlays para validar
la regresion multi-submapa y detectar cambios de comportamiento del solver.
CTest 3Q selector fiducial comun intento 1: 8/9 targets; todas las suites salvo
un caso pasan. `AnchorsUnanchoredSubmapAfterAdaptiveIndependentSupport` obtiene
una sola ventana y ningun `PriorLoop` al sustituir el anchor soft del hijo por
fiducial hard. El nuevo selector no esta siguiendo en ese fixture la relacion
padre-hijo hasta el segundo submapa. No se cambia el contrato; siguiente accion
exacta: inspeccionar IDs y representacion de esa dependencia, corregir la
incidencia y repetir build/CTest.
Diagnostico selector fiducial: la arista/dependencia del fixture estaba anclada
en un KF posterior al target y fuera de su intervalo; incluirla reproduciria el
cierre transitivo obsoleto. La regresion ahora exige una sola ventana y ningun
prior en ese caso. Se anadio un caso directo donde una arista
`ServerLoopGeometric` toca el intervalo: debe crear dos ventanas segmentadas,
un `PriorLoop` con soporte/peso reales y cero `CurrentLoop`. Siguiente accion:
rebuild y CTest de `orbslam3_multi`.
Build 3Q regresion de incidencia fiducial: `orbslam3_multi` 1/1 correcto,
exit 0, 12.2 s. Siguiente accion: CTest completo con overlays.
CTest 3Q incidencia fiducial: 9/9 targets correctos, 100 %, 29.89 s. La ruta
fiducial comun queda validada con expansion solo por arista incidente y sin
loops sinteticos. Siguiente accion exacta 3Q: hacer explicita y testeable la
regla de consenso de al menos tres segmentos y cobertura 60 % como refuerzo de
`PriorLoop`, sin convertir ningun KF no hard en fixed.
Consenso 3Q implementado: cobertura = KFs activos del intervalo que son
extremos de `PriorLoop` inter-submapa / KFs activos del intervalo. Una
componente de al menos 3 segmentos donde todos cubren al menos 0.60 multiplica
por 2 los pesos prior (cap 60), sin alterar `fixed`. Parametros ROS/YAML
explicitos y copias sincronizadas. Regresion de tres segmentos anadida con
cobertura minima 0.60, pesos 10->20 y solo KF hard fijo. Siguiente accion:
build y CTest `orbslam3_multi`.
Build 3Q consenso: `orbslam3_multi` 1/1 correcto, exit 0, 1 min 5 s.
Siguiente accion: CTest completo con overlays.
CTest 3Q consenso: 9/9 targets correctos, 100 %, 29.25 s. Quedan validados
selector segmentado loop/fiducial, incidencia server, apoyo adaptativo,
histeresis, pesos efectivos, corredor y consenso 3/60 sin hard artificial.
Siguiente accion exacta 3Q: build seleccionado y CTest completo de
`orbslam3_server`, seguido de integracion `simulacion_dron`.
Build integracion 3Q Servidor: `orbslam3_server` 1/1 correcto, exit 0, 20.3 s.
Siguiente accion: CTest del paquete Servidor con overlays.
CTest integracion 3Q Servidor intento 1: 11/12 tests correctos; unico fallo
`uncrustify` por formato de una llamada `std::max` en la carga del apoyo
adaptativo. No hay fallo funcional. Correccion mecanica exacta indicada por el
linter y repeticion de build/CTest.
Build Servidor tras formato: `orbslam3_server` 1/1 correcto, exit 0, 20.0 s.
Siguiente accion: repetir CTest 12 targets.
CTest integracion 3Q Servidor intento 2: 12/12 correctos, 100 %, 5.65 s,
incluidos linters y contratos de configuracion. Siguiente accion exacta: build
seleccionado y CTest completo de `simulacion_dron`.
Build integracion 3Q Simulacion: `simulacion_dron` 1/1 correcto, exit 0,
1.00 s. Siguiente accion: CTest completo del paquete con overlays.
CTest integracion 3Q Simulacion: 10/10 correctos, 100 %, 8.37 s, incluidos
contratos global map/fiducial y linters. Builds y suites seleccionadas quedan
verdes. Siguiente accion exacta 3Q: leer el workflow de simulacion, recuperar
el comando vigente equivalente a la prueba 213 con trayectoria tipica,
fiduciales reales, Gazebo/RViz2 y logs F3Q; registrar la prueba antes de lanzar.
Prueba 218 preparada para 3Q: YAML absoluto
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
launch `ros2 launch simulacion_dron multi_dron.launch.py` con Gazebo GUI=true,
RViz2 sparse=true, mission GUI=false, ambos grafos/navegadores y telemetria web
false, logs F3 terminal=true, visualizacion fiducial=true durante 5.0 s,
`rawdb_record_enabled=false`; fiduciales reales/spawn vigentes y GT fiducial
funcionalmente OFF por el perfil posterior a Fase 4. Startup 20 s, timeout
900 s, post-scenario 30 s, un retry Gazebo y monitor de recursos. Criterio:
runner 17/17 y todos los goals correctos; optimizaciones 3Q que terminen en
commit seguro, HOLD adaptativo o rechazo justificado sin mover hard ni mantener
backpressure; continuidad de wrappers/RViz2 y revision visual final del usuario.
Siguiente accion exacta: ejecutar prueba 218 y registrar exit/success/log antes
de reducir o analizar.
Prueba 218 finalizada: primer startup murio antes del escenario y la herramienta
uso correctamente su unico retry; el segundo intento completo scenario exit 0,
`[SIM-DONE] success=true` y `[SIM-EXIT-CODE] 0`. Duracion operativa resumida
505 s; `guard_triggered=false`, minimo disponible 4900.6 MiB, servidor RSS max
289.6 MiB, grupo RSS max 2642.5 MiB, RViz max 225.0 MiB y web 0 MiB. Log
completo conservado en `codex/archivos_auxiliares/logs/prueba_218.log` y no
leido. Siguiente accion exacta: generar reducciones por runner/goals, decisiones
3Q/solver/commit, invariantes hard/backpressure, fiduciales reales/visualizacion
y errores; analizar solo artefactos reducidos.
Reduccion runner 218: bloque completo de 196 lineas; 17/17 pasos, 22/22 goals
con `success=true`, `SCENARIO-RUNNER-DONE success=true`, scenario/tool exit 0.
Siguiente accion: reducir exclusivamente marcadores 3Q de evidencia, grafo,
solver, validator, commit, prior y backpressure para clasificar cada intento.
Diagnostico reducido 3Q prueba 218: 14 intentos loop, cero commits. Dos fallan
builder por `loop_submap_interval_too_small`; dos convergen pero degradan
covisibilidad nativa; diez convergen a error final aprox. 0.018-0.050 m y
0.0025-0.0108 rad, pero validator rechaza por
`hard_corridor_displacement_exceeded`. En siete rechazos repetidos el exceso
final queda casi constante 0.0266-0.0268 m / 0.00533-0.00537 rad; otros quedan
0.0336-0.0438 m. Fiduciales absolutos si hacen commits, pero no son commits
loop. Backpressure se libera tras cada intento y no hay hard failure. Conclusion
provisional: 3Q NO CONSEGUIDA aun; la deadband se esta aplicando como limite
absoluto casi-hard sobre KFs intermedios y sigue cancelando las optimizaciones.
Siguiente accion exacta: inspeccionar formula builder/validator del corredor y
corregir su semantica para proteger hard exactos y estructura relativa sin
impedir el movimiento necesario de KFs internos; repetir tests y simulacion.
Auditoria formula corredor 218: `RefreshHardCorridorLocked()` guarda como
referencia la pose vigente tras cada commit fiducial; por ello el exceso inicial
de todos los KFs internos es cero. `OptimizationValidator` rechaza cualquier
movimiento posterior superior a la deadband 0.02 m / 0.00872665 rad, de modo
que trata de facto todo el tramo como casi-hard. Esto contradice el objetivo de
deformar el segmento manteniendo solo los fiduciales inmoviles. Aumentar la
deadband solo desplazaria el mismo fallo. Recomendacion: retirar la guarda
absoluta/deadband de commit para KFs internos y conservar el corredor como
senal de riesgo/apoyo adaptativo; proteger con hard exactos, estructura
temporal/covis/prior, coste mejorado y maximo seguro 0.25/0.15. Eliminar campos
y parametros muertos de deadband para no dejar logica obsoleta.
Autorizacion funcional 3Q: SUSPENDIDA por esta decision material detectada en
la prueba 218.
Dudas abiertas 3Q: confirmar sustitucion de la guarda absoluta del corredor
por las guardas estructurales recomendadas y autorizar repetir build/tests y
trayectoria tipica.
Siguiente accion exacta: esperar confirmacion del usuario; no editar codigo ni
repetir simulacion hasta cerrar esta decision.
Clarificacion posterior a prueba 218: el limite de 2 cm sobre KFs intermedios
fue una interpretacion incorrecta y se retirara. Solo los fiduciales hard son
inmoviles permanentemente. Los KFs normales pueden moverse segun CurrentLoop,
aristas temporales, covisibilidad y PriorLoop. Los KFs ya optimizados pueden
reajustarse, pero una nueva optimizacion se rechaza si pretende moverlos mas de
5 m o 20 grados respecto a su pose optimizada protegida. Cuando al menos tres
segmentos independientes tienen fusiones en al menos el 60 % de sus KFs, esos
segmentos se fijan temporalmente durante ese solve y actuan como autoridad; se
mueve el lado query. No se convierten en hard fiduciales permanentes.
Preparacion 3Q tras diagnostico 218: CERRADA.
Acuerdo cerrado 3Q: si.
Autorizacion funcional 3Q: PENDIENTE de una orden explicita para aplicar esta
correccion y repetir build, tests y trayectoria tipica.
Dudas abiertas 3Q: ninguna.
Siguiente accion exacta: tras autorizacion explicita, retirar la deadband
absoluta y su configuracion obsoleta, restaurar la guarda 5 m/20 grados solo
para KFs previamente optimizados, fijar temporalmente el consenso 3/60,
actualizar regresiones y repetir validacion completa y simulacion.
Autorizacion funcional 3Q tras prueba 218: CONCEDIDA el 2026-08-26 para
actualizar contrato y codigo con la politica aclarada, compilar, ejecutar tests
y repetir la trayectoria tipica completa.
Trabajo activo 3Q: retirar la guarda absoluta de 2 cm; implementar proteccion
5 m/20 grados solo respecto a poses previamente optimizadas; fijar de forma
temporal, no hard persistente, los segmentos de consenso 3/60; limpiar campos,
parametros y tests obsoletos; validar con builds, suites y nueva simulacion.
Dudas abiertas 3Q: ninguna.
Checkpoint de reanudacion tras compactacion 2026-08-26: releido fisicamente el
estado obligatorio y reconciliado con la ultima orden del usuario. La
autorizacion sigue CONCEDIDA y el alcance no cambia. El contrato 3Q ya refleja
la politica aclarada; en codigo ya se retiraron los campos de deadband, se
anadio la guarda de KFs previamente optimizados y el marcado temporal del
consenso, pero faltan cerrar la regresion 3/60, buscar restos obsoletos,
compilar, ejecutar suites y repetir la trayectoria tipica.
Siguiente accion exacta tras reanudacion: adaptar el test de consenso para un
query movil y tres segmentos soporte fijados solo durante el solve; despues
realizar auditoria estatica y compilar `orbslam3_multi`.
Auditoria previa al build de correccion 3Q: regresion de consenso adaptada a un
query movil y tres segmentos soporte; solo `ServerLoopGeometric` es elegible
para formar consenso, no las dependencias soft que tambien se representan como
`PriorLoop`. No quedan simbolos de deadband en codigo/configuracion, ambas
copias YAML son identicas y `git diff --check` es correcto.
Build preparado: ejecutar
`./codex/herramientas/build_selected_packages.sh orbslam3_multi` para validar
tipos, builder, validator y regresiones tras retirar 2 cm e introducir
5 m/20 grados y fixed temporal 3/60. Siguiente accion tras terminar: registrar
exit/paquetes/log antes de cualquier diagnostico o CTest.
Build correccion 3Q intento 1: no inicio colcon; exit 2 porque la herramienta
requiere `--group`. No es evidencia del codigo ni genero un fallo de paquete.
Correccion mecanica: ejecutar
`./codex/herramientas/build_selected_packages.sh --group servidor orbslam3_multi`
y registrar inmediatamente su resultado.
Build correccion 3Q intento 2: `orbslam3_multi` fallo, exit 2, 0/1 paquetes,
53.2 s. El unico error mostrado es mecanico en la nueva expectativa del test:
`RawKeyFrameId` no tiene miembro `submap_id`. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido directamente.
Siguiente accion exacta: usar el campo real de `RawKeyFrameId` en esa lambda,
repetir `git diff --check` y relanzar el mismo build.
Build correccion 3Q intento 3: `git diff --check` correcto, pero colcon no
inicio el paquete porque el sandbox impidio crear
`/home/chenfu/Gazebo/log/servidor/build_*` (`Read-only file system`). No es
evidencia del codigo. Siguiente accion exacta: repetir el mismo build con el
permiso operativo preaprobado para la herramienta de compilacion.
Build correccion 3Q intento 4: `orbslam3_multi` 1/1 correcto, exit 0, 15.9 s.
Log completo conservado en `codex/archivos_auxiliares/colcon_build.log` y no
leido. Siguiente accion exacta: ejecutar el CTest completo de
`/home/chenfu/Gazebo/build/servidor/orbslam3_multi` con overlays ROS/Servidor;
registrar el resultado antes de corregir o integrar otros paquetes.
CTest correccion 3Q `orbslam3_multi`: 9/9 targets correctos, 100 %, 29.09 s,
incluidos `test_fiducial_optimization` y `test_loop_pipeline`. Quedan verdes
las regresiones de movimiento intermedio >2 cm, guarda de revisitados
5 m/20 grados, consenso temporal 3/60, ventanas y soporte adaptativo.
Siguiente accion exacta: compilar `orbslam3_server` para validar carga de
parametros y telemetria F3Q; despues ejecutar sus tests completos.
Build integracion correccion 3Q Servidor: `orbslam3_server` 1/1 correcto,
exit 0, 20.8 s. Log completo conservado por la herramienta y no leido.
Siguiente accion exacta: CTest completo de
`/home/chenfu/Gazebo/build/servidor/orbslam3_server` con overlays cargados.
CTest integracion correccion 3Q Servidor: 12/12 targets correctos, 100 %,
6.59 s, incluidos contratos de configuracion y linters. Siguiente accion
exacta: compilar `simulacion_dron` para instalar y validar la copia YAML de la
nueva politica antes de ejecutar su CTest.
Build integracion correccion 3Q Simulacion: `simulacion_dron` 1/1 correcto,
exit 0, 0.97 s. Log completo conservado y no leido. Siguiente accion exacta:
CTest completo de `/home/chenfu/Gazebo/build/simulacion/simulacion_dron` con
los overlays Dron/Servidor/Simulacion cargados.
CTest integracion correccion 3Q Simulacion: 10/10 targets correctos, 100 %,
8.33 s, incluidos `global_map_config_contract`, fiduciales y linters. Builds y
suites seleccionadas quedan verdes. Siguiente accion exacta: releer el workflow
de simulacion, registrar la prueba 219 equivalente a 218 y ejecutarla con
trayectoria tipica, fiduciales reales, Gazebo/RViz2 y logs F3Q.
Prueba 219 preparada para la correccion 3Q: YAML absoluto
`/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
launch `ros2 launch simulacion_dron multi_dron.launch.py` con Gazebo GUI=true,
RViz2 sparse=true, mission GUI=false, grafos web/telemetria web=false, logs F3
terminal=true, visualizacion fiducial=true durante 5.0 s y
`rawdb_record_enabled=false`. Startup 20 s, post-scenario 30 s, timeout 900 s,
un retry Gazebo y monitor de recursos. Criterio: runner 17/17 y goals
correctos; commits loop cuando la propuesta sea valida; ningun rechazo por la
deadband retirada; hard inmoviles; revisitados dentro de 5 m/20 grados;
consenso temporal si aparece; sin backpressure persistente. Siguiente accion:
ejecutar 219 y registrar exit/success/ruta de log antes de reducir o analizar.
Prueba 219 finalizada: scenario exit 0, `[SIM-DONE] success=true` y
`[SIM-EXIT-CODE] 0`; duracion resumida 508 s, `guard_triggered=false`, minimo
disponible 4702.2 MiB, servidor RSS max 265.5 MiB, grupo RSS max 2596.0 MiB y
RViz max 226.6 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_219.log` y no leido. Siguiente accion
exacta: generar reducciones tematicas de runner/goals, decisiones y commits
F3Q, invariantes hard/revisitados/consenso, backpressure y errores; analizar
solo esos artefactos.
Analisis reducido prueba 219: runner 17/17 y 22/22 goals con exito. Hubo 31
inicios F3Q y 30 resultados: 22 commits loop y 8 descartes (2 ventana pequena,
2 sin mejora, 2 sobre umbral seguro, 1 coste global mayor y 1 degradacion de
covisibilidad nativa). Tres commits fusionaron landmarks post-opt. No aparece
la razon obsoleta de 2 cm, ni movimiento hard, violacion de revisitados,
`blocking_failure`, fatal o guarda de recursos. Cinco commits terminaron en
fusion post-opt. Maximo reajuste medido en KFs
ya optimizados: 1.404495 m / 0.181256 rad, bajo 5 m / 0.349066 rad.
Riesgo residual 219: los commits reencolan KFs movidos; `pending` alcanzo 51 y
quedo un solve iniciado al apagar tras la espera final. La funcionalidad de
optimizar loops queda demostrada, pero no se declara eliminada la presion de
cola/escalabilidad. No se cambia unilateralmente la politica de reencolado
porque no forma parte del mecanismo concreto acordado.
Siguiente accion exacta: actualizar docs de paquete e historial 3Q, conservar
218 y 219 como entradas separadas, ejecutar verificaciones documentales finales
y cerrar 3Q como PARCIAL por el riesgo de cola pendiente de decision posterior.
Revision visual del usuario sobre 219: la esquina inferior izquierda mirando
hacia +Y queda completamente corregida; las dos esquinas derechas corrigen peor,
posiblemente por varios cambios de `map_epoch`; el usuario vuelve a mencionar
una esquina inferior izquierda mejorada solo parcialmente, referencia espacial
que debe aclararse porque contradice la primera frase. Solicita correlacionar
estas zonas con optimizaciones canceladas, motivos y anchors por loop.
Siguiente accion exacta: generar reducciones tematicas de 219 para anchors,
epochs, decisiones F3O/F3Q y fiduciales; cruzar la cronologia con el recorrido
sin leer el log completo y actualizar la conclusion viva de la misma prueba.
Correlacion visual/log 219 completada: no hubo anchors loop ni reanchors; los
cinco anchors fueron fiduciales directos en `(2,0)`, `(1,1)`, `(2,1)`, `(1,2)`
y `(1,3)`. Los cambios `(2,1)`, `(1,2)` y `(1,3)` aparecen en tramos derechos,
donde temporal no cruza epochs. Tras el bloque inicial casi todos los solves
usan query `(2,1)`; `(1,2)` no inicia ninguno y `(1,3)` completa uno. Esto es
consistente con la mejora visual desigual, sin demostrar una causa unica.
Documentacion viva actualizada: contrato 3Q, resumen/historial/indice, docs de
componentes, contexto minimo, estado y ultima sesion. Conclusion 219/3Q:
PARCIAL; no queda implementacion funcional activa, pendiente aclaracion visual
de la esquina repetida y decision del usuario sobre cierre o mejora multi-epoch.
Verificacion final documental: `git diff --check` correcto; no quedan
afirmaciones vigentes del exceso de corredor antiguo en docs de componentes.
Los cambios paralelos de Fase 5 permanecen intactos y no se han mezclado con
la implementacion 3Q. Trabajo activo 3Q: no; espera respuesta del usuario sobre
la referencia de esquina y la decision de cierre o nueva preparacion.
Diagnostico conversacional posterior sobre anchors loop 219: el mecanismo no
esta deshabilitado. Requiere activar una constraint con apoyo independiente y
que su componente contenga ya autoridad world. `(1,1)` activo constraints con
2/2 y 3/2 antes de los primeros fiduciales, pero ambos lados estaban aun sin
anchor y `BuildAnchorCascade()` no podia sembrar world. `(2,1)` y `(1,2)`
recibieron geometria rechazada mientras estaban sin anchor y luego fiducial
directo. `(1,3)` acumulo como maximo 4/6 apoyos en una hipotesis riesgosa y no
activo constraint antes de su fiducial. `BuildAnchorCascade()` se invoca al
activar una constraint, no automaticamente al llegar luego un fiducial.
Trabajo activo: no; explicacion documental sincronizada, sin cambios de codigo.
Checkpoint de reanudacion 2026-08-26: contexto fisico releido y reconciliado
con la pregunta vigente. Confirmado en codigo que `BuildAnchorCascade()` recorre
la componente de `active_constraints_` y puede anclar en lote sus submapas no
anclados cuando ya existe autoridad world, pero solo se invoca desde una nueva
activacion de constraint. `ProcessFiducialObservation()` no dispara esa cascada:
si A y B conservan una constraint relativa sin world y A recibe despues un
fiducial, B permanece sin anchor hasta que otro loop compatible vuelva a activar
el procesamiento. Trabajo activo: no; no se ha modificado codigo.
Preparacion mejora conservadora 3Q: CERRADA. Acuerdo cerrado: si.
Autorizacion funcional: CONCEDIDA el 2026-08-26. Dudas abiertas: ninguna.
Objetivo: conservar el comportamiento estable de 3Q y corregir solo dos huecos:
disparar la cascada/reconciliacion de constraints activas al aparecer nueva
autoridad world y permitir recuperacion reciente con un unico loop cuando la
continuidad sea inequivoca. Fuera de esa banda se conserva el apoyo adaptativo
2/4/6; superar 2 m no impide anclar. Parametros iniciales YAML: 0.50 m,
0.15 rad y recorrido maximo 2.0 m. El anchor de un solo loop no habilita fusion,
scaffold ni propagacion amplia hasta segundo loop independiente o fiducial.
Exclusiones: no cambiar solver, validator, umbrales de fusion ni scheduler
general; la deduplicacion adicional de cola queda supeditada a nueva evidencia.
Prueba acordada: unitarias deterministas, builds/CTest afectados y repeticion
de la trayectoria con fiduciales reales equivalente a 219, comparando anchors,
optimizaciones, descartes, map_epoch, pending y revision visual humana.
Riesgos aceptados: anchor soft provisional mal asociado y trabajo adicional al
llegar world; se contienen mediante continuidad estricta, ausencia de
ambiguedad, commit atomico y sin propagacion/fusion provisional.
Plan: actualizar contrato, localizar simbolos y YAML; implementar y probar el
bloque minimo; compilar/CTest; ejecutar simulacion; reducir logs y documentar.
Siguiente accion exacta: leer resúmenes/MDs de componentes y localizar las
rutas de configuracion, commit fiducial, constraints y tests antes de editar.
Contrato 3Q actualizado con la politica acordada: recuperacion reciente en tres
bandas, parametros YAML 0.50 m/0.15 rad/2.0 m, constraint provisional sin
fusion/scaffold/propagacion, fallback 2/4/6 y cascada atomica al aparecer world.
Archivos criticos confirmados: `loop_pipeline.hpp/.cpp` para evidencia y grafo
de constraints; `sparse_global_backend.hpp/.cpp` para continuidad y commits;
`fiducial_types.hpp` y `global_map_server.cpp` para devolver/reencolar cascada;
las dos copias `config/global_map/loop_fusion.yaml`; tests de loop pipeline y
contratos de Servidor/Simulacion. No se tocara solver, validator, fusion ni cola
general. Bloque conservador 3Q implementado: contexto de perdida y cuatro
parametros YAML; via de un loop con continuidad estricta; constraint provisional
que bloquea fusion y cascada hasta confirmacion; promocion por segundo apoyo;
`BuildAnchorCascade()` acotado a componente y sin provisionales; commits
fiduciales directos/optimizados disparan cascada y devuelven endpoints para
reconciliacion normal. Servidor carga ambas copias YAML, reencola endpoints y
expone `[F3O-WORLD-CASCADE]`/`single_recovery`. Regresiones añadidas para
cascada sin tercer loop, recuperacion cercana 1/1 y fallback sobre 2 m. Solver,
validator, fusion y scheduler general no se han modificado. Siguiente accion
exacta: build aislado de `orbslam3_multi`; registrar resultado y corregir solo
errores mecanicos antes de ejecutar CTest.
Build mejora conservadora 3Q intento 1: `orbslam3_multi` 1/1 correcto, exit 0,
1 min 20 s. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido. Siguiente accion exacta:
ejecutar CTest completo de `orbslam3_multi` con overlays ROS/Servidor; registrar
resultado antes de cualquier correccion o build del servidor.
CTest mejora conservadora 3Q `orbslam3_multi`: 9/9 targets correctos, exit 0,
incluido `test_loop_pipeline` con cascada fiducial, anchor 1/1 cercano y fallback
adaptativo sobre 2 m. No hubo regresiones en raw, pose store, optimizacion,
scalability ni fusion. Siguiente accion exacta: compilar `orbslam3_server` para
validar carga de parametros, telemetria y reencolado de reconciliacion.
Build mejora conservadora 3Q `orbslam3_server`: 1/1 correcto, exit 0, 21.3 s;
log completo conservado y no leido. Siguiente accion exacta: ejecutar CTest
completo de `orbslam3_server` con overlays y registrar el resultado.
CTest mejora conservadora 3Q `orbslam3_server`: 12/12 correctos, exit 0;
configuracion, backpressure, tests funcionales y linters pasan. Siguiente accion
exacta: compilar `simulacion_dron` para instalar/validar la replica YAML y luego
ejecutar su CTest completo antes de preparar la simulacion larga.
Build mejora conservadora 3Q `simulacion_dron`: 1/1 correcto, exit 0, 0.71 s;
replica YAML instalada. Siguiente accion exacta: ejecutar CTest completo de
`simulacion_dron` con los tres overlays y registrar el resultado.
CTest mejora conservadora 3Q `simulacion_dron`: 10/10 correctos, exit 0;
`global_map_config_contract`, fiduciales y linters pasan. Builds y suites quedan
verdes: `orbslam3_multi` 9/9, `orbslam3_server` 12/12 y Simulacion 10/10.
Prueba 220 preparada como repeticion exacta de 219: YAML
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
launch `ros2 launch simulacion_dron multi_dron.launch.py` con Gazebo GUI,
RViz2 sparse, mission GUI/web desactivados, logs F3 terminal, visualizacion
fiducial 5 s y raw record off. Startup 20 s, post-scenario 30 s, timeout 900 s,
un retry Gazebo y monitor de recursos. Criterio: 17/17 pasos, 22/22 goals,
success/exit 0; hard inmoviles; sin regresion de commits loop; medir cascadas,
single recovery, anchors, descartes, epochs y pending. La revision visual final
corresponde al usuario. Siguiente accion exacta: ejecutar prueba 220 y registrar
resultado/ruta del log antes de reducir o analizar.
Prueba 220 finalizada: primer arranque Gazebo murio temprano y el retry previsto
arranco correctamente; escenario exit 0, `[SIM-DONE] success=true` y
`[SIM-EXIT-CODE] 0`. Segundo intento: 505 s, `guard_triggered=false`, minimo
MemAvailable 4443.3 MiB, servidor RSS max 273.7 MiB, grupo RSS max 2634.9 MiB
y RViz max 225.9 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_220.log` y no leido. Siguiente accion
exacta: generar reducciones separadas para runner/goals, single recovery y
cascadas, anchors/epochs, F3Q solves/commits/descartes, pending/invariantes y
errores; analizar exclusivamente esos artefactos.
Revision visual humana prueba 220: resultado general excelente; unico problema
observado, una optimizacion/movimiento de KFs sin sentido que deterioro algo la
esquina superior derecha mirando hacia +Y. Esta observacion sustituye cualquier
cierre visual provisional y debe incorporarse a la entrada de la misma prueba.
Trabajo activo: diagnosticar esa correccion cruzando el paso por la esquina con
solves F3Q, submapas/epochs, ventanas, aristas y magnitudes; no cambiar codigo
hasta aislar causa. Siguiente accion exacta: reducir/leer cronologia F3Q del
intervalo entre pasos 7-13 y luego ampliar solo los patrones que falten.
Checkpoint de reanudacion 2026-08-26: archivo releido fisicamente tras la
compactacion y estado reconciliado con la ultima peticion. La prueba 220 no se
repite ni se modifica codigo: se compararan el commit fiducial `task=6`, el
commit loop `task=1000000005590` y las correcciones fiduciales posteriores
usando solo reducciones tematicas, para atribuir la deformacion de la esquina
superior derecha y actualizar la conclusion viva de esa misma prueba.
Diagnostico visual 220 cerrado: causa principal con alta confianza en
`task=1000000005590`, reencolada por `anchor_revision_changed` tras anclar
`(1,2)`. Fue la unica optimizacion loop que salto de ventanas habituales de
49-75 KFs a 3 submapas/296 KFs y movio 277. Los candidatos consecutivos
65/66/67 dieron error world casi `0/1.0118/0 m`; la seleccion incorporo las
tres regiones compatibles no fusionables como `CurrentLoop`, aunque solo la
constraint central discrepaba. Esto no demuestra que la pose del 66 fuese la
unica incorrecta: puede ser la medida regional o una deformacion distribuida.
El validator acepto porque el deterioro estructural maximo
`0.288825 m/0.041285 rad` y el reajuste de optimizados
`0.326020 m/0.054135 rad` quedan bajo 2 m temporal, 1 m covisible y 5 m/20
grados. La fusion posterior stale no revirtio el commit de poses. `task=6`
respondio en cambio a un error fiducial absoluto real de 1.183 m y el dron 2 ya
tenia control coherente en fiducial 1; no presenta la firma aislada 0/1/0.
Conclusion prueba 220 revisada por decision del usuario: `A REVISAR`. Resultado
global excelente y defecto residual aislado aceptado sin otra correccion ahora.
Trabajo funcional activo: no. Continuar pipeline; reabrir 3Q solo si reaparece
el mismo fallo. Punto de reentrada conservado: mantener las tres medidas como
`CurrentLoop` sobre la ventana completa y corregir enforcement, porque
`OptimizationManager` declara `Converged` tras 160 iteraciones aunque no alcance
`0.05 m/0.03 rad`, mientras el validator admite `0.25 m/0.15 rad`, mejora OR y
degradacion estructural local amplia. No asumir que el 66 sea la unica pose
incorrecta ni cambiar cascada, recuperacion 1/1 o fiduciales.
Peticion de cierre vigente: crear un commit unico con el trabajo pendiente
coherente de 3Q y la reconciliacion documental 5A ya realizada, y publicarlo en
GitHub. Siguiente accion exacta: verificar diff/status/branch/remoto, ejecutar
`git diff --check`, incluir los archivos nuevos de historial Fase 5, crear el
commit y hacer push sin reescribir historia ni revertir cambios.
Verificacion previa al commit: rama `main`, HEAD y `origin/main` en `a44b8b8`;
remoto `origin` correcto; `git diff --check` sin errores. El lote contiene 49
archivos: runtime/tests/config/docs 3Q y reconciliacion documental 5A, incluidos
sus tres historiales nuevos. No contiene logs de prueba. Siguiente accion
exacta: stage completo del lote coherente, commit unico descriptivo y push
normal a `origin/main`.
Cierre Git 2026-08-26: commit principal `b29d7a6`
(`Mejora optimizacion 3Q y prepara fase 5`) creado y publicado correctamente en
`origin/main`, avance `a44b8b8..b29d7a6`, sin force. 3Q queda `A REVISAR` por
decision del usuario y solo se reabrira si reaparece el fallo de enforcement de
la ventana 0/1/0 documentado en prueba 220. Trabajo activo: no. Siguiente accion
exacta: ninguna; continuar con el pipeline cuando el usuario lo solicite.
revisar restos obsoletos/contratos de configuracion y compilar
`orbslam3_server`; despues ejecutar sus tests antes de Simulacion.
Build 3Q PriorLoop/progresion: `orbslam3_multi` 1/1 correcto, exit 0, 19.4 s;
log completo conservado y no leido. Siguiente accion: CTest completo con overlay.
CTest `orbslam3_multi` intento 4: 8/9 targets, 9/12 casos de pipeline. Ya pasan
anclaje adaptativo, HOLD repetido y asimetria; quedan tres expectativas legacy:
rigidez exacta del hijo durante reanchor (el nuevo solver reparte correccion),
diagnostico de continuidad sin detalle de razones y prohibicion global de toda
reevaluacion tras aceptar una pareja (debe omitirse la pareja, no candidatos
nuevos). Siguiente accion: ajustar esas expectativas al contrato vigente y
añadir detalle estable para diagnosticar el caso de perdida reciente.
Build 3Q ajuste contratos pipeline: `orbslam3_multi` 1/1 correcto, exit 0,
5.03 s; log completo conservado y no leido. Siguiente accion: ejecutar primero
`test_loop_pipeline` para obtener la razon exacta del caso de perdida reciente.
Test dirigido pipeline: 2/3 correctos; anclaje adaptativo y optimizacion/fusion
pasan. El caso de perdida queda en `waiting_second_independent_query` para sus
ocho tareas y no alcanza la guarda de continuidad porque mezcla dos propiedades
en una sola regresion. Correccion de test: fijar soporte 2/2/2 solo en ese caso
para aislar y verificar la continuidad espacial; la politica 2/4/6 permanece
cubierta por los tests adaptativos separados y por los defaults runtime.
Regresion de ventana anadida: dos submapas con hard en KF 0/4 y loop en KF 7
deben producir exactamente un intervalo `[4,7]` por submapa y ocho KFs totales;
la arista server incidente en KF 5 entra sin recuperar el tramo anterior.
Siguiente accion exacta 3Q: rebuild y CTest completo de `orbslam3_multi`.
Estado posterior Fase 5: 5B conseguida; preparar conversadamente el bloque
5C+5D+5E+5F sobre el HEAD vigente.
Cualquier implementacion funcional requiere preparación y autorización propias.
```

## Estado vivo

```text
Estado: Fase 2 CONSEGUIDA; Fase 3 CONSEGUIDA; 4A-4F CONSEGUIDAS
Objetivo vigente: revisar y repetir la trayectoria tipica tras convertir los
giros que cruzan/permanecen en ±180 grados a comandos yaw relativos
Preparacion de repeticion post-4E+4F: CERRADA
Acuerdo de repeticion post-4E+4F cerrado: si
Autorizacion funcional de repeticion post-4E+4F: CONCEDIDA el 2026-08-25
Prueba propuesta: trayectoria tipica completa con Gazebo/RViz2, ambos grafos
web apagados y debug visual fiducial activo durante 5 s
Dudas abiertas: ninguna; confirmados tambien los dos giros de salida desde
+180 a -90 y desde -180 a +90, usando respectivamente +90 y -90 relativos
Preparacion del bloque 4E+4F: CERRADA
Acuerdo del bloque 4E+4F cerrado: si
Autorizacion funcional de 4E+4F: CONCEDIDA y consumida el 2026-08-25
Prueba acordada 4E+4F: unitarias/componentes, builds selectivos y trayectoria
tipica completa con Gazebo, RViz2 y ambos grafos web activos; ventanas
fiduciales desactivadas
Preparacion del bloque 4C+4D: CERRADA
Acuerdo del bloque 4C+4D cerrado: si
Autorizacion funcional de 4C+4D: CONCEDIDA para visualizador separado y prueba 207
Preparacion del bloque 4A+4B: CERRADA
Acuerdo del bloque 4A+4B cerrado: si
Autorizacion funcional de 4A+4B: CONCEDIDA y consumida
Bloques acordados: 4A+4B; 4C+4D; 4E+4F; 4G+4H; 4I
Primer bloque propuesto: 4A+4B
Prueba acordada 4C+4D: unitarias/componentes; trayectoria tipica revisada con
Gazebo y RViz2; pipeline_flow apagado y system_architecture activo; smoke corto
posterior con system_architecture apagado; revision visual humana del debug
fiducial por el usuario, sin transferir imagenes por chat
Dudas abiertas 4E+4F: ninguna; modulo/modo ESP32-CAM aplazado a 4I
Debug de prueba 4E+4F: pipeline_flow=true; system_architecture=true;
debug_architecture_telemetry=true en la comprobacion live 211; Gazebo
GUI=true; RViz2=true; ventanas fiduciales=false
Plan activo: modificar las seis transiciones yaw en las copias auxiliar e
instalada, validar contrato, compilar `simulacion_dron` y repetir la trayectoria
tipica completa con el perfil visual acordado
Trabajo activo: si; repeticion visual post-4E+4F autorizada
Siguiente accion exacta: editar ambos YAML de trayectoria, manteniendo
posiciones absolutas y usando yaw relativo +90/0/+90 para dron 2 y
-90/0/-90 para dron 1 en las transiciones alrededor de ±180 grados.
Cambios de trayectoria aplicados en la copia auxiliar y el escenario de
`simulacion_dron`: seis comandos con `absoluto_yaw=false`; dron 2 usa
+90/0/+90 y dron 1 usa -90/0/-90. Posiciones, tiempos y demas comandos sin
cambios. Siguiente accion exacta: comprobar igualdad de copias, contrato YAML
y `git diff --check`; despues compilar solo `simulacion_dron`.
Validacion previa: copias de trayectoria identicas, exactamente seis
`absoluto_yaw=false`, contratos fiducial/config 19/19 y `git diff --check`
correctos. Paquete a compilar: solo `simulacion_dron`. Siguiente accion exacta:
ejecutar build selectivo y registrar el resultado antes de preparar la prueba.
Build de repeticion visual: `simulacion_dron` 1/1 correcto, exit 0. Log completo
de build conservado y no leido. Prueba siguiente: 212, YAML absoluto
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
Gazebo GUI y RViz2 activos; mission GUI, pipeline_flow, system_architecture,
ambos navegadores y telemetria arquitectonica apagados; visualizacion fiducial
activa con 5.0 s y logs F3 en terminal. Timeout 900 s, post-scenario 30 s y
monitor de recursos. Criterio: escenario completo sin giros largos observados
por el usuario, ventanas visibles/aisladas, wrappers y RViz2 continuos y
sincronizacion 4E+4F sin fallos graves. Siguiente accion exacta: ejecutar 212 y
registrar el resultado antes de reducir o interpretar el log.
Prueba 212: detenida por Codex tras confirmacion visual del usuario de que los
drones permanecian en fiducial 2. El runner completo goals del paso 3 y la
espera de 8 s, inicio el paso 5 pero no envio nuevos goals porque el servidor
habia activado `blocking_failure=true` tras
`commit_pose_store_hard_constraint_violation` en task 1000000000461; el mission
gate mantuvo backpressure activo. No es evidencia de fallo de yaw. Ejecucion
interrumpida con Ctrl-C, shell exit 130; cleanup informa `SIM-EXIT-CODE 0` pero
no hubo `SCENARIO-RUNNER-DONE`. Log completo en
`codex/archivos_auxiliares/logs/prueba_212.log`, no leido. Recursos estables,
guard false y minimo 5213.5 MiB. Autorizacion funcional de repeticion:
SUSPENDIDA ante el fallo duro no previsto. Siguiente accion exacta: reducir
solo la tarea 1000000000461 y sus constraints/anchors para precisar la causa;
no repetir ni alterar el gate/GT/backend sin nuevo acuerdo.
Diagnostico reducido 212: la tarea 1000000000461 era un `LoopTask`, no una
orden de trayectoria ni un batch visual. Confirmo tres regiones RANSAC para
query `(1,0,2)`, construyo un grafo de dos submapas/45 KFs/17 controles y el
solver redujo el error, pero el commit rechazo la propuesta porque violaba una
hard constraint ya existente. La salvaguarda evito escritura, pero el servidor
clasifico el rechazo como `F3L-HARD-FAILURE`, fijo `blocking_failure=true` y
mantuvo el mission gate activo. El runner completo paso 3 y espera 4; en paso 5
no envio goals y finalmente marco failed al recibir Ctrl-C. Los seis giros yaw
modificados aun no se alcanzaron. Causa confirmada: loop incompatible con hard
fiducial + politica de fallo bloqueante; no visualizador, yaw, action server,
recursos ni backpressure por cola. Dudas abiertas: decidir entre corregir la
clasificacion/causa del fallo antes de repetir, o ejecutar una prueba
explicitamente solo visual que ignore el mission gate aceptando ese riesgo.
Acuerdo correctivo posterior a prueba 212: CERRADO; el usuario autoriza eliminar
por completo `secondary_blocking_failure_` y cualquier contribucion persistente
equivalente al backpressure. Los fallos duros conservan log y metricas, pero al
terminar una tarea rechazada o fallida se libera el mission gate; una corrupcion
severa se diagnosticara deteniendo manualmente la simulacion. Autorizacion
funcional: CONCEDIDA el 2026-08-25. Prueba acordada: repetir la trayectoria
tipica completa con Gazebo/RViz2, ambos grafos web apagados y visualizacion de
tags durante 5 s. Dudas abiertas: ninguna. Plan: retirar flag, condicion y
telemetria asociada; mantener `hard_failure` como resultado observable; ajustar
tests y documentacion; compilar `orbslam3_server` y `simulacion_dron`; ejecutar
la nueva prueba y analizar solo logs reducidos. Siguiente accion exacta: leer
docs y tests del servidor, aplicar el cambio minimo y su regresion.
Cambio correctivo aplicado: eliminado `secondary_blocking_failure_`, su escritura
en fallos duros, su condicion en `UpdateBackpressure()` y su campo de log. Los
fallos conservan contador/evento y `[F3L-HARD-FAILURE ... action=continue]`.
Anadido contrato pytest del servidor; resultado directo 2/2 y `diff --check`
correcto. Siguiente accion exacta: compilar `orbslam3_server` y registrar el
resultado antes de ejecutar CTest.
Build `orbslam3_server` intento 1: no iniciado, exit 2 de la herramienta porque
se uso por error `--group Servidor` en vez de `--group servidor`. Correccion
operativa mecanica: repetir con el nombre de grupo valido.
Build `orbslam3_server` intento 2: exit 0, 1/1 paquete correcto en 25.8 s.
Log completo de build conservado y no leido. Siguiente accion exacta: ejecutar
CTest del servidor con el overlay cargado; si pasa, actualizar docs de paquete
y preparar la simulacion visual acordada.
CTest servidor intento 1: no iniciado, exit 2 porque `--log-base` se coloco
despues del subcomando `test`. Correccion mecanica: repetir situando la opcion
global antes de `test`.
CTest servidor intento 2: no iniciado funcionalmente, exit 1 por sandbox
read-only al crear `/home/chenfu/Gazebo/log/servidor/test_*`. Repetir la misma
orden con permiso escalado; no hay evidencia de fallo del paquete.
CTest servidor intento 3: permiso automatico agotado antes de iniciar; reintento
escalado posterior correcto. Suite `orbslam3_server` 12/12 targets, 61 tests,
0 errores y 0 fallos (9 skipped). Documentacion de paquete sincronizada con la
politica sin latch persistente. No se recompila `simulacion_dron`: launch y YAML
instalados no cambiaron desde 212. Siguiente accion exacta: registrar y ejecutar
prueba 213 con la trayectoria tipica, Gazebo/RViz2 activos, ambos grafos web
apagados, debug fiducial 5 s y logs F3 visibles; timeout 900 s.
Prueba 213 preparada: YAML absoluto
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
`launch_gazebo_gui=true`, `launch_mission_gui=false`,
`debug_sparse_global_rviz=true`, ambos grafos/navegadores y telemetria web
apagados, `debug_fase3_logs_terminal=true`,
`debug_fiducial_visualization=true`, duracion 5.0 s. Timeout 900 s,
post-scenario 30 s y monitor de recursos. Criterio: runner completo; un hard
failure, si aparece, debe ir seguido de liberacion del backpressure y nuevos
goals; wrappers/RViz2 continuos y revision visual humana de yaw/ventanas.
Siguiente accion exacta: ejecutar 213 y registrar el resultado antes de reducir.
Prueba 213 finalizada: scenario exit 0, `[SIM-DONE] success=true` y
`[SIM-EXIT-CODE] 0`; duracion total 504 s. Recursos estables,
`guard_triggered=false`, minimo disponible 4996.2 MiB, servidor RSS maximo
269.0 MiB y grupo RSS maximo 2623.7 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_213.log` y no leido. Siguiente accion
exacta: reducir por runner/goals, backpressure, optimizacion/hard failures,
visualizadores, procesos y errores; analizar exclusivamente artefactos reducidos.
Analisis 213: 17/17 pasos, 22/22 goals `success=true`, scenario/tool exit 0.
Varias propuestas loop fueron rechazadas, pero cada optimizacion termino con
`optimization_active=false`, publico backpressure false y permitio continuar;
cero `[F3L-HARD-FAILURE]` reales. Visualizacion: 2 READY, 74 PUB y 74 SHOW,
cierres por timeout, sin muerte de wrappers, visualizadores o RViz2. Gazebo 255
solo en cleanup posterior a `SIM-DONE`. Conclusion tecnica: CONSEGUIDA;
pendiente unicamente la valoracion visual humana del usuario sobre los giros y
las ventanas. Documentacion de paquete, historial 4B, resumen de Fase 4,
contexto minimo, estado resumen y ultima sesion sincronizados. Trabajo activo: no; siguiente
accion exacta: recibir la observacion visual del usuario y actualizar la
conclusion viva si aporta matices.
Verificacion final: `git diff --check` correcto; runtime del servidor sin
referencias a `secondary_blocking_failure_` ni campo `blocking_failure=`. El
diff acotado conserva `secondary_hard_failed_` solo como metrica diagnostica.
Revision visual posterior 213: el usuario da por concluidas las subfases 4A-4F,
pero observa que practicamente no hubo correccion loop util y permanecieron
derivas. Analisis reducido: 15 intentos 3Q, seis commits tempranos con ventanas
30-69 KFs y nueve rechazos posteriores con ventanas 288-313 KFs; siete por
`hard_corridor_displacement_exceeded` y dos por
`prior_loop_structure_degraded`. Varias propuestas rechazadas reducian mucho el
error loop, pero creaban 0.000416-0.130115 m de exceso nuevo de corredor; varias
eran ambiguas, hasta 29 competidores y apoyo no independiente. Hipotesis breve:
combinacion de seleccion ambigua/ventanas grandes y validator de corredor
posiblemente sobrerrestrictivo. Prueba 213 marcada `A REVISAR DE NUEVO EN 3Q`
en contrato e historial 3Q; Fase 4 no se reabre. Trabajo activo: no. Siguiente
accion exacta: preparar 4G+4H cuando lo solicite el usuario; al retomar 3Q,
reproducir 213 y correlacionar derivas visibles con cada propuesta/rechazo.
El contexto de reentrada en `subfase_3Q.md` se ha condensado por peticion del
usuario: sintoma, 6 commits/9 rechazos, causa posible y accion futura; el detalle
permanece en `historial_3Q_RESUMEN.md`.
Preparacion 4G+4H: EN_DEBATE. Acuerdo cerrado: no. Autorizacion funcional:
PENDIENTE. Objetivo provisional: interpretar en Servidor los batches 4F como
objetos fiduciales, aplicar rango/fusion/seleccion y conectar un primary visual
al `FiducialAnchorManager`, primero con GT fiducial funcional OFF y retirandolo
solo tras pasar la cadena. Prueba provisional: unitarias de semantica y backend,
builds/CTests selectivos y trayectoria integral con dos drones/tres objetos,
Gazebo/RViz2 y observabilidad a acordar. Dudas abiertas: semantica visual de
`fiducial_visit_id`; thresholds multicara; formula de calidad/pesos; retencion
acotada para secundarios/no aptos de Fase 6; estrategia controlada para validar
residual alto/rollback; momento exacto de retirar GT y perfil visual de prueba.
Trabajo activo: no, solo preparacion conversada. Siguiente accion exacta:
explicar alcance/riesgos y cerrar estas decisiones con el usuario antes de
modificar codigo, launch, YAML, configuracion, tests o ejecutar builds.
Acuerdos parciales posteriores 4G+4H: todos los KFs con primary fiducial deben
llegar a `FiducialAnchorManager`; el manager compara cada uno con sus umbrales
y descarta funcionalmente los coherentes sin crear optimizacion. La fusion usa
como peso base `max(quality_score, epsilon) * sqrt(tag_area/max_tag_area)`.
Servidor conserva una FIFO diagnostica separada con los ultimos 50 KFs
interpretados por dron, incluidos secundarios/no aptos, para uso futuro en
Fase 6; documentar este contrato en `subfase_6S.md` cuando termine el debate.
El wrapper/ORB-SLAM3 envia cada `camera_T_tag` y sus metricas; no recibe
semantica de objeto ni calcula `camera_T_object` o medias SE(3). El Servidor
resuelve `object_T_tag`, calcula distancia/rango y fusiona, aceptando el pequeno
trafico adicional. No se forzara residual alto ni rollback en 4H: se exigira
regresion equivalente a la ruta GT y se diagnosticaran fallos si aparecen.
Prueba integral aceptada: trayectoria tipica con Gazebo/RViz2, primero GT
fiducial compilado pero OFF; tras pasar la cadena visual se elimina por completo
y se repite, con smoke corto separado para grafos web. Dudas pendientes:
delimitar `fiducial_visit_id` sin omitir KFs ni promover cada KF como control;
cerrar fusion multicara robusta porque `quality_score` solo refleja reproyeccion
individual y no garantiza que una cara incoherente tenga score bajo; confirmar
carga C++ de `fiducial_objects.yaml` y thresholds/configuracion asociados.
Preparacion 4G+4H cerrada tras la ultima revision conversada. Acuerdo cerrado:
si. Autorizacion funcional: PENDIENTE. `fiducial_visit_id` agrupa por
`(drone_id,map_epoch,object_id)` mientras no haya un hueco superior al parametro
configurable inicial de 2.0 s; todos los KFs primary se envian igualmente al
manager y se evaluan. Fusion multicara acordada: conservar todas las caras,
peso base por calidad/area y reponderacion robusta por residual geometrico;
thresholds iniciales configurables de 0.15 m y 15 grados; si no queda solucion
estable, no hay anchor funcional para ese object/KF. El rango inclusivo [1,5]
m se comprueba por cada `camera_T_tag`; cualquiera fuera vuelve no apto al
object completo de ese KF, conservando todas las observaciones. Servidor carga
directamente `fiducial_objects.yaml` con `yaml-cpp`; el servicio 4D no expone
semantica de objetos y wrapper/ORB-SLAM3 no la calculan. FIFO reciente: 50 KFs
interpretados por dron, documentada tambien en `subfase_6S.md`. Prueba larga:
Gazebo, RViz2 y ventanas de tags 5 s; grafos web en smoke separado. Exclusiones:
sin GT fiducial funcional, sin cambios de scoring 3R, sin forzar residual alto,
sin reimplementar manager/optimizer ni modificar poses raw. Riesgos aceptados:
una mayoria geometrica erronea con score alto no es desambiguable sin prior;
se rechaza una fusion sin consenso estable. Criterio: tres objetos y ambos
drones recorren la cadena visual exacta, anchors/revisitas se comportan como la
ruta GT, no hay contaminacion ni bloqueo, y tras el primer pase se elimina GT
fiducial por completo y se repite. Dudas abiertas: ninguna. Trabajo activo: no.
Siguiente accion exacta: esperar autorizacion explicita del usuario para editar
los contratos 4G/4H y despues implementar, compilar y probar el bloque acordado.
Contratos 4G+4H actualizados por autorizacion documental del usuario: el MD
corto 4G y su detalle fijan ownership en Servidor, carga `yaml-cpp`, rango por
tag, fusion robusta ponderada, thresholds 0.15 m/15 grados y FIFO de 50 KFs;
4H fija visita visual configurable de 2.0 s sin omitir ningun KF, perfil de
prueba Gazebo/RViz2/ventanas 5 s, smoke web separado y ausencia de residual o
rollback forzado. `subfase_6S.md` conserva el uso futuro de la FIFO solo como
hint reciente. `git diff --check` correcto. Autorizacion funcional de codigo,
build y simulacion: PENDIENTE. Trabajo activo: no. Siguiente accion exacta:
esperar orden explicita para implementar 4G+4H segun los contratos cerrados.
Ejecucion 4G+4H autorizada explicitamente por el usuario el 2026-08-25.
Preparacion: CERRADA. Acuerdo cerrado: si. Autorizacion funcional: CONCEDIDA.
Dudas abiertas: ninguna. Plan activo: localizar integracion/config; implementar
interpretador robusto, FIFO y visitas; unitarias/builds; regresion visual con GT
fiducial OFF; retirar por completo la ruta GT y repetir; smoke web; documentar.
Prueba acordada: trayectoria tipica con 2 drones/3 objetos, Gazebo/RViz2 y
ventanas de tags 5 s, grafos web apagados; smoke web corto separado. Criterio:
anchors/revisitas visuales por KFs exactos, sin GT funcional, sin doble primary,
contaminacion ni bloqueo; trafico medido; segunda regresion tras borrar GT.
Trabajo activo: si. Siguiente accion exacta: leer docs vigentes de los paquetes
afectados y localizar con `rg` los simbolos de configuracion, handoff 4F,
FiducialAnchorManager y retirada GT antes de editar codigo.
Localizacion 4G+4H completada. Cambio funcional inicial aplicado con GT legacy
aun compilado pero `fiducial_sim_enabled=false`: nuevo
`FiducialObjectInterpreter` carga YAML con yaml-cpp, deriva object_T_tag por
cara, agrupa, aplica rango por tag, fusion robusta SE(3), primary determinista,
visitas 2 s y FIFO 50; handoff 4F crea `visual_fiducial` y llama al manager para
cada KF primary. Anade metricas de batches/tags/bytes/rates, parametros launch
y pruebas unitarias. Siguiente accion exacta: ejecutar comprobaciones estaticas
y unitarias directas disponibles; corregir solo fallos mecanicos y despues
registrar el build selectivo de orbslam3_server/simulacion_dron.
Validacion estatica inicial 4G+4H: 31/31 pytest correctos, launch Python valido,
guarda workspace 15/15 y `git diff --check` correcto. La primera guarda detecto
solo `__pycache__` generados por los propios tests; se limpiaron y la repeticion
pasa. Build siguiente: `orbslam3_server` en grupo servidor para compilar nuevo
interpretador, yaml-cpp, integracion y gtest. Si pasa, ejecutar CTest del
servidor y despues compilar `simulacion_dron`. Siguiente accion exacta:
`./codex/herramientas/build_selected_packages.sh --group servidor orbslam3_server`.
Build 4G+4H `orbslam3_server` intento 1: exit 2, 0/1 paquetes; log completo en
`codex/archivos_auxiliares/colcon_build.log`, no leido. La salida del comando
apunta a asignaciones Eigen con initializer-list restantes en el nuevo test;
se reducira el log para confirmar el primer error real antes de corregir.
Siguiente accion exacta: ejecutar `reduce_build_log.sh` y leer solo el reducido.
Diagnostico reducido build intento 1 confirmado: tres lineas del nuevo gtest
asignaban initializer-list a `Eigen::Isometry3d::translation()` (lineas
136/156/186); no hay error de produccion ni dependencia. Correccion mecanica:
usar `Eigen::Vector3d` en las tres. Siguiente accion exacta: repetir el mismo
build selectivo de `orbslam3_server`.
Build 4G+4H `orbslam3_server` intento 2: exit 0, 1/1 paquete correcto en 4.6 s.
Log completo de build conservado y no leido. Siguiente accion exacta: ejecutar
CTest del paquete con overlay servidor para validar el nuevo gtest y regresion
completa; despues registrar resultado antes de compilar Simulacion.
CTest servidor 4G+4H intento 1: las pruebas funcionales pasan, incluidos los
5/5 casos nuevos de `FiducialObjectInterpreter`; unico fallo `uncrustify` por
tres diferencias mecanicas de formato en el nuevo `.cpp`. Resultado global:
13 targets, 1 fallo de lint, sin error funcional. Siguiente accion exacta:
aplicar exactamente el formato exigido, sustituir texto no ASCII del archivo
nuevo y repetir CTest antes de compilar `simulacion_dron`.
CTest servidor 4G+4H intento 2: 73 tests, 0 errores y 0 fallos (12 skipped);
los 5 casos del interpretador, integracion previa y linters pasan. Siguiente
accion exacta: revisar el diff/integracion arquitectonica pendiente y compilar
`simulacion_dron` antes de preparar la primera regresion larga con GT OFF.
Build 4G+4H `simulacion_dron` intento 1: exit 0, 1/1 paquete correcto. Perfiles,
launch y contratos quedan instalados. Siguiente accion exacta: ejecutar CTest
de Simulacion y las guardas estaticas; si pasan, fijar y lanzar la primera
regresion larga con GT fiducial compilado pero funcionalmente OFF.
CTest Simulacion 4G+4H: 85 tests, 0 errores y 0 fallos (8 skipped). Guarda
workspace intento 1: 14/15 por `__pycache__` generados por CTest; retirados
solo esos caches y el temporal pytest. Guarda intento 2: 15/15. `git diff
--check`: correcto. Siguiente accion exacta: preparar la prueba larga 214 con
trayectoria tipica, Gazebo/RViz2 y ventanas 5 s, grafos web apagados y GT
fiducial funcionalmente OFF; registrar el perfil antes de ejecutarla.
Prueba 214 preparada: YAML absoluto
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
Gazebo GUI=true, RViz2=true, mission GUI=false, ambos grafos/navegadores y
telemetria web=false, logs F3 terminal=true, visualizacion fiducial=true durante
5.0 s. El perfil instalado mantiene `fiducial_sim_enabled=false`. Timeout 900 s,
espera posterior 30 s y monitor de recursos. Criterio: escenario completo;
batches visuales exactos interpretados en ambos drones, primary/visitas y
manager sin contaminacion, bloqueo ni uso GT; revision visual humana de RViz2
y ventanas. Siguiente accion exacta: ejecutar 214 y registrar el resultado
antes de reducir o interpretar su log completo.
Prueba 214 finalizada: scenario exit 0, `[SIM-DONE] success=true` y exit global
0; duracion 498 s. Recursos estables, `guard_triggered=false`, minimo disponible
5435.8 MiB, servidor RSS maximo 261.6 MiB y grupo RSS maximo 2601.6 MiB; web
0 MiB como se acordo. Log completo en
`codex/archivos_auxiliares/logs/prueba_214.log`, conservado y no leido.
Siguiente accion exacta: reducir 214 por runner, batches, interpretacion de
objetos/tags, primary/visitas, manager/anchors, GT, visualizadores,
backpressure y errores; leer solo el artefacto reducido.
Analisis parcial reducido 214: cadena visual activa sin GT (`F3E-FID-CONFIG
enabled=false`), 80 batches recibidos/interpretados, 94 tags, 80 primary, cero
no-primary/rejected, tres object_id cubiertos y ambos drones aportando anchors;
manager procesa las 80 observaciones, crea cinco anchors de submapa y el
escenario termina sin bloqueo persistente. Defecto encontrado: batches del
mismo tramo llegan ligeramente fuera de orden y `AssignVisitLocked` abre una
visita nueva siempre que `stamp < last_stamp`; aparecen IDs distintos dentro
de una misma aproximacion. La prueba no se cierra aun. Correccion dentro del
contrato: representar intervalos recientes por clave y asociar cada timestamp
al intervalo situado a <=2 s, ampliando sus extremos; test explicito de llegada
desordenada, rebuild/CTest y nueva regresion antes de retirar GT.
Correccion de visitas aplicada: cada clave conserva intervalos temporales; una
llegada fuera de orden se asigna al intervalo mas cercano si queda dentro de
`fiducial_visual_visit_gap_sec`, ampliando min/max, y solo una separacion real
crea otro ID. Unit test ampliado con orden 10.0, 11.0, 10.5 y 13.1 s. Rebuild
`orbslam3_server`: exit 0, 1/1 paquete correcto en 35.5 s; resultado confirmado
en log reducido porque la primera espera de herramienta vencio antes del cierre.
Siguiente accion exacta: CTest completo del servidor y, si pasa, repetir la
regresion visual con el mismo perfil antes de retirar GT.
CTest servidor tras intervalos: 73 tests, 0 errores y 0 fallos (12 skipped).
Prueba siguiente: 215, repeticion exacta de 214 con mismo YAML, Gazebo/RViz2,
ventanas 5 s, grafos web OFF, recursos monitorizados y GT fiducial funcional
OFF. Criterio adicional: los KFs desordenados de una aproximacion conservan el
mismo visit_id; ninguna visita nueva salvo hueco temporal superior a 2 s.
Siguiente accion exacta: ejecutar 215 y registrar su cierre antes del reducido.
Prueba 215 finalizada: scenario exit 0, `[SIM-DONE] success=true`, exit global
0 y duracion 500 s. Recursos estables, guard false, minimo 5400.5 MiB,
servidor RSS maximo 287.0 MiB y web 0 MiB. Log completo en
`codex/archivos_auxiliares/logs/prueba_215.log`, conservado y no leido.
Siguiente accion exacta: generar reducido tematico 215 y verificar cadena,
cobertura, visitas por timestamps, manager, ausencia GT y fallos antes de
autorizar internamente la retirada fisica del camino GT.
Analisis 215: 73/73 batches interpretados como primary, 83 tags, cero rechazos,
objetos 1/2/3 cubiertos y ambos drones activos; las llegadas desordenadas del
mismo arrival conservan visit_id. Cuatro anchors creados. Derivas reales de
4.64-23.39 m activan el manager; varias tareas existentes terminan
`loop_submap_window_too_small`, con `action=continue` y sin bloqueo persistente.
Se clasifica como evidencia del backend 3H/3L, no fallo de 4G; la cadena visual
entrega exactamente los datos que antes recibia la ruta GT.
Retirada GT fiducial aplicada: eliminados subscription/callback/buffer,
conversion body-camera, parametros/radios legacy, test/header, arista web y
runtime edge. Replays aceptan solo `source=visual_fiducial`; referencias de
tests backend migradas. El servidor ya no carga `calibration_dron.yaml`.
Validacion estatica tras una correccion contractual mecanica: 26/26 pytest.
Siguiente accion exacta: builds/CTests de backend, servidor y Simulacion;
despues repetir trayectoria sin codigo GT y ejecutar smoke web separado.
Builds post-retirada GT: `orbslam3_multi`, `orbslam3_server` y
`simulacion_dron`, 1/1 correctos cada uno. CTest Servidor agregado: 150 tests,
0 errores/fallos (10 skipped); el proceso supero la primera espera, confirmado
por `colcon test-result`. CTest Simulacion: 85 tests, 0 errores/fallos
(8 skipped). Siguiente accion exacta: limpiar solo caches de CTest, guarda y
diff; despues preparar la prueba 216, misma regresion larga ya sin ruta GT.
Guardas post-CTest: dos intentos iniciales detectaron solo caches Python
generados; retirados exclusivamente esos artefactos. Resultado final 15/15 y
`git diff --check` correcto. Prueba 216 preparada con el mismo YAML/perfil de
215: Gazebo/RViz2, ventanas 5 s, grafos web OFF, timeout 900 s, post 30 s y
recursos; el binario/config ya no contienen ruta GT fiducial. Criterio:
resultado equivalente a 215, solo source visual, cobertura y visitas validas,
sin suscripcion GT ni bloqueo persistente. Siguiente accion exacta: ejecutar
216 y registrar el cierre antes de reducir.
Prueba 216 finalizada: scenario exit 0, `[SIM-DONE] success=true`, exit global
0 y duracion 504 s. Guard false, minimo 4617.4 MiB, servidor RSS maximo
272.7 MiB y web 0 MiB. Cleanup necesito SIGTERM/SIGKILL del launch despues de
`SIM-DONE`; se analizara si fue solo cierre de Gazebo/procesos auxiliares. Log
completo en `codex/archivos_auxiliares/logs/prueba_216.log`, no leido.
Siguiente accion exacta: reducido tematico y comprobacion de cadena visual,
ausencia GT, procesos y errores antes del smoke web.
Analisis reducido 216: 52 batches sincronizados y 52 primary; cobertura
object 1/2/3, ambos drones en objetos 1/2 y dron 2 en objeto 3; 52 PUB/SHOW,
cero no-primary/unknown y ninguna referencia runtime GT ni hard failure. Un
anchor inicial y revisitas/optimizaciones visuales observables. Scenario y
herramienta correctos. Gazebo exit 255 y SIGKILL ocurrieron solo durante cleanup
posterior a `SIM-DONE`; el destructor del servidor no alcanzo a imprimir sus
contadores finales, pero toda la mision y espera post finalizaron. Conclusion
funcional de regresion sin GT: CONSEGUIDA. Siguiente accion exacta: smoke 217
con `tray_prueba_211.yaml`, ambos grafos web y telemetria activos, ventanas OFF;
verificar edges visuales actuales y ausencia de la arista GT eliminada.
Smoke 217 finalizado: scenario/tool exit 0, `SIM-DONE success=true`, duracion
105 s, guard false, minimo 5172.7 MiB y web RSS maximo 83.0 MiB. Cierre limpio
con SIGINT. Log completo conservado y no leido. Siguiente accion exacta:
reducir por READY/health/actividad/edges fiduciales/errores; si pasa, actualizar
documentacion de paquete e historiales 4G/4H y ejecutar guardas finales.
Archivos criticos 4E+4F localizados: dos copias de
`orbslam3_msgs/{msg,CMakeLists.txt}`; wrapper
`orbslam3_ros2/src/stereo/stereo-slam-node.{hpp,cpp}`; servidor
`orbslam3_multi/{raw_map_types.hpp,raw_map_database.hpp,raw_map_database.cpp,
sparse_global_backend.hpp, sparse_global_backend.cpp}`;
`orbslam3_server/src/global_map_server.cpp`; perfiles
`config/global_map/runtime.yaml`; grafos y tests de `simulacion_dron`.
APIs confirmadas: worker 4D conserva identidad/metricas; `RawInsertResult`
expone `new_keyframe_ids`; `RawMapDatabase` tiene mutex unico y lookup raw;
`SparseGlobalBackend` devuelve `PrimaryBackendResult`; `WorkerLoop` recibe el
commit raw antes de derivados. No hay dudas funcionales nuevas.
Siguiente accion exacta: aplicar interfaces/publicacion 4E y sidecar de
sincronizacion 4F con pruebas unitarias, antes de cualquier build.
Bloque de cambios 4E+4F aplicado: mensajes replicados; publisher reliable
KeepLast(32) de observaciones validas/ordenadas; helper temporal comun y
camera_T_tag; sidecar pending en RawMapDatabase con capacidad 10 por dron,
FIFO sin TTL, digest, first_arrival_id, validacion y matches en RawInsertResult;
subscription/handoff/logs del servidor; parametro YAML replicado; aristas live
en ambos grafos y pruebas raw/contractuales. Prueba estatica inicial: 38/39;
unico fallo por expectativa textual antigua de EnqueueFiducialJob, corregida
mecanicamente. Replicas msg identicas y git diff --check correcto.
Siguiente accion exacta: repetir contratos y compilar primero orbslam3_msgs
Servidor y Dron de forma aislada.
Contratos 4E+4F tras correccion: 39/39 correctos. Build interfaces Servidor
intento 1: exit 0, orbslam3_msgs 1/1 correcto. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido. Siguiente accion
exacta: compilar orbslam3_msgs de Dron de forma aislada.
Build interfaces Dron intento 1: exit 0, orbslam3_msgs 1/1 correcto. Log
completo conservado y no leido. Siguiente accion exacta: compilar el paquete
wrapper `orbslam3` para validar publisher, transform y mensajes generados.
Build wrapper intento 1: exit 0, orbslam3 1/1 correcto; solo warnings legacy
de ORB-SLAM3/Eigen y cv_bridge. Log completo conservado y no leido. Siguiente
accion exacta: compilar `orbslam3_multi` para validar sidecar y sus unit tests.
Build backend intento 1: exit 0, orbslam3_multi 1/1 correcto. Log completo
conservado y no leido. Siguiente accion exacta: ejecutar CTest del backend;
despues compilar orbslam3_server si las unitarias pasan.
CTest backend intento 1: 0/9 ejecutables arrancan, exit 8; todos fallan antes
de los tests por `liborbslam3_msgs__rosidl_generator_c.so` no localizada. Es
un problema de entorno: el CTest directo no cargo el overlay Servidor, no hay
evidencia de fallo funcional. Siguiente accion exacta: repetir CTest tras
source de `/home/chenfu/Gazebo/install/servidor/setup.bash`.
CTest backend intento 2: 9/9 correctos, incluidas las nuevas pruebas de ambos
ordenes de llegada, FIFO por dron, duplicate/conflict y no reactivacion por
update. Siguiente accion exacta: compilar `orbslam3_server` para validar
subscription, configuracion, logs y handoff fuera del mutex.
Build servidor intento 1: exit 0, orbslam3_server 1/1 correcto. Log completo
conservado y no leido. Siguiente accion exacta: ejecutar CTest del servidor
con el overlay cargado y despues compilar Simulacion.
CTest servidor intento 1: 10/11 correctos; unico fallo `uncrustify` por dos
espacios de sangria en la declaracion de `fiducial_subscriptions_`. Correccion
mecanica aplicada, sin cambio funcional. Siguiente accion exacta: repetir
CTest del servidor.
CTest servidor intento 2: 11/11 correctos. Siguiente accion exacta: compilar
`simulacion_dron` para instalar perfiles, grafos y contratos 4E+4F.
Build Simulacion intento 1: exit 0, simulacion_dron 1/1 correcto. Log completo
conservado y no leido. Siguiente accion exacta: ejecutar CTest de Simulacion y
revisar la guarda de replicas/configuracion antes de preparar la simulacion.
CTest Simulacion: 10/10 correctos; pasan contratos de pipeline_flow,
system_architecture, configuracion global y fiduciales, ademas de linters.
Siguiente accion exacta: ejecutar CTest del wrapper y una verificacion estatica
final; despues preparar el YAML/comando de simulacion tipica acordada.
CTest wrapper: 1/1 correcto. Verificacion final previa: mensajes y perfiles
Servidor/Simulacion identicos; `git diff --check` correcto. Prueba siguiente:
209, usando
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`.
Launch: `multi_dron.launch.py` con Gazebo GUI, RViz2, pipeline_flow y
system_architecture activos; ambos navegadores activos; mission GUI y ventanas
fiduciales desactivadas; telemetria arquitectonica derivada del master activa.
Timeout 900 s, post-wait 30 s y monitor de recursos. Patrones previstos:
`FID-BATCH-PUB|FID-SYNC|FID-SERVER-SUBSCRIBED|F3C-RAW-COMMIT|F3P-FLOW-WEB-READY|SYSTEM-ARCHITECTURE|SCENARIO-RUNNER|SIM-|ERROR|FATAL|process has died`.
Siguiente accion exacta: ejecutar simulacion 209 y registrar su resultado antes
de reducir o analizar el log.
Simulacion 209: finalizada con exit 1; `scenario_runner_node` termino con codigo
1 antes de recorrer el escenario. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_209.log` y no leido. Recursos estables,
`guard_triggered=false`, minimo disponible 5897.8 MiB. Siguiente accion exacta:
reducir el log 209 con marcadores de scenario/YAML/launch y diagnosticar el
primer error real antes de decidir si la repeticion conserva el acuerdo.
Diagnostico reducido 209: `[SCENARIO-RUNNER-ERROR] bad file` porque el YAML se
paso como ruta relativa y el scenario se ejecuta desde otro cwd. Launch,
wrappers, configuracion fiducial, RViz2 y ambos bridges arrancaron; cierre
limpio salvo Gazebo 255 conocido. Correccion operativa mecanica: nueva prueba
210 con la misma trayectoria mediante ruta absoluta y
`debug_fase3_logs_terminal=true` para observar los marcadores FID del servidor.
No cambia comportamiento ni criterio acordado. Timeout 900 s, post-wait 30 s,
GUI/RViz2/ambos grafos activos y debug visual fiducial apagado. Siguiente
accion exacta: ejecutar prueba 210 y registrar resultado antes del analisis.
Simulacion 210: finalizada con scenario exit 0, `[SIM-DONE] success=true` y
`SIM-EXIT-CODE 0`. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_210.log` y no leido. Recursos estables,
`guard_triggered=false`, minimo disponible 5085.9 MiB. Siguiente accion exacta:
reducir el log por scenario, publisher, sincronizacion, grafos, procesos y
errores; despues contrastar conteos y criterios 4E+4F.
Analisis reducido 210: trayectoria tipica completa, 17 pasos y ambos drones
correctos; 68 `FID-BATCH-PUB`, 68 altas pending y 68
`FID-SYNC-MATCHED`, sin evictions, duplicates, conflicts ni rejects; 33 batches
del dron 1, 35 del dron 2, 11 multitag y pico pending observado 7/10. Todos los
batches publicados quedaron consumidos. Pipeline flow arranco live con
23 nodos/41 aristas; system architecture arranco visible pero en `mode=static`
porque la ejecucion omitio `debug_architecture_telemetry:=true`. No es un fallo
de codigo, sino una opcion incompleta de la prueba, y queda pendiente verificar
la arista wrapper-servidor en live.
Checkpoint de reanudacion 2026-08-25: archivo releido fisicamente tras
compactacion y reconciliado con la orden `Sigue con las subfases`. Autorizacion
4E+4F permanece CONCEDIDA, dudas funcionales ninguna. Siguiente accion exacta:
leer el workflow local de simulacion/documentacion, preparar un smoke corto que
produzca un fiducial y ejecutarlo con ambos grafos activos y
`debug_architecture_telemetry:=true`; despues documentar y cerrar 4E+4F.
Prueba siguiente 211: YAML
`codex/archivos_auxiliares/trayectorias/tray_prueba_211.yaml`, formado por
readiness, espera de tracking, el primer tramo tipico de ambos drones hasta
`(0,-10)` y espera final de 8 s. Launch con Gazebo GUI, RViz2, pipeline_flow,
system_architecture, ambos navegadores y `debug_architecture_telemetry=true`;
ventanas fiduciales desactivadas y logs F3 en terminal. Timeout 360 s,
post-wait 5 s y monitor de recursos. Criterio enfocado: scenario correcto,
`SYSTEM-ARCH-READY mode=live`, batches y matches reales sin errores graves.
Siguiente accion exacta: ejecutar prueba 211 y registrar su resultado antes de
reducir o analizar el log.
Simulacion 211: finalizada con scenario exit 0, `[SIM-DONE] success=true` y
`SIM-EXIT-CODE 0`. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_211.log` y no leido. Recursos estables,
`guard_triggered=false`, minimo disponible 5532.3 MiB; duracion observada 90 s.
Siguiente accion exacta: reducir 211 por readiness de grafos, batches,
sincronizacion, scenario, procesos y errores; analizar solo el reducido y
decidir el cierre tecnico 4E+4F.
Analisis reducido 211: `SYSTEM-ARCH-READY mode=live` y pipeline flow live con
23 nodos/41 aristas; servidor suscrito a ambos topics con capacidad 10. El
smoke produjo 18 batches reales (8 dron 1, 10 dron 2), todos recibidos pending
y todos matched; pico pending 5/10, sin evictions, duplicates, conflicts ni
rejects. El conteo bruto 19 de `rg -c` incluia la propia linea de patrones del
reductor; la comparacion `(drone,kf)` confirma 18 publicaciones y 18 matches.
Scenario completo y cierre global correctos; solo Gazebo 255 durante cleanup,
incidencia conocida no funcional. Con 210 completa y 211 live, criterios
tecnicos 4E+4F satisfechos. Siguiente accion exacta: actualizar docs de paquetes,
historiales 4E/4F, estado/pipeline y ultima sesion; ejecutar guardas finales y
cerrar trabajo activo.
Cierre documental 4E+4F completado. Docs de paquetes, contratos, estado,
pipeline e historiales 4E/4F sincronizados. Guardas finales: contratos 39/39,
arquitectura workspace 15/15, replicas de mensajes y runtime YAML exactas y
`git diff --check` correcto. Conclusion: 4E y 4F CONSEGUIDAS. Trabajo activo:
no. Siguiente accion solo tras nueva peticion: preparar conversacionalmente
4G+4H; ese bloque no tiene autorizacion funcional.
Archivos criticos 4C+4D: ORB_SLAM3/{System,Tracking};
orbslam3_ros2/{stereo_slam_node,FiducialDetector,CMake}; dos copias de
orbslam3_msgs; orbslam3_server/{fiducial_config_server,launch,config};
simulacion_dron/{multi_dron,system_architecture,tests}; perfil canonico en
orbslam3_server/config/fiducial_objects.yaml
Diseno localizado: spawner Python + readiness Bool reliable/transient-local;
scenario_runner espera `wait_for_bool`; SDF/texturas runtime desde YAML;
perfil semantico replicado y rendering exclusivo de Simulacion
Cambios completados: fiducial_objects replicado, rendering Gazebo, spawner
Python, readiness transient-local, `wait_for_bool`, trayectoria +-10,
fiduciales GT transitorios +-8.5, metadata system_architecture y prueba 201
Pruebas estaticas directas: 38/38 correctas; py_compile y git diff --check correctos
Paquete a compilar: simulacion_dron (unico con codigo/launch/tests modificados)
Build 4A+4B intento 1: exit 0; simulacion_dron 1/1 correcto
Log completo conservado: codex/archivos_auxiliares/colcon_build.log (no leido)
CTest intento 1: no ejecutado, exit 8; sandbox impidio escribir LastTest.log
fuera de src, sin evidencia de fallo funcional
CTest intento 2: 9/10 correctos; unico fallo flake8 por seis lineas >99 y dos
imports no usados en archivos nuevos; tests funcionales 4/4 correctos
Diagnostico: correccion mecanica de estilo, sin cambio funcional
Build 4A+4B intento 2 tras flake8: exit 0; simulacion_dron 1/1 correcto
Log completo: codex/archivos_auxiliares/colcon_build.log (no leido)
CTest intento 3: 10/10 correctos; fiducial_contract, config, arquitectura,
pipeline_flow y seis linters pasan
Guarda completa intento 1: 14/15; unico fallo paths por cuatro __pycache__
generados por CTest/py_compile dentro de simulacion_dron; sin fallo funcional
Guarda completa intento 2: 15/15, cero fallos
Verificacion de install: spawner/config Simulacion instalados; el nuevo perfil
canonico de Servidor requiere rebuild de orbslam3_server para instalarse
Build Servidor intento 1: exit 0; orbslam3_server 1/1 correcto
Log completo: codex/archivos_auxiliares/colcon_build.log (no leido)
CTest orbslam3_server: 10/10 correctos
Install verificado: perfiles Servidor/Simulacion identicos; spawner ejecutable
Prueba siguiente: 201
YAML: codex/archivos_auxiliares/trayectorias/tray_prueba_201.yaml
Launch: multi_dron con Gazebo GUI=true, RViz2=true, mission GUI=false,
pipeline_flow=false, system_architecture=false, navegadores/telemetria web=false
Timeout scenario: 900 s; espera visual final de escenario 30 s; post-wait 30 s
Patrones: FID-TEXTURE|FID-SDF|FID-SPAWN|SCENARIO-RUNNER|SIM-|RViz|ERROR|FATAL
Simulacion 201: finalizada; scenario exit 0, `SIM-DONE success=true` y
`SIM-EXIT-CODE 0`. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_201.log` (no leido).
Reducido 201: 15/15 texturas verificadas, 3/3 SDF, 3/3 spawn, readiness
recibido, 10/10 goals correctos y scenario success=true. RViz2 arranco y cerro
limpio; ambos grafos web y telemetria arquitectonica quedaron desactivados.
Incidencia: al SIGINT final, `fiducial_spawner.py` intenta `rclpy.shutdown()`
cuando el contexto ya esta cerrado y sale 1; no afecta al escenario pero debe
corregirse mecanicamente. Gazebo mantiene su exit 255 conocido tras cleanup.
Correccion de cierre aplicada: `ExternalShutdownException`/`KeyboardInterrupt`
son cierres normales y `rclpy.shutdown()` solo se llama con contexto activo.
Verificacion directa: py_compile correcto; contratos 38/38 correctos. Un intento
pytest previo no ejecuto tests por tres rutas mal nombradas, sin fallo funcional.
Rebuild tras correccion: simulacion_dron 1/1 correcto. CTest: 10/10 correcto.
Prueba siguiente: 202, smoke sin movimiento; espera readiness y 2 s.
YAML: codex/archivos_auxiliares/trayectorias/tray_prueba_202.yaml
Launch: Gazebo GUI=true, RViz2=true, ambos grafos/navegadores y telemetria
arquitectonica=false. Timeout 180 s; post-wait 0 s.
Patrones: FID-SPAWN|SCENARIO-RUNNER|SIM-|rviz2|Traceback|process has died
Simulacion 202: finalizada; scenario exit 0, `SIM-DONE success=true` y
`SIM-EXIT-CODE 0`. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_202.log` (no leido).
Reducido 202: 15 texturas, 3 SDF y 3 spawn correctos; readiness recibido;
spawner y RViz2 terminan limpiamente; sin traceback ni `FID-SPAWN-ERROR`.
Grafos web/telemetria no arrancan. Solo queda el exit 255 conocido de Gazebo
tras SIGINT. El usuario confirma que la revision visual de 201 fue perfecta.
Trayectoria tipica revisada: cuadrado ±10 con paradas en `(0,±10)` y
`(±10,0)`; copia auxiliar y escenario de paquete identicos; contrato 6/6.
Por decision del usuario no se simula esta variante ahora y se verificara con
4C+4D. Documentacion de paquete, estado e historial sincronizada. 4A y 4B
quedan CONSEGUIDAS.
Verificacion de cierre: `test_fiducial_contract.py` 6/6, `git diff --check`
correcto y guarda de workspace 15/15. No se ejecuto build ni simulacion tras
el ajuste de trayectoria, conforme a la peticion del usuario.
Verificacion final: `git diff --check` correcto; guarda de workspace 15/15;
check de longitud termina correctamente con avisos preexistentes y los detalles
largos 4A/4B ya disponen de contratos/resumen cortos.
Checkpoint de reanudacion tras compactacion: releido fisicamente este archivo y
reconciliado con la peticion vigente; se mantiene preparacion 4C+4D sin cambios
funcionales, builds ni simulaciones.
Hallazgo 4C: ya existen LastKeyFrameInfo/GetLastKeyFrameInfo y comparacion
before/after en GrabStereo, pero no evento explicito one-shot; la imagen del
wrapper tampoco es genericamente la imagen efectiva si System rectifica/resize.
Hallazgo 4D: no existe detector/config server; OpenCV 4.5.4 dispone de aruco y
highgui ya es dependencia; se requieren servicio replicado, nodo independiente,
worker acotado y metadata. El usuario acepta API minima 4C; detector SUBPIX,
IPPE_SQUARE y 3 px; rechazo por reproyeccion pero no por rango 1-5 m; score
lineal inicial; retry 1 s, timeout 2 s, cola 4/drop oldest y las dos pruebas.
Nuevo alcance solicitado: debug opcional por wrapper que muestre durante 5 s
el frame procesado con ROI y tag_id al detectar fiduciales, sin bloquear worker.
Acuerdo visual: una ventana por wrapper; el KF mas reciente reemplaza al actual
y reinicia el contador; todos los tags decodificados se muestran, aceptados en
verde y rechazados/desconocidos en rojo con motivo; duracion parametrizable con
default 5.0 s; cierre y fallo headless aislados del SLAM.
Los candidatos OpenCV no decodificados y sin tag_id quedan excluidos por acuerdo.
Ultima auditoria: el wrapper actual carga fx/fy/cx/cy/bf y dimensiones, pero no
un modelo completo de distorsion ni una calibracion reconciliada con rectificacion
o resize interno de System; 4D necesita imagen, K y distorsion efectivos unidos.
Decision cerrada: calibracion propiedad de Dron; System realiza una unica
rectificacion/resize y entrega en el recibo 4C evento, imagen, K, distorsion,
dimensiones y estado efectivos; el detector no vuelve a corregir. El servicio
fiducial no transporta calibracion. Una guarda impide doble rectificacion.
Documentacion preparatoria sincronizada: contratos cortos y detalles largos de
4C/4D reflejan preparacion cerrada, autorizacion pendiente, API/calibracion,
servicio/worker/detector, valores acordados, pruebas y debug visual completo.
Documentacion de ORB_SLAM3 y orbslam3_ros2 actualizada con el estado parcial.
Autorizacion recibida: el usuario ordena realizar modificaciones y pruebas el
2026-08-24 conforme a todo el acuerdo cerrado, sin ampliar el alcance.
Bloque 4C ORB aplicado: Tracking registra/limpia/consume evento one-shot en
StereoInitialization, CreateNewKeyFrame y resets; System::TrackStereo devuelve
recibo opcional con imagen/K/distorsion/dimensiones efectivas y expone guardas
de rectificacion/resize. Getter permanente eliminado. No cambia politica de KF.
Siguiente accion exacta: compilar ORB_SLAM3 nativo con `cmake --build ... -j4`
antes de adaptar el wrapper al nuevo contrato
Build ORB intento 1: no ejecutado; la ruta documental `src/dron/ORB_SLAM3/build`
no existe. Cache real localizada en `/home/chenfu/Gazebo/build/dron/ORB_SLAM3`.
Correccion mecanica: usar ese build dir aislado con paralelismo `-j4`.
Build ORB intento 2: detenido tras primer error real; LoopClosing requiere el
getter interno Tracking::GetLastKeyFrame en tres rutas IMU/merge. Correccion
mecanica: restaurar solo ese getter interno; System/wrapper siguen usando
exclusivamente el recibo one-shot. Repetir build nativo.
Build ORB intento 3: target completo `all` alcanzo y enlazo libORB_SLAM3 al 55%;
se detuvo deliberadamente al compilar ejemplos upstream no necesarios.
Verificacion target exacto `ORB_SLAM3`: exit 0, 100% construido.
Siguiente accion exacta: crear interfaces de configuracion identicas en las dos
copias orbslam3_msgs y preparar fiducial_config_server independiente
Checkpoint de reanudacion 2026-08-25: releido fisicamente tras compactacion y
reconciliado con la ultima orden `sigue`; autorizacion 4C+4D permanece CONCEDIDA,
sin dudas nuevas ni cambio de alcance.
Bloque interfaces/config server aplicado: las dos copias de orbslam3_msgs
incorporan FiducialTagConfig y GetFiducialConfig con definiciones identicas;
los perfiles Servidor/Simulacion incorporan SUBPIX, IPPE_SQUARE y umbral 3 px;
orbslam3_server incorpora el nodo independiente
`/global_mapping/get_fiducial_config`, validacion del perfil, logs y actividad
arquitectonica opcional, integrado en su launch. Pendiente validar/build.
Siguiente accion exacta: validar sintaxis y replicas; despues implementar el
detector, cliente/worker y debug visual no bloqueante en orbslam3_ros2.
Bloque detector/wrapper aplicado inicialmente: FiducialDetector usa
APRILTAG_36H11, SUBPIX e IPPE_SQUARE, conserva metricas y rechaza por geometria
o reproyeccion; StereoSlamNode consume exclusivamente StereoTrackingReceipt,
valida imagen/calibracion, solicita configuracion con retry 1 s/timeout 2 s,
encola hasta 4 con drop-oldest y ejecuta detector/GUI en hilos separados. La
GUI solo nace con flag activo y reemplaza el KF mostrado durante 5 s por defecto.
Validacion previa: replicas msg/srv identicas, Python de nodo/launch compila y
`git diff --check` correcto. Pendiente compilar para corregir API/ABI real.
Siguiente accion exacta: build aislado de orbslam3_msgs Servidor y Dron.
Build 4C+4D interfaces Servidor intento 1: exit 0; orbslam3_msgs 1/1 correcto,
incluyendo FiducialTagConfig y GetFiducialConfig.
Siguiente accion exacta: build aislado de orbslam3_msgs Dron.
Build 4C+4D interfaces Dron intento 1: exit 0; orbslam3_msgs 1/1 correcto.
Las dos replicas generan correctamente. Siguiente accion exacta: build aislado
del wrapper orbslam3 para validar detector, servicio e hilos contra APIs reales.
Build wrapper 4C+4D intento 1: exit 0; orbslam3 1/1 correcto. Solo advertencias
legacy de ORB-SLAM3/Eigen, sin errores nuevos.
Cableado aplicado: flags visuales atraviesan multi_dron -> generar_dron ->
orbslam_use; perfil fiducial de Simulacion llega al config server; grafo y
metadata incorporan arista runtime `fiducial_config_server_to_wrapper`.
Pruebas contractuales directas: fiducial+arquitectura 16/16 correctas.
Test directo parser intento 1: no recolectado; la shell directa no tenia
orbslam3_msgs instalado en PYTHONPATH. No es fallo funcional; repetir dentro
del build/CTest de orbslam3_server con entorno colcon.
Siguiente accion exacta: rebuild orbslam3 con gtest sintetico y ejecutar CTest.
Rebuild wrapper intento 2: exit 0; orbslam3 1/1 correcto con GTest instalado.
CTest wrapper intento 1: 1 test suite ejecutada, 1/3 casos correcto; unknown tag
se conserva rechazado, pero dos casos PnP fallan como invalid_pose_geometry.
Diagnostico: OpenCV produce soluciones IPPE con z positiva; la copia temporal
`convertTo(cv::Mat(cv::Vec3d))` no escribe de forma fiable en el Vec y deja la
traslacion a cero. Correccion mecanica: convertir a Mat persistente y extraer
sus tres escalares; construir Matx33d explicitamente. Rebuild y repetir CTest.
Rebuild wrapper intento 3 tras correccion IPPE: exit 0, 1/1 correcto.
CTest wrapper intento 2: suite 1/1 correcta; sus tres casos (aceptado con pose
finita, unknown rechazado y escala segun size_m) pasan.
Siguiente accion exacta: build y CTest de orbslam3_server con nodo/config test.
Build orbslam3_server intento 1: exit 0, 1/1 correcto.
CTest servidor intento 1: 10/11 correctos; unico fallo
test_fiducial_config_server no recolecta porque el entorno de ament pytest no
expone orbslam3_msgs Python. Linters y cuatro GTests existentes pasan.
Diagnostico/correccion mecanica: extraer `load_fiducial_config` a modulo Python
puro instalado junto al ejecutable; nodo y test importan la misma funcion sin
necesitar interfaces ROS durante la coleccion. Rebuild y repetir CTest.
Rebuild orbslam3_server intento 2: exit 0, 1/1 correcto.
CTest servidor intento 2: 11/11 correctos; parser canonico/empty/duplicados,
cuatro GTests previos y seis linters pasan.
Siguiente accion exacta: validar launch, build dron_individual y
simulacion_dron; ejecutar sus CTests/guardas.
Validacion sintactica de cuatro launch modificados: correcta.
Build dron_individual intento 1: exit 0, 1/1 correcto; launch con flags
fiduciales instalado. Siguiente accion exacta: build simulacion_dron.
Build simulacion_dron intento 1: exit 0, 1/1 correcto.
CTest simulacion intento 1: 9/10 correctos; unico fallo contractual porque la
lista cerrada de debug.yaml aun esperaba siete claves y detecta correctamente
las dos nuevas. Correccion mecanica: incluir visualizacion y duracion, y fijar
sus defaults false/5.0. Rebuild y repetir CTest.
Rebuild simulacion intento 2: exit 0. CTest intento 2: 9/10; la misma prueba
ya reconoce defaults nuevos pero conserva una segunda asercion legacy que exige
que todo valor, incluida la duracion 5.0, sea false. Ajustar esa asercion para
aplicarla solo a flags booleanos y repetir.
CTest simulacion intento 3: 10/10 correctos; cuatro contratos y seis linters.
Estado previo a simulacion: builds de interfaces Dron/Servidor, ORB target,
wrapper, server, dron_individual y simulacion correctos; CTests especificos
wrapper 1/1, server 11/11 y simulacion 10/10.
Siguiente accion exacta: preparar prueba tipica revisada 203 con Gazebo/RViz2,
pipeline_flow off, system_architecture on, telemetria on y debug fiducial on.
Prueba siguiente: 203, trayectoria tipica revisada completa.
YAML: codex/archivos_auxiliares/trayectorias/
prueba_tipica_rodeo_edificio_dos_fiduciales.yaml.
Launch: Gazebo GUI=true, RViz2=true, mission GUI=false, pipeline_flow=false,
system_architecture web/browser=true, architecture telemetry=true,
debug fiducial=true durante 5 s; timeout scenario 900 s, post-wait 30 s.
Patrones de reduccion previstos: FID-CONFIG|KF-EVENT|FID-TAG|FID-KF|FID-VISUAL|
SCENARIO-RUNNER|SIM-|ERROR|FATAL|Traceback|process has died.
Simulacion 203 intento 1: NO EJECUTADA funcionalmente; los tres arranques
(inicial + 2 reintentos) terminaron antes del healthcheck Gazebo y el script
salio 1. No se inicio scenario_runner. Recursos holgados, guard_triggered=false.
Log completo conservado en codex/archivos_auxiliares/logs/prueba_203.log y no
leido. Siguiente accion exacta: reducir 203 buscando excepcion de launch y
primer error causal; corregir mecanicamente y repetir como nueva prueba 204.
Diagnostico reducido 203: causa unica repetida en los tres intentos,
`fiducial_config_server.py` no encontrado como ejecutable libexec. El archivo
si esta instalado por symlink, pero la fuente tiene modo 0664 y symlink-install
lo deja no ejecutable. Correccion mecanica: chmod +x sobre la fuente, verificar
resolucion ROS y repetir como prueba 204 con la misma configuracion.
Permiso corregido y ejecutable instalado verificado con `test -x`.
Prueba siguiente: 204, repeticion integra de 203 (misma trayectoria, launch,
debug y criterios), conservando 203 como fallo de arranque independiente.
Simulacion 204: arranque Gazebo/launch conseguido, wrappers activos; el
scenario_runner termino inmediatamente con exit 1 y la prueba cerro con exit 1
sin ejecutar la trayectoria. Recursos correctos, guard_triggered=false.
Log completo conservado en codex/archivos_auxiliares/logs/prueba_204.log y no
leido. Siguiente accion exacta: reducir 204 por scenario/config/FID y localizar
el primer error causal antes de repetir con nuevo identificador.
Diagnostico reducido 204: scenario_file relativo no existe desde el cwd del
proceso (`bad file`); error de invocacion, no de codigo. Evidencia parcial
positiva: config server READY con 15 tags, ambos wrappers hacen request/READY,
KF inicial one-shot tiene timestamp_delta=0 y el primer KF preconfig se omite
sin buffering; todos los nodos relevantes cierran limpios.
Prueba siguiente: 205, misma prueba completa usando ruta YAML absoluta
`/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/
prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`.
Simulacion 205: interrumpida manualmente por peticion del usuario tras 214 s
(exit de sesion 130; cleanup del script registra SIM-EXIT-CODE 0). No alcanzo
conclusion de escenario y no debe contarse como prueba completa. Recursos
estables, guard_triggered=false. Revision visual del usuario: RViz2 no mostraba
nada; solicita usar el log parcial actual y parar.
Log completo conservado en codex/archivos_auxiliares/logs/prueba_205.log y no
leido. Siguiente accion exacta: reducir 205, diagnosticar deteccion 4C/4D y el
vacio de RViz2; no lanzar otra simulacion sin nueva conversacion/autorizacion.
Diagnostico 205: 4C/4D detectan tag 202 en ambos drones con z=1.498/1.444 m,
error=0.252/0.206 px y quality=0.916/0.931. Justo al activar HighGUI, ambos
wrappers mueren exit 127 por cargar `/snap/core20/.../libpthread.so.0` y faltar
`__libc_pthread_init`; por eso no vuelven a publicar deltas. Sin deltas que
incorporen esos KFs, el adaptador GT rechaza todos los KFs anteriores como
outside_fiducial_radius, no crea anchors y RViz queda vacio. Correccion
mecanica: sanear rutas `/snap/` del entorno de los nodos ORB en launch, como ya
hace RViz2. No repetir simulacion por peticion vigente del usuario.
Correccion Snap aplicada en orbslam_use.launch.py para mono/stereo; launch
compila, contrato directo 7/7, rebuild dron_individual 1/1 y CTest final de
simulacion_dron 10/10. No se relanza simulacion. Estado tecnico: 4C validada en
runtime; 4D detector/config/worker validados y dos poses reales correctas;
debug visual corregido pero pendiente de verificacion runtime tras el crash.
Siguiente accion exacta: actualizar docs de paquetes e historial vivo 4C/4D,
guardas finales y cerrar como PARCIAL sin ocultar pruebas 203-205.
Checkpoint de reanudacion 2026-08-25 tras compactacion: releido fisicamente
este archivo y reconciliado con la ultima peticion. La simulacion 205 permanece
detenida por orden del usuario y no se lanzara otra. Se usara exclusivamente el
log reducido actual para cerrar el diagnostico, completar la guarda headless,
verificarla con build/tests locales y documentar 4C CONSEGUIDA y 4D PARCIAL.
Guarda headless aplicada: `StereoSlamNode` solo crea el hilo HighGUI si existe
`DISPLAY` o `WAYLAND_DISPLAY`; si no, registra el motivo y mantiene SLAM activo.
Build siguiente: `./codex/herramientas/build_selected_packages.sh --group dron orbslam3`.
Motivo: validar la ultima correccion defensiva del wrapper sin nueva simulacion.
Build wrapper final: exit 0; `orbslam3` 1/1 correcto. El stderr contiene solo
warnings legacy de cv_bridge/ORB-SLAM3/Eigen. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido. Siguiente accion:
ejecutar CTest de `build/dron/orbslam3` y despues cerrar documentacion.
CTest wrapper final: 1/1 suite correcta; los tres casos sinteticos del detector
siguen pasando tras la guarda headless. No se ejecuta otra simulacion. Siguiente
accion exacta: sincronizar contratos, historiales, estado y docs de paquetes;
despues ejecutar guardas finales de texto/workspace.
Documentacion 4C/4D sincronizada en contratos, historiales, estado y paquetes.
Guarda workspace intento 1: 13/16 checks correctos; falla solo porque la guarda
clasifica todo `debug_*` como booleano y no contempla la duracion numerica 5.0,
y por `__pycache__` generados por tests. Correccion mecanica: declarar esa clave
numerica en la guarda y retirar exclusivamente caches generadas; repetir.
Guarda workspace intento 2: 15/15 checks correctos; replicas de interfaces y
config, ownership, paths, visualizers y docs coherentes. Siguiente accion:
auditoria textual final, `git diff --check` y cierre operativo sin trabajo activo.
Verificacion final: guarda workspace 15/15, replicas msg/srv exactas,
`fiducial_config_server.py` ejecutable y `git diff --check` correcto. Conclusion
agregada: 4C CONSEGUIDA; 4D PARCIAL porque la correccion HighGUI/Snap/headless
compila y pasa CTest, pero no se verifico en otra simulacion por orden del
usuario. Trabajo activo: no. Siguiente accion: ninguna hasta nueva peticion.
Nueva autorizacion 2026-08-25: el usuario ordena realizar la correccion ya
aplicada y ejecutar su prueba. No hay cambios funcionales adicionales.
Prueba siguiente: 206, repeticion completa de la trayectoria tipica con ruta
YAML absoluta. Gazebo GUI=true, RViz2=true, mission GUI=false, ambos grafos web
y telemetria=false, debug fiducial=true durante 5.0 s. Timeout 900 s,
post-wait 30 s y monitor de recursos activo. Criterios: ventanas visibles y
autocierre/reemplazo, tags anotados, wrappers vivos despues de HighGUI y deltas
posteriores; RViz2 se observa pero 4D aun no publica detecciones visuales al
Servidor. Siguiente accion exacta: ejecutar `run_simulation.sh --prueba 206`.
Simulacion 206: interrumpida manualmente por peticion del usuario tras 331 s;
sesion exit 130 y cleanup `SIM-EXIT-CODE 0`. No alcanzo conclusion del escenario.
Recursos estables, `guard_triggered=false`, minimo disponible 5802.5 MiB. El
usuario confirma que la ventana fiducial funciono al principio, pero RViz2 dejo
de publicar puntos y solicita comprobar si el wrapper volvio a cerrarse. Log
completo conservado en `codex/archivos_auxiliares/logs/prueba_206.log` y no
leido. Siguiente accion exacta: reducir 206 por HighGUI/crash, lifecycle de
wrappers, detecciones, KFs, deltas y publicacion del servidor antes de concluir.
Diagnostico revisado 206: las ventanas funcionaron y los wrappers publicaron
deltas despues de las primeras detecciones. A los 176-178 s ambos `stereo`
terminaron con exit `-9`; no aparece `libpthread`, OOM ni guarda. El usuario
aclara que el escritorio ofrecio `cerrar ventana/esperar` y eligio cerrar. Esa
accion fuerza `SIGKILL` sobre el proceso propietario de HighGUI y explica el
cese de puntos en RViz2. La separacion por hilo no aisla un cierre forzado del
proceso. Propuesta material pendiente de acuerdo: visualizador ROS separado que
reciba la imagen anotada opt-in; si se fuerza su cierre, ORB-SLAM3 sigue vivo.
Acuerdo y autorizacion 2026-08-25: el usuario acepta esa propuesta y ordena
aplicarla y probarla. Contrato cerrado: topic relativo por dron
`orbslam/fiducial_debug/image`, QoS latest-only y solo con debug activo; el
wrapper anota/publica pero no ejecuta HighGUI; un ejecutable
`fiducial_visualizer` separado posee ventana y temporizador de 5 s. Un cierre
normal o forzado del visualizador no puede terminar `stereo`. Prueba 207 repite
la trayectoria tipica con Gazebo/RViz2, grafos off y debug visual on.
Siguiente accion exacta: editar wrapper, nuevo ejecutable, CMake, launch y tests.
Correccion visual separada aplicada: `stereo` ya no contiene HighGUI ni hilo de
ventana; publica imagen anotada latest-only en
`orbslam/fiducial_debug/image`. El nuevo ejecutable
`fiducial_visualizer` posee HighGUI, procesa eventos en su main executor y
cierra por timeout o boton. Launch lo crea solo con debug activo y mantiene el
saneamiento Snap. Contrato actualizado para impedir regresion. `git diff
--check` correcto; pytest directo no disponible en la shell (`command not
found`), se validara por CTest instalado. Build siguiente:
`./codex/herramientas/build_selected_packages.sh --group dron orbslam3`.
Build orbslam3 visual separado intento 1: exit 2, 0/1 paquetes. Log completo
conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido. Primer
error aparente: API Iron `rclcpp::Time` no ofrece `to_msg()`. Siguiente accion:
reducir el log, confirmar el primer error real y aplicar solo la conversion
mecanica de timestamp compatible.
Reducido confirma un unico error funcional: `rclcpp::Time::to_msg` inexistente;
el resto son warnings legacy. Corregido con campos `sec/nanosec` explicitos y
añadido include estandar del nuevo nodo. Siguiente accion: repetir build
aislado de `orbslam3`.
Checkpoint de reanudacion tras compactacion 2026-08-25: releido fisicamente
este archivo y reconciliado con la ultima orden. La autorizacion para el
visualizador separado y la prueba 207 permanece CONCEDIDA, sin dudas nuevas.
Build orbslam3 visual separado intento 2: exit 0, `orbslam3` 1/1 correcto;
solo warnings legacy de cv_bridge/ORB-SLAM3/Eigen. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido. Siguiente accion
exacta: ejecutar CTest de `build/dron/orbslam3`.
CTest wrapper visual intento 1: no ejecutado; la revision automatica del
permiso para escribir en el directorio de build agoto su plazo. No aporta
evidencia funcional. Siguiente accion: unico reintento autorizado de CTest.
CTest wrapper visual intento 2: 1/1 correcto; los tres casos sinteticos del
detector pasan tras aislar HighGUI. Siguiente accion exacta: build aislado de
`dron_individual` para instalar y validar el launch del visualizador separado.
Build dron_individual visual separado intento 1: exit 0, 1/1 correcto; launch
instalado. Log completo conservado en `codex/archivos_auxiliares/colcon_build.log`
y no leido. Siguiente accion exacta: build de `simulacion_dron` para instalar
el contrato actualizado y ejecutar su CTest.
Build simulacion_dron visual separado intento 1: exit 0, 1/1 correcto. Log
completo conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido.
Siguiente accion exacta: ejecutar CTest de `build/simulacion/simulacion_dron`.
CTest simulacion visual separado intento 1: 10/10 correctos, incluido
`fiducial_contract` y los seis linters. Siguiente accion exacta: verificar el
ejecutable instalado, launch/guardas y actualizar documentacion de paquetes
antes de preparar la simulacion 207.
Verificacion install: `fiducial_visualizer` existe y es ejecutable. Guarda
completa intento 1: 14/15 checks correctos; unico fallo `paths` por cinco
`__pycache__` generados durante CTest. Sin fallo funcional. Siguiente accion:
retirar exclusivamente esos caches y repetir la guarda.
Guarda completa intento 2: 15/15 checks correctos tras retirar solo caches
generados. Siguiente accion exacta: actualizar documentacion vigente de
`orbslam3_ros2` y `dron_individual` con el aislamiento implementado; despues
registrar y ejecutar la prueba 207.
Documentacion de paquetes actualizada: publisher debug latest-only en wrapper,
HighGUI exclusivo del ejecutable separado y cierre forzado aislado de `stereo`.
`git diff --check` correcto.
Prueba siguiente: 207, trayectoria tipica completa con YAML absoluto; Gazebo
GUI=true, RViz2=true, mission GUI=false, ambos grafos web/telemetria=false,
debug fiducial=true y display 5.0 s. Timeout 900 s, post-wait 30 s, recursos
monitorizados. Criterios: visualizador muestra/cierra; tags y deltas continúan;
si el visualizador se cierra o muere, ambos `stereo` permanecen activos y RViz2
sigue recibiendo mapa. Siguiente accion exacta: ejecutar prueba 207.
Simulacion 207: finalizada completa; scenario exit 0, `SIM-DONE success=true`
y `SIM-EXIT-CODE 0`. Recursos estables, `guard_triggered=false`, minimo
disponible 5272.9 MiB. Revision visual del usuario: RViz2 y wrappers funcionaron
con normalidad durante toda la prueba, pero no aparecio ninguna ventana con
tags. Conclusion provisional: aislamiento del wrapper conseguido; salida visual
no conseguida. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_207.log` y no leido. Siguiente accion:
reducir por deteccion, publisher, visualizador, deltas y muertes de procesos.
Diagnostico reducido 207: ambos visualizadores arrancan READY; ambos wrappers
detectan tags 202/204 y publican `FID-VISUAL-PUB`; cada visualizador recibe y
registra `SHOW`, pero 3-4 ms despues registra `CLOSE reason=user_close`. Causa:
`WND_PROP_VISIBLE` devuelve 0 transitorio inmediatamente tras `namedWindow` y
el nodo lo interpreta como cierre humano antes de que la ventana sea pintada.
Correccion mecanica dentro del acuerdo: no aceptar `user_close` hasta haber
observado al menos una vez la ventana visible; conservar timeout de 5 s.
Siguiente accion exacta: aplicar guarda de visibilidad, actualizar contrato,
rebuild/CTest y decidir la prueba corta de verificacion.
Guarda de visibilidad aplicada en `fiducial_visualizer`: `user_close` solo se
acepta tras observar visibilidad; el cero transitorio inicial queda ignorado y
el timeout permanece. Contrato y documentacion de paquete actualizados.
Siguiente accion exacta: rebuild aislado de `orbslam3`, CTest y contrato de
Simulacion antes de una nueva prueba runtime.
Rebuild orbslam3 tras carrera de visibilidad: exit 0, 1/1 correcto; solo aviso
legacy de cabecera cv_bridge. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido. Siguiente accion:
CTest wrapper y rebuild/CTest de `simulacion_dron`.
Validacion tras carrera: CTest wrapper 1/1 correcto; build simulacion_dron
1/1 correcto; CTest simulacion 10/10 correcto, incluido contrato que exige la
guarda `window_was_visible_`. Prueba siguiente: 208, repeticion integra de 207
con igual YAML/launch/criterios para comparar solo la correccion visual.
Siguiente accion exacta: ejecutar prueba 208.
Simulacion 208: finalizada completa; scenario exit 0, `SIM-DONE success=true`
y `SIM-EXIT-CODE 0`. Recursos estables, `guard_triggered=false`, minimo
disponible 5388.2 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_208.log` y no leido. Revision visual
humana de las ventanas aun no comunicada. Siguiente accion: reducido tematico
de publisher/visualizador/procesos y contraste con observacion del usuario.
Diagnostico reducido 208: 2 READY, 80 publicaciones, 79 SHOW, 17 cierres por
timeout, 0 `user_close`, 0 desactivaciones y tags 202/204. Tras el ultimo
timeout ambos wrappers publican deltas unos 57 s mas; no muere ningun `stereo`,
solo Gazebo exit 255 en cleanup. Historial 207/208, contrato 4D, resumen de
fase, estado y ultima sesion sincronizados. Estado 4D: PARCIAL pendiente
exclusivamente de confirmacion visual humana de la prueba 208.
Guarda final tras CTest/documentacion: intento 1 14/15 solo por caches
generados; caches retirados; intento 2 15/15 correcto. `git diff --check`
correcto. Trabajo activo: si, cierre conversacional 4D. Siguiente accion
exacta: recibir la observacion visual del usuario sobre las ventanas de 208 y
actualizar la misma entrada/conclusion; no repetir simulacion mientras tanto.
Revision final 2026-08-25: el usuario acepta la prueba 208 y da 4C+4D por
concluidas. Historial, contratos, pipeline, estado y ultima sesion actualizados;
4D pasa a CONSEGUIDA sin borrar los intentos previos. Preparacion 4E+4F:
EN_DEBATE. Acuerdo cerrado: no. Autorizacion funcional: PENDIENTE. Prueba
acordada: pendiente de conversacion. Dudas abiertas: revisar contrato, codigo
vigente, sincronizacion batch-KF y pruebas antes de pedir autorizacion.
Trabajo activo: si; solo preparacion 4E+4F, sin cambios funcionales.
Inspeccion preparatoria 4E+4F: el wrapper ya conserva en `FiducialJob` la clave
`drone_id/map_epoch`, el evento exacto con `keyframe_id/source_frame_id/stamp`
y todas las metricas 4D. `OrbKeyFrame` raw conserva id/stamp pero no
`source_frame_id`. El servidor tiene subscriptions reliable KeepLast(10),
`PrimaryWorker` compromete raw antes de derivados y el backend expone
`GetRawKeyFrame(RawKeyFrameId)`. Punto seguro: sincronizador acotado en capa
ROS, callback batch + notificacion post raw-commit, sin contaminar backend ni
activar semantica 4G. La conversion temporal actual usa redondeo en debug y
truncado en `FillKeyFrameMsg`; 4E debe unificarla para igualdad exacta o acordar
tolerancia minima. Dudas abiertas: tags validos publicados, QoS, TTL/capacidad,
politica al llenarse, handoff pre-4G y prueba visual de ambos grafos.
Preparacion estatica completada: contratos 4E/4F, docs de wrapper, servidor,
RawMapDatabase y visualizadores contrastados; fragmentos minimos de interfaces
y puntos de commit inspeccionados. `git diff --check` correcto. Propuesta a
debatir: solo detecciones validas; mensaje sin timestamp duplicado; QoS reliable
KeepLast(32); sincronizador ROS con pending 30 s/capacidad global 512 y rechazo
explicito al llenarse; handoff sin semantica ni persistencia backend; prueba
tipica completa con Gazebo/RViz2 y ambos grafos web activos. Siguiente accion:
recibir decisiones del usuario; no modificar codigo/build/simulacion.
Revision conversada 4F: el usuario prefiere una bandeja fiducial muy pequena
integrada en `RawMapDatabase`, consultada al insertar KFs nuevos, y FIFO
drop-oldest en vez de TTL/capacidad 512. Propuesta reconciliada: sidecar
pending, no mezclarlo con OrbMap raw; reutilizar el mutex interno ya existente;
`unordered_map<RawKeyFrameId,batch>` para lookup O(1) y deque solo para orden
de expulsion; al llegar batch se resuelve inmediatamente si el KF ya existe;
al commit delta/full se consulta un hash por cada KF nuevo. `RawMapDatabase`
devuelve matches en `RawInsertResult` y `GlobalMapServer` hace el handoff fuera
del lock; la base no llama a `FiducialManager`. Sin matrices, PnP ni semantica
bajo mutex. Acordado: solo valid=true, header.stamp canonico, QoS reliable
KeepLast(32), handoff sin 4G y prueba tipica con ambos grafos. Dudas abiertas:
capacidad 10 global o por dron, confirmar FIFO sin TTL y ledger reciente
acotado para duplicate/conflict. Autorizacion funcional sigue PENDIENTE.
Acuerdo adicional: capacidad pending 10 por dron; al emparejar y entregar un
batch se extrae inmediatamente y libera hueco, con independencia de que 4G lo
acepte o descarte; sin TTL; FIFO drop-oldest por dron al insertar el undecimo.
Distincion pendiente explicada: otro KF parecido tiene clave distinta y debe
llegar normalmente a 4G; duplicate/conflict significa exclusivamente una
segunda entrega de la misma clave exacta. Sin marca consumida, al haberse
retirado de pending y existir ya el KF raw, esa reentrega se entregaria otra
vez. Propuesta minima: sidecar por KF con digest del batch consumido, sin
conservar el batch ni usar una lista reciente de 32; igual=DUPLICATE,
distinto=CONFLICT. Duda abierta unica sobre sincronizacion: aceptar esta marca
consumida/digest exacta. Autorizacion funcional sigue PENDIENTE.
Clarificacion sobre consumed: no se necesita un bool
`fiducial_batch_consumed` separado. Deltas y full snapshots solo intentaran
resolver `raw.new_keyframe_ids`; un KF existente/actualizado no reactiva el
handoff. El unico estado recomendado es un digest opcional por clave exacta:
su presencia ya significa consumido y protege solo reentregas del batch ROS,
no revisitas de otros KFs. El pending se elimina al match. Snapshots no son la
razon del digest. Duda abierta: confirmacion final de mantener ese digest
opcional minimo para cumplir DUPLICATE/CONFLICT. Autorizacion PENDIENTE.
Auditoria final preparatoria: `RawInsertResult` ya contiene `arrival_id` para
matches resueltos durante un commit, pero `RawMapDatabase` no conserva hoy el
primer arrival por KF; un batch tardio puede recuperar el KF pero no su commit
raw original. Propuesta: metadato interno `keyframe_first_arrival_id`, sin
modificar OrbKeyFrame, para que todo `SynchronizedFiducialBatch` lleve el
arrival exacto. Capacidad pending debe ser parametro YAML configurable, default
10 por dron; medir current/peak/evicted/matched_immediate/matched_pending.
Validaciones sin decision abierta: topic/drone coherentes, batch no vacio,
tags unicos/ordenados, numeros finitos, quality [0,1], timestamp exacto tras
helper compartido y quaternion normalizado. Full snapshot solo resuelve KFs
clasificados como nuevos; updates no reactivan batches. Preguntas finales:
aceptar parametro default 10 y conservar first_arrival_id interno. Sin cambios
funcionales ni autorizacion todavia.
Checkpoint de reanudacion 2026-08-25: archivo releido fisicamente tras la
compactacion y estado reconciliado con la ultima respuesta del usuario. Quedan
aceptados el parametro YAML `fiducial_pending_capacity_per_drone` con default
10 y el metadato interno `keyframe_first_arrival_id` sin modificar
`OrbKeyFrame`. Dudas tecnicas abiertas: ninguna. Preparacion 4E+4F permanece
EN_DEBATE y acuerdo cerrado: no, solo hasta que el usuario confirme el resumen
final; autorizacion funcional: PENDIENTE. Prueba acordada propuesta: unitarias
y de componentes, builds selectivos y trayectoria tipica completa con Gazebo,
RViz2 y ambos grafos web activos; ventanas fiduciales desactivadas. Siguiente
accion exacta: presentar el acuerdo final y esperar su confirmacion, sin codigo,
build ni simulacion.
Confirmacion documental 2026-08-25: el usuario acepta el resumen final y pide
reflejarlo en los MD de 4E y 4F. Preparacion: CERRADA. Acuerdo cerrado: si.
Autorizacion funcional: PENDIENTE. Dudas abiertas: ninguna. Prueba acordada:
unitarias/componentes, builds selectivos y trayectoria tipica completa con
Gazebo, RViz2 y ambos grafos web activos; ventanas fiduciales desactivadas.
Contratos actualizados con mensaje/QoS, sidecar pending por dron, capacidad YAML
10, FIFO sin TTL, digest exacto, first_arrival_id y handoff fuera del mutex.
No se ha modificado codigo, launch, configuración, ni se ha ejecutado build o
simulacion. Siguiente accion exacta: esperar una orden posterior y explícita de
ejecución del bloque 4E+4F.
```

Documentacion preparatoria 4A+4B sincronizada con el acuerdo conversado:
objetos a ±8.5 m, trayectoria a ±10 m, rango fiducial configurable con perfil
inicial 1-5 m, scoring 3R sin cambios, revision visual a cargo del usuario y
ejecucion posterior por bloques. Tambien quedan documentados los grafos de
4E+4F, la retirada de GT fiducial en 4H y la decision de camara aplazada a 4I.

## Acuerdo ejecutado

```text
mi_tfg se conserva como legacy fuera de los grupos
ORB_SLAM3_MULTI y fase45_sandbox se retiraron
orbslam3_msgs se duplica; Servidor es canonico y Dron replica exacta
ADR 0009 gobierna ownership y replicas YAML
cada build selecciona exactamente un paquete
debug oficial: RViz2, pipeline_flow, system_architecture, navegadores,
telemetria arquitectonica y logs F3 activos
todos los flags conservan default false
```

## Implementacion

- grupos fisicos `dron`, `servidor` y `simulacion` con distribucion 5/3/1;
- bases `build/install/log` independientes por grupo;
- YAML por dominio, `use_sim_time` explicito y Xacro conectado a la masa configurada;
- ORBvoc completo preparado en `build/dron/_phase2_resources` e instalado;
- `pipeline_flow` lazy-gated antes de serializar;
- `system_architecture` declarativo, estatico/live y con topic propio;
- guarda de layout, interfaces, dependencias, config, paths, visualizers y docs.

## VALIDACIÓN FINAL

```text
build: 9/9 invocaciones correctas, un paquete por invocacion
CTest: lib_tray 4/4, orbslam3_multi 9/9, orbslam3_server 10/10,
       simulacion_dron 9/9
rebuild de cierre: simulacion_dron 1/1; CTest 9/9
dron_individual: deuda legacy global de linters; archivos tocados correctos
guardas funcionales: todas correctas
guarda final completa: 15 checks, 0 fallos
layout: capturas 1440x900 y 820x1000 inspeccionadas sin solapes
git diff --check: correcto
```

Prueba 199:

```text
smoke debug-off: 5/5 pasos, 4/4 goals, success=true, exit 0
RViz/web 0 MiB, sin marcadores de debug especifico
guard_triggered=false
```

Prueba 200, regresión equivalente a 198:

```text
vuelta oficial: 14/14 pasos, 20/20 goals, success=true, exit 0
RViz2, ambos bridges y navegadores activos
system_architecture live abre con latest_sequence=1
guard_triggered=false; minimo MemAvailable 3873.8 MiB
bridges, RViz2, wrappers y servidor cierran limpiamente
revision visual humana: confirmada correcta por el usuario el 2026-08-24
```

Despues de `SIM-DONE`, `gui_tray_multi` emitio un traceback de shutdown de
`rclpy` y Gazebo termino con el exit 255 conocido. No afectaron al escenario.
El antiguo `ValueError` de cleanup de `system_architecture_bridge` no se
reprodujo.

## Fuente de verdad

```text
codex/pipeline/fase_2_separacion_paquetes/RESULTADO_FINAL_FASE_2.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
codex/pipeline/fase_2_separacion_paquetes/historial/INDEX.md
codex/archivos_auxiliares/logs/prueba_199.reduced.log
codex/archivos_auxiliares/logs/prueba_200.reduced.log
```

La auditoria contra `main` confirma que el commit publicado es `4424a586`.
La implementacion sigue sus decisiones materiales, pero los contratos locales
de 2C-2G deben reconciliar clausulas definitivas eliminadas durante el cierre.
Plan autorizado: layout de referencia, guarda de posiciones, contratos 2C-2G,
build/test/guardas y capturas desktop/viewport estrecho; sin simulacion larga.
Layout implementado en `graph_layout.js`, cargado por `index.html` y aplicado
por `app.js`. El test contractual comprueba cobertura de paquetes y relaciones
espaciales sin fijar pixeles exactos.
Contratos 2C-2G reconciliados con las decisiones definitivas publicadas; 2C
vuelve a distinguir réplica parcial y `deployment profile` completo. Docs de
`simulacion_dron` actualizados con el layout declarativo.
Build siguiente: `./codex/herramientas/build_selected_packages.sh --group simulacion simulacion_dron`.
Motivo: instalar el nuevo asset web y ejecutar la suite contractual del único
paquete afectado, reutilizando los prefijos ya validados de Dron y Servidor.

Build layout 2026-08-24: exit 0; `simulacion_dron` 1/1 correcto.
Log: `codex/archivos_auxiliares/colcon_build.log`.
Siguiente accion exacta: ejecutar CTest de `simulacion_dron` y las guardas.

CTest intento 1: no ejecutado, exit 8 porque el sandbox impidio escribir
`Testing/Temporary/LastTest.log` bajo `build/simulacion`; no es un fallo de test.
CTest intento 2: 9/9 correctos, incluido `system_architecture_contract` con la
nueva guarda de layout.
Guarda completa intento 1: 14 checks correctos y 1 fallo de paths. CTest genero
dos `__pycache__` dentro de `simulacion_dron`; no hay incumplimiento funcional.
Artefactos retirados. Guarda completa intento 2: 15 checks, 0 fallos.
Siguiente accion exacta: capturas web desktop y viewport estrecho, seguidas de
inspeccion visual.

Servidor estatico intento 1: no arranco; el sandbox bloqueo la apertura del
socket local con `PermissionError`. Siguiente accion: repetir con permiso local.
Servidor estatico intento 2: activo en `127.0.0.1:8765`.
Chrome headless intento 1: exit 133 sin captura bajo sandbox.
Chrome headless con permiso: capturas desktop 1440x900 y estrecha 820x1000
generadas e inspeccionadas; contenedores completos, texto legible y sin solapes.
Servidor temporal detenido limpiamente.
Siguiente accion exacta: cierre documental y verificacion final de coherencia.

Cierre documental aplicado. Guarda posterior a toda la documentacion: 15/15.
Asset `graph_layout.js` instalado en el prefijo aislado de Simulacion. No quedan
estados pendientes vigentes y `git diff --check` es correcto.
Guarda tras el checkpoint final: 15/15, 0 fallos.
Commit principal de cierre creado correctamente en `main`.
Push intento 1: rechazado por `non-fast-forward`; `origin/main` contiene cambios
nuevos y no se usara force push.
Fetch correcto: remoto añade `63d677a` y `4424a58`, ambos documentales.
Rebase completado sobre `origin/main`; 26 conflictos documentales resueltos con
autorización explícita conservando las versiones finales ya reconciliadas.
Guarda posterior al rebase: 15/15, 0 fallos.
Commit principal publicado: `d189b0e` en `origin/main`.
Siguiente accion exacta: ninguna; esperar preparación solicitada de 4A.
Fase 4 queda actual, pero ninguna subfase puede ejecutarse sin preparación y
autorización explícitas.
Guarda tras activar Fase 4: 15/15, 0 fallos.

Actualizacion documental Fase 4 2026-08-24: integrado
`Fase_4_completa_4A_4I_muy_detallada.zip` como contrato preparatorio vigente.
`pipeline_fase_4.md`, `pipeline_fase_4_RESUMEN.md` y subfases activas 4A-4I
quedan reconciliadas con Fase 2 ya cerrada. Las antiguas 4J-4L salen del flujo
activo y se conservan en `subfases/legacy/`. El detalle largo importado de 4A,
4B, 4C, 4D y 4G se conserva en `subfases/detalle/`, dejando los contratos
ejecutables cortos conforme a la politica de documentacion. Verificacion:
`git diff --check` correcto y busqueda de frases obsoletas sin coincidencias.
