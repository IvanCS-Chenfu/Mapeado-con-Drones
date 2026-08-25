# 00 - Contexto de compactacion

## Checkpoint vigente 2026-08-25

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
Preparacion 3Q: NO_INICIADA
Acuerdo cerrado 3Q: no
Autorizacion funcional 3Q: PENDIENTE
Prueba acordada 3Q: pendiente de preparacion conversada
Dudas abiertas 3Q: causa exacta de los rechazos tardios, seleccion ambigua,
tamano de ventana y posible sobrerigidez del validator de corredor.
Trabajo activo: no; solo queda fijado el nuevo punto de entrada.
Siguiente accion exacta: preparar 3Q con el usuario antes de modificar codigo o
ejecutar pruebas. Este retorno no reabre 4G+4H ni Fase 4.
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
