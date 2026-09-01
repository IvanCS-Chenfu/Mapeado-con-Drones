# Contexto de compactacion

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
