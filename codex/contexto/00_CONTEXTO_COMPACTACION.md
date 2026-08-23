# 00 - Contexto de compactacion

## Estado vivo

```text
Estado: Fase 3 CONSEGUIDA; Fase 2 en ejecucion
Objetivo vigente: ejecutar integralmente Fase 2 (2A-2G)
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA
Prueba acordada: dos drones completan la vuelta al edificio con RViz2, pipeline_flow, system_architecture, navegadores, telemetria arquitectonica y logs F3 activos; smoke separado con defaults false
Dudas abiertas: ninguna
Trabajo activo: implementacion 2A-2G y validacion estatica completadas; siguiente accion exacta: limpiar artefactos generados y compilar los nueve paquetes, uno por invocacion
```

## Preparacion de Fase 2

Se propone ejecutar Fase 2 completa como un unico trabajo paraguas con
checkpoints internos 2A-2G. La preparacion no autoriza movimientos de paquetes,
cambios de build, YAML, launch, codigo, tests ni simulaciones.

Acuerdo final confirmado:

```text
mi_tfg se conserva en raiz como legacy
ORB_SLAM3_MULTI se retira completamente como correccion de 3X/3T
orbslam3_msgs se duplica; Servidor es canonico y Dron replica exacta
ADR 0009 gobierna ownership y replicas de YAML
el build se ejecuta paquete a paquete, con un unico paquete por invocacion
la prueba oficial usa dos drones y debug activo
system_architecture se crea en modo estatico y vivo, separado de pipeline_flow
RViz2, ambos visualizadores, navegadores, telemetria y logs F3 se activan en la prueba oficial
los flags conservan default false y un smoke separado valida su desactivacion
```

Checkpoint previo a Fase 2:

```text
f725edc checkpoint: consolidar estado tras cierre de fase 3
```

Plan autorizado:

```text
1 inventario y 2A: grupos, movimientos y replica orbslam3_msgs
2 2B-2C: herramientas/builds aislados y configuracion ADR 0009
3 2F-2G: system_architecture y guardas automaticas
4 build/test de exactamente un paquete por invocacion y recursos limitados
5 smokes y vuelta oficial de dos drones con debug visual activo
6 documentacion 2A-2G y conclusion tecnica pendiente de revision visual del usuario
```

Inventario previo 2A:

```text
colcon descubre ORB_SLAM3, dron_individual, fase45_sandbox, lib_tray, mi_tfg,
orbslam3, orbslam3_msgs, orbslam3_multi, orbslam3_server y simulacion_dron
los ocho arboles activos que se moveran conservan 2300 archivos regulares
unica ruta fuente rigida detectada: orbslam3_ros2/CMakeModules/FindORB_SLAM3.cmake
launch/runtime restantes localizan recursos mediante ament index
```

Resultado 2A:

```text
dron descubre ORB_SLAM3, dron_individual, lib_tray, orbslam3 y orbslam3_msgs
servidor descubre orbslam3_msgs, orbslam3_multi y orbslam3_server
simulacion descubre simulacion_dron
las dos copias de orbslam3_msgs son identicas segun diff -qr
mi_tfg y fase45_sandbox permanecen fuera de los grupos
```

Checkpoint de reanudacion tras compactacion:

```text
2A: arbol fisico dron/servidor/simulacion y replica orbslam3_msgs completados
2B-2C: herramientas de build/simulacion agrupadas, FindORB_SLAM3 y YAML segun ADR 0009 adaptados
2F-2G: system_architecture estatico/vivo y check_workspace_architecture.py implementados
verificacion fuente: 21/21 tests Python, py_compile, bash -n, guardas paths/config y git diff --check correctos
pendiente: captura desktop/mobile del visualizador, builds y tests paquete a paquete, smoke, prueba oficial y documentacion
```

Resultado de validacion estatica de `system_architecture`:

```text
bridge fuente: [SYSTEM-ARCH-READY], modo static, 3 grupos y 9 paquetes
endpoint /health: ready, telemetry_enabled=false
capturas Chrome headless: desktop 1440x900 y movil efectivo 500x844 correctas
se corrigieron contraste de etiquetas, ajuste responsive de controles y encuadre al redimensionar
bridge temporal cerrado limpiamente
```

Build limpio autorizado:

```text
artefactos /home/chenfu/Gazebo/{build,install,log} eliminados completamente
orden Dron: orbslam3_msgs, lib_tray, ORB_SLAM3, orbslam3, dron_individual
orden Servidor: orbslam3_msgs, orbslam3_multi, orbslam3_server
orden Simulacion: simulacion_dron
recursos: sequential executor, CMAKE_BUILD_PARALLEL_LEVEL=1 y MAKEFLAGS=-j1
siguiente comando: ./codex/herramientas/build_selected_packages.sh --group dron orbslam3_msgs
```

Resultados de build acumulados:

```text
Dron/orbslam3_msgs: exit 0, 1 paquete terminado en 1min 58s
Dron/lib_tray: exit 0, 1 paquete terminado en 16.0s
Dron/ORB_SLAM3: exit 0, 1 paquete terminado en 30min 46s; sin target install, warnings upstream de Eigen/realsense2
Dron/orbslam3: exit 0, 1 paquete terminado en 2min 38s; ruta relativa ORB_SLAM3 y enlace correctos, warnings upstream/cv_bridge
Dron/dron_individual: exit 0, 1 paquete terminado en 1min 17s
Servidor/orbslam3_msgs: exit 0, 1 paquete terminado en 37.3s
Servidor/orbslam3_multi: exit 0, 1 paquete terminado en 1min 34s
Servidor/orbslam3_server: exit 0, 1 paquete terminado en 33.2s
Simulacion/simulacion_dron: exit 0, 1 paquete terminado en 1min 34s
log completo: codex/archivos_auxiliares/logs/colcon_build.log
siguiente accion: verificar arboles build/install/log, replicas y ejecutar tests por paquete
```

Conclusion de build limpio:

```text
9/9 invocaciones terminan con exit 0 y exactamente un paquete cada una
las bases separadas dron/servidor/simulacion se usan correctamente
no fue necesario reducir ningun log por fallo
```

Verificacion posterior al build:

```text
build/install/log contienen exclusivamente dron, servidor y simulacion
diff -qr dron/orbslam3_msgs servidor/orbslam3_msgs: exit 0
declaran CTest: ambos orbslam3_msgs, lib_tray, orbslam3, dron_individual, orbslam3_multi, orbslam3_server y simulacion_dron
ORB_SLAM3 no declara CTest
siguiente prueba: CTest Dron/orbslam3_msgs
```

```text
CTest Dron/orbslam3_msgs intento 1: no ejecutado, exit 8 por sandbox al crear build/dron/orbslam3_msgs/Testing/Temporary
CTest Dron/orbslam3_msgs intento 2: exit 0, 0 tests declarados
CTest Dron/lib_tray intento 1: exit 8, 3/4; cppcheck/lint_cmake/xmllint pasan, uncrustify falla por divergencia previa en 8 archivos
siguiente accion: aplicar ament_uncrustify --reformat, reconstruir solo lib_tray y repetir CTest
```

```text
ament_uncrustify --reformat: 8 archivos corregidos mecanicamente
rebuild Dron/lib_tray: exit 0, 1 paquete terminado en 2.85s
CTest Dron/lib_tray intento 2: exit 0, 4/4
CTest Dron/orbslam3: exit 0, 0 tests declarados
siguiente prueba: CTest Dron/dron_individual
```

```text
CTest Dron/dron_individual intento 1: exit 8, 1/6; cppcheck pasa
xmllint: fallo atribuible a orden invalido introducido en package.xml, corregido
flake8/pep257/uncrustify/lint_cmake: deuda legacy extensa fuera del alcance funcional de Fase 2
ament_xmllint local tras correccion: no concluyente por bloqueo de red al XSD remoto
xmllint focal repetido: exit 0, 1/1
archivos tocados: generar_dron.launch.py flake8 pasa; control_calcular_fuerzas.cpp uncrustify pasa tras reformateo focal; CMakeLists.txt lint_cmake pasa tras corregir un espacio
rebuild Dron/dron_individual: exit 0, 1 paquete terminado en 36.1s
CTest Servidor/orbslam3_msgs: exit 0, 0 tests declarados
CTest Servidor/orbslam3_multi: exit 0, 9/9
CTest Servidor/orbslam3_server: exit 0, 10/10
CTest Simulacion/simulacion_dron: exit 0, 9/9; incluye contratos pipeline_flow, global_map y system_architecture
siguiente accion: ejecutar guarda arquitectonica y preparar smoke con defaults false
```

Conclusion de tests compilados:

```text
lib_tray 4/4, orbslam3_multi 9/9, orbslam3_server 10/10 y simulacion_dron 9/9
ambos orbslam3_msgs y orbslam3 declaran 0 tests; ORB_SLAM3 no declara CTest
dron_individual conserva deuda legacy global de linters; package.xml y los archivos tocados pasan comprobaciones focalizadas y el rebuild termina exit 0
```

Guarda y smoke:

```text
check all: layout/interfaces/dependencies/config/visualizers pasan; paths limpio tras retirar __pycache__; docs pendiente hasta cierre
smoke runtime creado: simulacion_dron/config/scenarios/smoke_debug_desactivado.yaml
rebuild Simulacion/simulacion_dron: exit 0, 1 paquete terminado en 0.90s
```

Prueba 197 preparada:

```text
objetivo: smoke funcional con los siete flags de debug en default false
YAML: /home/chenfu/Gazebo/install/simulacion/simulacion_dron/share/simulacion_dron/config/scenarios/smoke_debug_desactivado.yaml
launch: ros2 launch simulacion_dron multi_dron.launch.py
startup: 30s; timeout scenario: 300s; post-scenario: 20s; monitor recursos activo
criterio: success=true y ausencia de procesos/marcadores visuales o F3 condicionados por debug
siguiente accion: ejecutar ./codex/herramientas/run_simulation.sh --prueba 197
```

Resultado bruto prueba 197:

```text
run_simulation exit 0; scenario exit 0; success=true
duracion monitor 132s; 103 muestras; guard_triggered=false
min MemAvailable 5494.0 MiB; max grupo RSS 869.7 MiB
max RViz RSS 0.0 MiB; max web RSS 0.0 MiB
log completo preservado y no leido: codex/archivos_auxiliares/logs/prueba_197.log
siguiente accion: reducir prueba 197 para comprobar pasos, debug inactivo, errores y cleanup
```

Conclusion prueba 197:

```text
smoke CONSEGUIDO: 5/5 pasos, 4/4 goals, scenario y run success=true
sin marcadores SYSTEM-ARCH, PIPELINE-FLOW ni F3; RViz/web RSS y PSS 0.0 MiB
unico ERROR: Gazebo exit 255 posterior a SIM-DONE durante cleanup conocido
reducido: codex/archivos_auxiliares/logs/prueba_197.reduced.log
```

Prueba 198 preparada:

```text
objetivo: vuelta oficial de dos drones con debug visual completo activo
YAML: /home/chenfu/Gazebo/install/simulacion/simulacion_dron/share/simulacion_dron/config/scenarios/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
launch: ros2 launch simulacion_dron multi_dron.launch.py con los siete debug_*:=true
startup: 30s; timeout scenario: 900s; post-scenario: 60s; monitor recursos activo
criterio tecnico: scenario success, RViz2 y ambos web vivos, bridges ready, telemetria system_architecture y sin error funcional
criterio visual final: pendiente de revision del usuario
siguiente accion: ejecutar ./codex/herramientas/run_simulation.sh --prueba 198
```

## Resultado final

La numeracion activa de cierre es:

```text
3Q optimizacion -> 3R scoring -> 3S debug -> 3T limpieza/handoff
```

- 3R conserva el scoring raw/fused antes identificado como 3S;
- 3S incorpora `simulacion_dron/config/fase3_debug.yaml`;
- 3T es la limpieza/handoff ejecutada inicialmente como 3X;
- los historiales de auditorias provisionales se preservan en
  `historial/absorbidas/`;
- los marcadores de scoring vigentes son `[F3R-*]`; los logs historicos
  192-195 conservan `[F3S-*]`.

3Q queda conseguida para el cierre de Fase 3. La deformacion de la prueba 194
no se borra y permanece en su historial. La prueba 195 no la reproduce y el
usuario confirma el resultado visual correcto. Como mejora futura no
implementada, un candidato cercano podria exigir dos apoyos independientes y
uno lejano o ambiguo hasta 8-10 antes de una unica optimizacion.

## Implementacion 3S

`fase3_debug.yaml` define, todos inicialmente a false:

```text
fase3_rviz2
fase3_grafo_web
fase3_abrir_navegador_web
fase3_logs_terminal
```

`multi_dron.launch.py` valida el YAML, expone overrides homonimos y condiciona
RViz2, bridge y navegador. El navegador requiere tambien el grafo. El launch
del servidor acepta `log_level`; con logs false recibe `error`, por lo que se
ocultan los diagnosticos `[F3*]` sin ocultar errores reales. El procesamiento,
las colas y las publicaciones funcionales no se desactivan.

## Verificacion

- `py_compile` y flake8 de los launches tocados: correctos;
- contratos configuracion/web: 15/15; repeticion final 15/15 en 0.19 s;
- build `orbslam3_multi orbslam3_server simulacion_dron`: 3/3, exit 0;
- CTest: multi 9/9, servidor 10/10 y simulacion 8/8;
- launch instalado: cuatro argumentos 3S presentes con default false;
- prueba 196: `success=true`, cinco pasos y cuatro goals correctos;
- servidor operativo, backpressure observado y cierre limpio;
- cero marcadores `[F3*]` y ningun proceso RViz2/bridge/navegador;
- RSS maximo RViz/web 0.0 MiB, servidor 99.2 MiB y guarda inactiva;
- unico ERROR: exit 255 conocido de Gazebo durante cleanup posterior a
  `SIM-DONE`, sin impacto funcional.
- cierre documental: `git diff --check` correcto y sin referencias activas a
  `subfase_3X`, `historial_3X` ni marcadores actuales `[F3S-*]`.

El log completo `prueba_196.log` se preserva y nunca se leyo directamente. El
analisis uso exclusivamente su reducido y el resumen de recursos.

## Handoff

Fuente de verdad del resultado:

```text
codex/pipeline/fase_3_sparse_global/RESULTADO_FINAL_FASE_3.md
codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md
codex/pipeline/fase_3_sparse_global/historial/INDEX.md
```

La fase actual es Fase 2, separacion servidor/dron/simulacion. No queda trabajo
activo de Fase 3. No se creo un commit final; existe el checkpoint previo a la
limpieza:

```text
1b96a7a checkpoint: guardar estado de fase 3 antes de limpieza 3X
```

## Cambios ajenos preservados

No tocar ni atribuir a Fase 3 los cambios previos del usuario en:

```text
ORB_SLAM3/include/System.h
ORB_SLAM3/src/System.cc
orbslam3_ros2/src/stereo/stereo-slam-node.cpp
```

`fase45_sandbox/` permanece ignorado y fuera del alcance.
