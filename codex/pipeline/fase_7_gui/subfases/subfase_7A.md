# Subfase 7A — Arquitectura, dependencias gráficas y contratos ROS 2

## Estado

```text
conseguida
```

## Dependencia

Fases 2–6 documentadas/ejecutadas hasta el punto necesario. Es la primera subfase de Fase 7.

## Objetivo técnico

Estudiar el workspace real posterior a Fases 3–6 y cerrar la arquitectura técnica de la GUI sin implementar todavía todas las capas. Debe quedar definido qué proceso/nodo se crea en Servidor, qué versión de Qt/OpenGL se usa, cómo se integra `rclcpp` con el event loop de Qt y cuál es la fuente canónica de cada dato que la GUI consumirá o enviará.

## Comportamiento esperado

Al terminar deben existir dos paquetes GUI independientes dentro de `src/servidor/` —`multidron_gui_lib` para lógica/modelos/render/widgets y `multidron_gui` para orquestación/ejecutable/launch, salvo equivalente ya existente— con un ejecutable mínimo capaz de iniciar ROS 2 + Qt y cerrarse limpiamente. Debe existir un inventario explícito de entradas/salidas para sparse, score, KFs, fiduciales, pose/tracking, tarea/progreso, trayectoria y futura dense.

El `global_map_server` y los nodos de Dron deben funcionar aunque `multidron_gui` no se ejecute en runtime. Las dependencias gráficas no deben introducirse dentro del proceso crítico de mapa.

## Contexto obligatorio a leer


```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_7_gui/pipeline_fase_7_RESUMEN.md
codex/pipeline/fase_7_gui/pipeline_fase_7.md
```

Antes de modificar código, leer además los resúmenes/contratos finales **reales** de Fases 3, 4, 5 y 6 que sean productores de los datos usados por esta subfase, y los MD vigentes de cada paquete afectado en `codex/contexto/paquetes/`.

No asumir que los nombres de topics/messages del snapshot documental siguen siendo los definitivos tras ejecutar Fases 3–6. Si ya existe un contrato equivalente, reutilizarlo. No crear una segunda fuente de verdad solo para la GUI.


## Diagnóstico de partida

El snapshot original ya publica `/global_sparse_cloud` y `/global_keyframes`, pero Fases 4–6 pueden haber creado interfaces nuevas. No existe todavía un paquete operativo de GUI Fase 7. También falta confirmar si todos los datos requeridos —especialmente score por MapPoint y trayectoria actual— están disponibles en un formato consumible sin depender de RViz2.

## Invariantes y decisiones cerradas

- La GUI pertenece físicamente a `src/servidor/`.
- Debe ser proceso/nodo independiente del backend de mapa.
- La separación acordada es `multidron_gui_lib` + `multidron_gui`, para poder
  testear modelos/render/widgets sin acoplarlos al `main()`.
- C++ + Qt Widgets + OpenGL es la dirección acordada; elegir Qt5/Qt6 según el entorno real, sin cambiar de toolkit a web.
- `world` es el frame funcional de la escena y de los objetivos manuales.
- ROS 2 proporciona datos/órdenes; RViz2 no es una dependencia.
- Las interfaces existentes de Fases 3–6 tienen precedencia sobre contratos duplicados para GUI.
- Si falta telemetría, añadir el contrato en el productor correcto o reabrir su fase; no inferirla visualmente.

## Archivos permitidos a modificar

```text
src/servidor/multidron_gui_lib/              # paquete nuevo propuesto para lógica/render/widgets
src/servidor/multidron_gui/                  # paquete nuevo propuesto para ejecutable/launch
src/servidor/orbslam3_server/                # solo telemetría/contrato mínimo si falta
src/servidor/orbslam3_msgs/                  # solo si un contrato compartido real necesita ampliarse
src/dron/orbslam3_msgs/                      # réplica si Fase 2 exige sincronía
src/dron/dron_individual/                    # solo si falta telemetría canónica de Fase 5/6 y se reabre la fase propietaria
codex/contexto/paquetes/
```

Las rutas nuevas son de contrato. Antes de crearlas, comprobar el árbol posterior a Fases 2–6. Si existe un componente equivalente, reutilizarlo en lugar de duplicarlo.

## Archivos prohibidos

```text
ORB_SLAM3/
src/servidor/orbslam3_multi/                 # no mover algoritmos de mapa a GUI
src/simulacion/simulacion_dron/gui_tray_multi.py  # legacy no se convierte en nueva GUI
pipeline_flow / visualizador web             # no reutilizar como frontend operativo
```

Además:


- No usar Ground Truth como entrada funcional de GUI, pose, mapa, tareas o trayectorias; GT solo puede usarse como métrica externa de simulación cuando una prueba lo necesite.
- No incrustar ni depender de RViz2.
- No crear una aplicación web ni reutilizar `pipeline_flow` como GUI operativa.
- No ejecutar SLAM, fusión, optimización, planner, obstacle avoidance ni reconstrucción densa dentro del thread gráfico.
- No mandar `TrayAction` directamente desde la GUI para saltarse el sistema de tareas de Fase 6.
- No modificar `ORB_SLAM3` salvo una necesidad separada, demostrada y autorizada; `CAPTURE_SPARSE` no permite forzar KFs.
- No implementar Fase 8 dentro de Fase 7.
- No limpiar legacy o tocar paquetes ajenos como cambio colateral.
- No rellenar historiales con resultados ficticios; se crean únicamente tras ejecuciones reales.


## Funciones, clases, nodos o interfaces que hay que localizar

```text
/global_sparse_cloud y productor real
/global_keyframes y productor real
pose global/estado tracking de Fase 5
task manager / task state / progress de Fase 6
representación de trayectoria vigente de Fase 6
fiducial/anchor topics de Fase 4
global_map_server y launch de Servidor
launch multi-dron posterior a Fase 6
CMake/package manifests del grupo Servidor
```

Localizar también si `PointCloud2` sparse incluye score/IDs. Si no los incluye, documentar dónde vive el score y cuál es la ampliación mínima viable; no implementarla silenciosamente sin clasificar ownership.

Los nombres de componentes nuevos definidos por este contrato pueden implementarse con una estructura equivalente si existe una razón técnica clara. Los nombres de **interfaces procedentes de fases anteriores** no se inventan: se localizan físicamente primero.

## Cambios requeridos

1. Inventariar físicamente productores, tipos ROS, frames, frecuencias, QoS y lifecycle de todos los datos que usará la GUI.
2. Crear la estructura mínima de `multidron_gui_lib` y `multidron_gui` o reutilizar equivalente, con `package.xml`, `CMakeLists.txt`, dependencia correcta entre ambos y ejecutable ROS 2/Qt vacío pero funcional.
3. Determinar Qt5/Qt6 y módulos OpenGL disponibles en el entorno objetivo; documentar dependencias de sistema y ROS sin meterlas en otros paquetes.
4. Definir que el thread principal sea el event loop Qt y que ROS 2 use un executor/thread separado o mecanismo equivalente no bloqueante; ningún callback tocará widgets directamente.
5. Definir el `GuiDataModel`/snapshot como frontera de thread.
6. Congelar tabla `dato -> productor -> topic/service/action -> tipo -> frame -> frecuencia/QoS -> subfase consumidora`.
7. Clasificar cualquier dato ausente: extensión de Fase 7 puramente telemétrica o retorno a Fase 3/4/5/6.
8. Definir un launch/forma reproducible de arrancar la GUI sola y junto a la simulación, sin hacerla obligatoria para el backend.
9. Añadir logs baseline `GUI-BOOT`, `GUI-ROS-READY`, `GUI-SHUTDOWN` y versión de Qt/OpenGL detectada sin saturar logs.

## Cambios prohibidos

- No cambiar comportamiento de un productor previo solo para simplificar el renderer.
- No convertir telemetría descartable/visual en una dependencia del pipeline.
- No realizar una decisión funcional no acordada si durante la integración aparecen varias alternativas razonables; parar y preguntar al usuario.


## Puerta de validación hacia fases anteriores

La GUI es herramienta de observación, no capa de maquillaje. Para cualquier anomalía:

1. capturar primero el valor/mensaje que recibe la GUI y la transformación aplicada;
2. si el dato de entrada ya es incorrecto, detener esta subfase y localizar la fase propietaria;
3. no aplicar offsets, escalados, filtros o estados falsos para que “se vea bien”;
4. si la corrección de origen implica comportamiento funcional, suspender autorización y consultar al usuario conforme a `AGENTS.md`;
5. corregir en la fase de origen, revalidarla y repetir después esta prueba de Fase 7;
6. si el dato recibido es correcto y solo se representa mal, corregir Fase 7.

Ejemplos de ownership: sparse/KFs Fase 3, fiduciales Fase 4, pose/tracking Fase 5, tarea/progreso/trayectoria Fase 6.

Si aparece una duda funcional no acordada por este contrato, Codex debe parar y preguntarle al usuario. Para dron perdido/stale/sin pose nueva válida ya rige la decisión cerrada: conservar última pose válida, mostrar `PERDIDO` y usar representación más transparente.


## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh --group servidor multidron_gui_lib
./codex/herramientas/build_selected_packages.sh --group servidor multidron_gui
```

Si 7A añade únicamente telemetría mínima a otro paquete, incluirlo en el build real y justificarlo.

Si la separación de Fase 2 utiliza builds por grupo, usar el helper vigente para el grupo Servidor y, cuando haya pruebas de integración, los grupos Dron/Simulación correspondientes. Registrar el comando exacto solo en el historial real.

## Pruebas Gazebo requeridas

### Prueba 1 — Build y arranque mínimo sin simulación

Compilar `multidron_gui_lib` y `multidron_gui`, y ejecutar el nodo con ROS 2 sin topics de datos. Debe abrir una ventana mínima, mantener vivo el executor y cerrar limpiamente sin crash ni esperar a ningún publisher.

### Prueba 2 — Backend sin GUI

Arrancar una prueba corta normal de servidor/drones **sin** iniciar GUI y confirmar que nada de Fases 3–6 depende del proceso gráfico.

### Prueba 3 — GUI conectada/desconectada

Con backend activo, abrir y cerrar la GUI dos veces. El mapa/control/tareas deben continuar. Esta prueba valida arquitectura, todavía no contenido visual.

No arrancar Gazebo artificialmente para una prueba puramente gráfica/unitaria. Cuando se use simulación, el comando base es:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_7_7A \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El mecanismo exacto para arrancar `multidron_gui` junto a ese launch se fija en 7A/7B y debe reutilizarse después. La GUI nunca depende de que RViz2 esté abierto.

## Patrones de reducción de logs

```text
GUI-BOOT|GUI-ROS-READY|GUI-SHUTDOWN|Qt|OpenGL|multidron_gui_lib|multidron_gui|global_sparse_cloud|global_keyframes|task|trajectory|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo alimentan reductores. Si falta evidencia, regenerar el reducido con patrones más precisos; no abrir el log completo directamente.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Existen los paquetes `multidron_gui_lib` y `multidron_gui`, con proceso GUI mínimo independiente en Servidor.
2. La versión/toolchain Qt/OpenGL queda cerrada y compilable.
3. Existe inventario completo de interfaces reales de Fases 3–6 o una lista explícita de gaps con ownership.
4. Cerrar/no abrir GUI no afecta al backend.
5. No se introduce dependencia de RViz2/web/GT.
6. La estrategia de threads evita que callbacks ROS ejecuten UI directamente.

Además, todo build requerido debe devolver `0`, todas las pruebas obligatorias deben ejecutarse, no puede haber errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si el paquete/ventana compila pero faltan interfaces críticas por localizar.
- `PARCIAL` si la GUI es independiente en runtime pero la dependencia Qt se ha acoplado innecesariamente al ejecutable del mapa.
- `BLOQUEADA` si una fase anterior no ofrece un dato imprescindible y no puede definirse el contrato sin reabrirla.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos o porque una fase anterior debe corregirse antes de continuar.

## Riesgos

- Elegir una versión de Qt incompatible con el SO/ROS real.
- Duplicar datos existentes mediante nuevos topics “para GUI”.
- Meter Qt/OpenGL en `global_map_server` y degradar el despliegue headless.
- Descubrir tarde que score/trayectoria/progreso no están publicados canónicamente.

## Documentación a actualizar


Al ejecutar realmente la subfase, actualizar solo documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7A.md
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7A_RESUMEN.md
codex/pipeline/fase_7_gui/pipeline_fase_7_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md              # si cambia el estado real
codex/contexto/paquetes/<paquete_afectado>/
```

Los historiales **no existen en este ZIP a propósito**. Deben crearse solo cuando la subfase se ejecute de verdad. La documentación de paquete debe describir el código actual: ejecutables/nodos, clases, topics/services/actions, parámetros, markers y limitaciones vigentes.


## Dudas funcionales de contrato

```text
ninguna en el acuerdo actual
```

Cualquier duda funcional nueva descubierta durante preparación/ejecución debe presentarse al usuario antes de continuar.
