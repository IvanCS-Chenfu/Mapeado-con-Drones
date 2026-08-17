# Subfase 6D — Contratos ROS 2 y ciclo de vida de tareas

## Estado

```text
sin hacer
```

## Dependencia

6C y contratos compartidos de Fases 2–5.

## Objetivo técnico

Definir interfaces compartidas entre Servidor y Dron para representar tareas, asignación, petición de nueva tarea, progreso, finalización y error, sin mezclar la tarea de larga duración con las trayectorias locales que se usarán para ejecutarla.

## Comportamiento esperado

El contrato debe soportar al menos:

```text
MAP_SECTION     -> tarea de cobertura generada por la misión
GO_TO           -> orden futura/manual (x,y,z,yaw)
ANCHOR_SUBMAP   -> comportamiento de sistema para recuperar anclaje global
```

Lifecycle mínimo conceptual:

```text
QUEUED -> ASSIGNED -> ACCEPTED -> RUNNING -> COMPLETED
                                  |            |
                                  +-> FAILED   +-> ...
                                  +-> CANCELLED
```

El dron, al quedar disponible, debe poder solicitar/indicar que necesita la siguiente tarea. El servidor sigue siendo autoridad de la cola. Los nombres exactos de `.msg/.srv/.action` se deben cerrar tras localizar las interfaces ya incorporadas por Fase 5; no duplicar mensajes equivalentes.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6_RESUMEN.md
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6.md
```

Antes de modificar código, Codex debe leer también el MD vigente de cada paquete afectado en `codex/contexto/paquetes/` y el contrato final de Fase 5 que exista en el workspace real. El snapshot usado para preparar esta Fase 6 no contiene la Fase 5 recién ejecutada por el usuario, por lo que no se deben inventar nombres nuevos de topics, frames o mensajes si Fase 5 ya proporciona un contrato equivalente.



## Diagnóstico de partida

`orbslam3_msgs` del snapshot contiene contratos de mapa/fiduciales, pero no tareas de misión. `TrayAction` pertenece a ejecución de trayectoria local y no debe reutilizarse como si representara una `MAP_SECTION` de larga duración.

## Invariantes y decisiones cerradas

- El paquete compartido mantiene la política de Fase 2: copia canónica de interfaces en Servidor y réplica idéntica en Dron.
- `task_id` debe ser estable y diferenciarse de `trajectory_id`.
- Una tarea puede permanecer `RUNNING` mientras muchas trayectorias terminan o se cancelan.
- Los estados y motivos de fallo deben ser explícitos; no inferir estado por ausencia de mensajes.
- El contrato no transporta GT ni datos de simulación.
- `GO_TO` incluirá `x,y,z,yaw`; `MAP_SECTION` transportará su objetivo nominal/ID y no una ruta física precalculada.

## Archivos permitidos a modificar

```text
src/servidor/orbslam3_msgs/msg/  # copia canónica
src/servidor/orbslam3_msgs/srv/
src/servidor/orbslam3_msgs/action/  # si la solución acordada usa action
src/dron/orbslam3_msgs/  # réplica completa conforme a Fase 2
src/servidor/orbslam3_msgs/CMakeLists.txt
src/dron/orbslam3_msgs/CMakeLists.txt
src/servidor/orbslam3_server/  # consumidores mínimos para test
src/dron/dron_individual/  # consumidor mínimo para test
codex/contexto/paquetes/orbslam3_msgs/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
ORB_SLAM3/
orbslam3_ros2/  # no hace falta para contrato de tareas
src/servidor/orbslam3_multi/
src/simulacion/simulacion_dron/  # salvo test de interfaces si imprescindible
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
política/guardas de copia de `orbslam3_msgs` creada en Fase 2
interfaces de pose/estado incorporadas en Fase 5
`TrayAction.action` para no duplicar semántica de trayectoria
nodos servidor/dron que serán consumidores en 6E/6G
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Cerrar el mecanismo ROS 2 de petición/asignación y feedback de tareas reutilizando acciones/servicios existentes cuando sea posible.
2. Definir campos comunes: `task_id`, tipo, prioridad, `drone_id`/destinatario cuando proceda, estado, progreso, resultado y motivo de fallo.
3. Definir payload específico de `MAP_SECTION`: nivel/banda, IDs/puntos nominales A/B/C y sentido todavía seleccionable o seleccionado por asignador.
4. Definir payload `GO_TO` con pose objetivo `(x,y,z,yaw)` en `world` y metadatos de validez necesarios.
5. Definir `ANCHOR_SUBMAP` de forma que se refiera al `(drone_id,map_epoch)` vigente y no a GT.
6. Añadir tests de serialización/generación de interfaces y comprobar que las dos copias de `orbslam3_msgs` quedan byte/semánticamente idénticas según la guarda existente.
7. Añadir marcadores de consumidores de prueba `TASK-CONTRACT` sin implementar todavía asignación completa.

## Cambios prohibidos

- No ampliar `TrayAction` para representar tareas: eso pertenece a 6H/6I.
- No crear mensajes separados incompatibles en Dron y Servidor.
- No acoplar el contrato a Gazebo o a una cantidad fija de drones.
- No usar Ground Truth como entrada funcional para pose, asignación, navegación, obstáculos, autorización de trayectorias, cobertura o criterio de finalización.
- No modificar `ORB_SLAM3` como primera opción; ampliar el wrapper o reutilizar interfaces existentes siempre que sea suficiente.
- No implementar la nube densa global, TSDF/Open3D global ni la reconstrucción final de Fase 8.
- No convertir el servidor en planificador de paredes u obstáculos físicos: esa responsabilidad pertenece al dron.
- No borrar ni reescribir datos raw de ORB-SLAM3 para adaptar la navegación.
- No introducir un modo separado de navegación interior y otro exterior.
- No rellenar historiales con resultados ficticios: se crean únicamente cuando exista una ejecución real.

## Paquetes a compilar

Comando base esperado:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_server dron_individual
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6D   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Generación y round-trip de interfaces

Compilar ambas copias de `orbslam3_msgs` en sus grupos y ejecutar un publicador/consumidor mínimo o test que serialice los tres tipos de tarea con campos no triviales.

### Prueba 2 — Guarda de copias compartidas

Ejecutar la guarda de Fase 2 y confirmar que la copia canónica y la de Dron son idénticas tras añadir los nuevos contratos.

## Patrones de reducción de logs

```text
TASK-CONTRACT|task_id|MAP_SECTION|GO_TO|ANCHOR_SUBMAP|orbslam3_msgs|mismatch|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Las interfaces se generan en Dron y Servidor y las copias son idénticas.
2. Existe representación inequívoca de los tres tipos de tarea baseline.
3. El lifecycle puede expresarse sin usar `TrayAction` como sustituto.
4. No hay campos GT/simulación.
5. Los contratos son compatibles con namespaces/IDs reales de Fase 5.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si compila pero falta un estado/error necesario para implementar 6E/6G sin inferencias.
- `PARCIAL` si las interfaces funcionan pero la guarda de copias no se actualiza.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Mezclar `task_id` y `trajectory_id`.
- Romper la duplicación controlada de `orbslam3_msgs`.
- Diseñar un mensaje demasiado específico que no pueda representar `GO_TO`/`ANCHOR_SUBMAP` o futura GUI.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6D.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6D_RESUMEN.md
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md              # si cambia el estado real
codex/contexto/paquetes/<paquete_afectado>/
```

Los historiales anteriores **no existen en este ZIP a propósito**. Deben crearse solo tras una ejecución real. La documentación de paquete debe reflejar el estado actual del código, no limitarse a añadir una nota histórica.

## Dudas funcionales de contrato

```text
ninguna
```

Cualquier duda nueva que cambie el comportamiento acordado suspende la autorización funcional y debe discutirse antes de continuar, conforme a `AGENTS.md`.
