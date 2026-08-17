# Subfase 6S — Tareas/comportamientos especiales `ANCHOR_SUBMAP` y `GO_TO`

## Estado

```text
sin hacer
```

## Dependencia

6D–6O y 6P–6R; contratos de anclaje/pose de Fases 3–5.

## Objetivo técnico

Integrar dos comportamientos que no deben confundirse con la cobertura normal: recuperación de anclaje global de un submapa manteniendo tracking local, y orden `GO_TO(x,y,z,yaw)` de máxima prioridad pendiente para la futura GUI.

## Comportamiento esperado

### `ANCHOR_SUBMAP`

Se activa cuando el `(drone_id,map_epoch)` vigente mantiene tracking local pero no dispone de anchor global utilizable. El dron no puede ejecutar con seguridad una tarea global normal, así que realiza una búsqueda **relativa y conservadora**:

```text
mantener tracking local
 -> mirar progresivamente alrededores
 -> buscar fiducial visual o zona conocida que produzca loop/anclaje servidor
 -> si soporte visual cae, dejar de girar y volver a vista estable
 -> cuando exista anchor, continuar trabajo normal
```

Puede desplazarse con trayectorias locales seguras dentro de la información local disponible. Debe tener límites de tiempo/distancia/intentos para no vagar indefinidamente. El resultado de anchor lo determina el contrato real de Fases 3–5, no GT.

### `GO_TO`

Un futuro GUI/cliente puede insertar `GO_TO(x,y,z,yaw)` en el gestor de tareas. Tiene **máxima prioridad entre tareas pendientes**, pero no interrumpe una tarea que ya está `RUNNING`. Si llega durante `MAP_SECTION_B`, se encola delante del resto y se ejecuta inmediatamente después de que B termine. Usa exactamente el mismo LocalPlanner, depth, tracking, reservas y replanning que cualquier movimiento autónomo.

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

El proyecto necesita recuperar un dron cuyo mapa local todavía no está unido al global y también dejar preparado el contrato de comando manual de Fase 7. Si ambos se resuelven saltándose el planificador/reservas se crearían rutas inseguras y semánticas paralelas.

## Invariantes y decisiones cerradas

- `ANCHOR_SUBMAP` nunca usa GT para localizar fiducial, loop o pose.
- La búsqueda de anchor prioriza conservar el tracking local existente.
- Si durante una tarea aparece un nuevo `map_epoch` no anclado, el movimiento global normal se suspende de forma segura y la tarea conserva su progreso hasta recuperar anchor o fallar; esto es una precondición de seguridad, no una preempción manual `GO_TO`.
- `GO_TO` no preempciona una tarea `RUNNING`; solo domina la cola pendiente.
- `GO_TO` no envía directamente un `TrayAction`: pasa por planificación local y autorización servidor.
- Un `GO_TO` inalcanzable/fuera de `flight_bounds` falla con razón explícita.

## Archivos permitidos a modificar

```text
src/dron/dron_individual/src/control_tray/
src/dron/dron_individual/config/
src/servidor/orbslam3_server/src/
src/servidor/orbslam3_server/include/
src/servidor/orbslam3_msgs/
src/dron/orbslam3_msgs/
src/dron/orbslam3_ros2/  # solo consumo de estado ya expuesto, no detección nueva
codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/orbslam3_server/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
ORB_SLAM3/  # no modificar para “buscar” loop en esta subfase salvo bloqueo separado
src/servidor/orbslam3_multi/  # no alterar loop/anchor backend salvo integración mecánica con contrato existente
src/simulacion/simulacion_dron/plugins/GT*  # no usar GT como atajo
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
estado `anchored/unanchored` y `(drone_id,map_epoch)` tras Fase 5
eventos de fiducial/loop/anchor del servidor
TaskManager 6E y prioridades
TaskExecutor 6G
LocalPlanner/ViewPlanner/Replan 6M–6O
API de entrada de futuros comandos de GUI si ya existe; si no, crear solo backend agnóstico a GUI
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Implementar detector de precondición `submap anchored?` usando exclusivamente estado del pipeline real y activar `ANCHOR_SUBMAP` cuando corresponda.
2. Crear patrón de búsqueda local conservador: pequeños giros/traslaciones, preferencia por vistas con soporte visual, depth para obstáculos y límites configurables de búsqueda.
3. Detener/revertir un giro de búsqueda si 6L indica pérdida de soporte; no interpretar esa caída como “zona vacía”.
4. Terminar `ANCHOR_SUBMAP` al recibir confirmación de anchor válido para el epoch actual; rechazar anchors obsoletos de otro epoch según Fase 5.
5. Definir timeout/distancia/intentos y resultado `FAILED/BLOCKED` seguro si no se encuentra fiducial/loop; no explorar indefinidamente.
6. Implementar API servidor para encolar `GO_TO` con prioridad máxima pendiente y validar frame/`flight_bounds`/datos.
7. Garantizar que `GO_TO` recibido durante otra tarea queda primero pendiente sin cancelar la tarea actual; después se entrega como siguiente tarea.
8. Ejecutar `GO_TO` usando los mismos módulos de planificación/reserva/replanning, con logs `ANCHOR-TASK` y `GO-TO-TASK`.

## Cambios prohibidos

- No hacer spins de 360° agresivos sin vigilar tracking.
- No usar posición GT para encontrar una zona conocida/fiducial.
- No interrumpir una `MAP_SECTION RUNNING` por un `GO_TO` manual.
- No saltarse 6Q/6K por ser una orden de operador.
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
./codex/herramientas/build_selected_packages.sh dron_individual orbslam3_server orbslam3_msgs orbslam3_ros2
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6S   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — `ANCHOR_SUBMAP` por fiducial o loop

Iniciar un dron/submapa sin anchor global pero con tracking local. Debe ejecutar búsqueda conservadora y terminar cuando el mecanismo real de Fases 3–5 confirme anchor, sin consultar GT.

### Prueba 2 — Protección de tracking durante búsqueda

Durante la exploración de anchor, orientar hacia una zona visualmente pobre. El giro debe limitarse/revertirse en vez de continuar hasta perder tracking deliberadamente.

### Prueba 3 — Prioridad `GO_TO` sin preempción

Con `MAP_SECTION` ya `RUNNING`, encolar un `GO_TO`. La tarea de mapeo sigue hasta `COMPLETED`; el `GO_TO` queda por delante de todas las demás pendientes y se asigna inmediatamente después.

### Prueba 4 — `GO_TO` seguro

Ejecutar un `GO_TO` que requiera replanning/reserva y comprobar que usa obstacle avoidance local y coordinación Dron-Dron, no un goal directo sin validación.

## Patrones de reducción de logs

```text
ANCHOR-TASK|GO-TO-TASK|anchored|map_epoch|fiducial|loop|TASK-STATE|TASK-ALLOC|TRAJ-CONFLICT|LOCAL-OBSTACLE|SLAM-VISUAL-STATE|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. `ANCHOR_SUBMAP` recupera anchor mediante percepción/loop real sin GT y protege tracking.
2. La búsqueda tiene límites y puede fallar de forma segura.
3. `GO_TO` queda primero entre pendientes pero nunca interrumpe una tarea running.
4. `GO_TO` reutiliza toda la cadena de planificación/reservas.
5. Eventos de epoch obsoleto no se aplican.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si `ANCHOR_SUBMAP` funciona solo por fiducial o solo por loop cuando el otro mecanismo disponible no se integra.
- `PARCIAL` si `GO_TO` respeta prioridad pero se salta obstacle avoidance/reservas.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Búsqueda de anchor que aleje el dron de su zona local conocida y pierda tracking.
- GO_TO que se quede esperando detrás de una tarea muy larga; las tareas usan segmentos cortos pero la **tarea** puede durar, y esta prioridad no implica preempción por decisión explícita del usuario.
- Evento de anchor obsoleto tras cambio de epoch.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6S.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6S_RESUMEN.md
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
