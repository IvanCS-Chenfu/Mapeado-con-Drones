# Subfase 6E — Gestor de misión y cola global de tareas en el servidor

## Estado

```text
sin hacer
```

## Dependencia

6C y 6D.

## Objetivo técnico

Implementar en el servidor la autoridad de estado de misión: almacenar todas las tareas iniciales, mantener su lifecycle, conocer drones disponibles y entregar trabajo cuando un dron lo solicite, sin decidir todavía la métrica avanzada de asignación de 6F.

## Comportamiento esperado

Al arrancar la misión:

```text
tarea_principal.yaml
        -> niveles/secciones
        -> todas las MAP_SECTION
        -> TaskManager/cola
```

Los drones pueden quedar `IDLE`/disponibles y solicitar una tarea. Cuando terminan una tarea, informan el resultado y solicitan otra. El servidor no bloquea niveles: si hay más drones que tareas útiles en el nivel inferior, los sobrantes pueden recibir trabajos de niveles superiores.

La misión se declara completada únicamente cuando todas las tareas iniciales de mapeo están `COMPLETED`. Una tarea `FAILED` no se convierte silenciosamente en completada; la política de reintento/reasignación debe conservarla pendiente o marcar el bloqueo explícito.

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

El servidor actual coordina el mapa global, no una misión de cobertura. No existe una fuente central de estados de tareas ni una cola que evite asignaciones duplicadas.

## Invariantes y decisiones cerradas

- Cada tarea tiene un único estado autoritativo en servidor.
- Una tarea no puede estar asignada simultáneamente a dos drones salvo que una futura política explícita lo permita; el solape se obtiene mediante tareas vecinas diferentes.
- Los niveles pueden ejecutarse en paralelo.
- Una tarea finalizada no desaparece del registro: conserva resultado para misión/GUI futura.
- La gestión de tareas debe estar separada de `orbslam3_multi` y, preferiblemente, de la lógica monolítica de `global_map_server.cpp` mediante clases/archivos propios.

## Archivos permitidos a modificar

```text
src/servidor/orbslam3_server/include/  # TaskManager/MissionManager propuesto
src/servidor/orbslam3_server/src/
src/servidor/orbslam3_server/launch/
src/servidor/orbslam3_server/config/
src/servidor/orbslam3_server/test/
codex/contexto/paquetes/orbslam3_server/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
src/servidor/orbslam3_multi/
src/dron/dron_individual/src/control_tray/  # ejecutor en 6G
ORB_SLAM3/
src/simulacion/simulacion_dron/src/control_tray/scenario_runner_node.cpp  # no sustituir el manager por el runner
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
nodo servidor principal posterior a Fase 5
callbacks de nuevas interfaces de tareas de 6D
estado de drones/pose global disponible desde Fase 5
generador de tareas de 6C
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Crear `MissionManager`/`TaskManager` o equivalente como componente separado con registro por `task_id` y estados válidos.
2. Cargar al manager todas las `MAP_SECTION` generadas antes de aceptar peticiones de trabajo.
3. Implementar transición atómica `QUEUED -> ASSIGNED` cuando se entrega una tarea y rechazar dobles asignaciones.
4. Procesar aceptación, inicio, progreso, completado y fallo con validación de `task_id` y `drone_id`.
5. Cuando un dron queda disponible, permitir que solicite siguiente trabajo; dejar el hook de selección en una función de coste que 6F completará.
6. No bloquear por nivel: la lista de candidatas puede contener trabajos de distintos niveles.
7. Implementar criterio de misión final: todas las tareas iniciales `COMPLETED`; registrar `MISSION-COMPLETE` una sola vez.
8. Añadir logs `TASK-QUEUED`, `TASK-ASSIGNED`, `TASK-STATE`, `TASK-REQUEST`, `MISSION-COMPLETE`.

## Cambios prohibidos

- No implementar todavía el planificador de trayectorias.
- No escoger tareas por orden fijo como solución definitiva si eso contradice 6F; usar una política simple claramente provisional/testeable.
- No dar por completada una tarea `FAILED` para conseguir cerrar la misión.
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
./codex/herramientas/build_selected_packages.sh orbslam3_server orbslam3_msgs
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6E   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Lifecycle sintético de varias tareas

Sin movimiento físico obligatorio, simular/inyectar dos drones que solicitan tareas, aceptan, completan y vuelven a solicitar. No debe asignarse dos veces el mismo `task_id`.

### Prueba 2 — Más drones que tareas en un nivel

Configurar suficientes drones/tareas para demostrar que un dron sobrante puede recibir una tarea de un nivel superior sin esperar a que el inferior termine.

## Patrones de reducción de logs

```text
TASK-QUEUED|TASK-REQUEST|TASK-ASSIGNED|TASK-STATE|MISSION-COMPLETE|duplicate|invalid transition|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Todas las tareas iniciales viven en un registro autoritativo.
2. Las transiciones válidas son deterministas y las inválidas se rechazan.
3. Un dron puede pedir otra tarea al quedar libre.
4. No existe barrera obligatoria por nivel.
5. La misión solo termina cuando todas las tareas de mapeo están completadas.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si la cola funciona pero faltan validaciones de transición/reintento.
- `PARCIAL` si el reparto sigue bloqueado por niveles.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Transiciones inválidas por mensajes duplicados/reordenados.
- Bloquear involuntariamente niveles superiores.
- Acoplar la misión al backend sparse y al worker de loops/optimización.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6E.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6E_RESUMEN.md
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
