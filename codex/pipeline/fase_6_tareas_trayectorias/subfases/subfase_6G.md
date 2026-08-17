# Subfase 6G — Ejecutor embarcado de tareas y nodo `control_trayectorias`

## Estado

```text
sin hacer
```

## Dependencia

6D, 6E y 6F.

## Objetivo técnico

Crear en el grupo Dron un ejecutor de alto nivel que reciba una tarea, mantenga su lifecycle local y coordine planificación, solicitud de reserva, ejecución de `TrayAction`, replanning y resultado sin delegar control de alta frecuencia al servidor.

## Comportamiento esperado

El componente, denominado conceptualmente `control_trayectorias`/`TaskExecutor`, debe ser el puente entre una tarea de misión y las trayectorias cortas que se implementarán después:

```text
Task asignada
  -> validar precondiciones
  -> ACCEPTED/RUNNING
  -> preparar/planificar
  -> proponer trayectoria al servidor
  -> ejecutar TrayAction si está autorizada
  -> observar/replanificar
  -> repetir
  -> COMPLETED/FAILED
  -> solicitar otra tarea
```

En esta subfase se construye la máquina de estados y las integraciones mínimas, usando una planificación trivial/placeholder segura si todavía no existen 6K–6O. No se debe fingir cobertura avanzada antes de esas subfases.

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

`dron_individual` ya tiene `gen_tray`, `control_calcular_fuerzas` y `aplicar_fuerzas_dron`, pero no existe un nodo que interprete una `MAP_SECTION` de larga duración ni que distinga tarea y trayectoria.

## Invariantes y decisiones cerradas

- El servidor asigna qué hacer; el dron decide cómo hacerlo.
- Una sola tarea de alto nivel puede sobrevivir a muchas cancelaciones de trayectoria.
- El ejecutor no controla motores directamente: reutiliza la cadena de Fase 1/5 mediante `TrayAction`.
- La pose usada para planificación procede de Fase 5, no GT.
- Si la tarea requiere anclaje global no disponible, el ejecutor debe poder solicitar/activar el comportamiento 6S en lugar de improvisar.

## Archivos permitidos a modificar

```text
src/dron/dron_individual/src/control_tray/  # nuevo nodo/archivos separados
src/dron/dron_individual/include/  # si el paquete dispone/crea headers
src/dron/dron_individual/launch/
src/dron/dron_individual/config/
src/dron/dron_individual/CMakeLists.txt
src/dron/dron_individual/package.xml
codex/contexto/paquetes/dron_individual/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
src/servidor/orbslam3_server/src/  # salvo adaptación mecánica del contrato ya cerrado
src/dron/lib_tray/  # waypoints se implementan en 6I
ORB_SLAM3/
src/simulacion/simulacion_dron/src/control_tray/gui_tray_multi.py  # no usar GUI como ejecutor
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
`AccionTrayectoria`/action server real en `gen_tray.cpp`
cliente/action pattern usado por escenario/GUI actuales
fuente de pose/velocidad posterior a Fase 5
interfaces de tareas 6D
launch que crea cada namespace de dron
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Crear el nodo/estado `TaskExecutor` por dron y parametrizar su `drone_id`/namespace desde el launch real.
2. Implementar recepción/aceptación de tarea y transiciones locales coherentes con 6D.
3. Crear un estado `PREPARING` interno para colocación previa a una `MAP_SECTION`; `GO_TO_START` no es una tarea global separada.
4. Encapsular el envío/cancelación de `TrayAction` en una interfaz que 6I/6J ampliarán.
5. Reportar progreso/resultado al servidor y solicitar nueva tarea solo cuando la tarea actual termina o falla según política.
6. Evitar ejecutar dos tareas `RUNNING` simultáneamente en el mismo dron.
7. Añadir logs `TASK-EXEC-ACCEPT`, `TASK-EXEC-STATE`, `TASK-EXEC-RESULT` y trazabilidad `task_id`↔`trajectory_id` cuando exista.

## Cambios prohibidos

- No implementar ahora el algoritmo completo de cobertura/frontiers.
- No reemplazar control dinámico ni `gen_tray`.
- No permitir que una nueva tarea normal preempcione silenciosamente una tarea `RUNNING`.
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
./codex/herramientas/build_selected_packages.sh dron_individual orbslam3_msgs
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6G   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Lifecycle de tarea simple

Con un solo dron, asignar una tarea de prueba que pueda resolverse con un único movimiento ya soportado. Debe pasar por ACCEPTED/RUNNING/COMPLETED y solicitar otra tarea sin tocar GT.

### Prueba 2 — Rechazo de segunda tarea concurrente

Mientras una tarea está `RUNNING`, intentar entregar otra tarea normal. Debe quedar en servidor/cola o rechazarse de forma coherente; el dron no ejecuta dos tareas a la vez.

## Patrones de reducción de logs

```text
TASK-EXEC|TASK-ASSIGNED|TASK-REQUEST|TrayAction|GOAL|RESULT|success|task_id|trajectory_id|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Existe un ejecutor por dron integrado con launch/namespaces.
2. Tarea y trayectoria son estados distintos.
3. La cadena de control existente se reutiliza mediante action/contrato, no se duplica.
4. El dron reporta resultado y pide nuevo trabajo.
5. No se ejecutan dos tareas concurrentes.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si el lifecycle funciona pero el ejecutor aún no puede encadenar movimientos.
- `PARCIAL` si servidor/dron pierden sincronía en cancelaciones recuperables.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Crear una segunda cadena de control paralela.
- Confundir final de una trayectoria con final de tarea.
- Estados servidor/dron divergentes tras cancelación o error.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6G.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6G_RESUMEN.md
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
