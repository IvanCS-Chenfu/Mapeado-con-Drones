# Subfase 7I — Tarjetas de drones y progreso de tareas

## Estado

```text
sin hacer
```

## Dependencia

7B, 7C y Fase 6 con lifecycle/progreso disponible.

## Objetivo técnico

Conectar el panel derecho a los estados reales de drones/tareas. Crear/actualizar una tarjeta por dron con nombre/ID, estado útil, tarea actual, barra de progreso y metadata importante disponible. La lista debe escalar mediante scroll.

## Comportamiento esperado

Las tarjetas aparecen/desaparecen según el conjunto real de drones o misión, sin número fijo. La barra representa `progress` del productor: puede saltar por bloques (10%, 20%, etc.) si el planificador de cobertura actualiza por hitos. Para `GO_TO`, si el backend publica progreso geométrico fácil, se refleja de forma más continua.

La GUI no deduce porcentaje por tiempo ni por distancia si el TaskManager no lo declara como progreso real.

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

Fase 6 define campos conceptuales de estado/progreso, pero la implementación real puede usar nombres/interfaces distintos. El panel layout existe desde 7B. Hace falta comprobar si la Fase 6 ejecutada publica una vista por dron suficiente para GUI o si solo existen logs/estado interno.

## Invariantes y decisiones cerradas

- El servidor/TaskManager es autoridad de estado/progreso.
- Progreso discreto es válido.
- No exigir 100% suave ni update por frame.
- No inventar batería/latencia si no existe sensor/contrato.
- Tracking/pose validity proviene de Fase 5 si se muestra.
- Scroll solo afecta visualización, no orden/prioridad de drones.
- No añadir pause/resume/cancel.

## Archivos permitidos a modificar

```text
src/servidor/multidron_gui/src/widgets/drone_card.*
src/servidor/multidron_gui/src/widgets/drone_cards_panel.*
src/servidor/multidron_gui/src/ros_data_bridge.*
src/servidor/multidron_gui/src/gui_data_model.*
src/servidor/orbslam3_server/              # solo si falta estado/progreso canónico y se reabre Fase 6
src/servidor/orbslam3_msgs/
src/dron/orbslam3_msgs/
```

Las rutas nuevas son de contrato. Antes de crearlas, comprobar el árbol posterior a Fases 2–6. Si existe un componente equivalente, reutilizarlo en lugar de duplicarlo.

## Archivos prohibidos

```text
cronómetros GUI usados como progreso
datos GT para estado del dron
botones pause/resume/cancel
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
task_id / task type / state / progress / result / failure reason de Fase 6
drone_id/namespaces reales
estado pose/tracking de Fase 5 si aparece en card
TaskManager / TaskExecutor productor de feedback
DroneCard / DroneCardsPanel
```

Los nombres de componentes nuevos definidos por este contrato pueden implementarse con una estructura equivalente si existe una razón técnica clara. Los nombres de **interfaces procedentes de fases anteriores** no se inventan: se localizan físicamente primero.

## Cambios requeridos

1. Localizar el stream/consulta canónica de estado de tareas por dron.
2. Si el progreso real no se publica, volver a Fase 6 y añadirlo en el productor; no calcularlo en GUI.
3. Mapear cada `drone_id` a una tarjeta estable y actualizar solo widgets afectados.
4. Mostrar tarea actual y estado de lifecycle con labels claros.
5. Mostrar barra `0..100` o escala equivalente proveniente del contrato; soportar saltos grandes.
6. Para `GO_TO`, aprovechar progreso continuo solo si Fase 6 lo ofrece de forma barata/canónica; no crear una segunda fórmula en GUI.
7. Mostrar fallo/razón cuando exista, sin ocultar la última tarea detrás de “0%”.
8. Probar 20+ tarjetas sintéticas y N drones reales.
9. Añadir logs `GUI-TASK-CARD`, `GUI-TASK-PROGRESS` agregados/configurables.

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

Si aparece una duda funcional no acordada —incluido cómo representar un dron perdido, stale o sin pose válida— Codex debe parar y preguntarle al usuario. No escoger arbitrariamente una representación.


## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh multidron_gui
```

Si Fase 6 necesita exponer estado/progreso, compilar también interfaces/TaskManager modificados después de reabrir esa fase.

Si la separación de Fase 2 utiliza builds por grupo, usar el helper vigente para el grupo Servidor y, cuando haya pruebas de integración, los grupos Dron/Simulación correspondientes. Registrar el comando exacto solo en el historial real.

## Pruebas Gazebo requeridas

### Prueba 1 — Progreso discreto

Ejecutar una `MAP_SECTION` o replay de estados que avance por hitos. La barra debe saltar exactamente según los porcentajes reportados.

### Prueba 2 — GO_TO

Si Fase 6 reporta progreso, comprobar que la barra sigue ese valor. Si no lo reporta, no inventarlo; clasificar el gap y volver a Fase 6 si el usuario quiere progreso real.

### Prueba 3 — Scroll

Con muchas tarjetas, recorrer la lista mientras el formulario de tareas sigue accesible y las cards continúan actualizándose.

### Prueba 4 — Fallo de tarea

Inyectar/observar un estado `FAILED` con motivo. La tarjeta debe mostrar estado/razón sin traducirlo a éxito parcial.

No arrancar Gazebo artificialmente para una prueba puramente gráfica/unitaria. Cuando se use simulación, el comando base es:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_7_7I \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El mecanismo exacto para arrancar `multidron_gui` junto a ese launch se fija en 7A/7B y debe reutilizarse después. La GUI nunca depende de que RViz2 esté abierto.

## Patrones de reducción de logs

```text
GUI-TASK-CARD|GUI-TASK-PROGRESS|TASK-STATE|TASK-EXEC|task_id|progress|FAILED|COMPLETED|GO-TO-TASK|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo alimentan reductores. Si falta evidencia, regenerar el reducido con patrones más precisos; no abrir el log completo directamente.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Una card dinámica representa cada dron real.
2. Tarea/estado/progreso coinciden con el mensaje del productor.
3. Progreso discreto se representa correctamente.
4. Scroll funciona con muchas tarjetas.
5. No existe progreso inventado ni botones no acordados.
6. Cualquier error de lifecycle/progreso de Fase 6 se corrige allí y se revalida.

Además, todo build requerido debe devolver `0`, todas las pruebas obligatorias deben ejecutarse, no puede haber errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si cards funcionan pero el backend no expone progreso fiable.
- `PARCIAL` si hay progreso pero tarjetas se mezclan entre drones por identidad/namespaces.
- `PARCIAL` si muchas tarjetas bloquean el event loop.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos o porque una fase anterior debe corregirse antes de continuar.

## Riesgos

- Confluir `task_id` de drones distintos.
- Redibujar/recrear todas las cards en cada mensaje.
- Mostrar “100%” por estado temporal que no significa `COMPLETED`.
- Añadir campos decorativos sin fuente real.

## Documentación a actualizar


Al ejecutar realmente la subfase, actualizar solo documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7I.md
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7I_RESUMEN.md
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
