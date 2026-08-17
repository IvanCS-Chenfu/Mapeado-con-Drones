# Subfase 7G — Visualización de la trayectoria actual de cada dron

## Estado

```text
sin hacer
```

## Dependencia

Fase 6 ejecutada hasta disponer de trayectoria vigente; 7D y 7F conseguidas.

## Objetivo técnico

Mostrar en el viewport la curva/recta exacta que cada dron tiene previsto recorrer actualmente, usando la representación canónica de trayectoria de Fase 6. No reconstruir el futuro a partir de posiciones pasadas.

## Comportamiento esperado

Cuando Fase 6 cree/replanifique una trayectoria, la layer del dron se reemplaza por la nueva trayectoria vigente. Al completarla/cancelarla y no existir otra activa, debe desaparecer o quedar vacía según el estado real. Si la trayectoria es polinómica/paramétrica, el renderer la muestrea para dibujar una línea suficientemente fiel; si ya llega como waypoints/muestras, respeta esa forma.

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

Fase 6 diferencia `task_id` y `trajectory_id`, usa trayectorias cortas y replanning. El snapshot documental no garantiza que publique la geometría actual como topic para visualización. Si esa telemetría no existe, no se puede inferir con el histórico de pose: debe reabrirse la subfase propietaria de Fase 6 y publicar el plan vigente.

## Invariantes y decisiones cerradas

- Mostrar solo trayectoria futura/current baseline; no historial completo de propuestas/reservas.
- `trajectory_id` distingue replans dentro de la misma tarea.
- La geometría visual no autoriza ni modifica reservas.
- El dato debe ser el plan que realmente consume el ejecutor.
- `world` como frame visual final; si Fase 6 conserva una trayectoria local por regla de Fase 5, usar el transform canónico vigente y no GT.
- Si lo dibujado no coincide con el plan real, revisar primero el mensaje del productor.

## Archivos permitidos a modificar

```text
src/servidor/multidron_gui/src/render/trajectory_layer.*
src/servidor/multidron_gui/src/ros_data_bridge.*
src/servidor/multidron_gui/src/gui_data_model.*
src/servidor/orbslam3_server/              # solo si es propietario de telemetría de reservas/trayectorias
src/dron/dron_individual/                  # solo reabriendo Fase 6 si el plan vigente solo existe embarcado
src/servidor/orbslam3_msgs/
src/dron/orbslam3_msgs/
```

Las rutas nuevas son de contrato. Antes de crearlas, comprobar el árbol posterior a Fases 2–6. Si existe un componente equivalente, reutilizarlo en lugar de duplicarlo.

## Archivos prohibidos

```text
histórico de poses usado como trayectoria futura
TrayAction directo desde GUI
algoritmo de replanning/reservas salvo corrección en Fase 6
ORB_SLAM3/
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
trajectory_id / task_id de Fase 6
representación multi-waypoint/lib_tray vigente
planner local / executor que posee trayectoria activa
telemetría/feedback de trayectoria si ya existe
TrajectoryLayer
```

Los nombres de componentes nuevos definidos por este contrato pueden implementarse con una estructura equivalente si existe una razón técnica clara. Los nombres de **interfaces procedentes de fases anteriores** no se inventan: se localizan físicamente primero.

## Cambios requeridos

1. Localizar la fuente de verdad de la trayectoria actualmente ejecutable por cada dron.
2. Si no existe topic/contrato de telemetría, detener Fase 7 y reabrir la subfase apropiada de Fase 6 para exponerla sin cambiar planificación.
3. Consumir `drone_id`, `task_id`, `trajectory_id`, frame y geometría suficiente.
4. Transformar/representar en `world` únicamente con transformaciones canónicas de Fase 5/6.
5. Muestrear curvas continuas con resolución visual configurable, evitando enviar miles de muestras por ROS si puede reconstruirse desde parámetros canónicos ya publicados.
6. Reemplazar el buffer cuando cambie `trajectory_id` y retirar al finalizar/cancelar.
7. Implementar toggle `Trajectories`.
8. Añadir `GUI-TRAJECTORY-UPDATE` con drone/task/trajectory/count, sin registrar todos los puntos.

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

Si se reabre Fase 6 para telemetría, compilar también los productores/interfases realmente modificados.

Si la separación de Fase 2 utiliza builds por grupo, usar el helper vigente para el grupo Servidor y, cuando haya pruebas de integración, los grupos Dron/Simulación correspondientes. Registrar el comando exacto solo en el historial real.

## Pruebas Gazebo requeridas

### Prueba 1 — Recta

Enviar/ejecutar un `GO_TO` cuyo plan sea aproximadamente recto. Verificar que la línea visual coincide con el plan vigente y termina en el objetivo.

### Prueba 2 — Curva

Provocar una trayectoria curvada/multi-waypoint real. Verificar que el dibujo sigue la curva y no une solo inicio/fin con una recta.

### Prueba 3 — Replanning

Durante una tarea, provocar un replan de Fase 6. Debe cambiar `trajectory_id` y la GUI debe sustituir la trayectoria mostrada sin mantener la anterior como activa.

No arrancar Gazebo artificialmente para una prueba puramente gráfica/unitaria. Cuando se use simulación, el comando base es:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_7_7G \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El mecanismo exacto para arrancar `multidron_gui` junto a ese launch se fija en 7A/7B y debe reutilizarse después. La GUI nunca depende de que RViz2 esté abierto.

## Patrones de reducción de logs

```text
GUI-TRAJECTORY-UPDATE|trajectory_id|task_id|TRAJ-LIFECYCLE|REPLAN-|TRAJ-RELEASE|GO-TO-TASK|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo alimentan reductores. Si falta evidencia, regenerar el reducido con patrones más precisos; no abrir el log completo directamente.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Cada dron con trayectoria activa muestra su plan vigente real.
2. Rectas y curvas se representan fielmente.
3. Replan reemplaza la trayectoria anterior según `trajectory_id`.
4. No se reconstruye trayectoria a partir del rastro de pose.
5. Si falta telemetría, se corrige Fase 6 y se revalida antes de cerrar 7G.

Además, todo build requerido debe devolver `0`, todas las pruebas obligatorias deben ejecutarse, no puede haber errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si se muestran waypoints pero no la curva realmente ejecutada cuando la diferencia es significativa.
- `PARCIAL` si la layer no distingue replans y deja varias trayectorias activas para el mismo dron.
- `BLOQUEADA` si Fase 6 no expone el plan vigente y requiere una modificación funcional no autorizada.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos o porque una fase anterior debe corregirse antes de continuar.

## Riesgos

- Saturar ROS publicando demasiadas muestras por trayectoria.
- Usar pose pasada como “trayectoria”.
- Mezclar trayectoria local congelada con `world` incorrectamente.
- Ocultar un error de Fase 6 deformando la curva en GUI.

## Documentación a actualizar


Al ejecutar realmente la subfase, actualizar solo documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7G.md
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7G_RESUMEN.md
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
