# Subfase 7J — Creación de tareas `GO_TO` desde la GUI

## Estado

```text
sin hacer
```

## Dependencia

7B/7C/7I conseguidas y Fase 6 con `GO_TO` operativo.

## Objetivo técnico

Conectar el formulario de tareas a la cadena real de misión para que un operador elija un dron e introduzca `(x,y,z,yaw)` absolutos en `world`, envíe `GO_TO` y reciba ACK/resultado/estado sin saltarse seguridad ni coordinación.

## Comportamiento esperado

El formulario valida números finitos, dron válido y campos completos. Al enviar, usa el service/action/API canónica de Fase 6. `GO_TO` conserva la semántica ya acordada: no preempciona una tarea `RUNNING` si Fase 6 define máxima prioridad pendiente, y usa planner local, obstacle avoidance y reservas antes de ejecutar.

La GUI muestra aceptación/rechazo y deja que la tarjeta 7I muestre progreso/resultado.

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

Fase 6 documenta `GO_TO(x,y,z,yaw)` para futura GUI, pero la interfaz exacta depende de la implementación real. No se debe inventar un publisher paralelo si ya existe service/action de task submission.

## Invariantes y decisiones cerradas

- Todos los objetivos son absolutos en `world`.
- Seleccionar un MapPoint no rellena/envía el objetivo automáticamente.
- `GO_TO` entra por TaskManager; nunca `TrayAction` directo.
- No introducir preempción nueva.
- Validar `flight_bounds`/alcanzabilidad en el backend, no confiar solo en GUI.
- La GUI puede hacer validación sintáctica, pero servidor sigue siendo autoridad de aceptación.

## Archivos permitidos a modificar

```text
src/servidor/multidron_gui/src/widgets/task_creation_panel.*
src/servidor/multidron_gui/src/ros_command_bridge.*
src/servidor/multidron_gui/include/multidron_gui/...
src/servidor/orbslam3_server/              # solo integración de endpoint real si Fase 6 lo dejó incompleto
src/servidor/orbslam3_msgs/
src/dron/orbslam3_msgs/
```

Las rutas nuevas son de contrato. Antes de crearlas, comprobar el árbol posterior a Fases 2–6. Si existe un componente equivalente, reutilizarlo en lugar de duplicarlo.

## Archivos prohibidos

```text
TrayAction enviado por GUI
control/motores directos
goal basado en coordenadas de pixel/MapPoint click automático
GT
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
GO_TO Task contract de Fase 6
TaskManager / endpoint de submit
flight_bounds / validación world
TaskCreationPanel
RosCommandBridge
feedback/ACK/result de task submission
```

Los nombres de componentes nuevos definidos por este contrato pueden implementarse con una estructura equivalente si existe una razón técnica clara. Los nombres de **interfaces procedentes de fases anteriores** no se inventan: se localizan físicamente primero.

## Cambios requeridos

1. Añadir selector de dron y tipo `GO_TO` al formulario.
2. Añadir campos numéricos X/Y/Z/Yaw y unidades visibles/documentadas.
3. Validar formato/NaN/inf/campos vacíos antes de enviar.
4. Enviar por el service/action/API real de Fase 6 y conservar request/task ID para feedback.
5. Deshabilitar temporalmente doble submit de la misma interacción si el endpoint no es idempotente; no bloquear toda la GUI mientras espera.
6. Mostrar ACK/rechazo/razón de forma clara.
7. Verificar que un GO_TO durante otra tarea sigue la política de prioridad/preempción de Fase 6.
8. Verificar en 7G que la trayectoria mostrada corresponde al GO_TO finalmente aceptado.
9. Añadir `GUI-GO-TO-SUBMIT`, `GUI-GO-TO-ACK`, `GUI-GO-TO-REJECT`.

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
./codex/herramientas/build_selected_packages.sh multidron_gui orbslam3_server orbslam3_msgs
```

Ajustar al conjunto real de paquetes modificados; no tocar Dron si el endpoint ya estaba completo.

Si la separación de Fase 2 utiliza builds por grupo, usar el helper vigente para el grupo Servidor y, cuando haya pruebas de integración, los grupos Dron/Simulación correspondientes. Registrar el comando exacto solo en el historial real.

## Pruebas Gazebo requeridas

### Prueba 1 — GO_TO válido

Elegir un dron, introducir pose world válida y enviar. Debe aparecer una tarea real y, cuando corresponda, el dron moverse mediante planner/reservas. La trayectoria actual debe verse en 7G.

### Prueba 2 — Entrada inválida

Campos vacíos, NaN/no numérico o dron inexistente: rechazo local o backend explícito sin crash.

### Prueba 3 — Fuera de límites

Enviar pose fuera de `flight_bounds`. El backend debe rechazar/fallar con razón de Fase 6; la GUI no puede obligar al dron a ejecutarla.

### Prueba 4 — GO_TO mientras hay tarea RUNNING

Comprobar que no se introduce preempción distinta a la acordada en Fase 6.

No arrancar Gazebo artificialmente para una prueba puramente gráfica/unitaria. Cuando se use simulación, el comando base es:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_7_7J \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El mecanismo exacto para arrancar `multidron_gui` junto a ese launch se fija en 7A/7B y debe reutilizarse después. La GUI nunca depende de que RViz2 esté abierto.

## Patrones de reducción de logs

```text
GUI-GO-TO|GO-TO-TASK|TASK-STATE|TASK-ALLOC|TRAJ-CONFLICT|LOCAL-OBSTACLE|flight_bounds|task_id|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo alimentan reductores. Si falta evidencia, regenerar el reducido con patrones más precisos; no abrir el log completo directamente.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. `GO_TO` se envía desde GUI y llega al TaskManager real.
2. Objetivo usa `(x,y,z,yaw)` en `world`.
3. Planner/obstacle avoidance/reservas siguen activos.
4. ACK/rechazo y resultado son visibles sin bloquear UI.
5. No hay `TrayAction` directo ni preempción nueva.
6. La trayectoria mostrada coincide con la ejecutada.

Además, todo build requerido debe devolver `0`, todas las pruebas obligatorias deben ejecutarse, no puede haber errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si la GUI envía la tarea pero no muestra ACK/rechazo.
- `PARCIAL` si `GO_TO` funciona solo cuando no hay otras tareas y contradice la cola acordada.
- `NO CONSEGUIDA` si se salta TaskManager/reservas.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos o porque una fase anterior debe corregirse antes de continuar.

## Riesgos

- Doble submit por doble click.
- Confundir grados/radianes de yaw; reutilizar unidad del contrato real y mostrarla.
- Validar límites solo en GUI y dejar backend vulnerable.
- Enviar a drone_id incorrecto por orden visual de cards.

## Documentación a actualizar


Al ejecutar realmente la subfase, actualizar solo documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7J.md
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7J_RESUMEN.md
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
