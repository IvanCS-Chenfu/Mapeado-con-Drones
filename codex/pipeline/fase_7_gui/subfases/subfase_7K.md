# Subfase 7K — `CAPTURE_SPARSE` y preparación de `CAPTURE_DENSE`

## Estado

```text
sin hacer
```

## Dependencia

7J conseguida; Fase 3/wrapper produce KFs identificables; Fase 6 permite tareas manuales. Fase 8 todavía no es requisito.

## Objetivo técnico

Añadir al formulario las dos órdenes de captura acordadas. `CAPTURE_SPARSE` debe ser funcional sin modificar ORB-SLAM3: viajar a `(x,y,z,yaw)` y esperar un KF creado naturalmente. `CAPTURE_DENSE` queda contratada/preparada para Fase 8 sin simular una reconstrucción inexistente.

## Comportamiento esperado

`CAPTURE_SPARSE` reutiliza exactamente la navegación `GO_TO`. Tras llegada/estabilización, registra el estado actual `(drone_id,map_epoch)` y espera un KF **nuevo**, posterior a la fase de llegada, asociado a ese dron/epoch y razonablemente correspondiente a la pose solicitada según tolerancias canónicas. Si aparece, completa. Si no aparece dentro del límite acordado, falla. El operador decide después otra pose/orientación.

`CAPTURE_DENSE` presenta los mismos campos `(x,y,z,yaw)`. Antes de Fase 8 no puede afirmar captura densa exitosa. El contrato debe permitir que Fase 8 conecte el disparo real sin rediseñar la GUI.

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

Fase 6 solo cerró baseline `MAP_SECTION`, `GO_TO` y `ANCHOR_SUBMAP`. `CAPTURE_SPARSE/DENSE` son ampliaciones de Fase 7. ORB-SLAM3 decide cuándo insertar KFs; no existe acuerdo para forzarlos. La interfaz real del wrapper/server para detectar KFs nuevos debe localizarse.

## Invariantes y decisiones cerradas

- No forzar KeyFrames ni tocar heurísticas de inserción ORB-SLAM3.
- Un KF previo a la llegada no satisface la captura.
- Debe pertenecer al dron y `map_epoch` vigentes.
- Si no aparece KF, resultado `FAILED`, no éxito aproximado.
- El operador puede mandar otra tarea después; la GUI no genera automáticamente maniobras de búsqueda.
- `CAPTURE_DENSE` real pertenece a Fase 8.
- Las tres órdenes GUI usan coordenadas absolutas world.
- Prioridad/preempción hereda el contrato manual de Fase 6; no crear reglas paralelas.

## Archivos permitidos a modificar

```text
src/servidor/multidron_gui/src/widgets/task_creation_panel.*
src/servidor/multidron_gui/src/ros_command_bridge.*
src/servidor/orbslam3_server/              # TaskManager/observador de KFs si es propietario
src/servidor/orbslam3_msgs/
src/dron/orbslam3_msgs/                    # réplica de interfaces si aplica
src/dron/dron_individual/                  # solo si TaskExecutor necesita handler y tras respetar ownership Fase 6
codex/contexto/paquetes/
```

Las rutas nuevas son de contrato. Antes de crearlas, comprobar el árbol posterior a Fases 2–6. Si existe un componente equivalente, reutilizarlo en lugar de duplicarlo.

## Archivos prohibidos

```text
ORB_SLAM3/                                 # no forzar KF
reconstrucción Open3D/TSDF de Fase 8
captura “densa” falsa con PointCloud synthetic como resultado funcional
maniobra automática tras fallo sparse no acordada
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
Task contract/TaskManager de Fase 6
GO_TO handler
(drone_id,map_epoch,local_kf_id) / stream de KFs de Fase 3
pose de KF global y timestamp si existe
criterio de llegada GO_TO
CAPTURE_SPARSE / CAPTURE_DENSE tipos de tarea
capability/endpoint futuro de Fase 8 si ya fue esbozado, sin implementarlo
```

Los nombres de componentes nuevos definidos por este contrato pueden implementarse con una estructura equivalente si existe una razón técnica clara. Los nombres de **interfaces procedentes de fases anteriores** no se inventan: se localizan físicamente primero.

## Cambios requeridos

1. Extender el contrato de tareas compartido con `CAPTURE_SPARSE` y `CAPTURE_DENSE` sin romper copias Dron/Servidor ni tipos anteriores.
2. En GUI, mostrar ambos tipos con selector de dron y campos X/Y/Z/Yaw.
3. Implementar `CAPTURE_SPARSE` como composición/estado del TaskManager: navegación por la misma cadena de `GO_TO`, luego estado `WAITING_FOR_KF` o equivalente.
4. Capturar al entrar en espera el `map_epoch` y la frontera temporal/ID para distinguir KFs nuevos.
5. Aceptar únicamente un KF posterior y del mismo dron/epoch. Usar pose/tolerancias existentes para asociarlo cuando estén disponibles.
6. Definir timeout configurable. Reutilizar convenciones existentes; si fijar un nuevo valor funcional no está cubierto por parámetros previos, preguntar al usuario durante preparación antes de elegirlo.
7. Ante timeout, finalizar `FAILED` con motivo explícito; no seleccionar silenciosamente “el KF más cercano” antiguo.
8. Para `CAPTURE_DENSE`, crear solo el tipo/contrato y punto de integración. Si Fase 8 no está disponible, backend debe rechazar/indicar capacidad no disponible y GUI no informar éxito.
9. Añadir `GUI-CAPTURE-SPARSE`, `CAPTURE-SPARSE-WAIT-KF`, `CAPTURE-SPARSE-SUCCESS/FAILED`, `GUI-CAPTURE-DENSE-UNAVAILABLE` o markers equivalentes.

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
./codex/herramientas/build_selected_packages.sh multidron_gui orbslam3_msgs orbslam3_server dron_individual
```

Reducir el conjunto si el handler de tarea queda enteramente en Servidor; respetar builds por grupo de Fase 2.

Si la separación de Fase 2 utiliza builds por grupo, usar el helper vigente para el grupo Servidor y, cuando haya pruebas de integración, los grupos Dron/Simulación correspondientes. Registrar el comando exacto solo en el historial real.

## Pruebas Gazebo requeridas

### Prueba 1 — CAPTURE_SPARSE con KF natural

Elegir una vista con textura donde ORB-SLAM3 genere un KF tras llegar. Confirmar orden: tarea aceptada -> navegación -> llegada -> espera -> KF nuevo mismo epoch -> `COMPLETED`.

### Prueba 2 — CAPTURE_SPARSE sin KF

Elegir/crear una situación donde no aparezca KF en la ventana acordada. Debe terminar `FAILED`, mantener el sistema operativo y permitir que el usuario envíe después otro GO_TO/CAPTURE.

### Prueba 3 — KF inválido

Un KF antiguo, de otro dron o de otro epoch no puede completar la tarea.

### Prueba 4 — CAPTURE_DENSE antes de Fase 8

Intentar la orden con backend dense ausente. Debe quedar explícitamente no disponible/rechazada, nunca completar con datos sintéticos como si fueran reales.

No arrancar Gazebo artificialmente para una prueba puramente gráfica/unitaria. Cuando se use simulación, el comando base es:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_7_7K \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El mecanismo exacto para arrancar `multidron_gui` junto a ese launch se fija en 7A/7B y debe reutilizarse después. La GUI nunca depende de que RViz2 esté abierto.

## Patrones de reducción de logs

```text
GUI-CAPTURE-SPARSE|CAPTURE-SPARSE|WAITING_FOR_KF|map_epoch|local_kf_id|GO-TO-TASK|CAPTURE-DENSE|UNAVAILABLE|TASK-STATE|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo alimentan reductores. Si falta evidencia, regenerar el reducido con patrones más precisos; no abrir el log completo directamente.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. `CAPTURE_SPARSE` usa navegación segura de Fase 6 y no toca ORB-SLAM3.
2. Solo un KF nuevo y del dron/epoch correcto puede completar.
3. El caso sin KF falla de forma explícita y recuperable.
4. No se usa un KF antiguo “cercano” como éxito silencioso.
5. `CAPTURE_DENSE` queda representada por contrato/UI pero no finge implementación antes de Fase 8.
6. Las interfaces compartidas siguen coherentes entre grupos.

Además, todo build requerido debe devolver `0`, todas las pruebas obligatorias deben ejecutarse, no puede haber errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si CAPTURE_SPARSE navega pero el backend no puede asociar de forma fiable un KF nuevo.
- `PARCIAL` si CAPTURE_DENSE no tiene aún endpoint futuro pero la GUI está preparada y lo declara no disponible.
- `NO CONSEGUIDA` si se modifica ORB-SLAM3 para forzar KFs o se informa éxito con KF antiguo.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos o porque una fase anterior debe corregirse antes de continuar.

## Riesgos

- Timeout demasiado corto/largo decidido arbitrariamente.
- Race entre llegada, cambio de epoch y creación de KF.
- Completar con un KF que ORB creó antes de alcanzar la pose.
- Empezar a implementar Fase 8 por querer “probar” CAPTURE_DENSE.

## Documentación a actualizar


Al ejecutar realmente la subfase, actualizar solo documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7K.md
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7K_RESUMEN.md
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
