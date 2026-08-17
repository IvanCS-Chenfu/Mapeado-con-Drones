# Subfase 7C — Modelo de datos y bridge ROS 2 asíncrono

## Estado

```text
sin hacer
```

## Dependencia

7A y 7B conseguidas.

## Objetivo técnico

Implementar la frontera robusta entre ROS 2 y la interfaz: subscriptions/clientes actualizan estado en memoria y el thread Qt obtiene snapshots coherentes sin bloquear callbacks ni tocar widgets desde threads ROS.

## Comportamiento esperado

Cada tipo de dato debe tener revisión/timestamp suficiente para descartar updates obsoletos cuando el productor lo proporcione. La llegada de una nube grande no debe congelar la GUI ni mantener locks largos. El modelo puede notificar “datos cambiados”, pero la renderización ocurre en el thread de GUI.

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

El proyecto ya posee publishers de mapa y tendrá publishers de pose/tarea/trayectoria, pero no existe una capa de cache específica para la GUI. Conectar directamente callbacks a OpenGL produciría carreras y bloqueo del executor.

## Invariantes y decisiones cerradas

- `GuiDataModel` es memoria de visualización, no fuente de verdad funcional.
- Nunca escribir widgets/OpenGL desde callback ROS.
- Preferir swap/snapshot breve de buffers y procesamiento pesado fuera de locks.
- Mantener timestamps/revisiones del productor; no generar una revisión “más verdadera” en GUI.
- Datos ausentes/stale deben representarse como estado de disponibilidad, no reutilizarse como válidos sin contrato.
- La política visual concreta de un estado perdido/stale no se decide aquí si no está acordada: preguntar al usuario cuando aparezca.

## Archivos permitidos a modificar

```text
src/servidor/multidron_gui/src/ros_data_bridge.*
src/servidor/multidron_gui/include/multidron_gui/ros_data_bridge.*
src/servidor/multidron_gui/src/gui_data_model.*
src/servidor/multidron_gui/include/multidron_gui/gui_data_model.*
src/servidor/multidron_gui/src/main.*
src/servidor/multidron_gui/test/
```

Las rutas nuevas son de contrato. Antes de crearlas, comprobar el árbol posterior a Fases 2–6. Si existe un componente equivalente, reutilizarlo en lugar de duplicarlo.

## Archivos prohibidos

```text
ORB_SLAM3/
src/servidor/orbslam3_multi/
algoritmos de Fase 3–6
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
rclcpp executor del paquete GUI
subscriptions inventariadas en 7A
GuiDataModel
snapshot/copy/swap de cada dataset
lifecycle de QApplication + rclcpp::shutdown
QoS real de cada topic
```

Los nombres de componentes nuevos definidos por este contrato pueden implementarse con una estructura equivalente si existe una razón técnica clara. Los nombres de **interfaces procedentes de fases anteriores** no se inventan: se localizan físicamente primero.

## Cambios requeridos

1. Implementar `RosDataBridge` con subscriptions reales o adaptadores tipados definidos por 7A.
2. Implementar `GuiDataModel` separado por datasets: sparse, dense, drones, KFs, fiduciales, trayectorias, tareas.
3. Para datos grandes, usar ownership move/swap o buffers inmutables compartidos para minimizar copias y locks.
4. Mantener `received_at`, timestamp del mensaje y revisión/map_epoch cuando existan.
5. Implementar señales thread-safe hacia Qt indicando dataset actualizado, sin transportar payload pesado por señales si eso duplica memoria.
6. Implementar shutdown ordenado: detener executor, destruir subscriptions, cerrar ventana y terminar sin deadlock.
7. Añadir tests con publishers sintéticos que envíen mensajes rápidos, reordenados y vacíos.
8. Añadir métricas/logs acotados `GUI-DATA-UPDATE`, `GUI-SNAPSHOT`, `GUI-DATA-STALE` solo en debug/configurable para no inundar logs.

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

Si la separación de Fase 2 utiliza builds por grupo, usar el helper vigente para el grupo Servidor y, cuando haya pruebas de integración, los grupos Dron/Simulación correspondientes. Registrar el comando exacto solo en el historial real.

## Pruebas Gazebo requeridas

### Prueba 1 — Flood sintético

Publicar actualizaciones pequeñas a alta frecuencia y varias nubes de tamaño creciente. La GUI debe seguir respondiendo al resize/cierre mientras el modelo recibe datos.

### Prueba 2 — Reordenación/stale

Cuando la interfaz real tenga timestamp/revisión, enviar una actualización antigua después de una nueva y comprobar que el modelo no retrocede silenciosamente si el contrato permite rechazarla.

### Prueba 3 — Shutdown bajo carga

Cerrar la GUI mientras entran mensajes. Debe terminar sin deadlock/use-after-free y el backend debe continuar.

No arrancar Gazebo artificialmente para una prueba puramente gráfica/unitaria. Cuando se use simulación, el comando base es:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_7_7C \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El mecanismo exacto para arrancar `multidron_gui` junto a ese launch se fija en 7A/7B y debe reutilizarse después. La GUI nunca depende de que RViz2 esté abierto.

## Patrones de reducción de logs

```text
GUI-DATA-UPDATE|GUI-SNAPSHOT|GUI-DATA-STALE|GUI-ROS-READY|shutdown|revision|timestamp|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo alimentan reductores. Si falta evidencia, regenerar el reducido con patrones más precisos; no abrir el log completo directamente.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Ningún callback ROS toca widgets ni contexto OpenGL.
2. El modelo entrega snapshots coherentes y los locks son breves.
3. El cierre bajo carga es limpio.
4. Datos grandes no congelan la UI por copias/locks evitables.
5. Revisión/timestamp/epoch se conservan cuando los productores los proporcionan.

Además, todo build requerido debe devolver `0`, todas las pruebas obligatorias deben ejecutarse, no puede haber errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si funciona a baja frecuencia pero se bloquea con nubes grandes.
- `PARCIAL` si callbacks y GUI comparten contenedores mutables sin snapshot seguro.
- `PARCIAL` si shutdown requiere matar el proceso.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos o porque una fase anterior debe corregirse antes de continuar.

## Riesgos

- Copias completas repetidas de PointCloud2 en cada frame gráfico.
- Locks de larga duración entre ROS y renderer.
- Eventos Qt encolando payloads de cientos de MB.
- Mantener como válida una pose/tarea stale solo porque fue la última recibida.

## Documentación a actualizar


Al ejecutar realmente la subfase, actualizar solo documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7C.md
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7C_RESUMEN.md
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
