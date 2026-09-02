# Subfase 7D — Motor 3D, grid y cámara interactiva

## Estado

```text
conseguida
```

## Dependencia

7B y 7C conseguidas.

## Objetivo técnico

Crear el viewport 3D propio con OpenGL, grid en `world`, cámara perspectiva y controles de navegación tipo visor 3D: orbit/rotación, pan, zoom y resize. Debe existir una arquitectura de `RenderLayer` que permita añadir datos reales sin reescribir el motor.

## Comportamiento esperado

El viewport debe renderizar de manera continua y fluida aun cuando no lleguen topics nuevos. La cámara no modifica datos ROS. Un objeto sintético colocado en coordenadas conocidas debe mantenerse coherente al rotar/zoom/pan. El grid es referencia visual, no parte del mapa.

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

Hasta 7C existe ventana/modelo, pero no renderer propio. La guía original menciona RViz2 como referencia funcional, pero el acuerdo es no usarlo. Esta subfase debe demostrar que la navegación 3D básica funciona antes de introducir una nube real.

## Invariantes y decisiones cerradas

- `world` es frame de escena.
- OpenGL vive en el thread Qt/contexto válido.
- Layers separadas y activables; `GridLayer` es la primera.
- La cámara es solo visual y no envía comandos a drones.
- No se exige replicar todos los controles de RViz2; solo navegación acordada.
- No usar estilos/engine 3D externos que conviertan la GUI en integración de otro software.

## Archivos permitidos a modificar

```text
src/servidor/multidron_gui_lib/src/render/
src/servidor/multidron_gui_lib/include/multidron_gui_lib/render/
src/servidor/multidron_gui_lib/src/scene3d_widget.*
src/servidor/multidron_gui_lib/include/multidron_gui_lib/scene3d_widget.*
src/servidor/multidron_gui_lib/test/
```

Las rutas nuevas son de contrato. Antes de crearlas, comprobar el árbol posterior a Fases 2–6. Si existe un componente equivalente, reutilizarlo en lugar de duplicarlo.

## Archivos prohibidos

```text
RViz2 plugins/configs
WebGL/pipeline_flow
src/servidor/orbslam3_server/
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
Scene3DWidget / QOpenGLWidget equivalente
initializeGL / resizeGL / paintGL equivalentes
CameraController
RenderLayer interfaz
GridLayer
shader/buffer helpers
mouse/wheel events
```

Los nombres de componentes nuevos definidos por este contrato pueden implementarse con una estructura equivalente si existe una razón técnica clara. Los nombres de **interfaces procedentes de fases anteriores** no se inventan: se localizan físicamente primero.

## Cambios requeridos

1. Crear contexto OpenGL y shaders mínimos con manejo explícito de errores de compilación/link.
2. Implementar matrices view/projection y convención de ejes coherente con `world` del proyecto; no introducir conversiones ocultas.
3. Implementar grid horizontal con escala configurable visual.
4. Implementar orbit alrededor de un target, pan y zoom mediante ratón/rueda.
5. Gestionar resize, DPI y clipping near/far sin desaparecer geometría normal de la casa.
6. Crear interfaz `RenderLayer` con `setVisible`, actualización de buffers y `render` o equivalente.
7. Añadir una capa/objeto sintético de ejes/puntos para test; retirarlo del runtime normal.
8. Medir FPS/tiempo de frame de forma opcional y añadir `GUI-RENDER-READY`, `GUI-CAMERA`, `GUI-GL-ERROR`.

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

Si la separación de Fase 2 utiliza builds por grupo, usar el helper vigente para el grupo Servidor y, cuando haya pruebas de integración, los grupos Dron/Simulación correspondientes. Registrar el comando exacto solo en el historial real.

## Pruebas Gazebo requeridas

### Prueba 1 — Navegación 3D sintética

Sin Gazebo, dibujar grid + varios puntos/segmentos en posiciones conocidas. Rotar, pan, zoom y resize durante varios minutos sin artefactos/crash.

### Prueba 2 — Coordenadas conocidas

Colocar objetos sintéticos en ejes positivos/negativos y verificar visualmente que la convención coincide con `world` documentado; no corregir por intuición si existe duda de frames.

### Prueba 3 — Backend activo

Con simulación activa pero layers de datos aún ocultas, mover la cámara mientras ROS recibe mensajes. La interacción no debe congelarse.

No arrancar Gazebo artificialmente para una prueba puramente gráfica/unitaria. Cuando se use simulación, el comando base es:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_7_7D \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El mecanismo exacto para arrancar `multidron_gui` junto a ese launch se fija en 7A/7B y debe reutilizarse después. La GUI nunca depende de que RViz2 esté abierto.

## Patrones de reducción de logs

```text
GUI-RENDER-READY|GUI-CAMERA|GUI-FRAME|GUI-GL-ERROR|OpenGL|shader|FPS|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo alimentan reductores. Si falta evidencia, regenerar el reducido con patrones más precisos; no abrir el log completo directamente.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Grid y objetos 3D se renderizan en un viewport propio.
2. Orbit/pan/zoom/resize funcionan de forma estable.
3. La convención del frame está documentada y no invierte ejes silenciosamente.
4. Existe API de layers reutilizable para 7E–7L.
5. No hay dependencia de RViz2 ni bloqueo ROS.

Además, todo build requerido debe devolver `0`, todas las pruebas obligatorias deben ejecutarse, no puede haber errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si el viewport dibuja pero la cámara tiene clipping/resize que impide operar.
- `PARCIAL` si los layers todavía dependen de código hardcoded en `paintGL` y no son extensibles.
- `PARCIAL` si el backend se bloquea por el ritmo de render.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos o porque una fase anterior debe corregirse antes de continuar.

## Riesgos

- Errores Tcw/Twc o handedness disfrazados como “problema visual”.
- Recrear shaders/VBOs completos en cada frame.
- Near/far planes tan extremos que destruyan precisión de profundidad.
- Event handling que interfiera con click de picking futuro.

## Documentación a actualizar


Al ejecutar realmente la subfase, actualizar solo documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7D.md
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7D_RESUMEN.md
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
