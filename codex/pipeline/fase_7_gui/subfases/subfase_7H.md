# Subfase 7H — Picking 3D e inspector genérico de entidades

## Estado

```text
conseguida
```

## Dependencia

7D, 7E y 7F conseguidas.

## Objetivo técnico

Implementar selección con ratón en el viewport. El requisito mínimo es clicar un MapPoint y mostrar su posición `x,y,z` en `world`; la infraestructura debe ser extensible a KFs, drones, fiduciales y otras entidades renderizadas.

## Comportamiento esperado

Un click genera un rayo/consulta de selección contra entidades visibles. Se elige una entidad razonable según proximidad/ID del render y se almacena como `SelectedEntity`. El inspector muestra tipo y metadata real. La entidad seleccionada queda resaltada. Cambiar cámara o filtros no debe convertir una entidad invisible en un objetivo de tarea automático.

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

No existe selección en la GUI nueva. El número de MapPoints puede ser grande, por lo que un scan ingenuo de todos los puntos por click puede ser aceptable al principio solo si se mide; GPU ID buffer, spatial index o proyección CPU son decisiones técnicas permitidas según rendimiento.

## Invariantes y decisiones cerradas

- Click en MapPoint no crea ni envía tareas.
- Coordenadas mostradas son `world`.
- Solo se pueden seleccionar entidades actualmente conocidas/visibles según la política de layer.
- La selección conserva identidad, no solo coordenadas, cuando el productor la ofrece.
- La selección debe invalidarse/actualizarse con seguridad si la revisión nueva elimina la entidad.
- El estilo exacto de highlight es técnico; no debe ocultar otros datos.

## Archivos permitidos a modificar

```text
src/servidor/multidron_gui_lib/src/selection/
src/servidor/multidron_gui_lib/include/multidron_gui_lib/selection/
src/servidor/multidron_gui_lib/src/render/selection_layer.*
src/servidor/multidron_gui_lib/src/widgets/selection_inspector.*
src/servidor/multidron_gui_lib/src/scene3d_widget.*
```

Las rutas nuevas son de contrato. Antes de crearlas, comprobar el árbol posterior a Fases 2–6. Si existe un componente equivalente, reutilizarlo en lugar de duplicarlo.

## Archivos prohibidos

```text
task_server / task_manager / planner / GO_TO backend
map databases de Fase 3
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
Scene3DWidget camera matrices
SparseMapLayer point metadata
KeyFrameLayer metadata
DronePoseLayer metadata
FiducialLayer metadata
SelectionInspector
mouse click event distinguible de drag/orbit
```

Los nombres de componentes nuevos definidos por este contrato pueden implementarse con una estructura equivalente si existe una razón técnica clara. Los nombres de **interfaces procedentes de fases anteriores** no se inventan: se localizan físicamente primero.

## Cambios requeridos

1. Definir `SelectedEntity` con tipo, stable ID opcional, posición world y metadata ligera.
2. Implementar ray/picking para MapPoints con tolerancia razonable en pantalla; medir coste con nube real.
3. Extender a KFs/drones/fiduciales mediante bounding primitive, ID buffer o mecanismo equivalente.
4. Distinguir click corto de drag de cámara para no seleccionar mientras se rota.
5. Resaltar la entidad seleccionada con `SelectionLayer` o estado de layer.
6. Mostrar para MapPoint al menos `x`, `y`, `z`; añadir score/IDs solo si reales.
7. Mostrar campos útiles de KFs/drones/fiduciales sin inventar datos.
8. Gestionar selección que deja de existir tras una nueva revisión sin use-after-free.
9. Añadir `GUI-PICK`/`GUI-SELECTION-CLEAR` con tipo/ID, no coordenadas masivas.

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

### Prueba 1 — Picking sintético

Puntos/objetos aislados y superpuestos en profundidad. Confirmar que el click selecciona el elemento esperado bajo distintas cámaras.

### Prueba 2 — MapPoint real

Seleccionar varios puntos del sparse. Comparar `x,y,z` mostrados con el dato almacenado en `GuiDataModel`.

### Prueba 3 — Entidades distintas

Seleccionar al menos un KF, un dron y un fiducial si están disponibles; comprobar inspector y highlight.

### Prueba 4 — Revisión nueva

Mantener un punto seleccionado y sustituir la nube. La selección no puede apuntar a memoria liberada ni mostrar metadata falsa.

No arrancar Gazebo artificialmente para una prueba puramente gráfica/unitaria. Cuando se use simulación, el comando base es:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_7_7H \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El mecanismo exacto para arrancar `multidron_gui` junto a ese launch se fija en 7A/7B y debe reutilizarse después. La GUI nunca depende de que RViz2 esté abierto.

## Patrones de reducción de logs

```text
GUI-PICK|GUI-SELECTION|GUI-SELECTION-CLEAR|MapPoint|KeyFrame|Fiducial|Drone|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo alimentan reductores. Si falta evidencia, regenerar el reducido con patrones más precisos; no abrir el log completo directamente.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. MapPoints reales se pueden seleccionar y muestran coordenadas correctas.
2. Click y orbit/pan no interfieren de forma grave.
3. La infraestructura soporta varias clases de entidad.
4. La selección es segura entre revisiones.
5. No se envían tareas como efecto lateral de un click.

Además, todo build requerido debe devolver `0`, todas las pruebas obligatorias deben ejecutarse, no puede haber errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si MapPoints funcionan pero el algoritmo es demasiado lento con la nube real.
- `PARCIAL` si otras entidades todavía no son seleccionables aunque la API genérica exista; documentar cuál falta y por qué.
- `PARCIAL` si la selección queda stale tras actualizar layers.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos o porque una fase anterior debe corregirse antes de continuar.

## Riesgos

- Picking O(N) costoso con cientos de miles/millones de puntos.
- Confundir click con drag.
- Guardar punteros a buffers GPU/modelos reemplazados.
- Convertir accidentalmente el punto seleccionado en target de GO_TO.

## Documentación a actualizar


Al ejecutar realmente la subfase, actualizar solo documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7H.md
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7H_RESUMEN.md
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
