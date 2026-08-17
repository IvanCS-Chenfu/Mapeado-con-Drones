# Subfase 7L — Capa de nube densa y rendimiento/extensibilidad del renderer

## Estado

```text
sin hacer
```

## Dependencia

7D/7E conseguidas. No depende de que Fase 8 esté implementada.

## Objetivo técnico

Preparar `DenseMapLayer` y la infraestructura de GPU/memoria para que Fase 8 pueda entregar una nube densa global grande sin rediseñar la GUI. Validar rendimiento con datos sintéticos o replay no funcional.

## Comportamiento esperado

La toolbar dispone de `Dense ON/OFF`. Si no hay publisher de Fase 8, la layer permanece sin datos y la GUI sigue funcionando. Para test se pueden inyectar nubes sintéticas de tamaños crecientes a la cache GUI, pero esos datos nunca se publican como resultado real del proyecto.

Cuando llegue una revisión densa real en Fase 8, la GUI debe poder reemplazar/actualizar buffers sin bloquear el executor ni copiar toda la nube en cada frame.

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

Fase 8 aún no existe funcionalmente en el orden del proyecto. Diseñar el renderer solo para sparse puede provocar una reescritura cuando haya millones de puntos. Ya existen QOpenGL/VBO helpers de 7D/7E que deben generalizarse.

## Invariantes y decisiones cerradas

- No implementar generación/fusión densa.
- `DenseMapLayer` es consumidora.
- Ausencia de datos dense no es error de Fase 7.
- Toggle no solicita parar/arrancar reconstrucción; solo visibilidad.
- Preferir buffers persistentes/actualizaciones por revisión y evitar gl upload por frame.
- El contrato final del topic dense se cerrará en Fase 8; 7L define una interfaz interna/adaptador flexible.

## Archivos permitidos a modificar

```text
src/servidor/multidron_gui/src/render/dense_map_layer.*
src/servidor/multidron_gui/src/render/point_cloud_buffer.*
src/servidor/multidron_gui/src/gui_data_model.*
src/servidor/multidron_gui/test/
```

Las rutas nuevas son de contrato. Antes de crearlas, comprobar el árbol posterior a Fases 2–6. Si existe un componente equivalente, reutilizarlo en lugar de duplicarlo.

## Archivos prohibidos

```text
src/servidor/orbslam3_multi/ dense inexistente
Open3D/TSDF/fusion backend
cámaras/depth pipeline de Fase 8 salvo mock/replay externo de prueba
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
SparseMapLayer/VBO helpers
DenseMapLayer
GuiDataModel::dense o equivalente
memory/FPS instrumentation
future adapter point format
```

Los nombres de componentes nuevos definidos por este contrato pueden implementarse con una estructura equivalente si existe una razón técnica clara. Los nombres de **interfaces procedentes de fases anteriores** no se inventan: se localizan físicamente primero.

## Cambios requeridos

1. Generalizar helpers de point cloud para soportar buffers grandes y atributos opcionales de color.
2. Implementar `DenseMapLayer` vacía por defecto con toggle.
3. Definir API interna para reemplazo por revisión/chunks sin fijar todavía el mensaje ROS definitivo de Fase 8.
4. Crear generador/test local de puntos sintéticos con varios tamaños; no conectarlo al runtime normal.
5. Medir upload time, frame time y memoria aproximada con sparse+dense sintético simultáneos.
6. Evitar duplicados innecesarios CPU/GPU; liberar buffers al ocultar solo si la política de memoria lo justifica, no en cada toggle.
7. Probar que la cámara sigue fluida al actualizar una nube grande en background/cache.
8. Añadir markers `GUI-DENSE-UPDATE`, `GUI-GPU-UPLOAD`, `GUI-RENDER-PERF` configurables.

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

### Prueba 1 — Dense ausente

Ejecutar GUI real antes de Fase 8. Activar/desactivar `Dense`; no debe aparecer error grave ni nube falsa.

### Prueba 2 — Nubes sintéticas crecientes

Cargar datasets deterministas de tamaño pequeño/medio/grande según memoria del equipo. Medir FPS, tiempo de upload y capacidad de cámara interactiva.

### Prueba 3 — Sparse + dense sintético

Mantener sparse real o sintético visible y añadir dense grande. Alternar layers, score sparse y navegación sin bloqueo del executor.

### Prueba 4 — Update de revisión

Reemplazar nube dense sintética mientras se mueve la cámara. Debe cambiar buffer de forma segura sin use-after-free.

No arrancar Gazebo artificialmente para una prueba puramente gráfica/unitaria. Cuando se use simulación, el comando base es:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_7_7L \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El mecanismo exacto para arrancar `multidron_gui` junto a ese launch se fija en 7A/7B y debe reutilizarse después. La GUI nunca depende de que RViz2 esté abierto.

## Patrones de reducción de logs

```text
GUI-DENSE-UPDATE|GUI-GPU-UPLOAD|GUI-RENDER-PERF|FPS|memory|buffer|OpenGL|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo alimentan reductores. Si falta evidencia, regenerar el reducido con patrones más precisos; no abrir el log completo directamente.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Dense layer existe y puede permanecer vacía sin fallar.
2. Toggle es puramente visual.
3. Renderer soporta nubes significativamente mayores que sparse sin rediseño arquitectónico.
4. Actualizaciones no se realizan cada frame si no cambian datos.
5. No se implementa ni simula como real ninguna reconstrucción de Fase 8.

Además, todo build requerido debe devolver `0`, todas las pruebas obligatorias deben ejecutarse, no puede haber errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si DenseMapLayer existe pero el rendimiento con dataset grande hace inusable la cámara.
- `PARCIAL` si cada update duplica excesivamente la nube en memoria.
- `NO CONSEGUIDA` si para probar se introduce generación densa funcional dentro de GUI.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos o porque una fase anterior debe corregirse antes de continuar.

## Riesgos

- GPU/driver con límites inferiores a los asumidos.
- Buffer demasiado grande para una sola asignación; puede requerir chunking técnico.
- Medir solo FPS sin observar bloqueos largos de upload.
- Acoplar el formato interno a un mensaje Fase 8 todavía no cerrado.

## Documentación a actualizar


Al ejecutar realmente la subfase, actualizar solo documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7L.md
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7L_RESUMEN.md
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
