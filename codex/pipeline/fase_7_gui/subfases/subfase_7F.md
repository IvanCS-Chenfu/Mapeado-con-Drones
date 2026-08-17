# Subfase 7F — Capas de drones, KeyFrames y fiduciales

## Estado

```text
sin hacer
```

## Dependencia

7D conseguida; productores válidos de Fases 3, 4 y 5.

## Objetivo técnico

Añadir al viewport las capas 3D de poses estimadas de drones, KeyFrames y fiduciales usando exclusivamente los contratos canónicos posteriores a Fases 3–5. Cada layer debe poder activarse/desactivarse desde la toolbar.

## Comportamiento esperado

Los drones se representan en su pose global estimada vigente y con orientación visible. Los KFs se representan mediante frustums/ejes u otra geometría ligera en su pose global. Los fiduciales se muestran con pose/ID cuando el backend la conoce. Ninguna capa consulta GT.

La GUI debe soportar N drones/submapas sin estar codificada a `drone_0`/`drone_1`.

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

El snapshot tiene `/global_keyframes` orientado a RViz y Fase 5 todavía era documental. Tras ejecutar Fases 4/5 pueden existir nuevas interfaces más adecuadas. Debe localizarse la fuente real. También pueden aparecer estados degradados de pose que no tienen una representación visual acordada; si ocurre, preguntar al usuario.

## Invariantes y decisiones cerradas

- Pose visual = pose estimada canónica sin GT.
- Identidad de submapa sigue `(drone_id,map_epoch)`.
- KFs se colocan según pose global aceptada, no raw local si ya existe globalización.
- Fiduciales son observaciones/anchors absolutos de Fase 4; no loops.
- Toggle de una layer no cambia su productor.
- No decidir automáticamente cómo dibujar un dron perdido/stale; consultar si aparece el caso.

## Archivos permitidos a modificar

```text
src/servidor/multidron_gui/src/render/drone_pose_layer.*
src/servidor/multidron_gui/src/render/keyframe_layer.*
src/servidor/multidron_gui/src/render/fiducial_layer.*
src/servidor/multidron_gui/src/gui_data_model.*
src/servidor/multidron_gui/src/ros_data_bridge.*
src/servidor/orbslam3_server/              # solo si falta telemetría canónica y se acuerda
src/dron/...                               # solo mediante retorno a fase propietaria si falta pose/estado
```

Las rutas nuevas son de contrato. Antes de crearlas, comprobar el árbol posterior a Fases 2–6. Si existe un componente equivalente, reutilizarlo en lugar de duplicarlo.

## Archivos prohibidos

```text
sensor/GT/* como fuente visual funcional
ORB_SLAM3/
algoritmos de anchor/pose graph
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
pose global estimada y tracking de Fase 5
/global_keyframes o interfaz estructurada equivalente
FiducialObservation/estado de fiduciales/anchors Fase 4
(drone_id,map_epoch,local_id) de KFs
DronePoseLayer
KeyFrameLayer
FiducialLayer
```

Los nombres de componentes nuevos definidos por este contrato pueden implementarse con una estructura equivalente si existe una razón técnica clara. Los nombres de **interfaces procedentes de fases anteriores** no se inventan: se localizan físicamente primero.

## Cambios requeridos

1. Localizar la pose global estimada vigente y sus estados de validez/tracking.
2. Implementar `DronePoseLayer` con geometría/orientación y etiqueta/ID ligera.
3. Localizar/decodificar KFs globales; implementar frustum/ejes sin depender de RViz.
4. Mantener metadata de submapa/KF para futuro picking.
5. Localizar fiduciales/anchors y dibujar pose/ID coherentes en `world`.
6. Añadir toggles independientes `Drones`, `KeyFrames`, `Fiducials`.
7. Probar aparición/desaparición dinámica de drones/submapas sin recrear toda la ventana.
8. Añadir markers `GUI-DRONE-POSE`, `GUI-KF-UPDATE`, `GUI-FIDUCIAL-UPDATE` con counts/IDs agregados.

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

Añadir `orbslam3_server` u otro productor solo si 7F modifica telemetría después de aplicar la puerta de fase anterior.

Si la separación de Fase 2 utiliza builds por grupo, usar el helper vigente para el grupo Servidor y, cuando haya pruebas de integración, los grupos Dron/Simulación correspondientes. Registrar el comando exacto solo en el historial real.

## Pruebas Gazebo requeridas

### Prueba 1 — Dos drones con pose estimada

Mover dos drones usando el sistema posterior a Fase 5 y comprobar que sus representaciones siguen la pose estimada sin GT.

### Prueba 2 — KFs de varios submapas

Durante mapeo, comprobar que nuevos KFs aparecen y quedan alineados con el sparse. Activar/desactivar la layer sin afectar mapa.

### Prueba 3 — Fiduciales

Pasar por un fiducial real/simulado visual de Fase 4 y comprobar pose/ID. Si se ve desplazado, comparar el dato recibido antes de modificar el renderer.

### Prueba 4 — Estado degradado encontrado

Si durante pruebas aparece un dron perdido/stale y el comportamiento visual no está acordado, detener la subfase y preguntar al usuario; esta situación no se resuelve inventando un estilo.

No arrancar Gazebo artificialmente para una prueba puramente gráfica/unitaria. Cuando se use simulación, el comando base es:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_7_7F \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El mecanismo exacto para arrancar `multidron_gui` junto a ese launch se fija en 7A/7B y debe reutilizarse después. La GUI nunca depende de que RViz2 esté abierto.

## Patrones de reducción de logs

```text
GUI-DRONE-POSE|GUI-KF-UPDATE|GUI-FIDUCIAL-UPDATE|tracking|map_epoch|global_keyframes|fiducial|pose|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo alimentan reductores. Si falta evidencia, regenerar el reducido con patrones más precisos; no abrir el log completo directamente.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Drones/KFs/fiduciales se renderizan en posiciones/orientaciones coherentes desde fuentes reales.
2. Todos los toggles funcionan localmente.
3. N drones/submapas se gestionan dinámicamente.
4. No existe suscripción funcional a GT.
5. Si la GUI descubre un defecto real de Fase 3/4/5, se corrige allí y se revalida.

Además, todo build requerido debe devolver `0`, todas las pruebas obligatorias deben ejecutarse, no puede haber errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si una de las tres capas carece de un contrato de datos suficiente.
- `PARCIAL` si la pose funciona solo con un número fijo de drones.
- `BLOQUEADA` si la pose global/canonical state de Fase 5 no está disponible o está demostrablemente mal y requiere volver a Fase 5.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos o porque una fase anterior debe corregirse antes de continuar.

## Riesgos

- Mezclar frames local/world.
- Interpretar `MarkerArray` RViz de forma frágil si existe un mensaje estructurado mejor.
- Dibujar último estado stale como válido sin semántica acordada.
- Introducir estilos que oculten desalineaciones reales.

## Documentación a actualizar


Al ejecutar realmente la subfase, actualizar solo documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7F.md
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7F_RESUMEN.md
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
