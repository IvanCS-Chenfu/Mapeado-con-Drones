# Subfase 7B — Ventana principal y layout operativo

## Estado

```text
conseguida
```

## Dependencia

7A conseguida.

## Objetivo técnico

Construir la ventana principal con la distribución acordada: toolbar superior, gran zona central/izquierda para viewport 3D, columna derecha con tarjetas de drones y scroll, bloque inferior de creación de tareas e inspector de selección.

## Comportamiento esperado

La GUI debe poder abrirse sin datos y conservar una geometría usable al redimensionar. La lista de tarjetas es scrollable; el formulario de tareas y el inspector no deben quedar enterrados por una cantidad grande de drones. Todavía puede usarse un placeholder en el viewport, sin OpenGL real hasta 7D.

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

No existe layout Fase 7. El `gui_tray_multi.py` legacy de simulación no es base arquitectónica y no debe copiarse como solución final. El layout debe prepararse para contenido dinámico, no fijarse a dos drones.

## Invariantes y decisiones cerradas

- El viewport es el componente dominante de la ventana.
- Tarjetas dinámicas para N drones; scroll vertical cuando no caben.
- Formulario de tareas accesible debajo de tarjetas.
- No existen botones pause/resume/cancel en baseline.
- Toolbar reserva controles para sparse/dense/drones/KFs/trayectorias/fiduciales, score y color por score.
- Inspector de entidad seleccionado visible sin invadir el viewport.

## Archivos permitidos a modificar

```text
src/servidor/multidron_gui_lib/src/main_window.*
src/servidor/multidron_gui_lib/include/multidron_gui_lib/main_window.*
src/servidor/multidron_gui_lib/src/widgets/
src/servidor/multidron_gui_lib/include/multidron_gui_lib/widgets/
src/servidor/multidron_gui_lib/CMakeLists.txt
src/servidor/multidron_gui/                  # integración ejecutable/launch si hace falta
codex/contexto/paquetes/multidron_gui_lib/
codex/contexto/paquetes/multidron_gui/
```

Las rutas nuevas son de contrato. Antes de crearlas, comprobar el árbol posterior a Fases 2–6. Si existe un componente equivalente, reutilizarlo en lugar de duplicarlo.

## Archivos prohibidos

```text
src/simulacion/simulacion_dron/gui_tray_multi.py
src/servidor/orbslam3_server/src/global_map_server.cpp  # no hace falta para layout
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
MainWindow o clase equivalente creada en 7A
QMainWindow / central widget
contenedor toolbar
contenedor viewport
scroll area de cards
widget de creación de tareas
widget inspector
```

Si 7A fijó nombres distintos, usar esos nombres documentados.

Los nombres de componentes nuevos definidos por este contrato pueden implementarse con una estructura equivalente si existe una razón técnica clara. Los nombres de **interfaces procedentes de fases anteriores** no se inventan: se localizan físicamente primero.

## Cambios requeridos

1. Crear `MainWindow` con layout responsive y tamaños mínimos razonables.
2. Crear toolbar con controles visibles pero inicialmente desacoplados de datos: toggles de layers, control de score y modo de color.
3. Crear contenedor del viewport con placeholder reemplazable por `Scene3DWidget` en 7D.
4. Crear `DroneCardsPanel` o equivalente con `QScrollArea` y API para añadir/eliminar tarjetas dinámicamente.
5. Mantener `TaskCreationPanel` fuera del scroll de tarjetas si la estructura Qt lo permite de forma limpia.
6. Crear `SelectionInspector` vacío con campos tipo/posición/metadata preparados.
7. Comprobar tab order, resize y que ningún panel colapse a tamaño inutilizable.
8. Añadir un modo de datos sintéticos de test solo para crear 1, 2, 8, 20 tarjetas; no conservarlo como fuente funcional de runtime.

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

### Prueba 1 — Layout sin ROS data

Abrir la GUI a resolución normal, redimensionar grande/pequeña y comprobar que viewport, toolbar y panel derecho siguen accesibles.

### Prueba 2 — Escalabilidad de tarjetas

Inyectar tarjetas sintéticas en test: 1, 8 y 20. Confirmar scroll y que el formulario inferior permanece accesible.

### Prueba 3 — Integración de arranque

Abrir el backend real + GUI. Aunque todavía no se pinten datos, la ventana no debe bloquear el spin ROS ni el servidor.

No arrancar Gazebo artificialmente para una prueba puramente gráfica/unitaria. Cuando se use simulación, el comando base es:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_7_7B \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El mecanismo exacto para arrancar `multidron_gui` junto a ese launch se fija en 7A/7B y debe reutilizarse después. La GUI nunca depende de que RViz2 esté abierto.

## Patrones de reducción de logs

```text
GUI-LAYOUT|GUI-DRONE-PANEL|GUI-TASK-PANEL|GUI-INSPECTOR|GUI-BOOT|GUI-SHUTDOWN|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo alimentan reductores. Si falta evidencia, regenerar el reducido con patrones más precisos; no abrir el log completo directamente.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. La distribución acordada existe y resiste resize.
2. La lista de drones escala mediante scroll.
3. El formulario de tareas e inspector son accesibles con muchas tarjetas.
4. Los controles baseline están presentes sin ejecutar todavía lógica de mapa.
5. No se reutiliza la GUI legacy ni RViz2.

Además, todo build requerido debe devolver `0`, todas las pruebas obligatorias deben ejecutarse, no puede haber errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si el layout funciona pero el formulario queda dentro del scroll y se vuelve impráctico con muchos drones.
- `PARCIAL` si la ventana depende de recibir datos para construirse.
- `PARCIAL` si el resize rompe el viewport/panel principal.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos o porque una fase anterior debe corregirse antes de continuar.

## Riesgos

- Sobrecargar la toolbar con controles no acordados.
- Codificar posiciones absolutas de widgets que fallen con DPI/resolución distinta.
- Fijar el número de tarjetas a los drones de una prueba concreta.

## Documentación a actualizar


Al ejecutar realmente la subfase, actualizar solo documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7B.md
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7B_RESUMEN.md
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
