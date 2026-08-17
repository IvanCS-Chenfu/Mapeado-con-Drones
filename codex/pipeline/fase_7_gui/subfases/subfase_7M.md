# Subfase 7M — Integración multi-dron, validación visual cruzada y cierre

## Estado

```text
sin hacer
```

## Dependencia

7A–7L conseguidas en sus alcances; Fases 3–6 funcionales y disponibles.

## Objetivo técnico

Integrar toda la GUI con un escenario multi-dron real y demostrar que sirve simultáneamente como herramienta de operación y de corroboración visual de Fases 3–6, sin convertirse en dependencia del pipeline. Clasificar/corregir cualquier defecto en la fase propietaria antes de cerrar.

## Comportamiento esperado

La prueba final debe mostrar sparse, drones, KFs, fiduciales y trayectorias reales; actualizar cards/progreso; permitir score/filter/picking; enviar GO_TO y CAPTURE_SPARSE; mantener CAPTURE_DENSE como capacidad preparada si Fase 8 aún no existe. Cerrar/reabrir GUI no afecta misión/mapa/control.

La GUI debe poder arrancar cuando el sistema ya está en marcha y reconstruir su vista a partir del estado/topics vigentes, dentro de lo que los QoS/contratos permitan.

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

Cada subfase validó un bloque aislado. Falta demostrar la composición con N drones, carga gráfica real, tareas y replanning. Además, esta es la puerta explícita para descubrir errores visuales de Fases 3–6 que pruebas de logs quizá no hicieron evidentes.

## Invariantes y decisiones cerradas

- No declarar Fase 7 conseguida con un mapa/pose/trayectoria visualmente incoherente sin determinar causa.
- La GUI no es autoridad ni requisito de ejecución.
- Fase 8 no bloquea el cierre salvo la preparación de layer/contrato dense.
- No usar RViz2 como requisito; puede usarse excepcionalmente como diagnóstico comparativo externo, pero la evidencia principal debe ser la GUI propia y los mensajes/logs.
- Cualquier vuelta a Fases 3–6 conserva el historial real de intentos y luego repite 7M.

## Archivos permitidos a modificar

```text
src/servidor/multidron_gui/
src/servidor/orbslam3_server/              # solo correcciones mínimas de integración/telemetría
src/servidor/orbslam3_msgs/                # si contratos GUI/captura lo requieren
src/dron/...                               # solo si se reabre Fase 5/6 por defecto demostrado
src/simulacion/simulacion_dron/launch/     # integración de arranque/prueba
src/simulacion/simulacion_dron/config/scenarios/fase_7/
codex/contexto/paquetes/
```

Las rutas nuevas son de contrato. Antes de crearlas, comprobar el árbol posterior a Fases 2–6. Si existe un componente equivalente, reutilizarlo en lugar de duplicarlo.

## Archivos prohibidos

```text
cambios algoritmos Fases 3–6 para “pasar” la GUI sin volver a su fase/contrato
ORB_SLAM3/
Fase 8 dense real
RViz2 como dependencia de aceptación
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
launch oficial multi-dron posterior a Fase 6
launch/ejecutable multidron_gui
markers/logs 7A–7L
productores sparse/KFs/fiducials/pose/task/trajectory
task submission GO_TO/CAPTURE_SPARSE
shutdown/reconnect de GUI
```

Localizar también herramientas de captura de pantalla/replay disponibles si la evidencia visual debe archivarse; no inventar resultados visuales en historial.

Los nombres de componentes nuevos definidos por este contrato pueden implementarse con una estructura equivalente si existe una razón técnica clara. Los nombres de **interfaces procedentes de fases anteriores** no se inventan: se localizan físicamente primero.

## Cambios requeridos

1. Crear un escenario reproducible de Fase 7 con al menos 2 drones y datos suficientes de mapa/fiducial/tareas.
2. Arrancar backend y GUI con una única receta documentada.
3. Validar cámara/grid y todas las layers baseline.
4. Validar score threshold y gradiente con sparse real.
5. Validar coherencia visual de KFs, fiduciales y poses contra los mensajes recibidos.
6. Ejecutar movimiento/replanning y comprobar trayectoria actual por dron.
7. Validar cards/progreso y scroll; si el escenario real no tiene suficientes drones, complementar solo la prueba de scroll con cards sintéticas, no la integración funcional.
8. Seleccionar MapPoints/KFs/drones/fiduciales implementados y verificar inspector.
9. Enviar `GO_TO` y observar task→trajectory→motion→result.
10. Ejecutar `CAPTURE_SPARSE` en un caso de éxito y uno sin KF.
11. Comprobar `CAPTURE_DENSE` no disponible antes de Fase 8 sin falso éxito.
12. Cerrar GUI durante una misión/trayectoria; confirmar continuidad del backend y después reabrir.
13. Clasificar toda anomalía `SOURCE_DATA` vs `GUI_RENDER`; si es origen, volver a fase propietaria y repetir.
14. Medir CPU/memoria/FPS de GUI de forma orientativa y descartar que bloquee threads críticos.
15. Actualizar documentación final e índice/estado solo después de evidencia real.

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
./codex/herramientas/build_selected_packages.sh multidron_gui orbslam3_server orbslam3_msgs dron_individual lib_tray orbslam3_ros2 orbslam3_multi simulacion_dron
```

Respetar la estrategia de build por grupos de Fase 2; el helper/comando exacto real puede dividirse en builds pequeños para paquetes pesados.

Si la separación de Fase 2 utiliza builds por grupo, usar el helper vigente para el grupo Servidor y, cuando haya pruebas de integración, los grupos Dron/Simulación correspondientes. Registrar el comando exacto solo en el historial real.

## Pruebas Gazebo requeridas

### Prueba 1 — Smoke completo

Dos drones, GUI abierta, sparse/KFs/poses/fiducials visibles. Mover cámara, toggles y score durante actividad real.

### Prueba 2 — Operación desde GUI

Enviar `GO_TO` y después `CAPTURE_SPARSE` a un dron. Comprobar card, progreso, trayectoria y resultado. Repetir un CAPTURE_SPARSE donde no aparezca KF y verificar fallo recuperable.

### Prueba 3 — Replanning/trajectory update

Provocar o aprovechar un replan real y comprobar sustitución de la curva mostrada sin perder la task card.

### Prueba 4 — Cierre/reapertura

Cerrar la GUI con backend activo. Esperar continuidad de misión/mapa/control, reabrir y comprobar reconstrucción de estado visual.

### Prueba 5 — Validación cruzada

Inspeccionar al menos un caso por productor: sparse/KF, fiducial, pose y tarea/trayectoria. Si un dato es incorrecto antes del renderer, no cerrar 7M hasta corregir la fase propietaria y repetir.

No arrancar Gazebo artificialmente para una prueba puramente gráfica/unitaria. Cuando se use simulación, el comando base es:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_7_7M \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El mecanismo exacto para arrancar `multidron_gui` junto a ese launch se fija en 7A/7B y debe reutilizarse después. La GUI nunca depende de que RViz2 esté abierto.

## Patrones de reducción de logs

```text
GUI-INTEGRATION|GUI-BOOT|GUI-SHUTDOWN|GUI-SPARSE|GUI-DRONE|GUI-KF|GUI-FIDUCIAL|GUI-TRAJECTORY|GUI-TASK|GUI-PICK|GUI-GO-TO|CAPTURE-SPARSE|TASK-|TRAJ-|F1F-|fiducial|tracking|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo alimentan reductores. Si falta evidencia, regenerar el reducido con patrones más precisos; no abrir el log completo directamente.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Todos los componentes 7A–7L funcionan juntos en una prueba multi-dron.
2. Viewport sigue siendo operable con carga real.
3. Sparse/poses/KFs/fiduciales/trayectorias son visualmente coherentes y respaldados por mensajes correctos.
4. Cards/progreso coinciden con Fase 6.
5. GO_TO y CAPTURE_SPARSE operan por TaskManager/planner/reservas.
6. CAPTURE_DENSE queda preparada sin implementación falsa.
7. Cerrar/reabrir GUI no afecta al pipeline.
8. No existe dependencia de RViz2 ni GT funcional.
9. Toda regresión de Fases 3–6 detectada ha sido corregida en su fase y revalidada.
10. Documentación/historial real refleja intentos, correcciones y conclusión agregada.

Además, todo build requerido debe devolver `0`, todas las pruebas obligatorias deben ejecutarse, no puede haber errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si la GUI funciona pero falta una layer/operación baseline por dependencia aún no disponible.
- `PARCIAL` si la operación es correcta pero rendimiento gráfico impide uso fluido con carga representativa.
- `BLOQUEADA` si una regresión de Fase 3–6 debe corregirse antes de poder evaluar Fase 7.
- No usar `PARCIAL` para ocultar un falso éxito de CAPTURE_SPARSE, una trayectoria incorrecta o una GUI que bloquea el backend.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos o porque una fase anterior debe corregirse antes de continuar.

## Riesgos

- Integración larga que oculte un error simple; ejecutar smoke por layers antes.
- Confundir error visual con error del productor o viceversa.
- Consumo GPU/CPU que compita con SLAM/servidor en el mismo equipo.
- Reapertura que dependa de topics no latched/transient y deje capas vacías; resolver en el contrato productor correcto si el estado debe ser recuperable.
- Cerrar la fase por estética sin validar acciones reales.

## Documentación a actualizar


Al ejecutar realmente la subfase, actualizar solo documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7M.md
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7M_RESUMEN.md
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
