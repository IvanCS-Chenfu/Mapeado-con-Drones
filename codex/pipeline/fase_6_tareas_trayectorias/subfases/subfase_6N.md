# Subfase 6N — Cobertura adaptativa de `MAP_SECTION` mediante exploración y frontiers

## Estado

```text
sin hacer
```

## Dependencia

6G, 6K, 6L y 6M; utiliza las tareas generadas en 6C.

## Objetivo técnico

Implementar el significado real de una `MAP_SECTION`: descubrir y observar todo lo accesible desde su entrada hasta su salida nominal, adaptándose a paredes, habitaciones, pasillos, geometría en L y laberintos, con solape deliberado entre tareas vecinas.

## Comportamiento esperado

`MAP_SECTION_B = A-B-C` es una semilla de cobertura, no una polilínea. El dron parte de la entrada elegida por 6F, avanza hacia la región de B y la salida opuesta, pero puede separarse del borde nominal para seguir la geometría y explorar ramificaciones accesibles.

La primera arquitectura debe inspirarse en exploración por **known free / occupied / unknown**, frontiers y next-best-view, sin obligar a usar una librería concreta. El dron crea subobjetivos internos; **no** crea nuevas tareas globales por cada pasillo.

Caso esperado:

```text
trayectoria nominal: A -------- B -------- C
pared real:          A ----+
                           |-----+
                                 |---- C
ramificación/pasillo:            +------ ...
```

La tarea debe explorar la geometría accesible relevante y luego continuar, no intentar atravesar la línea nominal ni ignorar la ramificación solo porque no coincide con el ROI.

Las tareas adyacentes repiten intencionadamente su tramo común. `Task B: A-B-C` y `Task C: B-C-D` deben observar B-C de nuevo aunque exista información previa, para reforzar solape entre submapas.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6_RESUMEN.md
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6.md
```

Antes de modificar código, Codex debe leer también el MD vigente de cada paquete afectado en `codex/contexto/paquetes/` y el contrato final de Fase 5 que exista en el workspace real. El snapshot usado para preparar esta Fase 6 no contiene la Fase 5 recién ejecutada por el usuario, por lo que no se deben inventar nombres nuevos de topics, frames o mensajes si Fase 5 ya proporciona un contrato equivalente.



## Diagnóstico de partida

La definición inicial “mapear una esquina” no basta para un pabellón con paredes interiores o un laberinto. Seguir A-B-C literalmente puede llegar al destino dejando habitaciones/pasillos sin observar. Tampoco existe un mapa denso global previo que pueda usarse como plano conocido.

## Invariantes y decisiones cerradas

- Todo entorno accesible relevante del ROI debe poder descubrirse sin cambiar entre algoritmos interior/exterior.
- Áreas físicamente inaccesibles no bloquean indefinidamente la tarea/misión.
- El ROI es objetivo de cobertura, no obstáculo ni ruta.
- Una frontier fuera del ROI+histeresis no se persigue como objetivo, aunque los sensores/ORB puedan observarla.
- El solape nominal entre tareas vecinas es obligatorio y no se deduplica por “ya visto”.
- La tarea conserva su `task_id` aunque genere subobjetivos/frontiers y muchas trayectorias.
- La cobertura se apoya en 6K para geometría y 6L/6M para calidad visual; no usa GT.

## Archivos permitidos a modificar

```text
src/dron/dron_individual/src/control_tray/  # CoveragePlanner/LocalPlanner
src/dron/dron_individual/include/
src/dron/dron_individual/config/
src/dron/dron_individual/test/
src/servidor/orbslam3_server/src/  # solo estado global ligero de cobertura si el diseño acordado lo requiere
src/servidor/orbslam3_msgs/
src/dron/orbslam3_msgs/
codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/orbslam3_server/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
ORB_SLAM3/
src/servidor/orbslam3_multi/  # no convertir RawMap/landmarks en occupancy planner
código de nube densa global Fase 8
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
TaskExecutor 6G
representación local 6K
ViewPlanner 6M
ROI/histeresis y A/B/C de la tarea
pose global/local de Fase 5
posible información de cobertura compartida ya existente; no inventarla si no está
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Definir una representación de exploración local suficiente para distinguir libre/ocupado/desconocido y extraer frontiers/subobjetivos alcanzables.
2. Crear un `CoveragePlanner` que use A/B/C como orientación/semilla de progreso, no como obligación geométrica.
3. Seleccionar next-best-view/subobjetivo con restricciones: dentro del objetivo ROI+histeresis, alcanzable, seguro, tracking sostenible y ganancia de información.
4. Permitir explorar ramificaciones/pasillos descubiertos durante la tarea y volver después al progreso A→B→C; registrar estos subobjetivos dentro del mismo `task_id`.
5. Definir cuándo una frontier se marca resuelta, bloqueada/inaccesible o fuera de alcance; un obstáculo cerrado no debe provocar búsqueda infinita.
6. Definir criterio de finalización de tarea: se ha cubierto el recorrido nominal desde entrada a salida y no queda una frontier relevante/alcanzable asociada a esa cobertura que deba resolverse antes de completar. Si una frontier reachable del ROI se descubre y queda sin dueño, no permitir que todas las tareas se declaren completas; asignarla como subobjetivo a una tarea activa/adecuada sin crear top-level task nueva.
7. Conservar el tramo de solape nominal aunque la información global diga `known`; marcarlo como `required_overlap` para que otra tarea lo observe desde su propia ejecución.
8. Añadir logs `COVERAGE-FRONTIER`, `COVERAGE-SUBGOAL`, `COVERAGE-OVERLAP`, `COVERAGE-COMPLETE/BLOCKED`.

## Cambios prohibidos

- No completar una tarea solo por alcanzar C.
- No particionar el ROI en territorios rígidos que los drones no puedan cruzar.
- No evitar todo solape para maximizar eficiencia.
- No seguir frontiers indefinidamente fuera del ROI/histeresis.
- No usar la nube sparse como único mapa de espacio libre/ocupado.
- No usar Ground Truth como entrada funcional para pose, asignación, navegación, obstáculos, autorización de trayectorias, cobertura o criterio de finalización.
- No modificar `ORB_SLAM3` como primera opción; ampliar el wrapper o reutilizar interfaces existentes siempre que sea suficiente.
- No implementar la nube densa global, TSDF/Open3D global ni la reconstrucción final de Fase 8.
- No convertir el servidor en planificador de paredes u obstáculos físicos: esa responsabilidad pertenece al dron.
- No borrar ni reescribir datos raw de ORB-SLAM3 para adaptar la navegación.
- No introducir un modo separado de navegación interior y otro exterior.
- No rellenar historiales con resultados ficticios: se crean únicamente cuando exista una ejecución real.

## Paquetes a compilar

Comando base esperado:

```bash
./codex/herramientas/build_selected_packages.sh dron_individual orbslam3_server orbslam3_msgs
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6N   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Pared no alineada/L

En un escenario donde la superficie real se desvíe del borde nominal, `MAP_SECTION` debe adaptar el recorrido y cubrir la geometría sin intentar atravesar la pared ni exigir una línea A-B-C.

### Prueba 2 — Ramificación/pasillo

Crear/usar una sección con un pasillo o espacio lateral accesible. La tarea debe generar un subobjetivo de exploración y no marcarse completa al llegar a C mientras quede esa frontier relevante.

### Prueba 3 — Solape de dos tareas vecinas

Ejecutar `Task B` y `Task C` con drones/submapas distintos y verificar que ambos observan deliberadamente el tramo B-C, sin que el segundo lo omita por aparecer como conocido.

### Prueba 4 — Zona inaccesible

Una región dentro del ROI pero cerrada/no alcanzable no debe producir un loop infinito. Debe quedar clasificada como no alcanzable con evidencia del planner y permitir cierre cuando el resto esté cubierto.

## Patrones de reducción de logs

```text
COVERAGE-FRONTIER|COVERAGE-SUBGOAL|COVERAGE-OVERLAP|COVERAGE-COMPLETE|COVERAGE-BLOCKED|task_id|frontier|unreachable|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Una `MAP_SECTION` se adapta a geometría no rectangular y explora ramificaciones accesibles.
2. Llegar a C por sí solo no completa la tarea si queda cobertura relevante pendiente.
3. Una región inaccesible no genera exploración infinita.
4. Tareas vecinas conservan su solape requerido.
5. No existe partición rígida interior/exterior ni dependencia de GT.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si sigue paredes en L pero no explora ramificaciones.
- `PARCIAL` si explora correctamente pero puede finalizar dejando una frontier relevante reachable sin asignar.
- `PARCIAL` si elimina el solape por optimización global de cobertura.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Frontiers espurias por ruido depth que causen exploración infinita.
- Una definición demasiado local que deje un pasillo accesible sin asignar a ninguna tarea.
- Perseguir cada hueco pequeño y degradar enormemente tiempo de misión.
- Solape tan agresivo que varios drones repitan todo el mapa en vez del tramo común deseado.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6N.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6N_RESUMEN.md
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md              # si cambia el estado real
codex/contexto/paquetes/<paquete_afectado>/
```

Los historiales anteriores **no existen en este ZIP a propósito**. Deben crearse solo tras una ejecución real. La documentación de paquete debe reflejar el estado actual del código, no limitarse a añadir una nota histórica.

## Dudas funcionales de contrato

```text
ninguna
```

Cualquier duda nueva que cambie el comportamiento acordado suspende la autorización funcional y debe discutirse antes de continuar, conforme a `AGENTS.md`.
