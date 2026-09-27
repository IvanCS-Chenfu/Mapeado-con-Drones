# Pruebas del Capítulo 8 — instrucciones para Codex

## 1. Objetivo general

Este documento define las pruebas experimentales que deben ejecutarse para cerrar el **Capítulo 8 — Planificación y movimiento autónomo** del TFG.

El objetivo del capítulo no es volver a validar:

- el controlador básico del Capítulo 4;
- ORB-SLAM3 y el mapa sparse del Capítulo 5;
- los cierres de bucle y optimizaciones del Capítulo 6;
- ni la localización funcional con ORB del Capítulo 7.

Aquí se quiere demostrar específicamente el funcionamiento de la capa de autonomía construida sobre esos bloques:

1. generación de la geometría de misión y de las subROIs;
2. asignación de tareas a varios drones;
3. construcción incremental del mapa voxel;
4. incorporación de evidencia sparse procedente de KFs y MapPoints;
5. incorporación de profundidad bajo demanda;
6. evolución de la cobertura de fachada;
7. planificación sobre espacio FREE;
8. tratamiento de UNKNOWN sin atravesarlo;
9. uso de `VIEW_UNKNOWN`, `VIEW_WALL` y `VIEW_ADVANCE`;
10. coordinación espacial mediante vóxeles `RESERVED`;
11. ejecución integrada de la autonomía con dos drones.

Las pruebas a preparar son:

- **8.2-A — generación de subROIs en la GUI**.
- **8.2-C — asignación multidrón de tareas**.
- **8.2-B — evolución de la cobertura**, obtenida principalmente a partir de 8.4-B y, si resulta útil, de 8.5.6.
- **8.3 — construcción del mapa voxel sin depth**.
- **8.4-A — resolución de una región UNKNOWN mediante `LOOK_AND_CAPTURE` / `VIEW_UNKNOWN`**.
- **8.4-B — observación frontal de fachada mediante `VIEW_WALL` o `VIEW_ADVANCE` y actualización de coverage**.
- **8.5.6 — ejecución autónoma integrada con dos drones**.

La subsección **8.5.7 no es una prueba adicional**. Debe redactarse después como síntesis funcional y de limitaciones a partir de las pruebas anteriores.

---

## 2. Reglas generales

### 2.1. Revisar primero el estado actual del repositorio

Antes de preparar escenarios o modificar código:

1. trabajar sobre el estado actual de `main`;
2. revisar especialmente:
   - `servidor/task_server`;
   - `servidor/task_lib`;
   - `servidor/multidron_gui`;
   - `servidor/multidron_gui_lib`;
   - `dron/task_manager`;
   - `dron/orbslam3_ros2`;
   - `servidor/mission_msgs` y `dron/mission_msgs`;
   - configuraciones y escenarios de simulación;
3. reutilizar topics, mensajes, logs y scripts existentes;
4. no duplicar telemetría si ya existe;
5. no reintroducir lógica legacy retirada del runtime.

Archivos especialmente relevantes:

```text
servidor/task_server/src/task_server_node.cpp
servidor/task_server/src/evidence_pipeline.cpp
servidor/task_lib/src/voxel_map.cpp
servidor/task_lib/src/facade_coverage.cpp
servidor/task_lib/src/dstar_lite.cpp
servidor/task_lib/src/reservation_overlay.cpp
servidor/task_server/config/mission_house.yaml
servidor/task_server/config/mission_house_single_drone.yaml
simulacion/simulacion_dron/config/scenarios/autonomous_gt_fiducial2.yaml
simulacion/simulacion_dron/config/mission_profiles/autonomous_gt.yaml
```

### 2.2. No cambiar el comportamiento para fabricar resultados

No modificar arbitrariamente:

- umbrales de cobertura;
- criterios de FREE/OCCUPIED;
- criterios de aceptación depth;
- pesos o lógica de D* Lite;
- reglas de reservas;
- selección de tareas;
- lógica de `VIEW_UNKNOWN`, `VIEW_WALL` o `VIEW_ADVANCE`;
- reglas de materialización;
- política de navegación estricta por FREE;

sólo para hacer que una prueba produzca una figura concreta.

Sí se permite:

- añadir instrumentación;
- crear scripts de captura y análisis;
- añadir escenarios YAML de prueba;
- activar/desactivar temporalmente mecanismos que interfieran con una prueba aislada, dejando constancia;
- utilizar GT como fuente de navegación cuando la prueba no pretende validar ORB;
- añadir parámetros debug desactivados por defecto;
- corregir bugs reales detectados durante las pruebas.

### 2.3. Uso de GT

En estas pruebas se puede utilizar GT como fuente funcional de navegación cuando el objetivo experimental sea aislar la lógica del Capítulo 8.

Esto es especialmente apropiado para:

- llevar drones hasta fiduciales;
- posicionarlos en puntos concretos;
- ejecutar desplazamientos controlados durante pruebas específicas;
- evitar mezclar errores de localización del Capítulo 7 con la validación de planificación/autonomía.

La percepción ORB, los KFs, MapPoints, fiduciales y backend deben seguir funcionando normalmente cuando sean necesarios para la prueba.

### 2.4. Trazabilidad mínima

Cada ejecución debe conservar como mínimo:

```text
commit.txt
config/
raw/
processed/
figures/
summary.md
```

`commit.txt` debe incluir:

- SHA;
- rama;
- fecha/hora;
- cambios locales.

`config/` debe incluir los YAML utilizados.

`raw/` debe contener:

- logs;
- rosbag o CSV de topics relevantes;
- snapshots brutos cuando proceda.

`processed/` debe contener:

- tablas;
- CSV procesados;
- métricas.

`figures/` debe contener:

- capturas;
- gráficas;
- vistas 3D o cortes 2D usados en la memoria.

---

## 3. Organización de resultados

Crear una estructura similar a:

```text
Pruebas/
└── Capítulo 8/
    ├── 8_2_A_subrois_gui/
    ├── 8_2_C_asignacion_multidron/
    ├── 8_2_B_coverage/
    ├── 8_3_voxel_sparse/
    ├── 8_4_A_view_unknown/
    ├── 8_4_B_view_wall_coverage/
    ├── 8_5_6_autonomia_integrada/
    ├── scripts/
    └── resumen_resultados.md
```

Si el repositorio ya utiliza otra convención equivalente, mantenerla.

---

## 4. Prueba 8.2-A — generación de subROIs

### 4.1. Objetivo

Demostrar que la configuración de misión genera correctamente las subROIs laterales usadas por el sistema.

La prueba es principalmente visual.

No es necesario:

- mover drones;
- habilitar autonomía;
- ejecutar D* Lite;
- capturar depth.

### 4.2. Procedimiento

1. lanzar servidor con la misión de referencia;
2. lanzar la GUI;
3. comprobar que `/mission/geometry` contiene la ROI, niveles y regiones;
4. mostrar en la GUI las regiones del nivel que se vaya a documentar;
5. mostrar claramente:
   - ROI;
   - AB;
   - BC;
   - CD;
   - DA.

La configuración principal actual contiene una ROI de referencia y `level_height: 2.0`. No hardcodear valores en scripts si ya pueden leerse del YAML.

### 4.3. Capturas

Preparar al menos una captura limpia en planta donde se vea:

- ROI;
- AB;
- BC;
- CD;
- DA;
- nivel seleccionado;
- etiquetas de regiones.

Evitar una captura saturada con varios niveles superpuestos si dificulta la interpretación.

El usuario tomará la captura definitiva para la memoria.

### 4.4. Verificaciones automáticas

Guardar un pequeño resumen con:

```text
numero_niveles
numero_subrois_total
subrois_por_nivel
ids_de_region
bounds_de_cada_region
```

Comprobar que existen cuatro subROIs por nivel.

### 4.5. Criterio de éxito

La prueba es válida si:

- la geometría se carga sin error;
- aparecen las cuatro subROIs laterales por nivel;
- coinciden con la configuración principal;
- la GUI permite mostrarlas de forma individual o agrupada por nivel.

---

## 5. Prueba 8.2-C — asignación multidrón

### 5.1. Objetivo

Demostrar que:

1. dos drones se registran;
2. ambos obtienen pose global válida;
3. pasan a ser elegibles;
4. el servidor asigna una tarea `MAP_SECTION` a cada uno;
5. la GUI muestra la tarea real de cada dron;
6. al seleccionar/clicar la tarjeta del dron se puede identificar visualmente su subROI.

### 5.2. Procedimiento

Usar dos drones.

Llevar ambos con GT hasta un fiducial para que dispongan de anclaje global.

Secuencia mínima:

```text
D1 -> fiducial
D2 -> fiducial
```

Después permitir que el servidor realice la asignación.

Si resulta sencillo y estable, después del anclaje dejar a los dos drones en posiciones ligeramente distintas antes de habilitar la asignación. Esto mejora la evidencia de que la selección responde a proximidad. Si complica innecesariamente la prueba, mantenerlos en el mismo entorno del fiducial.

### 5.3. Evidencia a registrar

Guardar:

- registro de D1 y D2;
- pose global válida;
- instante en que pasan a ser elegibles;
- tarea seleccionada para cada dron;
- `task_id`;
- `region_id`;
- `assigned_drone_id`;
- estado de tarea;
- distancia usada para la elección si aparece en logs.

Aprovechar eventos existentes como:

```text
[F6B-ASSIGNED]
TASK_ASSIGNED
```

y `/mission/task_states`.

### 5.4. Evidencia GUI

Preparar una captura con:

- tarjetas de D1 y D2;
- tarea mostrada en cada tarjeta;
- una tarjeta seleccionada;
- subROI correspondiente visible/resaltada.

Si resulta útil, realizar una segunda captura seleccionando el otro dron.

### 5.5. Criterio de éxito

La prueba es válida si:

- ambos drones están registrados;
- ambos tienen pose global válida;
- cada uno recibe una tarea coherente;
- las asignaciones aparecen en GUI;
- la tarjeta permite relacionar claramente dron y subROI.

No es obligatorio provocar `TO_FINISH` en esta prueba.

---

## 6. Prueba 8.2-B — evolución de la cobertura

### 6.1. Objetivo

Demostrar que la cobertura de una subROI no se incrementa simplemente porque el dron se acerque o recorra una zona, sino cuando se materializa evidencia depth válida asociada a la sección correspondiente.

Esta prueba no necesita necesariamente una ejecución separada.

La principal evidencia se obtendrá de:

```text
8.4-B
```

y, si resulta útil:

```text
8.5.6
```

### 6.2. Evidencia requerida

Con una tarea seleccionada en GUI:

1. captura inicial:
   - secciones pendientes;
   - `progress` inicial;
2. ejecutar una observación frontal válida;
3. esperar a que la evidencia se materialice;
4. captura posterior:
   - nuevas secciones activas;
   - `progress` actualizado.

Idealmente conservar varios estados intermedios si la ejecución integrada activa secciones progresivamente.

### 6.3. Datos

Registrar:

```text
time
task_id
region_id
progress
progress_known
active_sections
total_sections
task_state
source_id
coverage_claims
```

Aprovechar logs como:

```text
[F6H-COVERAGE-SOURCE-PENDING]
[F6H-COVERAGE-CLAIMS]
```

### 6.4. Criterio de éxito

La cobertura sólo debe cambiar después de materializar una fuente válida que reclame una o varias secciones.

No presentar como coverage:

- movimiento puro;
- evidencia sparse;
- un `VIEW_UNKNOWN`;
- UNKNOWN convertido únicamente en FREE.

---

## 7. Prueba 8.3 — construcción del mapa voxel sin depth

### 7.1. Objetivo

Demostrar qué mapa voxel puede construirse utilizando:

- KFs;
- MapPoints;
- rayos sparse;
- pose de los KFs;
- volumen atravesado por el dron;

sin incorporar todavía depth.

La prueba debe mostrar el crecimiento incremental del mapa antes y después del anclaje global.

### 7.2. Condiciones

Desactivar para esta prueba la incorporación de depth.

No ejecutar `LOOK_AND_CAPTURE`, `MOVE_AND_CAPTURE` con depth ni cualquier otra captura densa que contamine la prueba.

### 7.3. Trayectoria

Usar un solo dron.

#### Tramo 1

Mover desde la posición inicial hasta el fiducial utilizado para el anclaje usando GT.

Objetivo:

- generar KFs;
- generar MapPoints;
- disponer de evidencia sparse previa al anclaje.

#### Tramo 2

Esperar al anclaje.

Comprobar cómo la evidencia asociada a KFs y MapPoints puede materializarse en `W`.

#### Tramo 3

Continuar con GT hasta:

```text
(10, -10, 1, 90°)
```

para generar nuevos KFs y nueva evidencia.

### 7.4. Interpretación correcta de las fuentes

No afirmar que “cada KF genera FREE y OCCUPIED”.

La implementación separa varias fuentes:

#### MapPoints

Los MapPoints con score suficiente pueden contribuir a:

```text
OCCUPIED
```

#### Rayos sparse desde KFs

Desde el origen del KF hacia MapPoints cualificados se generan fuentes como:

```text
sparse_ray_free
ransac_free
```

que materializan:

```text
FREE
```

a lo largo de los rayos.

#### Pose del KF

La posición del propio KF puede respaldar un pequeño volumen FREE.

#### Movimiento físico

La región ocupada físicamente por el dron durante el movimiento añade evidencia FREE asociada al KF de referencia.

La prueba debe distinguir estas fuentes cuando sea posible.

### 7.5. Eventos útiles

Registrar, entre otros:

```text
[F6E-KF-EVIDENCE-WRITTEN]
[F6G-VOXEL-DELTAS-APPLIED]
[F6I-FREE-KF]
[F6I-FREE-VOXEL]
```

### 7.6. Snapshots

Obtener al menos tres snapshots:

- **A:** antes del anclaje;
- **B:** inmediatamente después del anclaje y de que se materialice la evidencia histórica;
- **C:** después de llegar a `(10, -10, 1, 90°)`.

### 7.7. Métricas

Para cada snapshot guardar:

```text
map_revision
num_FREE
num_OCCUPIED
num_sources
num_keyframes_con_evidencia
```

Si UNKNOWN no se publica explícitamente como celdas materializadas, no inventar un conteo total de UNKNOWN.

### 7.8. Figuras

Preparar:

1. vista 3D del mapa voxel;
2. si resulta útil, corte 2D;
3. comparación visual:
   - antes de anclar;
   - después de anclar;
   - después del movimiento adicional.

### 7.9. Criterio de éxito

La prueba es válida si:

- no se utiliza depth;
- al anclarse aparecen/materializan fuentes sparse en W;
- se distinguen FREE y OCCUPIED;
- los nuevos KFs generados después del anclaje siguen añadiendo evidencia;
- el mapa aumenta de forma incremental.

---

## 8. Prueba 8.4-A — `VIEW_UNKNOWN`

### 8.1. Objetivo

Demostrar el comportamiento de una captura dirigida a una región UNKNOWN.

Esta es la prueba que en el texto actual del capítulo corresponde a:

```text
Prueba 8.4-A — resolución de una región UNKNOWN
```

### 8.2. Terminología

El comando ejecutado es:

```text
LOOK_AND_CAPTURE
```

El servidor interpreta esa observación como:

```text
VIEW_UNKNOWN
```

No utilizar en documentación un comando inexistente llamado `LOOK_UNKNOWN`.

### 8.3. Condición especial

Desactivar temporalmente `TRACKING_RISK` para esta prueba si es necesario.

Motivo: el dron se orientará deliberadamente hacia una región con baja información visual y no interesa que el mecanismo preventivo del Capítulo 7 aborte la maniobra.

La desactivación debe estar limitada al escenario de prueba y quedar documentada.

### 8.4. Secuencia

1. usar un dron;
2. llevarlo con GT hasta el fiducial;
3. esperar al anclaje;
4. situarlo en una posición estable;
5. ordenar un giro de aproximadamente 90° a la derecha;
6. comprobar que la dirección observada contiene una zona UNKNOWN útil;
7. ejecutar `LOOK_AND_CAPTURE`;
8. esperar a que la observación depth se reciba, acepte, escriba y materialice;
9. comparar mapa antes/después.

### 8.5. Comportamiento esperado

Para `VIEW_UNKNOWN`:

- se generan rayos FREE;
- no se debe crear `OCCUPIED` frontal por el mero hecho de mirar;
- no se debe activar coverage;
- las regiones no observadas deben continuar UNKNOWN.

### 8.6. Datos a registrar

Guardar:

```text
command_id
workflow_id
command_type
inspection_kind
tracking_frame_id
local_keyframe_id
confidence
normal_valid
normal_support
normal_confidence
source_revision
map_revision_before
map_revision_after
free_voxels_before
free_voxels_after
occupied_voxels_before
occupied_voxels_after
```

Aprovechar:

```text
[F6F-DEPTH-SOURCES-WRITTEN]
[F6F-DEPTH-SOURCES-APPLIED]
[F6G-VOXEL-DELTAS-APPLIED]
```

### 8.7. Capturas

Preparar:

- mapa antes;
- mapa después;
- imagen/depth usada si resulta legible;
- corte del corredor observado si ayuda.

### 8.8. Criterio de éxito

La prueba es válida si:

- `LOOK_AND_CAPTURE` se ejecuta;
- se clasifica como `VIEW_UNKNOWN`;
- la observación supera los filtros;
- aparecen nuevos vóxeles FREE;
- no aparece coverage por esta observación;
- no se fabrican endpoints OCCUPIED de fachada;
- las zonas no observadas siguen UNKNOWN.

---

## 9. Prueba 8.4-B — observación frontal de fachada y coverage

### 9.1. Objetivo

Demostrar que una captura frontal válida puede generar:

```text
FREE
DIRECT_FREE
OCCUPIED
```

y que los `OCCUPIED` directos asociados a la fachada pueden activar secciones U y aumentar el porcentaje de coverage.

Esta misma ejecución debe alimentar la evidencia principal de la prueba 8.2-B.

### 9.2. No usar `LOOK_AND_CAPTURE` como captura de fachada

No ejecutar simplemente:

```text
LOOK_AND_CAPTURE
```

esperando que aumente coverage.

En el flujo actual, `LOOK_AND_CAPTURE` se interpreta como `VIEW_UNKNOWN` y no debe activar coverage.

Para obtener evidencia frontal utilizable por la cobertura, la observación debe formar parte del flujo que el servidor clasifica como:

```text
VIEW_WALL
```

o:

```text
VIEW_ADVANCE
```

mediante un comando `MOVE_AND_CAPTURE` y conservando el contexto de tarea, workflow, candidato y sección U.

### 9.3. Posición inicial propuesta

Usar un dron.

1. anclar el dron;
2. posicionarlo aproximadamente en:

```text
(4, -10, 1, 90°)
```

3. disponer de una tarea/subROI seleccionada cuyo candidato de fachada corresponda a la zona observada;
4. ejecutar el flujo frontal normal.

Si esa pose concreta no ofrece una observación frontal válida por la geometría real del escenario, ajustar mínimamente la posición y documentarlo.

### 9.4. Flujo requerido

El flujo debe ser equivalente a:

```text
tarea asignada
  ->
sección pendiente seleccionada
  ->
candidato de observación
  ->
MOVE_AND_CAPTURE
  ->
VIEW_WALL o VIEW_ADVANCE
  ->
depth válido
  ->
fuentes escritas
  ->
fuentes materializadas
  ->
claims de coverage
  ->
progress actualizado
```

### 9.5. Evidencia depth

Registrar:

- `confidence`;
- soporte;
- KF de referencia;
- normal;
- `normal_confidence`;
- puntos aceptados;
- fuentes:
  - `depth_free`;
  - `depth_direct_free`;
  - `depth_occupied`.

### 9.6. Evidencia coverage

Registrar:

```text
task_id
section_index
source_id
kind
claims
active_sections_before
active_sections_after
progress_before
progress_after
```

Aprovechar:

```text
[F6H-COVERAGE-SOURCE-PENDING]
[F6H-COVERAGE-CLAIMS]
```

### 9.7. Capturas GUI

Antes de la captura:

- seleccionar la tarea;
- mostrar U/secciones;
- guardar `progress`.

Después de materializar:

- mostrar la misma tarea;
- comprobar nuevas secciones activas;
- guardar `progress`.

Estas capturas serán la evidencia principal de 8.2-B.

### 9.8. Criterio de éxito

La prueba es válida si:

- la observación se clasifica como `VIEW_WALL` o `VIEW_ADVANCE`;
- se generan fuentes depth aceptadas;
- existe evidencia OCCUPIED directa cuando la geometría lo permite;
- se materializan claims de coverage;
- `progress` cambia sólo después de la materialización;
- la GUI refleja las nuevas secciones activas.

---

## 10. Prueba 8.5.6 — autonomía integrada multidrón

### 10.1. Objetivo

Demostrar el ciclo autónomo real con dos drones, dejando que el servidor decida:

- qué tarea recibe cada dron;
- qué sección se inspecciona;
- qué pose se selecciona;
- qué ruta genera D* Lite;
- cuándo se crea una reserva;
- cuándo se mueve el dron;
- cuándo se toma depth;
- cuándo aparece `VIEW_ADVANCE`;
- cuándo aparece `VIEW_WALL`;
- cuándo se actualiza el mapa;
- cuándo se actualiza coverage;
- cuándo se selecciona un nuevo punto.

No introducir goals manuales después de habilitar la autonomía.

### 10.2. Preparación

Usar dos drones.

Llevarlos inicialmente con GT hasta una situación donde ambos:

- estén registrados;
- estén anclados;
- tengan pose global autoritativa;
- puedan entrar en el sistema de tareas.

Después habilitar la ejecución autónoma mediante el mecanismo normal del servidor.

### 10.3. Principio de la prueba

A partir de ese momento:

```text
NO imponer trayectorias manuales
NO seleccionar manualmente el siguiente punto
NO elegir manualmente las tareas
```

El servidor debe gobernar el ciclo.

### 10.4. Elementos que deben observarse

#### Tareas

- asignaciones;
- estados;
- progreso;
- posible `TO_FINISH` si aparece.

#### Selección

- sección U seleccionada;
- pose candidata;
- candidato descartado si ocurre.

#### D* Lite

Registrar:

```text
success
map_revision
expanded
queue_pops
stale_queue_pops
incremental_repair
coarse_factor
waypoints
corridor
latency
```

Aprovechar:

```text
[F6G-PLAN]
[F6I-WORKFLOW-PLAN]
```

#### FREE / UNKNOWN

Comprobar que:

- el movimiento real se ejecuta sólo por FREE;
- UNKNOWN puede intervenir en el razonamiento exploratorio;
- nunca se envía una trayectoria física atravesando UNKNOWN.

#### `VIEW_ADVANCE`

Cuando una ruta estricta hasta el objetivo no sea posible pero exista un prefijo FREE:

```text
D* exploratorio
  ->
último punto FREE alcanzable
  ->
movimiento hasta ese punto
  ->
VIEW_ADVANCE
  ->
actualización del mapa
  ->
nueva selección
```

#### `VIEW_WALL`

Cuando la pose de inspección sea alcanzable por FREE:

```text
MOVE_AND_CAPTURE
  ->
VIEW_WALL
  ->
evidencia de fachada
  ->
coverage
```

---

## 11. Reservas y evidencia para 8.5.3

### 11.1. Objetivo

La propia ejecución 8.5.6 debe servir para obtener las capturas usadas en la subsección 8.5.3.

Antes de ejecutar una trayectoria, el corredor debe reservarse.

Para el otro dron:

```text
RESERVED ≈ obstáculo temporal
```

a efectos de planificación.

La reserva no modifica el mapa raw.

### 11.2. Qué registrar

Guardar:

```text
reservation_id
drone_id
trajectory_id
command_id
reservation_revision
num_reserved_voxels
timestamp_commit
timestamp_release
```

Registrar `/mission/voxel_map` incluyendo `reserved_voxels`.

### 11.3. Capturas GUI

Activar la capa `Reservados` y obtener al menos una captura donde se vea:

- corredor reservado de D1;
- D1 o su trayectoria activa;
- posición de D2;
- resto del mapa voxel.

Si es posible, obtener otra captura después de liberar la reserva para demostrar que desaparece sin alterar el estado raw.

### 11.4. Conflicto entre drones

No es obligatorio fabricar un conflicto extremo.

Si las reservas son compatibles, ambos pueden moverse.

Si aparece un conflicto de forma natural:

- registrarlo;
- registrar espera o ruta alternativa;
- no forzarlo sólo para obtener una figura.

---

## 12. Timeline de la prueba integrada

Crear un CSV temporal con, al menos:

```text
time
drone_id
task_id
task_state
workflow_id
command_id
event
map_revision
coverage_progress
planning_expanded
planning_queue_pops
incremental_repair
reservation_active
reserved_voxels
view_kind
trajectory_active
trajectory_result
```

Debe permitir reconstruir secuencias como:

```text
TASK_ASSIGNED
POINT_SELECTION
DSTAR_PLAN
RESERVATION_COMMIT
MOVE
VIEW_ADVANCE
DEPTH_APPLIED
POINT_SELECTION
DSTAR_PLAN
VIEW_WALL
COVERAGE_UPDATE
RESERVATION_RELEASE
```

Usar los nombres reales de los eventos del código.

---

## 13. Gráficas y capturas de 8.5.6

Generar como mínimo:

### Figura A — trayectorias globales

XY de ambos drones durante la ejecución.

Superponer, si es legible:

- ROI;
- subROIs;
- fiduciales.

### Figura B — evolución de coverage

Por tarea/subROI:

```text
progress(t)
```

o una tabla temporal si resulta más clara.

### Figura C — reservas

Puede ser:

- captura GUI;
- más una pequeña línea temporal de commit/release.

### Figura D — tipos de observación

Línea temporal marcando:

```text
VIEW_UNKNOWN
VIEW_ADVANCE
VIEW_WALL
```

si aparecen.

### Figura E — planificación

Tabla o gráfica con:

```text
expanded
queue_pops
incremental_repair
num_waypoints
```

por plan.

---

## 14. Duración y final de la prueba integrada

El texto actual del capítulo habla de una “misión completa”.

Si por tiempo o estabilidad la ejecución no completa todas las subROIs:

1. no falsear la finalización;
2. conservar la ejecución si demuestra varios ciclos autónomos correctos;
3. indicar en `summary.md`:
   - duración;
   - tareas iniciadas;
   - tareas completadas;
   - cobertura alcanzada;
   - causa de parada;
4. preparar los datos para que posteriormente la memoria pueda denominarla, si procede:

```text
ejecución autónoma integrada representativa
```

en lugar de afirmar que se completó toda la misión.

---

## 15. Criterio de éxito de 8.5.6

La ejecución se considera útil si demuestra de forma reproducible varios ciclos de:

```text
asignar
→ seleccionar
→ planificar
→ reservar
→ ejecutar
→ observar
→ materializar
→ actualizar mapa/coverage
→ seleccionar de nuevo
```

y permite observar al menos:

- dos drones activos;
- tareas reales;
- planes reales;
- reservas reales;
- una actualización depth;
- una actualización de coverage;
- nueva selección o replanificación posterior a la percepción.

No es necesario que aparezcan todos los mecanismos excepcionales en una única ejecución.

---

## 16. 8.5.7 — no ejecutar una prueba nueva

La subsección:

```text
8.5.7 Resultado funcional y límites de la autonomía
```

no requiere una ejecución adicional.

Después de terminar las pruebas anteriores, generar:

```text
Pruebas/Capítulo 8/resumen_resultados.md
```

con:

- qué partes del ciclo autónomo se observaron;
- cuánto avanzó la cobertura;
- si aparecieron `VIEW_UNKNOWN`, `VIEW_WALL` y `VIEW_ADVANCE`;
- comportamiento de las reservas;
- comportamiento de D* Lite;
- si hubo replans;
- si hubo STOP;
- si hubo `TRACKING_RISK`;
- tareas terminadas;
- tareas incompletas;
- limitaciones reales observadas.

No inventar limitaciones que no se hayan visto experimentalmente.

---

## 17. Instrumentación recomendada

Antes de añadir nuevos logs, aprovechar los existentes.

Entre otros, revisar:

```text
[F6A-MISSION-CONFIG]
[F6B-GEOMETRY]
[F6B-ASSIGNED]
[F6D-VOXEL]
[F6E-KF-EVIDENCE-WRITTEN]
[F6F-DEPTH-SOURCES-WRITTEN]
[F6F-DEPTH-SOURCES-APPLIED]
[F6G-VOXEL-DELTAS-APPLIED]
[F6G-PLAN]
[F6H-COVERAGE-SOURCE-PENDING]
[F6H-COVERAGE-CLAIMS]
[F6I-FREE-KF]
[F6I-FREE-VOXEL]
[F6I-WORKFLOW-PLAN]
```

Además:

```text
/mission/geometry
/mission/task_states
/mission/voxel_map
/mission/planned_routes
/mission/flow_events
```

y los topics de navegación/ORB necesarios.

---

## 18. Scripts de análisis

Crear scripts reproducibles bajo:

```text
Pruebas/Capítulo 8/scripts/
```

o la convención vigente.

Los scripts deben poder:

- extraer timeline;
- contar voxels FREE/OCCUPIED/RESERVED;
- obtener revisiones del mapa;
- reconstruir coverage;
- extraer asignaciones;
- extraer planes D*;
- generar gráficas;
- generar tablas resumen.

No editar manualmente los datos procesados.

---

## 19. Material para vídeo y capturas

El usuario tomará los vídeos y fotografías finales.

Codex debe dejar preparado el entorno para que pueda verse de manera clara:

### 8.2-A

- subROIs en GUI.

### 8.2-C

- tarjetas de ambos drones;
- tarea asignada;
- subROI seleccionada.

### 8.2-B / 8.4-B

- coverage antes y después.

### 8.3

- FREE/OCCUPIED del mapa voxel.

### 8.4-A

- región UNKNOWN antes;
- nuevos FREE después.

### 8.5.6

- trayectorias;
- reservas;
- `VIEW_ADVANCE`;
- `VIEW_WALL`;
- coverage;
- movimiento autónomo de ambos drones.

---

## 20. Preguntas que Codex debe responder al finalizar

### 8.2-A

- ¿cuántos niveles se generaron?
- ¿hay cuatro subROIs por nivel?
- ¿la GUI las representa correctamente?

### 8.2-C

- ¿ambos drones quedaron registrados?
- ¿ambos fueron elegibles?
- ¿qué tarea recibió cada uno?
- ¿qué subROI corresponde a cada tarea?
- ¿la GUI lo muestra correctamente?

### 8.3

- ¿qué evidencia existía antes del anclaje?
- ¿qué se materializó al anclar?
- ¿cuántos FREE/OCCUPIED había después del anclaje?
- ¿cuántos había tras llegar a `(10,-10,1,90°)`?
- ¿qué fuentes contribuyeron?

### 8.4-A

- ¿se ejecutó `LOOK_AND_CAPTURE`?
- ¿se clasificó como `VIEW_UNKNOWN`?
- ¿cuántos FREE nuevos aparecieron?
- ¿apareció OCCUPIED?
- ¿cambió coverage?
- ¿quedaron zonas UNKNOWN sin observar?

### 8.4-B

- ¿la captura fue `VIEW_WALL` o `VIEW_ADVANCE`?
- ¿cuántas fuentes depth se escribieron?
- ¿cuántos FREE/DIRECT_FREE/OCCUPIED se materializaron?
- ¿qué sección U recibió claims?
- ¿cuánto cambió `progress`?
- ¿la GUI refleja ese cambio?

### 8.5.6

- ¿qué tareas recibió cada dron?
- ¿qué planes D* se generaron?
- ¿cuántos waypoints?
- ¿cuántas expansiones?
- ¿hubo reparaciones incrementales?
- ¿qué reservas se crearon?
- ¿hubo conflictos?
- ¿apareció `VIEW_ADVANCE`?
- ¿apareció `VIEW_WALL`?
- ¿cuántas capturas depth válidas hubo?
- ¿cuánto avanzó cada coverage?
- ¿qué tareas terminaron?
- ¿la ejecución completó la misión o fue parcial?
- ¿qué limitaciones reales aparecieron?

---

## 21. Principio final

Estas pruebas deben demostrar el comportamiento real del sistema, no producir artificialmente una narrativa favorable.

Si un mecanismo no aparece:

```text
registrar el resultado
→ comprobar si el escenario era adecuado
→ modificar sólo el escenario si procede
→ no cambiar arbitrariamente la lógica del algoritmo
```

La memoria final debe describir exactamente lo que se haya observado.
