# Pipeline Fase 6 — Tareas, cobertura autónoma y trayectorias multi-dron

Resumen de entrada:

```text
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6_RESUMEN.md
```

## Estado

```text
FASE: SIN HACER
Preparación documental: CERRADA
Acuerdo cerrado: sí
Autorización funcional de ejecución: PENDIENTE
Prueba final acordada: dar el ROI de la casa y comprobar que N drones mapean autónomamente todo el entorno accesible
Dudas abiertas: ninguna
```

Este pipeline es un **contrato documental**. No contiene resultados reales de build, simulación ni historial. Las carpetas de historial se entregan vacías de forma intencionada y solo se crearán MD de historial cuando exista una ejecución real.

## Objetivo general

Convertir el sistema posterior a Fase 5 en una plataforma capaz de recibir una única configuración de misión antes de arrancar la autonomía, generar todos los trabajos de cobertura sparse, repartirlos entre N drones y permitir que cada dron descubra y recorra autónomamente un entorno desconocido, manteniendo tracking ORB-SLAM3, evitando paredes/objetos y coordinándose con el resto de drones mediante reservas de trayectorias.

La Fase 6 **no** construye la nube densa global. Debe dejar preparado el uso local de stereo/depth y contratos reutilizables para que Fase 8 pueda combinar después sparse+dense, mejorar geometría y construir la reconstrucción densa final.

## Fuente de misión: `tarea_principal.yaml`

Antes de iniciar el servidor de misión existe un YAML de tarea principal. En esta fase solo describe una misión de **mapeo sparse completo**.

Esquema conceptual, no nombre de campos definitivo si el código real ya posee equivalentes:

```yaml
drones:
  - drone_id: ...
    namespace: ...
    # tamaño/radio de seguridad o referencia a config local del Servidor,
    # según ownership YAML definido por Fase 2

mapping_roi:
  min: [x, y, z]
  max: [x, y, z]

mapping_hysteresis: ...

flight_bounds:
  min: [x, y, z]
  max: [x, y, z]

level_height: ...
tasks_per_level: 4   # baseline alternativo: 8
```

### `mapping_roi`

El ROI es el volumen cuyo contenido accesible se quiere reconstruir. **No define por dónde tiene que volar el dron.** El dron puede salir del ROI para conseguir una buena vista, rodear una pared o realizar una trayectoria segura.

La misión persigue todo lo accesible dentro del ROI. Si una zona está dentro del cuboide pero físicamente no se puede alcanzar —por ejemplo una cara exterior inaccesible desde el interior de una casa cerrada— no debe provocar una exploración infinita.

Los MapPoints que ORB-SLAM3 produzca fuera del ROI siguen siendo datos válidos del mapa y pueden publicarse en RViz2. La autonomía simplemente no debe seguir alejándose para completar regiones exteriores que no forman parte de la misión.

### `mapping_hysteresis`

Permite continuar brevemente una superficie/estructura que sobrepasa el ROI. Evita cortar el mapeo exactamente en un plano matemático. No convierte toda la región exterior en objetivo obligatorio.

### `flight_bounds`

Es un segundo cuboide, también axis-aligned en `world`, independiente del ROI. Actúa como límite duro para impedir que un fallo de planificación lleve un dron demasiado lejos. **Nunca se usa para generar secciones de cobertura.**

### Drones y parámetros físicos

La misión conoce qué drones participan y su identidad real. El servidor necesita tamaño/radio conservador y margen de seguridad para coordinación Dron-Dron. La fuente exacta debe respetar la política de ownership YAML de Fase 2: si el dato físico vive en Dron, el Servidor usa una réplica parcial declarada o una referencia/config local coherente; no lee directamente YAML del otro grupo en runtime.

## Rebanadas verticales

El ROI se divide verticalmente mediante `level_height`. La altura sobrante se suma a la **última** rebanada.

Ejemplo:

```text
ROI z=[0,7], level_height=2

level 0 = [0,2]   z nominal ~= 1
level 1 = [2,4]   z nominal ~= 3
level 2 = [4,7]   z nominal ~= 5.5
```

No se crea `[4,6] + [6,7]`.

La altura nominal guía la cobertura. El dron puede subir o bajar para esquivar obstáculos, mantener tracking o conseguir una vista mejor, pero debe cubrir la banda vertical asignada.

No existe una barrera de ejecución entre pisos: si hay más drones que trabajos útiles en un nivel, los sobrantes pueden recibir tareas del siguiente.

## Puntos de sección y `tasks_per_level`

Baseline obligatorio:

```text
tasks_per_level = 4
  -> cuatro esquinas

tasks_per_level = 8
  -> cuatro esquinas + centro de cada lado
```

Cada punto B conoce los dos puntos adyacentes A y C. No existe un “anterior” universal ni un sentido horario obligatorio.

Para B se crea:

```text
MAP_SECTION_B
    entradas posibles: A o C
    objetivo nominal: A -> B -> C  o  C -> B -> A
```

La entrada se decide al asignar la tarea en función del dron. El servidor **no** asigna B al dron más próximo a B; compara el coste para llegar a sus entradas A/C.

## Solape deliberado entre tareas

El solape no es un defecto. Es parte de la estrategia multi-dron.

```text
Task B: A -> B -> C
Task C: B -> C -> D
```

Ambas tareas deben observar B→C. Si las ejecutan drones/submapas distintos, esta repetición aumenta la evidencia compartida para loops, fusión y unión de submapas.

El estado global de cobertura puede reducir exploración redundante no necesaria, pero **no** debe eliminar el tramo `required_overlap` de tareas vecinas.

## Significado real de `MAP_SECTION`

Una `MAP_SECTION` no significa “volar por tres puntos”. A/B/C son una **semilla de cobertura**.

El dron tiene que descubrir lo accesible desde la entrada hasta la salida pasando por la región principal y adaptarse a la geometría:

- fachada simple;
- pared en L;
- habitación;
- pabellón con paredes interiores;
- pasillos;
- casa compleja;
- laberinto.

El mismo programa debe funcionar en interior y exterior. No habrá un modo `interior` y otro `exterior`.

La implementación debe combinar una representación local `known free / occupied / unknown`, frontiers/next-best-view o un método equivalente que cumpla el contrato. Las nuevas ramificaciones descubiertas se convierten en **subobjetivos internos de la tarea**, no en nuevas tareas globales que cambien el criterio de misión.

Una tarea termina cuando se ha cubierto su progreso nominal entrada→B→salida y no quedan frontiers relevantes/alcanzables asociadas a esa cobertura que deba resolver. Llegar a C no es suficiente. Regiones demostrablemente inaccesibles pueden cerrarse como bloqueadas/inaccesibles sin mantener la tarea infinita.

La misión completa termina cuando todas las tareas iniciales de mapeo están `COMPLETED`. El sistema no debe permitir que todas terminen si existe una frontier accesible del ROI descubierta pero olvidada/sin responsabilidad.

## Tarea ≠ trayectoria

Invariante central:

```text
MAP_SECTION_B  (tarea larga)
    |
    +-- trajectory_001
    +-- trajectory_002
    +-- trajectory_003 CANCELLED
    +-- trajectory_004
    +-- ...
```

Las trayectorias son **locales, cortas y reemplazables**. Deben existir límites configurables de longitud y duración. Esto permite:

1. replanificar cuando aparece geometría nueva;
2. liberar rápido regiones reservadas;
3. reducir el conservadurismo del anticolisión puramente espacial;
4. evitar comprometerse con una ruta larga en mapa desconocido.

Al completar/cancelar una trayectoria, el dron lo comunica inmediatamente al servidor y la reserva desaparece. La `MAP_SECTION` continúa `RUNNING` salvo que su criterio de cobertura se haya completado/fallado.

## Waypoints y `lib_tray`

`TrayAction`/`lib_tray` se amplían para aceptar rutas de múltiples waypoints manteniendo compatibilidad con los generadores legacy.

Cada waypoint incluye:

```text
x, y, z, yaw
```

El yaw es parte funcional de la planificación porque ORB-SLAM3 depende de lo que mira la cámara.

`lib_tray` debe producir referencias temporales continuas entre waypoints y mantener cancelación/resultados. No se reemplaza la cadena de control de Fase 1; se la alimenta con una trayectoria más expresiva.

## Responsabilidad Dron vs Servidor

### Dron

Responsable de:

- paredes y obstáculos físicos;
- espacio conocido/desconocido local;
- evitar salirse de `flight_bounds` como defensa local;
- conservar tracking ORB-SLAM3;
- decidir yaw/vista;
- explorar/cubrir la tarea;
- generar y replanificar trayectorias locales;
- frenar/hover por seguridad sin pedir permiso.

### Servidor

Responsable de:

- cargar misión;
- generar tareas;
- mantener cola/lifecycle;
- asignar tareas;
- conocer reservas de todos los drones;
- autorizar/rechazar propuestas por Dron-Dron;
- conocer el volumen actual de drones parados;
- sugerir desvíos alrededor de otros drones;
- no decidir cómo rodear paredes.

`orbslam3_multi` sigue siendo backend de mapa sparse; la misión/coordinación no debe convertirse en lógica de `RawMapDatabase`, loops, fusión o pose graph.

## Percepción local de obstáculos

Los MapPoints ORB ayudan a localización y a saber dónde existe información visual, pero **no** son un mapa de colisiones fiable. Una pared blanca puede tener pocos MapPoints.

Fase 6 incorpora una percepción local/temporal estéreo/depth basada en las cámaras disponibles. Puede reutilizar ideas de los experimentos de `dron_individual/src/vision/`, tras auditarlos, pero no construye TSDF/Open3D global.

```text
stereo/depth -> obstáculos / free / unknown -> seguridad LocalPlanner
ORB tracking -> soporte visual -> mantener localización/yaw
```

Fase 8 retomará estas fuentes y podrá combinarlas con sparse/dense para una reconstrucción global de mayor calidad.

## Información visual de ORB-SLAM3

El `control_trayectorias` necesita conocer el estado de tracking y soporte visual actual. Se reutiliza lo que Fase 5/wrapper ya publique. Si falta, se amplía el wrapper con una señal ligera de frame actual; no se usa `OrbMap` global como sustituto.

Si el wrapper no puede obtener un dato necesario mediante la API disponible, se debe parar y acordar una modificación mínima del núcleo ORB_SLAM3 antes de tocarlo.

Pocos MapPoints significan “mala información visual”, **no** “no hay objeto”.

## Política de yaw/vista

No habrá reglas rígidas `exterior -> mirar dentro` / `interior -> mirar fuera`. Se seleccionan vistas con una jerarquía:

```text
1. no colisionar
2. permanecer dentro de flight_bounds
3. conservar tracking
4. avanzar cobertura
5. observar información nueva + solape
6. eficiencia
```

Al avanzar lateralmente junto a una superficie, el dron debe mirarla la mayor parte del tiempo. Puede girar gradualmente hacia el siguiente tramo para inspeccionarlo. Si el soporte ORB cae, deja de girar y vuelve a una vista estable; depth sigue siendo quien decide si hay obstáculo.

## Replanning online

El mapa se descubre mientras el dron vuela. Por tanto ninguna trayectoria local se considera válida para siempre.

```text
plan corto -> autorización -> ejecución
                    |
             nueva percepción
                    |
          ¿sigue siendo válido?
           | sí           | no
           v              v
        seguir     STOP/HOVER
                       -> cancelar/release
                       -> replanificar
                       -> nueva propuesta
```

Una trayectoria autorizada por el servidor **no se modifica silenciosamente**. Si cambia su geometría, se cancela/libera y se presenta otra.

## Coordinación Dron-Dron: reservas espaciales

Las solicitudes de trayectoria se encolan en el servidor y se procesan secuencialmente. Una reserva aceptada entra en el registro antes de comprobar la siguiente solicitud. Esto evita la carrera “ambos vieron libre”.

Cada trayectoria se convierte en un corredor/volumen barrido conservador usando:

```text
tamaño/radio del dron
+ distancia de seguridad
+ margen de error acordado
```

Un dron parado también ocupa un volumen de seguridad.

### Sin tiempo en el baseline

La validación es **puramente espacial**.

Si dos corredores se cruzan, la segunda propuesta se rechaza aunque los drones pudieran cruzar en tiempos distintos. No se implementan ventanas temporales, predicción de retrasos ni sincronización como requisito de Fase 6.

Las trayectorias cortas y el release inmediato evitan que esta política bloquee regiones durante demasiado tiempo.

## Hints de desvío `A→C→B`

Ante un conflicto Dron-Dron, el servidor puede devolver uno o varios puntos C que eviten las reservas conocidas.

```text
Dron propone A -> B
Servidor: conflict con D2, hint C
Dron:
  valida C contra depth/tracking
  calcula ruta A -> C -> B (o equivalente)
  genera NUEVO trajectory_id
  la vuelve a proponer
Servidor:
  vuelve a comprobarla
```

C es un **hint**, nunca autorización. El servidor no sabe si hay una pared en C. El dron puede rechazarlo.

## `ANCHOR_SUBMAP`

Si el `(drone_id,map_epoch)` actual no está anclado pero el dron conserva tracking local, se activa un comportamiento de recuperación:

- movimientos relativos/locales conservadores;
- pequeños giros para buscar entorno;
- buscar fiducial visual o región conocida que permita loop/anclaje en servidor;
- mantener vistas con suficientes MapPoints/soporte;
- abortar un giro si empeora tracking;
- límites de tiempo/distancia/intentos;
- sin GT.

Cuando el servidor confirma anchor del epoch vigente, se puede continuar la tarea global.

## `GO_TO(x,y,z,yaw)`

La futura GUI de Fase 7 podrá insertar una tarea `GO_TO`.

Política acordada:

- máxima prioridad **entre tareas pendientes**;
- no interrumpe una tarea ya `RUNNING`;
- cuando la tarea actual acaba, `GO_TO` es la siguiente;
- usa el mismo LocalPlanner, depth, tracking, reservas y anticolisión que una tarea automática;
- no se transforma en un `TrayAction` directo sin validación.

## Comunicación perdida

No es escenario obligatorio de la simulación actual. Si se implementa de forma defensiva, el criterio acordado es:

- el dron puede terminar su trayectoria activa de forma segura;
- no inicia otra trayectoria sin recuperar comunicación/autorización;
- el servidor no considera inmediatamente libre el último volumen conocido.

Esto no es requisito de cierre salvo que la arquitectura real introduzca esa necesidad durante implementación.

## Interfaces compartidas

Fase 2 dejó `orbslam3_msgs` duplicado de forma controlada entre Dron y Servidor, con copia canónica en Servidor. Los nuevos contratos de tarea/trayectoria deben seguir esa política o reutilizar interfaces equivalentes ya creadas por Fase 5.

No crear dos mensajes semánticamente distintos para Dron/Servidor. Cada modificación de la copia canónica debe replicarse y pasar la guarda de igualdad.

## Arquitectura objetivo

```text
                         SERVIDOR

                 tarea_principal.yaml
                         |
                         v
                  MissionGeometry
                         |
                         v
                  Mission/TaskManager
                         |
                    TaskAllocator
                         |
        +----------------+----------------+
        |                |                |
      Drone 0          Drone 1          Drone N
        |                |                |
        |  trajectory proposals / releases|
        +----------------+----------------+
                         |
                ReservationManager
                         |
                  ConflictDetector
                         |
                 ConflictHintGenerator


                         DRON

                  TaskExecutor
                       |
                CoveragePlanner
                       |
       +---------------+---------------+
       |               |               |
  local depth     ORB visual state   ROI/task
       |               |               |
       +-------> Local/View Planner <---+
                       |
               short waypoints
                       |
              propose to server
                       |
                   ACCEPTED
                       |
              TrayAction/lib_tray
                       |
              existing controller
```

## Subfases

| Subfase | Nombre | Objetivo |
|---|---|---|
| `6A` | Contrato de `tarea_principal.yaml` y configuración de misión | Definir una única configuración de misión que el servidor cargue antes de comenzar la autonomía. |
| `6B` | Geometría del ROI, rebanadas verticales y puntos nominales de sección | Transformar el ROI validado en niveles verticales y en un conjunto ordenado de puntos nominales por nivel que sirvan como semillas de cobertura, sin convertir las aristas del cuboide en trayectorias físicas obligatorias. |
| `6C` | Generación inicial de tareas `MAP_SECTION` con solape deliberado | Crear, antes de empezar la misión, todas las tareas de mapeo de todos los niveles y definir su objetivo nominal A–B–C con solape explícito entre secciones vecinas. |
| `6D` | Contratos ROS 2 y ciclo de vida de tareas | Definir interfaces compartidas entre Servidor y Dron para representar tareas, asignación, petición de nueva tarea, progreso, finalización y error, sin mezclar la tarea de larga duración con las trayectorias locales que se usarán para ejecutarla. |
| `6E` | Gestor de misión y cola global de tareas en el servidor | Implementar en el servidor la autoridad de estado de misión: almacenar todas las tareas iniciales, mantener su lifecycle, conocer drones disponibles y entregar trabajo cuando un dron lo solicite, sin decidir todavía la métrica avanzada de asignación de 6F. |
| `6F` | Asignación de tareas por entrada, cercanía y continuidad | Sustituir la selección provisional de la cola por un asignador que elija tareas cercanas y un sentido de ejecución adecuado, usando la posición global del dron y las dos entradas posibles de cada `MAP_SECTION`. |
| `6G` | Ejecutor embarcado de tareas y nodo `control_trayectorias` | Crear en el grupo Dron un ejecutor de alto nivel que reciba una tarea, mantenga su lifecycle local y coordine planificación, solicitud de reserva, ejecución de `TrayAction`, replanning y resultado sin delegar control de alta frecuencia al servidor. |
| `6H` | Contrato de trayectorias locales por waypoints `(x,y,z,yaw)` | Definir una representación compartida de trayectoria local propuesta al servidor y ejecutable por el dron, basada en una lista de waypoints con posición y `yaw`, identidad/revisión y límites suficientes para reserva y cancelación. |
| `6I` | Extensión de `lib_tray` y `TrayAction` para trayectorias multi-waypoint | Hacer ejecutables las listas de waypoints sin sustituir el controlador existente: `lib_tray` debe generar referencias continuas entre puntos y `gen_tray`/`TrayAction` debe evaluar el segmento correcto en cada instante, incluyendo `yaw`. |
| `6J` | Lifecycle de trayectorias locales cortas, cancelación y liberación | Imponer que las tareas largas se ejecuten mediante trayectorias locales de horizonte acotado, con límites configurables de distancia/duración y un lifecycle claro que permita terminar/cancelar una trayectoria sin terminar la tarea. |
| `6K` | Percepción estéreo/depth local para seguridad y espacio navegable | Crear en el dron una percepción local y temporal de obstáculos físicos a partir de estéreo/depth que permita decidir si el siguiente tramo es seguro, sin construir ni fusionar todavía la nube densa global de Fase 8. |
| `6L` | Estado de tracking y soporte visual actual de ORB-SLAM3 para navegación | Proporcionar al `control_trayectorias` información ligera y actual sobre la calidad de tracking y lo que ORB-SLAM3 está viendo para evitar movimientos/orientaciones que puedan hacer perder la localización. |
| `6M` | Planificación de yaw y selección de vistas perceptivamente seguras | Hacer que el dron planifique explícitamente hacia dónde mirar durante el movimiento, maximizando información útil y continuidad de tracking sin codificar modos distintos para interior/exterior. |
| `6N` | Cobertura adaptativa de `MAP_SECTION` mediante exploración y frontiers | Implementar el significado real de una `MAP_SECTION`: descubrir y observar todo lo accesible desde su entrada hasta su salida nominal, adaptándose a paredes, habitaciones, pasillos, geometría en L y laberintos, con solape deliberado entre tareas vecinas. |
| `6O` | Replanning incremental/receding horizon durante la ejecución | Hacer que cada tarea se ejecute mediante planificación de horizonte corto que se reevalúa continuamente al descubrir nueva geometría, obstáculos o degradación visual, cancelando de forma segura el segmento actual y proponiendo otro cuando deja de ser válido. |
| `6P` | Cola serializada y registro de reservas de trayectorias en el servidor | Crear la autoridad de coordinación Dron-Dron del servidor: recibir propuestas de trayectoria en una cola, procesarlas secuencialmente, registrar una única versión aceptada como reserva activa y liberarla inmediatamente al terminar/cancelar. |
| `6Q` | Detección espacial de conflictos Dron-Dron mediante corredores 3D | Rechazar una propuesta cuando su volumen barrido/corredor de seguridad intersecta la trayectoria reservada o el volumen actual de otro dron, usando una política puramente espacial y conservadora, sin planificación temporal. |
| `6R` | Sugerencias de desvío servidor `A→C→B` ante conflicto multi-dron | Hacer que un rechazo Dron-Dron pueda incluir uno o varios waypoints intermedios que eviten los corredores reservados, sin convertir al servidor en planificador de paredes ni considerar la sugerencia una autorización de vuelo. |
| `6S` | Tareas/comportamientos especiales `ANCHOR_SUBMAP` y `GO_TO` | Integrar dos comportamientos que no deben confundirse con la cobertura normal: recuperación de anclaje global de un submapa manteniendo tracking local, y orden `GO_TO(x,y,z,yaw)` de máxima prioridad pendiente para la futura GUI. |
| `6T` | Integración autónoma multi-dron, cobertura completa del ROI y cierre de Fase 6 | Integrar toda la misión de Fase 6 y demostrar que un `tarea_principal. |

## Dependencias principales

```text
6A -> 6B -> 6C
                           -> 6D -> 6E -> 6F -> 6G
                                                                 -> 6H -> 6I -> 6J
                                                                             -> 6P -> 6Q -> 6R

6G -> 6K -> 6M -> 6N -> 6O
               -> 6L -> 6M

6J + 6O + 6R -> 6S
6A..6S -> 6T
```

6K/6L pueden prepararse en paralelo con parte de 6H–6J cuando el contrato de 6G ya esté estable. La ejecución real debe respetar las dependencias funcionales, no necesariamente el orden alfabético si dos subfases son independientes.

## Reglas transversales de seguridad y arquitectura

1. No usar GT como entrada funcional en ninguna decisión de Fase 6.
2. No usar MapPoints como única fuente de colisión física.
3. No implementar nube densa global/TSDF de Fase 8.
4. No introducir Nav2 como columna vertebral baseline: la autonomía requiere 3D, yaw perceptivo, frontiers y reservas multi-dron propias. Una librería externa solo puede adoptarse tras demostrar que satisface el contrato sin forzar un modelo 2D.
5. No permitir que el servidor esquive paredes.
6. No permitir que el dron ignore una reserva aceptada por servidor.
7. Un dron siempre puede frenar por seguridad.
8. Toda modificación geométrica de una trayectoria requiere nueva propuesta.
9. No reservar la trayectoria completa de una `MAP_SECTION`.
10. No usar tiempo para aceptar cruces Dron-Dron en el baseline.
11. No eliminar solape nominal por optimización de cobertura.
12. No bloquear niveles inferiores/superiores artificialmente.
13. No marcar una tarea completa solo por llegar al waypoint final.
14. No inventar interfaces de Fase 5: localizar y reutilizar las existentes.
15. No engordar `global_map_server.cpp` con toda la misión si pueden crearse componentes/clases separados dentro de `orbslam3_server`.
16. `orbslam3_multi` conserva su ownership del backend sparse; misión y navegación no se meten en RawMapDatabase/pose graph.

## Política de ejecución de las subfases

Cada MD de `subfases/` es un contrato ejecutable, no una autorización automática. Al recibir en el futuro “haz 6X”, Codex debe aplicar la puerta de preparación de `AGENTS.md`: leer el estado real, Fase 5 vigente, docs de paquetes e historial, explicar el plan/prueba, cerrar dudas y esperar autorización explícita antes de modificar/compilar/simular.

## Prueba oficial de cierre

Entrada: `tarea_principal.yaml` con ROI de la casa y N drones (mínimo 2 para demostrar coordinación multi-dron; N sigue configurable).

El sistema debe:

1. generar todas las tareas de todos los niveles;
2. asignarlas automáticamente;
3. permitir niveles concurrentes;
4. ejecutar cobertura adaptativa interior/exterior con el mismo algoritmo;
5. evitar obstáculos físicos con percepción local;
6. mantener tracking mediante planificación de vistas;
7. usar trayectorias cortas y replanning online;
8. serializar/autorizar reservas Dron-Dron;
9. resolver al menos un conflicto mediante rechazo y replan/hint si el escenario lo produce/inyecta;
10. liberar trayectorias terminadas/canceladas inmediatamente;
11. conservar solape entre tareas vecinas/submapas;
12. completar todas las tareas iniciales;
13. mostrar en RViz2 un sparse global razonablemente completo de todo lo accesible dentro del ROI;
14. dejar claras las regiones inaccesibles sin convertirlas en bucles infinitos;
15. no utilizar GT funcional.

## Criterio de cierre de Fase 6

`CONSEGUIDA` únicamente si:

- builds de los grupos/paquetes afectados pasan;
- la misión autónoma se inicia desde YAML y no desde rutas manuales precalculadas;
- todas las tareas obligatorias terminan;
- no existen colisiones Dron-Dron ni con obstáculos en la evidencia de prueba;
- las trayectorias se mantienen cortas, se liberan y se replantean cuando toca;
- tracking/pose usados por control proceden del pipeline sin GT;
- el sparse global cubre el entorno accesible del ROI con solape suficiente;
- no quedan reservas huérfanas;
- no se ha adelantado la nube densa global de Fase 8;
- documentación e historiales reales reflejan cada ejecución sin borrar fallos anteriores.

`PARCIAL` si existe evidencia positiva pero falta una condición obligatoria. `NO CONSEGUIDA` si no compila, no se ejecuta la prueba o el resultado contradice la misión. `BLOQUEADA` solo ante dependencia externa real no resoluble con cambios mínimos.
