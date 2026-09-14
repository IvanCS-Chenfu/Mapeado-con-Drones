# Subfase 6G - Planificador global D* Lite y waypoints XYZ

## Estado

```text
CONSEGUIDA
```

## Dependencia

6D, 6E y dimensiones registradas por 6C.

## Objetivo tecnico

Implementar en `task_lib` D* Lite 3D reproducible e incremental sobre el mapa
voxel y simplificar su ruta a waypoints geometricos. El resultado vigente se
solicita por `/mission/plan_route` y se publica como `TrajectoryPlan` previsto
en `/mission/planned_routes`; 6I sigue siendo la propietaria de la trayectoria
continua ejecutable.

## Contrato

D* responde como llegar a un objetivo XYZ; no elige coverage, yaw, pitch ni la
trayectoria fisica. Usa 26-connectivity con longitudes reales y validacion de
diagonales para no cortar esquinas ocupadas.

```text
FREE normal | UNKNOWN transitable penalizado | OCCUPIED bloqueado
```

Para exploracion, 6H entrega como destino solamente un voxel `UNKNOWN` seguro
dentro de la subROI y a la distancia minima configurada desde la pose actual;
D* no escoge ese destino, solo calcula o repara el camino. Puede recorrer
`UNKNOWN` con coste penalizado y `FREE` con coste normal porque ambos son parte
del camino de descubrimiento. Si un delta de `VoxelMapWorker` degrada una celda
del corredor, D* repara hacia la misma meta; si la meta deja de ser `UNKNOWN`,
se vuelve `OCCUPIED`, inflada o inalcanzable, devuelve el rechazo para que 6H
la descarte y seleccione otra. Esta regla no incorpora todavia rayos depth:
esos deltas pertenecen a su subfase propia.

El footprint se aproxima mediante inflacion conservadora XYZ: por cada eje usa
la semidimension registrada redondeada a voxeles mas
`extra_obstacle_clearance_voxels=2`. Este margen adicional se mide desde la
envolvente fisica del dron. La bounding box orientada/swept volume de la curva
continua se valida en 6I antes de ejecutar. D* recibe el plan completo hasta la
meta; `task_server` asigna sus tiempos desde `vel_max`, sin recortarlo por una
distancia o duracion artificial.

El margen se aplica en D*, nunca al marcar FREE. La pose inicial real es una
excepcion local: puede estar dentro del margen de una superficie, porque el
dron ya ocupa esa posicion. En ese caso D* busca por coste minimo una salida
6-conexa hasta la primera celda que recupera el clearance completo; `FREE`
tiene menor coste que `UNKNOWN`, pero ambos son admisibles y `OCCUPIED` nunca
se atraviesa. Desde esa celda la ruta vuelve a usar el perfil inflado normal y
la meta conserva el margen duro. Si no existe salida devuelve
`NO_SAFE_ESCAPE`. Si el objetivo deja de ser alcanzable con el mapa actual
devuelve `UNREACHABLE_CURRENT_MAP`, para que 6E/6H decidan alternativa o
pendiente sin reintentos infinitos.

## Cambios requeridos

1. Implementar D* Lite puro con costes actualizables y estado incremental.
2. Consumir `MapChangeEvent` por revision/chunks emitido por `VoxelMapWorker`,
   no reconstruir mapa completo ni inspeccionar snapshots globales.
3. Reparar solo cambios relevantes y medir plan inicial frente a repair.
4. Aplicar occupancy, clearance, UNKNOWN y costes configurables sin fijar pesos.
5. Simplificar ruta mediante LOS/shortcutting medido, sin confundirla con trayectoria.
6. Exponer el volumen navegable inflado del corredor futuro, no solo su linea
   central, para que 6I detecte los deltas que exigen reparacion.
7. Actualizar grafo con queue, reason, current drone y latencias.
8. Incorporar escape inicial local por FREE/UNKNOWN no ocupado hasta recuperar
   el perfil completo, y propagar `NO_SAFE_ESCAPE` y
   `UNREACHABLE_CURRENT_MAP` al servicio, log y grafo.

## Implementacion validada

- `ReversibleVoxelMap::TakeChanges()` entrega deltas before/after a un
  `DStarLitePlanner` persistente por dron en `task_server`.
- `PlanningWorker` consume el snapshot navegable inmutable de una revision y
  sus deltas locales desde `VoxelMapWorker`; no recalcula inflacion, diagonales
  ni costes base durante cada expansion. `g/rhs` siguen siendo privados y
  dependientes del objetivo de cada planificador.
- Cada `NavigationCell` materializa solo estado raw, transitabilidad, una
  mascara compacta de 26 conexiones y coste de entrada. `VoxelMapWorker`
  actualiza estas celdas por `footprint_profile_id` y `PlanningWorker` aplica
  exclusivamente sus deltas al estado D* del dron. Ni heuristica ni `g/rhs`
  se comparten entre objetivos, y las reservas espacio-temporales solo tienen
  interfaz prevista hasta que 6J asuma su autoridad.
- La guia gruesa por perfil usa `coarse_voxel_factor=4` por defecto. El planner
  obtiene de ella un corredor orientativo y comienza la busqueda fina dentro de
  una banda de 8 voxeles finos; si no hay ruta en esa banda, la amplia de 4 en
  4 hasta la ventana completa. La malla gruesa no puede aceptar ni rechazar una
  ruta: occupancy, inflacion y diagonales se validan solo en la malla fina.
- La cola D* es un heap indexado: actualiza en sitio la unica entrada activa de
  cada voxel y la retira al quedar consistente. El log distingue `queue_pops`,
  `stale_queue_pops` y expansiones de estado reales; los obsoletos deben tender
  a cero y no consumen el limite. La prioridad fina aplica una heuristica
  ponderada configurable con `heuristic_weight=1.2`; favorece respuesta rapida,
  sin alterar transitabilidad ni convertir una trayectoria en orden ejecutable.
- La implementacion usa un heap binario indexado por voxel: cada estado tiene
  una sola entrada activa y una modificacion de `g/rhs` actualiza esa prioridad
  en sitio. Al quedar consistente se elimina del heap. Se mantienen los dos
  contadores de cola para comparar rendimiento, pero los `stale_queue_pops`
  deben tender a cero; no son una via para omitir validaciones ni expansiones.
- La busqueda respeta el volumen duro y usa una ventana inicio-objetivo
  voxelizada y margen configurable, sin limite propio de distancia. 6I limitara
  la longitud de cada tramo que vaya a ejecutar.
- `PlanRoute` aporta la entrada explicita de pruebas; el servidor publica
  `dstar_<drone>_<revision>` transient-local y emite `DSTAR_PLAN`,
  `DSTAR_REPLAN` o `DSTAR_REJECT` al grafo con revision, latencia y expansiones.
  Los detalles de cola, guia y ensanchamiento solo se escriben en el log de
  fase; el grafo no muestra esos datos y la GUI F7 solo muestra el plan final.
- La GUI F7 consume exclusivamente ese plan como `Plan previsto`; no se envia
  al controlador de vuelo ni se finge que sea una trayectoria 6I.

## Limites

No integrar yaw/pitch en el estado D*, no validar dinamica con la polilinea y no
interrumpir el planner desde otro thread. Al acabar se comparan revisiones.

## Pruebas

Los GTests cubren mapa UNKNOWN, pared con hueco, pasillo bloqueado, obstaculo
añadido o retirado con reparacion, diagonales inseguras, horizonte y estabilidad
ante jitter subvoxel. La prueba Gazebo 615 con GUI F7 y GT valido un plan inicial
(`expanded=607`) y un replan incremental (`expanded=0`), ambos con dos waypoints
y reemplazo visual por `trajectory_id`.
La 616 arranco/cerró limpiamente pero no pudo solicitar plan/replan por un
rechazo externo de autorizacion; se conserva como evidencia de infraestructura,
no como sustituto de 615.

La reapertura 617/618 retiro el limite de distancia y materializo hasta 2.562
voxeles desde MapPoints frente al fiducial 2. El plan a `(-9,-4,1.0)` se rechazo
con `start_occupied_or_inflated`; se acuerda ahora habilitar la evidencia FREE
reversible de 6I y el escape local, sin excepcion global de seguridad.

La prueba siguiente conserva esa politica y explora candidatos cercanos al
objetivo deseado `(-9,-4,Z)` hasta hallar el primero que D* acepte. Es una
seleccion explicita de banco de pruebas: en produccion el punto de observacion
con clearance sera responsabilidad del planificador de tareas/vista de 6H/6M,
no del generador de trayectorias ni de D*.

No se anade todavia un presupuesto temporal de planificacion. Si las metricas
posteriores lo justifican, se acordara uno monotono que devuelva
`planning_time_budget_exhausted` junto con expansiones y latencia, sin ampliar
distancia, relajar el margen ni cambiar la clasificacion del objetivo.

Si aparece revision `R+1` durante un plan contra snapshot `R`, el plan no lee
datos mutables a mitad de expansion. Al acabar, el servidor compara el delta
con el volumen navegable inflado del corredor propuesto: si lo degrada, lo
repara o descarta antes de publicarlo. Un cambio ajeno al volumen no altera el
plan. La reparacion de una trayectoria activa pertenece a 6I y siempre parte
de la pose canonica actual, nunca de los waypoints almacenados anteriormente.

## Criterio de exito

Rutas XYZ seguras segun el modelo voxel, repairs incrementales correctos,
UNKNOWN navegable con cautela y metricas suficientes para ajustar parametros.
La telemetria adicional se compilo y paso CTest; su inspeccion en vivo queda
validada por la prueba 625: la actualizacion local que en 624 tardo 16.060 ms
para 24 cambios raw y 1.056 celdas se redujo a 110.169 ms para 24 cambios y
1.122 celdas. La mayor actualizacion observada de 988 cambios/34.703 celdas
tardo 2.633 s. La solicitud a `(-8.5,-4,1.0)` respondio correctamente en el
cliente en 2.35 s, pero fue rechazada con `goal_occupied_or_inflated` y cero
expansiones; no demuestra todavia una ruta de desvio visual hacia un objetivo
con clearance.
