# Subfase 6D - Mapa voxel global y evidencia reversible

## Estado

```text
PARCIAL: OCCUPIED reversible y el filtro de cuatro MapPoints cualificados
distintos por voxel estan implementados y validados. Se habilita ahora el tramo
de 6I que materializa FREE de trayectoria ligado a keyframes; depth metrico
real y una resincronizacion global ejercida quedan para Fase 8.
```

## Dependencia

6A/6C y fronteras publicas reales de mapa/pose de Fases 3-5.

## Objetivo tecnico

Construir en `task_lib` un mapa voxel navegable incremental que se corrija al
mover/eliminar KFs, MPs o depth asociado, sin acceder a `orbslam3_multi`.

## Modelo

Separar estrictamente `occupancy/free evidence` de `coverage evidence`.

- MapPoint: indicio debil de superficie/ocupacion, ponderable por score y por
  soporte local. Solo contribuye a `OCCUPIED` si `score >=
  min_occupied_map_point_score` y su voxel reune al menos
  `min_occupied_mappoints_per_voxel` identidades de MapPoint distintas que
  tambien superan ese score. Los valores iniciales de mision son `0.2` y `4`.
  Una republicacion del mismo MapPoint no incrementa el soporte. Un MapPoint
  de score menor, o un voxel con menos de cuatro identidades cualificadas, se
  conserva como dato sparse/visual, pero no crea ni mantiene ocupacion sparse
  ni declara FREE.
- Depth endpoint: ocupacion fuerte solo si su retorno cae dentro de la banda
  fiable configurable. El valor inicial es 1--5 m, igual que la banda neutra
  del score geometrico de 3R para la calibracion estereo actual; el limite
  lejano podra derivarse del baseline como en 3R.
- Depth ray: FREE solo en el tramo dentro de esa misma banda fiable. Antes del
  minimo y despues del maximo no se infiere evidencia: permanece `UNKNOWN`.
- Trayectoria realmente estimada: FREE fuerte en el volumen fisico barrido por
  el dron, sin incluir su margen de seguridad.
- KeyFrame: referencia geometrica/coverage; nunca obstaculo por si solo.

Toda contribucion conserva procedencia `(source,drone_id,map_epoch,kf/chunk,
pose_revision)` y permite add/remove/move. No se guarda solo una probabilidad
saturada imposible de deshacer.

Depth se transforma al KF de referencia y se agrupa en `LocalVoxelSubmap[KF]`.
Si cambia `W_T_KF`, se retira la rasterizacion world anterior y se reintegra el
submapa local. El pitch actual queda incorporado en `Kref_T_C`.

La evidencia FREE de paso real se liga al `reference_keyframe_id`: antes de
tener `W_T_KF` no se acumula ni se publica barrido. Al anclarse, el KF libera
su voxel y su volumen fisico; despues, cada cruce a un voxel local nuevo anade
una huella centrada en ese voxel. Un cambio de
`GlobalKeyFramePose.pose_revision` retira y reinserta todas esas fuentes. La
evidencia de paso real prevalece sobre OCCUPIED sparse debil en la misma celda;
no convierte ausencia de datos ni la linea de vision de MapPoints en FREE.
Los rayos FREE metricos se reservan para depth real de Fase 8. Cuando se
incorporen, una muestra fuera de 1--5 m no podra inventar FREE ni OCCUPIED;
los limites seran parametros congelados por mision y auditables en telemetria.

`VoxelMapWorker` es el unico writer de evidencia raw, capa navegable derivada y
revision voxel publicada. Sus callbacks ROS no recalculan mapa: encolan cambios
netos y el worker los drena en una ventana configurable
`voxel_worker_coalesce_ms=100`. La entrada normal es
`GlobalSparseMapDelta`, emitida por `global_map_server` despues de que
`GlobalMapBuilder` haya materializado una revision coherente en `world`. Cada
delta contiene revision, `upsert` con identidad estable/pose world/score y
`delete` por identidad. La nube global completa solo sirve para bootstrap o
recuperar un salto de revisiones; los `orb_map_delta` locales no alimentan el
mapa voxel directamente porque no incorporan anclas ni optimizaciones globales.

Al drenar un lote, el worker aplica primero los acumuladores reversibles raw y
despues actualiza solo la influencia navegable de las celdas que cambiaron por
cada `footprint_profile_id`: transitabilidad, mascara de conexiones 26-validas
y costes base. Mantiene un indice `MapPointId -> voxel cualificado` y, por cada
voxel, el conjunto o contador reversible de identidades cualificadas. Un
`upsert` repetido de la misma identidad es idempotente; un movimiento, una
bajada bajo el score, un `delete` o una retirada de score elimina antes su
contribucion anterior. Cruzar el umbral de cuatro identidades es lo que anade o
retira la ocupacion sparse de la celda, salvo que otra fuente de evidencia
mantenga el estado resultante. Cada `OCCUPIED` mantiene de forma incremental
que celdas navegables infladas bloquea; no se vuelve a recorrer la caja completa
de seguridad para cada celda afectada. La actualizacion incluye las celdas de
aristas vecinas y la guia gruesa tocada, pero no materializa 26 objetos de
arista por voxel. Publica un snapshot inmutable y un delta con la misma
revision; los cambios llegados durante el commit quedan para el lote siguiente.

Las poses de paso se comprimen antes de entrar al worker a centros de voxel
locales unicos por `(drone_id,map_epoch,reference_keyframe_id)`. Al drenar,
si existe `W_T_KF`, el worker los transforma a `world`, marca `FREE` el volumen
fisico del dron sin margen y elimina las muestras crudas consumidas. Conserva la
evidencia local compacta para retirarla/reintegrarla si ese KF recibe otra
`pose_revision`; si aun no hay pose global, la conserva pendiente. Esta capa no
calcula coverage ni completa tareas: entrega snapshots y deltas a
`PlanningWorker` y al analisis topologico periodico de 6H.

El margen de navegacion se expresa en voxeles adicionales:
`extra_obstacle_clearance_voxels=2`. Para cada eje, la inflacion es
`ceil(semidimension_fisica/voxel_size) + extra_obstacle_clearance_voxels`.
El margen adicional se mide desde la envolvente fisica, no desde el centro del
dron. Se elimina `coverage_min_obstacle_clearance_m`; con `voxel_size` fijo
por mision la nueva semantica es determinista y visible en el perfil registrado.

Cada perfil mantiene ademas una guia jerarquica derivada. El parametro entero
`coarse_voxel_factor` expresa el lado del macro-voxel en unidades de voxel fino:
con `voxel_size=0.25 m` y factor `4`, una celda gruesa representa un cubo de
`4x4x4` (`1 m` por eje). Esta capa resume transitabilidad y coste orientativo
por macro-voxel y se actualiza solo en los macro-voxeles tocados por el delta
fino. No es una autoridad de seguridad ni una segunda clasificacion raw.

## Cambios requeridos

1. Implementar `VoxelMapWorker` como unico writer, cola coalescente de 100 ms
   configurable y snapshots de lectura por revision.
2. Definir acumuladores reversibles y fuentes identificables para `upsert` y
   `delete` de `GlobalSparseMapDelta`.
3. Emitir desde `GlobalMapBuilder` un delta world incremental con continuidad de
   revision; usar snapshot completo solo para bootstrap o resync.
4. Integrar add/remove/move de MP y KF sin acoplamiento interno.
5. Preparar depth chunks ligados a KF y FREE de trayectoria real, con volumen
   fisico configurable sin margen y reintegracion al corregirse cada KF. El
   depth futuro aplicara endpoint/rayo unicamente dentro de su banda fiable
   configurable, inicialmente 1--5 m segun 3R.
6. Emitir `MapChangeEvent` con revision, chunks/voxels, AABB y tipo de cambio.
7. Congelar `voxel_size` por mision desde handshake; medir su valor.
8. Alimentar grafo con queue/revision/changed voxels sin eventos por voxel.
9. Mantener una capa navegable incremental por perfil fisico registrado mediante
   cuentas de influencia de ocupacion; los drones iguales comparten perfil y
   los distintos no comparten inflacion.
10. Mantener la guia gruesa por el mismo perfil, con factor entero configurable
    y deltas locales; nunca usarla para publicar bloqueo, FREE u OCCUPIED.
11. Congelar por mision y publicar en telemetria
    `min_occupied_map_point_score`, inicialmente `0.2`, y
    `min_occupied_mappoints_per_voxel`, inicialmente `4`. Solo identidades
    distintas que cumplan ambos filtros participan en occupancy y en las
    barreras usadas por coverage; altas repetidas no cuentan dos veces y toda
    retirada, movimiento o cambio de score revierte su contribucion exacta.
12. Construir el perfil navegable con semidimensiones fisicas mas
    `extra_obstacle_clearance_voxels=2`; no usar un standoff blando de
    seleccion de meta como sustituto de seguridad de corredor.

## Limites

No usar ausencia de MPs ni score bajo como FREE, no usar KF como obstacle ni
consumir GT para el mapa final. En simulacion GT solo puede gobernar la pose canonica de
navegacion que alimenta la misma evidencia ligada a KF; en real la fuente es
ORB anclada. El filtrado de otros drones observados por depth queda como mejora
futura salvo evidencia que lo haga imprescindible.

## Revision futura coordinada con 3R

El `score` de un MapPoint y la evidencia de ocupacion de un voxel no deben ser
la misma magnitud ni compartir definitivamente el mismo umbral. El primero es
la confianza individual publicada por `LandmarkScoreManager` en 3R; la segunda
debera ser una evidencia espacial agregada, mantenida por `VoxelMapWorker` al
sumar las contribuciones de MapPoints identificados contenidos en el voxel.

En una futura revision conjunta de 3R y 6D se sustituira el filtro transitorio
`score >= min_occupied_map_point_score` mas
`min_occupied_mappoints_per_voxel` por un acumulador reversible de
`voxel_occupancy_score` y un parametro de mision
`min_voxel_occupancy_score`. Cada identidad contribuira una sola vez; un
movimiento, retirada o cambio de score restara primero su contribucion anterior.
El voxel sera `OCCUPIED` solo si la evidencia agregada supera el umbral. La
formula concreta, posibles pesos por calidad/fusion y sus valores iniciales se
decidiran al recalibrar 3R, no se infieren de GT ni se implantan ahora.

La migracion debe ser atomica entre ambas fases, con regresiones de suma,
repeticion idempotente, movimiento, retirada y variacion de score. Hasta
entonces los parametros `0.2` y `4` siguen siendo el comportamiento vigente y
no deben coexistir como una segunda barrera con el futuro score de voxel.

La futura señal de 3R `MapPoint corresponde al cuerpo de otro dron` obliga a
retirar su contribucion del acumulador voxel: el MP publica score `0` de forma
reversible y `VoxelMapWorker` resta exactamente su aporte anterior. Asi un
voxel no queda `OCCUPIED` por la presencia temporal de otro dron. Esta regla no
se aplica por proximidad simple a un KF ajeno ni se implementa en la iteracion
actual; requiere asociacion fiable y regresiones coordinadas con 3R.

## Pruebas

- Add/remove/move por fuente y varias contribuciones en un voxel.
- Depth ray/endpoint, pitch y reintegracion al mover KF, incluyendo retornos
  por debajo, dentro y por encima de la banda fiable inicial 1--5 m: solo el
  intervalo valido cambia FREE/OCCUPIED.
- Culling/reset/epoch y evidencia FREE realmente recorrida.
- Umbral sparse y soporte: score `0.2` incluido puede contribuir; score menor
  no cambia occupancy aunque el MapPoint siga disponible para
  visualizacion/diagnostico. Tres identidades cualificadas en un voxel no crean
  ocupacion sparse; la cuarta la crea, y eliminar, mover o descalificar una de
  ellas debe retirarla de nuevo si no existe otra evidencia dominante.
- Separacion occupancy/coverage y lista exacta de voxeles afectados.
- Delta global `upsert/delete`, bootstrap y resync tras salto de revision.
- Coalescencia de 100 ms, contribuciones FREE comprimidas y reintegracion tras
  optimizacion de KF, sin perder ni duplicar fuentes.
- Rendimiento/memoria multi-dron: medir cola, cambios netos, influencia
  navegable, publicacion y latencia de cada commit.

## Criterio de exito

No quedan fantasmas tras correcciones, las fuentes se deshacen aisladamente,
coverage no altera occupancy y cada cambio produce una revision incremental.

## Ejecucion vigente

La implementacion vigente conserva contribuciones identificadas y sustituye
`OCCUPIED` al recibir `/global_sparse_cloud`; `FREE` se mantiene como fuente
independiente y reversible por KF. Es una solucion transitoria: aun aplica
snapshots completos y llama a la actualizacion navegable desde callbacks. El
contrato aprobado sustituye esa entrada por `GlobalSparseMapDelta` y concentra
los commits en `VoxelMapWorker`. Publica `/mission/voxel_map` transient-local
con revision y celulas `UNKNOWN`/`FREE`/`OCCUPIED`. La prueba 605 valido
snapshots reversibles reales de sparse y su consumo en GUI.
La capa visual no dibuja retícula espacial adicional: representa solo las
celulas `OCCUPIED`/`FREE` solicitadas, preservando el rendimiento de la GUI.
`NavigationSnapshotFor(profile)` y `RefreshNavigation(profile, raw_changes)`
son la frontera de lectura: el primero entrega una revision inmutable y el
segundo un delta `before/after` con numero de celdas recalculadas y tiempo de
actualizacion. Las tablas de navegacion se comparten por perfil igual, no por
dron; la evidencia raw continua teniendo una unica autoridad.

El soporte sparse vigente mantiene dos indices locales: identidad hacia voxel y
voxel hacia identidades cualificadas. `ApplySparseSnapshot` y
`ApplySparseDelta` reciben el score minimo y el minimo de identidades por
voxel; con los valores de mision `0.2` y `4`, almacenan candidatos aunque aun
no materialicen `OCCUPIED`. Al alcanzar cuatro fuentes distintas se incorpora
la contribucion agregada; una republicacion identica no cambia revision, y un
move, delete o descenso de score la retira de forma reversible. La regresion de
`task_lib` cubre tres fuentes sin ocupacion, cuarta fuente, republicacion
idempotente y retirada por movimiento. La prueba 678 propago el valor `4` al
servidor y mantuvo la ejecucion GT de coverage sin error de escenario.

## Dependencias abiertas

La politica ante desaparicion definitiva del KF se decide tras auditar F5:
identidad historica o reparentado. No inventarla durante preparacion.
