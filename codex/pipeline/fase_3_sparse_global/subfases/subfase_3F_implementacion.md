# Subfase 3F - Implementacion acordada

## 1. Identidades y resultados de cambio

La identidad de un submapa sigue siendo `(drone_id, map_epoch)`. Las claves
internas usan `RawKeyFrameId` y `RawMapPointId`; no se identifican entidades
solo por el ID local de ORB-SLAM3.

3F reutiliza los IDs exactos ya devueltos por `RawInsertResult` y
`PoseChangeSet`. Se añadiran, donde sea necesario, resultados ligeros como:

```text
ScoreChangeSet
  created_ids
  updated_ids
  invalidated_ids
  score_revision

GlobalMapDirtySet
  raw_keyframe_ids
  raw_mappoint_ids
  moved_keyframe_ids
  score_mappoint_ids
  fusion_member_or_track_ids   # reservado para fases posteriores
```

Un change set contiene IDs y revisiones, nunca una copia completa de las bases.

## 2. `LandmarkScoreManager`

### Base de datos autoritativa

Cada registro conserva al menos:

```text
RawMapPointId
score actual en [0, 1]
revision del registro
observations_count usado
found_ratio usado
descriptor_valid usado
estado is_bad
origen/ultimo evento
```

Las consultas deben permitir `GetScore`, consulta con valor por defecto
documentado, revision global y estadisticas. Ninguna clase externa escribe un
valor arbitrario directamente.

### Score inicial de 3F

ORB-SLAM3 no entrega un campo `score`; entrega las evidencias que lo producen.
La politica base sera determinista, acotada y sin GT:

```text
si is_bad:
  score = 0
si no:
  descriptor_valid = 1 si algun byte del descriptor es no nulo; 0 si no
  score = clamp(
      0.55 * min(observations_count / 8, 1)
    + 0.35 * clamp(found_ratio, 0, 1)
    + 0.10 * descriptor_valid,
    0, 1)
```

Los MapPoints `is_bad` conservan registro de score cero para trazabilidad, pero
se retiran de la vista publica.

### Orden de actualizacion

Tras el commit en `RawMapDatabase`:

1. tomar `new_mappoint_ids`, `updated_mappoint_ids` y `removed_mappoint_ids`;
2. consultar solo esos MapPoints raw;
3. crear, actualizar o invalidar sus scores;
4. devolver `ScoreChangeSet`;
5. entregar esos IDs a `GlobalMapBuilder`.

El flujo visible en web sera `RawMapDatabase -> LandmarkScoreManager` y despues
`LandmarkScoreManager -> GlobalMapBuilder`.

Eventos como fusion confirmada, reobservacion, inconsistencia u optimizacion
pueden reservarse en la API, pero en 3F tienen efecto neutro o no se invocan.

## 3. Estado incremental de `GlobalMapBuilder`

El builder es ROS-independiente y mantiene:

```text
keyframe_world_cache: RawKeyFrameId -> pose world + revision
sparse_point_cache: RawMapPointId -> punto world + score + associated_kf
keyframe_to_mappoints: RawKeyFrameId -> set<RawMapPointId>
mappoint_to_keyframe: RawMapPointId -> RawKeyFrameId
submap_to_keyframes / submap_to_mappoints
stable_point_slots + free point slots
stable_marker_ids
dirty keyframes / MapPoints / scores / fusion
ultima revision construida
```

Los slots estables permiten reemplazar o retirar elementos sin reordenar toda
la cache. La serializacion ROS puede recorrer la vista completa porque
`PointCloud2` representa el estado autoritativo completo, pero no debe volver a
transformar toda la geometria.

## 4. KF asociado a cada MapPoint

La asociacion es estable para que una actualizacion no cambie de observador sin
necesidad:

1. si existe una asociacion cacheada, el KF sigue observando el punto y posee
   pose world valida, conservarla;
2. en una insercion sin asociacion, preferir `reference_keyframe_id` de
   `OrbMapPoint` si tiene pose world valida;
3. si el KF de referencia no es publicable, elegir deterministamente entre las
   `observations` un KF con pose world valida;
4. ordenar candidatos por identidad global para que replay y live elijan igual;
5. no volver automaticamente al KF de referencia si mas tarde aparece: se
   conserva el observador elegido mientras siga siendo valido;
6. si la asociacion deja de ser valida, retirar el indice inverso anterior y
   elegir de nuevo;
7. si no existe ningun observador publicable, omitir temporalmente el punto.

## 5. Reconstruccion geometrica

Para el KF asociado:

```text
p_kf    = inverse(T_local_kf) * p_local_mp
p_world = T_world_kf * p_kf
```

`T_local_kf` y `p_local_mp` proceden de `RawMapDatabase`; `T_world_kf` procede
de `GlobalPoseStore`. No existe fallback rigido desde el frame del submapa.

Se omiten y contabilizan:

- submapas no anclados;
- MapPoints `is_bad`;
- posiciones o transforms no finitos;
- MapPoints sin KF observador world utilizable.

El contador de fallback geometrico debe permanecer siempre en cero.

## 6. Aplicacion de dirty sets

### Raw

- KF nuevo/modificado: actualizar cache KF y revisar asociaciones afectadas.
- MP nuevo/modificado: recalcular solo ese punto y sus indices.
- KF/MP eliminado o invalidado: retirar cache, slot, marcador e indices.

### Anchor y poses

- Primer anchor: marcar todos los KFs y MPs acumulados del submapa; esta es la
  unica expansion masiva legitima porque el submapa completo se hace visible.
- KF nuevo en submapa anclado: añadirlo y proyectar sus MPs afectados.
- KF movido: consultar su pose nueva en `GlobalPoseStore` y reproyectar solo
  `keyframe_to_mappoints[kf]`.

### Score

- Consultar en `LandmarkScoreManager` solo los IDs de `ScoreChangeSet`.
- Actualizar score y atributo visual sin recalcular posicion cuando no cambio
  geometria ni KF.

### Fusion futura

La API admite dirty IDs de miembros/tracks. Cuando 3P exista, el builder
consultara `FusedLandmarkManager`, retirara miembros sustituidos y actualizara
el representante. 3F no crea tracks ni comportamiento de fusion ficticio.

## 7. Salida de dominio coherente

El builder devuelve un resultado ROS-independiente:

```text
GlobalMapBuildResult
  publication_revision
  raw_revision
  pose_revision
  score_revision
  fusion_revision opcional
  timestamp
  puntos globales publicables
  poses/identidades de KFs publicables
  conteos de dirty/recalculados/omitidos
```

La nube y los KFs se materializan desde este mismo resultado. Si no existe
cambio publicable, devuelve `changed=false` y no se publican mensajes nuevos.

## 8. Adaptacion ROS en `GlobalMapServer`

### `/global_sparse_cloud`

Publicar un `sensor_msgs/msg/PointCloud2` autoritativo con `frame_id=world` y,
como minimo:

```text
x, y, z             FLOAT32
score               FLOAT32
rgb                 FLOAT32 packed, temporal para RViz2
drone_id            UINT32
map_epoch_low       UINT32
map_epoch_high      UINT32
```

`map_epoch` se divide porque `PointField` no dispone de `UINT64`. El backend
conserva la identidad completa; la codificacion ROS no modifica su semantica.

El servidor no calcula el score. Solo convierte `score` a RGB para RViz2:

```text
s = clamp(score, 0, 1)
0.0 <= s <= 0.5: rojo -> amarillo
0.5 <  s <= 1.0: amarillo -> verde
azul = 0
```

Esta conversion es presentacion temporal de Fase 3. En 7E la GUI calculara el
color desde `score` y se retirara del servidor esa responsabilidad; el campo
`score` seguira siendo dato canonico.

### `/global_keyframes`

Publicar `visualization_msgs/msg/MarkerArray` con frustums pequeños:

- pose world procedente del mismo `GlobalMapBuildResult` que la nube;
- color determinista por `(drone_id, map_epoch)`;
- todos los KFs de un submapa comparten color;
- namespace/ID estable y sin colisiones entre submapas;
- emitir `DELETE` para marcadores retirados, evitando restos en RViz2.

Ambos topics comparten timestamp y `publication_revision` correlacionable en
logs. QoS debe conservar la ultima vista para un RViz2 que se conecte tarde,
sin introducir ACK ni dependencia funcional del visualizador.

## 9. Integracion con `PrimaryWorker`

El orden de una entrada material sera:

```text
RawMapDatabase commit
-> RawInsertResult
-> LandmarkScoreManager / ScoreChangeSet
-> GlobalPoseStore y FiducialAnchorManager / PoseChangeSet
-> acumular dirty sets
-> GlobalMapBuilder actualiza caches
-> GlobalMapServer serializa PointCloud2 y MarkerArray
-> publish de ambos
-> fin de PrimaryTask
```

Un delta sin cambios publicables puede recorrer ingesta/score/pose, pero debe
terminar con `GlobalMapBuilder changed=false`, sin reconstruccion ni publish.

Los commits secundarios futuros solo insertan dirty IDs/revisiones. No llaman
al builder, no publican y no despiertan al principal. El siguiente
`PrimaryTask` drena esos dirty sets antes de construir su revision.

## 10. Grafo web

Añadir vertices:

```text
LandmarkScoreManager
GlobalMapBuilder
RViz2
```

Añadir aristas con eventos runtime reales:

```text
RawMapDatabase -> LandmarkScoreManager   score raw creado/actualizado
LandmarkScoreManager -> GlobalMapBuilder scores dirty aplicados
RawMapDatabase -> GlobalMapBuilder       geometria/asociacion dirty
GlobalPoseStore -> GlobalMapBuilder      anchor/poses dirty
GlobalMapBuilder -> GlobalMapServer      revision construida
GlobalMapServer -> RViz2                 PointCloud2
GlobalMapServer -> RViz2                 MarkerArray
```

Las dos aristas hacia RViz2 son independientes pero comparten `flow_id`,
revision y timestamp. La arista `first anchor` de 3E se conserva y se observara
otra vez durante 3F sin cambiar su logica.

## 11. Marcadores tecnicos sugeridos

Usar prefijo nuevo `F3F`, no reutilizar marcadores legacy `F1F` como contrato:

```text
[F3F-SCORE-INIT]
[F3F-SCORE-UPDATE]
[F3F-SCORE-STATS]
[F3F-BUILDER-DIRTY]
[F3F-BUILDER-UPDATE]
[F3F-BUILDER-SKIP]
[F3F-GLOBALMAP-BUILD]
[F3F-GLOBALMAP-PUBLISH]
[F3F-KF-PUBLISH]
[F3F-REVISION]
[F3F-POINT-STATS]
```

Los resúmenes deben permitir medir IDs recibidos, elementos recalculados,
cache hits, omisiones, fallback cero, revisiones y tiempo de serializacion.
