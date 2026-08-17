# `FusedLandmarkManager`

## Rol

`FusedLandmarkManager` es la base de tracks globales creada en `3P`. Conserva
equivalencias entre MapPoints raw sin borrar ni modificar `RawMapDatabase`.

La identidad de cada miembro es:

```text
RawMapPointId = (drone_id, map_epoch, local_mp_id)
```

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/fused_landmark_track.hpp
orbslam3_multi/include/orbslam3_multi/fused_landmark_manager.hpp
orbslam3_multi/src/fused_landmark_manager.cpp
orbslam3_multi/src/test_fused_landmark_manager.cpp
```

## Modelo

`FusedLandmarkTrack` conserva miembros, KFs observadores, drones, submapas,
posición world cacheada, descriptor medoid, score, confianza, soporte y
revisión. Existen dos índices:

```text
track_id -> FusedLandmarkTrack
RawMapPointId -> track_id
```

El segundo es `unordered_map` y permite a `GlobalMapBuilder` resolver en O(1)
si un punto raw debe omitirse.

## Unión

`FuseInlierPairs` recibe exclusivamente correspondencias inlier confirmadas por
`SubcloudLoopVerifier`. Valida existencia, IDs distintos, `is_bad=false`,
posición finita y descriptor válido.

Casos:

- dos miembros libres: crea track;
- uno libre: lo añade al track existente;
- ambos en el mismo track: refuerza soporte;
- tracks distintos: conserva el ID menor y combina ambos.

La secuencia `A=B`, `B=C` produce un único track `{A,B,C}`. Los eventos
explícitos y el orden estable permiten reconstrucción determinista en replay.

## Descriptor y score

El descriptor es el miembro que minimiza la suma de distancias Hamming. En
empate se usa mayor score raw y después menor `RawMapPointId`.

El score queda en `[0,1]`: parte del mejor score raw y añade bonus acotados por
miembros, drones y submapas independientes. No penaliza puntos unmatched.

La posición world se actualiza durante la construcción de la nube como media
ponderada por score raw y número de observaciones.

## Restricciones

- no modifica MapPoints raw;
- no modifica poses;
- no hace matching ni RANSAC;
- no fusiona pares fuera de los inliers entregados por 3O;
- no crea grafos ni tareas de optimización.
