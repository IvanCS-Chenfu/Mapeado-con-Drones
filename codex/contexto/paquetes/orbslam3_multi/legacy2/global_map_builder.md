# `GlobalMapBuilder`

## Rol

`GlobalMapBuilder` es la capa creada en `3F` para construir puntos sparse globales publicables.

Consulta:

- `RawMapDatabase`;
- `GlobalPoseStore`;
- `LandmarkScoreManager`.
- `FusedLandmarkManager` opcional desde `3P`.

Devuelve puntos en frame `world` solo para submapas que tienen `world_T_local` registrado en `GlobalPoseStore`.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/global_sparse_point.hpp
orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp
orbslam3_multi/src/global_map_builder.cpp
```

## Salida

El tipo principal de salida es `GlobalSparsePoint`.

Campos relevantes:

- `global_mappoint_id`;
- `drone_id`;
- `map_epoch`;
- `local_mappoint_id`;
- `x`, `y`, `z` en `world`;
- `score`;
- `observations`;
- `from_anchored_submap`;
- `is_fused`.

Desde `3P`, `is_fused=true` identifica el representante publicado de un track.

## Comportamiento

- Recorre submapas raw de `RawMapDatabase`.
- Pide `world_T_local` a `GlobalPoseStore`.
- Salta submapas no anclados.
- Salta MapPoints `is_bad`.
- Salta posiciones locales no finitas.
- Pide score a `LandmarkScoreManager`.
- Consulta en O(1) si el MapPoint raw pertenece a un track fusionado.
- Aplica `min_score_to_publish` si el servidor lo configura.
- Desde la revisión visual posterior a `prueba_69`, publica cada MapPoint
  exclusivamente desde un KeyFrame utilizable con pose world:
  - prefiere el KF de referencia si tiene pose válida en `GlobalPoseStore`;
  - en caso contrario elige un observador con pose world válida, priorizando
    los corregidos por servidor;
  - calcula `p_kf = raw_local_T_kf^-1 * p_local_raw`;
  - publica `p_world = final_world_T_kf * p_kf`.
- Si no hay KF publicable:
  - omite siempre el punto y lo contabiliza como
    `server_corrected_missing_keyframe_skipped` e `invalid_pose_skipped`;
  - no existe ya el fallback rígido `world_T_local * p_local_raw`.
- `fallback_submap_points` se conserva como métrica de regresión y debe ser
  siempre cero. Así no puede aparecer una nube sin un KF world coherente tras
  un apply de optimización.
- `GetSubmapLastServerCorrection` no se usa como veto global para prohibir el
  fallback de todo un submapa. Tampoco debe ser la autoridad futura para KFs de
  cola en trayectorias fiduciales largas; esa ruta debe pasar a
  `active_tail_anchor` en `GlobalPoseStore`.
- Contabiliza `server_corrected_points`,
  `server_corrected_mappoint_candidates`,
  `server_corrected_missing_keyframe_skipped`,
  `keyframe_projected_points` y `fallback_submap_points`.
- Para miembros fusionados:
  - proyecta cada miembro con la misma política de KF que los raw;
  - acumula su posición con peso `score_raw * max(observations, 1)`;
  - omite el miembro raw;
  - publica un único `GlobalSparsePoint` por track;
  - actualiza la posición cacheada del track.

Esta política pertenece conceptualmente al apply/publicación de `3K` y busca que
RViz2 siga la misma deformación de KeyFrames que se ve en el HTML diagnóstico
del grafo. Ese HTML pertenece a `3J`, aunque algunos artefactos conserven
prefijo legacy `f1l_*`.

Validación final del 2026-07-28: en `prueba_41` la publicación termina con
`invalid_pose_skipped=0` y
`server_corrected_missing_keyframe_skipped=0`. El usuario confirma en RViz2
que ambos applies, los KFs posteriores y sus MapPoints se ven correctamente,
sin recuperar poses raw anteriores. La ruta de publicación de `3K` queda
conseguida.

## Logs

Los logs los emite el servidor al publicar el resultado:

```text
[F1F-GLOBALMAP-BUILD]
[F1F-GLOBALMAP-SKIP-UNANCHORED]
[F1F-GLOBALMAP-POINT-STATS]
[F1F-GLOBALMAP-PUBLISH]
[F1P-GLOBALMAP-FUSED-BUILD]
[F1P-FUSED-POSITION-UPDATE]
[F1P-GLOBALMAP-FUSED-PUBLISH]
```

Desde la validación post-apply, el servidor añade un resumen diagnóstico:

```text
[F1L-GLOBALMAP-KF-PROJECTION]
```

En una optimización aceptada, el valor esperado es que
`invalid_pose_skipped` no aumente y que
`server_corrected_missing_keyframe_skipped=0` salvo causa documentada.
`fallback_submap_after` debe ser cero. Los MapPoints sin KF world utilizable se
omiten hasta que exista una asociación publicable.

Evidencia validada en `3F`:

```text
[F1F-GLOBALMAP-BUILD] ... anchored_submaps=2 skipped_unanchored=1 raw_points=22394 candidate_points=22394 returned_points=22394
[F1F-GLOBALMAP-PUBLISH] ... topic=/global_sparse_cloud frame_id=world points_published=22394 ... score_field=true drone_id_field=true map_epoch_field=true
```

Desde `3K`, `[F1F-GLOBALMAP-POINT-STATS]` incluye
`server_corrected_points`, `server_corrected_mappoint_candidates` y
`server_corrected_missing_keyframe_skipped`. En la validacion del 2026-07-23:

- `prueba_31` acepta dos optimizaciones fiduciales de la prueba típica corta;
- `task_id=1` publica `server_corrected_points=27180`,
  `server_corrected_mappoint_candidates=27180`,
  `server_corrected_missing_keyframe_skipped=0` e
  `invalid_pose_skipped_after=0`;
- `task_id=2` mantiene `published_points_before=37415` y
  `published_points_after=37415`, con `invalid_pose_skipped 0 -> 0`;
- `[F1K-GLOBALMAP-PUBLISH-AFTER-APPLY]` aparece tras cada apply aceptado.

## Restricciones

- No crear anchors.
- No modificar `RawMapDatabase`.
- No modificar `GlobalPoseStore`.
- No calcular score por cuenta propia.
- No publicar submapas sin anchor.
- No implementar fusión real en `3F`.
- Desde `3P`, no publicar simultáneamente un track y sus miembros raw.
- Desde `3K`, reconstruir los puntos publicados desde el KF elegido y su pose
  final; no modificar raw, scores ni anchors.
- No usar la transformación rígida del submapa como sustituto de un KF world
  utilizable; `fallback_submap_points` es una alarma de regresión y vale cero.
- Desde `3K`, no usar `GetSubmapLastServerCorrection` para declarar corregido
  todo un submapa a efectos de publicación de MapPoints; la decisión se toma
  por cobertura de KFs corregidos.

Validación `3P` en `prueba_48`: con `16259` puntos raw, el estado final omite
`3333` miembros y publica `674` tracks, dando exactamente `13600` puntos
(`16259 - 3333 + 674`). La revisión visual en RViz2 sigue pendiente.
