# `LoopPairAttemptDatabase`

## Rol

`LoopPairAttemptDatabase` es la memoria por etapa de pares KF-KF introducida en
el rediseño de rendimiento de `3P`.

Su clave es el par canonico de `RawKeyFrameId`; el valor conserva las
`RawKeyFrameRevision` de ambos extremos, estado, `arrival_id`, razón y, para
confirmaciones, el resultado geométrico.
La cache es independiente de `CovisibilityDatabase`: una representa intentos
negativos para una version concreta y la otra conserva relaciones positivas
confirmadas.

## Archivos

```text
orbslam3_multi/include/orbslam3_multi/loop_pair_attempt_database.hpp
orbslam3_multi/src/loop_pair_attempt_database.cpp
orbslam3_multi/src/test_loop_pair_attempt_database.cpp
```

## Politica

- Canoniza el orden de los dos KFs para que `A-B` y `B-A` sean la misma clave.
- Modela `BOW_REJECTED`, `DEFERRED_NO_POSE`, `PENDING_GEOMETRY`,
  `GEOMETRY_REJECTED`, confirmaciones, `OPTIMIZATION_PENDING`,
  `FUSION_PENDING` y `FUSED`.
- BoW se reutiliza por `appearance`; geometría y estados terminales se
  reutilizan por revisiones de pose/asociaciones/apariencia/geometría.
- Una revisión relevante distinta permite un nuevo intento; metadata no
  invalida BoW ni geometría.
- Falta de pose, falta de anchor, estado incompleto, resultado stale o trabajo
  cancelado por una arista confirmada son temporales/diferibles y no se
  cachean.
- Una confirmacion positiva borra cualquier rechazo previo del par.

## API principal

- `ShouldSkipBow`, `ShouldSkipDefinitiveReject` y
  `ShouldSkipTerminalState`: consultan la etapa y revisiones pertinentes.
- `RecordState`: persiste estados temporales, definitivos o terminales.
- `RecordBowRejected`: registra un rechazo BoW explícitamente puntuado.
- `RecordDefinitiveReject`: guarda o actualiza un rechazo cacheable.
- `RecordNonCacheable`: contabiliza un rechazo temporal que no debe persistir.
- `Erase`: elimina la entrada canonica cuando el par se confirma.
- `GetStats`: expone consultas, hits, misses, stores, invalidaciones por
  revision y rechazos no cacheables.

## Integracion

`global_map_server` consulta la cache antes de admitir fallback BoW/worker y
registra el resultado despues de validar el commit contra revisiones y poses.
Los marcadores principales son:

```text
[F1P-NEGATIVE-CACHE-HIT]
[F1P-NEGATIVE-CACHE-MISS]
[F1P-NEGATIVE-CACHE-STORE]
[F1P-NEGATIVE-CACHE-NO-STORE]
```

## Validacion y limite vigente

`test_loop_pair_attempt_database` pasa tras verificar canonizacion, hit por
revision, invalidacion al cambiar una revision y separacion de rechazos
temporales.

La corrección posterior a `prueba_61` elimina `submap_geometry`: un stale se
descarta sin retry inmediato y solo una revisión material coalescida puede
liberar un trabajo posterior para el mismo par.

En `prueba_62` se almacenan `142` rechazos geométricos definitivos y no hay hits
porque los pares/revisiones no reaparecen. Se descartan `97` resultados stale
sin retry inmediato y solo se observa una revisión coalescida. Los `112` loops
de error alto quedan como `OPTIMIZATION_PENDING` sin crear tareas 3Q.
