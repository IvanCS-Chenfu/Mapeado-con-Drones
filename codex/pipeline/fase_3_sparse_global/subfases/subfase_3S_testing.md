# Subfase 3S - Pruebas

## Estado vigente

```text
REHACER
```

## Pruebas nuevas obligatorias

1. `RawScoreBaseUsesPrimaryChangeSet` actualiza solo MPs afectados sin esperar
   al worker secundario.
2. `ScoreChangeSetIsExact` solo contiene MPs/tracks realmente afectados.
3. `NoFullScoreSnapshotForPublication` valida acceso incremental.
4. `RejectedPatchLeavesNoScore` cubre stale/reject de fusión.
5. `ScoreUpdateDoesNotBlockPrimaryPublish` mantiene progreso observable.
6. Un calculo derivado opcional reutiliza `DatabaseUpdateTask` MEDIA y no crea
   otra prioridad o cola.

## Build

```text
orbslam3_multi
orbslam3_server
simulacion_dron si cambia integracion
```

## Tests locales

1. Un `RAW_CREATED` crea score base una vez.
2. Cambio de metadata no material es no-op.
3. `RAW_MARKED_BAD` aplica la politica central.
4. Clases externas no pueden modificar score directamente.
5. Fusion aceptada actualiza track y score en revision comun.
6. Fusion rechazada/stale no deja eventos ni score.
7. Fused score es determinista ante orden distinto de miembros.
8. Un score candidate descartado no necesita rollback live.
9. Snapshot de score es coherente durante commit concurrente.
10. `GlobalMapBuilder` usa score sin escribirlo y no espera al worker.
11. RANSAC rechazado, stale y error alto no crean eventos numericos.
12. Inlier aplica `+0.04` a ambos MPs una sola vez.
13. Un outlier solo penaliza al MP con contradiccion de visibilidad fiable.
14. Agotar el presupuesto conserva positivos y omite negativos incompletos.
15. El builder publica el mismo numero de entidades geometricas para score alto
    o bajo; solo cambia el campo score.

## Integracion

En la prueba larga verificar actualizaciones raw progresivas, al menos una
fusion con score y publicacion posterior. Mientras una `LoopTask` esta activa,
los MPs raw nuevos deben obtener score base y poder publicarse.

## Evidencia

- revisiones y entidades afectadas;
- razones de eventos;
- score min/mean/max y conteos acotados;
- tracks/MPs publicados y sustituciones debidas solo a fusion, nunca a score;
- tiempo bajo lock;
- cero eventos derivados de tareas rechazadas;
- cero GT y cero worker propio.

Usar logs reducidos. Las metricas deciden la validez numerica, pero grafo y
RViz2 son obligatorios para cerrar la integracion; sin inspeccion visual la
conclusion es `PARCIAL`.
