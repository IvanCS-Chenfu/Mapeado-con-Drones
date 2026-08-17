# Subfase 3P - Criterios

## Estado

```text
CRITERIOS AUTOMATICOS CONSEGUIDOS; REVISION VISUAL 161 PENDIENTE
```

## Conseguida

`3P` solo puede declararse `CONSEGUIDA` si:

1. Fusion continua dentro de una unica `LoopTask`, sin cola/worker adicional.
2. `LoopPipeline` decide y una clase especializada fusiona; no existe una
   autoridad de decision duplicada.
3. Se usan subnubes/RANSAC de 3O y todas las regiones compatibles.
4. Raws iguales y tracks ya unidos son no-op idempotente.
5. Create/add/merge son transitivos, estables, trazables y protegidos por
   dispersion.
6. RANSAC rechazado, stale y error alto no cambian score.
7. Solo inliers o contradicciones de visibilidad fiables alteran score.
8. Toda evidencia negativa elegible se procesa sin presupuesto temporal; la
   carga permanece estructuralmente acotada y puede activar backpressure.
9. Tracks, covisibilidad y score positivo se comprometen logicamente todos o
   ninguno, con rollback/inverse patch probado.
10. `RawMapDatabase` y poses no cambian.
11. El calculo costoso ocurre fuera de locks. El commit se mide pero no tiene
    objetivo, timeout ni warning contractual.
12. Changesets dirty son exactos y el siguiente principal publica un
    representante por track y todos los raws no fusionados.
13. `GlobalMapBuilder` no filtra puntos por score.
14. El worker secundario no publica, despierta al principal ni espera ACK.
15. El flujo principal progresa durante fusion y se conserva un solo worker
    secundario.
16. Todo stale/rollback de fusion termina sin estado parcial y encola un intento
    BAJA fresco, coalescido, con revisiones y `task_id` nuevos.
17. Build, tests, simulacion live, logs reducidos, RViz2 y grafo web son
    coherentes y no muestran crash, deadlock, NaN ni corrupcion.

## Umbrales cuantitativos iniciales

```text
inlier reward: +0.04
expected visible miss: -0.01
foreground contradiction: -0.03
hard outlier: >= 2 * ransac_inlier_threshold
track merge dispersion: 0.50 m
visibility temporal budget: ninguno
commit coordinator target/timeout: ninguno
```

Son parametros configurables. Ajustarlos con evidencia no invalida la subfase
si se mantienen ownership, idempotencia, atomicidad y ausencia de GT.

## Parcial

`PARCIAL` si las invariantes y tests deterministas pasan pero:

- no aparece una fusion real suficiente en live;
- falta inspeccion RViz2/web;
- la formula fused sigue provisional;
- la visibilidad completa provoca backlog/backpressure no drenado;
- stale/rollback no generan un retry fresco observable.

## No conseguida

- se crea otra tarea/worker para fusion;
- se usa GT, solo BoW o pose aproximada como aceptacion;
- un outlier o RANSAC rechazado penaliza automaticamente;
- se duplica score o soporte por replay de la misma evidencia;
- una fusion modifica raw/poses o deja bases parcialmente visibles;
- builder filtra por score o la tarea secundaria publica;
- aparece un bucle inmediato de la misma revision, retries duplicados, bloqueo
  principal, deadlock, crash, NaN o tracks corruptos. No es fallo que existan
  retries sucesivos con revisiones frescas, coalescidos y al final de BAJA.

## Documentacion tras implementar

Actualizar los MD vigentes de `loop_pipeline`, `fused_landmark_manager`,
`covisibility_database`, `landmark_score_manager`, `global_map_builder`,
`sparse_global_backend`, `global_map_server` y visualizador si cambia. Registrar
cada build/prueba real en el historial 3P sin reescribir intentos anteriores.

## Bloqueos funcionales

Si la implementacion exige mensajes nuevos, otra autoridad de estado, cambiar
backpressure, persistir evidencia de error alto o relajar la geometria de 3O,
suspender la autorizacion y acordarlo antes de continuar.
