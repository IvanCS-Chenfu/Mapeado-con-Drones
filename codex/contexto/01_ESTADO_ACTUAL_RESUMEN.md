# Estado actual - resumen

## Situacion

```text
Fase 3: ACTUAL - REIMPLEMENTACION EN CURSO
3B-3O: CONSEGUIDAS
3P: CONSEGUIDA; CIERRE FUNCIONAL Y VISUAL CONFIRMADO
3Q: PREPARACION CERRADA; IMPLEMENTACION PENDIENTE DE AUTORIZACION
```

## Runtime activo

```text
wrappers -> PrimaryQueue -> PrimaryWorker -> SparseGlobalBackend -> ROS
fiducial MAX / database MEDIA / loop BAJA -> SecondaryTaskQueue
  -> SecondaryWorker -> dominio -> pose/fusion/score/covis -> builder dirty
```

- una tarea secundaria activa no se interrumpe;
- MEDIA actualiza covisibilidad y despues crea BAJAS;
- BAJA incluye BoW, hasta tres regiones, subnubes, RANSAC y decision;
- dos queries independientes pueden anclar un submapa sin fiducial;
- 3P fusiona la rama de error bajo dentro de la misma LoopTask, reutiliza
  RANSAC, aplica score/visibilidad y commit con rollback;
- stale/rollback reencola una BAJA fresca despues de completar la anterior;
  visibilidad procesa toda evidencia elegible sin presupuesto temporal;
- el builder oculta miembros raw y publica un representante por track en el
  siguiente principal; el runtime aun termina error alto como evidencia 3Q;
- commits de anchor son atomicos y el siguiente principal hace backfill.

## Siguiente cambio acordado

3Q generalizara 3I-3L a un grafo covisible comun para fiducial absoluto y loop
relativo. Seleccionara el subgrafo minimo entre hard, tramos, dependencias soft
y constraints previas; usara dos queries, controles base 30 % ampliables,
accept completo y fusion 3P directa. No excluira loops inter/intra. La rama
loop seguira siendo BAJA, pero activara `stop_drones` hasta finalizar toda la
optimizacion/fusion.

## Evidencia

- build final 2/2; CTests funcionales 8/8 + 4/4;
- prueba 157: optimizacion del padre propaga 78 KFs del hijo blando;
- prueba 156: reanchor hard post-loop de 32 KFs y tres commits completos;
- carga reducida de 9.20 tareas/KF a 2.18; 1060 secundarias, 89 stale,
  `pending=0`, `hard_failed=0`, `max_active=1`;
- 486 poses registradas, 439 activas, cuatro anchors y siete hard; siete
  submapas raw, tres aun diferidos/no anclados al cierre;
- servidor PSS max 204.8 MiB, MemAvailable min 6134.8 MiB, PSI full 0 y guard
  inactivo;
- el usuario confirma que RViz2 y el grafo web de la prueba 156 se ven bien;
  3P/3Q validaran integralmente las ramas que 3O deja como evidencia.
- prueba 159 conservada como fallo: `map::at` por track retirado dentro del
  mismo patch; el `success=true` del runner no oculto el aborto del servidor.
- prueba 160 corregida: 62 intentos/56 commits, cinco stale, un rollback,
  1116 secundarias, `pending=0`, cero hard y servidor limpio;
- builder consume tracks en 228/383 publicaciones; la final recalcula 87 con
  `fusion_revision=56`;
- tests 9/9 + 4/4 + 1/1; MemAvailable minimo 6568.8 MiB y guardia inactiva.
- prueba 161: 27 intentos, ocho commits, 19 stale/rollback con 19 retries,
  `56/56` regiones completas, cuatro commits fiduciales full y cola vacia;
- prepare aceptado 633.852/1087.130 ms de media/maximo; pese al aumento de
  coste, escenario, backpressure, recursos y drenaje terminan correctamente.

## Referencias

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3M.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3N.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3O.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3P.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3Q.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3Q_RESUMEN.md
```
