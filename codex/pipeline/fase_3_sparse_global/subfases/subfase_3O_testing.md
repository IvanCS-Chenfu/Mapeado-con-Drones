# Subfases 3M-3O - Plan de pruebas

## Estado

```text
PRUEBAS EJECUTADAS; RESULTADO TECNICO CONSEGUIDO EL 2026-08-15
```

La validacion combina unit tests deterministas, replay y una prueba live. No se
declarara `CONSEGUIDA` solo porque compile o porque RANSAC produzca algun accept.

## Build

```bash
./codex/herramientas/build_selected_packages.sh \
  orbslam3_multi orbslam3_server simulacion_dron
```

Si falla, se reducira el log con `reduce_build_log.sh` y solo se leera el log
reducido. Se corregira el primer error real antes de repetir.

## Unit tests 3M

### CovisibilityDatabase

1. `CanonicalPair`: `A-B` y `B-A` producen una arista.
2. `RejectSelfAndNonFinite`: self-edge y SE3 no finita se rechazan.
3. `AffectedOnlyImport`: un patch consulta/modifica solo KFs afectados.
4. `EquivalentSnapshotIdempotent`: delta/snapshot equivalente no incrementa
   revision ni duplica aristas.
5. `MeasuredPoseImmutable`: una actualizacion no sobrescribe la medida original.
6. `CurrentPoseMutable`: una actualizacion autorizada cambia solo current.
7. `SourcesSeparated`: ORB y server conservan origen/estadisticas.
8. `BoundedImmutableView`: un lector conserva snapshot estable durante commit.

### Cola y DatabaseUpdateTask

1. `PriorityOrder`: MAXIMA > MEDIA > BAJA.
2. `FifoWithinPriority`: FIFO dentro de cada clase.
3. `ActiveTaskNotPreempted`: una tarea activa termina antes de otra.
4. `OneUpdatePerChangeSet`: un `ChangeSet` crea como maximo una MEDIA.
5. `UpdateEnqueuesLoops`: tras commit encola una BAJA por KF elegible.
6. `DirectLoopWithoutCovisChange`: un KF sin patch recibe BAJA directa.
7. `EquivalentSnapshotCreatesNoTask`: snapshot equivalente no mete ruido.

## Unit tests 3N

### LoopBoWIndex

1. upsert de un KF y consulta por words compartidas;
2. reemplazo por `appearance_revision`;
3. eliminacion/invalidation de postings antiguos;
4. reconstruccion desde raw equivalente al estado incremental;
5. exclusión de self y KFs bad/incompletos;
6. similitud coseno y ranking deterministas.

### Regiones candidatas

1. una sola busqueda devuelve varios candidatos;
2. KFs unidos por covisibilidad se agrupan en una region;
3. el mejor score es seed de su region;
4. se entregan inicialmente como maximo tres regiones;
5. una arista ORB nativa ayuda a agrupar pero no confirma fusion;
6. una arista server resuelta evita geometria repetida;
7. candidatos anclados y no anclados se conservan;
8. query/candidate invertidos producen el mismo par canonico.

### Ledger y cache

1. una revision solo ocupa un estado pendiente/activo;
2. snapshot equivalente se coalesce;
3. cambio durante `ACTIVE` crea como maximo un `DIRTY_AFTER_RUN`;
4. `STALE` no reintenta inmediatamente;
5. rechazo definitivo solo hace hit con las mismas revisiones;
6. ambiguedad y falta de anchor no son rechazo definitivo;
7. un cambio de anchor permite nueva pasada de los KFs afectados.
8. refinamientos de posiciones/descriptores cambian `validation_revision` pero
   no la huella de scheduling;
9. cruzar `min_query_mappoints` o el estado denso si habilita una nueva pasada;
10. un cambio exacto durante compute produce `STALE` antes de cualquier commit.

## Unit tests 3O

### Subnubes y geometria

1. `BoundedGeometryInput`: no hay copia global de bases.
2. `QueryObservedPointsOnly`: query usa sus MPs observados.
3. `CandidateWindowNotSeedOnly`: candidate usa region/vecinos acotados.
4. `DescriptorsAndIdsDeduplicated`: no hay puntos duplicados.
5. `RobustReductionFallback`: percentiles degenerados usan fallback.
6. `DeterministicRansac`: mismo input produce mismo resultado.
7. `RejectDegenerateGeometry`: alineacion/cobertura insuficiente rechaza.
8. `AllRegionsAccounted`: cada region tiene resultado y motivo.
9. `VerifierDoesNotMutateLiveState`: la geometria pura no escribe bases.

### Rama rapida y decision

1. `FastOverlapRequiresWorldPose`;
2. `FastOverlapRequiresDistributedMatches`;
3. `FastFusionSkipsBow`;
4. `FusionDominatesOptimization`;
5. `AllCompatibleFusionPairsKept`;
6. `NoFusionCallsIn3O`;
7. `NoOptimizationCallsIn3O`;
8. `NoServerCovisibilityCommitIn3O`.

### Consistencia y ambiguedad

1. una sola query deja anchor/optimizacion pendiente;
2. dos queries con baseline menor no cuentan como independientes;
3. `0.20 m` o `5 grados` habilitan independencia inicial;
4. dos transformaciones compatibles activan hipotesis;
5. dos regiones incompatibles producen `DEFERRED_AMBIGUOUS`;
6. la ganadora necesita margen de dos observaciones;
7. ambiguedad no se cachea como rechazo definitivo;
8. una fusion valida suprime una hipotesis de optimizacion repetitiva.

### Anchor y componentes

1. `QueryUnanchoredCandidateAnchored`: se ancla query.
2. `QueryAnchoredCandidateUnanchored`: se ancla candidate.
3. `BothUnanchoredCreatesProvisionalConstraint`.
4. una segunda query coherente activa la constraint.
5. un componente sin raiz world no crea poses globales.
6. al anclarse la raiz se anclan todos los nodos conectados.
7. un ciclo coherente valida y uno incompatible bloquea.
8. el commit de componente es atomico.
9. un conflicto de revision no deja poses parciales.
10. KFs llegados durante RANSAC se incluyen en el batch final.
11. raw permanece bit a bit/logicamente inalterada.
12. todos los KFs creados/movidos se notifican dirty.
13. se encola una pasada coalescida por nueva `anchor_revision`.
14. anchor commit no activa `optimization_active`.

### Fiducial posterior

1. mover el KF de apoyo del padre aplica el mismo delta rigido a todos los KFs
   del hijo blando;
2. la propagacion conserva las poses relativas internas del hijo;
3. el commit es atomico y el siguiente principal recalcula los MPs dirty;
4. el primer fiducial directo reancla todo el hijo como first anchor tanto con
   error alto como con error bajo;
5. ese KF fiducial queda hard y `last_accepted_control_kf`;
6. tras ese fiducial el hijo deja de seguir rigidamente al padre;
7. mover posteriormente el padre no mueve al hijo hard;
8. un segundo fiducial del hijo usa el flujo ordinario 3H-3L;
9. la constraint loop permanece disponible para la optimizacion covisible 3Q,
   sin autoridad live sobre el hijo hard en el runtime transitorio 3O.

La regresion 3O conserva estos asserts historicos. Las nuevas pruebas 3Q deben
demostrar la politica sustituta de promocion/optimizacion conjunta sin reanchor
rigido previo.

## Prueba de concurrencia controlada

Se introducira un delay de test en matching/RANSAC. Durante el delay:

- el PrimaryWorker debe comprometer KFs nuevos;
- el query/component no puede observar un lote parcial;
- al cerrar el anchor, los KFs tardios deben incluirse;
- `active_secondary_workers` nunca supera uno;
- una fiducial MAXIMA que llegue espera a la tarea activa y despues adelanta a
  MEDIAS/BAJAS pendientes;
- el visualizador mantiene el flujo activo sin parpadeo por subetapas.

## Replay determinista

Dataset inicial:

```text
codex/archivos_auxiliares/repeticiones/rawdb_prueba_151.record
```

Objetivos:

1. verificar que dos replays generan el mismo ranking, regiones y RANSAC;
2. contar tareas MEDIA/BAJA y confirmar ausencia de duplicados por snapshot;
3. comprobar cache hits/misses por revisiones;
4. medir tiempos, memoria y backlog;
5. confirmar que hasta tres regiones quedan contabilizadas;
6. comprobar que 3O no fusiona ni optimiza;
7. ejecutar un replay sintetico/unit-integrado de componente `A -> B -> C` y
   validar cascada atomica;
8. analizar un caso ambiguo con regiones repetitivas sin apply incorrecto.

El replay puede requerir fixtures derivados/acotados si el record 151 no
contiene de forma natural una cascada o ambiguedad. Esos fixtures no usaran GT
para decidir; solo fijaran entradas y resultados esperados.

## Prueba live obligatoria

Se creara una trayectoria nueva con este comportamiento:

```text
dron A
  -> pasa por fiducial
  -> queda anclado
  -> observa una zona compartida

dron B
  -> evita todos los fiduciales
  -> observa la misma zona desde al menos dos KFs independientes
  -> confirma geometria con A
  -> ancla todo su submapa por loop
```

La prueba se ejecutara exclusivamente mediante:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba <id_nuevo> \
  --yaml /home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/<trayectoria_loop_anchor>.yaml \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

Evidencia automatica esperada:

- A anclado por fiducial y B inicialmente sin world;
- BoW unico con regiones candidatas;
- dos queries independientes y transformaciones compatibles;
- constraint activa y commit atomico de B;
- todos los KFs actuales de B registrados en `GlobalPoseStore`;
- IDs dirty enviados a `GlobalMapBuilder`;
- siguiente `PrimaryInput` publica B en RViz2;
- ningun apply de fusion/optimizacion por loop;
- flujo principal progresa durante la tarea;
- backpressure coherente y liberado al bajar del low watermark.

Revision humana:

- RViz2 muestra A y B en una colocacion coherente y con color por submapa;
- B no aparece globalmente antes del anchor;
- el grafo web muestra MEDIA, BAJA, BoW, regiones, RANSAC, constraint, commit y
  pose dirty con lifecycle continuo;
- el usuario comunica la observacion visual antes del cierre final.

## Reduccion de logs

Los logs completos nunca se leen directamente. Se reduciran con patrones como:

```text
F3M-DB-UPDATE|F3M-COVIS-PATCH|F3N-LOOP|F3N-BOW|F3N-REGION|
F3O-FAST|F3O-SUBCLOUD|F3O-RANSAC|F3O-DECISION|F3O-HYPOTHESIS|
F3O-ANCHOR|F3O-COMPONENT|POSE-DIRTY|BACKPRESSURE|PRIMARY-(START|END)
```

Cada ejecucion conserva su log completo como artefacto, su reducido, evidencia
y conclusion propia. Una repeticion no borra el resultado anterior.

## Adaptacion de pruebas y algoritmo

La prueba principal valida la idea acordada, pero no presupone que los defaults
funcionen sin ajustes. Si falla se distinguira entre:

- error de implementacion o invariante;
- datos insuficientes de la trayectoria;
- threshold demasiado estricto/laxo;
- agrupacion BoW incorrecta;
- geometria degenerada o escena ambigua;
- coste de RAM/CPU/backlog excesivo.

Se podran ajustar trayectoria, thresholds, numero de regiones, baseline,
soporte, margen de ambiguedad o limites de ventana, documentando antes/despues y
repitiendo la prueba. Un cambio de arquitectura o semantica funcional exige un
nuevo acuerdo con el usuario.
