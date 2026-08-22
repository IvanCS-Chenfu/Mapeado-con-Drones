# Subfase 3P - Especificacion de fusion

## Estado

```text
AJUSTE POST-160 IMPLEMENTADO; REVISION VISUAL 161 PENDIENTE
```

## Propiedad funcional

`LoopPipeline` recibe el resultado completo de `3O`, evalua el conjunto de
regiones y elige el desenlace. Cuando existe al menos una region de error bajo
compatible, construye una peticion tipada para fusion. `SparseGlobalBackend`
despacha esa peticion al componente especializado en la misma pila de llamada,
el mismo worker y el mismo `task_id`.

No se creara una clase de decision separada. Tampoco se encolara una
`FusionTask`: la unidad planificable sigue siendo la `LoopTask` BAJA completa.

## Entrada minima

```text
FusionRequest
  task_id y loop_result_id
  query/candidate KF y submapas
  regiones compatibles seleccionadas
  relative_pose_measured por region
  matches: raw IDs, distancia Hamming, residual e inlier/outlier
  soporte, ratio, cobertura y error de cada region
  raw/pose/fusion/covisibility/score revisions consumidas
```

La peticion solo puede proceder de RANSAC aceptado. BoW, cercania de pose, GT o
una arista ORB debil no bastan. Si 3O rechaza RANSAC, no hay fusion ni cambios
de score.

## Decision sobre todas las regiones

Antes de escribir se consideran todas las regiones geometrizadas:

- se descartan regiones ambiguas o incompatibles entre si;
- si existe evidencia de fusion valida, la fusion domina y no se abre una
  optimizacion para esa tarea;
- se agrupan todos los pares compatibles, conservando region, KFs, medida y
  revisiones de procedencia;
- si solo existe error alto, 3P termina sin guardar pares, fusionar, optimizar
  ni cambiar score.

No se fusionan simplemente los MPs de los KFs seed de BoW. Se usan las
subnubes query/candidate construidas en 3O con el enfoque validado previamente.

## Tracks fusionados

`FusedLandmarkManager` es propietario de:

```text
RawMapPointId -> FusedTrackId                    # consulta O(1)
FusedTrackId -> FusedLandmarkTrack
FusedLandmarkTrack:
  miembros raw y toda su procedencia
  submapas/KFs observadores
  representante publicable
  descriptor medoid
  posicion derivada
  contador exacto de miembros/evidencias
  estado normal o degraded
  fusion_revision
```

Reglas de identidad:

- mismo raw ID: no-op, sin crear track artificial;
- miembros ya en el mismo track: no-op; solo una evidencia realmente nueva
  puede reforzar soporte/score;
- un raw libre y un track: añadir el raw;
- dos raws libres: crear track;
- dos tracks distintos: union transitiva;
- el ID estable es el menor `FusedTrackId` ya existente; al crear uno se usa la
  politica monotona del manager.

Cada track conserva todos sus miembros raw. `RawMapDatabase` no se reescribe y
la posicion world del track es derivada/recalculable.

## Representante y guardas geometricas

- Posicion: media ponderada de las posiciones world vigentes de miembros.
- Peso inicial: score raw por numero de observaciones, con guardas de finitud.
- Descriptor: medoid de los descriptores validos.
- Dispersion maxima inicial al unir tracks: `0.50 m`, configurable.
- Una pareja incompatible se rechaza individualmente; la tarea solo se
  rechaza si el soporte restante cae por debajo del minimo geometrico.

Si una pose o reanchor posterior separa temporalmente miembros ya fusionados,
la identidad del track se conserva y se marca `degraded`. Hasta una futura
optimizacion covisible, el representante publicable se toma del miembro con
mayor autoridad hard/soporte, sin romper automaticamente la equivalencia.

## Evidencia RANSAC y score

La clasificacion por pareja es:

- `INLIER`: candidato de fusion y refuerzo inicial `+0.04` para ambos MPs;
- `AMBIGUOUS_OUTLIER`: residual apenas superior al umbral; solo diagnostico;
- `HARD_OUTLIER`: residual inicial `>= 2 * ransac_inlier_threshold`; pasa a
  comprobacion de visibilidad, sin penalizacion automatica;
- `ISOLATED_OUTLIER`: sin evidencia contextual suficiente; solo diagnostico.

Los deltas son configurables. No se penalizan todas las diferencias entre
nubes, ambos extremos de un outlier ni una region RANSAC rechazada.

## Visibilidad sparse simetrica

Para una region aceptada se proyectan de forma simetrica las dos subnubes:

```text
query -> candidate
candidate -> query
```

Se usa un z-buffer sparse temporal con intrinsecos, dimensiones de imagen,
poses de KF, rangos y como maximo los puntos acotados ya reunidos por 3O
(`~320` por lado inicialmente). Clasifica:

```text
VISIBLE_EXPECTED_MISS       -> -0.01 al MP que debia observarse
FOREGROUND_CONTRADICTION    -> -0.03 al MP contradicho
OCCLUDED / OUTSIDE / BEHIND / UNCERTAIN -> sin cambio
```

La evaluacion es simetrica para evitar favorecer query o candidate. No tiene
presupuesto temporal: debe recorrer toda la evidencia negativa elegible de las
subnubes ya acotadas por 3O. La terminacion queda garantizada por un maximo de
tres regiones, unas 320 muestras por lado y colecciones finitas. Si aumenta la
carga, actuan la cola secundaria, coalescencia y backpressure 64/16.

Este buffer no calcula disparity, no persiste depth, no modela superficies
densas y no puede autorizar trayectorias. Las fases 6 y 8 contienen referencias
expresas para reevaluar si su interfaz o telemetria merece reutilizarse.

## Score y publicacion

`LandmarkScoreManager` es la unica autoridad numerica. Sus registros deben
distinguir:

```text
base_score_orb
positive_evidence / positive_adjustment
negative_evidence / negative_adjustment
final_score
```

El score fused conserva provisionalmente la fórmula histórica útil: combina
los mejores/scores de miembros y bonuses acotados por miembros, drones y
submapas. `3R` podra refinar la formula sin cambiar ownership.

`GlobalMapBuilder` no filtra por score. Publica todos los raws no sustituidos y
un representante por track, copiando el score vigente como atributo. El
filtrado por score se reserva a la GUI de Fase 7.

## Atomicidad e idempotencia

Toda evidencia incluye identidad y revisiones. Repetir exactamente la misma
evidencia/revision es no-op y no incrementa dos veces miembros, soporte o score.
Una revision materialmente nueva puede aportar refuerzo nuevo.

El commit positivo hace visibles de forma logicamente atomica:

- tracks y representantes;
- arista `SERVER_LOOP_GEOMETRIC` de covisibilidad;
- score fused derivado;
- revision derivada y changesets exactos.

Score raw/fused, tracks y covisibilidad forman el mismo intento logico. Si una
revision cambia antes del commit, no se aplica nada. Si un paso posterior exige
rollback, se restaura todo lo aplicado. En ambos casos el intento termina y el
servidor encola una `LoopTask` BAJA fresca para el mismo KF.

El retry captura revisiones actuales y repite BoW, regiones, matching, RANSAC,
fusion y score; no reutiliza geometria stale. Se inserta al final de BAJA tras
completar/liberar la identidad anterior, permitiendo que MAX/MEDIA precedan. No
hay limite fijo de reintentos, pero solo puede existir una tarea equivalente
pendiente y no se reencola si el KF ya no existe, esta inactivo o es `bad`.

## Fronteras

Permitido: componentes activos de fusion, loop pipeline/backend,
covisibilidad, score, builder, servidor y visualizador estrictamente necesarios.

Prohibido:

- modificar `ORB_SLAM3`, `orbslam3_ros2` u `orbslam3_msgs`;
- escribir raw o poses;
- usar GT;
- crear otra cola/worker o cambiar backpressure;
- publicar/despertar al principal desde el worker secundario;
- retener locks durante matching, medoid, visibilidad o reconstruccion;
- conservar ahora inliers de la rama de error alto.
