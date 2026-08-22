# Subfases 3M-3O - Criterios de cierre

## Resultado exigido

3M-3O solo podran marcarse `CONSEGUIDAS` cuando exista evidencia automatica,
replay, live y revision visual coherente con este contrato. Compilar o producir
algun RANSAC valido no basta.

## Invariantes obligatorios

- un unico `SecondaryWorker` activo;
- tarea activa no interrumpible;
- orden MAXIMA > MEDIA > BAJA y FIFO interno;
- `RawMapDatabase` no se modifica por loops, anchors, fusion u optimizacion;
- snapshots equivalentes no crean trabajo repetido;
- matching/RANSAC fuera de locks live;
- ningun uso funcional de GT para loops;
- ninguna publicacion desde el worker secundario;
- ningun apply parcial de patches o componentes;
- 3M no activa covisibilidad en grafos fiduciales;
- 3O no fusiona, no optimiza y no inserta covisibilidad server;
- 3O solo puede escribir `GlobalPoseStore` para anchor por loop validado;
- el visualizador no condiciona el pipeline.

Si se rompe cualquiera de estos invariantes, el resultado no puede ser
`CONSEGUIDA` aunque RViz2 parezca correcto.

## Criterios 3M

1. existe `CovisibilityDatabase` con pares canónicos y fuentes separadas;
2. una MEDIA por `ChangeSet` prepara y compromete un patch acotado;
3. patches equivalentes son idempotentes;
4. `relative_pose_measured` permanece inmutable;
5. lectores obtienen vistas inmutables/acotadas;
6. la MEDIA encola una BAJA por KF elegible y termina;
7. KFs sin cambio de covisibilidad pueden recibir BAJA directa;
8. la prioridad y backpressure se observan en tests/logs;
9. no hay barrido global ni payload pesado por delta;
10. la fuente ORB no se interpreta como fusion automatica.

## Criterios 3N

1. BoW original permanece en raw;
2. el indice derivado se actualiza/reconstruye deterministicamente;
3. existe una sola busqueda BoW por query;
4. candidatos se agrupan por region y se seleccionan hasta tres seeds;
5. KFs anclados y no anclados generan loops;
6. query/candidate se tratan simetricamente;
7. ledger y cache coalescen revisiones semanticas equivalentes aunque cambie la
   geometria exacta de validacion;
8. snapshots equivalentes no repiten BoW/geometria;
9. stale no crea retry inmediato infinito;
10. dequeue/commit revalidan una huella exacta y nunca aceptan un calculo
    obsoleto;
11. una relacion confirmada puede evitar trabajo y un rechazo solo aplica a sus
    revisiones;
12. 3N no modifica poses, fusion, anchors o covisibilidad;
13. cada filtro/region queda contabilizado con motivo.

## Criterios 3O

### Geometria

1. query subcloud usa MPs observados por query;
2. candidate subcloud usa una ventana, no solo el seed;
3. matching inicial, reduccion/fallback, matching refinado y RANSAC quedan
   instrumentados;
4. geometria degenerada se rechaza;
5. el resultado incluye transformacion, inliers, residual, cobertura, confianza
   y revisiones;
6. todas las regiones seleccionadas tienen resultado explicito;
7. mismo input/revisiones produce resultado determinista.

### Decision

1. una fusion valida suprime optimizacion de la misma tarea;
2. todos los pares compatibles se conservan deduplicados para 3P;
3. una sola query no aplica anchor/optimizacion/constraint activa;
4. dos queries independientes y coherentes si pueden activarla;
5. baseline inicial `0.20 m` o `5 grados` es configurable;
6. hipotesis incompatibles quedan diferidas;
7. una ganadora necesita margen de dos observaciones;
8. un KF puede relacionarse con varios KFs; no existe resolved global por query;
9. ambos lados anclados generan evidencia de fusion/3Q, no re-anchor;
10. exactamente un lado anclado puede anclar el otro sin depender de la
    orientacion query/candidate.

### Anchor y componentes

1. dos submapas no anclados pueden conservar constraint relativa;
2. un componente sin raiz world no aparece en `GlobalPoseStore`;
3. al obtener raiz se valida el componente y sus ciclos;
4. una cascada valida compromete todos los anchors atomicamente;
5. un ciclo/revision incompatible no deja estado parcial;
6. se incluyen KFs llegados durante RANSAC;
7. todos los KFs creados/movidos se notifican dirty;
8. el siguiente flujo principal, no el secundario, actualiza/publica;
9. se encolan loops coalescidos por nueva `anchor_revision`;
10. el anchor commit no activa parar drones por si mismo;
11. antes del primer fiducial, mover el KF de apoyo del padre propaga el mismo
    delta rigido al componente hijo blando mediante commit atomico;
12. el primer fiducial directo reancla todo el hijo como first anchor y siempre
    queda hard/control;
13. tras ese fiducial se corta el seguimiento rigido del padre;
14. los fiduciales posteriores usan 3H-3L;
15. ningun hard se mueve por propagacion de una dependencia loop.

Los puntos 12-14 son criterios del runtime transitorio con el que se cerro 3O.
3Q debe sustituirlos por promocion hard sin movimiento dentro de umbral o MAX
covisible fuera de umbral, cortando la dependencia solo tras `ACCEPT_FULL`.

## Criterios de rendimiento y concurrencia

- `active_secondary_workers <= 1`;
- el PrimaryWorker progresa durante BoW/RANSAC ralentizado;
- no hay locks live mantenidos durante calculo largo;
- memoria por tarea esta acotada a query/regiones/ventanas;
- no aparece backlog creciente sin que actue backpressure;
- los KFs tardios no se pierden;
- lifecycle web representa duracion real y no parpadea por subpasos;
- desconectar el visualizador no cambia resultados funcionales.

## Evidencia minima de cierre

```text
build de orbslam3_multi orbslam3_server simulacion_dron: exit 0
unit tests 3M/3N/3O: pass
replay determinista: pass y resultados repetibles
fixture/replay de componente A->B->C: commit atomico
live A fiducial + B loop anchor: success=true
logs reducidos analizados contra criterios
RViz2 y grafo web revisados por el usuario
documentacion e historial sincronizados
```

La prueba live debe demostrar que B no tiene world antes del loop y que todos
sus KFs aparecen coherentemente despues del commit y del siguiente flujo
principal.

## Clasificacion

`CONSEGUIDA`: todos los invariantes y evidencia minima pasan.

`PARCIAL`: arquitectura y tests principales pasan, pero falta un caso live,
visual o de calidad geometrica; debe decir exactamente que falta.

`NO CONSEGUIDA`: no compila, hay crash, falso apply, estado parcial, raw mutada,
bloqueo principal, anchor incoherente, uso de GT o incumplimiento de separacion
3O/3P/3Q.

`BLOQUEADA`: solo conforme a la politica general del proyecto tras repetirse el
mismo bloqueo y no poder progresar sin entrada externa.

## Politica explicita de adaptacion

Los siguientes valores son hipotesis iniciales, no invariantes:

```text
max_candidate_regions = 3
min_independent_query_translation = 0.20 m
min_independent_query_yaw = 5 grados
required_consistent_queries = 2
ambiguity_support_margin = 2
thresholds BoW/matching/RANSAC validados en la baseline anterior
limites de ventana y puntos
```

Si la evidencia muestra falsos positivos, falsos negativos, demasiada latencia,
RAM o backlog, se podran cambiar. Cada cambio exige:

1. conservar el intento anterior y su conclusion;
2. identificar el criterio que fallo;
3. justificar el parametro/algoritmo modificado;
4. repetir tests afectados, replay y live cuando corresponda;
5. actualizar la conclusion agregada;
6. pedir nuevo acuerdo si cambia la semantica funcional o un invariante.

No se declarara fracaso de la idea por un threshold inicial malo, ni se
declarara exito ocultando ajustes o pruebas fallidas.
