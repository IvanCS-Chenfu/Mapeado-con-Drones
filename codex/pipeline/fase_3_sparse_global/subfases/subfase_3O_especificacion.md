# Subfase 3O - Especificacion

## Alcance

3O transforma regiones candidatas de 3N en evidencia geometrica y aplica el
anclaje por loop cuando existe autoridad suficiente. Toda la secuencia pertenece
a la `LoopTask` BAJA activa y conserva su `task_id`.

### Incluido

- rama rapida para queries con pose world;
- construccion acotada de subnubes estilo `legacy2`;
- matching ORB, reduccion robusta y RANSAC 3D-3D;
- error respecto a poses world cuando ambos lados estan anclados;
- evaluacion conjunta de hasta tres regiones;
- fusion dominante como evidencia para 3P;
- hipotesis persistentes y control de ambiguedad;
- anclaje simetrico cuando exactamente un lado esta anclado;
- relaciones relativas y componentes cuando ninguno esta anclado;
- commit atomico de anchors y poses globales;
- continuidad con el primer fiducial posterior.

### Fuera de alcance

- fusionar MapPoints o crear tracks: 3P;
- insertar `SERVER_LOOP_GEOMETRIC` en covisibilidad: 3P;
- construir/optimizar un grafo por error alto: 3Q;
- usar covisibilidad en optimizaciones fiduciales 3H-3L;
- modificar datos crudos de `RawMapDatabase`;
- usar GT para buscar, validar, elegir o aceptar loops;
- publicar ROS desde el worker secundario.

## Entradas

`LoopGeometryInput` es una vista privada y acotada:

```text
task_id
query_kf_id
query_submap_id
query raw/appearance/geometry/anchor revisions
query local pose y world pose opcional
query observed MapPoint IDs
candidate regions de 3N, maximo inicial 3
candidate seed y member KF IDs acotados
snapshot de covisibilidad necesario
estado canonico de pares
pose, anchor, fusion y constraint revisions relevantes
```

No contiene copias completas de `RawMapDatabase`, `GlobalPoseStore` o
`CovisibilityDatabase`. Los descriptores y puntos se consultan solo para IDs
seleccionados y se copian a buffers privados antes del calculo largo.

## Subnubes

BoW elige un seed, no una nube final.

### Query

`query_subcloud` se construye con los MapPoints observados por el KF query:

- identidad raw completa;
- posicion en el marco local del submapa;
- posicion world opcional si existe anchor;
- descriptor ORB valido;
- keypoint/observacion necesaria para cobertura;
- score solo si la politica lo habilita;
- revisiones consumidas.

### Candidate

`candidate_subcloud` se construye alrededor del seed usando:

1. seed;
2. KFs de su region BoW;
3. vecinos de covisibilidad ORB fuerte;
4. parent/children si estan disponibles;
5. vecinos temporales acotados del mismo submapa;
6. vecinos espaciales solo cuando existe pose world.

Los MapPoints se deduplican por `RawMapPointId`. La ventana y el numero de
puntos tienen limites configurables. No se fusiona ni optimiza la nube completa.

## Rama rapida

Solo se ejecuta si el query tiene pose world. Busca en el mapa global puntos
cercanos y compatibles por descriptor. Para aceptar requiere:

- correspondencias uno a uno;
- ratio test y cross-check acordados;
- cobertura distribuida en imagen y espacio 3D;
- suficientes pares de IDs distintos y no fusionados;
- residual bajo y transformacion coherente con la pose actual;
- ausencia de degeneracion.

Si acepta, produce `FUSION_CANDIDATE` y omite BoW/RANSAC normal para ese query.
3P recibira los pares; 3O no escribe fusion.

## Verificacion normal

Por cada region seleccionada:

1. aplicar salidas tempranas por identidad, cache o relacion resuelta;
2. construir ambas subnubes;
3. matching ORB inicial por Hamming;
4. ratio test, unicidad y cross-check configurable;
5. calcular caja/region percentil robusta de matches;
6. reducir `candidate_subcloud` con fallback si la reduccion degenera;
7. repetir matching refinado;
8. ejecutar RANSAC 3D-3D determinista;
9. comprobar inliers, ratio, cobertura, residual y degeneracion;
10. calcular transformacion relativa y error cuando haya poses world;
11. conservar todas las revisiones consumidas.

Los parametros geometrico-algoritmicos parten de los defaults de `legacy2` y se
ajustan solo mediante evidencia de pruebas.

## Resultados por region

```text
GEOMETRY_REJECTED
DEFERRED
FUSION_CANDIDATE
LOOP_ANCHOR_CANDIDATE
LOOP_OPTIMIZATION_CANDIDATE
ALREADY_RESOLVED
STALE
```

`DEFERRED` incluye razones como:

```text
INSUFFICIENT_EVIDENCE
AMBIGUOUS_REPETITIVE_SCENE
BOTH_UNANCHORED_PENDING
WAITING_SECOND_QUERY
CONFLICTING_COMPONENT_CYCLE
```

El resultado geometrico conserva:

```text
relative_pose_measured
inlier_mappoint_pairs
ransac_inliers e inlier_ratio
mean/max residual
coverage_2d y coverage_3d
confidence
error_translation / error_rotation / error_yaw opcionales
dependency_revisions
```

## Decision conjunta

La `LoopTask` decide despues de evaluar todas las regiones seleccionadas:

1. si la rama rapida o cualquier region produce fusion valida compatible con la
   pose actual, conserva todos los pares compatibles, deduplica y suprime toda
   optimizacion de esa tarea;
2. si exactamente un lado esta anclado, crea o refuerza una hipotesis de anchor
   para el otro;
3. si ambos estan anclados, no hay fusion y el error es alto, crea o refuerza
   hipotesis de optimizacion para 3Q;
4. si ninguno esta anclado, crea o refuerza una constraint relativa;
5. si varias regiones implican la misma transformacion, suman soporte;
6. si implican transformaciones incompatibles, ninguna se aplica.

Un KF puede confirmar relaciones con varios KFs. No se marca el query como
resuelto globalmente tras su primer loop; la memoria es por par y revisiones.

## Consistencia entre queries

Fusiones por proximidad global pueden aceptarse con una evidencia geometrica
fuerte. Anchor, optimizacion y constraints entre submapas no anclados requieren
dos KFs query independientes.

Independencia inicial:

```text
RawKeyFrameId distinto
y
baseline local >= 0.20 m o yaw local >= 5 grados
```

Las transformaciones deben ser compatibles bajo thresholds configurables. Si
hay una hipotesis incompatible, la ganadora necesita una ventaja de al menos
dos observaciones. Mientras no la tenga, el estado es `DEFERRED_AMBIGUOUS` y no
se cachea como rechazo definitivo.

## LoopAnchorConstraintStore

Almacena constraints de anchor, no covisibilidad:

```text
LoopAnchorConstraint
  child_submap
  parent_submap
  evidence query/candidate KFs
  parent_local_T_child_local
  support_queries
  confidence
  state = PROVISIONAL | ACTIVE | AMBIGUOUS | STALE
  dependency_revisions
```

La orientacion query/candidate no define padre e hijo. Si exactamente un lado
tiene world, ese lado es la referencia. Si ninguno lo tiene, la relacion se
guarda sin world hasta que el componente conecte con un anchor.

## Anclaje y propagacion

Cuando una constraint alcanza soporte y el componente tiene una raiz world:

1. capturar el componente y sus revisiones;
2. elegir relaciones activas coherentes;
3. comprobar ciclos y transformaciones incompatibles;
4. calcular `world_T_local` para cada submapa no anclado;
5. reconsultar en raw todos sus KFs actuales, incluidos los llegados durante
   RANSAC;
6. preparar un batch completo de anchors y `world_T_kf`;
7. revalidar revisiones;
8. comprometer todo en `GlobalPoseStore` atomicamente o no comprometer nada;
9. registrar la dependencia relativa para propagacion futura;
10. notificar todos los KFs movidos/creados como dirty;
11. encolar loops por la nueva `anchor_revision`;
12. dejar la publicacion al siguiente flujo principal.

Si el componente ya contiene dos autoridades world independientes, no se
re-ancla rigidamente: la relacion queda como evidencia de optimizacion futura.

## Fiducial posterior

Mientras el loop sea la unica autoridad, el submapa hijo sigue rigidamente al
KF de apoyo del padre. Si ese KF cambia de `T_old` a `T_new`, se aplica
`delta = T_new * inverse(T_old)` a todo el componente descendiente que siga
siendo blando, mediante un batch atomico. No se alteran las poses relativas
internas ni se ejecuta una optimizacion multi-submapa.

Al observar directamente su primer fiducial:

- se calcula el anchor absoluto `world_T_local` como en un first anchor;
- se reancla rigidamente todo el submapa al fiducial, con independencia del
  error previo;
- el KF fiducial queda hard y se convierte en `last_accepted_control_kf`;
- se corta la propagacion desde el padre;
- la constraint loop se conserva como evidencia para la optimizacion 3Q;
- los fiduciales posteriores usan normalmente el flujo 3H-3L.

Al implementar 3Q, esta politica transitoria se sustituye: dentro de umbral el
hijo soft promociona hard sin mover; fuera de umbral ejecuta MAX con una
ventana conectada que distribuye la correccion y solo corta la dependencia tras
accept. Este bloque sigue describiendo el runtime previo validado de 3O.

## Invariantes

- un solo worker secundario;
- ninguna publicacion desde 3O;
- ninguna escritura raw;
- ningun uso funcional de GT;
- ningun apply de fusion u optimizacion por loop;
- anchor commit breve, atomico y versionado;
- matching/RANSAC fuera de locks live;
- tarea activa no interrumpible;
- flujo principal libre para ingresar y publicar.
