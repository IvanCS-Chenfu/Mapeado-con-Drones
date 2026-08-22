# Subfase 3O - Geometria de loop y anclaje por loop

## Estado

```text
CONSEGUIDA; CORRECCIONES TECNICAS Y VALIDACION VISUAL COMPLETADAS
```

3O conserva el matching y RANSAC validados previamente, pero rehace la
integracion, la politica de candidatos y el tratamiento de submapas no anclados.

## Sucesion acordada en 3Q

3O sigue siendo autoridad de subnubes/RANSAC. 3Q conserva los inliers de la
tarea que obtiene el segundo apoyo y continua sin repetir geometria. No se
excluyen loops por ser inter/intra dron o submapa; permanecen las guardas
causales y de ambiguedad. La propagacion/reanchor soft transitorios de 3O se
sustituyen por grafo covisible al entrar en los casos definidos por 3Q.

## Objetivo resumido

Dentro de la misma `LoopTask` BAJA:

```text
query con pose world
  -> rama rapida de solape global
  -> FUSION_CANDIDATE si basta

si no resuelve
  -> regiones candidatas de 3N
  -> query_subcloud + candidate_subcloud por region
  -> matching ORB + reduccion + RANSAC 3D-3D
  -> evaluar el conjunto completo
  -> fusion, anchor, optimizacion futura, rechazo o espera
```

3O no fusiona MapPoints, no ejecuta optimizacion por loop y no inserta
`SERVER_LOOP_GEOMETRIC` en `CovisibilityDatabase`. Si confirma un anclaje por
loop, si puede modificar `GlobalPoseStore` mediante un commit atomico y
versionado; esta es la unica excepcion de efecto live de 3O.

## Politica de decision

- una fusion valida y compatible con las poses actuales domina y suprime toda
  optimizacion de esa `LoopTask`;
- se conservan todos los pares de fusion compatibles y deduplicados para 3P;
- exactamente un lado anclado permite anclar el otro, sin importar quien sea
  query o candidate;
- ambos anclados y error alto producen evidencia para 3Q, no otra tarea;
- ambos no anclados producen una constraint relativa provisional;
- hipotesis incompatibles terminan `DEFERRED_AMBIGUOUS`;
- geometria rechazada se cachea solo para las mismas revisiones;
- relaciones ya resueltas evitan repetir trabajo costoso.

## Consistencia y escenas repetitivas

- una evidencia geometrica fuerte basta para fusion por proximidad global;
- anchor, optimizacion y constraints entre no anclados requieren dos KFs query
  independientes;
- independencia inicial: baseline local de al menos `0.20 m` o `5 grados`;
- las transformaciones medidas deben ser compatibles;
- si compiten hipotesis incompatibles, la ganadora necesita una ventaja minima
  de dos observaciones;
- todos los valores son parametros iniciales ajustables mediante evidencia.

## Componentes no anclados

`LoopAnchorConstraintStore` conserva relaciones relativas entre submapas. Al
obtener world uno de ellos, se valida el componente, se calculan anchors y se
comprometen atomicamente todos los submapas conectados. Se incluyen KFs llegados
durante RANSAC, se notifican dirty y se encolan sus loops por nueva revision de
anchor.

El commit de anchor no para drones. El watermark de la cola mantiene el
backpressure normal. Mientras el hijo solo tenga autoridad loop, cualquier
movimiento del KF de apoyo del padre se propaga como delta rigido y atomico al
componente hijo blando. Su primer fiducial directo corta esa dependencia y
reancla todo el submapa de forma absoluta, como un first anchor, aunque el error
previo ya estuviera bajo umbral. Los fiduciales posteriores recuperan el flujo
normal 3H-3L.

Las reevaluaciones se coalescen mediante una huella semantica de apariencia,
pose gruesa, madurez de asociaciones y soporte covisible. Una
`validation_revision` exacta distinta se usa al dequeue/commit para conservar
la proteccion frente a calculos obsoletos sin volver a encolar cada refinamiento
interno de ORB.

## Documentos

- `subfase_3O_especificacion.md`: alcance, datos, decisiones e invariantes.
- `subfase_3O_implementacion.md`: clases, algoritmo, commits y telemetria.
- `subfase_3O_testing.md`: unit tests, replay, live y analisis de logs.
- `subfase_3O_criterios.md`: exito, fallo y politica de adaptacion.

## Visualizador

El grafo web incluye `SubcloudLoopVerifier`, decision geometrica,
`LoopAnchorConstraintStore` y commit en `GlobalPoseStore`. Cada etapa usa el
mismo `task_id`; la iluminacion dura lo que dura la etapa real y no parpadea por
subpasos internos.

## Evidencia de cierre

- B/KF5 confirma geometria y queda `waiting_second_independent_query`;
- B/KF7 confirma una hipotesis compatible y aplica un batch de anchor con 8
  KFs dirty, sin ninguna observacion fiducial del dron B;
- el siguiente `PrimaryInput` hace backfill de 9 KFs y recalcula 1013 MPs;
- cierre live 154: 2 anchors, 1 hard, 248 poses, 222 activas, cola secundaria
  vacia y cero fallos duros.

## Revision tras live 154

- El control derivado de loop no adquiere autoridad fiducial. El primer
  fiducial directo del hijo sustituye el anchor blando mediante reanchor rigido
  absoluto y conserva la constraint loop solo como evidencia futura.
- Antes de ese fiducial, si una optimizacion mueve el KF de apoyo del padre, el
  hijo y sus dependencias blandas reciben el mismo delta rigido en un batch
  atomico. Esto mantiene coherencia, pero no es una optimizacion covisible.
- La optimizacion 3Q multi-submapa con aristas de covisibilidad sustituira
  tanto el reanchor directo transitorio como la propagacion rigida en los casos
  acordados; 3O conserva su evidencia historica y la ruta previa como regresion.
- El flujo web continuamente activo queda aceptado como representacion de
  trabajo real consecutivo. Las reevaluaciones redundantes se redujeron de
  9.20 a 2.18 tareas por KF sin perder la validacion exacta al commit.
- La prueba 156 fue validada por el usuario en RViz2 y el grafo web. 3O queda
  cerrada; 3P y 3Q comprobaran de extremo a extremo las decisiones de fusion y
  optimizacion que 3O solo produce como evidencia.

## Principio de adaptacion

Este diseño es la hipotesis principal acordada. No se presupone que tres
regiones, dos queries o los thresholds iniciales sean perfectos. Si las pruebas
revelan falsos positivos, falta de detecciones, exceso de RAM o latencia, se
ajustaran parametros o detalles algoritmicos y se documentara cada intento. No
se cambiaran silenciosamente los invariantes de raw inmutable, un worker,
prioridades, commit atomico, ausencia de GT y separacion 3O/3P/3Q.
