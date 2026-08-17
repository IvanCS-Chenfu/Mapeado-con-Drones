# Subfase 3S - Implementacion acordada

## Estado vigente

```text
REHACER
```

## Reimplementación obligatoria

- conservar el score base de 3F inmediatamente despues del commit raw, solo
  para MPs de `RawChangeSet`;
- reutilizar `DatabaseUpdateTask` MEDIA para calculos derivados/avanzados que
  3S añada y que no sean necesarios para crear el score ORB inicial;
- `PrepareFusionUpdates` permanece dentro del patch de `3P`;
- todo cálculo se hace fuera de locks y el commit breve valida revisiones;
- el resultado devuelve `ScoreChangeSet` con altas/cambios/retiradas;
- `3F` recibe dirty IDs y no necesita `CreateSnapshot()` completo en runtime.

## Lo implementado antes que no debe repetirse

No se recalcularán todos los scores por llegada, no se bloqueará publicación y
ningún resultado rechazado/stale dejará score visible.

## API principal

La clase debe ofrecer operaciones equivalentes a:

```cpp
ScoreUpdateCandidate PrepareRawUpdates(
    const RawChangeSet&, const RawMapSnapshot&, const ScoreSnapshot&);

ScoreUpdateCandidate PrepareFusionUpdates(
    const FusionCommitCandidate&, const ScoreSnapshot&);

ScoreCommitResult Commit(
    ScoreUpdateCandidate&&, const ExpectedRevisions&);

ScoreSnapshot CreateSnapshot() const;
```

Preparar no escribe. `Commit` valida revision y publica un lote corto.

## Integracion principal

```text
RawMapDatabase commit
-> ChangeSet
-> calcular/actualizar score ORB base solo para MPs afectados
-> commit score base
-> ScoreChangeSet hacia dirty sets de GlobalMapBuilder
```

El score base no se difiere: forma parte del flujo principal acordado en 3F.
Si calcular una estadistica avanzada es costoso, se crea una actualizacion
derivada secundaria y su commit solo deja IDs dirty; nunca es dependencia para
publicar la geometria raw disponible.

## Integracion secundaria

`3P` prepara tracks y eventos de score sobre snapshots. El commit coordinator
aplica fusion y score bajo una revision comun. Si el candidato queda stale,
ambos se descartan.

No se llama a `LandmarkScoreManager` despues de terminar la `LoopTask` para
completar trabajo pendiente: score forma parte del mismo commit.

Los cambios negativos que solo afectan raw pueden constituir un segundo patch
breve inmediatamente posterior al commit positivo, siempre dentro de la misma
`LoopTask`. Si ese patch queda stale se omite sin revertir la fusion.

## Evidencia geometrica introducida por 3P

```text
INLIER_CONFIRMED                 +0.04 por MP
EXPECTED_VISIBLE_MISS            -0.01 al MP contradicho
FOREGROUND_CONTRADICTION         -0.03 al MP contradicho
```

No existen eventos numericos para RANSAC rechazado, outlier ambiguo/aislado,
occlusion, punto fuera/detras de camara, incertidumbre, tarea stale o error alto
sin optimizacion. `LandmarkScoreManager` deduplica por identidad de evidencia y
revision.

## Fused score

Debe ser una funcion determinista de:

- scores de miembros;
- soporte y numero de observaciones;
- numero/diversidad de submapas observadores;
- confirmaciones geometricas;
- estado bad/invalido.

La formula inicial conserva el baseline de `legacy2`: mejores/scores de los
miembros mas bonuses acotados por numero de miembros, drones y submapas. Se
mantiene configurable y 3S podra recalibrarla con evidencia real.

Conservar la formula en una unica politica configurable y testeable. El track
no duplica una segunda autoridad numerica fuera de `LandmarkScoreManager`.

## Snapshots y consultas

`ScoreSnapshot` se usa para calculos secundarios privados. Para publicacion,
`GlobalMapBuilder` consulta por ID los registros dirty de
`LandmarkScoreManager`; no se crea ni se copia un `PublicationSnapshot`
completo de scores. Estadisticas, journal y export nunca entran en el camino de
publicacion.

## Limpieza

Durante la implementacion eliminar, tras localizar consumidores y cubrirlos:

- modificaciones numericas directas fuera del manager;
- journals usados como rollback normal de estado ya visible;
- actualizaciones de score posteriores/desacopladas al commit de fusion;
- parametros duplicados o sin consumidor;
- exports con prefijo de subfase incorrecto;
- reconstrucciones completas por cada evento.

## Logs/eventos

```text
F1S-RAW-SCORE-COMMIT
F1S-FUSED-SCORE-PREPARE
F1S-FUSED-SCORE-COMMIT
F1S-SCORE-NOOP
F1S-SCORE-STATS
```

Incluyen revision, task/arrival ID, entidades afectadas, razones y tiempo de
preparacion/commit. La exportacion detallada es opcional, fuera de locks y
desactivada por defecto.

## Publicacion

`GlobalMapBuilder` consulta y copia el campo score, pero no filtra puntos por
score, no recalcula la politica ni escribe el manager. Cada commit devuelve IDs
dirty; no solicita una revision, no llama al builder y no publica. La siguiente
`PrimaryTask` aplica esos IDs y actualiza el atributo score sin reproyectar XYZ
cuando la geometria no cambio.

## Exclusiones

No cambiar geometria, BoW, optimizacion ni fusion para ajustar score. No añadir
nuevos mensajes.

Los eventos visuales de score son patches ligeros conforme a
`../CONTRATO_VISUAL_INCREMENTAL.md`; nunca snapshots completos de scores.
