# Subfase 4F — Recepción y sincronización exacta de observaciones en el servidor

<!-- ACUERDOS_CIERRE_F2_2026_08_24_START -->
## Impacto obligatorio en system_architecture

> **Vigencia:** acuerdo cerrado el 2026-08-24. Este bloque prevalece sobre cualquier
> frase anterior incompatible del mismo documento. No borra ni reescribe evidencia
> histórica; distingue siempre entre estado actual, deuda conocida y arquitectura objetivo.

4F crea la comunicación runtime real `orbslam3 -> orbslam3_server` para el batch
fiducial. Añadir/actualizar esa arista y su evidencia live directa. No reutilizar GT ni
`flow_events` para fingir actividad. La arista debe poder indicar `drone_id` y KF sin
transportar payload pesado al visualizador.
<!-- ACUERDOS_CIERRE_F2_2026_08_24_END -->

## Estado

```text
sin hacer
```

## Dependencia

`4E`.

## Objetivo técnico

Hacer que `orbslam3_server` reciba los batches de tags y los asocie exclusivamente al KeyFrame identificado por `(drone_id, map_epoch, local_keyframe_id)`. El orden de llegada entre `orb_map_delta` y el batch fiducial no debe afectar al resultado.

Esta subfase no interpreta todavía qué tag pertenece a qué cubo ni calcula anchor. Su salida es una observación visual sincronizada con un KF crudo real de `RawMapDatabase`.

## Comportamiento esperado

```text
Caso A: delta con KF -> batch -> procesar inmediatamente
Caso B: batch -> todavía no hay KF -> guardar pendiente -> delta con KF -> procesar
Caso C: batch -> KF nunca aparece/epoch inválido -> expirar/rechazar explícitamente
```

Nunca se reasigna una observación al KF anterior/posterior.

## Contexto obligatorio a leer


```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4.md
```


Además:

```text
subfases/subfase_4E.md
codex/contexto/paquetes/orbslam3_server/00_summary.md
codex/contexto/paquetes/orbslam3_multi/00_summary.md
src/orbslam3_server/src/global_map_server.cpp
src/orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp
src/orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp
```

## Diagnóstico de partida

El servidor actual genera observaciones fiduciales dentro de `ProcessFiducialsForDelta()` a partir de la asociación temporal entre KeyFrames y un buffer GT. Esa ruta ya conoce el KF porque recorre el delta, pero no existe una suscripción visual independiente ni una cola para batches que puedan llegar antes que el KF.

La identidad canónica del submapa sigue siendo:

```text
submap = (drone_id, map_epoch)
```

La identidad del KF debe añadir `local_keyframe_id`.

## Archivos permitidos a modificar

```text
src/orbslam3_server/include/orbslam3_server/global_map_server.hpp
src/orbslam3_server/src/global_map_server.cpp
src/orbslam3_server/launch/global_orb_map_server.launch.py
src/orbslam3_server/CMakeLists.txt
src/orbslam3_server/package.xml
src/orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp      # solo si hace falta un tipo de clave ya inexistente
codex/contexto/paquetes/orbslam3_server/
codex/contexto/paquetes/orbslam3_multi/
```

Preferir mantener la cola pendiente en el adaptador ROS/server y no contaminar el backend puro si no aporta valor.

## Archivos prohibidos

```text
ORB_SLAM3/
ORB_SLAM3_ROS2/
src/simulacion_dron/
src/dron_individual/
src/orbslam3_multi/src/fiducial_anchor_manager.cpp   # integración de anchor en 4H
```

## Funciones, clases o nodos que hay que localizar

```text
global_map_server
callback de orbslam/orb_map_delta
RawMapDatabase inserción/consulta de KeyFrames
ProcessFiducialsForDelta
HandleFiducialObservation
RawSubmapId
RawKeyFrameId
```

Crear el callback visual y la estructura de pendientes solo tras localizar el orden real de ingestión del delta.

## Cambios requeridos

1. Añadir suscripción al topic de 4E.
2. Validar que `drone_id`, `map_epoch`, `local_keyframe_id` son sintácticamente aceptables y que `observations` no está vacío.
3. Construir la clave exacta del KF usando los tipos de `raw_map_types` existentes.
4. Si el KF ya está en `RawMapDatabase`, pasar el batch a una función interna de “observación sincronizada” sin reinterpretarlo todavía.
5. Si el KF no existe, guardar el batch en una cola/mapa pendiente acotado por clave y tiempo/numero de elementos.
6. Tras aplicar cada delta a `RawMapDatabase`, intentar resolver únicamente los pendientes cuyas claves hayan aparecido.
7. Verificar `keyframe_stamp` contra `OrbKeyFrame.stamp` con una tolerancia muy pequeña para detectar corrupción; un mismatch no autoriza asociar a otro KF.
8. Rechazar batches de epochs antiguos/futuros incoherentes según el estado conocido del dron.
9. Definir política de duplicados: el mismo batch/KF no debe procesarse dos veces por reentrega QoS.
10. Definir expiración de pendientes y log explícito `FID-SYNC-EXPIRED`; no mantener memoria sin límite.
11. Añadir marcadores `FID-SYNC-PENDING`, `FID-SYNC-MATCHED`, `FID-SYNC-REJECT` con clave exacta y tag_count.
12. Mantener `ProcessFiducialsForDelta()` GT activo solo mientras sea necesario para transición/pruebas antiguas, pero no mezclar sus observaciones con la ruta visual en pruebas de Fase 4.

## Cambios prohibidos

- No buscar el KF “más cercano” por timestamp.
- No cambiar `map_epoch` recibido para hacer que encaje.
- No crear un KF sintético si no ha llegado el delta.
- No calcular `world_T_camera` aún.
- No anclar el submapa aún.
- No borrar la ruta GT legacy hasta que 4H/4K demuestren reemplazo completo y se acuerde su limpieza.
- No guardar imágenes en pendientes.

## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_msgs orbslam3_multi orbslam3_server
```

`orbslam3_multi` solo necesita rebuild si se toca un header compartido.

## Pruebas Gazebo requeridas

### Prueba 1 — Delta antes que observación

Forzar/observar un KF cuyo delta llegue antes del batch. Debe aparecer:

```text
FID-SYNC-MATCHED mode=immediate
```

con la misma clave que `OrbKeyFrame`.

### Prueba 2 — Observación antes que delta

Usar temporización/QoS de prueba o un test controlado que entregue primero el batch. Debe aparecer `PENDING` y, al llegar el KF exacto, `MATCHED` sin cambiar ID.

### Prueba 3 — KF inexistente

Inyectar un mensaje de prueba con ID no existente. Debe expirar/rechazarse y nunca asociarse a un KF distinto.

### Prueba 4 — Duplicado

Reenviar el mismo batch; la segunda entrega debe clasificarse como duplicado sin duplicar la futura restricción.

## Patrones de reducción de logs

```text
FID-SYNC|orb_map_delta|RawMapDatabase|drone_id|map_epoch|keyframe_id|pending|matched|duplicate|expired|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

1. Los paquetes afectados compilan.
2. Ambos órdenes de llegada se resuelven al mismo KF exacto.
3. Nunca se reasigna por proximidad temporal.
4. Duplicados no se procesan dos veces.
5. Pendientes son acotados y expiran con diagnóstico.
6. Timestamp/epoch inconsistentes se rechazan.
7. No se ha calculado anchor ni lógica tag-cubo todavía.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: asociación con KF incorrecto, pérdida silenciosa de batch o cola no acotada.
- `PARCIAL`: orden normal funciona pero fuera de orden/duplicados no están resueltos.
- `BLOQUEADA`: `RawMapDatabase` no ofrece una consulta segura del KF exacto y requeriría un rediseño mayor no acordado.

## Riesgos

- carrera entre callback de delta y callback fiducial;
- keyframes modificados que reaparecen en deltas;
- epoch cambiado mientras hay pendientes;
- deduplicación solo por `keyframe_id` ignorando dron/epoch.

## Documentación a actualizar

```text
codex/contexto/paquetes/orbslam3_server/
codex/contexto/paquetes/orbslam3_multi/
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4F.md
codex/pipeline/fase_4_fiducial_real/historial/por_subfase/historial_4F_RESUMEN.md
```
