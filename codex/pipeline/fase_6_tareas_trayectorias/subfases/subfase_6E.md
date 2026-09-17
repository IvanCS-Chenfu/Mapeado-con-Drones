# Subfase 6E - EvidenceDatabase y KeyframeEvidenceWorker

## Estado

`PARCIAL`.

`EvidenceDatabase` y `KeyframeEvidenceWorker` ya existen en `task_server`.
El worker recibe `/global_keyframe_sparse_evidence_delta`, construye las
fuentes locales `sparse_ray_free` y `ransac_free`, y escribe una transaccion
por KF sin tocar el mapa mundial. Upserts, cambios de pose/revision y deletes
se deduplican frente a la ultima fuente o tombstone conocida.

## Objetivo

Crear la base de evidencia reversible que separa la geometria local de cada
KF del mapa voxel mundial.

## Contrato funcional

`KeyframeEvidenceWorker` consume deltas de KF y calcula por fuente local:

- asociaciones `KF -> MapPoints`;
- rayos FREE directos de MapPoints cualificados;
- planos RANSAC y huellas FREE cuando procedan;
- revisiones, deletes y pose de referencia.

Cada actualizacion se escribe como transaccion atomica en `EvidenceDatabase`.
Una revision nunca deja media fuente KF visible al consumidor. Las fuentes se
identifican por `(drone_id, map_epoch, keyframe_id, source_id)` y permanecen
en coordenadas locales al KF.

## Cambios requeridos

- Sustituir aplicacion directa de evidencia KF al mapa por escritura de fuentes
  y tombstones en la base.
- Persistir revision geometrica, asociaciones y ultima pose aplicada.
- Notificar `database_dirty` sin llamar directamente a `VoxelMapBuilder`.

## Exclusiones

- Este worker no publica voxeles mundiales ni decide transitabilidad.
- No procesa imagenes depth.

## Validacion y exito

KF nuevo, modificado, optimizado y eliminado afectan solo a sus fuentes.
Lecturas concurrentes no observan transacciones parciales. La evidencia sparse
queda lista para reproyeccion incremental sin reconstruir el mapa.

La suite nueva cubre alta atomica, reproyeccion y tombstone. `task_server`
compila y CTest pasa 9/9; no se requiere simulacion hasta conectar depth en 6F.
