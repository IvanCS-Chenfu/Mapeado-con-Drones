# Historial 4E - Resumen

## Estado agregado

`CONSEGUIDA`.

Se añadieron las dos interfaces replicadas, el publisher
`orbslam/fiducial_keyframe_observations` reliable/volatile KeepLast(32) y la
conversion exacta compartida de timestamp. Cada resultado valido genera un
batch no vacio, ordenado por `tag_id`, con `camera_T_tag` y metricas finitas;
no transporta GT ni semantica de objeto.

Interfaces Dron/Servidor, wrapper, servidor, backend y Simulacion compilan.
Contratos 39/39 y CTests de wrapper/backend/servidor/Simulacion pasan. La 209
fallo por ruta YAML relativa; la 210 completo la trayectoria tipica y publico
68 batches, todos sincronizados. La 211 verifico ambos grafos live y otros
18 batches sincronizados.

Detalle cronologico: `historial_4E.md`.
