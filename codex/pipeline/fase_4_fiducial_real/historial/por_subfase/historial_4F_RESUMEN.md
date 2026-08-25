# Historial 4F - Resumen

## Estado agregado

`CONSEGUIDA`.

`RawMapDatabase` sincroniza batches y KFs por identidad exacta en ambos ordenes
de llegada. El sidecar usa lookup O(1), FIFO por dron, capacidad configurable
10, sin TTL y el mutex raw existente. Consume al emparejar, conserva el primer
`arrival_id` y distingue duplicate/conflict mediante digest, sin flag
`fiducial_batch_consumed`.

Las unitarias cubren llegada inmediata/pending, FIFO, dos drones,
duplicate/conflict y no reactivacion por updates/snapshots. En la prueba 210
hubo 68/68 matches, pico pending 7/10 y cero evictions/rejects/conflicts. La
211, con ambos grafos live, añadio 18/18 matches y pico 5/10.

Detalle cronologico: `historial_4F.md`.
