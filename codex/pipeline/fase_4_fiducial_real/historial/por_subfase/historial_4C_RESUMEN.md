# Historial 4C - Resumen

## Estado agregado

`CONSEGUIDA`.

`Tracking` produce un evento one-shot exactamente al crear un KF y `System`
lo entrega por valor junto con imagen izquierda, intrinsecos, distorsion,
dimensiones y estado de rectificacion efectivos. El wrapper ya no infiere la
asociacion por timestamp ni consulta un ultimo KF persistente.

El target nativo ORB-SLAM3, el wrapper y sus tests compilan. Las pruebas 204 y
205 mostraron eventos con identidad coherente y `timestamp_delta=0`; los KFs
anteriores a `READY` se omitieron segun contrato.

Detalle cronologico: `historial_4C.md`.
