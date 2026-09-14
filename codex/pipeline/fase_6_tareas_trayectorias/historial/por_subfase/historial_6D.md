# Historial - 6D

## 2026-09-07 - Implementacion y validacion

- Se anadieron `VoxelCell`/`VoxelMap` y `ReversibleVoxelMap` por fuentes.
- El primer unitario detecto que un snapshot sparse borraba `FREE`; se corrigio
  para reemplazar exclusivamente contribuciones `OCCUPIED`.
- Build y CTest final de `task_lib`: 6/6 correctos.
- Prueba 605: snapshots sparse reversibles publicados y visibles en GUI;
  escenario final con `SIM-EXIT-CODE 0`.

Conclusion: CONSEGUIDA dentro del alcance acordado; no hay FREE live fingido.

## 2026-09-08 - FREE fisico reversible por keyframe

- `ReversibleVoxelMap` separa contribuciones sparse y de paso real. La nueva
  operacion `ReplaceFreeVolume` rasteriza el volumen fisico configurable del
  dron, retira la version anterior de la misma fuente y da prioridad a `FREE`
  sobre ocupacion sparse en una celda coincidente.
- `task_server` guarda cada pose canonica elegible como `K_T_body` respecto de
  su `reference_keyframe_id`. Escucha y consulta la pose global del KF; ante
  una revision distinta retira y reinserta el volumen `FREE` de ese KF y sus
  muestras asociadas. En simulacion GT solo alimenta la pose canonica; no se
  usa para fabricar un mapa fuera de este contrato.
- Builds de `task_lib` y `task_server` correctos. CTest finales: 7/7 y 7/7.
- Prueba 619: interrumpida antes de pedir rutas al revelar que el requery de
  todos los KFs cada 2 s generaba trabajo y ruido excesivos. Se limito a un KF
  por tick y se omiten revisiones sin cambios.
- Prueba 620: Gazebo y GUI F7 con GT hacia fiducial 2 terminaron correctamente.
  Los marcadores `F6I-FREE-KF` confirman respuestas push/service y revisiones
  reales, incluida reintegracion de muestras asociadas.

Conclusion vigente: PARCIAL. OCCUPIED y FREE reversible por KF estan
conseguidos; depth metrico real queda para Fase 8 y la prueba no demostro aun
un corredor D* visual completo en el mapa denso.

## 2026-09-09 - Capa navegable incremental por perfil

- `ReversibleVoxelMap` paso a ser tambien la unica autoridad de la capa
  navegable por `footprint_profile_id`: snapshot inmutable y deltas locales de
  transitabilidad, coste de entrada y mascara compacta de 26 conexiones.
- La primera version, probada en 624, repetia la comprobacion de inflacion para
  cada vecino: 24 cambios raw y 1.056 celdas recalculadas bloquearon el worker
  16.060 ms.
- La version vigente separa transitabilidad y aristas en dos pasadas locales.
  En 625, 24 cambios/1.122 celdas tardaron 110.169 ms. Tambien atendio lotes de
  988 cambios/34.703 celdas en 2.633 s sin reconstruir todo el mapa.
- Build de `task_lib` y `task_server` correctos; CTest focales 7/7 y 7/7.

Conclusion: la capa derivada incremental por perfil esta CONSEGUIDA. El estado
agregado de 6D permanece PARCIAL exclusivamente por depth metrico real,
aplazado a Fase 8.

## 2026-09-09 - Macro-voxeles locales para guia D*

- Cada `NavigationProfile` incorpora `coarse_voxel_factor=4` por defecto. El
  `VoxelMapWorker` deriva `NavigationCoarseCell` de bloques `4x4x4`, con coste
  medio, fraccion bloqueada y transitabilidad; publica solo los macro-deltas
  afectados por cambios finos.
- El macro-mapa no sustituye ni relaja la capa fina: es exclusivamente una guia
  para D*. La evidencia raw, inflacion, mascaras diagonales y decisiones de
  seguridad siguen bajo la autoridad de `ReversibleVoxelMap` fino.
- Build de `task_lib` y `task_server` correctos. CTest finales: 7/7 y 7/7.
- Prueba 628 no llego a ejecutar escenario porque el YAML relativo no era
  visible desde `scenario_runner_node`; queda registrada como fallo mecanico de
  infraestructura. La 629 uso la ruta absoluta y ejercio macro-deltas reales
  sin reconstruir el mapa completo.

Conclusion: CONSEGUIDA para la guia gruesa incremental por perfil. 6D conserva
el estado agregado PARCIAL solo por depth metrico real, aplazado a Fase 8.

## 2026-09-11 - Delta global coalescido e influencia navegable incremental

- cambio: `GlobalMapBuilder` produce upserts/deletes por identidad estable y
  `global_map_server` los publica en `/global_sparse_map_delta`. El snapshot
  compatible se usa solo como bootstrap; `VoxelMapWorker` coalesce 100 ms y es
  el unico escritor de evidencia voxel, FREE y capa navegable.
- cambio: `ReversibleVoxelMap::ApplySparseDelta` evita sustituir la fuente
  sparse completa. Los perfiles navegan con conteos inversos de influencia
  OCCUPIED, de modo que una celda raw modifica solo la vecindad inflada que
  realmente afecta, sin volver a comprobar el volumen por cada candidata.
- prueba 664: D1 se anclo con GT en fiducial 2 y el escenario termino
  `SIM-DONE success=true`. Un lote de 828 cambios raw actualizo 50.886 celdas
  navegables en 2.143 s; lotes de 2--12 cambios tardaron 14--89 ms. Las pruebas
  anteriores de rafagas comparables registraban entre 13 y 45 s.
- limite: la produccion del builder aun materializa la vista compatible para
  cloud/visualizacion y el consumidor no ha ejercido una resincronizacion por
  snapshot real; la mejora comprobada es la aplicacion incremental del worker.

Conclusion: PARCIAL. El camino incremental de consumo y la reduccion de
latencia estan CONSEGUIDOS; depth metrico y la prueba de resincronizacion real
permanecen pendientes.

## 2026-09-12 - Soporte sparse mínimo de cuatro fuentes (prueba 678)

- `ReversibleVoxelMap` conserva candidatos sparse cualificados por identidad y
  por voxel. Solo agrega `OCCUPIED` cuando reúne cuatro `MapPointId` distintos
  con score `>= 0.2`; republicar no infla el conteo y move, delete o cambio de
  score lo revierten.
- La regresión cubre tres fuentes sin ocupación, la activación de la cuarta, la
  repetición idempotente y la retirada por movimiento. `task_lib` pasó 9/9 y
  `task_server` 7/7 tras compilar ambos paquetes.
- La prueba 678 arrancó con `min_occupied_mappoints=4`, ancló D1 con GT en el
  fiducial 2 y terminó `SIM-DONE success=true`. Acredita propagación y flujo
  integrado, no depth métrico ni una resincronización global real.

Conclusión: el filtro de soporte sparse queda CONSEGUIDO dentro de 6D. El
estado agregado sigue PARCIAL por depth métrico y resync global ejercido.
