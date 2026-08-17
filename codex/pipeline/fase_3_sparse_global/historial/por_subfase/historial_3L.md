# Historial 3L

Este índice de subhistoriales resume y organiza el historial de la subfase `3L`.
Las entradas largas se han dividido en varios archivos para facilitar la lectura y evitar archivos monolíticos.

## Reclasificación 2026-07-14

Durante la ejecución de `3H` a `3L` se hicieron pruebas técnicas con nombres
`f1l_*` que no pertenecen funcionalmente a `3L`. Para leer este historial:

- lo que habla de selección de vértices, vecindad fiducial, splits de aristas,
  cobertura del grafo y soporte por KFs pertenece a `3I`;
- lo que habla de solver, rigidez, pesos, `RunDryRunGraphOnly`, replay offline,
  dump TSV o HTML diagnóstico pertenece a `3J`, aunque el archivo se llame
  `f1l_graph_*` o `f1l_debug_animation_*`;
- lo que habla de escritura en `GlobalPoseStore`, poses optimizadas/propagadas,
  publicación de MapPoints desde KFs finales y corrección heredable pertenece a
  `3K`;
- `3L` queda limitado a validación y diagnóstico post-apply: logs, GT debug,
  checks de propagación/global map, decisión `ACCEPT`, `PARTIAL` o
  `REJECT_ROLLBACK`.

- `historial_3L_1.md`: 2026-07-10 a 2026-07-12 00:54 — validación post-apply, rollback, pruebas iniciales de cargo/propagación, métricas de grafos y primeras hipótesis de rigidez.
- `historial_3L_2.md`: 2026-07-12 23:06 a 2026-07-13 00:14 — exploraciones de rigidez angular, soporte de KeyFrames y diagnósticos offline con dumps y HTML.
- `historial_3L_3.md`: 2026-07-13 00:27 a 2026-07-13 13:18 — pruebas de fiduciales y aristas, diseño de target/cola, restauración de líneas activas y primeros enfoques de vecindades fiduciales.
- `historial_3L_4.md`: 2026-07-14 a 2026-07-14 13:40 — consolidación de vecindades fiduciales, herencia de MapPoints, publicación final y ajustes de poses inducidas.
- `historial_3L_5.md`: 2026-07-27/28 — loop interno de ORB-SLAM3 desactivado y `prueba_41` live fiducial 2 -> 1 -> 2; logs/HTML correctos y validación RViz2 perfecta del usuario. `3L` queda conseguida.
- `historial_3L_6.md`: 2026-07-28 — `prueba_42` larga multi-dron: tres applies coherentes por log y nube estable; queda parcial por reset del dron 1, una arista deformable rota y revision RViz2 pendiente.
- `historial_3L_7.md`: 2026-07-28 — `prueba_43` valida por logs la cola llegada durante el solver; la inspección que entonces faltaba queda superada por el cierre posterior.
- `historial_3L_8.md`: 2026-07-28 — `prueba_44` valida por logs/HTML nueve KFs tardíos dentro de ventana; la validación RViz2 final está en `historial_3L_9.md`.
- `historial_3L_9.md`: 2026-07-28 — El usuario valida el resultado final en RViz2, sin KFs aislados ni MapPoints en poses no optimizadas; `3L` queda conseguida y cerrada.
- `historial_3L_10.md`: 2026-08-14 - Validacion privada reimplementada y
  pruebas 142-146; live 145 revisada como no conseguida por continuidad futura.
- `historial_3L_11.md`: 2026-08-14 - Correcciones de continuidad/control/web/color,
  live 148 fallida, replays 149/150 y live 151 tecnicamente conseguida.

Leer primero este resumen antes de abrir cualquiera de los subhistoriales.

## 2026-07-28 — Contrato actualizado para usar resumen historico

- objetivo:
  alinear `subfase_3L.md` con la nueva politica de historiales por resumen.
- archivos modificados:
  `subfase_3L.md`, `historial_3L_RESUMEN.md` e historial.
- cambio:
  el contrato de `3L` ya indica leer primero
  `historial_3L_RESUMEN.md`; el historial largo queda como detalle bajo demanda.
- build y pruebas:
  no ejecutadas; cambio documental sin codigo.
- conclusion:
  sin cambio funcional: `3L` permanece `CONSEGUIDA Y CERRADA`.
