# Historial 3K

Este índice de subhistoriales resume y organiza el historial de la subfase `3K`.
Las entradas se han separado en archivos numerados para que el historial sea más fácil de leer y mantener.

## Nota de reclasificación 2026-07-14

Parte de lo probado durante `3L` pertenece realmente a `3K`: backup previo al
apply, escritura en `GlobalPoseStore`, poses propagadas, publicación de nube
tras apply, proyección de MapPoints desde KFs finales y corrección heredable para
KFs futuros. `3L` solo valida ese estado y decide si se conserva o se restaura.

- `historial_3K_1.md`: 2026-07-10 — Apply seguro en `GlobalPoseStore`, publicación de correcciones, raw intacto y primeros `partial_candidate=true` como deuda para `3L`.
- `historial_3K_2.md`: 2026-07-11 — Revalidación del apply con preservación de anclajes y checks adicionales de integridad antes de escribir en `GlobalPoseStore`.
- `historial_3K_3.md`: 2026-07-23 a 2026-07-28 — Hotfix de MapPoints por cobertura y autoridad `accepted`/`active_tail_anchor`/`derived_tail`; `prueba_41` y RViz2 validan dos applies y cierran `3K`.
- `historial_3K_4.md`: 2026-07-28 — Resultado de `prueba_42` sobre el runtime historico, hoy sustituido por el worker unico.
- `historial_3K_5.md`: 2026-07-28 — Controles fiduciales de cola pendiente y `prueba_43`; los KFs llegados durante el solver se refinan hasta el último control.
- `historial_3K_6.md`: 2026-07-28 — Reconciliación de KFs tardíos dentro de ventana y `prueba_44`; nueve KFs omitidos por el snapshot inicial se refinan y aparecen en HTML/publicación.
- `historial_3K_7.md`: 2026-07-28 — Confirmación visual final en RViz2; no quedan KFs/MapPoints en poses previas y `3K` se cierra como conseguida.
- `historial_3K_8.md`: 2026-08-05 — Sustitucion por un unico worker priorizado, flujo principal no bloqueante y validacion integrada en `prueba_73-76`.
- `historial_3K_9.md`: 2026-08-14 - Reimplementacion del commit atomico,
  propagacion de ventana/tail y pruebas 142-146.
- `historial_3K_10.md`: 2026-08-14 - Continuidad atomica para KFs llegados
  despues del commit, replays 149/150 y live 151.

Leer primero este resumen antes de abrir cualquiera de los subhistoriales.
