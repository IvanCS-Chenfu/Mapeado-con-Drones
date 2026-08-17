# Historial 3K — cierre final del apply fiducial

## 2026-07-28 — Validación visual final y cierre

- objetivo:
  cerrar `3K` después de reconciliar KFs tardíos dentro de la ventana y KFs
  posteriores al target.
- archivos modificados:
  solo documentación de cierre; la implementación ya quedó registrada en
  `historial_3K_3.md` a `historial_3K_6.md`.
- paquetes compilados y pruebas:
  no se repitieron en este cierre documental; se reutiliza el build, test local
  y simulación `prueba_44` ya documentados.
- patrones/logs:
  `[F1K-LATE-WINDOW-REFINE]`, `[F1K-RAWDB-NOT-MODIFIED]`,
  `[F1L-POST-APPLY-ACCEPT]`, `invalid_pose_skipped` y errores graves.
- evidencia técnica heredada de `prueba_44`:
  build y test local correctos; nueve KFs tardíos detectados y refinados, cero
  skips, target `20.133619 -> 0 m`, hard fiducial inmóvil, apply `ACCEPT`,
  HTML poscommit con los 95 KFs del intervalo e
  `invalid_pose_skipped=0` tras 300 s.
- evidencia visual:
  el usuario confirmó en RViz2 que no quedan KFs ni MapPoints en la pose previa
  a la optimización y que el mapa final se ve correctamente.
- runtime historico:
  `prueba_42` conserva evidencia de tres applies. Ese scheduling fue sustituido
  por el worker unico documentado en `historial_3K_8.md`.
- invariantes preservados:
  `RawMapDatabase` sigue cruda; las poses aceptadas se escriben únicamente en
  `GlobalPoseStore`; la autoridad `accepted`/`server_optimized` no vuelve a
  convertirse en `derived_tail`.
- conclusión:
  `3K CONSEGUIDA Y CERRADA`.
- siguiente paso:
  revalidar `3M`, después `3N` y continuar `3O`.
