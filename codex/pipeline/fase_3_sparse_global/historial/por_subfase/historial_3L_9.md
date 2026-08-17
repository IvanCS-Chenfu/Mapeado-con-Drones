# Historial 3L — cierre de validación post-apply

## 2026-07-28 — RViz2 correcto tras `prueba_44`

- objetivo:
  completar la evidencia humana pendiente de la última corrección de KFs
  llegados durante el solver.
- archivos modificados:
  solo documentación de cierre.
- paquetes compilados y pruebas:
  no se repitieron; la evidencia automática procede de `prueba_44`.
- patrones/logs:
  `SCENARIO-RUNNER-DONE`, `SIM-EXIT-CODE`, `[F1L-POST-APPLY-ACCEPT]`,
  aristas rotas, poses inválidas e `invalid_pose_skipped`.
- evidencia automática:
  `prueba_44` termina con `SCENARIO-RUNNER-DONE success=true`,
  `SIM-EXIT-CODE 0`, apply `ACCEPT`, cero aristas rotas, cero poses inválidas y
  nube estable durante 300 s.
- evidencia visual:
  el usuario examinó RViz2 y considera correctos los resultados finales. Ya no
  observa el KF aislado ni los MapPoints en la zona correspondiente a su pose
  no optimizada.
- conclusión:
  `3L CONSEGUIDA Y CERRADA`. Las incidencias anteriores permanecen en
  `historial_3L_1.md` a `historial_3L_8.md` como diagnóstico histórico.
- siguiente paso:
  ejecutar la regresión integrada `3M -> 3N -> 3O`.
