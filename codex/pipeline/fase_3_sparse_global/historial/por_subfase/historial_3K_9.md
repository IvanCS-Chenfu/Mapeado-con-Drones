# Historial 3K_9 - Commit atomico reimplementado

## 2026-08-14 - Subfase 3K - Ventana, late-window y tail

- objetivo intentado: comprometer un batch atomico de poses y notificar solo
  KFs movidos al builder, sin publicar desde el worker secundario.
- archivos modificados: `global_pose_types.hpp`, `global_pose_store.hpp/.cpp`,
  `sparse_global_backend.hpp/.cpp`, worker servidor y tests.
- resultado de build: build final correcto. Se preservan los fallos mecanicos
  de compilacion por una expectativa legacy y dos usos iniciales de
  `operator!=` no definido.
- pruebas 142/143: `NO CONSEGUIDAS`. La 142 revelo `PoseChangeSet` calculado
  contra el registro ya sobrescrito y controles invertidos; la 143 confirmo
  moved KFs, pero una carrera de revision y KFs inactivos dejaron tres fallos.
- correcciones: copia previa antes de commit, orden temporal, ventana que omite
  inactivos, commit que no los reactiva y retry acotado de propuesta stale.
- prueba 144: `CONSEGUIDA`; 10/10 commits full, error cero, moved KFs positivos,
  cero hard failures y backpressure liberado.
- live 145, evidencia automatica inicial: 30 commits full, 282 KFs movidos
  acumulados, 9 commits con tail, tail maximo 4, ventana maxima 128, moved
  maximo 115 y cero fallos. El siguiente `PrimaryInput` recalcula KFs/MPs dirty.
- replay v3 146: reproduce exactamente 30 commits, 14 `STALE`, 282 moved y
  tail acumulado 14 sobre 496 entradas.
- revision visual y causal de live 145: el tail que existe al hacer commit si
  se reancla. Los KFs que llegan despues pasan por `ApplyRawPoseChanges()` y se
  calculan con el anchor original. Por eso KF158 queda corregido por task27,
  pero KF159-KF162 reaparecen con unos 0.9 m; el mismo patron llega a unos
  4.0 m en KF273/KF274/KF277 de `(2,2)`.
- hueco de tests: la prueba sintetica inserta KF10/KF11 antes de
  `CommitFiducialProposal()` y solo valida tail concurrente; no inserta un KF
  nuevo despues del commit.
- conclusion revisada: `NO CONSEGUIDA`; atomicidad y tail presente funcionan,
  pero no se cumple la propagacion indefinida de KFs posteriores acordada.
- siguiente paso recomendado: guardar atomicamente una transformacion de
  continuidad por submapa/control y aplicarla a cada delta futuro; añadir test
  del primer KF posterior al commit y repetir replay/live.
