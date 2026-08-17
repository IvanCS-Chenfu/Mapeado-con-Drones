# Historial 3L_10 - Validacion privada y cierre tecnico

## 2026-08-14 - Subfase 3L - Validacion end-to-end 3H-3L

- objetivo intentado: validar propuestas antes de commit, soportar parcial
  seguro, rechazar fallos duros y demostrar el flujo completo live/replay.
- archivos modificados: `optimization_validator.hpp/.cpp`, worker secundario,
  backpressure, logs, tests, launches y topologia web 16/25.
- tests: 45/45 C++ y 8/8 web.
- prueba 142: `NO CONSEGUIDA`; 5 commits y 5 fallos por control invertido.
- prueba 143: `NO CONSEGUIDA`; 7 commits, 1 conflicto de revision y 2 ventanas
  con pose intermedia ausente; `blocking_failure=true`.
- prueba 144: `CONSEGUIDA` automatica; 10 commits `ACCEPT_FULL`, target a cero,
  cero fallos y cierre `processed=10 committed=10 max_active=1`.
- live 145: scenario completo fid2-fid1-fid2, 79 observaciones, 44 tareas, 30
  `ACCEPT_FULL`, 14 `STALE`, cero hard failures, cola vacia al cierre y
  `max_active=1`. Backpressure se activa/libera alrededor de las optimizaciones.
- recursos live: 412 s, server RSS maximo 173.1 MiB, grupo 1492.5 MiB,
  MemAvailable minimo 4675.1 MiB, PSI memoria cero y guard inactivo.
- replay 146: record v3 de 496 entradas/7 submapas/517 KFs/58268 MPs; reproduce
  exactamente 44 tareas, 30 commits, 14 stale y cero fallos. Server RSS maximo
  131 MiB.
- observacion visual incorporada: ambos drones generaron nuevos epochs; sus
  colores parecen por dron porque epochs consecutivos cambian solo 1-7 grados
  de tono. El flujo secundario parpadea porque cada evento dura 240 ms y no
  existe estado de tarea persistente.
- revision causal: los 30 commits fueron reales. `CommitFiducialProposal()`
  reancla el tail presente, pero el siguiente `PrimaryInput` crea KFs nuevos
  con el anchor inicial. Los errores repetidos de aproximadamente 0.9 m en
  `(1,0)` y 4.0 m en `(2,2)` confirman lo visto en RViz2.
- por que paso la validacion automatica: comprueba target, commit y tail ya
  presente, no el primer KF que entra despues del commit. El replay reproduce
  los mismos commits y por ello tambien oculta el defecto bajo correcciones
  fiduciales sucesivas.
- conclusion revisada de live 145: `NO CONSEGUIDA`; los resultados objetivos
  y recursos se conservan, pero no se cumple la continuidad ni la claridad
  visual. Estado agregado 3H-3L: `NO CONSEGUIDA`.
- siguiente paso recomendado: acordar correccion y nueva prueba que compruebe
  explicitamente un KF posterior sin necesitar otra optimizacion.
