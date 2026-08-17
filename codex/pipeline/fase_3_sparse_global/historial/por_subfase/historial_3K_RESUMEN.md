# Historial 3K - resumen

## Estado vigente

`CONSEGUIDA`: continuidad futura validada en tests, replays 149/150, live 151 y
confirmacion visual del usuario; los KFs posteriores no vuelven a su pose
previa.

## Estado actual

- `CommitAcceptedPoses()` valida revisiones y hard antes de aplicar un unico
  batch/revision.
- El patch cubre controles, no controles activos, late-window compatible y KFs
  posteriores al target que ya existen al entrar en commit.
- `ContinuationRecord` conserva control, transformacion y revision. Los IDs
  posteriores derivan `world_pose` desde el ultimo control, manteniendo
  `raw_world_pose` bajo el anchor inicial.
- Un conflicto de revision descarta la propuesta y revalida/reconstruye de
  forma acotada; nunca deja estado parcial.
- `ACCEPT_FULL` promueve target a ultimo control. Un parcial conserva el
  control anterior.
- Solo KFs realmente movidos llegan dirty al builder; el siguiente flujo
  principal recalcula sus MPs y publica.

## Evidencia

- Pruebas 142/143: fallos preservados y corregidos.
- Replay 144: 10/10 commits full.
- Live 145/replay 146: 30 commits, 282 KFs movidos acumulados, 9 commits con
  tail, ventana maxima 128, moved maximo 115 y cero fallos.
- En `(1,0)`, task27 corrige KF157 y su tail KF158 queda stale, pero KF159 llega
  despues con 0.900 m de error; KF160-KF162 repiten aproximadamente 0.88-0.91 m.
- En `(2,2)`, KF272 se corrige y KF273 llega despues con 3.984 m; KF274 y KF277
  repiten alrededor de 4.0 m. En cambio, KF275/KF276 presentes durante task41
  se propagan y quedan stale con 0.053/0.056 m.
- Replay 149 corregido: 7 tareas/3 commits/4 stale/0 hard; KFs posteriores de
  `(1,0)` quedan en 0.183-0.248 m y de `(2,2)` en 0.019-0.190 m.
- Live 151: commit control187 mueve116 KFs y KFs194-209 posteriores quedan en
  0.081-0.239 m; cero hard y cola vacia.

Detalle nuevo: `historial_3K_10.md`; el indice `historial_3K.md` enlaza el legado.
