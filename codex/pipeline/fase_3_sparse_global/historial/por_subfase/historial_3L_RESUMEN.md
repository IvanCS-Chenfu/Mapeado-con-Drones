# Historial 3L - resumen

## Estado vigente

`CONSEGUIDA`: validacion integral cerrada tras corregir continuidad, reserva de
control y observabilidad. Live 151 cumple los criterios tecnicos y el usuario
confirma el resultado visual.

## Estado actual

- Valida finitud, cobertura de controles, control fijo y error absoluto del
  target antes del commit.
- Decide `AcceptFull`, `AcceptPartialRetry` o `HardFailure`.
- El worker conserva task/ownership durante refinamientos y activa mission gate.
- Fallo numerico, hard movido o incoherencia persistente no hacen commit y son
  bloqueantes.

## Pruebas vigentes

- 142: `NO CONSEGUIDA`, 5 commits y 5 controles invertidos.
- 143: `NO CONSEGUIDA`, 7 commits y 3 fallos de revision/inactivos.
- 144: `CONSEGUIDA` automatica, 10 commits y cero fallos.
- 145: `NO CONSEGUIDA` integral; conserva 79 observaciones, 44 tareas, 30 full,
  14 stale y cero hard, pero falla continuidad futura y la observabilidad
  visual acordada.
- 146: replay v3 reproduce exactamente 44/30/14/0 y usa 131 MiB RSS de
  servidor.
- 148: `NO CONSEGUIDA`, carrera KF149/KF150, hard constraint y timeout con
  recursos sanos; se conserva como intento fallido.
- 149: replay bueno, 7/3/4/0 y cola vacia.
- 150: replay de regresion de 148, un commit control149 y cero hard.
- 151: `CONSEGUIDA tecnicamente`, 11 tareas, 3 commits, 8 stale, cero hard,
  pending0, seis submapas y recursos sanos.

Detalle nuevo: `historial_3L_11.md`.
