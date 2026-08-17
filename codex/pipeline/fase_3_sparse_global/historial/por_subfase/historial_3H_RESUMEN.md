# Historial 3H - resumen

## Estado vigente

`CONSEGUIDA`: queue/worker, revalidacion y lifecycle persistente por tarea
quedan validados tecnica y visualmente en live 151.

## Estado actual

- Observaciones live/replay llevan `fiducial_visit_id`; record v3 lo persiste y
  replay v1/v2 infiere visitas temporalmente.
- Primer anchor conserva el flujo 3E. Una revisita calcula error de traslacion,
  rotacion y yaw; cada KF distinto fuera de umbral encola MAX.
- `SecondaryTaskQueue` usa MAX/HIGH/NORMAL, FIFO por carril, dedup exacto y un
  worker global no preemptivo.
- El dequeue revalida y termina `STALE` si otra optimizacion ya corrigio el KF.
- Backpressure se activa durante optimizacion y por watermarks 64/16 sin
  cancelar el goal activo.
- El primer KF observado reserva el control de la visita; otro KF coherente no
  puede adelantarse mientras el primero se optimiza.
- El servidor emite lifecycle `start/done` y el frontend mantiene el camino
  secundario acumulado por `task_id` hasta finalizar.

## Evidencia

- 49/49 C++ y 9/9 web.
- Replay 144: 10 commits, cero fallos.
- Live 145 y replay v3 146: 44 tareas, 30 commits, 14 `STALE`, cero fallos y
  `max_active=1`.
- La observacion visual confirma que cada arista se apaga tras 240 ms. No hay
  estado persistente desde `SECONDARY-START` hasta `SECONDARY-DONE`.
- Los 30 commits fueron reales: 13 en `(1,0)`, 10 en `(2,0)` y 7 en `(2,2)`;
  varios no habrian sido necesarios con propagacion futura correcta.
- Pruebas 142/143 fallidas se conservan; motivaron orden temporal, filtrado de
  inactivos y retry de revisiones.
- Live 148 fallida se conserva: la carrera KF149/KF150 produjo hard constraint
  y timeout. Replay 150 demuestra la correccion con un commit y cero hard.
- Live 151: 11 tareas, 3 commits, 8 stale, cero hard, pending0 y escenario
  completo; el usuario confirma visualmente que todo funciona correctamente.

Detalle: `historial_3H.md`.
