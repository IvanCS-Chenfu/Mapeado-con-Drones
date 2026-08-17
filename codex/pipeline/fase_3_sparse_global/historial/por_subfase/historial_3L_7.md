# Historial 3L.7 - Validación de cola pendiente

## 2026-07-28 - `prueba_43`

`prueba_43` repite el rodeo largo multi-dron tras añadir controles fiduciales
de cola pendiente en `3K`.

Evidencia post-apply:

```text
task 2: 20.430510 -> 0 m
pending controls=4
pending refined=7
pending derived=1
active_tail_anchor=KF250
internal_edges_broken=0
strong_edges_broken=0
deformable_edges_broken=0
invalid_pose_skipped=0
rollback=0
```

Los controles `KF245`, `KF246`, `KF247` y `KF250` tienen error de posición y
orientación nulo después del apply. El HTML post-commit incluye `KF243-251` y
la nube final permanece estable con 64.335 puntos durante 300 s.

La evidencia técnica es positiva, pero `3L` permanece `PARCIAL/REABIERTA` hasta
la revisión visual del usuario. Además, el dron 1 cambió de epoch y la ruta de
ventanas solapadas no se ejecutó.
