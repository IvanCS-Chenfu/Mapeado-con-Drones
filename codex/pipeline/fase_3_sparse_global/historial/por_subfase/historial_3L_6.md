# Historial 3L.6 - Diagnostico largo concurrente

## 2026-07-28 - Prueba 42

La prueba típica larga ejecutó dos drones alrededor del edificio en sentidos
opuestos, desde fiducial 2 hasta fiducial 1 y de vuelta a fiducial 2.

### Resultado post-apply

| Tarea | Target | Error traslación | Error yaw | Media GT ventana | Validación |
|---|---|---|---|---|---|
| 1, dron 1, fid 1 | KF 203 | `0.502755 -> 0 m` | `0.038615 -> 0 rad` | `0.314690 -> 0.208900 m` | `ACCEPT` |
| 2, dron 2, fid 1 | KF 156 | `27.247778 -> 0 m` | `3.078937 -> 0 rad` | `8.493385 -> 1.388419 m` | `ACCEPT` |
| 3, dron 2, fid 2 | KF 354 | `0.405740 -> 0 m` | `0.059753 -> 0 rad` | `0.255687 -> 0.289270 m` | `ACCEPT` |

En las tres tareas:

```text
real_after_t=0
hard_fixed_moved=false
raw_db_modified=false
invalid_pose_skipped_before=0
invalid_pose_skipped_after=0
server_corrected_missing_kf_after=0
propagation_discontinuity_max_t=0
propagation_discontinuity_max_yaw=0
```

Durante los 300 s posteriores al escenario la nube se estabilizó en 63.737
puntos, 55.931 corregidos por el servidor, sin poses inválidas ni KFs de
referencia corregidos ausentes.

### Observaciones

- `task_id=2` partía de una trayectoria raw muy degradada. Redujo la media GT
  de forma fuerte, pero dejó `strong_edges_broken=0` y
  `deformable_edges_broken=1`, con máximo 2.918214 m. Esta aceptación es
  coherente con la política actual, pero conserva una deuda de calidad en
  `3I/3J`.
- `task_id=3` llevó el target a cero y redujo el máximo GT
  `1.081490 -> 0.814681 m`, aunque la media subió ligeramente. También queda
  como dato para revisar pesos, no como fallo concurrente.
- El dron 1 entró en `tracking_state=3` y el wrapper detectó cambios/reset de
  mapa. Terminó en `epoch=3`; el fiducial 2 creó el primer anchor de ese nuevo
  submapa y su error posterior fue solo 0.022152 m.

### Conclusion

Los commits, propagación y publicación observables por log fueron coherentes y
no reapareció `invalid_pose_skipped`. La prueba queda `PARCIAL` como cierre de
`3L`: falta inspección RViz2 de `prueba_42` y una ejecución en la que ambos
drones mantengan su `map_epoch` hasta la segunda revisita.
