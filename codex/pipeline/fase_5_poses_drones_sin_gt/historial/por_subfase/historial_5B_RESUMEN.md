# Historial 5B - resumen

## Estado

```text
Conclusion: CONSEGUIDA
Prueba de cierre: 225
Siguiente bloque: preparar 5C+5D+5E+5F
```

5B añade una muestra `NavigationState` coherente con el mismo
`TrackStereo`: tracking, epoch, reference KF real, `Tcr`, `O_T_B` y validez
explícita. Los campos globales y de velocidad permanecen inválidos. El gate de
`gen_tray` rechaza absolutos sin global, acepta relativos con estado fresco y
congela epoch/muestra al aceptar; el control efectivo sigue usando GT legacy.

Builds correctos: ambas copias de `orbslam3_msgs`, target core `ORB_SLAM3`,
`orbslam3`, `dron_individual` y `simulacion_dron`. Tests nuevos de continuidad
y goals correctos; `simulacion_dron` 10/10. La suite global de
`dron_individual` conserva deuda legacy de linters, pero el test y formato
nuevos pasan de forma aislada.

La prueba 225 confirma dos anchors hard, cambios de reference KF con paso cero,
rechazo absoluto, snapshots relativos y pérdida real 2->3->0->1 de ambos
drones tras girar 180 grados, sin continuidad ficticia. Los fallos 221/223 del
spawner y las ejecuciones 222/224 con telemetría insuficiente se conservan en
el historial largo. No repetir `fase3_logs_terminal`; el argumento correcto es
`debug_fase3_logs_terminal`.
