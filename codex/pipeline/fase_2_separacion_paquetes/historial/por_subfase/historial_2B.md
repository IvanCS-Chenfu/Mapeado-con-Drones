# Historial 2B

## 2026-08-24 - Build aislado paquete a paquete

- Dron: `orbslam3_msgs`, `lib_tray`, `ORB_SLAM3`, `orbslam3`, `dron_individual`;
- Servidor: `orbslam3_msgs`, `orbslam3_multi`, `orbslam3_server`;
- Simulación: `simulacion_dron`;
- resultado de build: 9/9 invocaciones, exactamente un paquete, exit 0;
- CTest: 4/4, 9/9, 10/10 y 9/9 en las suites con pruebas;
- limitación: `dron_individual` conserva deuda legacy global de linters; todo
  archivo tocado pasó comprobación focal y rebuild;
- conclusión: `CONSEGUIDA`.
