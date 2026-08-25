# Historial 4C

## 2026-08-25 - Evento exacto de KeyFrame

- objetivo intentado: asociar deterministicamente cada KF nuevo con su imagen y calibracion efectivas;
- archivos modificados: `ORB_SLAM3/{System,Tracking}` y consumo en `orbslam3_ros2`;
- paquetes compilados: target nativo `ORB_SLAM3` y paquete `orbslam3`;
- resultado de build: correcto;
- pruebas Gazebo/replay: 204 aporto arranque parcial y 205 ejecucion runtime hasta la interrupcion;
- evidencia positiva: eventos one-shot con identidad exacta y `timestamp_delta=0`; el detector proceso los KFs asociados;
- evidencia negativa o ausente: 203 no arranco y 204 no recorrio la trayectoria por errores externos a 4C;
- conclusion: CONSEGUIDA;
- siguiente paso recomendado: consumir el recibo como unica fuente de identidad en las subfases posteriores.
