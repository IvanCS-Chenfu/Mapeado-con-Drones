# Historial 3S - Perfil de debug

## 2026-08-22 - Implementacion y prueba 196

- objetivo intentado: ejecutar el mapa sparse global sin RViz2, grafo web,
  navegador ni telemetria `[F3*]` en terminal, conservando errores reales;
- archivos modificados: `fase3_debug.yaml`, `multi_dron.launch.py`, launch del
  servidor, test contractual, trayectoria corta y documentacion;
- comportamiento: cuatro booleanos launch con default false; el servidor usa
  nivel ROS `ERROR` cuando `fase3_logs_terminal=false`;
- paquetes compilados: `orbslam3_multi`, `orbslam3_server`,
  `simulacion_dron`;
- resultado de build: 3/3, exit 0, 18.4 s;
- regresiones: CTest 9/9, 10/10 y 8/8; contratos configuracion/web 15/15;
- prueba Gazebo: 196 con `prueba_debug_fase3_silencioso.yaml`, 143 s incluidos
  45 s de drenaje, scenario/tool exit 0 y `success=true`;
- patrones usados: scenario runner, lifecycle de servidor/RViz2/web,
  marcadores `[F3*]`, errores, cierre y recursos;
- evidencia positiva: cinco pasos, cuatro goals `success=true`, servidor
  arrancado/terminado limpio y cambios de backpressure que demuestran
  procesamiento activo;
- observabilidad apagada: cero marcadores `[F3*]`, ningún proceso RViz2,
  bridge o navegador, `max_rviz_rss_mib=0.0` y `max_web_rss_mib=0.0`;
- recursos: servidor RSS 99.2 MiB, grupo 893.6 MiB, PSI memoria 0,
  MemAvailable minima 5146.4 MiB y guarda inactiva;
- evidencia negativa: Gazebo informa exit 255 durante cleanup posterior a
  `SIM-DONE`; no afecta escenario ni servidor;
- conclusion: `CONSEGUIDA`;
- siguiente paso recomendado: mantener defaults false y habilitar cada vista o
  los logs solo durante diagnosticos concretos.
