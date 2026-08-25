# Historial 4E

## 2026-08-25 - Implementacion y pruebas de componente

- objetivo intentado: transportar observaciones visuales validas por KF desde cada wrapper;
- archivos modificados: interfaces `orbslam3_msgs`, `StereoSlamNode`, grafos, metadata y contratos;
- paquetes compilados: ambas copias de `orbslam3_msgs`, `orbslam3`, `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`;
- resultado de build: correcto en todos los paquetes;
- pruebas: contratos 39/39; CTest wrapper 1/1, backend 9/9, servidor 11/11 y Simulacion 10/10;
- evidencia positiva: replicas exactas, batch no vacio, tags validos/ordenados, timestamp comun y quaternion normalizado;
- conclusion: PARCIAL hasta validacion live.

## 2026-08-25 - Prueba 209

- objetivo intentado: trayectoria tipica con Gazebo, RViz2 y ambos grafos;
- evidencia negativa: `scenario_runner_node` rechazo la ruta YAML relativa con `bad file`;
- evidencia positiva: launch, wrappers, servidor, RViz2 y bridges arrancaron; recursos estables;
- conclusion: NO CONSEGUIDA como prueba integral; repetir con ruta absoluta.

## 2026-08-25 - Prueba 210

- objetivo intentado: repetir la trayectoria tipica completa con ruta absoluta;
- pruebas Gazebo/replay: 17 pasos correctos, scenario exit 0, `SIM-DONE success=true` y recursos estables;
- patrones reducidos: `FID-BATCH-PUB|FID-SYNC|SCENARIO-RUNNER|SYSTEM-ARCH|FLOW-WEB|SIM-*`;
- evidencia positiva: 68 batches publicados, 33 del dron 1 y 35 del dron 2; 11 batches multitag; 68/68 matches;
- limitacion: `system_architecture` estuvo visible pero `mode=static` por omitir la bandera de telemetria;
- conclusion: PARCIAL para el criterio web live; transporte funcional conseguido.

## 2026-08-25 - Prueba 211

- objetivo intentado: verificar la arista live sin repetir toda la trayectoria;
- prueba Gazebo: primer tramo tipico de ambos drones hasta `(0,-10)`, scenario exit 0 y `SIM-DONE success=true`;
- evidencia positiva: ambos grafos `mode=live`, 18 batches y 18 matches exactos de ambos drones;
- evidencia negativa: solo Gazebo exit 255 durante cleanup, incidencia conocida posterior al escenario;
- conclusion: CONSEGUIDA; contrato y transporte 4E validados end-to-end.
