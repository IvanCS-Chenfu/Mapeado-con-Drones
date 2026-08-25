# Historial 4F

## 2026-08-25 - Implementacion y pruebas de componente

- objetivo intentado: asociar cada batch visual con su KF raw exacto sin depender del orden de llegada;
- archivos modificados: tipos/backend `orbslam3_multi`, `global_map_server`, runtime YAML, grafos y tests;
- resultado de build: `orbslam3_multi`, `orbslam3_server` y `simulacion_dron` correctos;
- pruebas unitarias: backend 9/9, servidor 11/11 y Simulacion 10/10;
- evidencia positiva: ambos ordenes, primer `arrival_id`, FIFO/aislamiento por dron, duplicate/conflict y no reactivacion cubiertos;
- conclusion: PARCIAL hasta simulacion completa.

## 2026-08-25 - Prueba 209

- objetivo intentado: validacion integral de sincronizacion;
- evidencia negativa: escenario no comenzo por ruta YAML relativa invalida;
- conclusion: NO CONSEGUIDA como prueba 4F; no aporta conteos funcionales.

## 2026-08-25 - Prueba 210

- objetivo intentado: trayectoria tipica completa y carga real de batches fuera de orden;
- evidencia positiva: 68 pending y 68 matches `source=pending`; pico 7/10, cero evicted, duplicate, conflict o reject;
- trafico: 33 batches dron 1, 35 dron 2 y 11 multitag; todos consumidos;
- limitacion: pipeline flow live, pero system architecture quedo estatico por bandera de prueba omitida;
- conclusion: PARCIAL solo para evidencia arquitectonica live; sincronizacion 4F conseguida.

## 2026-08-25 - Prueba 211

- objetivo intentado: comprobar la arista wrapper→Servidor con telemetria arquitectonica activa;
- evidencia positiva: `SYSTEM-ARCH-READY mode=live`, pipeline flow live, subscriptions capacidad 10 y 18/18 matches; pico 5/10 y cero incidencias 4F;
- recursos/cierre: scenario y herramienta exit 0, guarda inactiva; Gazebo 255 solo en cleanup;
- conclusion: CONSEGUIDA; sidecar, handoff y observabilidad 4F validados.
