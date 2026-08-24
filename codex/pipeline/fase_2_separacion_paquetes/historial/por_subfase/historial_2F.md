# Historial 2F

## 2026-08-24 - Visualización arquitectónica

- estático: health ready, telemetría false y assets HTTP 200;
- live aislado: evento real en `/system_architecture/activity`, secuencia 1;
- `pipeline_flow` aislado: health ready, HTML 200 y cierre limpio;
- prueba 200: ambos bridges ready/live, navegadores abiertos y RViz2 activo;
- cierre: los bridges terminan limpiamente, sin el `ValueError` de prueba 198;
- conclusión inicial: `CONSEGUIDA TECNICAMENTE`.

## 2026-08-24 - Redistribución y cierre visual

- cambio: layout declarativo con Simulación/Servidor arriba y Dron abajo;
- archivos: `graph_layout.js`, `app.js`, `index.html` y test contractual;
- build: `simulacion_dron` 1/1, exit 0;
- tests: CTest 9/9, incluido el contrato espacial;
- capturas: 1440x900 y 820x1000 inspeccionadas sin solapes ni recortes;
- simulación: no repetida por acuerdo, al no cambiar telemetría ni lógica runtime;
- conclusión agregada: `CONSEGUIDA`.
