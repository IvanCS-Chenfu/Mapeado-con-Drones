# Historial de la subfase 1K

## 2026-09-01 - Subfase 1K - limpieza y cierre

- objetivo: retirar codigo sin uso, centralizar logs F1 y comprobar vuelo;
- limpieza: retirados `control_dron`, el `clock` duplicado y los prototipos no
  instalados/referenciados de `dron_individual/src/vision/`;
- build: `dron_individual` 1/1 y `simulacion_dron` 1/1, ambos codigo 0;
- tests rapidos: 24 passed;
- CTest: `dron_individual` 8/8; `simulacion_dron` 9/12, con tres fallos
  preexistentes/ajenos por copia auxiliar divergente y entorno Python;
- prueba 374: flag false, exit 0, 7/7 pasos, gate 0.005084 m y 0.007652 grados,
  pitch `+30/-30/0`, sin telemetria F1 seleccionada;
- prueba 375: flag true, exit 0, 7/7 pasos, gate 0.004990 m y 0.001472 grados;
  reaparecen logs de motores, GT, trayectoria y pitch;
- evidencia ajena: ORB conserva trazas F5 `ERROR` durante arranque, sin efecto
  sobre el control GT;
- conclusion: `CONSEGUIDA`.
