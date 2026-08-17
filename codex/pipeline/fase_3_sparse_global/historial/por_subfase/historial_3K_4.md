# Historial 3K.4 - runtime historico sustituido

## 2026-07-28 - Prueba larga multi-dron

- objetivo intentado: aplicar varias optimizaciones fiduciales de submapas
  distintos y conservar `RawMapDatabase` intacta;
- build: `orbslam3_multi orbslam3_server simulacion_dron`, codigo `0`;
- prueba: `prueba_42`, scenario `success=true`, `SIM-EXIT-CODE 0`;
- evidencia: tres tareas se aplicaron y aceptaron, los hard fiducials no se
  movieron y `raw_db_modified=false`;
- limitaciones: el dron 1 perdio tracking en la segunda mitad y una arista
  deformable quedo clasificada como rota;
- conclusion historica: `PARCIAL` por continuidad/calidad geometrica;
- estado actual: el scheduling ejecutado en esta prueba fue sustituido el
  2026-08-05. La propiedad vigente de `3K` esta en `historial_3K_8.md`: un
  unico worker, tarea activa no interrumpible y prioridad fiducial.
