# Historial 5J - Limpieza y cierre

## 2026-08-31 - Reestructuracion autorizada

- objetivo intentado: separar 5H/5I/5J, retirar laboratorios cerrados y crear
  una puerta maestra de debug sin modificar el estimador productivo.
- checkpoint previo: commit `80dd2ae`, subido a `origin/main`.
- decisiones: conservar metricas/logs crudos localmente; versionar solo
  resumenes y artefactos reproducibles; poda conservadora.
- cambios iniciales: retirados launch args de forcing, shadow y overrides;
  eliminado `gt_timing_diagnostic` del build; `debug_fase_5` gobierna los
  subdebug de control y evidencia visual.
- pruebas: pendientes en este punto del historial.
- conclusion: `EN EJECUCION`.

## 2026-09-01 - Builds, tests y prueba 356

- paquetes compilados: `orbslam3`, `dron_individual` y `simulacion_dron`.
- resultado de build: correcto en los tres paquetes.
- tests: `orbslam3` 3/3; GTests de `dron_individual` 2/2; analizadores Python
  11/11. El CTest agregado conserva deuda lint legacy ajena; el codigo Fase 5
  modificado pasa formato y tests funcionales.
- prueba 356: escenario completo y debug master apagado correctamente, pero sin
  deteccion fiducial valida ni anchor; todo el movimiento uso GT.
- conclusion: `NO CONSEGUIDA` como regresion ORB por ausencia del prerrequisito
  visual; valida debug OFF y no demuestra regresion del mux.

## 2026-09-01 - Prueba 357

- objetivo: validar debug ON y primera regresion ORB productiva.
- resultado: escenario completo; fiducial valido, anchor, cualificacion y
  autoridad ORB antes de X; X, hover, Y y hover final sin fallback posterior
  ni tracking no `OK`.
- evidencia negativa: los CSV no se abrieron porque el directorio de salida no
  existia, aunque las trazas de debug si reaparecieron.
- conclusion: `CONSEGUIDA` funcionalmente; `PARCIAL` para evidencia CSV.

## 2026-09-01 - Prueba 358 y cierre

- objetivo: repetir 357 con el directorio de evidencia preparado.
- prueba: runner y escenario codigo 0, `success=true`, 131 s; guard inactivo,
  minimo 5649.0 MiB y ORB maximo 1101.8 MiB RSS.
- evidencia positiva: autoridad ORB antes del primer movimiento; ruta completa
  sin fallback posterior ni tracking no `OK`. Ambos CSV se generaron y
  parsearon. Drone 1 aporta 1808 frames `OK`, mediana 324 inliers, ratio 0.900
  y cobertura 0.917.
- conclusion: `CONSEGUIDA`. 357 y 358 son las dos regresiones ORB
  post-limpieza; debug OFF/ON y evidencia recuperable quedan validados.
- siguiente paso: iniciar Fase 6 sin reabrir Fase 5 salvo divergencia con
  evidencia visual buena o ruptura de los contratos causales.
