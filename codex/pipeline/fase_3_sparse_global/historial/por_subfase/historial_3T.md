# Historial 3T - Limpieza, configuracion y handoff

## 2026-08-22 21:06 - Ejecucion completa

- objetivo intentado: retirar legacy de Fase 3, absorber contratos ya
  implantados, ordenar configuracion por despliegue, documentar ownership y
  producir un handoff reproducible sin corregir 3Q;
- checkpoint previo: `1b96a7a`;
- archivos modificados: codigo activo y comentarios contractuales de
  `orbslam3_multi`/`orbslam3_server`, launches, manifests, CMake, YAML, tests,
  ADR y documentacion de Fase 3;
- codigo retirado: directorios `legacy/`/`legacy2/`, snapshots y contratos
  legacy; los historiales y artefactos de prueba se conservaron;
- contratos transversales absorbidos: sus conclusiones viven en el runtime,
  resultado final e `historial/absorbidas/`;
- diagnostico de duplicacion: una sola ruta activa de scheduling, autoridad y
  publicacion; copias YAML servidor/simulacion deliberadas y cubiertas;
- configuracion: seis perfiles tematicos exactos por despliegue y perfil
  `replay_debug` fuera del arranque normal;
- paquetes compilados: `orbslam3_multi`, `orbslam3_server`,
  `simulacion_dron`;
- resultado de build: 3/3, exit 0; builds finales adicionales de servidor y
  simulacion tambien correctos tras formato mecanico;
- tests: CTest 9/9, 10/10 y 8/8 respectivamente; contrato YAML/web 14/14;
- launches instalados: resolución y `--show-args` correctos para servidor y
  simulacion;
- prueba Gazebo: 195 con
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`, `success=true`, exit 0,
  586 s incluidos 180 s de drenaje;
- patrones de reduccion: escenario/cierre, configuracion, colas,
  optimizacion, fusion, score, publicacion, errores y recursos;
- evidencia positiva: 741 principales y 1262 secundarias, ambas colas a cero,
  11 loops comprometidos, 23.978 puntos y 453 KFs publicados con identidad,
  score y rgb, un worker activo maximo y cero hard failures;
- recursos: servidor 249.5 MiB RSS maximo, grupo 1571.4 MiB, PSI full maximo
  0.18, minimo disponible 4406.2 MiB y guarda inactiva;
- revision visual posterior: el usuario confirma que RViz2 se vio perfecto;
  el exit Gazebo 255 ocurre en cleanup posterior a `SIM-DONE`;
- conclusion: `CONSEGUIDA`; no hay regresion tecnica de limpieza o
  configuracion y el resultado final conserva 3Q `A REVISAR`;
- siguiente paso recomendado: iniciar el handoff de Fase 2 conservando ADR
  0009 y volver a 3Q si reaparece la deformacion conocida.

## 2026-08-22 - Correccion final de limpieza 3X

- objetivo intentado: retirar `ORB_SLAM3_MULTI/`, paquete obsoleto que debio
  eliminarse durante la limpieza ejecutada originalmente como 3X;
- archivos retirados: directorio completo `ORB_SLAM3_MULTI/`, incluida su copia
  de `g2o` y `MultiEssentialGraphOptimizer`;
- recuperacion: ultimo estado disponible en `f725edc`;
- build y simulacion: no ejecutados; el paquete no formaba parte del pipeline
  activo ni era descubierto como paquete ROS 2 vigente;
- conclusion: `CONSEGUIDA`; la correccion no cambia el resultado funcional de
  Fase 3.

## 2026-08-22 - Renumeracion y cierre definitivo

- la subfase de scoring pasa de 3S a 3R y su telemetria vigente de `[F3S-*]` a
  `[F3R-*]`; los logs historicos mantienen el marcador original;
- el perfil de debug ocupa 3S y la limpieza/handoff pasa de 3X a 3T;
- los historiales transversales absorbidos 3T-3W se conservan en
  `historial/absorbidas/` para evitar colisiones;
- 3S se valida con build 3/3, CTest 9/9 + 10/10 + 8/8 y prueba 196 silenciosa
  `success=true`;
- el usuario confirma que RViz2 de la prueba 195 se vio perfecto;
- conclusion: `CONSEGUIDA`; Fase 3 queda concluida y Fase 2 pasa a ser actual,
  manteniendo 3Q como mejora futura documentada.
