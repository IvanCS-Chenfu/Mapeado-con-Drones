# Historial 3R - Scoring raw y fused

Las pruebas 192-194 se ejecutaron cuando esta capacidad se denominaba 3S. Por
fidelidad historica, sus marcadores `[F3S-*]` no se reescriben. Desde la
renumeracion final, la telemetria vigente usa `[F3R-*]`.

## 2026-08-22 - Implementacion y regresiones

- Objetivo: score raw por calidad ORB, distancia y aislamiento; fused por media
  mas `0.04*N`; oclusion aplazada a Fase 8.
- Cambios: manager, indice voxel, calibracion `bf`, backend, fusion, servidor,
  grafo web y tests.
- Primer CTest: 7/9; fixture builder esperaba score antiguo y el indice inicial
  agotaba 60 s por expansion de vecinos por cada alta.
- Correccion: agrupar voxels por lote, contar ocupacion en 27 voxels y cachear
  baseline. Resultado: build 3/3, multi 9/9 en 25.49 s, servidor 4/4 y web 1/1.

## 2026-08-22 - Prueba 192

- YAML: `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`.
- Resultado objetivo: `success=true`, scenario/tool exit 0, 578 s.
- Evidencia: commits raw/fused, propagacion a tracks, publicaciones con
  `score_field=true rgb_field=true`, negativos sparse cero y sin error grave.
- Recursos: server RSS 254.5 MiB, grupo 1590.3 MiB, PSI memoria 0, guarda false.
- Limitacion: tras 180 s la cola primaria descendia 49->45. Cada input ORB con
  geometria identica reabria su vecindad; stats de scoring no se emitian en live.
- Conclusion: `PARCIAL`; no se reescribe como pasada tras la correccion.

## 2026-08-22 - Correccion y prueba 193

- Una geometria identica solo reevalua su MP; stats live cada 25 arrivals.
- Build 3/3 y tests 9/9 + 4/4 + 1/1.
- Resultado: `success=true`, scenario/tool exit 0, 572 s.
- Cierre: principal 722 procesadas/pending 0; secundario 1288 procesadas,
  stale 517, committed 771, hard_failed 0, pending 0.
- Stats finales: tracked 60.524, bad 30.836, anchored 24.969, isolated 529,
  near 1, far 24.195, min/media/max `0/0.1502/1`.
- Fused reducido completo: 166 intentos, 77 commits, 12.672 positivos, 6.319
  diagnosticos, 4.528 raw dirty y cero negativos.
- Publicacion final: 23.531 puntos, 472 KFs, score/rgb/identity presentes.
- Recursos: server RSS 269.5 MiB, grupo 1639.2 MiB, MemAvailable 4445.3 MiB,
  PSI memoria 0, guarda false.
- El exit 255 de Gazebo sucede durante cleanup posterior a `SIM-DONE`, no es
  fallo del escenario.
- Revision visual posterior: el usuario confirma que las revisitas elevan el
  score de la zona, comportamiento buscado. Sin embargo, la mayoria de puntos
  estructurales queda demasiado baja y puntos a menos de 1 m conservan score
  excesivo.
- Interpretacion revisada: con `bf/fx` aproximadamente `0.06 m`, el multiplicador
  lejano actual `40` inicia la penalizacion a `2.4 m`; esto concuerda con
  `far=24.195/24.969` y degrada paredes validas habituales entre 2.4 y 5 m. El
  umbral cercano de `0.20 m` explica que casi ningun punto cercano se penalice
  (`near=1`).
- Conclusion tecnica: `CONSEGUIDA`. Conclusion visual/calibracion: `NO
  CONSEGUIDA`. Conclusion agregada: `PARCIAL` hasta recalibrar y repetir la
  validacion.

## Pendiente visual

Acuerdo cerrado: limite cercano fisico fijo en 1 m con caida cuadratica hasta
`0.05`; limite lejano `83.333333*baseline`, fallback 5 m y caida cuadratica
hasta `0.25`. Con baseline actual `0.06 m`, la banda neutra es 1-5 m. La
penalizacion pertenece al raw y puede diluirse mediante la media fused, sin cap
permanente. Pendiente autorizacion para implementar y repetir RViz2 sin borrar
la prueba 193.

## 2026-08-22 - Recalibracion y prueba 194

- Objetivo: aplicar la banda neutra 1-5 m acordada para baseline `0.06 m`,
  penalizar cuadraticamente por debajo de 1 m y conservar alcance lejano
  proporcional al baseline.
- Cambios: defaults near `1.0/0.05`, far `83.333333/5.0/0.25`, curva cercana
  cuadratica, parametros de servidor, regresiones numericas y documentacion.
- Build: 3/3 paquetes en 32.9 s. Tests: score 8/8, fused 4/4, CTest multi 9/9,
  servidor funcional 4/4 y web 1/1.
- Prueba: mismo YAML tipico de 193; `success=true`, scenario/tool exit 0, 582 s
  incluidos 180 s de drenaje.
- Reduccion: `F3S-SCORE-STATS`, `F3S-FUSED-SCORE-COMMIT`, publicaciones,
  cierres de colas, escenario, errores y recursos; log completo no leido.
- Cierre: principal 739/pending 0; secundario 1273/pending 0/hard_failed 0.
- Stats finales: tracked 59.271, bad 30.641, anchored 24.977, isolated 435,
  near 99, far 11.433 y min/media/max `0/0.2596/1`.
- Comparacion con 193: anchored casi identico (24.969->24.977), far baja
  24.195->11.433, near sube 1->99 y la media sube 0.1502->0.2596.
- Fused: 139 intentos, 53 commits, 14.448 positivos, 6.434 diagnosticos,
  3.931 dirty y cero negativos sparse.
- Publicacion final: 23.564 puntos con score/rgb/identity. Recursos estables:
  servidor RSS 248.0 MiB, grupo 1571.3 MiB, PSI memoria 0, guarda false.
- El unico `ERROR` es Gazebo exit 255 durante cleanup posterior a `SIM-DONE`.
- Conclusion tecnica de recalibracion: `CONSEGUIDA`. Conclusion agregada:
  `CONSEGUIDA`; el usuario confirma posteriormente que los scores visuales han
  salido perfectos y da la subfase por concluida.
- Incidencia ajena al scoring observada en la misma ejecucion: mala
  optimizacion final del dron antihorario cerca del fiducial 2. Se abre solo
  diagnostico 3Q/fiducial, sin reabrir 3R ni modificar codigo. La revision
  identifica dos loops 3Q asimetricos sobre el dron 2; el fiducial posterior no
  fue la causa.
