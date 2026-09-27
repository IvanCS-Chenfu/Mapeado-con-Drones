# Prueba 6.3.3 - Optimizacion mediante revisita fiducial

## Configuracion

- Un dron con control GT estable.
- Ruta: `(0,-10,1,90 deg) -> (-10,-10,1,90 deg) -> (-10,10,1,0 deg) -> (0,10,1,-90 deg)`.
- Los tramos XYZ largos usan `tx=ty=tz=24 s`; los giros finales usan
  `tyaw=8 s`, por debajo de la mitad del tiempo lineal (`12 s`).
- Fase 6 y protocolo de perdida ORB desactivados. La mascara corporal del
  score permanece fija a `false`.
- GUI: Gazebo y `multidron_gui`; sin RViz2.

## Instrumentacion pasiva

- `[C6-KF-POSE]`: solo KFs ya incluidos en la publicacion incremental del
  builder, con identidad, timestamp raw, pose world y revision.
- `fase45_recorder`: GT real desde `/dron_1/sensor/GT/pose`; el procesador
  interpola ese GT en el timestamp de cada KF.
- Eventos existentes: `[F3H-FID-POSE-ERROR]`, `[F3I-GRAPH-BUILD]`,
  `[F3J-OPTIMIZE]`, `[F3L-VALIDATE]` y `[F3K-ATOMIC-COMMIT]`.

## Criterio de analisis

La ejecucion solo se aceptara si aparece `[SCENARIO-RUNNER-DONE]`. Las poses
iniciales y finales publicadas de cada KF se contrastan con GT externo. No se
alteran umbrales ni decisiones del backend para provocar una optimizacion.

## Resultado

El primer lanzamiento se interrumpio externamente durante la espera final y
un segundo intento fallo en el arranque de Gazebo al expirar el spawner del
tercer objeto fiducial. Ambos logs se conservan en `datos_brutos/intentos/`.

La ejecucion valida `c6_3_3_optimizacion_fiducial_reintento_1` completo la
ruta y emitio `[SCENARIO-RUNNER-DONE]` y `[SIM-DONE]` con codigo 0. Los logs
bruto y reducido se conservan como `ejecucion_valida_reintento_1.*`.

En el segundo fiducial se midio un error de traslacion de `0.734454 m`, que
requirio optimizacion. El backend construyo un grafo con control KF 31, target
KF 115, ventana de 84 KFs, 45 controles y 44 aristas temporales. La pasada
convergio a error residual nulo en los tres componentes reportados, fue
validada como `accept_full` y realizo el commit 36 de la revision 36, moviendo
83 KFs.

Sobre las 118 identidades de KF con GT interpolado, la ultima pose publicada
por el servidor obtuvo RMSE de posicion `0.122149 m`, error medio `0.118810 m`
y maximo `0.191917 m`. Las series, la tabla de la pasada y las figuras se
generan reproduciblemente desde el log reducido y `gt_pose.csv` mediante
`scripts/procesar_c6_3_3_optimizacion_fiducial.py`.

La figura temporal reconstruye el mapa vivo despues de cada publicacion: cada
KF conserva solamente su ultima pose disponible y se suma su error de posicion
frente a GT. Por tanto, la serie crece al incorporar deriva y cae cuando el
commit sustituye las poses corregidas. La serie fuente queda en
`datos_procesados/error_mapa_vivo_temporal.csv`; los instantes de observacion
que exige optimizacion y de commit se marcan en la figura.
