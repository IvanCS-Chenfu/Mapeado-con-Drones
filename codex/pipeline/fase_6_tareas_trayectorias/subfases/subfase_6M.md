# Subfase 6M - Transporte de yaw y pitch

## Estado

CONSEGUIDA. La seleccion geometrica de observacion se realiza en 6N; 6M solo
garantiza transporte y ejecucion reproducibles.

## Contrato

- D* planifica exclusivamente XYZ.
- `TrajectoryPlan` transporta yaw y `camera_pitch` por waypoint.
- `dron_individual` ejecuta yaw y el joint de pitch con limites fisicos de 1J.
- La ruta de fachada mantiene durante todo el tramo la orientacion obtenida en
  la primera captura depth.
- La mirada temporal hacia el objetivo y la correccion `TRACKING_RISK` son
  maniobras locales; no modifican el destino XYZ.

## Cambios de esta migracion

Eliminar cualquier selector residual del servidor basado directamente en
voxeles sparse. La orientacion procede de normales depth locales y se devuelve
como parte del unico resultado de inspeccion definido en 6N.

## Pruebas

Transporte de yaw/pitch de fachada, limites del joint, orientacion constante en
un tramo y restauracion despues de mirar el objetivo o corregir riesgo visual.

## Criterio de exito

Servidor, dron y GUI observan la misma orientacion sin ampliar el estado de D*
ni crear una cadena de control paralela.
