# Subfase 4C - Evento exacto de creacion de KeyFrame e imagen asociada

## Estado

```text
CONSEGUIDA
Preparacion: cerrada
Acuerdo cerrado: si
Autorizacion funcional: concedida y consumida
```

## Detalle largo

```text
subfases/detalle/subfase_4C_DETALLE.md
```

## Objetivo

Exponer desde ORB-SLAM3/wrapper un evento one-shot que indique que la llamada
actual a `TrackStereo()` creo un KeyFrame concreto, conservando la imagen
izquierda exacta asociada. La asociacion funcional nunca se hace por cercania
temporal.

## Decisiones activas

- ORB-SLAM3 no detecta fiduciales;
- el wrapper compara/consume un evento inequívoco de KF creado;
- identidad de KF: `(drone_id, map_epoch, local_keyframe_id)`;
- `TrackStereo()` entrega un recibo por valor con evento, imagen izquierda
  efectiva, `K`, distorsion, dimensiones y estado de rectificacion efectivos;
- `System` aplica una unica rectificacion/resize antes de Tracking y 4D;
- una guarda rechaza la doble rectificacion wrapper+`System`;
- la calibracion pertenece a Dron y nunca llega desde el servicio fiducial;
- timestamps solo sirven como comprobacion de coherencia.

## Archivos probables al ejecutar

- wrapper `ORB_SLAM3_ROS2`;
- zona minima de ORB-SLAM3 si ya existe API parcial y se justifica tocarla;
- tests de evento, identidad y one-shot;
- `system_architecture` si aparece una interfaz/relacion observable nueva.

## Prohibido

- detectar tags dentro de ORB-SLAM3;
- enviar imagenes dentro de `OrbMap`/`OrbKeyFrame` como solucion principal;
- asociar por timestamp o por KF mas cercano;
- bloquear el tracking.

## Pruebas requeridas

Validar KF creado/no creado, one-shot, IDs monotonicos/coherentes, imagen
izquierda exacta conservada, geometria efectiva coherente, configuraciones
rectificada/no rectificada, guarda de doble rectificacion, reinicios/epochs y
ausencia de heuristica temporal.

## Criterio de exito

Cada evento identifica deterministicamente el KF de esa llamada a `TrackStereo()`
y deja listas para 4D la imagen y calibracion efectivas exactas, con build y
pruebas documentadas.

## Resultado

`Tracking` emite y consume un evento one-shot y `System::TrackStereo()` entrega
el recibo exacto con KF, imagen y calibracion efectivas. El build nativo y el
del wrapper pasan; las pruebas 204/205 comprobaron en runtime identidad y
timestamp exactos. Evidencia: `historial/por_subfase/historial_4C_RESUMEN.md`.
