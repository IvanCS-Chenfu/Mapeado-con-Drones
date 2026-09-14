# Subfase 6L - TRACKING_RISK y reorientacion local

## Estado

CONSEGUIDA para deteccion direccional, STOP y reorientacion local. La migracion
la integra con la inspeccion de fachada sin cambiar su autoridad.

## Deteccion

Para LEFT, RIGHT, TOP y BOTTOM se examina una region solapada que ocupa el 75 %
del ancho o alto de la imagen. Si no contiene ningun keypoint ORB usado para
tracking, el dron se mueve o gira hacia ella y la condicion persiste tres
frames, se activa `TRACKING_RISK`.

Con el flag de debug de Fase 6 se publica/muestra el frame con los keypoints y
la region pobre marcada. Esta imagen no pertenece a la GUI F7.

## Protocolo

1. El dron ordena STOP local sin esperar permiso del servidor.
2. Comunica el evento para retirar lifecycle, ruta y reserva movil.
3. Reorienta yaw o pitch en incrementos configurables de 25 grados, evitando
   exclusivamente el primer sector pobre.
4. Clasifica esa direccion como precaucion local.
5. En una inspeccion de fachada, restaura despues el yaw/pitch de fachada.

Yaw y pitch son autoridad local del dron durante la maniobra. El servidor no
cancela una reorientacion porque no tiene corredor XYZ.

## Integracion con depth bajo demanda

Si `TRACKING_RISK` ocurre al mirar el objetivo de una inspeccion, solo el frame
exacto que dispara la persistencia se usa como segunda captura depth. Un buffer
estereo local y acotado permite recuperarlo por `frame_id`; no se procesan ni se
transmiten los frames anteriores. Ausencia de inliers no implica espacio FREE:
solo disparidad valida genera rayos.

## Limites

No se implementa `VISUAL_RETREAT` ni politica permanente de zonas prohibidas.
LOST conserva el HOLD de Fase 5 durante 10 s y no hace fallback ORB->GT.

## Pruebas

- Persistencia y regiones LEFT/RIGHT/TOP/BOTTOM.
- STOP y giro local sin cambio de `map_epoch`.
- Inspeccion pobre: confirmar que el frame disparador es la segunda captura y
  que se restaura la orientacion de fachada.

## Criterio de exito

El riesgo se detecta antes de LOST, la parada es segura, la correccion evita el
sector pobre y la trayectoria/coverage no quedan huerfanos.
