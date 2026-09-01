# Subfase 1J - Pitch fisico de las camaras

## Objetivo

Incorporar un unico joint de pitch entre cuerpo y rig estereo, controlado por
torque con consigna, limites configurables y extrinseca body-camara dinamica.

## Contrato

- eje `+Y`; pitch positivo mira hacia abajo;
- limites por defecto `+-70 deg`, velocidad y aceleracion parametrizables;
- rig de masa/inercia bajas y camaras rigidas entre si;
- `camera_pitch_enabled` selecciona topologia revolute o fixed;
- publicar estado del joint y TF body-camara al stamp de simulacion;
- permitir pasos `pitch` verificables en los escenarios;
- no introducir GT en fiduciales ni en el producto cartografico.

## Cierre

Conseguida mediante las pruebas 364-372: control finito y asentado, limites,
movimiento del dron con pitch no neutral, topologia fixed/revolute y barrido
completo. El detalle permanece en `historial_1J.md`.
