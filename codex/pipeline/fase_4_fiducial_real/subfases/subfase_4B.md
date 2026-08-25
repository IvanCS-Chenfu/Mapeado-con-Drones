# Subfase 4B - Generacion visual y spawn de objetos fiduciales en Gazebo

## Estado

```text
CONSEGUIDA el 2026-08-24
Preparacion: cerrada junto con 4A
Acuerdo: confirmado y ejecutado
Autorizacion funcional: consumida para el bloque 4A+4B
Prueba: Gazebo 201 automatica/visualmente correcta y smoke 202 correcto
```

Evidencia: `historial/por_subfase/historial_4B_RESUMEN.md`. La trayectoria
tipica revisada queda preparada y su ejecucion se aplaza a 4C+4D por decision
explicita del usuario.

## Detalle largo

```text
subfases/detalle/subfase_4B_DETALLE.md
```

## Dependencia

`4A` conseguida.

## Objetivo

Crear los assets visuales AprilTag y spawnear en Gazebo los objetos fiduciales
estaticos definidos en 4A, con texturas correctas, colision/visual coherentes y
sin sustituir el launch actual por snapshots antiguos.

## Decisiones activas

- Gazebo usa configuracion propia de Simulacion derivada del contrato 4A;
- las texturas deben ser inspeccionables y corresponder al `tag_id` esperado;
- los objetos son estaticos y colisionables;
- los tres objetos baseline se colocan a ±8.5 m y la trayectoria mantiene el
  cuadrado a ±10 m, sin goals sobre el centro de los objetos;
- se evita z-fighting con un offset visual documentado;
- no se introduce semantica de Servidor en el wrapper.

## Archivos probables al ejecutar

- scripts o generadores de texturas/modelos en `simulacion_dron`;
- assets/SDF/launch necesarios para spawn;
- pruebas de configuracion y guardas;
- `system_architecture` si cambia deployment o recursos de Simulacion.

## Prohibido

- modificar ORB-SLAM3;
- implementar detector/PnP;
- usar GT para validar anchors;
- sobrescribir launches actuales con material de sandbox sin reconciliar.

## Pruebas requeridas

Comprobar generacion de tags, spawn de los tres objetos baseline, texturas en
caras correctas, ausencia de z-fighting evidente, colision/estaticidad,
configuracion vacia segura y coherencia con 4A. La simulacion acordada debe
completar la trayectoria a ±10 m sin colisiones; el usuario revisara Gazebo
visualmente y no se transferiran capturas por el chat.

## Criterio de exito

Los objetos aparecen en las poses configuradas, con tags correctos, trayectoria
sin colisiones y validadores verdes. No se marca conseguida sin build,
prueba/simulacion acordada, logs reducidos, revision visual del usuario y
documentacion real.
