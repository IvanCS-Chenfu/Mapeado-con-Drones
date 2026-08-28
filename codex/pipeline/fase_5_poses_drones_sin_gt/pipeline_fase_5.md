# Pipeline Fase 5 — Poses de los drones sin Ground Truth

## Estado

```text
5A: CONSEGUIDA (auditoría y reconciliación documental)
5B: CONSEGUIDA (estado local coherente, `O_T_B` y gate)
5C-5E: CONSEGUIDAS; 5F PARCIAL
5G-5H: PARCIAL; diagnostico 251 completo, extrinseca optica pendiente
5I: absorbida en 5H; no ejecutar de forma independiente
Fase 5 funcional: en curso
```

## Objetivo

Sustituir progresivamente `sensor/GT/pose` y `sensor/GT/vel` como entradas
normales de trayectoria y control por un estado derivado de ORB-SLAM3, sin
mezclar la continuidad necesaria para controlar con la autoridad global
corregible del mapa.

```text
pose de control continua: O_T_B
pose global corregible:   W_T_B
```

Durante Fase 5 se mantiene `GT_FALLBACK` como excepción temporal y explícita
cuando ORB entra en `RECENTLY_LOST`. Su finalidad es permitir que el dron
complete trayectorias largas mientras Fase 6 todavía no aporta recovery real.

## Autoridades

```text
ORB-SLAM3:
    tracking, mapa local y relaciones frame-KF

RawMapDatabase:
    datos ORB crudos; nunca se sobrescriben por optimización

GlobalPoseStore:
    poses world aceptadas, revisiones y anchors

orbslam3 del Dron:
    continuidad O y composición local/global por frame

dron_individual:
    semántica de goals, generación de trayectoria y control

simulacion_dron:
    Gazebo, GT para métricas y GT_FALLBACK temporal parametrizado
```

La frontera ROS 2 de pose global es siempre:

```text
Servidor <-> orbslam3 del Dron
```

`dron_individual` no consulta directamente al Servidor.

## Frames

```text
W = world global
L = mapa interno ORB del map_epoch
O = frame local continuo de control
K = reference KeyFrame real del frame
C = cámara
B = body
G = world de Gazebo, solo simulación
```

No asumir `O == L` sin medir la continuidad real de `pose_local`.

## Muestra ORB coherente

El estimador debe recibir del mismo `TrackStereo`:

```text
timestamp
map_epoch
tracking_state
reference_keyframe_valid
reference_keyframe_id
Tcr = C_T_Kref
pose_local
```

No usar el último KF creado, un nearest-KF ni datos mezclados entre frames.

## Matemática

Control:

```text
O_T_C = O_T_Kref * inverse(Tcr)
O_T_B = O_T_C * C_T_B
```

Global:

```text
W_T_C = W_T_Kref * inverse(Tcr)
W_T_B = W_T_C * C_T_B
```

Una revisión global puede mover `W_T_B`. No debe mover `O_T_B` ni producir
velocidad artificial en el controlador.

Al cambiar `Kold -> Knew`, se mantiene primero la continuidad local, se crea
una pose global provisional y se consulta al Servidor de forma asíncrona. La
respuesta autoritativa sustituye la provisional sin bloquear `TrackStereo`.

## API del Servidor

Debe resolver:

```text
(drone_id, map_epoch, keyframe_id)
    -> W_T_KF
    -> pose_revision
    -> validity/status
```

La primera obtención usa servicio asíncrono. Las revisiones posteriores usan
topic/push. Si el KF aún no existe, 5D reutiliza el patrón real final de Fase 4
para peticiones pendientes; no introduce polling por frame.

## Goals

Relativo: se acepta con pose local válida y se ejecuta en `O`.

Absoluto: exige `W_T_O` válida. Al aceptarse se convierte una vez:

```text
O_T_goal = inverse(W_T_O_snapshot) * W_T_goal
```

Después el goal queda congelado en `O`. Una revisión global posterior no lo
deforma ni lo cancela. Sin `W_T_O`, el goal absoluto se rechaza; nunca se
reinterpreta silenciosamente como relativo.

## Pérdida y GT_FALLBACK

Estados mínimos:

```text
GLOBAL_VALID
LOCAL_ONLY
GT_FALLBACK
UNANCHORED_NEW_EPOCH
```

`RECENTLY_LOST` activa el fallback inmediatamente:

```text
O_T_G = O_T_B_last * inverse(G_T_B_last)
O_T_B = O_T_G * G_T_B_GT
```

El goal activo y los siguientes tramos de la misión pueden continuar mientras
la referencia global previamente válida permita resolver su semántica. Al
volver ORB se realinea con el `O` mantenido por GT, sin salto. Si aparece un
epoch nuevo, GT conserva provisionalmente `O`, pero la pose global permanece
inválida hasta anchor/loop.

`GT_FALLBACK` debe ser parametrizable, visible mediante `pose_source` y estar
confinado al estado de control en Simulación/Fase 5. No alimenta KFs, mapa,
anchors, loops, optimización, `W_T_KF` ni la pose global final. Fase 6 debe
eliminarlo y sustituirlo por recuperación/reanclaje real sin GT.

## Velocidad y suavizado

La velocidad se deriva de `O_T_B` usando timestamps coherentes. Nunca se deriva
de `W_T_B`.

El smoothing no es un requisito. Solo se estudia si 5F demuestra una necesidad
concreta de un consumidor global; no puede usarse para ocultar discontinuidades
del control.

## Secuencia y puertas

```text
5A -> 5B -> 5C -> 5D -> 5E -> 5F -> 5G -> 5H
```

- Antes de 5C/5D se verifica el cierre vigente de 3Q y el HEAD real.
- 5F exige métricas y aceptación explícita del usuario antes de integrar.
- 5G exige reconciliar el ADR de GT antes de implementar el fallback.
- 5H absorbe la integración/regresión que antes pertenecía a 5I.
- Cada subfase requiere preparación y autorización propias.

Para el bloque 5C+5D+5E+5F se acuerda un servicio inicial más push dirigido,
un único pending de reference KF por dron, estado global
`INVALID/PROVISIONAL/AUTHORITATIVE` y `global_valid` solo autoritativa. Los
goals absolutos permanecen deshabilitados hasta 5H. 5F entrega métricas y
gráficas O/W/GT y se detiene en puerta humana.

## Prueba final

Dos drones deben completar la vuelta representativa al edificio con anchor
temprano, varios reference KFs, correcciones globales y goals relativos y
absolutos. Si aparece pérdida, debe medirse `GT_FALLBACK` y comprobarse la
continuidad de entrada/salida.

La Fase 5 solo puede cerrarse con pose y velocidad aceptadas, control normal sin
GT directo, uso de fallback perfectamente identificado y documentación e
historial coherentes.
