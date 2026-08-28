# Subfase 5E — Estimador embarcado de pose local/global

## Estado

```text
CONSEGUIDA tecnicamente; calidad global condicionada por 5F
```

## Objetivo

Implementar junto a `orbslam3` en el Dron un estimador que mantenga a
frecuencia ORB:

```text
O_T_B
W_T_B cuando global válida
reference KF
W_T_Kref provisional/autoritativa
pose_revision
estado, pose_source y timestamp
```

No depender de un round-trip de red por frame.

## Entradas y salidas

Entrada ORB coherente:

```text
timestamp, map_epoch, tracking_state
reference_keyframe_id, Tcr, pose_local
```

Entrada del Servidor:

```text
W_T_Kref, pose_revision, status
```

Calibración explícita: `B_T_C`/`C_T_B`.

Salidas conceptuales:

```text
pose_control_body = O_T_B
pose_global_body = W_T_B si válida
map_epoch, reference_kf, pose_revision
global_valid, pose_source, timestamp
```

`NavigationState` se ampliará de forma aditiva con `pose_revision` y un estado
global explícito:

```text
INVALID
PROVISIONAL
AUTHORITATIVE
```

## Matemática y estados

```text
O_T_B = O_T_Kref * inverse(Tcr) * C_T_B
W_T_B = W_T_Kref * inverse(Tcr) * C_T_B
```

Una revisión puede hacer saltar `W_T_B`, nunca `O_T_B`.

Al cambiar de KF se crea `W_T_Knew` provisional, marcada como
`PROVISIONAL`; la respuesta válida del Servidor pasa a `AUTHORITATIVE`. Una
respuesta tardía de un KF que dejó de ser reference no altera el estado activo.

Semántica acordada:

- `W_T_B` provisional puede publicarse para observabilidad;
- `global_valid=false` mientras el estado sea `INVALID` o `PROVISIONAL`;
- solo `AUTHORITATIVE` habilita `global_valid=true`;
- `pose_source=ORB` mientras ORB sea la fuente de la pose continua de control;
- disponer de W autoritativa no cambia por sí solo `pose_source`.

Los goals absolutos permanecen deshabilitados durante 5C-5F mediante una puerta
explícita. Aunque exista W autoritativa, la conversión world->O y su habilitación
pertenecen a 5H. Los goals relativos y el control GT legacy conservan su estado
vigente durante la validación.

Un cambio de epoch invalida la autoridad global anterior y ofrece una operación
clara de reset/realign para 5G. Con `RECENTLY_LOST/LOST` no se inventa pose ORB
válida.

## Ámbitos

```text
dron/orbslam3_ros2/include/**
dron/orbslam3_ros2/src/**
dron/orbslam3_ros2/config/**
dron/orbslam3_ros2/launch/**
dron/orbslam3_msgs/**        # solo contrato aprobado en 5D
simulacion/**                # instrumentación
```

Evitar paquete nuevo salvo necesidad demostrada. No implementar matching global
en Dron, GT funcional ni control final.

## Pruebas acordables

1. frecuencia y coste por frame;
2. continuidad al cambiar reference KF;
3. transición provisional→autoritativa sin mover `O`;
4. corrección global significativa: salto en `W`, no en `O`;
5. invalidación de revisiones/epoch antiguos;
6. extrínseca cámara↔body validada.

Patrones iniciales:

```text
F5E|POSE_CONTROL|POSE_GLOBAL|PROVISIONAL|AUTHORITATIVE|REFERENCE_SWITCH|POSE_REVISION|EPOCH_RESET|CONTINUITY|CAMERA_BODY|ERROR|FATAL
```

## Criterio de éxito

`O_T_B` continuo, global correcta por ref-KF, provisional/autoritativa
funcional, correcciones globales aisladas del control, extrínseca comprobada y
frecuencia/latencia aceptables.
