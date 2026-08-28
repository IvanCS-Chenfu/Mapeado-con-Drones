# Subfase 5H — Integración final de control y regresión multi-dron

## Estado

```text
preparada; absorbe la antigua subfase 5I
```

## Objetivo

Conectar el estado validado de Fase 5 a:

```text
gen_tray
control_calcular_fuerzas
```

GT deja de ser una entrada directa del funcionamiento normal. Solo aparece en
la interfaz común con `pose_source=GT_FALLBACK` bajo las condiciones de 5G.

## Puertas previas

- 5B: `O` y muestra ORB coherentes;
- 5C/5D: `W_T_KF` versionada y transporte asíncrono;
- 5E: pose local/global;
- 5F: relación pose/KFs y correcciones revisadas visualmente;
- 5G: velocidad, fallback y excepción GT documentados.

## Arquitectura

```text
ORB-SLAM3 OK + epoch anclado ------> estado O estimado
ORB-SLAM3 RECENTLY_LOST/LOST ------> GT_FALLBACK -> estado O
absoluto sin anchor ----------------> GT_FALLBACK -> estado O
                                                     |      |
                                                     v      v
                                                 gen_tray  control
```

El estado incluye pose/velocidad en `O`, timestamp, validez, `pose_source`,
causa de fallback y estado local/global. `gen_tray` y control no duplican ramas
ORB/GT ni se suscriben directamente a GT para su funcionamiento normal.

## Semántica de goals

Relativo: generar en `O`, conservando la semántica vigente de
`TrayAction`/`lib_tray`. La transformación recibe tests deterministas. La
primera prueba Gazebo completa con tareas relativas pertenece a Fase 6; si
revela un defecto propio de esta capa, se reabre 5H.

Absoluto con `W_T_O` válida: convertir al aceptar:

```text
O_T_goal = inverse(W_T_O_latest) * W_T_goal
```

Después queda congelado en `O`. Una revisión global no cambia el setpoint.

Absoluto sin `W_T_O`: en el perfil temporal de Fase 5 no se rechaza si
`GT_FALLBACK` está habilitado. Se ejecuta mediante la interfaz común con causa
`STARTUP_UNANCHORED_ABSOLUTE` o `NEW_EPOCH_UNANCHORED` hasta coincidir ORB `OK`
y anchor del epoch. Este bypass conserva default desactivado fuera del perfil
acordado y se elimina en Fase 6.

Una nueva orden sustituye/cancela la anterior según el action server. Auditar
la cancelación para no dejar velocidad, aceleración o jerk residuales. No
introducir replanning/recovery de Fase 6.

`gen_tray` coordina explicitamente el comienzo de cada goal con el mux. La
fuente anterior permanece bloqueada al terminar y durante las esperas. Solo al
llegar el siguiente goal se abre la frontera, se espera una muestra consumible,
se bloquea la nueva fuente y se congelan de esa misma muestra pose, velocidad y
transformacion absoluta. El primer feedback se publica con `t=0`, usando
`x0/v0/yaw0/yaw_rate0` de la muestra, para que `ep=ev=0`. La unica conmutacion
permitida dentro de un goal es `ORB -> GT_FALLBACK` por perdida real de
tracking; despues GT queda bloqueado hasta finalizar.

## Frecuencia

No cambiar 50 Hz->20 Hz por intuición. Medir edad de estado y latencia. El
control puede consumir la última muestra válida dentro de límites acordados.

## Ámbitos

```text
dron/orbslam3_ros2/src/stereo/**
dron/dron_individual/src/control_tray/gen_tray.cpp
dron/dron_individual/src/control_tray/control_calcular_fuerzas.cpp
dron/dron_individual/config/**
dron/dron_individual/launch/**
simulacion/simulacion_dron/**
instrumentación ya validada
```

No cambiar ganancias para ocultar mala estimación ni algoritmos `lib_tray` sin
un bug demostrado y nueva autorización. No corregir aquí los defectos ya
identificados del optimizador de Fase 3.

Diagnostico vigente tras 251: la autoridad `W_T_C` fiducial es coherente, pero
la rotacion configurada como `body_T_camera` es la inversa optica
`camera_T_body`. La composicion correcta del wrapper deja de ser correcta al
recibir ese parametro mal orientado y proyecta X world sobre Z control. La
correccion acordable debe mantener una unica convencion SE(3) completa; no se
debe compensar en `C_T_W`, el controlador ni las ganancias.

Correccion posterior a 252: `PoseStatePredictor` filtra medidas a 20 Hz y
publica estado propagado a 50 Hz; `ORB -> GT_FALLBACK` alinea GT contra el O
activo mediante `ContinuousSourcePose::Update`, sin cambiar el frame del goal.
El predictor es permanente. Suscripcion, transporte, lock, alineacion y handoff
GT quedan marcados `TODO FASE 6` para retirarlos con el fallback.

Diagnostico posterior a 253: esa implementacion no esta validada. Aplicar el
predictor tambien a la orientacion GT retrasa la actitud usada por el lazo de
torque y lo desestabiliza, aun sin conmutacion de fuente. La siguiente
correccion debe conservar una actitud de control sin ese retardo y validar por
separado el suavizado traslacional/velocidad ORB; requiere nuevo acuerdo.

Correccion acordada posterior: predictor y filtros pertenecen al estimador de
`orbslam3_ros2`, no a `dron_individual`. ORB publica a 50 Hz con traslacion y
velocidades filtradas, orientacion medida sin low-pass y propagacion angular
entre frames. El mux solo selecciona/alinea; GT usa `sensor/GT/pose` y
`sensor/GT/vel` exactos. La primera validacion puede motivar otro ajuste sin
ocultar el intento actual.

Diagnostico posterior a 254: aislar GT funciona, pero aceptar sin limite cada
orientacion ORB y limitar solo su velocidad genera estados incompatibles en los
giros. Los handoffs empiezan con salto cero; despues aparecen pasos de hasta
unos `0.28 rad/frame`, torque reactivo y perdida. La siguiente iteracion debe
suavizar o acotar la innovacion angular ORB y derivar de esa misma orientacion
corregida la velocidad publicada. GT permanece exacto y fuera del predictor.

Correccion acordada tras 255: la probation valida una cadena geometrica aunque
ORB cambie varias veces el ID del KF candidato. `Tcr` sigue gobernando dentro de
cada referencia; `local_t_camera` solo enlaza el instante de cambio si su
incremento es fisicamente plausible y nunca sustituye la pose local principal.
Tres frames geometricamente coherentes confirman la referencia; solo una
inconsistencia persistente agota el timeout.

El predictor mantiene un unico estado SE(3) a 50 Hz. Cada medida corrige la
prediccion de forma gradual, con ganancias y limites configurables de
innovacion, velocidad y aceleracion separados para traslacion y rotacion. La
pose se integra desde las mismas velocidades que se publican. Innovaciones
angulares moderadas se absorben gradualmente; solo las imposibles acumulan
rechazo. El fallback permanece como proteccion temporal ante perdida real, no
como mecanismo normal frente a cambios de KF. No se usa IMU.

Correccion acordada tras 256: una innovacion angular pequena se aplica de forma
normal; una moderada queda `MODERATE_PENDING` y no corrige la actitud hasta
confirmarse temporalmente por direccion, magnitud y plausibilidad dinamica.
La confirmacion exige tres frames y se refuerza a cuatro durante cinco frames
tras cambiar reference KF. Una evidencia aislada se marca
`MODERATE_DISCARDED`; una persistente pasa a `MODERATE_CONFIRMED` y se absorbe
con los limites SE(3) existentes. El gate duro conserva
`REJECTED_EXCESSIVE`. Todos los umbrales viven en
`config/navigation_state.yaml`.

El debug `debug_orb_control_state`, apagado por defecto, debe distinguir medida
raw, innovacion, correccion aplicada y paso realmente publicado, y
correlacionarlos con pose/omega, edad del estado, errores y torque del
controlador. No se usa un unico `rotation_step` ambiguo para inferir causalidad.

Limitacion vigente tras la etapa 2: esta politica no supera hover ORB. SMALL no
puede expandirse con gaps hasta aceptar correcciones muy superiores al umbral
base, y la persistencia del residual no basta como evidencia independiente de
movimiento fisico. Cualquier sustitucion por SMALL fijo o confirmacion basada
en incrementos raw/gauge requiere nuevo acuerdo antes de otra ejecucion.

## Pruebas de integración

1. relativa con `pose_source=ORB`, mediante tests deterministas;
2. absoluta con autoridad, revisión global durante el goal y setpoint O
   inmutable;
3. absoluto inicial sin global mediante fallback explícito;
4. perdida durante goal: `ORB -> GT_FALLBACK`, sin retorno a ORB hasta la
   frontera posterior;
5. dos drones ejecutan directamente la vuelta típica absoluta al edificio.

La regresión final usa sin modificar:

```text
codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
```

No existe bootstrap relativo. Gazebo GUI y RViz2 permanecen visibles
simultáneamente para comparar movimiento físico, KFs y poses estimadas. La
vuelta cubre anchor temprano, varios reference KFs, correcciones globales y
cambios de fuente.

RViz2 representa para cada dron un sistema XYZ y etiqueta construidos desde
`NavigationState.o_t_body`, exactamente la pose consumida por
`control_calcular_fuerzas`. Su visibilidad depende de la validez local de la
interfaz de control, no de `global_valid` ni de que exista `w_t_body`; la fuente
ORB/GT queda indicada para diagnosticar fronteras sin parpadeo global.

Métricas por dron:

```text
tiempo total/ORB/GT_FALLBACK y ratio ORB
causa y duración por entrada de fallback
pérdidas, map_epochs y cambios reference KF
revisiones globales, error pose/velocidad
latencia/jitter y resultados de goals
```

Los errores frente a GT se conservan como observacion externa de deriva, pero
no son criterio de conmutacion, perdida ni exito de navegacion estimada.

Patrones iniciales:

```text
F5H|GOAL|ACCEPT|REJECT|RESULT|POSE_SOURCE|STARTUP_UNANCHORED_ABSOLUTE|TRACKING_LOST|NEW_EPOCH_UNANCHORED|GT_FALLBACK|REFERENCE_KF|REVISION|ANCHOR|CONTROL|SUCCESS|ERROR|FATAL|Segmentation fault|Killed
```

## Criterio de éxito

Builds correctos; pose/velocidad aceptadas; ausencia de GT directo en
`gen_tray`/control normal; semántica absoluta correcta; transporte de KFs
funcional; ejecución de la vuelta; fallback perfectamente identificado y
documentación/historial coherentes.

Se exige que el primer error de posicion y velocidad sea cero al cambiar fuente
y comenzar el siguiente goal. La retirada de esta coordinacion temporal junto
con `GT_FALLBACK` queda como deuda explicita de Fase 6.
Tambien se exige que la extrinseca cargada sea realmente `B_T_C`; para la camara
optica frontal actual su rotacion es `RPY=(-90,0,-90)` bajo la composicion
`yaw*pitch*roll`, no la inversa `RPY=(0,-90,90)`.
`PARCIAL` si la vuelta termina pero GT aparece fuera del
fallback. `NO CONSEGUIDA` si hay mezcla silenciosa de fuentes, GT contamina
mapa/global o un fallo propio de Fase 5 impide ejecutar la trayectoria. Los
defectos del optimizador de Fase 3 se documentan y no se corrigen aquí.

## Deuda obligatoria de Fase 6

```text
eliminar GT_FALLBACK
implementar recovery real
implementar búsqueda/reanclaje sin GT
suavizar o eliminar saltos al cambiar de fuente
filtrar/interpolar el estado de control o consumir velocidad estimada robusta
retirar source lock, handshake entre goals y hold temporal del controlador
validar tareas relativas integradas y devolver a 5H cualquier defecto propio
```
