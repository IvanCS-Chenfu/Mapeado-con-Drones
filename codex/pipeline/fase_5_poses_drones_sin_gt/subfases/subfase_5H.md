# Subfase 5H — Integración final de control y regresión multi-dron

## Estado

```text
CONSEGUIDA; absorbe la antigua subfase 5I
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

Limitacion vigente tras la prueba 267: SMALL/plausible ya reancla `pose_` a la
orientacion visual en `t_visual`; la correccion de pose no entra en omega y se
conserva una unica propagacion. Esto reduce `tau_er` un `98.7 %` en ventana
comun en 266 y valida el principio. El anclaje completo de
MODERATE_CONFIRMED ensayado en 267 deja error after cero, pero acelera el tramo
de energia positiva y ORB cae a `5.56 s`. El codigo final exige ademas raw
plausible porque la ejecucion descubrio un anclaje con raw rechazado; esta
reparacion pasa 46/46 GTests y aun no tiene simulacion. No ejecutar 268 ni
etapas 3-8 sin nuevo acuerdo mientras falle el hover.

Acuerdo posterior para 268: repetir primero el mismo hover sin ningun otro
cambio funcional y validar exclusivamente el gate raw final. Si 268 falla, se
detiene la ejecucion antes de implementar otra arquitectura. El paso B ya
debatido, pero condicionado a una autorizacion posterior, no perseguira una
orientacion visual antigua: conservara un residual SO(3) propagable
`Delta_target = R_visual(t_visual) * inverse(R_pred(t_visual))` y lo aplicara
gradualmente sobre la trayectoria angular actual. La velocidad inicial
acordada es `0.30 rad/s` a 50 Hz, sin limite de aceleracion en la primera
version. SMALL anclara y cancelara el target; pending conservara el residual;
raw rejected, epoch incompatible o discontinuidad lo invalidaran. La
correccion no entrara en omega fisica.

Resultado 268: `NO CONSEGUIDA`. El gate raw final funciona y el unico anclaje
moderate es plausible, pero tras corregir `0.057317 rad` el lazo acumula
`+0.046416 J` en `0.88 s` y cae a fallback. No se ejecutan 269 ni etapa 3. El
paso B con `Delta_target` queda como siguiente propuesta ya debatida, no como
codigo autorizado bajo la ejecucion consumida.

Resultado diagnostico 269-272: A con GT normal 50 Hz es estable. B, usando GT
perfecto a 20 Hz a traves del mismo `OrbPosePredictor` y publicando a 50 Hz,
reproduce deriva angular y energia positiva sin fallback. C (+80 ms) y D
(timing/jitter de 268) agravan el fallo hasta rechazar el segundo goal. Por
tanto, antes de retomar `Delta_target`, la siguiente correccion debe tratar la
semantica temporal 20->50 Hz y la coherencia pose/omega del predictor. El nodo
GT diagnostico no es arquitectura final y debe retirarse al cerrar el estudio.

Resultado E/F/G 273-275: las tres variantes con omega GT exacta completan y
son disipativas. E conserva el predictor actual, F hace hold angular y G
propaga SO(3) directamente. Esto selecciona la causa A: el problema principal
esta en la derivacion/filtrado de `omega_motion`. El hold a 20 Hz y la
extrapolacion izquierda son viables con pose/omega coherentes. La siguiente
modificacion debe corregir la estimacion de omega desde ORB, no retocar gains
ni añadir `Delta_target` prematuramente.

Estado tras 276-277: `omega_motion` se deriva causalmente con tres poses
aceptadas, velocidad espacial world/O y proyeccion desde el midpoint hasta
`t_k`; entre medidas se mantiene la ultima omega. Pose GT perfecta a 20 Hz
completa dos veces el hover con energia total negativa y RMSE cercana a la
variante de omega GT. El siguiente escalon es validar delay y jitter antes de
volver a ORB real. No considerar 5H conseguida solo por el laboratorio.

Estado tras 278: con delay fijo de 80 ms el horizonte se clampa en `72.2 %`,
la energia vuelve a ser positiva y el hover falla. 279-281 quedan no ejecutadas
por el criterio de parada. Debe acordarse como compensar `t_k -> now`.

Estado tras 282/284: ampliar a `0.18 s` elimina el clamp pero empeora; integrar
`alpha_hat` reduce RMSE/energia y aun asi falla antes. La opcion alpha queda
apagada por defecto y 279-281 permanecen detenidas.

Estado tras 285-287: el cruce R/omega señala con fuerza a omega predicha, pero
el sanity con R/omega GT tambien falla porque conserva p/v lineales retrasadas.
No hay causalidad definitiva. El siguiente sanity debe fijar p/v GT actuales
en todas las ramas.

Estado tras 288-291: el sanity GT actual completo pasa. Con p/v GT comunes,
las dos ramas con `omega_pred(now)` fallan y las dos con `omega_GT(now)`
completan, incluso cuando 289 conserva `R_pred(now)`. El diagnostico causal
queda conseguido: el fallo inmediato bajo delay esta en omega predicha, no en
R ni p/v. La subfase sigue parcial hasta corregir ese canal sin GT y validar
ORB real. Mantener 279-281 detenidas.

Estado tras 292: el predictor rigido causal por torque pasa 75/75 GTests, pero
la J nominal `diag(1e-4)` compartida con el controlador no reproduce la planta
de Gazebo. El lazo alcanza cientos de rad/s predichos y colapsa en decimas.
292 es `NO CONSEGUIDA`; por el STOP acordado 293-295 no se ejecutan. Antes de
otra prueba debe revisarse la J efectiva y la equivalencia del torque aplicado,
sin cambiar gains, gates, mux, KF ni la ruta productiva.

Estado tras la correccion: la J compuesta es
`diag(0.00803107,0.00803107,0.015805)`. 296-298 y 293-295 completan seis hovers
sin fallback ni tracking no-OK; 294 valida R/omega dinamicas y 295 el estado
completo de laboratorio. Correccion posterior: 295 no tenia delay artificial;
su edad media es 31 ms frente a unos 110 ms en 296-294. El predictor angular
queda validado con delay fijo, pero el estado completo solo sin ese delay. La
subfase permanece parcial hasta timing/jitter medido y ORB real.

La prueba 299 usa el estado completo sin GT actual y la traza determinista de
268. Falla tras `16.94 s`, con cobertura continua de torque y sin fallback ni
tracking no-OK, pero con 48 intervalos `DEGRADED_DT`, edad maxima `0.20 s` y
energia neta `+0.0141 J`. Por el STOP acordado no se ejecutan 300-302 ni se
integra ORB productivo. Queda pendiente corregir la coherencia temporal bajo
periodos largos antes de reabrir esa validacion.

Bateria diagnostica siguiente acordada: ejecutar 303-306 siempre y sin
recalibrar entre pruebas. 303 cruza p/v GT(now) con angular dinamico; 304 usa
p/v GT(now) y R/omega GT interpoladas en `t_k`, propagadas hasta now solo con
torque/J; 305 cruza p/v predichas con angular GT(now); 306 usa GT(now) completo
como sanity. La interpolacion 304 es retrospectiva y no bloqueante, exige
bracket temporal y debe medir tambien el residual R dinamica-GT en `t_k` antes
de atribuir exclusivamente el fallo a omega. Mantener 300-302 detenidas.

Bloque translacional vigente: 307/308 aislan p y v y ambas fallan, por lo que
el diagnostico es `P Y V`. `BodyThrustDynamicPredictor` usa thrust body
sellado, masa compartida, gravedad, dt reales y `R_dynamic(t)`. 309 y su
repeticion 313 validan el modelo con p/v GT(t_k) solo como estado inicial. 310
falla con el estado causal estimado; por STOP, 311/312 y 300-302 no se
ejecutan.

Contrato 314-317 completado: `CausalLinearVelocityEstimator` conserva tres
posiciones visuales aceptadas, calcula velocidades de intervalo y aceleracion
entre midpoints y proyecta solo hasta `t_k`; rechazos y correcciones no
contaminan el historial. 314/315 validan p/v dinamicas con angular GT y 316/317
validan el estado completo dinamico, todas sin fallback ni tracking loss. La
clase permanece solo en laboratorio: conectar `StereoSlamNode` y ejecutar ORB
real requieren preparacion y autorizacion nuevas.

Las pruebas diagnósticas 263-267 conservan el mismo hover. La
telemetría separa input Gazebo de receive/publish/control ROS, transforma todas
las omegas a body y compara `tau_ew` con
`tau_ew_ideal=-Kw*ew_GT`. Si una ejecución no contiene el puente dual-clock,
su conclusión es `DATOS_INSUFICIENTES` y no se repite automáticamente.

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

## Estado de integracion post-317

La ruta productiva dispone de selector temporal `legacy|dynamic`; la rama
dinamica comparte estimadores, masa, J y buffers sellados con el laboratorio.
La paridad 318 no supera el contrato porque existe un hueco de torque al inicio
de un movimiento, aunque el escenario termine. Antes de ORB real se debe
garantizar cobertura causal de la base sin inventar una orden anterior. Las
pruebas 319/320 quedan detenidas.

La propagacion dinamica solo es valida si torque y thrust tienen cobertura
causal completa de `[t_base,t_target]`. El cold start puede sembrarse a cero
porque la cadena real de actuacion inicializa y mantiene cero hasta la primera
orden. Los resets visuales no borran historia fisica. La prueba 318R demuestra
que el buffer debe conservar además el ultimo predecesor ZOH al podar; hasta
corregirlo quedan detenidas 319R/320/321.

La poda vigente conserva una predecesora ZOH y queda validada por 318R2/319R.
Esto no valida el control ORB productivo. 320R demuestra que
`scenario success` basado en tiempo no basta: se debe comprobar seguimiento
de pose/velocidad al final de cada goal. La siguiente prueba productiva debe
garantizar aproximacion inicial con GT y cambio a ORB sólo en el nuevo goal.

La prueba shadow mantiene GT durante aproximacion, observa el mismo ORB
productivo y abre autoridad solo tras tracking, anchor y `1.5 s` de
estacionariedad. Un handoff limpio no basta: el hover exige errores acotados,
tracking sano y ausencia de fallback. 320R2R cumple la frontera pero diverge
hasta `~1.63 m` con tracking `2`.

La bateria 321 confirma autoridad ORB antes del goal y aisla p/v solo en la
salida de control. 321B (`p_ORB+v_GT+angular_ORB`) cumple
`0.159 m / 0.046 m/s / 0.027 rad`, sin fallback; 321AR y 321D divergen al usar
`v_ORB`. 321C cae a fallback y no valida por separado el angular. Diagnostico:
la velocidad lineal ORB es la causa principal demostrada; 5H sigue `PARCIAL`
hasta corregir `v_hat` sin GT y repetir ORB completo.

El diagnostico 322/323 mantiene GT como autoridad y ejecuta la rama ORB
`dynamic` real en shadow. Demuestra que THREE_SAMPLE amplifica el error frente
a TWO_SAMPLE y que la propagacion `t_k -> now` introduce una degradacion mucho
mayor. La salida productiva permanece intacta. Antes de otra prueba ORB
gobernada debe acordarse una correccion de ambas etapas; no ajustar gains ni
usar GT como entrada del estimador.

Contrato validado por 324/325: la gravedad de la dinamica translacional se
expresa en O como `g_O=O_R_W*g_W`. `O_R_W` procede del primer anclaje global
`AUTHORITATIVE`, queda congelada durante el `map_epoch` y se invalida al crear
un epoch nuevo. Sin gravedad autoritativa la rama dynamic no es consumible.
Las revisiones globales posteriores no cambian esta propiedad fisica de O.

Contrato post-325: THREE_SAMPLE queda exclusivamente diagnostico. La velocidad
productiva se obtiene con MIDPOINT_DYNAMIC desde dos posiciones aceptadas,
orientacion interpolada en SO(3), alineacion causal de relojes y propagacion
con torque, thrust y `g_O` hasta `t_k`. Solo cobertura `FULL` puede actualizar
la base. 326/327 seleccionan el estimador; 328/329 validan su integracion en
shadow; 330/331 validan de forma reproducible un hover ORB completo sin
fallback ni tracking no-OK. El criterio de cierre restante es validar
movimientos X/Y/Z/yaw y la trayectoria representativa.

Validacion progresiva 332-343: la arquitectura queda congelada y cada maniobra
elemental exige fallback cero y repeticion antes de avanzar. 332/333 validan X
2 m de forma reproducible. 334 Y 2 m es invalida porque el goal en +Y atraviesa
el fiducial 2 situado en `[0,-8.5,1]`; la colision precede a la perdida de
tracking y al fallback. 335-343 no se ejecutan. El siguiente acuerdo debe
repetir Y alejandose del obstaculo, sin retocar el estimador.

334R elimina el obstaculo y conserva tracking/fallback cero, pero falla el
criterio de frenado: tras el hover final ev ORB permanece en `0.187 m/s` y su
RMSE de los ultimos 3 s es `0.174 m/s`, muy por encima de GT. Y no queda
validado y el STOP mantiene 335-343 sin ejecutar. Cualquier correccion del
estimador requiere un acuerdo nuevo basado en esta evidencia.

La prueba diagnostica 334R3R y su repeticion 335R validan Y al acercar el dron
a la pared y evitar el fiducial; se conserva una velocidad residual aproximada
de `0.11 m/s`. 336/337R validan Z de forma reproducible. La bateria se detiene
en 338: durante yaw tracking pasa `2->3`, se activa GT fallback y los errores
angular/lineal crecen fuertemente. Yaw, combinacion y trayectoria
representativa siguen pendientes; no avanzar sin un nuevo diagnostico/acuerdo.

La auditoria 344-346 descarta que el historial raw cambie de frame con Kref y
localiza una retencion obsoleta tras rechazos. El rebase exclusivo del baseline
raw queda validado en shadow, pero 346 falla en dos fachadas con ORB: el control
se degrada y activa fallback antes de la perdida visual. La correccion se
conserva; la trayectoria representativa sigue pendiente y 347 no se ejecuta
por el STOP acordado.

La prueba diagnostica 348 no cambia comportamiento y reconstruye la secuencia
causal de 346. El fallback con tracking 2 lo dispara un pulso de
`local_valid=false` y `local_continuity_valid=false`; `velocity_valid` y
`reference_keyframe_valid` fallan a la vez, pero no pertenecen al source gate.
La validez se recupera en 100 ms y el bloqueo por goal conserva GT. La
divergencia lineal y angular precede unos 40 s al fallback y se clasifica como
acoplada/multicausal, no como fallo exclusivo de un canal. La siguiente
iteracion debe acordar un aislamiento controlado antes de ejecutar 349A/349B.

El aislamiento 349 usa GT solo en la frontera final de control. Con cobertura
causal validada, 349AR3 demuestra que p/v ORB fallan aun usando R/omega GT;
349B demuestra que R/omega ORB fallan aun usando p/v GT. La clasificacion
vigente es `MULTIPLE_INDEPENDENT_ERRORS`, no mera incoherencia conjunta. La
siguiente solucion debe debatirse por canales y no aplicar un parche doble sin
aislamiento adicional.

## Cierre por evidencia visual

Las pruebas 350R-355 sustituyen la hipótesis de defectos independientes por
evidencia causal más completa: en la ruta larga, la observabilidad visual cae
antes de la pérdida; en una ruta corta, lenta y próxima a textura, ORB gobierna
3/3 veces sin fallback ni tracking no `OK`. Por ello 5H se considera
`CONSEGUIDA` bajo su contrato: producir estado de control estable cuando ORB
dispone de evidencia suficiente.

La vuelta completa al edificio no se declara ORB-only. Fase 6 debe fragmentar
tareas, reducir velocidad y evitar o recuperar regiones visualmente pobres;
hasta entonces se conserva `GT_FALLBACK` para terminar la misión.
