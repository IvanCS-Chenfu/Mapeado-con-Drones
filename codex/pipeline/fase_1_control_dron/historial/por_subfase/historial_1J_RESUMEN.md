# Historial 1J - resumen

## Estado agregado

El selector global y por goal, el joint, su control por torque, la extrinseca
dinamica y la estabilidad fisica quedan **CONSEGUIDOS**. La regresion de vuelo
se corrigio reduciendo la masa descentrada del rig a `3e-5 kg`; gates reales
demuestran llegada con joint fijo y movil. En 372 ORB toma autoridad real antes
del barrido, pero pierde tracking al variar mucho el pitch y entra en el
`GT_FALLBACK` acordado. Estado agregado de 1J: **CONSEGUIDA**; la persistencia
visual ORB durante el barrido queda para Fase 5/6.

## Implementacion validada

- Selector estricto propagado desde `multi_dron.launch.py` al
  `navigation_state_mux` de cada dron.
- Fuente observable `POSE_SOURCE_GT_FORCED=4`, distinta de `GT_FALLBACK`.
- En `gt`, pose y velocidad GT gobiernan sin depender de ORB; ORB continua en
  sombra. En `orb`, la autoridad es ORB y las pruebas desactivan el fallback.
- Builds correctos de ambas replicas `orbslam3_msgs`, `dron_individual` y
  `simulacion_dron`; tests funcionales 2/2 y tests Python dirigidos 24/24.
- La suite global solo conserva 1540 errores `flake8` heredados en 26 scripts
  experimentales de vision, ajenos a esta entrega.

## Pruebas

- `359`: **INVALIDA DE INFRAESTRUCTURA**. La copia Server de
  `orbslam3_msgs` ocultaba la nueva constante; se reparo la replica y se
  recompilo. El escenario no llego a empezar.
- `359R` GT: **CONSEGUIDA**. 3479/3479 muestras `GT_FORCED`, cero ORB y cero
  fallback en la salida; goal completado y ORB mantuvo tracking en sombra.
- `360` ORB: **CONSEGUIDA PARA EL SELECTOR**. Autoridad ORB confirmada antes
  del goal, 3074 muestras ORB visibles, cero fallback y cero GT forzado; goal
  completado. La estabilidad o exactitud ORB pertenece a Fase 5 y no era
  criterio de esta prueba.

No hay simulaciones activas.

## Intentos de joint 361 y 362

Estado 1J: **PARCIAL, DETENIDA EN PUERTA DE ASENTAMIENTO**. El codigo de los
tres bloques compila y sus tests dirigidos pasan. En 361 el joint produjo
velocidad/esfuerzo NaN desde neutral y no se movio. Tras el ajuste fisico
acordado, 362 elimina los valores no finitos y alcanza `+30 deg` con unos
`0.40 deg` de error, pero conserva una oscilacion de velocidad entre
aproximadamente `-0.052` y `+0.126 rad/s`; no cumple el reposo de `0.03 rad/s`
y expira. No se valida aun el retorno a cero ni F5 con pitch. Se requiere
acordar el siguiente ajuste de damping/control antes de continuar.

En 363 se redujo friccion `0.001 -> 0.00005 Nm`, se aumento damping
`0.005 -> 0.01 Nms/rad` y `kd` `0.003 -> 0.004`, conservando el resto. No hay
NaN y el error angular es `0.67 deg`, pero la velocidad sigue alternando entre
aproximadamente `-0.053` y `+0.127 rad/s`; el paso vuelve a expirar. La
hipotesis vigente es ruido/impulso numerico de velocidad realimentado por D.
No se aplicara filtrado sin un nuevo acuerdo funcional.

Con autorizacion posterior, 364 incorpora un filtro de velocidad configurable
de `0.05 s`, usa la señal filtrada en D y `JointState`, y conserva la cruda en
logs. Resultado **CONSEGUIDO**: `+30 deg` termina en `29.006 deg` y
`0.011370 rad/s`; el retorno termina en `0.988 deg` y `-0.013038 rad/s`, sin
NaN. La puerta aislada queda superada y se habilita la bateria completa.

La bateria 365 tambien queda **CONSEGUIDA** bajo GT_FORCED: `+30`, `-30`,
`+90` saturado explicitamente a `+70` y retorno neutral completan dentro de
`1 deg` y `0.03 rad/s`, sin NaN. El hover previo completa. Falta la regresion
de movimiento del dron con pitch no neutral para cerrar 1J.

La prueba 366 completa dos goals absolutos con pitch `+30` y `-30`, ambos
aceptados y bloqueados en `source=gt_forced`, y retorna a neutral. Sin NaN ni
fallos. Regresiones finales: `117/117` estimador/predictores, `3/3` fiduciales,
`3/3` evidencia visual y `35/35` contratos Python dirigidos. La suite Python
amplia conserva 2 fallos heredados ajenos (`52/54`).

Estado corregido tras auditoria transversal: **1J PARCIAL**. Modelo fisico y
control del joint estan conseguidos bajo GT_FORCED. F1, F3 y F4 conservan
correctamente semantica de camara, pero requieren regresiones live con pitch.
F5 ya compone body-camera por TF al stamp exacto y falla cerrado, aunque GT no
valida esa salida ORB. Ademas, el modelo dinamico sigue usando `1.4 kg` e
inercia body fija, mientras Gazebo añade `0.04 kg` moviles y torque de reaccion
del servo no observado. Debe acordarse esa politica y ejecutar pruebas ORB/body
antes de cerrar 1J.

Matiz confirmado por rastreo exhaustivo: F1-F4 productivas no consumen la
calibracion rigida fuera del wrapper, pero `src/vision/` contiene numerosos
prototipos legacy de nube/TSDF/planos con matrices `camara2cuerpo` fijas. No se
instalan ni lanzan hoy; cualquier reutilizacion en F6 obliga a migrarlos a TF
dinamica sincronizada.

Pruebas 367/367R: 367 fue invalida por ruta YAML relativa. 367R valido la
llegada bloqueada en `GT_FALLBACK`, pero el segundo goal tambien quedo en
fallback y expiro esperando autoridad ORB. El wrapper mantuvo tracking local,
pero no decodifico ningun fiducial (`decoded=0 valid=0` en todas las KFs), por
lo que no obtuvo pose global autoritativa ni inicializo gravedad O. Torque y
thrust si llegaron a buffers de unas 26 muestras; la cobertura `EMPTY` era un
resultado no calculado por faltar gravedad, no ausencia de actuacion. El body
se invalido por `DYNAMIC_BASE_NOT_READY`. No se ejecuto ningun comando pitch.
1J permanece **PARCIAL** y el STOP
queda antes de corregir esta precondicion de Fase 5.

Prueba 368: incluso con spawn determinista `(-1,-10,0.025), yaw=90`, el goal
GT a `(0,-10,1), yaw=90` termino por tiempo con `success=true`, pero el estado
al siguiente goal era aproximadamente `(-0.003,-6.973,0.812)`. Hubo deriva de
unos 3 m en Y; no se alcanzo la pose de observacion. Todas las KFs conservaron
`decoded=0 valid=0`, no hubo anchor ni ORB y no se movio el pitch. Se requiere
aproximacion correctiva y gate de error real antes del handoff.

Prueba 369: **PARCIAL**. Se implemento `navigation_source: None|GT|ORB` por
goal. El primer tramo solicito `GT` y arranco con `POSE_SOURCE_GT_FORCED=4`; el
segundo solicito `None`, restauro el launch ORB y, al faltar anchor, bloqueo
`POSE_SOURCE_GT_FALLBACK=3`. El barrido completo `69.013 -> -69.012 ->
69.006 deg`, asentado y sin timeout ni NaN. El fallo primario fue anterior a
ORB: GT_FORCED no llevo el dron al punto de observacion; al segundo goal la pose
era `(-0.005,-6.974,0.809)` frente a `(0,-10,1)`. Por no quedar frente al
fiducial no hubo anchor ni autoridad ORB, y el segundo goal uso fallback. El
selector y el servo quedan demostrados; la llegada y el handoff ORB no.
Revision causal: GT y el target world fueron correctos; la telemetria muestra
que el error de posicion crecio de `0.028` a casi `3 m` y el error de actitud
alcanzo aproximadamente `0.4 rad`. El pendiente es seguimiento del controlador
bajo GT, agravado por una action que finaliza por tiempo sin gate de llegada.

Pruebas 370-372: 370 reprodujo la deriva con rig fijo y midio `~0.0392 Nm`,
igual al momento gravitatorio de los `0.04 kg` descentrados. 370R fue invalida
por ruta YAML relativa. Con `1e-5 kg` en cada camara y rig, 370R2 llego con
error `0.0153 m` y torque `~0.000025 Nm`; 371 repitio con joint activo neutral
y error `0.0161 m`. En 372 el gate GT dio `0.0157 m`, el segundo goal arranco
con ORB real (`source=1`) y el barrido alcanzo `69.007/-69.013/69.035 deg` con
error de posicion maximo `0.0776 m`. ORB perdio tracking durante el barrido y
paso al fallback permitido.
