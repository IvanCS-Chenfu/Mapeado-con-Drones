# Contexto de compactacion

Checkpoint de reanudacion post-compactacion: releido fisicamente tras la
peticion mas reciente. Se conserva el STOP de 320R: no se ejecuta 321 ni se
realizan nuevos cambios funcionales. Trabajo activo limitado a sincronizar la
documentacion final de la poda ZOH y del fallo de integracion ORB productiva.

## Estado operativo

- Fase activa: Fase 5, subfase 5H.
- Estado agregado: `PARCIAL`.
- Ultima ejecucion: prueba 322, con comparacion 323 sobre las mismas muestras.
- Resultado funcional: diagnostico `CONSEGUIDO`; THREE_SAMPLE amplifica frente
  a TWO_SAMPLE y la propagacion dinamica degrada mucho mas la velocidad. La
  salida productiva no se ha corregido; Fase 5H permanece `PARCIAL`.
- Trabajo activo: ninguno.

## Preparacion integracion productiva post-317

- Preparacion: `CERRADA`.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `CONCEDIDA`.
- Propuesta: reutilizar en `StereoSlamNode` los estimadores causales y
  predictores dinamicos compartidos, preservar O/W y validar primero paridad
  con geometria diagnostica y despues hover ORB real repetible.
- Congelado propuesto: gains, J, masa, integradores, SMALL/MODERATE, reference
  gate, W, mux, goals y ORB-SLAM3 core.
- Matiz pendiente: 316/317 no usan overrides GT(now) en el estado de control,
  pero sus observaciones visuales base proceden de geometria GT diagnostica;
  no equivalen todavia a entrada visual ORB real.
- Decisiones: hueco de torque/thrust invalida prediccion y habilita fallback;
  aproximacion/anclaje en GT y nuevo goal de hover en ORB; selector temporal
  `legacy|dynamic`; 318 paridad, 319 primer hover ORB y 320 repeticion.
- Prueba acordada: 318 equivalente a 316/317; solo si pasa, 319 hover ORB real;
  solo si pasa, 320 repeticion identica. STOP en el primer fallo.
- Criterios: 318 completa sin fallback ni huecos; 319/320 completan hover con
  ORB gobernando, sin inestabilidad de control previa a una eventual perdida
  visual. GT solo truth y fallback real.
- Riesgos aceptados: la observacion ORB real puede descubrir problemas de
  calidad/reference/gates aunque la matematica compartida pase; no se tocaran
  gains, J, masa, estimadores o gates para ocultarlo.
- Dudas abiertas: ninguna.
- Siguiente accion exacta: localizar desde docs la frontera productiva, clases
  y topics; integrar con cambios minimos y crear YAMLs 318-320.

Checkpoint integracion post-317: `StereoSlamNode` incorpora selector temporal
`legacy|dynamic`, consume torque/thrust sellados, forma base comun p/v/R/omega
en cada medida O aceptada y propaga con las tres clases ya validadas. Huecos
publican estado local invalido para habilitar fallback. Launches pasan modo y
`physical.yaml`; creados YAMLs 318 paridad y 319/320 ORB real. El banco añade
alias `dynamic_318`; matematicas, gains, gates, W, mux y core intactos.
Siguiente accion: validar Python/YAML/diff y compilar `orbslam3`, despues
`dron_individual` y `simulacion_dron` si el primero pasa.

Checkpoint build integracion 1/3: primer comando rechazo mecanicamente el grupo
inexistente `fase5_productivo`; repetido con grupo `dron`. Build `orbslam3`
codigo 0 en 58.9 s, solo warnings heredados. Siguiente accion: compilar
`dron_individual` por launch/config y despues `simulacion_dron`.

Checkpoint build integracion 2/3: `dron_individual` codigo 0 en 1.64 s.
Siguiente accion: compilar `simulacion_dron` para instalar la propagacion del
selector de launch.

Checkpoint build/tests integracion: `simulacion_dron` codigo 0 en 0.75 s.
`orbslam3`, `dron_individual` y `simulacion_dron` compilan. GTest directo
94/94; la invocacion inicial con `--gtest_brief` solo mostro ayuda y no cuenta.
Suite del analizador 8/8 mediante harness directo porque `pytest` no esta
instalado; `git diff --check`, Python y YAML correctos.
Siguiente accion larga: prueba 318, YAML `tray_prueba_318.yaml`, modo
`dynamic_318`, geometria GT diagnostica, headless/debug, timeout 180 s y
espera 5 s. STOP si falla; si pasa, reducir y preparar 319 ORB real.

Checkpoint simulacion 318: runner/escenario codigo 0, `success=true`, 92 s;
guard inactivo y minimo 4847.5 MiB. Log completo `logs/prueba_318.log`
conservado sin lectura. La reduccion detecta un unico
`F5H-DYNAMIC-MISSING`: buffer de torque con una muestra, pero sin cobertura
para la base dinamica al inicio de un movimiento; no hubo fallback ni hueco
translacional y el escenario completo. Conforme al criterio estricto de 318
sin huecos, resultado `NO CONSEGUIDA` y STOP: 319/320 no ejecutadas. La causa
probable es que la primera muestra disponible esta sellada despues de la base;
el bootstrap actual solo cubre buffer vacio. Corregir la politica de arranque
es una decision funcional nueva. Autorizacion funcional: `SUSPENDIDA`.
Siguiente accion exacta: documentar integracion, build/tests y fallo 318; no
modificar ni ejecutar mas sin nuevo acuerdo.

Cierre documental post-318: historial, contrato 5H, resumen de fase, estado,
ultima sesion y documentacion de paquetes sincronizados. `git diff --check`
correcto. Trabajo activo: ninguno. Estado agregado: `PARCIAL`; siguiente
decision funcional pendiente: politica causal de cobertura al crear una base.

## Preparacion politica causal post-318

- Preparacion: `CERRADA`.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `CONCEDIDA`.
- Propuesta recibida: auditar cold start fisico; seed cero solo si se demuestra;
  ZOH explicito; conservar buffers ante resets visuales; cobertura conjunta
  torque/thrust; repetir 318 antes de ORB real.
- Decision tecnica: bajo ZOH, una orden conocida cubre el sufijo hasta la
  siguiente y un mensaje perdido no es detectable solo por timestamps. Los
  estados `MISSING_INTERNAL/SUFFIX` requieren secuencia, heartbeat o timeout
  contractual; esta iteracion usa solo `EMPTY/MISSING_PREFIX/FULL`.
- Prueba acordada: 318R paridad, 319R repeticion, 320 ORB real y 321 repeticion
  ORB real, con STOP en el primer fallo.
- Alcance congelado: estimadores, J, masa, gravedad, integradores, gains,
  gates, W, mux y ORB-SLAM3 core.
- Dudas abiertas: ninguna.
- Siguiente accion exacta: auditar cold start de la cadena de actuacion y
  localizar resets/cobertura de ambos buffers; despues implementar y probar.

Checkpoint auditoria cold start: `aplicar_fuerzas_dron` inicializa
`fuerza_total=0` y `torque=0`, publica esa mezcla a 50 Hz desde su arranque;
`plugin_actuar_motores` inicializa a cero las fuerzas/torques de los motores
y los reaplica por ZOH en cada tick hasta recibir otra orden. El cero inicial
esta demostrado. `StereoSlamNode::ResetDynamicNavigationState` borra hoy
ambos buffers ante resets visuales y debe dejar de hacerlo. Predictores ya
integran ZOH correctamente; falta formalizar cobertura y seed explicito.
Siguiente accion: editar predictores, nodo productivo, GTests y YAMLs.

Checkpoint cambios causales: añadidos `ActuationCoverageStatus`
`EMPTY/MISSING_PREFIX/FULL` y cobertura explicita en torque/thrust; ZOH
existente se conserva. `StereoSlamNode` y el banco diagnostico insertan seed
cold-start cero sellado y trazable; el reset visual conserva ambos buffers.
El banco usa ya el stamp real del torque. Telemetria informa estado,
oldest/newest y prefijo faltante. Añadidos GTests y YAMLs 318R, 319R y 321;
320 conserva el hover ORB acordado. Python, YAML y `git diff --check`
correctos. Siguiente accion larga: build `orbslam3`.

Checkpoint build causal: `orbslam3` codigo 0 en 1 min 16 s; solo warnings
heredados de ORB-SLAM3/Eigen. Siguiente accion: GTest directo, analizador y
despues builds de launches si las suites pasan.

Checkpoint suites/builds causales: GTest directo 98/98 y analizador 8/8.
`dron_individual` codigo 0 en 1.41 s y `simulacion_dron` codigo 0 en
0.57 s. El primer intento de `dron_individual` fallo solo porque el sandbox
impidio crear el log fuera de `src`; repetido con permiso operativo, sin
cambio de codigo. Siguiente accion larga: 318R con modo `dynamic_318`, YAML
de paridad identico, timeout 180 s, espera 5 s y guard de recursos.

Checkpoint simulacion 318R: runner codigo 0, `success=true`, 92 s; guard
inactivo y minimo 5129.5 MiB. Log completo `logs/prueba_318R.log` conservado
sin lectura. Siguiente accion: reducir seed, cobertura, missing, fuentes,
tracking y escenario; solo si cumple todo ejecutar 319R.

Cierre funcional 318R: `NO CONSEGUIDA`. El escenario completa y no hay
fallback, pero aparece un unico `F5H-DYNAMIC-MISSING`:
`base=1788101246.970402241`, primera/ultima muestra
`1788101247.040409803`, `MISSING_PREFIX=0.070007563 s`. El seed cero se
creo en `1788101214.790371656`, pero `AddTorque` recorta todo lo anterior a
0.5 s al entrar el primer comando despues de unos 32 s de inactividad. Se
pierde asi el predecesor ZOH que demuestra el cero vigente. STOP aplicado:
319R/320/321 no ejecutadas. Siguiente correccion propuesta: al podar conservar
siempre una muestra predecesora al horizonte, sin ampliar ilimitadamente el
buffer ni inventar valores. Autorizacion funcional: `SUSPENDIDA`.
Siguiente accion exacta: documentar 318R y cerrar sin nuevas modificaciones.

Cierre documental 318R: historial, contrato, estado, pipeline, ultima sesion y
docs de paquetes sincronizados; `git diff --check` correcto. Trabajo activo:
ninguno. Estado agregado: `PARCIAL`. Preparacion siguiente: `NO_INICIADA`;
acuerdo cerrado: `no`; autorizacion funcional: `SUSPENDIDA`; prueba futura
pendiente de debate. Dudas abiertas: ninguna sobre el diagnostico; falta
acordar la correccion del pruning con predecesor ZOH.

## Preparacion poda ZOH post-318R

- Preparacion: `CERRADA`.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `CONCEDIDA`.
- Objetivo: conservar exactamente la ultima muestra `<= cutoff` y todas las
  posteriores, con poda al insertar y la misma politica en torque/thrust.
- Alcance congelado: estimadores, dinamica, J, masa, gravedad, integradores,
  gains, gates, KF/reference, W, mux, trayectoria y timing.
- Telemetria de poda: solo debug y solo cuando haya cambio/eliminacion; no
  generar ruido a 50 Hz.
- Prueba acordada: 318R2 paridad; si pasa, 319R repeticion; si pasa, 320 ORB
  real; si pasa, 321 repeticion ORB. STOP en el primer fallo.
- Criterio conceptual: 318R2/319R validan poda/cobertura/paridad; solo 320
  puede validar la integracion productiva ORB.
- Dudas abiertas: ninguna.
- Siguiente accion exacta: modificar solo la poda compartida, añadir GTests,
  compilar y ejecutar la bateria condicional.

Checkpoint cambios poda ZOH: `AddTorque/AddThrust` conservan `front()`
mientras la segunda muestra no haya quedado `<= cutoff`; asi mantienen una
predecesora y todas las recientes. Añadidos cuatro GTests de espera larga,
sustitucion de predecesora y memoria acotada; actualizado el test de poda
previo. Creado YAML 318R2 identico a 318R. YAML y `git diff --check`
correctos. Siguiente accion larga: build `orbslam3` y GTest esperado 102/102.

Checkpoint build/tests poda: `orbslam3` codigo 0 en 17.3 s; GTest directo
102/102 y analizador 8/8. Siguiente accion larga: 318R2, YAML identico a 318R,
modo `dynamic_318`, timeout 180 s, espera 5 s y guard de recursos. STOP si
aparece missing, fallback, fallo de escenario o inestabilidad.

Checkpoint simulacion 318R2: runner codigo 0, `success=true`, 92 s; guard
inactivo y minimo 5065.2 MiB. Log completo `logs/prueba_318R2.log`
conservado sin lectura. Siguiente accion: reducir cobertura/missing/fuentes y
solo si cumple el contrato ejecutar 319R sin cambios.

Cierre 318R2: `CONSEGUIDA`. Cero eventos reales
`PRODUCTIVE/DYNAMIC/TRANSLATIONAL-MISSING`, cero fallback, tracking loss,
NaN o FATAL; ambos goals correctos y fuentes con `fallback_samples=0`.
La unica coincidencia de esos textos era la cabecera del patron reducido.
Poda ZOH validada en primera ejecucion. Siguiente accion larga: 319R identica,
sin cambios, para confirmar paridad/cobertura.

Checkpoint simulacion 319R: runner codigo 0, `success=true`, 90 s; guard
inactivo y minimo 5063.6 MiB. Log completo `logs/prueba_319R.log` conservado
sin lectura. Siguiente accion: reducir y comprobar exactamente los mismos
criterios antes de autorizar 320 ORB real.

Cierre 319R: `CONSEGUIDA`; cero missing real, fallback, tracking loss,
NaN/FATAL y ambos goals correctos. Poda ZOH, cobertura de actuacion y paridad
dinamica `VALIDADAS`. Esto no valida aun la ruta productiva. Siguiente accion
larga: 320 con `f5h_gt_timing_mode=off`,
`orb_navigation_prediction_mode=dynamic`, aproximacion/anclaje mediante
fallback temporal y nuevo goal de hover con ORB real.

Checkpoint simulacion 320 ORB real: runner codigo 0, `success=true`, 90 s;
guard inactivo y minimo 4063.4 MiB. ORB llego a 1082.4 MiB RSS sin activar
guard. Log completo `logs/prueba_320.log` conservado sin lectura. Siguiente:
reducir fuentes/handoff, productive coverage, tracking, goals y control antes
de decidir si 321 queda autorizada.

Cierre 320: `INVALIDA PARA DINAMICA PRODUCTIVA`. El escenario completa y el
segundo goal arranca con ORB (`source=1`), pero ambos `stereo` registran
`[F5H-PRODUCTIVE-PREDICTOR] mode=legacy` pese al argumento launch
`orb_navigation_prediction_mode:=dynamic`. Por tanto no valida la rama
integrada. STOP: 321 no ejecutada. La perdida del argumento es un defecto
mecanico de propagacion launch que debe localizarse; no se tocaran estimadores
ni se interpretara la estabilidad de 320 como evidencia dinamica.

Diagnostico mecanico 320: `orbslam_use.launch.py` cargaba `stereo_params`
antes de `navigation_state.yaml`; el valor YAML `legacy` sobrescribia el
argumento launch `dynamic`. Correccion mecanica dentro del acuerdo: cargar
`stereo_params` al final para que el override explicito tenga precedencia.
La repeticion se registra como 320R; no cambia YAML ni funcionalidad acordada.

Checkpoint correccion launch: Python y `git diff --check` correctos; build
`dron_individual` codigo 0 en 1.30 s. Siguiente accion larga: repetir el mismo
YAML ORB como 320R y exigir marcador `mode=dynamic` antes de interpretar.

Checkpoint simulacion 320R: runner codigo 0, `success=true`, 92 s; guard
inactivo y minimo 4099.9 MiB. Log completo `logs/prueba_320R.log` conservado
sin lectura. Siguiente: reducir y verificar primero `mode=dynamic`, luego
missing, fuente ORB, tracking, control y goals.

Cierre 320R: `NO CONSEGUIDA`. Ambos stereo usan `mode=dynamic` y no hay
missing de actuacion. Sin embargo ORB gobierna ya el primer goal, no la
aproximacion GT acordada. Al terminar ese goal, el segundo arranca desde
`x0=(-0.030342,-8.995524,0.077654)`,
`v0=(0.064187,-0.625828,0.930526)` frente al objetivo
`(0,-10,1)`: el control no habia seguido la aproximacion aunque la action
finaliza por tiempo. Durante el hover, tras unos 20.3 s, tracking pasa
`2->3`, el mux conmuta ORB->GT con salto cero y queda bloqueado en fallback;
tracking recupera `3->2` 59 ms despues. Hay error/inestabilidad previa a la
perdida, por lo que 321 no se ejecuta. La prueba 320 anterior queda
`INVALIDA` por modo legacy. Estado: poda/cobertura/paridad validadas;
integracion productiva ORB `NO VALIDADA`. Autorizacion funcional:
`SUSPENDIDA`.
Siguiente accion exacta: documentar resultados y cerrar sin mas cambios.

Cierre documental post-320R: historial, contrato 5H, docs de paquetes, estado,
pipeline, indice y ultima sesion sincronizados; `git diff --check` correcto.
Resultado agregado `PARCIAL`: poda/cobertura/paridad `VALIDADAS`, integracion
ORB productiva `NO VALIDADA`. Preparacion siguiente: `NO_INICIADA`; acuerdo
cerrado: `no`; autorizacion funcional: `SUSPENDIDA`; prueba 321 detenida.
Trabajo activo: ninguno. Dudas abiertas: ninguna sobre esta ejecucion.

## Preparacion ORB shadow post-320R

- Preparacion: `CERRADA`.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `CONCEDIDA`.
- Propuesta recibida: mantener GT autoritativo durante aproximacion y
  asentamiento, ejecutar el mismo ORB dinamico en sombra y abrir una unica
  frontera GT->ORB al comenzar un goal nuevo de hover.
- Alcance propuesto: modificar solo la politica diagnóstica de autoridad y su
  telemetria; congelar estimadores, predictores, fisica, gains, buffers, gates,
  referencia KF, W y ORB-SLAM3 core.
- Prueba propuesta: 320R2 valida frontera y hover; solo si pasa, 321 identica.
- Criterio clave: cualquier autoridad ORB antes del handoff invalida 320R2;
  un salto grande ya presente en el handoff detiene la bateria.
- Decisiones: GT bloquea el handoff hasta confirmar estacionariedad; ventana de
  asentamiento de `1.5 s`; ambas son logica temporal diagnostica de Fase 5.
- Prueba acordada: 320R2; solo si pasa todos los criterios, 321 identica. STOP
  ante prueba invalida, salto grande de handoff o divergencia del hover.
- Dudas abiertas: ninguna.
- Plan: añadir al mux un modo diagnostico `shadow_gt`, mantener el ORB dinamico
  productivo activo en paralelo, abrir autoridad mediante un servicio solo tras
  tracking+anchor+estado consumible y `1.5 s` de estacionariedad GT, y hacer que
  el runner invoque esa frontera antes del nuevo goal de hover. Añadir tests de
  gate/frontera y telemetria de shadow/handoff; no tocar estimadores ni control.
- Archivos criticos: `navigation_state_mux.cpp/.hpp`, sus tests y CMake;
  `scenario_runner_node.cpp`, docs de ambos componentes y YAMLs 320R2/321.
- Umbrales diagnosticos iniciales no estrictos: velocidad lineal y angular GT
  <= `0.15` en unidades SI, configurables y exclusivos del modo shadow.
- Siguiente accion exacta: localizar tests y propagacion launch/config de mux y
  runner; despues editar el bloque minimo acordado.

Checkpoint cambios ORB shadow: añadido `OrbShadowActivationGate` testeable;
el mux admite `shadow_gt`, publica ORB productivo en sombra y expone
`control/activate_orb_shadow`, que solo acepta tras tracking, anchor, estado
consumible y `1.5 s` bajo `0.15 m/s` y `0.15 rad/s`. El runner incorpora el
paso generico `call_set_bool`; launch propaga `f5h_orb_shadow_mode`. Creados
320R2 y 321 identicos en frontera/hover. No cambiaron estimadores, predictores,
control, gains ni fisica. Python, YAML y `git diff --check` correctos; el intento
inicial de validar YAML con Ruby fallo solo porque Ruby no esta instalado y se
repitio correctamente con PyYAML. Siguiente accion larga: compilar primero
`dron_individual` y despues `simulacion_dron`; STOP ante fallo funcional.

Checkpoint build ORB shadow 1/2: `dron_individual` codigo 0 en 28.7 s.
Siguiente accion larga: compilar `simulacion_dron` por el nuevo paso
`call_set_bool` y su dependencia `std_srvs`.

Checkpoint build ORB shadow 2/2 intento 1: `simulacion_dron` codigo 2 en
8.64 s. Primer error real: `ament_target_dependencies` no encuentra
`std_srvs` porque falta `find_package(std_srvs REQUIRED)` en CMake. Correccion
mecanica dentro del acuerdo: añadir esa declaracion y repetir el mismo build.

Checkpoint build ORB shadow 2/2 final: tras añadir `find_package`,
`simulacion_dron` codigo 0 en 18.8 s. Ambos paquetes compilan. Siguiente
accion: ejecutar GTest focal del mux, GTest del estimador productivo y suite
del analizador antes de autorizar 320R2.

Checkpoint suites ORB shadow: mux 11/11 GTests, estimador productivo 102/102 y
analizador 8/8 mediante harness directo. La invocacion previa con `unittest`
descubrio cero tests por ser funciones estilo pytest y no cuenta. Siguiente
accion larga: prueba 320R2 con `tray_prueba_320R2.yaml`, launch multi-dron
headless, fallback habilitado, predictor `dynamic`, shadow habilitado, debug y
metricas; timeout 180 s, espera final 5 s y guard de recursos. Tras terminar se
registrara el resultado antes de reducir; 321 sigue condicionada.

Checkpoint simulacion 320R2 intento 1: runner codigo 1 tras unos 26 s;
guard inactivo y minimo 4045.9 MiB. Log completo
`logs/prueba_320R2.log` conservado sin lectura. Siguiente accion: reducir solo
errores de runner/YAML/servicio para determinar si es un fallo mecanico de
arranque; 321 permanece detenida.

Diagnostico 320R2 intento 1: `INVALIDA` por ruta YAML relativa; el runner
registra `Could not load scenario YAML ... bad file` antes del primer paso.
No hubo prueba funcional. Correccion mecanica: conservar este intento y repetir
como 320R2R pasando la ruta YAML absoluta, sin cambios de codigo, escenario ni
criterios. 321 permanece detenida.

Checkpoint simulacion 320R2R: runner codigo 0, `success=true`, 91 s; guard
inactivo y minimo 3941.7 MiB. Log completo `logs/prueba_320R2R.log`
conservado sin lectura. Siguiente accion: reducir predictor, pasos, fuentes,
shadow, readiness, activacion, handoff, control, tracking, missing y fallback;
solo si todos los criterios pasan se ejecutara 321.

Cierre funcional 320R2R: `NO CONSEGUIDA`, caso B/D del contrato. Validez de
frontera confirmada: predictor `dynamic`; aproximacion completa con source GT;
ORB en shadow; tracking+anchor+estado valido y asentamiento `1.5 s`; activacion
solo en la frontera del nuevo goal. Handoff: salto pose/rotacion `0`, salto de
velocidad `0.247078 m/s` y omega `0.003635 rad/s`; el goal ORB arranca en
`(0.011726,-10.002154,1.006141)` con
`v=(0.029463,-0.155310,0.194482)`. No hay missing de actuacion, tracking se
mantiene en `2` y no aparece fallback, pero el error de posicion crece hasta
aprox. `1.63 m`, el de velocidad presenta picos de `1.75 m/s` y el angular
alcanza aprox. `0.52 rad`. La action y el wait terminan, pero el hover diverge
con ORB aun sano. Activacion prematura de 320R queda confirmada como defecto de
esa prueba, pero queda descartada como causa suficiente del fallo productivo.
STOP aplicado: 321 no ejecutada. Autorizacion funcional: `SUSPENDIDA`.
Siguiente accion exacta: documentar 320R2 invalida y 320R2R fallida, actualizar
docs de mux/runner/launch y cerrar sin nuevos cambios funcionales.

Cierre documental post-320R2R: docs de `dron_individual` y
`simulacion_dron`, contrato 5H, historial largo/resumen, indice, estado,
pipeline y ultima sesion sincronizados. Resultado agregado `PARCIAL`; hover ORB
productivo `NO VALIDADO`. Preparacion siguiente: `NO_INICIADA`; acuerdo
cerrado: `no`; autorizacion funcional: `SUSPENDIDA`; 321 detenida. Trabajo
activo: ninguno. Dudas abiertas: ninguna sobre el resultado de 320R2R.

## Preparacion bateria ORB real p/v post-320R2R

- Preparacion: `CERRADA`.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `CONCEDIDA`.
- Propuesta recibida: eliminar primero la carrera activation/goal confirmando
  source ORB antes de enviar el hover y ejecutar 321A-D con sustituciones
  diagnosticas p/v GT para aislar posicion, velocidad y canal angular ORB.
- Alcance propuesto: instrumentacion diagnostica y seleccion de componentes en
  la salida de control; estimadores, predictores, fisica, gains, buffers, gates,
  referencia KF, W y ORB-SLAM3 core congelados.
- Evidencia que motiva la correccion: en 320R2R el goal fue aceptado con
  snapshot GT y el mux publico ORB unos milisegundos despues.
- Pruebas acordadas: 321A pGT/vORB, 321B pORB/vGT, 321C p/vGT y 321D ORB
  completo, siempre con angular ORB y autoridad confirmada antes del goal.
- Politica de ejecucion: ejecutar A-D completas aunque haya fallos funcionales;
  detener solo ante fallo de infraestructura o seguridad.
- Criterio numerico comun de hover estable: maximo `|e_p| <= 0.35 m`, maximo
  `|v| <= 0.50 m/s` y maximo `|e_R| <= 0.20 rad`, con tracking ORB sano y sin
  fallback.
- Riesgos aceptados: las sustituciones GT son diagnosticas y temporales; no
  validan por si mismas el control ORB completo ni deben entrar en sus buffers.
- Dudas abiertas: ninguna.
- Siguiente accion exacta: localizar la frontera de autoridad y salida de
  control, implementar confirmacion ORB y overrides p/v, validar y ejecutar
  321A-D.

Checkpoint de reanudacion bateria 321: contexto fisicamente releido y
reconciliado con la autorizacion mas reciente. Se ejecutara el bloque completo
con los umbrales acordados y sin alterar estimadores, dinamica o control.

Checkpoint plan bateria 321: frontera localizada en `navigation_state_mux`,
launches `generar_dron`/`multi_dron` y `wait_for_bool` transient-local del
runner. Plan: publicar autoridad ORB efectiva, alinear GT al O continuo solo
para overrides de salida, crear 321A-D, compilar/testear, ejecutar las cuatro y
documentar comparativa. Archivos criticos y criterios quedan fijados; dudas
abiertas: ninguna.

Checkpoint cambios bateria 321: el mux publica
`control/orb_authority_confirmed` y el marcador ordenado tras publicar el
primer estado ORB; soporta `normal|position_gt|velocity_gt|position_velocity_gt`
solo en salida de control, con alineamiento rigido GT capturado en handoff.
Orientacion y omega permanecen ORB. Launches propagan el selector; creados
YAMLs 321A-D con espera explicita antes del goal; añadidos dos GTests. Siguiente
accion: validacion estatica y builds.

Checkpoint build bateria 1/2: `dron_individual` compila con codigo 0 en
30.2 s. Siguiente accion: compilar `simulacion_dron` y ejecutar GTests del mux.

Checkpoint builds/tests bateria: `simulacion_dron` compila con codigo 0 en
0.82 s; GTest del mux 13/13. Validacion Python/YAML/diff tambien correcta.
Siguiente accion larga: ejecutar 321A con `position_gt`, autoridad ORB
confirmada antes del goal y criterios comunes acordados; despues continuar
B-D aunque haya fallos funcionales.

Checkpoint simulacion 321A intento 1: runner codigo 1 tras 83 s; guard
inactivo, minimo 3468.6 MiB. Log completo `logs/prueba_321A.log` conservado
sin lectura. Siguiente accion: reducir solo runner, activation, authority,
source y errores para clasificar si el fallo es funcional o de infraestructura
antes de decidir la continuidad A-D.

Diagnostico 321A intento 1: `INVALIDA`. Aproximacion y activacion completan,
pero el lock retenido por el goal GT devuelve
`trajectory_source_locked`; la autoridad ORB no puede confirmarse antes del
nuevo goal y el wait expira a 20 s. Correccion mecanica dentro del contrato:
abrir explicitamente `control/set_trajectory_active=false` antes de activar,
esperar autoridad y despues enviar el goal que vuelve a bloquear ORB. Aplicado
a A-D; siguiente accion: repetir como 321AR.

Checkpoint simulacion 321AR: runner codigo 0, `success=true`, 91 s; guard
inactivo y minimo 3516.3 MiB. Log completo `logs/prueba_321AR.log` conservado
sin lectura. Conforme a la politica acordada, siguiente accion larga: ejecutar
321B sin recalibrar; el analisis comparativo se hara tras completar A-D.

Checkpoint simulacion 321B: runner codigo 0, `success=true`, 90 s; guard
inactivo y minimo 3509.1 MiB. Log completo `logs/prueba_321B.log` conservado
sin lectura. Siguiente accion larga: 321C con p/v GT y angular ORB.

Checkpoint simulacion 321C: runner codigo 0, `success=true`, 92 s; guard
inactivo y minimo 3483.2 MiB. Log completo `logs/prueba_321C.log` conservado
sin lectura. Siguiente accion larga: 321D baseline ORB completo con autoridad
confirmada antes del goal.

Checkpoint simulacion 321D: runner codigo 0, `success=true`, 92 s; guard
inactivo y minimo 3375.7 MiB. Log completo `logs/prueba_321D.log` conservado
sin lectura. Bateria valida ejecutada completa: 321AR/B/C/D. Siguiente accion:
reducir los cuatro logs y comparar autoridad, validez, tracking, fallback,
missing y umbrales de hover.

Cierre funcional bateria 321: 321A inicial `INVALIDA` por lock retenido y
repetida validamente como 321AR. En todas las validas, autoridad precede al
goal, predictor es `dynamic`, cobertura FULL y missing cero. Maximos de hover
`ep/v/er`: 321AR `1.604/2.428/0.931` (falla), 321B
`0.159/0.046/0.027` (cumple), 321C `0.322/0.693/0.667` antes de caer a
fallback (falla y no aisla angular), 321D `1.761/2.799/0.965` (falla).
321AR/B/D conservan tracking 2 sin fallback; C pierde consumibilidad local con
tracking numerico 2 y conmuta. Diagnostico `CONSEGUIDO`: `v_ORB` es la causa
principal demostrada; hover ORB completo `NO CONSEGUIDO`, 5H `PARCIAL`.

Cierre documental bateria 321: contrato 5H, historial largo/resumen, indice,
estado, ultima sesion y docs de `dron_individual`/`simulacion_dron`
sincronizados. Autorizacion funcional: `SUSPENDIDA` por bateria consumida.
Trabajo activo: ninguno. Dudas abiertas: ninguna. Siguiente acuerdo debe
corregir `v_hat` lineal sin GT y repetir ORB completo.

Verificacion final: `git diff --check` y Python correctos. El GTest valido ya
registrado permanece 13/13; una invocacion final con `--gtest_brief` solo
mostro ayuda porque este binario no admite ese flag y no cuenta como ejecucion.

## Preparacion diagnostico v_hat post-321

- Preparacion: `CERRADA`.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `CONCEDIDA`.
- Propuesta recibida: prueba 322 con ORB productivo real en shadow y control
  estable externo; instrumentar p(k-2..k), velocidades midpoint, aceleracion,
  `v_hat(t_k)`, propagacion hasta now, dt, reference KF y correction class;
  ejecutar 323 TWO_SAMPLE/THREE_SAMPLE paralelo solo si 322 demuestra
  amplificacion entre midpoint y v_hat.
- Alcance propuesto: telemetria y analisis exclusivamente; no cambiar
  estimador, predictores, dinamica, gains, gates, W ni salida productiva.
- Ajuste tecnico acordado: usar GT completo como autoridad en 322, pues
  321C ya demostro que p/v GT con angular ORB puede perder consumibilidad y no
  garantiza una ventana estable.
- Duda critica: fijar explicitamente el puente entre stamp fisico de imagen,
  stamp interno usado por el estimador y stamp ROS de recepcion/publicacion;
  `v_GT(t_k)` debe interpolarse en el mismo dominio fisico y la propagacion
  compararse en el dominio de `target_stamp`.
- Prueba acordada: 322 obligatoria con GT completo gobernando y ORB productivo
  integro en shadow; 323 solo si 322 muestra `v_mid` razonable y degradacion
  clara posterior en `v_hat`.
- Criterios: ventana airborne+settled con tracking ORB y medidas reales,
  predictor dynamic, cobertura FULL y sin missing; diagnostico cuantitativo por
  etapa, dt, reference KF y correction class. Una perdida posterior permite
  analizar solo el tramo valido previo si es suficiente.
- Riesgos aceptados: GT es truth/metrica y autoridad diagnostica, nunca entrada
  del estimador; la instrumentacion puede aumentar logs pero no la latencia de
  la salida productiva.
- Dudas abiertas: ninguna.
- Siguiente accion exacta: localizar telemetria/analizador existentes,
  implementar solo observabilidad y ejecutar 322; decidir 323 con sus datos.

Checkpoint plan diagnostico 322: `CausalLinearVelocityEstimate` ya expone
p(k-2..k), dt, midpoints, v_mid, aceleracion, horizonte y v_hat; la ruta
productiva solo registra un resumen. `pose_metrics_node` conserva GT con stamp
fisico y receive ROS, pero no escribe velocidad lineal. Se añadira un marcador
por medida, campos lineales al CSV GT y un analizador offline que interpole en
cada dominio y alinee rigidamente O->GT. 322 usara GT completo gobernando y
ORB dynamic en shadow; 323 queda condicional. Dudas abiertas: ninguna.

Checkpoint de reanudacion diagnostico 322: contexto fisico releido y
reconciliado con la ultima orden. Ya estan instrumentados el marcador lineal
productivo, la velocidad GT en el CSV y el analizador offline, sin cambios en
la salida del estimador. Siguiente accion: completar en el analizador la
ventana settled, correction class y entorno de cambios de referencia; crear el
escenario 322, validar y compilar antes de ejecutar.

Checkpoint cambios diagnostico 322: marcador productivo lineal y CSV GT
incluyen las magnitudes acordadas; el analizador separa dominios fisico/ROS,
recorta desde `ORB-SHADOW settled=true` y agrupa por modo, correction class y
ventana de cambio de referencia. Creado `tray_prueba_322.yaml` con autoridad
GT completa y hover ORB shadow. Analizador 4/4, YAML y `git diff --check`
correctos. Siguiente accion larga: build `orbslam3`, seguido de los paquetes
de lanzamiento/metricas si pasa.

Checkpoint build/tests diagnostico 322: `orbslam3`, `dron_individual` y
`simulacion_dron` compilan con codigo 0; solo warnings heredados en ORB-SLAM3.
GTest `navigation_state_estimator` 102/102 y analizador lineal 4/4. Siguiente
accion larga: ejecutar 322 headless con autoridad GT, ORB shadow dynamic,
metricas activas, timeout 180 s y espera posterior de 5 s.

Checkpoint simulacion 322: runner codigo 0, `success=true`, 97 s; guard
inactivo y minimo 3594.3 MiB. Log completo `logs/prueba_322.log` conservado
sin lectura. Siguiente accion: reducir marcadores de escenario/autoridad/
tracking/cobertura y ejecutar el analizador lineal sobre el log como entrada
de reduccion, antes de decidir condicionalmente la prueba 323.

Cierre diagnostico 322/323: `CONSEGUIDO`. Ventana settled de unos 43 s,
tracking drone1=2, autoridad GT, ORB `dynamic`, 907 medidas validas y cero
missing. RMSE `v_mid=0.01984`, TWO_SAMPLE en `t_k=0.01988`, THREE_SAMPLE
`v_hat_tk=0.03457` y `v_dynamic_now=0.43308 m/s`. La 323 se calcula en
paralelo sobre las mismas muestras de 322, como preferia el acuerdo. Conclusion
`MULTICAUSAL`: `A_HAT_AMPLIFICATION` demostrada y degradacion dominante
adicional en `DYNAMIC_PROPAGATION`; DEGRADED_DT y reference KF no son causa
principal. Salida productiva sin cambios. Documentacion sincronizada y STOP
antes de corregir. Autorizacion funcional: `SUSPENDIDA`; trabajo activo:
ninguno; dudas abiertas: ninguna.

Verificacion final diagnostico 322/323: analizador 4/4, resumen regenerado con
907 medidas y un cambio de referencia dentro de la ventana settled,
`git diff --check` correcto y ningun proceso de simulacion activo.

## Preparacion gravedad en O post-323

- Preparacion: `CERRADA`.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `CONCEDIDA`.
- Auditoria inicial: `BodyThrustDynamicPredictor::Predict` recibe estado y
  orientacion realmente expresados en O, calcula `O_R_B * F_B / m`, pero suma
  `gravity_world_=(0,0,-9.81)` sin transformarla. La hipotesis GRAVITY_FRAME
  queda confirmada en codigo.
- Relacion existente: `StereoSlamNode` calcula `o_t_world_ = O_T_B *
  inverse(W_T_B)`, es decir `O_T_W`; hoy se actualiza con estados globales no
  invalidos. Para gravedad debe usarse solo el primer estado AUTHORITATIVE del
  epoch y congelar `O_R_W` hasta un epoch nuevo.
- Alcance propuesto: corregir solo gravedad O, ciclo por epoch, validez,
  telemetria/tests y repetir 322 como 324; 325 solo si 324 mejora claramente.
  THREE_SAMPLE y resto de dinamica/control quedan congelados.
- Prueba propuesta: 324 shadow GT identica a 322; 325 repeticion condicional.
- Politica confirmada: mientras no exista gravedad O obtenida de autoridad
  global `AUTHORITATIVE`, el predictor translacional ORB dinamico no es
  consumible y Fase 5 continua mediante GT fallback.
- Riesgos aceptados: el primer anclaje autoritativo fija la gravedad del epoch;
  revisiones posteriores no la corrigen para preservar continuidad. Un nuevo
  epoch la invalida.
- Dudas abiertas: ninguna.
- Siguiente accion exacta: implementar estado de gravedad O por epoch usando
  `o_t_world_=O_T_W`, tests de frame/freeze/epoch y telemetria; compilar y
  ejecutar 324, con 325 solo ante mejora clara.

Checkpoint cambios gravedad O: añadido `EpochGravityState`, inicializacion
desde el primer `O_T_W` autoritativo, freeze por epoch, invalidacion al cambiar
epoch y bloqueo dynamic mientras no sea valida. `BodyThrustDynamicPredictor`
suma ahora la gravedad O configurada y expone componentes diagnosticas. Seis
GTests cubren identidad, rotacion, sentido, norma, hover rotado, freeze y epoch;
analizador añade residual de aceleracion y YAML 324 replica 322. THREE_SAMPLE,
integradores, buffers, gains y control intactos. Siguiente accion: validaciones
mecanicas y build `orbslam3`.

Checkpoint build/tests gravedad O: `orbslam3`, `dron_individual` y
`simulacion_dron` codigo 0; solo warnings heredados. GTest sube y pasa 108/108,
analizador 5/5, YAML y `git diff --check` correctos. Siguiente accion larga:
prueba 324 identica a 322, GT gobierna, ORB dynamic shadow, timeout 180 s y
metricas en `metricas/prueba_324`.

Checkpoint simulacion 324: runner 0, `success=true`, 100 s; guard inactivo y
minimo 3322.3 MiB. Log completo `logs/prueba_324.log` conservado sin lectura.
Siguiente accion: reducir autoridad/tracking/gravedad/cobertura, ejecutar el
analizador lineal y comparar 322/324 antes de decidir 325.

Diagnostico 324: valido y mejora fuerte. Una sola inicializacion autoritativa
da `g_O=(0.544811,9.792267,0.225417)`, norma `9.810001`. Sobre 875 medidas,
RMSE mid/two/three/dynamic=`0.02119/0.02122/0.03613/0.03583 m/s`; dynamic baja
desde `0.43308` de 322 y gain RMSE `12.53 -> 0.99`. Residual de aceleracion
mediano `0.0129 m/s2`, p95 `0.1330`. Hipotesis GRAVITY_FRAME confirmada y
mejora clara; 325 queda autorizada por el acuerdo como repeticion sin cambios.
Siguiente accion larga: ejecutar 325 identica y analizar reproducibilidad.

Checkpoint simulacion 325: runner 0, `success=true`, 95 s; guard inactivo y
minimo 3329.9 MiB. Log completo `logs/prueba_325.log` conservado sin lectura.
Siguiente accion: reducir gravedad/validez y analizar con la misma herramienta
antes del cierre documental.

Cierre 324/325: ambas validas y reproducibles. Tras filtrar explicitamente
`dron_1` en el analizador, 324 tiene 875 medidas y RMSE dynamic `0.03583`; 325
906 medidas y `0.03707 m/s`, frente a `0.43308` en 322. Gains RMSE
`0.992/1.003` frente a `12.53`; gravedad inicializada una vez por ejecucion con
norma 9.81. Resultado `GRAVITY_FRAME CONFIRMADO` y
`DYNAMIC_PROPAGATION CORREGIDA`. THREE_SAMPLE permanece intacto y pendiente.
GTest 108/108, analizador 6/6, tres builds correctos. Autorizacion funcional:
`SUSPENDIDA`; trabajo activo: ninguno; dudas abiertas: ninguna. Siguiente
acuerdo: corregir aisladamente `A_HAT_AMPLIFICATION`.

## Preparacion MIDPOINT_DYNAMIC post-325

- Preparacion: `CERRADA`.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `CONCEDIDA`.
- Propuesta: diagnosticar en shadow `MIDPOINT_DYNAMIC`, interpretando la
  diferencia de dos posiciones como `v_mid(t_mid)` y propagandola fisicamente
  hasta `t_k`; comparar en las mismas muestras contra TWO_SAMPLE y
  THREE_SAMPLE sin cambiar aun la salida productiva.
- Alcance propuesto: prueba 326 en hover GT con ORB shadow y, si las variantes
  empatan, 327 con aceleracion suave; solo despues decidir una sustitucion
  productiva y sus pruebas 328-331.
- Congelado: gravedad O, masa, J, predictores, buffers/ZOH, gains, gates,
  referencias, W, mux y ORB-SLAM3 core.
- Puente acordado: `t_mid_ros=callback_arrival_ros_k-(t_k_image-t_mid_image)`;
  `R_mid` se obtiene por interpolacion SO(3) entre las dos orientaciones
  visuales aceptadas y `omega_mid` de forma causal sobre ese intervalo.
- Criterio adicional necesario: TWO/THREE/MIDPOINT_DYNAMIC deben evaluarse
  sobre la interseccion exacta de muestras validas e informar cobertura, para
  evitar sesgo por descartar intervalos dificiles.
- Prueba acordada: 326 en hover GT/ORB shadow; si TWO y MIDPOINT_DYNAMIC quedan
  casi equivalentes, 327 con aceleracion suave. Solo si los datos lo justifican,
  integrar productivamente y ejecutar 328/329 antes de 330/331 ORB real.
- Criterios: cobertura FULL de torque/thrust y estado angular/gravedad validos;
  comparacion sobre interseccion comun; no empeorar significativamente hover,
  mejorar o igualar TWO en aceleracion y superar THREE.
- Riesgos aceptados: el puente local conserva la convencion temporal vigente y
  puede exponer sensibilidad al delay; la interpolacion angular visual puede
  aportar ruido, que quedara medido sin filtros nuevos.
- Dudas abiertas: ninguna.
- Siguiente accion exacta: localizar APIs y pruebas actuales, implementar solo
  MIDPOINT_DYNAMIC shadow y su telemetria/tests, validar y compilar `orbslam3`.

Checkpoint cambios MIDPOINT_DYNAMIC: añadida funcion pura que mapea
`t_mid_image` al reloj ROS desde la llegada de `t_k`, interpola SO(3), deriva
omega causal del intervalo y reutiliza los predictores torque/thrust. El nodo
solo publica `[F5H-MIDPOINT-DYNAMIC]`; THREE_SAMPLE productivo permanece
intacto. Añadidos ocho GTests, soporte comparativo de muestras comunes en el
analizador y YAML 326. Archivos criticos: `navigation-state-estimator.*`,
`stereo-slam-node.*`, su GTest, analizador lineal y `tray_prueba_326.yaml`.
Siguiente accion: validaciones mecanicas y build `orbslam3`; si pasa, ejecutar
GTests y despues compilar launches dependientes.

Checkpoint build MIDPOINT_DYNAMIC 1/3: `orbslam3` codigo 0 en 1 min 12 s;
solo warnings heredados de cv_bridge/ORB-SLAM3/Eigen. Log completo conservado
en `codex/archivos_auxiliares/colcon_build.log`. Python, YAML y
`git diff --check` pasan; el primer comando unittest descubrio 0 porque la suite
usa funciones pytest, por lo que se ejecutara mediante su harness directo.
Siguiente accion: ejecutar GTest y las siete funciones del analizador; si pasan,
compilar `dron_individual` y `simulacion_dron`.

Checkpoint suites MIDPOINT_DYNAMIC: GTest real 116/116, incluidos 8/8 nuevos;
analizador 7/7 mediante harness directo. El intento con `--gtest_brief=1`
mostro ayuda por incompatibilidad de esta version y no cuenta como ejecucion.
Siguiente accion: build `dron_individual` y despues `simulacion_dron`; si ambos
pasan, registrar y ejecutar 326.

Checkpoint build MIDPOINT_DYNAMIC 2/3: `dron_individual` codigo 0 en 1.52 s.
Siguiente accion: compilar `simulacion_dron`; despues preparar 326.

Checkpoint build MIDPOINT_DYNAMIC 3/3: `simulacion_dron` codigo 0 en 0.68 s.
Los tres builds pasan; GTest 116/116 y analizador 7/7. Siguiente accion larga:
prueba 326 con `tray_prueba_326.yaml`, launch `multi_dron.launch.py`, GT
gobernando, ORB dynamic shadow, headless/debug, timeout 180 s y metricas en
`metricas/prueba_326`; despues registrar el runner antes de reducir.

Checkpoint simulacion 326: runner 0, escenario 0, `success=true`; log completo
`codex/archivos_auxiliares/logs/prueba_326.log` conservado sin lectura manual.
Siguiente accion: reducir marcadores de escenario, autoridad, tracking,
gravedad, cobertura y MIDPOINT_DYNAMIC; despues ejecutar el analizador lineal
contra el CSV GT y decidir si corresponde 327.

Diagnostico 326: `CONSEGUIDA` como shadow. Sobre 930 muestras comunes,
RMSE TWO=`0.023052`, THREE=`0.039098` y MIDPOINT_DYNAMIC=`0.023144 m/s`;
MAE=`0.017886/0.030446/0.018125`, p95=`0.047056/0.083280/0.047549`.
MIDPOINT_DYNAMIC tiene cobertura comun 100 %, queda solo 0.4 % peor que TWO
y mejora 40.8 % frente a THREE. Al ser empate practico en hover, el acuerdo
activa 327. Añadida clasificacion diagnostica hover/moving y YAML 327 con
desplazamiento X suave de 2 m en 12 s, GT gobernando y ORB shadow.
Siguiente accion: validar analizador/YAML y ejecutar 327 con el mismo launch,
timeout y metricas separadas; no cambiar salida productiva.

Checkpoint simulacion 327: runner 0, escenario 0, `success=true`; log completo
`logs/prueba_327.log` conservado sin lectura manual. Siguiente accion: reducir
marcadores y analizar contra GT, separando hover/moving sobre muestras comunes;
despues decidir TWO, MIDPOINT_DYNAMIC o conservar THREE conforme al criterio.

Diagnostico 327: `CONSEGUIDA`. En 814 muestras comunes, RMSE global
TWO/THREE/MIDPOINT=`0.074311/0.112426/0.074276`; en las 296 muestras moving,
`0.097242/0.136433/0.097166 m/s`; en hover MIDPOINT y TWO empatan a
`0.057222/0.057221`. Cobertura 100 %. Decision conforme al acuerdo:
`MIDPOINT_DYNAMIC` gana por semantica y ligera mejora en movimiento, sin
degradacion significativa en hover; THREE queda descartado por amplificacion.
Checkpoint cambio productivo: `StereoSlamNode` usa ahora exclusivamente la
velocidad MIDPOINT_DYNAMIC valida para formar la base productiva; p/R/omega,
gates, buffers, dinamica y control intactos. Telemetria declara
`linear_source=MIDPOINT_DYNAMIC|UNAVAILABLE`.
Siguiente accion: validar diff, build `orbslam3`, GTests 116/116 y despues
builds dependientes antes de 328 shadow productiva.

Checkpoint build productivo 1/3: `orbslam3` codigo 0 en 33.7 s, solo warnings
heredados. Siguiente accion: GTest 116/116; despues builds dependientes.

Checkpoint tests productivos: GTest 116/116. Siguiente accion: build
`dron_individual` y luego `simulacion_dron` antes de 328.

Checkpoint build productivo 2/3: `dron_individual` codigo 0 en 1.54 s.
Siguiente accion: build `simulacion_dron`.

Checkpoint build productivo 3/3: `simulacion_dron` codigo 0 en 0.67 s. Los
tres builds y 116/116 GTests pasan. Creados YAML 328/329 identicos al hover
326, ya con MIDPOINT_DYNAMIC productivo. Siguiente accion larga: validar YAML
y ejecutar 328 con GT gobernando/ORB shadow, debug y metricas; STOP si falla.

Checkpoint simulacion 328: runner 0, escenario 0, `success=true`; log completo
`logs/prueba_328.log` conservado sin lectura. Siguiente accion: reducir y
analizar fuente productiva, cobertura, `v_hat_tk`, `v_dynamic_now` y gain;
solo si cumple ejecutar 329 identica.

Diagnostico 328: `CONSEGUIDA`. El analizador se corrigio mecanicamente para
leer la fuente productiva del marcador y no confundirla con el THREE shadow.
En 880 bases productivas: RMSE productivo MIDPOINT=`0.021125` y dynamic_now
`0.021746 m/s` (ratio agregado 1.029); cobertura 100 %, escenario completo.
THREE diagnostico sigue en `0.035757`, pero ya no alimenta la base. El primer
intento de silenciar salida en `/tmp` fallo por sandbox sin afectar artefactos;
repetido escribiendo solo el summary en workspace. Siguiente accion larga:
ejecutar 329 identica, sin cambios funcionales, y comprobar reproducibilidad.

Checkpoint simulacion 329: intento 0 con muerte temprana de Gazebo detectada;
el reintento automatico 1 ejecuto escenario completo, runner 0 y
`success=true`. Log completo `logs/prueba_329.log` conserva ambos intentos sin
lectura manual. Siguiente accion: reducir y analizar el intento util; si
reproduce 328, avanzar a 330 ORB real conforme al acuerdo.

Diagnostico 329: `CONSEGUIDA` y reproducible. 914 bases, cobertura 100 %;
RMSE productivo=`0.024599`, dynamic_now=`0.024831 m/s` (ratio agregado 1.009).
328/329 validan `LINEAR_VELOCITY_ESTIMATOR` productivo en shadow. El intento
0 de Gazebo no genero escenario y no altera la conclusion del intento 1.
Siguiente accion: crear 330/331 desde el handoff completo vigente (cerrar
trayectoria GT, activar shadow, confirmar autoridad y nuevo goal hover),
ejecutar 330 ORB real y STOP si diverge.

Checkpoint pre-330: YAML 330/331 y `git diff --check` validos. Prueba 330 usa
`f5h_orb_shadow_mode=true` solo como frontera controlada de handoff,
`orb_navigation_prediction_mode=dynamic`, override normal y GT fallback
vigente; tras confirmacion, p/v/R/omega ORB gobiernan el nuevo goal. Headless,
debug, metricas en `metricas/prueba_330`, timeout runner 900 s. Siguiente
accion larga: ejecutar 330 y registrar resultado antes de analizar.

Checkpoint simulacion 330: runner 0, escenario 0, `success=true`; log completo
`logs/prueba_330.log` conservado sin lectura. Siguiente accion: reducir handoff,
source, tracking/fallback, errores de control y estado final; ejecutar
analizadores existentes sobre artefactos y clasificar hover estable/divergente.
No ejecutar 331 hasta cerrar ese diagnostico.

Diagnostico 330: `CONSEGUIDA`, `HOVER ORB ESTABLE`. ORB gobierna 34.78 s con
1734 publicaciones/695 medidas, tracking permanece OK y no hay fallback.
Max `er=0.0674 rad`, omega control `0.0722 rad/s`; energia total post-handoff
negativa `-0.000441 J`, sin clamp. Errores angulares control-vs-GT RMSE
`0.0200 rad/s`, max `0.0632`. La telemetria de pose agregada no emparejo
NavigationState por su filtro, pero control ORB confirma errores acotados
(muestra reducida max posicion `0.0115 m`, velocidad `0.0545 m/s`) y no hay
divergencia. Siguiente accion larga: ejecutar 331 identica para reproducibilidad.

Checkpoint simulacion 331: runner 0, escenario 0, `success=true`; log completo
`logs/prueba_331.log` conservado sin lectura. Siguiente accion: reducir y
ejecutar analizador angular; comparar estabilidad, tracking, fallback y energia
con 330 antes del cierre documental.

Diagnostico 331: `CONSEGUIDA` y reproducible frente a 330. ORB gobierna
`35.30 s`, con 1759 publicaciones/608 medidas, tracking OK y cero fallback o
clamp. Max `er=0.0631 rad`, max omega control `0.0774 rad/s`, RMSE omega contra
GT `0.0171 rad/s` y energia angular total `-0.000348 J`. Conclusion del bloque:
`A_HAT_AMPLIFICATION CORREGIDA`, `LINEAR_VELOCITY_ESTIMATOR VALIDADO` y
`HOVER ORB REAL VALIDADO`. 5H permanece `PARCIAL` hasta probar movimiento y
trayectoria representativa.

Checkpoint documental: contratos, docs de paquete, resumen de fase, estado,
pipeline maestro, historial 5H y ultima sesion actualizados con 326-331.
Trabajo activo: ninguno. Siguiente accion futura: preparar y acordar pruebas
ORB de X/Y/Z/yaw y trayectoria representativa; no hay simulaciones activas.

## Plan activo 332-343

- Preparacion: `CERRADA`.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `CONCEDIDA`.
- Objetivo: validar con arquitectura congelada movimiento ORB real X/Y/Z/yaw,
  combinacion y finalmente la trayectoria representativa.
- Maniobras: X/Y `2 m` en `12 s`; Z `0.5 m` conservador; yaw `90 deg` lento;
  combinacion horizontal `2 m` + yaw `90 deg`.
- 332/333 X, 334/335 Y, 336/337 Z, 338/339 yaw, 340/341 combinacion. Cada
  primera ejecucion debe pasar y repetirse; STOP ante fallo o no
  reproducibilidad. En 332-341 se exige fallback cero.
- 342/343: `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` con dos drones,
  solo si 332-341 pasan. GT normal solo como fallback por perdida visual real.
- Congelado: estimadores, predictores, gravedad, J/masa, buffers, gains, gates,
  KF/reference, W y mux productivo. Solo se permiten YAML, telemetria/analisis
  necesarios y correcciones puramente de infraestructura.
- Riesgos aceptados: bateria larga; STOP progresivo; una perdida visual en
  342/343 puede activar GT_FALLBACK si es causa y no consecuencia del control.
- Dudas abiertas: ninguna.
- Siguiente accion exacta: localizar la estructura vigente de 330/331 y crear
  YAMLs 332-341 sin modificar el estimador; despues validar y compilar antes de
  ejecutar 332.

Checkpoint escenarios 332-341: creados diez YAML desde la frontera exacta de
330/331. X/Y desplazan 2 m en 12 s; Z sube 0.5 m en 10 s; yaw gira 90 deg en
10 s; combinada une X 2 m y yaw 90 deg. Todos conservan 12 s de hover final.
No se modifica codigo, estimador ni configuracion productiva. Siguiente accion:
validar YAML/diff, compilar los tres paquetes y ejecutar GTests/analizador antes
de iniciar 332.

Checkpoint build 332-343 1/3: primer comando invalido por grupo descriptivo y
sin compilacion; corregido mecanicamente a `--group dron`. `orbslam3` compila
con codigo 0 en 1.08 s, solo warnings heredados. Siguiente accion: compilar
`dron_individual`.

Checkpoint build 332-343 2/3: `dron_individual` codigo 0 en 2.14 s.
Siguiente accion: compilar `simulacion_dron`.

Checkpoint build/tests 332-343: `simulacion_dron` codigo 0 en 0.99 s; los tres
paquetes compilan. GTest `116/116` y harness del analizador codigo 0 (7 casos)
pasan; YAML 332-341 y `git diff --check` correctos. Arquitectura congelada.
Siguiente accion larga: ejecutar 332 con ORB real, debug/metricas y STOP si
falla funcionalmente.

Checkpoint simulacion 332: runner 0, escenario 0, `success=true`; recursos
sanos y guard inactivo. Log completo `logs/prueba_332.log` conservado sin
lectura. Siguiente accion: reducir escenario, autoridad, tracking/fallback,
missing, control y MIDPOINT; analizar movimiento y frenado antes de decidir 333.

Diagnostico 332: `CONSEGUIDA`, X funcionalmente estable. ORB gobierna 29.56 s,
tracking permanece 2, sin fallback ni dynamic missing. Control: RMSE/max ep
`0.0360/0.0676 m`, RMSE/max ev `0.1253/0.5344 m/s`; al cierre ep/ev
`0.0439 m / 0.0283 m/s`. Max er/ew `0.2549 rad / 0.1588 rad/s`; energia
angular total `-0.001766 J`, sin clamp. MIDPOINT productivo tiene RMSE global
`0.1014 m/s`, cobertura 100 %. Hay picos de ev durante el hover final, pero no
divergencia y la velocidad final vuelve cerca de cero. Siguiente accion larga:
ejecutar 333 identica para reproducibilidad, sin cambios.

Checkpoint simulacion 333: runner 0, escenario 0, `success=true`; recursos
sanos y guard inactivo. Log completo `logs/prueba_333.log` conservado sin
lectura. Siguiente accion: reducir y medir con el mismo criterio que 332 antes
de declarar X reproducible o aplicar STOP.

Diagnostico 333: `CONSEGUIDA`; X reproducible. ORB 30.28 s, tracking 2, sin
fallback/missing/clamp. RMSE/max ep `0.0477/0.1125 m`, RMSE/max ev
`0.1166/0.7250 m/s`; cierre ep/ev `0.0507 m / 0.0294 m/s`. MIDPOINT RMSE
`0.1002 m/s`, cobertura 100 %; energia total `-0.002225 J`.
Conclusion 332+333: `MOVIMIENTO X ORB VALIDADO`. Siguiente accion larga:
ejecutar 334 Y, con STOP si falla.

Checkpoint simulacion 334: runner 0, escenario 0, `success=true`; recursos
sanos. Log completo conservado sin lectura. Siguiente accion: reducir y medir Y
antes de decidir 335.

Diagnostico 334 revisado tras observacion del usuario: `INVALIDA POR COLISION`.
ORB gobierna solo `6.62 s`; tracking pasa `2->3` a +6.64 s, despues `3->0->1`,
y el mux entra en `gt_fallback reason=tracking_lost`. No hay dynamic missing ni
clamp. Antes de perder tracking el control permanece acotado: RMSE/max ep
`0.0295/0.0648 m`, RMSE/max ev `0.0567/0.1660 m/s`; ultima muestra ORB
ep/ev `0.0212 m / 0.0989 m/s`. La cronologia no muestra divergencia previa:
durante los 6.54 s ep queda <=6.5 cm y ev <=0.166 m/s; el pico angular final
coincide inmediatamente con la perdida. El escenario completa despues mediante
GT fallback, pero no valida Y. La trayectoria parte de `[0,-10,1]` y avanza a
`[0,-8,1]`, atravesando el fiducial 2 situado en `[0,-8.5,1]`; la perdida es
consecuencia de la colision y no evidencia contra ORB ni contra el control Y.
335-343 NO ejecutadas. Siguiente accion futura: acordar 334R hacia `-Y`, por
ejemplo `[0,-12,1]`, conservando arquitectura y criterios.

Checkpoint documental 332-334: historial largo/resumen 5H, contrato, estado,
contexto minimo, pipeline de Fase 5, pipeline maestro, indice y ultima sesion
sincronizados. Resultado agregado revisado: `PARCIAL`; X validado/reproducido,
334 invalida por colision y Y sigue sin evaluar, Z/yaw/combinacion/trayectoria no
ejecutadas. Autorizacion funcional: `SUSPENDIDA` por STOP consumido. Trabajo
activo: ninguno; no hay simulaciones activas. Dudas abiertas: confirmar 334R
hacia `[0,-12,1]` para validar Y sin obstaculo y sin retocar el estimador.

Reanudacion bateria 334R-343: usuario confirma 334R hacia `[0,-12,1]` y
autoriza continuar las demas pruebas bajo el acuerdo original. Preparacion
`CERRADA`, acuerdo `si`, autorizacion funcional `CONCEDIDA`, dudas `ninguna`.
Corregir mecanicamente 335 para que repita el mismo Y hacia `-Y`; arquitectura,
criterios y STOP progresivo permanecen congelados. Siguiente accion exacta:
crear 334R, corregir 335, validar YAML/diff y ejecutar 334R.

Checkpoint simulacion 334R: runner 0, escenario 0, `success=true`; recursos
sanos. Log completo conservado sin lectura. Siguiente accion: reducir y medir
Y corregida antes de decidir 335.

Diagnostico 334R: `NO CONSEGUIDA` funcionalmente y STOP aplicado. El escenario
sin obstaculo confirma ORB durante `30.26 s`, tracking 2, cero fallback,
missing o clamp; RMSE/max ep `0.0327/0.0628 m` y energia angular
`-0.002717 J`. Sin embargo el frenado no converge: RMSE/max ev global
`0.1630/0.4118 m/s`, ev final `0.1873 m/s` y en los ultimos 3 s RMSE/max ev
`0.1739/0.3342 m/s`. GT fisico en esa ventana ya esta mucho mas cerca de cero:
RMSE/max `0.0491/0.0745 m/s`, final `0.0561 m/s`. MIDPOINT tiene RMSE global
`0.1442 m/s`, cobertura 100 %. Conclusion: Y mantiene posicion y tracking,
pero la velocidad ORB residual/oscilante incumple `v_ORB -> 0`; 335-343 no se
ejecutan. Autorizacion suspendida por STOP. Siguiente accion: documentar el
fallo de frenado Y sin modificar el estimador.

Checkpoint documental 334R: historial/resumen 5H, contrato, estado, contexto
minimo, pipelines, indice y ultima sesion sincronizados. Resultado agregado
`PARCIAL`: X validado; 334 invalida por colision; 334R valida tracking y
posicion Y pero no el frenado/velocidad; 335-343 no ejecutadas. Autorizacion
funcional `SUSPENDIDA` por STOP. Trabajo activo: ninguno; no hay simulaciones
activas. Duda abierta: diagnosticar la velocidad ORB residual en Y antes de
autorizar cualquier modificacion.

Repeticion visual 334R2: usuario autoriza repetir exactamente 334R con Gazebo
GUI visible. Sin cambios de YAML, codigo, estimador ni criterios; usar el mismo
`tray_prueba_334R.yaml`, log/metricas separados bajo 334R2 y no continuar a 335.
Siguiente accion larga: ejecutar 334R2 con `launch_gazebo_gui:=true` para
observacion humana.

Checkpoint simulacion 334R2 visual: runner 0, escenario 0, `success=true`;
Gazebo GUI visible durante toda la prueba, recursos sanos y guard inactivo.
Log completo conservado sin lectura. Siguiente accion: reducir y medir para
registrar si reproduce 334R, sin cambios funcionales.

Diagnostico 334R2 visual: `NO CONSEGUIDA` y reproduce el fallo con mayor
severidad. ORB `33.30 s`, tracking 2, cero fallback/missing y cobertura 100 %;
pero max ep/ev `0.4929 m / 2.1876 m/s`, cierre ep/ev
`0.2128 m / 0.5506 m/s`, RMSE/max ev en ultimos 3 s
`1.0076/2.1876 m/s` y RMSE productivo `0.4398 m/s`. Confirma que la velocidad
ORB lateral/frenado es inestable pese a tracking sano. No continuar a 335.
Trabajo activo: ninguno; no hay simulaciones activas.

Prueba diagnostica 334R3 autorizada: desde `[0,-10,1]`, ORB ejecuta primero X
hasta `[2,-10,1]`, hover 12 s, despues +Y hasta `[2,-8,1]` manteniendo 2 m de
separacion lateral respecto al fiducial 2, y hover final 12 s. Si pasa, repetir
identica como 335R y continuar Z/yaw/combinacion bajo STOP progresivo. Se
congelan estimador, control y criterios. Dudas abiertas: ninguna. Siguiente
accion: crear/validar 334R3 y 335R y ejecutar 334R3.

Checkpoint simulacion 334R3: runner/escenario codigo 1 tras 116 s; recursos
sanos, guard inactivo, sin muerte de Gazebo. Log completo conservado sin
lectura. Siguiente accion: reducir pasos, goals, autoridad, tracking/fallback y
control para determinar si es fallo funcional o infraestructura; no ejecutar
335R ni pruebas posteriores antes del diagnostico.

Diagnostico 334R3: `INVALIDA DE INFRAESTRUCTURA`. El runner queda en el paso 1
y agota 90 s esperando `/fiducial_spawn_ready`; no envia goals ni activa ORB.
No existe evidencia funcional. Correccion mecanica permitida: repetir el mismo
YAML sin cambios como 334R3R. Siguiente accion larga: ejecutar 334R3R; mantener
335R y posteriores detenidas hasta su diagnostico.

Checkpoint simulacion 334R3R: runner 0, escenario 0, `success=true`; recursos
sanos. Log completo conservado sin lectura. Siguiente accion: reducir y medir
por separado X, hover tras X, Y y hover final; decidir 335R solo despues.

Diagnostico 334R3R: `CONSEGUIDA PROVISIONAL` y habilita repeticion. ORB gobierna
54.30 s, tracking 2, cero fallback/missing/clamp, cobertura 100 %, max er
`0.2068 rad` y energia `-0.007397 J`. X move/hover: max ep
`0.0695/0.0444 m`; Y move/hover: max ep `0.0713/0.0589 m`. Cierre ep/ev
`0.0312 m / 0.1075 m/s`; GT fisico termina en `0.0146 m/s`. Persiste residual
ORB (RMSE/max ev ultimos 3 s `0.1433/0.2975 m/s`), pero no produce divergencia
y mejora fuertemente 334R2. Siguiente accion larga: ejecutar 335R identica para
decidir reproducibilidad; STOP si vuelve la inestabilidad.

Checkpoint simulacion 335R: runner 0, escenario 0, `success=true`; recursos
sanos. Log completo conservado sin lectura. Siguiente accion: reducir y medir
por fases con el mismo criterio de 334R3R antes de avanzar a Z.

Diagnostico 335R: `CONSEGUIDA` y reproduce 334R3R. ORB `54.34 s`, tracking 2,
cero fallback/missing/clamp, cobertura 100 %, max er `0.4933 rad` y energia
`-0.007341 J`. Y move/hover: max ep `0.0881/0.0921 m`; cierre ep/ev
`0.0639 m / 0.1111 m/s`; GT final `0.0545 m/s`. El residual ORB persiste,
pero no causa divergencia y el resultado contrasta fuertemente con 334R/334R2
lejos de la pared. Conclusion: movimiento Y funcional validado bajo geometria
con buena cobertura visual; hipotesis de calidad ORB apoyada, limitacion de
residual documentada. Siguiente accion larga: ejecutar 336 Z bajo STOP.

Checkpoint simulacion 336: runner 0, escenario 0, `success=true`; recursos
sanos y guard inactivo. Log completo `logs/prueba_336.log` conservado sin
lectura manual. Siguiente accion: reducir y medir movimiento/frenado Z antes
de decidir 337 bajo el STOP progresivo.

Diagnostico 336: `CONSEGUIDA`. ORB gobierna `30.24 s`, tracking 2 y cero
fallback/missing/clamp. En Z move: RMSE/max ep `0.0329/0.0510 m` y RMSE/max
ev `0.0521/0.1133 m/s`; en hover final: RMSE/max ep `0.0189/0.0337 m` y
RMSE/max ev `0.0454/0.1302 m/s`. Cierre ep/ev `0.0157 m / 0.0147 m/s`;
energia angular total `-0.000400 J`, max er `0.0782 rad` y MIDPOINT RMSE
`0.0401 m/s`. Movimiento Z estable; siguiente accion larga: ejecutar 337
identica para reproducibilidad.

Incidencia mecanica 337: el primer lanzamiento no inicio porque faltaba el
YAML previsto. Se crea `tray_prueba_337.yaml` identico funcionalmente a 336,
cambiando solo nombre/descripcion; no cuenta como ejecucion funcional ni
altera arquitectura o criterios. Siguiente accion: validar YAML y relanzar 337.

Checkpoint intento 337: runner/escenario codigo 1 tras solo 25 s, recursos
sanos y guard inactivo. No clasificar aun: reducir spawn, pasos y goals para
determinar si la maniobra llego a empezar. Pruebas posteriores detenidas.

Diagnostico intento 337: `INVALIDA DE INFRAESTRUCTURA`; fiduciales listos,
pero el runner no carga el YAML porque la ruta relativa se resuelve desde el
workspace padre. No hubo goals ni maniobra. Correccion mecanica: repetir 337
con ruta absoluta al mismo archivo, sin cambios funcionales.

Checkpoint simulacion 337R: runner 0, escenario 0, `success=true`; recursos
sanos y guard inactivo. Log completo `logs/prueba_337R.log` conservado sin
lectura manual. Siguiente accion: reducir y medir con el criterio de 336 antes
de declarar Z reproducible o aplicar STOP.

Diagnostico 337R: `CONSEGUIDA`; reproduce 336. ORB `29.71 s`, tracking 2,
cero fallback/missing/clamp. Z move: RMSE/max ep `0.0236/0.0464 m` y RMSE/max
ev `0.0490/0.1233 m/s`; hover: RMSE/max ep `0.0203/0.0337 m`, cierre ep/ev
`0.0228 m / 0.0176 m/s`. MIDPOINT RMSE `0.0400 m/s`, energia angular
`-0.000477 J`. Conclusion 336+337R: `MOVIMIENTO Z ORB VALIDADO`. Siguiente
accion: ejecutar 338 yaw bajo STOP.

Checkpoint simulacion 338: runner 0, escenario 0, `success=true`; recursos
sanos y guard inactivo. Log completo `logs/prueba_338.log` conservado sin
lectura manual. Siguiente accion: reducir y medir giro/frenado yaw antes de
decidir 339 bajo STOP.

Diagnostico 338: `NO CONSEGUIDA` y STOP aplicado. ORB gobierna `11.18 s`;
tracking pasa `2->3` durante el giro y el mux activa
`gt_fallback reason=tracking_lost`. Max er `0.995 rad`, max ew
`0.709 rad/s`, RMSE omega control-GT `0.409 rad/s` y RMSE lineal productivo
`0.530 m/s`. Aunque la energia angular neta es negativa, la degradacion y la
perdida invalidan yaw. 339-343 no ejecutadas.

Cierre documental 334R3-338: historial 5H, contrato, resumen de fase, estado,
pipeline maestro y ultima sesion sincronizados. Estado agregado `PARCIAL`:
X, Y con buena cobertura y Z validados; yaw no validado. Preparacion:
`NO_INICIADA`; acuerdo cerrado: `no`; autorizacion funcional: `SUSPENDIDA` por
STOP; prueba futura pendiente de debate; dudas abiertas: diagnosticar el fallo
de yaw. Trabajo activo: ninguno y no hay simulaciones activas.

## Prueba larga ORB post-338

- Preparacion: `CERRADA`.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `CONCEDIDA`.
- Objetivo: ejecutar con Gazebo visible la trayectoria completa
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` para dos drones.
- Comportamiento: aproximacion inicial GT; tras anclaje, ORB gobierna; una
  perdida visual real habilita GT_FALLBACK para continuar y un anclaje futuro
  permite que la siguiente trayectoria vuelva a ORB.
- Alcance congelado: estimadores, predictores, controladores, gains, gates,
  mux, ORB-SLAM3 core y waypoints/timings originales.
- Instrumentacion: autoridad, tracking, fallback, control y resultados de
  goals. Gazebo GUI visible; RViz no requerido.
- Criterios: exito ORB si completa estable sin fallback; exito funcional
  parcial si completa con perdidas recuperadas por GT; fallo si hay colision,
  inestabilidad, goals rechazados o vuelta incompleta.
- Riesgo aceptado: se levanta expresamente el STOP posterior a 338 para esta
  prueba representativa aun con yaw no validado.
- Prueba acordada: nueva ejecucion larga con artefactos separados.
- Dudas abiertas: ninguna.
- Siguiente accion exacta: preparar una variante declarativa con fronteras de
  autoridad ORB, validar YAML y ejecutar con Gazebo GUI.

Checkpoint escenario largo ORB: el YAML representativo conserva todos sus
waypoints/timings y añade, tras cada espera de anclaje, apertura de frontera,
activacion y confirmacion ORB para ambos drones. No se modifica codigo ni
control. Siguiente accion: validar YAML/diff y ejecutar con Gazebo visible.

Checkpoint simulacion 342: Gazebo GUI visible; runner/escenario codigo 1 tras
281 s. Recursos sanos y guard inactivo; el fallo no es de memoria. Log completo
`logs/prueba_342.log` conservado sin lectura manual. Siguiente accion: reducir
goals, pasos, autoridad, tracking/fallback y control para localizar el tramo y
clasificar la prueba.

Diagnostico 342: `INVALIDA PARA VUELTA COMPLETA POR GATE DEL ESCENARIO`, no por
caida. Ambos drones completan los tramos sur/laterales/norte y llegan al
fiducial 1. Dron 2 pierde tracking, crea epoch 1 y queda sin anclar; el paso 21
exige confirmacion ORB y agota 20 s. El acuerdo requiere continuar con GT si no
esta anclado. Correccion mecanica: tras fiducial 1 solicitar reactivacion sin
espera obligatoria; cada siguiente goal elegira ORB si esta cualificado o GT
fallback en caso contrario. Repetir como 342R con Gazebo visible.

## Auditoria reference_kf post-342R

- Preparacion: `CERRADA`.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `CONCEDIDA`.
- Objetivo: auditar si historicos raw dependientes de `reference_kf` mezclan
  muestras incompatibles y afectan productivamente al estimador.
- Secuencia: checkpoint git y push; tabla de historicos/consumidores;
  telemetria minima si falta evidencia; 344 dos fachadas con GT gobernando y
  ORB dynamic shadow; solo si confirma el bug, correccion quirurgica; 345
  shadow; 346 ORB; 347 repeticion.
- Congelado: gains, J, masa, `g_O`, MIDPOINT_DYNAMIC, thresholds, buffers,
  goal semantics, mux normal y ORB-SLAM3 core.
- Invariantes: preservar estado continuo en O y fisica; reset/rebase solo de
  historicos realmente dependientes de Kref; no fake delta cero.
- Prueba: rama del dron 2 durante unas dos fachadas, geometria/timing reales;
  dron 1 en hover si la infraestructura multi-dron lo exige.
- STOP: si 344 no confirma, si 345 conserva anomalías cross-KF o si 346
  diverge antes de perdida visual.
- Riesgos aceptados: la hipotesis puede descartarse; `raw_dt` antiguo puede
  proceder de rechazos acumulados y no de mezcla geometrica.
- Dudas abiertas: ninguna; la posibilidad de un solo dron se resuelve por
  auditoria de infraestructura sin cambiar el comportamiento acordado.
- Siguiente accion exacta: revisar estado git, excluir artefactos, crear commit
  del avance actual y subirlo; despues iniciar auditoria sin modificar aun.

## Puerta de continuidad

- Preparacion: `CERRADA` para 314-317.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `SUSPENDIDA`; bateria 314-317 terminada.
- Prueba acordada: 276, pose GT perfecta a 20 Hz y `omega_motion` calculada
  exclusivamente por el nuevo estimador; estimar en `t_k` y mantener la omega
  entre medidas, sin proyectarla aun hasta `now` mediante `alpha_hat`.
- Pruebas acordadas: 273=E predictor actual con omega GT; 274=F hold angular;
  275=G extrapolacion angular directa. Ejecutar las tres siempre y en orden.
- Pruebas acordadas: 269=A GT normal 50 Hz; 270=B GT 20 Hz sin delay;
  271=C GT 20 Hz +80 ms; 272=D GT con traza temporal determinista de 268.
- Prueba ejecutada: 268 con `f5h_etapa_2_hover_orb.yaml`.
- Prueba 269: no ejecutada porque 268 no completo el hover.
- Etapa 3: no ejecutada.
- Politica: ejecutar A-D salvo que A falle; D usa la traza de 268.
- Prueba acordada: bateria condicional 278 -> 279 -> 280 -> 281, con parada en
  el primer fallo y el mismo escenario de hover de 276-277. 278 usa GT 20 Hz
  con delay fijo de 80 ms; 279 timing/jitter medido; 280 ORB real; 281 repite
  280 solo si esta pasa.
- Criterio: hover completo, sin fallback ni tracking loss; conservar timestamps
  fisicos, usar `omega_GT` solo como truth y congelar el estimador causal.
- Propuesta pendiente: 278B eleva solo el horizonte diagnostico a `0.18 s`; si
  falla, 278C propaga pose y omega coherentes hasta `now` con `alpha_hat`,
  limites fisicos y GTests; mantener 279-281 detenidas.
- Prueba acordada: 282=278B con horizonte diagnostico `0.18 s`; repetir si
  completa. Si falla, 284=278C con propagacion pose/omega mediante `alpha_hat`,
  limites y GTests; repetir si completa. Mantener 279-281 detenidas.
- Pruebas acordadas: 285 usa R predicha + omega GT actual; 286 usa R GT
  actual + omega predicha; 287 usa ambas GT como sanity check. Mismo delay de
  80 ms, entrada visual 20 Hz, salida/control 50 Hz, predictor 284 y hover.
  Ejecutar siempre las tres aunque 285 o 286 fallen; solo un fallo de 287
  invalida la interpretacion. Mantener 279-281 detenidas.
- Propuesta pendiente: 288=`p/v GT + R/omega pred`; 289=`p/v/omega GT + R pred`;
  290=`p/v/R GT + omega pred`; 291=estado GT actual completo. Mismo hover,
  delay y predictor; 279-281 detenidas.
- Pruebas acordadas: ejecutar siempre 288-291 en orden. Usar la conversion de
  marco de la ruta GT estable; ultima muestra recibida antes del tick, sin
  interpolar/extrapolar, registrando edad y skew. Si 291 falla, no interpretar
  288-290. No implementar solucion y mantener 279-281 detenidas.
- Dudas abiertas: ninguna.

## Plan activo 303-306

- 303: p/v GT actuales con R/omega dinamicas actuales; 304: p/v GT actuales y
  estado angular GT interpolado retrospectivamente en `t_k`, propagado solo
  con torque/J hasta now; 305: p/v predichas con R/omega GT actuales; 306:
  estado GT actual completo.
- Las cuatro usan `TracePeriods/TraceDelays` de 299 y el mismo hover. GT es
  exclusivamente diagnostico; 304 usa buffer local acotado e interpolacion sin
  esperar nuevas muestras. Si no existe bracket temporal, la muestra es
  invalida y no se sustituye por GT incorrecto.
- Ejecutar las cuatro en orden cuando se autoricen las pruebas; no recalibrar
  entre ellas. 306 es sanity obligatorio. Mantener 300-302 detenidas.
- Congelado: J, predictor/integrador dinamico, buffer de torque, gains, gates,
  KF/reference, W, mux, trayectoria y arquitectura productiva.
- Riesgo aceptado: 303 frente a 304 cambia el estado angular inicial completo;
  medir residual R de 303 contra GT(t_k) para atribuir especificamente a omega.
- Dudas abiertas: ninguna.
- Siguiente accion exacta: ampliar el laboratorio diagnostico con modos
  `dynamic_303` a `dynamic_306`, buffer/interpolacion GT focal y GTests/YAMLs.

Checkpoint cambios 303-306: añadidos cuatro modos con traza 299, buffer GT
acotado e interpolacion retrospectiva no bloqueante para 304; si falta bracket
se emite `F5H-GT-TK-INVALID`. Telemetria distingue fuentes y registra bracket,
omega GT(t_k) y residual R en t_k. Añadidos GTest dinamico y YAMLs 303-306; J,
predictor, control y ruta productiva intactos. Siguiente accion: validar YAML,
`git diff --check` y compilar `orbslam3`.

Checkpoint build 303-306: YAMLs validos, `git diff --check` correcto y build
`orbslam3` codigo 0 en 17.5 s. Siguiente accion: GTest directo del estimador y
suite del analizador; no ejecutar simulaciones en esta peticion.

Cierre modificaciones 303-306: GTest directo 77/77; el nuevo caso confirma
propagacion desde GT(t_k) sin sustituir por GT(now). Suite directa del
analizador codigo 0; la invocacion previa como modulo descubrio cero tests y no
se cuenta. Documentacion de paquete y contrato sincronizados, sin cambios en J,
predictor, control ni ruta productiva. Pruebas 303-306 no ejecutadas todavia;
300-302 siguen detenidas. Autorizacion de modificaciones consumida.

Checkpoint pre-simulacion 303-306: el usuario autoriza ejecutar las cuatro en
orden, sin cambios intermedios. Siguiente accion larga: prueba 303 con YAML
absoluto, modo `dynamic_303`, timeout 180 s, espera 5 s y metricas propias.

Checkpoint simulacion 303: runner/escenario codigo 0, `success=true`, 92 s;
guard inactivo y minimo 4679.6 MiB. Log completo `logs/prueba_303.log`
conservado sin lectura. Siguiente accion larga: prueba 304 sin cambios.

Checkpoint simulacion 304: runner/escenario codigo 1 tras 63 s; guard inactivo
y minimo 5050.1 MiB. Log completo `logs/prueba_304.log` conservado sin lectura.
Conforme al acuerdo no interpretar ni modificar aun; siguiente prueba 305.

Checkpoint simulacion 305: runner/escenario codigo 1 tras 63 s; guard inactivo
y minimo 5002.0 MiB. Log completo `logs/prueba_305.log` conservado sin lectura.
Siguiente accion larga: prueba 306 sanity, sin cambios.

Checkpoint simulacion 306: runner/escenario codigo 0, `success=true`, 92 s;
guard inactivo y minimo 5141.6 MiB. Log completo `logs/prueba_306.log`
conservado sin lectura. Sanity funcional pasa. Siguiente: reducir 303-306 y
analizar fuentes, GT(t_k), torque, errores y energia antes de concluir.

Cierre 303-306: 303/306 `CONSEGUIDAS`, 304/305 `NO CONSEGUIDAS`. 303 completa
`54.54 s`, RMSE omega `0.00304 rad/s`, energia `-0.0000402 J`; 304 falla en
`2.52 s`, RMSE `3.13892`, pese a bracket GT(t_k) valido y torque cubierto; 305
falla en `15.06 s`, RMSE `1.86968`; 306 sanity completa `54.60 s`, RMSE
`0.00405`. Diagnostico agregado `CONSEGUIDO`: `PV PRINCIPAL`. Angular dinamica
es suficiente con p/v correctas; p/v predichas fallan aun con angular GT(now).
304 deja una limitacion secundaria de GT(t_k)+dinamica pendiente. 300-302 no
ejecutadas. Documentacion sincronizada y sin procesos activos.

## Plan activo 307-312

- Preparacion: `CERRADA`.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `CONCEDIDA` por el usuario.
- Objetivo: aislar posicion y velocidad con 307/308 y despues validar un
  predictor dinamico translacional causal bajo el timing/jitter de 299.
- 307: `p_GT(now) + v_pred(now) + R/omega_GT(now)`; 308:
  `p_pred(now) + v_GT(now) + R/omega_GT(now)`. Ejecutar ambas siempre.
- Tras diagnosticarlas, auditar thrust efectivo, frames, timestamps,
  saturaciones y masa real. Publicar thrust sellado en paralelo sin sustituir
  `control/tray/fuerza`; controlador y ruta GT quedan intactos.
- Predictor acordado: integrar thrust body, masa compartida y gravedad con dt
  reales; transformar cada intervalo con `R_dynamic(t)` del predictor angular.
- 309 usa `p/v_GT(t_k)` solo como estado inicial y angular GT(now) solo en la
  salida de control para aislar traslacion. Si pasa, repetir sin sobrescribir
  artefactos; despues 310 usa `p(t_k)` medida y `v_hat(t_k)` causal; 311 usa
  estado completo dinamico y 312 repite 311.
- Politica: STOP en el primer fallo funcional desde 309; 300-302 siguen
  detenidas. GT es exclusivamente laboratorio/truth. Congelados gains, J,
  predictor angular, gates, KF/reference, W, mux y ORB-SLAM3 core.
- Prueba acordada: bateria 307-312 con repeticion de 309 si funciona, mismo
  hover y traza determinista de 299; builds, GTests, reduccion y metricas p/v.
- Riesgos aceptados: posible fallo por frame/signo/timestamp de thrust o por
  estimacion causal de `v(t_k)`; no se compensara alterando ganancias.
- Dudas abiertas: ninguna.
- Siguiente accion exacta: localizar desde docs los selectores diagnosticos,
  publicador/mixer/plugin de fuerza, masa compartida y tests; implementar
  primero modos/YAMLs 307-308 y validarlos antes del predictor translacional.

Checkpoint cambios 307-308: `gt_timing_diagnostic` incorpora ambos modos con
la traza 299 y flags independientes de posicion/velocidad; 307 usa solo
posicion GT y 308 solo velocidad GT, mientras ambos usan R/omega GT actuales.
Anadidos dos GTests del selector y YAMLs 307/308. Predictor dinamico, control,
fuerza, masa y ruta productiva siguen intactos. Siguiente accion: validar YAML,
formato/diff y compilar `orbslam3`; despues ejecutar GTest y analizador.

Checkpoint build 307-308: `orbslam3` codigo 0 en 18.4 s; log completo en
`codex/archivos_auxiliares/colcon_build.log`, no leido. Siguiente accion:
ejecutar el binario GTest del estimador y la suite directa del analizador antes
de preparar la simulacion 307.

Checkpoint tests 307-308: GTest directo 79/79 y suite directa del analizador
codigo 0. Siguiente accion larga: prueba 307 con YAML absoluto, modo
`dynamic_307`, traza 299, timeout 180 s y espera posterior de 5 s; despues
registrar el resultado antes de reducir o ejecutar 308.

Checkpoint simulacion 307: runner/escenario codigo 1 tras 63 s; guard inactivo
y minimo 5063.4 MiB. Log completo `logs/prueba_307.log` conservado sin lectura.
Conforme al acuerdo no se interpreta ni recalibra aun. Siguiente accion larga:
prueba 308 con YAML absoluto, modo `dynamic_308`, mismo launch, timeout 180 s y
espera posterior 5 s.

Checkpoint simulacion 308: runner/escenario codigo 1 tras 63 s; guard inactivo
y minimo 4571.2 MiB. Log completo `logs/prueba_308.log` conservado sin lectura.
Ambas cruzadas han terminado; siguiente accion: reducir 307/308 con marcadores
de escenario, fuentes, p/v, control, tracking y fallback, ejecutar el analizador
y clasificar `V PRINCIPAL`, `P PRINCIPAL`, `P Y V` o incoherencia conjunta.

Diagnostico 307-308: ambas fallan durante la espera inicial, antes del primer
goal, sin fallback ni presion de recursos. Las fuentes registradas son las
acordadas. Clasificacion `P Y V`: posicion predicha sola y velocidad predicha
sola bastan para desestabilizar con los otros tres canales en GT. El analizador
angular estandar marca estas ramas diagnosticas como datos insuficientes; se
conserva la evidencia reducida y no se fuerza una metrica falsa.

Checkpoint predictor translacional: añadido topic paralelo sellado
`control/tray/thrust` sin cambiar `control/tray/fuerza`; buffer acotado e
integrador p/v con masa 1.4 kg, gravedad world, dt reales y `R_dynamic(t)` en
el punto medio. Mixer lineal sin saturacion y plugin `AddRelativeForce`
confirman `[0,0,T]` body. Anadidos siete GTests y YAMLs 309-313; 313 preserva
la repeticion de 309 sin sobrescribirla. Documentacion de `dron_individual` y
`orbslam3_ros2` sincronizada. Siguiente: compilar primero `orbslam3` y despues
`dron_individual`; corregir solo fallos mecanicos, ejecutar GTests y preparar
309 si todo pasa.

Checkpoint build predictor 1/2: `orbslam3` codigo 0 en 1min08s; solo warnings
heredados de cv_bridge/ORB-SLAM3. Log completo no leido. Siguiente accion:
compilar `dron_individual` por el nuevo publisher sellado, y registrar antes de
ejecutar tests.

Checkpoint build predictor 2/2: `dron_individual` codigo 0 en 14.5 s. El topic
legacy y el nuevo publisher compilan juntos. Siguiente accion: GTest directo
del estimador (esperados 86 tests), suite del analizador y validacion final de
YAML/diff antes de preparar 309.

Checkpoint tests predictor translacional: GTest directo 86/86, incluidos siete
casos de hover, gravedad, exceso, inclinacion/frame, dt irregular/cambio de
fuerza, coherencia p/v y masa compartida. Analizador codigo 0, YAMLs validos y
`git diff --check` correcto. Siguiente accion larga: prueba 309 con
`dynamic_309`, p/v GT(t_k), traslacion dinamica, angular GT(now), timeout 180 s
y espera 5 s. Si falla se aplica STOP; si pasa se reduce y repite como 313.

Checkpoint simulacion 309: runner/escenario codigo 0, `success=true`, 91 s;
guard inactivo y minimo 4803.7 MiB. Log completo `logs/prueba_309.log`
conservado sin lectura. Siguiente: reducir escenario, translacion, angular,
control, tracking y fallback; medir errores p/v y cobertura de fuerza antes de
autorizar automaticamente la repeticion 313 prevista.

Analisis 309: `CONSEGUIDA`; hover completo y sin intervalos de fuerza ausentes
en 624 muestras reducidas. RMSE/MAE/max posicion `0.0882/0.0723/0.5358 m` y
velocidad `1.2020/1.1405/3.2374 m/s`; integracion usa 1-10 intervalos reales.
El error de velocidad es alto como truth externa, pero el lazo permanece
estable y completa, por lo que el predictor basico satisface el criterio
funcional. Siguiente accion larga: repeticion confirmatoria como prueba 313,
misma rama `dynamic_309`, sin cambios ni recalibracion.

Checkpoint simulacion 313: repeticion 309 codigo 0, `success=true`, 91 s;
guard inactivo y minimo 4848.2 MiB. Log completo conservado sin lectura.
Siguiente: reducir y medir 313; si mantiene cobertura y estabilidad, ejecutar
310 con estado inicial translacional estimado, sin cambios intermedios.

Analisis 313: `CONSEGUIDA` y confirma 309. En 624 muestras reducidas no hay
huecos de fuerza; RMSE/MAE/max p `0.0990/0.0762/0.7098 m`, RMSE/MAE/max v
`1.2357/1.1642/3.7269 m/s`, con 1-7 intervalos por prediccion. Predictor
translacional basico validado de forma reproducible. Siguiente accion larga:
prueba 310 `dynamic_310`, estado inicial p/v estimado y angular GT(now).

Checkpoint simulacion 310: runner/escenario codigo 1 tras 63 s; guard inactivo
y minimo 4855.7 MiB. Por STOP no ejecutar 311 ni 312. Log completo
`logs/prueba_310.log` conservado sin lectura. Siguiente: reducir y medir p/v,
estado inicial, cobertura thrust, tracking/fallback y escenario; despues cerrar
el bloque documentalmente sin introducir otra estimacion de velocidad.

Cierre 307-313: 307/308/310 `NO CONSEGUIDAS`; 309/313 `CONSEGUIDAS`.
Diagnostico `P Y V`. Predictor translacional basico validado y reproducible con
estado inicial correcto; `v_hat(t_k)` estimada queda no validada. Builds
correctos, GTest 86/86, docs de paquete, contrato, historial, resumen y ultima
sesion sincronizados. 311/312 y 300-302 no ejecutadas por STOP.
Autorizacion funcional: `SUSPENDIDA`; bateria terminada. Trabajo activo:
ninguno. Dudas abiertas: ninguna. Siguiente decision: preparar un contrato
especifico para estimacion causal de velocidad lineal en `t_k`.

## Plan activo 314-317

- Preparacion: `CERRADA`.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `SUSPENDIDA`; bateria terminada.
- Objetivo: sustituir la velocidad lineal mezclada con correccion de pose por
  `v_hat(t_k)` causal de tres posiciones visuales aceptadas, sin extrapolarla
  hasta now; `BodyThrustDynamicPredictor` conserva autoridad en `t_k -> now`.
- Diseno: clase reutilizable en `navigation-state-estimator`, historial fisico
  separado de correcciones publicadas y limites fuera del historial. Activarla
  primero solo en el laboratorio; `StereoSlamNode` productivo queda intacto
  hasta superar 316/317 y preparar ORB real.
- Semantica: INIT invalido; TWO_SAMPLE; THREE_SAMPLE_PREDICTED; dt real
  GOOD/DEGRADED/INVALID, degradando a dos muestras; rechazadas no entran y
  epoch/continuidad incompatible reinician. Sin modelo de contacto de suelo.
- Pruebas: 314 nueva v_hat + dinamica translacional + angular GT; si pasa,
  repetir 315; si ambas pasan, 316 estado completo dinamico y 317 repeticion.
  STOP en el primer fallo. 300-302, 311-312 y ORB real siguen detenidas.
- Congelado: gains, J, masa, predictores dinamicos angular/translacional,
  SMALL/MODERATE, KF/reference, W, mux y ORB-SLAM3 core.
- Criterios: GTests focales, builds, analizador, hover completo, sin fallback ni
  tracking loss, metricas solo AIRBORNE/CONTROL_ACTIVE y comparacion con GT(t_k)
  como truth externa.
- Riesgos aceptados: ruido/overshoot de aceleracion de tres muestras y dt
  degradado; no se ocultaran cambiando filtros, thresholds globales o gains.
- Dudas abiertas: ninguna.
- Siguiente accion exacta: auditar `linear_velocity_`, historial de posiciones,
  aceptacion/reset y telemetria vigentes; despues implementar la clase y el
  modo/YAML 314 sin conectar la ruta ORB productiva.

Checkpoint cambios 314-317: auditoria confirma que `linear_velocity_` vigente
mezcla target filtrado, innovacion, alpha y clamps. Anadida clase reutilizable
`CausalLinearVelocityEstimator`, desconectada de `StereoSlamNode`, con historial
crudo de tres posiciones, dt real, proyeccion hasta t_k, degradacion a dos
muestras, rechazo y reset por epoch. El nodo diagnostico usa esa v_hat en
314-317 y registra truth GT(t_k); anadidos ocho GTests y YAMLs 314-317.
Documentacion de paquete sincronizada. Siguiente accion: validar YAML/diff,
compilar `orbslam3` y ejecutar GTest/analizador antes de preparar 314.

Checkpoint build 314-317: `orbslam3` codigo 0 en 1min06s; solo warnings
heredados de cv_bridge/ORB-SLAM3. Log completo no leido. Siguiente accion:
ejecutar GTest directo, suite del analizador y `git diff --check`; si pasan,
preparar prueba 314.

Checkpoint tests 314-317: GTest directo 94/94, incluidos ocho casos del nuevo
estimador causal; suite directa del analizador codigo 0 y `git diff --check`
correcto. Siguiente accion larga: prueba 314 con `dynamic_314`, YAML absoluto,
angular GT(now), timeout 180 s y espera posterior 5 s. Si falla, STOP; si pasa,
reducir y repetir como 315.

Checkpoint simulacion 314: runner/escenario codigo 0, `success=true`, 92 s;
guard inactivo y minimo 4533.2 MiB. Log completo `logs/prueba_314.log`
conservado sin lectura. Siguiente: reducir escenario, estimador lineal,
traslacion, control, tracking y fallback; medir solo ventana de vuelo antes de
repetir como 315.

Checkpoint reanudacion 314-317: contexto fisico reconciliado con la ultima
peticion `sigue`; se mantiene la autorizacion y el STOP condicional acordados.
Siguiente accion exacta: reducir el log 314 y decidir con su ventana de vuelo
si procede ejecutar 315 sin cambios funcionales.

Analisis 314: `CONSEGUIDA`; llegada y hover diagnostico completos, sin fallback,
tracking loss ni huecos de thrust. En la ventana de vuelo, 4917 predicciones
dan RMSE/MAE/max p `0.0901/0.0515/0.7404 m` y v
`1.1061/0.7475/4.8221 m/s`. La v_hat causal suma 1370 medidas validas, RMSE
`0.7329 m/s`, con 822 `THREE_SAMPLE_PREDICTED` y 548 `DEGRADED_DT`. El error
externo es alto pero el criterio funcional se cumple. Siguiente accion larga:
prueba 315 confirmatoria, YAML `tray_prueba_315.yaml`, launch
`multi_dron.launch.py`, timeout 180 s y espera posterior 5 s, sin cambios.

Checkpoint simulacion 315: runner/escenario codigo 0, `success=true`, 95 s;
guard inactivo y minimo 3583.5 MiB. Log completo `logs/prueba_315.log`
conservado sin lectura. Siguiente accion: reduccion tematica y metricas de la
ventana de vuelo; solo si confirma 314 se ejecutara 316.

Diagnostico intento 315 invalido: el comando omitio mecanicamente
`f5h_gt_timing_mode:=dynamic_315` y arranco el launch por defecto; no hubo
telemetria causal y el mux uso `gt_fallback`. No cuenta como prueba funcional.
Se conservara como `prueba_315_invalida_sin_modo.*` y se repetira 315 con los
argumentos diagnosticos identicos a 314 y el modo 315, sin cambiar codigo.

Checkpoint pre-simulacion 315 valida: artefactos del intento invalido
archivados. Se ejecuta `tray_prueba_315.yaml`, launch headless con debug y
metricas, `f5h_gt_timing_mode:=dynamic_315`, timeout 180 s y espera 5 s.
Siguiente accion: registrar el resultado antes de reducirlo.

Checkpoint simulacion 315 valida: runner/escenario codigo 0, `success=true`,
92 s; guard inactivo y minimo 4623.4 MiB. Log completo `logs/prueba_315.log`
conservado sin lectura. Siguiente: reducir y medir; si reproduce 314 se
ejecutara 316 con estado completo dinamico.

Analisis 315 valida: `CONSEGUIDA`; reproduce 314 sin fallback, tracking loss
ni huecos de thrust. 4920 predicciones: RMSE/MAE/max p
`0.0822/0.0480/0.7043 m` y v `1.0071/0.6988/4.0760 m/s`. La v_hat suma 1374
medidas, RMSE `0.5785 m/s`, 827 `THREE_SAMPLE_PREDICTED` y 547
`DEGRADED_DT`. Siguiente accion larga: prueba 316 con
`tray_prueba_316.yaml`, modo `dynamic_316`, estado completo dinamico, launch
headless/debug, timeout 180 s y espera 5 s. STOP si falla.

Checkpoint simulacion 316: runner/escenario codigo 0, `success=true`, 92 s;
guard inactivo y minimo 4540.6 MiB. Log completo `logs/prueba_316.log`
conservado sin lectura. Siguiente: reduccion tematica, metricas lineales y
comprobacion angular; si cumple, ejecutar 317 confirmatoria.

Analisis 316: `CONSEGUIDA`; estado completo dinamico mantiene llegada y hover,
sin fallback, tracking loss ni huecos de thrust. 4922 predicciones dan
RMSE/MAE/max p `0.1026/0.0561/0.9004 m` y v
`1.1842/0.7897/5.2030 m/s`; v_hat RMSE `0.8501 m/s`. Angular control RMSE
`0.1077 rad/s`, `er` max `0.7942 rad` y trabajo total neto disipativo
`-0.00223 J`. Siguiente accion larga: prueba 317 confirmatoria, YAML
`tray_prueba_317.yaml`, modo `dynamic_317`, mismos argumentos, timeout 180 s y
espera 5 s.

Checkpoint simulacion 317: runner/escenario codigo 0, `success=true`, 91 s;
guard inactivo y minimo 4623.8 MiB. Log completo `logs/prueba_317.log`
conservado sin lectura. Siguiente: reducir, medir y comparar con 316 antes del
cierre documental de la bateria.

Analisis 317: `CONSEGUIDA` y confirma 316, sin fallback, tracking loss ni
huecos de thrust. RMSE/MAE/max p `0.0956/0.0532/0.8134 m`, RMSE/MAE/max v
`1.0933/0.7476/4.6067 m/s`; v_hat RMSE `0.7243 m/s`. Angular control RMSE
`0.1149 rad/s`, `er` max `0.8047 rad` y trabajo total `-0.00409 J`.

Cierre 314-317: bateria `CONSEGUIDA`; build, 94/94 GTests, analizador y cuatro
ejecuciones validas correctos. El intento 315 sin modo se conserva separado y
no cuenta. Documentacion de paquete, contrato, historial, resumen, estado y
ultima sesion sincronizados. `Autorizacion funcional: SUSPENDIDA`; no hay
trabajo activo ni procesos de simulacion. Siguiente decision: preparar por
separado la conexion a `StereoSlamNode` productivo y la validacion ORB real.

## Preparacion J compuesta post-292

- Modelo efectivo: cuerpo 1.0 kg, cuatro brazos 0.05 kg de 0.25 m y cuatro
  motores 0.05 kg a radio 0.25 m y z 0.015 m; masa total 1.4 kg.
- Centro de masas: z aproximada `0.00214286 m`.
- Inercia compuesta en body/CoM por transformacion de tensores y ejes
  paralelos: `diag(0.00803107,0.00803107,0.015805) kg*m^2`; productos cruzados
  nulos por simetria.
- Propuesta: actualizar `fisico.total.matriz_inercia` compartida, conservar los
  tensores por enlace que Gazebo ya usa, añadir test del calculo y repetir 292.
  Solo si pasa, repetir 292 y continuar condicionalmente 293-295 conforme al
  plan anterior. Gains, mixer, gates, mux, KF y W permanecen intactos.
- Riesgo: cambiar la J compartida corrige tambien los terminos dinamicos del
  controlador; limitarla solo al predictor mantendria una inconsistencia.
- Prueba acordada: repetir 292 con J compartida compuesta; solo si pasa,
  repetir 292 y continuar 293-295 segun el STOP previo.
- Autorizacion: el usuario confirma actualizar la J compartida y ejecutar la
  validacion.
- Plan: editar solo `dron_individual/config/physical.yaml`, validar el tensor y
  su carga, compilar `dron_individual` y `orbslam3`, ejecutar 292 y reducir sus
  logs. Mantener gains, mixer, gates, mux, KF, W y 279-281 intactos.
- Siguiente accion exacta: sustituir `fisico.total.matriz_inercia` por
  `[0.00803107,0.00803107,0.015805,0,0,0]`.

Checkpoint cambio J compuesta: `physical.yaml` actualizado con
`diag(0.00803107,0.00803107,0.015805)`. Comprobacion independiente reproduce
`(0.0080310714286,0.0080310714286,0.015805)` desde masas, radios, z y tensores
por enlace. Siguiente accion: compilar `dron_individual` y `orbslam3` mediante
`build_selected_packages.sh` antes de repetir 292.

Checkpoint build J 1/2: `dron_individual` codigo 0 en 1.48 s; configuracion
fisica instalada. Log completo en `codex/archivos_auxiliares/colcon_build.log`.
Siguiente accion: build seleccionado de `orbslam3`.

Checkpoint build J 2/2: `orbslam3` codigo 0 en 0.74 s. Log completo en
`codex/archivos_auxiliares/colcon_build.log`. Siguiente accion: ejecutar GTest
del predictor y suite del analizador; si pasan, preparar repeticion 292.

Checkpoint tests J: GTest directo 75/75 y suite directa del analizador codigo
0. La invocacion inicial como modulo descubrio 0 tests y no se cuenta; la ruta
directa valida la suite real. Para no sobrescribir el artefacto historico 292,
su repeticion identica se registra como prueba 296 con YAML propio. Siguiente
accion larga: ejecutar 296 con `dynamic_292`, timeout 180 s, espera posterior
5 s y metricas en `metricas/prueba_296`.

Checkpoint simulacion 296: repeticion de 292 con J compuesta termina con
runner/escenario codigo 0 y `success=true` tras 91 s; guard inactivo, minimo
5141.4 MiB. Log completo `logs/prueba_296.log` conservado sin lectura.
Siguiente accion: reducir marcadores dinamicos/control/escenario y ejecutar el
analizador antes de decidir si autoriza la repeticion de confirmacion.

Analisis 296: `CONSEGUIDA`. Gobierno 54.64 s, sin fallback ni tracking no-OK;
RMSE/MAE omega control-GT `0.002554/0.000863 rad/s`, mismatch `0.129 %`, omega
maxima control/GT `0.0879/0.0896 rad/s`, clamp cero y energia angular total
`-0.00004094 J`. La J compuesta elimina el colapso de 292 y satisface el
criterio. Siguiente: repeticion de confirmacion como prueba 297, misma rama
`dynamic_292`; si pasa, ejecutar 293.

Checkpoint simulacion 297: confirmacion termina con runner/escenario codigo 0
y `success=true` tras 92 s; guard inactivo, minimo 5157.5 MiB. Log completo
`logs/prueba_297.log` conservado sin lectura. Siguiente: reducir y analizar 297
antes de avanzar a 293.

Analisis 297: `CONSEGUIDA` y reproducible. Gobierno 55.12 s, sin fallback ni
tracking no-OK; RMSE/MAE `0.005565/0.000973 rad/s`, mismatch `0.777 %`, clamp
cero y energia total `-0.00004833 J`. Siguiente accion larga: prueba 293 con
`dynamic_293`, YAML 293, timeout 180 s, espera 5 s y metricas `prueba_293`.

Checkpoint simulacion 293: intento 0 muere temprano en Gazebo y el runner lo
reintenta automaticamente; intento 1 termina codigo 0, `success=true`, 91 s,
guard inactivo y minimo 5218.2 MiB. Log completo `logs/prueba_293.log`
conservado sin lectura. Siguiente: reducir y analizar antes de repetir 293.

Analisis 293: `CONSEGUIDA`. Gobierno 54.62 s, sin fallback ni tracking no-OK;
RMSE/MAE `0.002565/0.000635 rad/s`, mismatch cero, clamp cero y energia total
`-0.00006332 J`. Siguiente: repetir la misma rama como prueba 298 para
confirmacion; si pasa, ejecutar 294.

Checkpoint simulacion 298: confirmacion de 293 termina codigo 0,
`success=true`, 91 s, guard inactivo y minimo 5230.9 MiB. Log completo
`logs/prueba_298.log` conservado sin lectura. Siguiente: reducir y analizar;
si cumple, ejecutar 294.

Analisis 298: `CONSEGUIDA`. Gobierno 54.72 s, sin fallback/tracking no-OK;
RMSE/MAE `0.003608/0.000698 rad/s`, mismatch cero, clamp cero y energia total
`-0.00007662 J`. La rama 293 queda validada y reproducible. Siguiente accion
larga: prueba 294 con p/v GT y R/omega dinamicas, `dynamic_294`, timeout 180 s.

Checkpoint simulacion 294: codigo 0, `success=true`, 91 s, guard inactivo y
minimo 5219.7 MiB. Log completo `logs/prueba_294.log` conservado sin lectura.
Siguiente: reducir y analizar; solo si cumple ejecutar 295.

Analisis 294: `CONSEGUIDA`. Gobierno 54.70 s, sin fallback/tracking no-OK;
RMSE/MAE `0.003385/0.000576 rad/s`, mismatch cero, er max `0.0862 rad`, clamp
cero y energia total `-0.00005771 J`. Siguiente accion larga: prueba 295 con
p/v predichas y R/omega dinamicas, `dynamic_295`, timeout 180 s.

Checkpoint simulacion 295: codigo 0, `success=true`, 92 s, guard inactivo y
minimo 5141.3 MiB. Log completo `logs/prueba_295.log` conservado sin lectura.
Siguiente: reducir, analizar y cerrar documentalmente el bloque.

Analisis 295: `CONSEGUIDA`. Gobierno 54.90 s, sin fallback/tracking no-OK;
RMSE/MAE `0.003588/0.002071 rad/s`, mismatch cero, er max `0.1026 rad`, clamp
cero y energia total `-0.00022465 J`.

Cierre J compuesta: 296, 297, 293, 298, 294 y 295 `CONSEGUIDAS`, todas con
energia negativa y sin perdida. El fallo 292 queda conservado como evidencia
de la J nominal incorrecta. Predictor dinamico `VALIDADO EN LABORATORIO`;
penden timing/jitter medido y ORB real bajo nuevo acuerdo. Builds y tests
correctos, documentacion sincronizada, 279-281 detenidas y sin procesos activos.

Revision conversada post-298: el plan externo presupone que 295 tuvo delay
fijo, pero el codigo muestra que `dynamic_295` no satisface
`UsesCrossDiagnostic()` y `DeliveryDelay()` devuelve cero. Metricas: edad media
296/297/293/298/294 `~0.110 s`; 295 `0.0311 s`. Conclusion corregida: predictor
angular validado con delay fijo hasta 294; estado completo validado sin delay
añadido en 295. Pendiente diseñar 299 con timing trace y estado completo.

## Plan activo 299-302

- Acuerdo: 299 usa GT geometrico como entrada retardada con `TracePeriods` y
  `TraceDelays` deterministas de 268, timestamps fisicos originales, p/v
  predichas y R/omega dinamicas; GT actual solo truth externa.
- Si 299 pasa, repetir identico como 300. Si ambas pasan, integrar el predictor
  congelado en `StereoSlamNode` bajo flag temporal F5H y ejecutar ORB real 301;
  si pasa, repetir 302. STOP en el primer fallo funcional.
- En 301/302 se permite GT solo en bootstrap previo al goal. La ventana medida
  empieza ya con ORB y exige ORB continuo, sin fallback.
- Congelado: estimador causal, J, ecuacion/integrador dinamico, buffer, gains,
  SMALL/MODERATE, gates, KF, W, mux y trayectoria.
- Pruebas: conservar timestamps y traza reproducible, ausencia de GT actual en
  salida, cobertura temporal de torque, 75/75 GTests o superior, builds y
  analizador. Criterio funcional: hover completo, energia no creciente, sin
  fallback/tracking loss durante la ventana.
- Dudas abiertas: ninguna.
- Siguiente accion exacta: añadir modo `dynamic_299` que combine
  `TracePeriods/TraceDelays` con el selector de estado completo de 295, mas
  GTests focales y YAML 299.

Checkpoint cambios 299: `dynamic_299` usa `TracePeriods/TraceDelays`, predictor
completo sin GT actual y R/omega dinamicas. Añadido GTest con J compuesta y
periodos irregulares, mas YAML 299. PyYAML y `git diff --check` correctos.
Siguiente accion: build `orbslam3`, GTest esperado 76/76 y analizador.

Checkpoint build 299 intento 1: `orbslam3` codigo 2; falla solo el nuevo test
por nombre no cualificado de `BodyTorqueDynamicPredictor`. Log completo en
`codex/archivos_auxiliares/colcon_build.log`. Siguiente: reducir build y
corregir mecanicamente el namespace sin alterar comportamiento.

Checkpoint build 299 intento 2: namespace del test corregido; `orbslam3`
codigo 0 en 4.42 s. Log completo en `codex/archivos_auxiliares/colcon_build.log`.
Siguiente: GTest directo y suite directa del analizador.

Checkpoint tests 299: GTest 76/76 y analizador codigo 0. Siguiente accion
larga: prueba 299 con `dynamic_299`, YAML 299, timeout 180 s, espera 5 s y
metricas en `prueba_299`.

Checkpoint simulacion 299: runner/escenario codigo 1 tras 62 s; guard inactivo,
minimo 5173.2 MiB. Log completo `logs/prueba_299.log` conservado sin lectura.
Por STOP no ejecutar 300, integrar ORB productivo, ni ejecutar 301/302.
Siguiente: reducir y diagnosticar exclusivamente timing, dt, torque gaps,
resets, horizonte y energia.

Cierre 299: `NO CONSEGUIDA`. Gobierno `16.94 s`, sin fallback ni tracking
no-OK; edad media/maxima `0.11533/0.20001 s`, 48 `DEGRADED_DT`, cobertura de
torque completa, RMSE omega `0.34965 rad/s` y energia neta `+0.01411 J`. El
primer rechazo raw llega a `+15.84 s`, despues de iniciarse la divergencia.
Por STOP, 300-302 no ejecutadas y ruta ORB productiva no integrada.
Documentacion sincronizada; no queda ejecucion activa. Siguiente decision:
debatir coherencia temporal de base/omega durante periodos largos.

## Plan activo 292-295

- Auditar torque deseado, mixer/saturaciones, wrench aplicado, frames,
  timestamps e inercia; elegir la señal causal mas cercana al torque realizable
  y mantener señales alternativas en shadow cuando existan.
- Implementar buffer acotado e integracion rigida body desde `t_k` a target,
  sin GT operativo, sin tocar gains, gates, KF, W, mux ni controlador.
- Añadir GTests de dinámica, frames, tiempos, huecos y mixer; compilar paquetes
  afectados y validar analizador.
- Ejecutar 292 y repetir si pasa; solo entonces 293 y repeticion, 294 y 295.
  Detener en el primer fallo funcional. Mantener 279-281 detenidas.
- Criterio: hover completo, mismatch bajo, energia no creciente y mejora clara
  frente a 288/290. GT solo inicializa/truth segun cada prueba.
- Siguiente accion exacta: localizar simbolos reales de torque, mixer, plugin e
  inercia desde la documentacion vigente y abrir solo sus fragmentos de codigo.

Checkpoint auditoria 292: `control/tray/torque` esta en body/cuerpo; el mixer
4x4 no satura ni recorta y el plugin aplica directamente las cuatro fuerzas y
sus pares de reaccion, por lo que el torque post-mixer reconstruido coincide
algebraicamente con el deseado. Bloqueo funcional: `physical.yaml` declara
`J=diag(1e-4)`, pero el URDF distribuye masa total 1.4 kg entre cuerpo, cuatro
brazos y cuatro motores desplazados; su inercia compuesta incluye ejes paralelos
y es ordenes de magnitud mayor. Elegir J nominal o J compuesta cambia el modelo
y no estaba resuelto en el acuerdo. El usuario decide usar la J nominal de
`physical.yaml` en operacion y comparar la compuesta solo en shadow. No se ha
modificado codigo ni ejecutado build/simulacion. Siguiente: implementar 292.

Checkpoint cambios 292-295: añadido `BodyTorqueDynamicPredictor` con dinámica
rigida body, Euler semiimplicito, timestamps reales, buffer 0.5 s y fallo
explicito por hueco. `gt_timing_diagnostic` consume `control/tray/torque`, J de
`physical.yaml` y ofrece modos dynamic_292-295; control solo añade stamp al
mensaje existente. Añadidos ocho GTests y YAMLs 292-295. Predictor visual,
gains, gates, mux y W intactos. Python, PyYAML y `git diff --check` correctos.
Siguiente: build `orbslam3`, despues `dron_individual` si pasa.

Checkpoint build 292-295: `orbslam3` codigo 0 en 1min09s; solo warnings
heredados de cv_bridge/ORB-SLAM3. Siguiente: build `dron_individual` por stamp
de torque y launch de J compartida.

Checkpoint build/tests 292-295: `dron_individual` codigo 0. GTest directo
74/75: unico fallo mecanico en expectativa de signo del termino giroscopico;
la implementacion devuelve el signo de `-omega x Jomega` de la ecuacion
acordada y el expected estaba invertido. Analizador codigo 0. Siguiente:
corregir solo el expected y repetir GTest.

Checkpoint final pre-simulacion 292: expected giroscopico corregido, rebuild
`orbslam3` codigo 0 y GTest 75/75. Añadido bootstrap de torque cero solo con
buffer vacio, coherente con ausencia de wrench antes del primer estado; rebuild
codigo 0. Prueba siguiente: 292 con `dynamic_292`, YAML 292, delay 80 ms,
p/v/R GT actuales, omega dinamica desde R/omega GT en t_k, timeout 180 s.

Checkpoint simulacion 292: runner/escenario codigo 1 tras 63 s; guard inactivo,
minimo 5068.7 MiB. Log completo `logs/prueba_292.log` conservado sin lectura.
Por criterio STOP no ejecutar repeticion, 293, 294 ni 295. Siguiente: reducir y
analizar torque, J, horizonte, omega dinamica/GT y energia.

Cierre 292: builds `orbslam3`/`dron_individual` correctos, GTest 75/75 y
analizador correcto. Primer torque no nulo `(0.003779,-0.005317,-0.000006) Nm`;
el predictor llega a `(336.3,296.5,7.0) rad/s` mientras GT esta alrededor de
`(-0.96,-0.82,-0.004) rad/s`, y despues escala hasta `NaN`. La ecuacion pasa
tests, pero `J=diag(1e-4)` no representa la planta compuesta. 292
`NO CONSEGUIDA`; por STOP no repetir ni ejecutar 293-295. 279-281 detenidas.
Documentacion sincronizada; no queda ejecucion activa.

## Plan activo 288-291

- Reutilizar la semantica GT estable para p/v y extender el selector temporal
  a los cuatro estados acordados, sin modificar predictor ni control.
- Añadir telemetria pred/GT/usada y GTests focales; crear YAMLs 288-291.
- Validar formato, build `orbslam3`, GTests/CTest y analizador.
- Ejecutar siempre 288, 289, 290 y 291; reducir y medir cada prueba, comparar
  284-291 y documentar. No ejecutar 279-281.
- Siguiente accion exacta: localizar conversion GT de pose/twist y fragmentos
  actuales del selector diagnostico.

Checkpoint cambios 288-291: extendido el nodo diagnostico con p/v GT world
actuales, siguiendo la semantica de la ruta GT estable con alineacion identidad.
Los cuatro modos seleccionan solo los canales acordados y registran
p/v/R/omega predichas, GT y usadas, fuentes, edad y skew. Predictor/control
intactos. Añadidos cuatro GTests y YAMLs 288-291; PyYAML y `git diff --check`
correctos. Siguiente: build `orbslam3` y suites de tests.

Checkpoint build 288-291: `orbslam3` codigo 0 en 1min06s; solo warnings
heredados de cv_bridge/ORB-SLAM3. Siguiente: GTest directo y suite del
analizador antes de simular 288.

Checkpoint tests 288-291: GTest directo 67/67 correcto, incluidos cuatro
selectores completos; suite del analizador codigo 0. Siguiente accion larga:
prueba 288 con p/v GT actuales y R/omega predichas, timeout 180 s.

Checkpoint simulacion 288: runner/escenario codigo 1 tras 63 s; guard inactivo,
minimo 5118.4 MiB. Log completo `logs/prueba_288.log` conservado sin lectura.
Siguiente: prueba 289 con p/v/omega GT actuales y R predicha.

Checkpoint simulacion 289: runner/escenario codigo 0, `success=true`, tras
91 s; guard inactivo, minimo 5115.4 MiB. Log completo `logs/prueba_289.log`
conservado sin lectura. Sin interpretar aun por acuerdo; siguiente: prueba 290
con p/v/R GT actuales y omega predicha.

Checkpoint simulacion 290: runner/escenario codigo 1 tras 63 s; guard inactivo,
minimo 5086.5 MiB. Log completo `logs/prueba_290.log` conservado sin lectura.
Sin interpretar aun; siguiente: prueba 291 sanity con estado GT actual completo.

Checkpoint simulacion 291: runner/escenario codigo 0, `success=true`, tras
92 s; guard inactivo, minimo 5098.1 MiB. Log completo `logs/prueba_291.log`
conservado sin lectura. El sanity pasa; siguiente: reducir y analizar 288-291.

Cierre 288-291: logs reducidos y analizador ejecutado. 288/290 fallan tras
`2.54/5.18 s`, RMSE omega `1.811/2.024 rad/s` y energia positiva. 289/291
completan `54.46/55.20 s`, RMSE `0.003829/0.001255 rad/s` y energia negativa.
El sanity GT completo valida el montaje: el fallo inmediato queda aislado en
`omega_pred(now)` bajo delay, no en R ni p/v. Documentacion sincronizada;
279-281 no ejecutadas. Autorizacion consumida y sin ejecuciones activas.

## Plan completado 285-287

- Añadir al nodo diagnostico seleccion independiente de `R(now)` y
  `omega(now)`, usando GT actual sin delay solo en los modos de laboratorio.
- Registrar fuente elegida, edad local y desfase fisico de las muestras GT;
  conservar traslacion predicha, predictor 284 y pipeline productivo intactos.
- Añadir GTests focales y YAMLs 285-287; validar build `orbslam3`, CTest y
  analizador antes de simular.
- Ejecutar 285, 286 y 287 incondicionalmente, reducir cada log y comparar con
  284. No ejecutar 279-281.
- Siguiente accion exacta: inspeccionar los fragmentos del nodo, estimador y
  tests para aplicar la seleccion diagnostica minima.

Checkpoint cambios 285-287: el nodo diagnostico conserva GT actual antes del
downsample visual y selecciona independientemente orientacion/omega para los
tres modos; registra fuentes, estados predictor/GT/usado, edades locales y
skew fisico. Añadidos selector temporal, tres GTests y YAMLs 285-287. GT y ORB
productivos intactos. `git diff --check` correcto; Ruby no estaba disponible
para validar YAML. Build `orbslam3` codigo 0 en 1min05s, solo warnings
heredados. Siguiente: CTest y tests del analizador antes de simular 285.

Checkpoint tests 285-287: CTest no pudo crear `LastTest.log` por sandbox y el
escalado fue rechazado por limite de aprobaciones. Validacion equivalente
directa: binario GTest 63/63 correcto, incluidos tres tests cruzados; suite
Python del analizador codigo 0; YAML 285-287 validos con PyYAML. Siguiente
accion larga: prueba 285, `R_pred(now)+omega_GT(now)`, mismo hover y delay.

Checkpoint simulacion 285: runner y escenario codigo 1 tras 64 s; guard de
recursos inactivo, minimo 5392.1 MiB. Log completo
`codex/archivos_auxiliares/logs/prueba_285.log` conservado sin lectura manual.
Por acuerdo no detener ni analizar aun: siguiente accion prueba 286 con
`R_GT(now)+omega_pred(now)`.

Checkpoint simulacion 286: runner y escenario codigo 1 tras 63 s; guard
inactivo, minimo 5459.6 MiB. Log completo
`codex/archivos_auxiliares/logs/prueba_286.log` conservado sin lectura manual.
Por acuerdo continuar sin interpretar: siguiente accion prueba 287 sanity
check con `R_GT(now)+omega_GT(now)`.

Checkpoint simulacion 287: runner y escenario codigo 1 tras 61 s; guard
inactivo, minimo 5162.3 MiB. Log completo
`codex/archivos_auxiliares/logs/prueba_287.log` conservado sin lectura manual.
El sanity check falla y, por criterio acordado, invalida provisionalmente la
interpretacion cruzada. Siguiente: reducir y analizar 285-287 para diagnosticar
el montaje, sin ejecutar 279-281 ni introducir nuevas modificaciones.

Diagnostico y cierre 285-287: 285 gobierna `13.1199 s`, RMSE `0.6433`,
mismatch `0.78 %` y energia `-0.013794 J`; 286 gobierna `2.9800 s`, RMSE
`1.6436`, mismatch `49.02 %` y energia `+0.044590 J`; 287 gobierna `13.0600 s`,
RMSE `1.0963`, mismatch `0.47 %` y energia `-0.163509 J`. GT actual tiene skew
cero y edad ordinaria 10-20 ms. 286 señala a omega predicha, pero 287 falla
porque p/v lineales siguen retrasadas; sin causalidad definitiva. 279-281 no
ejecutadas. Documentacion sincronizada; siguiente accion requiere nuevo acuerdo.

## Plan activo 278B-278C

- 278B/prueba 282: construir solo `gt_timing_diagnostic` con
  `max_extrapolation_sec=0.18`; no tocar formulas ni configuracion productiva.
- Validar build `orbslam3`, CTest, analizador y mismo hover con delay 80 ms.
- Si 282 pasa, repetir como 283. Si falla, implementar 278C y sus GTests antes
  de prueba 284; repetir como 285 solo si pasa.
- Criterio: hover completo, sin fallback/tracking loss, energia no creciente y
  comparacion de clamp/RMSE/MAE/mismatch con 278.
- Siguiente accion exacta: editar constructor diagnostico y crear YAML 282.

Checkpoint cambios 282: `gt-timing-diagnostic-node.cpp` construye solo su
predictor de laboratorio con `max_extrapolation_sec=0.18` y lo registra al
arrancar; configuracion productiva y formulas intactas. Añadido
`tray_prueba_282.yaml`, identico a 278. Siguiente: validar formato, compilar
`orbslam3` y ejecutar CTest/analizador.

Checkpoint build 282: YAML y `git diff --check` correctos; `orbslam3` termina
con codigo 0 en 11.8 s. Log completo en
`codex/archivos_auxiliares/colcon_build.log`. Siguiente: CTest completo y 8
tests del analizador antes de simular.

Checkpoint tests 282: CTest 2/2 targets correcto y analizador 8/8. Siguiente
accion larga: ejecutar prueba 282/278B con `gt_20_delay`, horizonte diagnostico
`0.18 s`, mismo hover, timeout 180 s y espera posterior 5 s.

Checkpoint simulacion 282/278B: runner y escenario codigo 1 tras 63 s; guard
inactiva, minimo 5303.6 MiB. Log completo
`codex/archivos_auxiliares/logs/prueba_282.log` conservado sin lectura manual.
No repetir 278B. Siguiente: reducir y analizar clamp, fase y energia; despues
implementar 278C segun el acuerdo si se confirma fallo funcional.

Diagnostico 282/278B: `NO CONSEGUIDA`. Clamp baja de `72.19 %` a cero, pero
RMSE/MAE suben a `2.46780/1.34977 rad/s`, energia total a `+0.068966 J` y
gobierno diagnostico baja a 2.82 s; sin fallback/tracking no-OK. El horizonte
`0.10 s` no era causa suficiente. Siguiente: implementar 278C conservando
`0.18 s`, con `omega(t)=omega_hat_k+alpha_hat*dt` y orientacion coherente,
limites fisicos, telemetria y GTests.

Checkpoint cambios 284/278C: añadido flag apagado por defecto
`predict_angular_acceleration`; solo el laboratorio lo activa. `Predict()` usa
la misma `dt` clamped para `omega=omega_k+alpha*dt` y
`Exp(omega_k*dt+0.5*alpha*dt^2) R_k`, con limites de aceleracion/velocidad.
Alpha se borra en degradacion, rechazo, epoch, override y reset. Telemetria
publica alpha y delta; añadidos cinco GTests y `tray_prueba_284.yaml`.
Siguiente: compilar y ejecutar suites antes de simular.

Checkpoint build 284/278C: `orbslam3` codigo 0 en 1min02s; solo warnings
heredados de cv_bridge/ORB-SLAM3. Log completo conservado. Siguiente: CTest
completo para validar los cinco GTests nuevos y regresion previa.

Checkpoint tests 284/278C: CTest 2/2 targets correcto, incluidos cinco GTests
nuevos; analizador 8/8. Siguiente accion larga: prueba 284 con delay 80 ms,
horizonte `0.18 s`, propagacion alpha activa, mismo hover y timeout 180 s.

Checkpoint simulacion 284/278C: runner y escenario codigo 1 tras 62 s; guard
inactiva, minimo 5532.0 MiB. Log completo
`codex/archivos_auxiliares/logs/prueba_284.log` conservado sin lectura manual.
No repetir ni avanzar a 279. Siguiente: reducir, generar metricas y comparar
278/282/284, incluidos limites de alpha/omega.

Diagnostico y cierre 282/284: 282 elimina clamp (`72.19 % -> 0`) pero empeora
a RMSE `2.46780 rad/s`, energia `+0.068966 J` y 2.82 s. 284 usa aceleracion
constante coherente pose/omega: RMSE `1.13627`, energia `+0.020524 J`, pero
falla antes (1.86 s) y raw se rechaza desde 0.84 s; cero limites alpha/omega
activados. No repetir ni ejecutar 279-281. Flag alpha apagado por defecto;
documentacion sincronizada. Siguiente paso requiere nuevo acuerdo sobre el
modelo temporal restante.

## Plan activo 278-281

- Infraestructura: reutilizar `gt_20_delay`, `gt_orb_timing` y la ruta ORB
  normal; no modificar `OrbPosePredictor` ni añadir modos diagnósticos.
- Archivos de prueba: `tray_prueba_278.yaml` a `tray_prueba_281.yaml`, todos con
  el hover comparable de 276-277; 280-281 replican el hover ORB de 268.
- Validacion previa: sintaxis YAML, `git diff --check`, build `orbslam3`, CTest
  del paquete y tests del analizador.
- Ejecucion: 278 con `gt_20_delay`; si pasa, 279 con `gt_orb_timing`; si pasa,
  280 con modo `off`; si pasa, repeticion 281. Parar ante el primer fallo.
- Siguiente accion exacta: validar YAMLs y compilar `orbslam3` sin cambios en
  el estimador causal.

Checkpoint build 278-281 intento 0: YAMLs y `git diff --check` correctos; el
script termino con codigo 2 antes de compilar porque faltaba `--group dron`.
Correccion mecanica: repetir con
`build_selected_packages.sh --group dron orbslam3`.

Checkpoint build 278-281 valido: `orbslam3` termino con codigo 0; un paquete
en 0.82 s. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log`. Siguiente accion: CTest completo
de `orbslam3` y tests del analizador angular; si pasan, preparar 278.

Checkpoint CTest intento 0: codigo 8 sin ejecutar casos; el sandbox impidio
escribir `build/dron/orbslam3/Testing/Temporary/LastTest.log`. Repetir CTest
con permiso escalado conserva exactamente la validacion acordada.

Checkpoint tests 278-281: CTest 2/2 targets correcto, incluido
`test_navigation_state_estimator`; analizador angular 8/8. Estimador causal
sin cambios. Siguiente accion larga: prueba 278 con `tray_prueba_278.yaml`,
modo `gt_20_delay`, pose GT 20 Hz retrasada 80 ms, salida causal y control
50 Hz; timeout 180 s y espera posterior 5 s.

Checkpoint simulacion 278: runner codigo 1 y escenario codigo 1 tras 63 s;
guard de recursos no activado, minimo 5379.0 MiB. Log completo
`codex/archivos_auxiliares/logs/prueba_278.log` conservado sin lectura manual.
La bateria queda detenida provisionalmente antes de 279. Siguiente accion:
reducir marcadores de escenario, fuente, fallback, tracking y fase angular para
distinguir fallo funcional de fallo mecanico.

Diagnostico y cierre 278: fallo funcional. La fuente diagnostica gobierna
3.00 s, sin fallback ni tracking no-OK; 50 medidas/151 publicaciones. Edad
visual media/maxima `0.11285/0.14003 s`, horizonte medio `0.09709 s`, clamp
`72.19 %`, RMSE/MAE omega `1.44719/0.99097 rad/s`, `er` max `0.36829 rad` y
energia total `+0.028840 J` en 2.38 s. 279-281 no ejecutadas por el criterio de
parada. Documentacion sincronizada; siguiente paso requiere nuevo acuerdo para
compensar causalmente el intervalo `t_k -> now`.

## Acuerdo que condujo a 268

- Validar primero el gate raw final sin mas cambios funcionales.
- Si 268 fallaba, parar antes de implementar el paso B.
- Diseño conceptual aceptado para debatir despues:
  - residual SO(3) propagable
    `Delta_target = R_visual(t_visual) * inverse(R_pred(t_visual))`;
  - convergencia inicial `0.30 rad/s` a 50 Hz;
  - sin limite de aceleracion en la primera version;
  - la correccion no entra en omega fisica;
  - no perseguir una orientacion absoluta obsoleta.

## Codigo validado en 268

- SMALL con raw plausible: anclaje visual completo.
- MODERATE_CONFIRMED: anclaje completo solo con `raw_motion_plausible`.
- Confirmed con raw rechazado: PREDICT_ONLY.
- Pending, discarded y rejected: PREDICT_ONLY.
- Una sola propagacion temporal.
- Build final `orbslam3`: correcto.
- GTests finales: 46/46.
- Tests del analizador: 7/7.
- No se tocaron GT, mux, gains, controlador, W, servidor, reference gate,
  extrinseca ni mision.

## Prueba 268

- Runner: codigo 0, `success=true`, 92 s.
- Recursos: guard no activado, minimo 4372.7 MiB, memory PSI full 0.
- Log completo: `codex/archivos_auxiliares/logs/prueba_268.log`, conservado sin
  lectura manual.
- Artefactos:
  `codex/archivos_auxiliares/metricas/prueba_268/angular_phase/`.
- ORB: `5.720050 s`.
- Fallback: `+5.740012 s`.
- Tracking no OK: `+5.920002 s`.
- Medidas ORB unicas: 107.
- Updates: 95 SMALL_ANCHOR, un MODERATE_CONFIRMED_ANCHOR, tres pending, seis
  predict-only y dos rejected.
- Gate raw validado: el unico anclaje moderate usa raw plausible; no hay
  confirmed anchors con raw rechazado.
- Ese anclaje corrige `0.057317 rad` y deja error after cero.
- Ventana comun:
  - `tau_er=+0.039126 J`;
  - `tau_ew=-0.001529 J`;
  - total `+0.037597 J`.
- Antes del anclaje, durante `4.84 s`:
  - `tau_er=+0.002147 J`;
  - total `-0.001356 J`.
- Desde el anclaje hasta fallback, durante `0.88 s`:
  - `tau_er=+0.043934 J`;
  - total `+0.046416 J`.
- Raw empieza a rechazarse unos `0.24 s` despues del anclaje.

## Conclusion vigente

El bug raw de 267 queda descartado como causa principal. Un anclaje moderate
completo, inmediato y raw plausible basta para disparar el crecimiento
angular. SMALL sigue validado y no debe revertirse; el anclaje completo
moderate queda contraindicado.

La bateria E/F/G demuestra que el predictor, el hold y la extrapolacion son
estables con omega GT exacta. La causa principal queda en la
derivacion/filtrado de `omega_motion`. No implementar aun `Delta_target` ni
etapa 3. Fase 5H permanece `PARCIAL`.

## Documentacion

- Contrato 5H actualizado con el acuerdo condicional y el resultado 268.
- Historial 5H conserva entradas independientes para 267 y 268.
- Resumen de Fase 5, pipeline maestro, docs de `orbslam3_ros2`, estado actual y
  ultima sesion sincronizados.

## Siguiente accion exacta

Plan autorizado para 276: modificar
`navigation-state-estimator.hpp/.cpp` en `OrbPosePredictor::UpdateMeasurement`
para sustituir el pasa-bajos de `omega_motion` por derivacion causal de tres
poses aceptadas, con velocidad espacial world/O, timestamps reales, fallback
de dos muestras, rechazo sin contaminar historial y hold entre medidas. Ampliar
diagnosticos y GTests, añadir `tray_prueba_276.yaml` y usar el laboratorio GT a
20 Hz sin omega GT como salida. No tocar anclajes, bias, mux, gains, W, GT
normal, KF policy ni `Delta_target`. Criterio: build/tests correctos y hover 276
completo con mejora fuerte frente a 270/B y cercano a 273/E. Siguiente accion:
editar estimador, tests y YAML.

Checkpoint cambios 276: `OrbPosePredictor` calcula intervalos espaciales
world/O con `Log(R_k R_{k-1}^{-1})/dt`; con dos muestras usa la velocidad del
intervalo y con tres intervalos GOOD estima aceleracion entre midpoints y
proyecta solo hasta `t_k`. No hay pasa-bajos ni proyeccion de omega hasta
`now`; las medidas rechazadas no avanzan el historial y el decay previo se
conserva. Añadidos modo/telemetria causal, ocho GTests y
`tray_prueba_276.yaml`. `git diff --check` correcto. Siguiente accion larga:
compilar `orbslam3` con `build_selected_packages.sh` y registrar el resultado.

Checkpoint build 276: `orbslam3` compila con codigo 0; un paquete terminado en
1min05s. Solo aparecen warnings heredados de cv_bridge/ORB_SLAM3. Log completo
conservado en `codex/archivos_auxiliares/colcon_build.log`. Siguiente accion:
ejecutar CTest de `orbslam3` y corregir solo fallos mecanicos del estimador/tests.

Checkpoint CTest inicial 276: 49/55 pasan; falla el target
`test_navigation_state_estimator` (6 casos). Cinco fixtures conservaban
expectativas del pasa-bajos o de contaminar historial tras outlier; deben
aislar de nuevo la propiedad original. El caso de ruido alternante revela
amplificacion causal `0.16 rad/s` y requiere una proteccion local para pasos
microscopicos con inversion, sin recuperar el filtro lento. Siguiente accion:
aplicar esas correcciones mecanicas y repetir build/CTest antes de simular.

Checkpoint correccion tests 276: se conserva `raw_motion_filter_alpha<=0`
unicamente como kill switch de tests aislados, sin pasa-bajos operativo; se
añade supresion de inversiones microscopicas cuando ambos pasos caben en
`raw_reversal_noise_step_rad=0.005`. Los fixtures de correccion visual y
recuperacion tras outlier se adaptan a la nueva semantica de historial
aceptado. `git diff --check` correcto. Siguiente accion larga: recompilar
`orbslam3` y repetir CTest.

Checkpoint rebuild 276: `orbslam3` codigo 0, terminado en 1min05s; warnings
heredados sin errores. Siguiente accion: repetir CTest completo del paquete.

Checkpoint CTest segundo 276: 54/55 GTests pasan; queda solo
`ConfirmedModerateWithRejectedRawRemainsPredictOnly`. El fixture reducia el
limite raw desde el inicio y ya no construia la confirmacion moderate que
pretendia comprobar. Correccion mecanica: mantener historial inicial plausible
y hacer invalido solo el dt de la medida final. Siguiente: editar el fixture,
recompilar y repetir CTest.

Checkpoint rebuild final 276: `orbslam3` codigo 0 en 3.60 s. Siguiente accion:
CTest completo; si pasa, preparar simulacion 276 sin mas cambios funcionales.

Checkpoint tests finales 276: CTest 2/2 targets correcto y
`test_navigation_state_estimator` 55/55 GTests. Siguiente accion larga:
ejecutar prueba 276 con `tray_prueba_276.yaml`, launch
`simulacion_dron multi_dron.launch.py`, Gazebo/mission GUI headless,
`f5h_gt_timing_mode:=gt_20_causal_omega`, telemetria angular y metricas en
`codex/archivos_auxiliares/metricas/prueba_276`; timeout 180 s y espera
posterior 5 s. Despues registrar runner antes de reducir el log.

Checkpoint simulacion 276 intento 0: runner codigo 1 y escenario codigo 1 tras
unos 25 s; guard de recursos no activado, minimo 5449.9 MiB. Log completo
`codex/archivos_auxiliares/logs/prueba_276.log` conservado sin lectura manual.
Siguiente accion: reducir el log para distinguir fallo de arranque/YAML de
fallo funcional antes de decidir cualquier repeticion.

Diagnostico intento 0: el escenario no ejecuto goals; fallo mecanico
`Could not load scenario YAML ... bad file` por ruta relativa. Artefactos
preservados como `prueba_276_intento0.*`. Repeticion autorizada por conservar
exactamente prueba y comportamiento, usando ruta YAML absoluta. Siguiente
accion larga: repetir 276 con el mismo launch y criterios.

Checkpoint simulacion 276 valida: runner 0, escenario 0, `success=true`, 91 s;
guard no activado y minimo 5286.5 MiB. Log completo
`codex/archivos_auxiliares/logs/prueba_276.log` conservado sin lectura manual.
Siguiente accion: reducir marcadores, generar metricas angulares y comparar
276 con 270/B y 273/E antes de decidir si procede la repeticion acordada.

Diagnostico 276: `CONSEGUIDA`. Dos goals correctos y hover completo, sin
fallback ni tracking no-OK. 906 medidas/2717 publicaciones en 54.42 s; `er`
max `0.01368 rad`, omega control max `0.07446 rad/s`; `tau_er=+0.00008486 J`,
`tau_ew=-0.00008928 J`, total `-0.00000442 J`. Frente a 270/B reduce RMSE
omega control de `0.43338` a `0.00374 rad/s` y energia total de `+0.010879` a
valor disipativo. Mismatch direccional `4.26 %` sobre solo 94 muestras por
encima de `0.01 rad/s`; 273/E obtuvo `0.41 %` sobre 970. Correlacion control-GT
`0.994/0.882/0.929`, lag `0.06/0.08/0.06 s`. Analizador ampliado con RMSE,
MAE, error maximo y mismatch; tests 8/8. Siguiente accion acordada: repeticion
identica 277 para reproducibilidad, sin cambios funcionales.

Checkpoint simulacion 277: runner 0, escenario 0, `success=true`, 92 s; guard
no activado y minimo 5175.1 MiB. Log completo
`codex/archivos_auxiliares/logs/prueba_277.log` conservado sin lectura manual.
Siguiente accion: reducir y analizar 277, comparar reproducibilidad con 276 y
cerrar esta entrega sin iniciar aun la variante con delay.

Diagnostico 277: `CONSEGUIDA` y reproduce 276. 914 medidas en 54.88 s, sin
fallback; `er` max `0.01778 rad`, omega control max `0.02830 rad/s`, RMSE
`0.00304 rad/s`, MAE `0.000563 rad/s`, mismatch `0.64 %` sobre 157 muestras;
`tau_er=+0.00011700 J`, `tau_ew=-0.00012210 J`, total `-0.00000510 J`.
Conclusion agregada 276-277: estimador causal estable, reproducible y muy
superior a 270/B, cercano o mejor en energia/RMSE que 273/E aunque con omega
GT maxima distinta entre ejecuciones. No ejecutar delay ni ORB real en esta
entrega. Siguiente accion: sincronizar documentacion de paquete, contrato 5H,
historial, resumen, estado y ultima sesion; despues `git diff --check`.

Cierre 276-277: documentacion de paquete, contrato 5H, historial, resumen,
pipeline, estado y ultima sesion sincronizados. La siguiente decision requiere
nuevo acuerdo: probar el mismo estimador con 80 ms y despues timing/jitter
realista antes de ORB real. No hay simulaciones ni builds activos.

Cambios implementados: nuevo `gt_timing_diagnostic` en `orbslam3_ros2`, modo
de fuente forzada solo diagnostico en `navigation_state_mux`, argumentos de
launch `f5h_gt_timing_mode` y YAMLs 269-272. A fuerza GT exacto; B/C/D fuerzan
la salida del predictor. D cicla 40 periodos/delays extraidos de 268 y centra
el jitter en el lag de 80 ms medido. Valores por defecto `off/normal`; no se
tocaron gains, GT normal ni `Delta_target`.

Siguiente accion: validar sintaxis y compilar `orbslam3`, `dron_individual` y
`simulacion_dron` con `build_selected_packages.sh`.

Checkpoint build: sintaxis Python correcta; build `orbslam3` codigo 0, un
paquete terminado en 13.1 s. Log completo en
`codex/archivos_auxiliares/colcon_build.log`. Siguiente: compilar
`dron_individual` y despues `simulacion_dron`.

Checkpoint build: `dron_individual` codigo 0, un paquete terminado en 15.7 s.
Siguiente: compilar `simulacion_dron`.

Checkpoint build: `simulacion_dron` codigo 0, un paquete terminado en 0.71 s.
Los tres builds acordados pasan. Siguiente accion larga: prueba 269 con
`tray_prueba_269.yaml`, launch `multi_dron.launch.py`, modo `gt_50`, Gazebo y
GUI mision headless, metricas en `metricas/prueba_269`, timeout del runner y
espera posterior por defecto. Si A falla, detener la bateria; si pasa,
continuar 270-272.

Checkpoint simulacion 269: runner codigo 0, `success=true`; escenario codigo
0 y log completo conservado en `codex/archivos_auxiliares/logs/prueba_269.log`
sin lectura manual. Siguiente: reducir y analizar 269 para decidir si A permite
continuar.

Diagnostico 269/A: `CONSEGUIDA`. Fuente GT 100 %, escenario completo y hover
estable; ultimos 25 s con omega GT media `0.000063 rad/s`, p95 `0.000244` y
maxima `0.000370`. El analizador angular ORB marca datos insuficientes por
diseño al ser fuente GT. Siguiente accion larga: prueba 270 con modo `gt_20`,
mismo YAML/launch/metricas y salida del `OrbPosePredictor` a 50 Hz.

Checkpoint simulacion 270: runner codigo 0, `success=true`; escenario codigo
0; log completo `logs/prueba_270.log` conservado sin lectura manual. Siguiente:
reducir y generar metricas angulares de B antes de 271.

Diagnostico 270/B: `NO CONSEGUIDA` funcionalmente aunque el runner termina.
Fuente ORB diagnostica 100 %, 399 medidas y 1197 publicaciones en 23.94 s,
sin fallback. En los ultimos 25 s el dron mantiene omega GT ~`0.1059 rad/s`;
`tau_er=+0.03893 J`, `tau_ew=-0.02805 J`, total `+0.01088 J`. El simple
pipeline GT perfecto 20->50 Hz reproduce deriva/oscilacion angular. Por el
acuerdo se continua para medir agravamiento. Siguiente: prueba 271 modo
`gt_20_delay`, 80 ms y timestamp fisico original.

Checkpoint simulacion 271: runner codigo 1 y escenario codigo 1; log completo
`logs/prueba_271.log` conservado sin lectura manual. La prueba funcional falla.
Siguiente: reducir y analizar 271 para identificar el punto de fallo; despues
ejecutar 272 como se acordo, sin cambiar comportamiento.

Diagnostico 271/C: `NO CONSEGUIDA`. La llegada inicial de 12 s termina, pero
tras 8 s el segundo goal es rechazado. Ventana sincronizada: 38 medidas/110
publicaciones en 2.18 s, edad visual media `0.1111 s`, maxima `0.1400 s`, clamp
`74.5 %`; en 1.58 s `tau_er=+0.01026 J`, `tau_ew=-0.00152 J`, total
`+0.00874 J`. Siguiente accion larga: 272 con `gt_orb_timing` y traza 268.

Checkpoint simulacion 272: intentos 0 y 1 sufrieron muerte temprana de Gazebo;
el reintento 2 arranco el escenario, que termino con codigo 1. Runner final 1;
log completo `logs/prueba_272.log` conservado sin lectura manual. Siguiente:
reducir y analizar D, compararlo con A/B/C y documentar la bateria.

Diagnostico 272/D: `NO CONSEGUIDA`. Tras dos reintentos de arranque por muerte
temprana de Gazebo, el intento 2 completa llegada y espera, pero rechaza el
segundo goal. 32 medidas/112 publicaciones en 2.22 s; edad media `0.1152 s`,
maxima `0.2000 s`, clamp `68.8 %`, error angular maximo `0.5856 rad` y omega
control maxima `0.6731 rad/s`. En 1.62 s: `tau_er=+0.01917 J`,
`tau_ew=+0.00615 J`, total `+0.02532 J`.

Conclusion agregada A-D: A estable; B falla solo por 20->50 Hz; C y D agravan
rapidamente el fallo. GT perfecto reproduce el problema sin ORB geometrico.
La causa principal queda localizada en la semantica temporal y la dinamica del
`OrbPosePredictor`/publicacion, con latencia y jitter como agravantes. Siguiente:
debatir y autorizar por separado una correccion de coherencia temporal
pose/omega. No implementar aun `Delta_target`. Documentacion sincronizada;
`git diff --check` correcto y no queda ninguna ejecucion activa.

Nueva preparacion E/F/G: propuesta E mantiene la dinamica interna actual del
predictor y sustituye solo `omega_motion` por omega GT sincronizada; F publica
ultima pose GT sin extrapolacion y omega GT exacta; G propaga directamente la
ultima pose GT con omega GT, sin gates/correcciones internas del predictor.
Mismos hover, control, gains y metricas; ejecutar en orden y conservar A-D.

Acuerdo E/F/G cerrado: omega GT se sincroniza sin usar muestras futuras y se
expresa en world/O, coherente con `exp(omega*dt)*R`. E sustituye solo
`omega_motion`; F/G conservan la rama lineal del predictor y reemplazan solo
orientacion/omega para aislar el efecto angular. No tocar gains, thresholds,
`Delta_target`, estimator retardado ni etapa 3. Siguiente: editar el laboratorio
diagnostico, añadir tests/YAMLs 273-275, compilar y ejecutar E/F/G.

Checkpoint cambios: `OrbPosePredictor::OverrideAngularVelocityForDiagnostics`
permite E sin alterar defaults; el nodo diagnostico se suscribe a GT velocity
y sincroniza la ultima muestra no futura. F mantiene orientacion; G usa
`exp(omega_GT*age)*R_GT`; ambas conservan traslacion lineal del predictor.
Añadido GTest y YAMLs 273-275. Siguiente: compilar `orbslam3` y ejecutar sus
tests; despues compilar launches dependientes si procede.

Checkpoint build: `orbslam3` codigo 0, un paquete terminado en 1min02s; solo
warnings heredados de cv_bridge/ORB_SLAM3. Log completo conservado. Siguiente:
ejecutar CTest del paquete y, si pasa, iniciar prueba 273/E.

Checkpoint tests: CTest `orbslam3` 2/2 targets correctos, incluido el nuevo
override diagnostico. Siguiente accion larga: prueba 273 con
`tray_prueba_273.yaml`, modo `gt_20_exact_omega`, headless, metricas en
`metricas/prueba_273` y telemetria angular activa.

Checkpoint simulacion 273/E: runner 0, escenario 0, `success=true`; log
completo `logs/prueba_273.log` conservado sin lectura manual. Siguiente:
reducir, analizar E y despues ejecutar F sin cambios funcionales.

Diagnostico 273/E: `CONSEGUIDA`. Escenario completo, sin fallback; 1160
medidas/3478 publicaciones en 69.66 s. `er` max `0.0784 rad`, omega control max
`0.1087 rad/s`; `tau_er=+0.002225 J`, `tau_ew=-0.002291 J`, total
`-0.000066 J`. Mismatch direccional GT/control `0.41 %` con ambas normas
>0.01 rad/s. Sustituir omega derivada por GT estabiliza el predictor actual.
Siguiente accion larga: prueba 274/F modo `gt_20_exact_omega_hold`.

Checkpoint simulacion 274/F: runner 0, escenario 0, `success=true`; log
completo `logs/prueba_274.log` conservado sin lectura manual. Siguiente:
reducir/analizar F y ejecutar G sin cambios funcionales.

Diagnostico 274/F: `CONSEGUIDA`. 1161 medidas/3483 publicaciones en 69.78 s;
`er` max `0.0974 rad`, omega control max `0.0964 rad/s`;
`tau_er=+0.003074 J`, `tau_ew=-0.003150 J`, total `-0.000076 J`; mismatch
direccional `0.29 %`. El hold angular 20 Hz es estable con omega correcta.
Siguiente accion larga: prueba 275/G modo `gt_20_exact_omega_extrapolate`.

Checkpoint simulacion 275/G: runner 0, escenario 0, `success=true`; log
completo `logs/prueba_275.log` conservado sin lectura manual. Siguiente:
reducir/analizar G, construir tabla E/F/G y documentar conclusion.

Diagnostico 275/G: `CONSEGUIDA`. 1160 medidas/3481 publicaciones en 69.66 s;
`er` max `0.0960 rad`, omega control max `0.1226 rad/s`;
`tau_er=+0.003336 J`, `tau_ew=-0.003430 J`, total `-0.000093 J`; mismatch
direccional `0.094 %`, horizonte medio `0.0293 s`, sin clamp.

Conclusion E/F/G: las tres completan y son disipativas. E demuestra que solo
sustituir `omega_motion` por omega GT estabiliza el predictor actual. F prueba
que el hold angular a 20 Hz es suficiente; G valida la propagacion izquierda
SO(3) con pose/omega coherentes. Seleccion: A, derivacion/filtrado de
`omega_motion` como causa principal. Siguiente: documentar sin implementar la
solucion ORB definitiva.

Cierre E/F/G: documentacion de paquete, contrato, historial, resumen, pipeline
y estado sincronizados. Override y modos GT quedan marcados como laboratorio
retirable. Siguiente accion: debatir una estimacion de omega ORB coherente; no
hay autorizacion para implementarla ni ejecuciones activas.
