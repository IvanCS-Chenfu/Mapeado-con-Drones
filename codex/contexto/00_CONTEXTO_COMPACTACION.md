# Contexto de compactacion

## Estado operativo

- Fase activa: Fase 5, subfase 5H.
- Estado agregado: `PARCIAL`.
- Ultima ejecucion: prueba 275.
- Resultado funcional: E/273, F/274 y G/275 `CONSEGUIDAS`; diagnostico E/F/G
  `CONSEGUIDO`, opcion A (`omega_motion`).
- Trabajo activo: implementar y validar la nueva estimacion causal de
  `omega_motion` mediante la prueba 276.

## Puerta de continuidad

- Preparacion: `CERRADA` para el estimador causal de tres poses.
- Acuerdo cerrado: `si`.
- Autorizacion funcional: `CONCEDIDA` por el usuario.
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
- Dudas abiertas: ninguna.

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
