# Contexto minimo actual

Precondicion: leer fisicamente `00_CONTEXTO_COMPACTACION.md` antes de este
archivo y reconciliarlo con la peticion mas reciente.

## Estado

```text
Fase 2: CONSEGUIDA el 2026-08-24
Fase 3: cierre previo conseguido; reabierta únicamente en 3Q
Fase 4: CONSEGUIDA Y CERRADA con alcance 4A-4H
Fase 5: funcionalmente CONSEGUIDA; 5H cerrada por evidencia visual y ORB 3/3
4A: CONSEGUIDA
4B: CONSEGUIDA
4C: CONSEGUIDA
4D: CONSEGUIDA; prueba 208 aceptada por el usuario
4E: CONSEGUIDA
4F: CONSEGUIDA
4G: CONSEGUIDA
4H: CONSEGUIDA
4I: APLAZADA como regresion opcional futura
Subfase actual: 5H CONSEGUIDA; deuda de observabilidad transferida a Fase 6
Preparacion 5H: cerrada; autorización consumida y batería final completada
Siguiente punto de entrada: diagnosticar velocidad residual Y sin cambiar gains
Punto de entrada siguiente: preparar Fase 6 y retirar progresivamente GT fallback
Revision visual humana de prueba 200: confirmada correcta
Cierre de Fase 2: completo
```

Pruebas 318R2/319R: la poda conserva una predecesora ZOH y todas las muestras
recientes; pasan 102/102 GTests, 8/8 tests del analizador y ambas simulaciones
sin missing ni fallback. Poda, cobertura y paridad dinamica validadas.

Prueba 320 fue invalida porque `navigation_state.yaml` sobrescribia el modo
`dynamic`; corregida mecanicamente la precedencia del launch. En 320R ambos
stereo usan `dynamic` y no hay missing, pero ORB gobierna ya la aproximacion y
el segundo goal arranca cerca del suelo con velocidad alta. El tracking se
pierde brevemente despues, cuando el error ya existia. 320R `NO CONSEGUIDA`;
321 no ejecutada y ORB productivo no validado.

Prueba shadow: 320R2 fue invalida por ruta YAML relativa. 320R2R confirma
aproximacion GT, ORB dinamico en sombra y handoff limpio tras `1.5 s`
estacionario. Pose/rotacion saltan cero, pero velocidad `0.247 m/s`; tracking
permanece `2` y no hay fallback ni missing. El hover diverge hasta `~1.63 m`
de error de posicion y `~0.52 rad` angular. Activacion prematura descartada
como causa suficiente.

Bateria 321: autoridad ORB confirmada antes del goal. 321B
(`p_ORB+v_GT+angular_ORB`) cumple los umbrales y no usa fallback; 321AR y 321D
divergen al usar `v_ORB`. 321C cae a fallback y no aisla el angular. La
velocidad lineal ORB queda demostrada como causa principal; ORB completo sigue
sin validar y 5H permanece `PARCIAL`.

Diagnostico 322/323: autoridad GT y ORB `dynamic` en shadow durante unos 43 s
settled. Sobre 907 medidas, RMSE `v_mid=0.01984`, TWO_SAMPLE en `t_k=0.01988`,
THREE_SAMPLE `v_hat_tk=0.03457` y salida dinamica now `0.43308 m/s`. Quedan
demostradas `A_HAT_AMPLIFICATION` y `DYNAMIC_PROPAGATION`; salida productiva
intacta y STOP antes de corregir.

Pruebas 324/325: se corrige exclusivamente el frame de gravedad mediante
`g_O=O_R_W*g_W`, congelada desde la primera autoridad global del epoch.
`v_dynamic_now` baja reproduciblemente de `0.43308` a
`0.03583/0.03707 m/s`, con gain aproximadamente uno y residual de aceleracion
pequeno. `DYNAMIC_PROPAGATION CORREGIDA`; THREE_SAMPLE sigue pendiente.

Pruebas 326-329: `MIDPOINT_DYNAMIC` reconstruye la velocidad en `t_k` desde
dos posiciones aceptadas, interpola R en el midpoint y propaga con thrust,
torque, gravedad O y buffers causales. En shadow obtiene cobertura 100 % y
mejora claramente a THREE_SAMPLE; 328/329 validan su uso productivo con RMSE
`0.02113/0.02460 m/s`. THREE_SAMPLE queda solo como diagnostico.

Pruebas 330/331: ORB completo gobierna un nuevo goal hover durante
`34.78/35.30 s`, sin fallback, tracking no-OK ni clamp. Los maximos de error
angular son `0.0674/0.0631 rad` y la energia angular total es negativa en las
dos ejecuciones. `A_HAT_AMPLIFICATION CORREGIDA`, estimador lineal y hover ORB
real validados de forma reproducible. 5H sigue `PARCIAL` hasta validar
movimientos y la trayectoria representativa.

Pruebas 332-334: X 2 m queda validado y reproducido con fallback cero. 334 es
invalida: desde `[0,-10,1]` avanza en +Y atravesando el fiducial 2 de
`[0,-8.5,1]`. El control permanece acotado hasta la colision y la posterior
perdida de tracking; no aporta evidencia contra Y. 335-343 no ejecutadas.

334R elimina la colision y mantiene ORB/tracking/fallback cero, pero falla el
frenado: ev final `0.187 m/s` y RMSE ev de los ultimos 3 s `0.174 m/s`, frente
a GT `0.049 m/s`. Y queda no validado por velocidad ORB residual/oscilante;
STOP mantiene 335-343 sin ejecutar.

334R2 visual reproduce y agrava el fallo con tracking sano: ep/ev final
`0.213/0.551`, max ev `2.188 m/s` y RMSE ev final `1.008 m/s`.

Integracion post-317: `StereoSlamNode` incorpora una rama temporal `dynamic`
y conserva `legacy` por defecto. Builds y tests pasan. La prueba 318 completa
el escenario, pero registra un hueco de torque porque la unica muestra del
buffer era posterior a la base. 318 queda `NO CONSEGUIDA`; 319/320 no se
ejecutan por STOP y ORB real sigue pendiente.

Prueba 278: pose GT a 20 Hz con delay fijo de 80 ms no completa el hover.
Edad visual media `0.1129 s`, clamp `72.2 %`, RMSE angular `1.447 rad/s` y
trabajo total `+0.02884 J`; sin fallback ni tracking loss. La bateria se
detiene y 279-281 no se ejecutan. Falta debatir `t_k -> now`.

Pruebas 282/284: `0.18 s` elimina el clamp pero empeora; propagar con
`alpha_hat` mejora RMSE/energia, aunque falla antes y raw se rechaza desde
`0.84 s`. El flag alpha queda apagado en produccion. 279-281 siguen detenidas.

Pruebas 285-287: con omega GT se gobiernan unos `13.1 s`; con omega predicha
solo `2.98 s` y energia positiva. Señala omega, pero 287 tambien falla porque
p/v lineales siguen retrasadas. Diagnostico `PARCIAL`; el siguiente aislamiento
debe fijar p/v GT actuales. 279-281 siguen detenidas.

Pruebas 288-291: con p/v GT comunes, 288 y 290 fallan cuando usan omega
predicha; 289 y 291 completan al usar omega GT, incluso conservando R predicha
en 289. El sanity GT completo pasa. La causa inmediata queda aislada en
`omega_pred(now)` bajo delay. Falta una correccion sin GT y validacion ORB real;
279-281 siguen detenidas.

## Arquitectura vigente

```text
src/dron/       -> ORB-SLAM3, wrapper, control, trayectorias e interfaces
src/servidor/   -> backend y servidor de mapa global e interfaces canonicas
src/simulacion/ -> Gazebo, escenarios, integracion y visualizadores
```

Los builds usan bases separadas `build/install/log/{dron,servidor,simulacion}`
y exactamente un paquete por invocacion. `orbslam3_msgs` es canonico en
Servidor y replica exacta en Dron. `mi_tfg` permanece como legacy fuera de los
tres grupos.

## Configuracion y debug

ADR 0009 gobierna ownership y replicas YAML. ADR 0010 exige coste especifico
practicamente nulo cuando `pipeline_flow` o `system_architecture` estan
desactivados. Los siete flags de debug quedan en `false` por defecto.

`pipeline_flow` muestra el flujo interno sparse/global.
`system_architecture` muestra paquetes, grupos e interfaces y recibe actividad
ligera por `/system_architecture/activity`.

## Evidencia de Fase 2

- nueve builds aislados correctos, uno por invocacion;
- CTest: 4/4, 9/9, 10/10 y 9/9 en suites funcionales;
- prueba 199: 5/5 pasos, 4/4 goals y debug especifico dormido;
- prueba 200: 14/14 pasos, 20/20 goals, RViz2 y ambos web activos;
- ambos visualizadores validados por separado; modo live con evento ROS real;
- layout final validado por CTest y capturas desktop/viewport estrecho;
- guardas de layout, interfaces, dependencias, config, paths y visualizers pasan.

La prueba 200 conserva dos incidencias de cleanup posteriores a `SIM-DONE`:
traceback de `gui_tray_multi` y Gazebo 255. Los bridges, RViz2, wrappers y
servidor cerraron limpiamente.

## Lectura siguiente

```text
codex/pipeline/fase_2_separacion_paquetes/RESULTADO_FINAL_FASE_2.md
codex/pipeline/fase_2_separacion_paquetes/historial/INDEX.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/contexto/05_MAPA_PAQUETES.md
```

Nota: Fase 4 fue reconciliada documentalmente desde
`Fase_4_completa_4A_4I_muy_detallada.zip`. Las antiguas 4J-4L quedan legacy,
no ejecutables como subfases activas.

Pruebas 201/202: contrato fiducial y spawn correctos. 4C+4D quedan conseguidas:
la prueba 208 completo la trayectoria con deteccion real, visualizadores
separados, cierres por timeout y wrappers publicando deltas posteriormente.
Los fallos 203-207 se conservan en el historial 4D.

Pruebas 210/211: 68/68 matches en la trayectoria tipica completa y 18/18 en
el smoke con ambos grafos live. Pending pico 7/10, sin expulsiones, duplicados,
conflictos ni rechazos. 4E+4F quedan conseguidas.

Pruebas 214-217: interpretacion visual robusta, visitas fuera de orden,
handoff al manager y retirada total de GT fiducial conseguidos. La 216 completa
la trayectoria sin GT con 52/52 primary y tres objetos; la 217 valida ambos
grafos live. El GT de control/Fase 5 permanece independiente.

Fase 5A reconcilia el pipeline con la arquitectura `O_T_B` continua y
`W_T_B` global corregible. Los goals absolutos sin `W_T_O` se rechazan,
reference KF real + `Tcr` sustituyen nearest-KF y smoothing no es obligatorio.
`GT_FALLBACK` se mantiene temporalmente durante Fase 5 para completar misiones
ante `RECENTLY_LOST`, aislado de mapa/global y con retirada obligatoria en Fase
6. 5B queda conseguida: estado local coherente, `O_T_B` intra-epoch y gate.

Prueba 225: ambos submapas reciben anchor hard, los cambios de reference KF no
introducen salto, el absoluto sin global se rechaza y los relativos congelan
epoch/muestra. Tras girar 180 grados ambos ORB pasan 2->3->0->1 con local y
continuidad inválidas. Siguiente bloque: 5C+5D+5E+5F.

Bloque 5C-5F ejecutado: consulta backend, servicio/push dirigido y composicion
O/W pasan builds y tests. La prueba 230 termina `success=true` con anchors,
loops y revisiones naturales. 5F queda PARCIAL. Por acuerdo posterior, la
diferencia GT-pose estimada no se usa para decidir fuente ni validez debido a
la deriva acumulada.

Bloque 5G-5H ejecutado: mux ORB/`GT_FALLBACK`, velocidad comun, goals absolutos
y ejes RViz2 desde la `O_T_B` exacta del controlador. La fuente se congela por
goal; solo la perdida ORB permite pasar inmediatamente a GT, que se mantiene
hasta terminarlo. Tras corregir el handshake detectado en 242, la prueba 243
completa 17/17 pasos, 22/22 goals y 44/44 handshakes, sin ningun `GT -> ORB`
dentro de goals. 249 descarta un impulso angular y 250/251 localizan la
extrinseca inversa. La 252 corrige `B_T_C` y completa 17/17 pasos y 22/22 goals.
Quedan tirones por derivadas ORB sin filtrar (20 Hz frente a control 50 Hz) y
movimientos bruscos cuando una perdida hace `ResetToSource` hacia GT dentro de
una trayectoria congelada en O. Estado PARCIAL.

La prueba 254 deja GT exacto y publica ORB predicho a 50 Hz desde el wrapper.
Los handoffs empiezan con salto cero, pero la vuelta se interrumpe tras 13/17
pasos: en giros, la pose acepta saltos ORB de hasta unos `0.28 rad/frame`
mientras la velocidad angular queda limitada. El estado angular incoherente
desestabiliza el control y precede las perdidas de tracking.

La prueba 255 pone references KFs en probation y rechaza outliers angulares de
forma coherente. Build y 13/13 tests finales correctos. Antes de interrumpirse
completa 7/17 pasos; evita publicar los saltos graves, pero registra 10 timeouts
y pasa a GT frecuentemente. La revision visual confirma inestabilidad tras unos
segundos en ORB. La cronologia descarta los commits globales: los tres episodios
siguen churn de referencias/outliers locales y dos suceden sin optimizacion
activa ni reciente.

La prueba 256 sustituye la estabilidad de ID por probation geometrica multi-KF
y usa correccion SE(3) gradual; build y 15/15 GTests pasan. La simulacion falla:
el cambio GT->ORB es continuo y empieza con error angular cero, pero drone2
acepta una innovacion de `0.125261 rad`, registra
`rotation_step_rad=0.119002` y pierde tracking `0.793 s` despues. Sin
optimizacion W concurrente, queda una hipotesis local fuerte, pero ese campo no
es el paso publicado. Falta medir correccion aplicada, pose/omega, error/torque
y tracking para establecer causalidad; la siguiente iteracion incorpora
confirmacion temporal e instrumentacion.

La nueva probation pasa 21/21 GTests. La etapa 1 de prueba 258 consigue 11/11
pasos con GT gobernando y observa perdida ORB solo durante yaw rapido. La etapa
2 de prueba 259 falla: el hover ORB dura 227 muestras, la oscilacion aparece
antes del primer pending con un paso publicado de `0.058777 rad`, y despues la
confirmacion moderada llega a `0.075 rad/paso`, outliers, fallback y tracking 3.
Por el criterio acordado no se ejecutan etapas 3-8. El siguiente debate debe
evitar que SMALL crezca con gaps y basar confirmacion en evidencia independiente
del residual realimentado.

La correccion 262 añade deadband/histeresis y confirmacion del bias, supresion
por movimiento y decay de raw rechazado. Pasa 37/37 GTests. En hover,
`omega_bias=0` y el decay funciona, pero `omega_motion` oscila hasta
~`0.617 rad/s`; ORB dura ~`5.92 s` y el fallback precede ~`0.54 s` a tracking
2->3. No se ejecuta etapa 3. Siguiente diagnostico: timestamps y fase de
`omega_raw -> omega_motion -> control omega -> ew -> torque`, sin tocar GT,
mux, ganancias ni W.

La prueba 263 ejecuta el mismo hover y captura ORB/control, pero queda
`DATOS_INSUFICIENTES`: GT usa header Gazebo y control reloj ROS, así que no se
calculan correlaciones ni potencias falsas. `gt_receive_stamp` y el analizador
dual-clock ya están implementados, probados y compilados.

La prueba 264 usa ese puente y sincroniza 323 ciclos ORB durante `6.44 s`.
Raw sigue al GT con correlacion `~0.98` y lag `~0.08 s`; tracking permanece en
2 hasta fallback. El termino `tau_ew` es contrario al torque ideal en `34.0 %`
de los ciclos, aunque netamente disipativo. `tau_er` es la evidencia dominante:
inyecta `+0.005173 J` y hace trabajo positivo en `80.9 %` del tramo
post-handoff. Diagnostico temporal conseguido, hover no conseguido. No hay
cambio funcional ni etapa 3; el siguiente acuerdo debe corregir la fase de la
pose angular usada por control.

La prueba 265 corrige el horizonte saturado: usa edad local media `51.5 ms`,
horizonte medio `43.2 ms`, clamp `10.4 %` y una unica propagacion. Pasa 40/40
GTests y 5/5 tests del analizador, pero el hover queda `NO CONSEGUIDO`: ORB dura
`5.56 s`, `tau_er` inyecta `+0.160266 J` y el torque total `+0.145081 J`.
`visual_q -> base_q` alcanza `0.339 rad`, mientras la prediccion añade poco;
el desfase dominante vive en `pose_` base integrada. 5H sigue `PARCIAL`, sin
etapa 3. La autorizacion anterior no cubre la siguiente fusion/anclaje visual.

La prueba 266 aplica ese reanclaje: SMALL/plausible deja error after cero y la
correccion de pose no se convierte en omega. Pasa 44/44 GTests y 7/7 tests del
analizador. ORB mejora de `5.56` a `8.06 s`; en ventana comun `tau_er` cae de
`+0.153559` a `+0.002067 J` y el torque total pasa a `-0.001945 J`. No completa
el hover: MODERATE_CONFIRMED limitado a `0.015 rad` deja residual, aparecen
tramos PREDICT_ONLY, raw se rechaza y llega fallback antes de tracking 3. 5H
sigue `PARCIAL`, sin etapa 3; siguiente debate limitado a politica moderate.

La prueba 267 aplica anclaje moderate completo y vuelve a fallar a `5.56 s`.
Tras el primer anclaje la energia dañina crece mas deprisa que en 266. Un
confirmed anchor uso raw rechazado; el codigo final exige raw plausible y pasa
46/46 GTests, pero no se ha simulado. No ejecutar 268 ni etapa 3 sin acuerdo.

La prueba 268 valida ya ese gate: el unico anclaje moderate es plausible y
deja error after cero, pero acumula `+0.046416 J` hasta fallback en `0.88 s`.
ORB dura `5.72 s`; no se ejecutan 269 ni etapa 3. El siguiente diseño propuesto
es `Delta_target` propagable a `0.30 rad/s`, aun no autorizado para codigo.

Repeticion visual 212: seis yaw relativos aplicados y compilados, pero no
alcanzados. Un `LoopTask` fue rechazado por
`commit_pose_store_hard_constraint_violation`; su clasificacion hard fijo
`blocking_failure=true` y el mission gate impidio enviar el paso 5. Repeticion
suspendida hasta acordar correccion o prueba visual sin gate.

Correccion posterior autorizada: eliminado `secondary_blocking_failure_`; los
fallos siguen siendo observables pero no enclavan el gate. Build y CTest del
servidor correctos. La prueba 213 completa 17/17 pasos y 22/22 goals, libera el
backpressure, registra 74/74 PUB/SHOW fiduciales y termina con exit 0. El usuario
da 4A-4F por concluidas, pero observa derivas no corregidas; 213 queda marcada
para revisarla de nuevo en 3Q.

Correccion 3Q posterior: retirada la deadband de 2 cm, ventana segmentada comun,
apoyo 2/4/6, revisitados 5 m/20 grados y consenso temporal 3/60. Builds y CTests
9/9, 12/12 y 10/10. Prueba 219: 17/17 pasos, 22/22 goals y 22 commits loop de
30 solves; cinco fusiones post-opt, cero movimiento hard y cero anchors loop.
La revision visual confirma correccion completa de una zona y parcial en las
esquinas derechas multi-epoch. Queda carga de reruns y una referencia de
esquina por aclarar antes del cierre definitivo.
