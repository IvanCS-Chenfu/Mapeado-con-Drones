# Contexto minimo actual

Precondicion: leer fisicamente `00_CONTEXTO_COMPACTACION.md` antes de este
archivo y reconciliarlo con la peticion mas reciente.

## Estado

```text
Fase 2: CONSEGUIDA el 2026-08-24
Fase 3: cierre previo conseguido; reabierta únicamente en 3Q
Fase 4: CONSEGUIDA Y CERRADA con alcance 4A-4H
Fase 5: 5A-5E CONSEGUIDAS; 5F PARCIAL; 5G-5H PARCIAL tras pruebas 273-275
4A: CONSEGUIDA
4B: CONSEGUIDA
4C: CONSEGUIDA
4D: CONSEGUIDA; prueba 208 aceptada por el usuario
4E: CONSEGUIDA
4F: CONSEGUIDA
4G: CONSEGUIDA
4H: CONSEGUIDA
4I: APLAZADA como regresion opcional futura
Subfase actual: 5H PARCIAL; causa principal aislada en `omega_motion`
Preparacion 5H: cerrada; autorizacion E/F/G consumida
Siguiente punto de entrada: corregir derivacion/filtrado de omega ORB
Punto de entrada Fase 5: evitar inyeccion de energia por `tau_er` sin GT
Revision visual humana de prueba 200: confirmada correcta
Cierre de Fase 2: completo
```

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
