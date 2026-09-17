# Resumen 6L

Estado agregado: CONSEGUIDA para deteccion preventiva, STOP y correccion local
en los escenarios ensayados. La captura depth posterior pertenece a 6N.
`TRACKING_RISK`
usa inliers ORB de las franjas LEFT `[0,0.65W]`, RIGHT `[0.35W,W]`, TOP
`[0,0.65H]` y BOTTOM `[0.35H,H]`: una franja dirigida sin inliers durante tres
frames ejecuta STOP local y el dron ejecuta despues la reorientacion a XYZ fijo.
El salto configurado es inicialmente 25 grados, aplicable a yaw o
`camera_pitch` segun el sector. El servidor solo limpia ruta/reserva, mantiene
HOLD, persiste el terminal local y reencola; no sirve `REORIENT_FOR_TRACKING`.
La 696 valida la cadena previa de STOP, precaucion y reorientacion;
697 y 698/699 con D1/GT, GUI F7 y Gazebo confirman que el nuevo umbral no causa
falsos positivos en sus recorridos, pero no generan una franja vacia real.
698/699 validan además `PlanRoute.dispatch_execution` por la cadena productiva,
sin cambio de epoch; sus STOPs fueron de corredor ocupado. La 719 prolonga
solo el volumen vertical de prueba y activa TOP/BOTTOM real en el paso 13: el
protocolo ordena pitch `-25 grados` sin cambio de epoch, pero la reorientacion
es reemplazada por STOP al cambiar su corredor a ocupado/inflado. Pendiente:
su terminal normal, no la deteccion ni el transporte de pitch.
Las pruebas 700/701 corrigen y validan la secuencia de escenarios: cada paso
espera el terminal de la ruta previa y un STOP de corredor solo permite
continuar tras `STOP completado`. Ambos recorridos ordinarios conservaron la
orientacion inicial. La 717 completa el nuevo caso horizontal real: con
D1/GT y epoch 0, una franja vacia (`sector=2`, fraccion 0.75, inliers cero)
activa STOP y `reorient_1_19`; yaw pasa de 90 a 115 grados y el runner recibe
su terminal. La 718 completa doce ascensos Z+ con STOPs de corredor, pero no
activa TOP/BOTTOM. La 719 si lo activa y revela que ese giro no debe estar
sujeto al corredor del servidor. La 720 no es evidencia funcional por
suspension del ordenador. La 721 repite limpia el ascenso vertical: en
`dstar_1_18`, sector 4 vacio activa STOP y el giro local de pitch `-25 grados`;
el terminal exitoso llega al runner y el servidor registra
`F6M-ORIENTATION-UPDATED local=true`, sin `reorient_*` ni reserva asociada y
con epoch estable. Los ROI ampliados usados por 717--721 son YAML aislados de
pruebas y conservan los vetos D*.
`VISUAL_RETREAT`, veto de rutas por precauciones y calibracion fuera del caso
controlado son mejoras futuras.

La 722 termina `success=true` y la revision visual del usuario confirma el
movimiento fisico del joint al ordenar `pitch=-25 grados` con XYZ fijo.

Las pruebas 741--743 validan que el riesgo cancela el giro de inspeccion y que
el STOP se ejecuta, pero revelan una recuperacion encadenada incorrecta. En 743,
ya corregido el arco yaw largo de 742, el target termina `success=false`; tras
el STOP se lanza una restauracion de `-83.591 grados`, esta activa otro riesgo y
otro STOP, y despues se ejecuta una correccion local de `+25 grados`. ORB habia
entrado en `RECENTLY_LOST` aproximadamente 1.2 s despues del primer riesgo. La
deteccion y el STOP quedan probados, pero falta acordar y validar una unica
maniobra de recuperacion que no encadene giros cuando el tracking ya se degrado.

La 744 limita el giro a `5 deg/s` y no presenta `RECENTLY_LOST` ni `LOST`.
Visualmente el usuario confirma un movimiento estable. Aparecen riesgos LEFT
posteriores, pero ORB permanece en estado OK; por tanto, la velocidad de
inspeccion queda conseguida y el bloqueo restante pertenece a la captura e
integracion depth de 6N.

La comparacion 745/746 valida la anticipacion vigente. Con 0.75, 745 alcanzo
`RECENTLY_LOST` y cambio de epoch. Con la unica variacion a 0.65, 746 detecto
LEFT vacio en el frame 1338, cancelo el giro objetivo, completo STOP y una
correccion `-25 deg`, y mantuvo ORB en estado 2 sin cambio de epoch. La prueba
se detuvo por el bucle de confianza depth, no por tracking. Por tanto, 6L queda
CONSEGUIDA para este escenario; calibracion amplia y `VISUAL_RETREAT` siguen
aplazados.
