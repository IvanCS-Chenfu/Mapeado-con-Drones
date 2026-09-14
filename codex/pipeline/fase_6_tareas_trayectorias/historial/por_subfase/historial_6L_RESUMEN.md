# Resumen 6L

Estado agregado: CONSEGUIDA para el alcance acordado. `TRACKING_RISK`
usa inliers ORB de las franjas LEFT `[0,0.75W]`, RIGHT `[0.25W,W]`, TOP
`[0,0.75H]` y BOTTOM `[0.25H,H]`: una franja dirigida sin inliers durante tres
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
