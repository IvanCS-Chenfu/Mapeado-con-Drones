# Resumen 6M

Actualizacion 2026-09-13: la conclusion se limita al transporte fisico. Las
rutas ordinarias ya no eligen pared, normal, distancia ni direccion de avance:
conservan XYZ y orientacion neutra/de entrada. 698/699 confirmaron pitch neutro
en las rutas D*; una correccion visual real de +/-25 grados sigue pendiente de
la evidencia activadora de 6L. Depth/normales quedan para 6N.

Estado agregado: CONSEGUIDA con el alcance acordado. La cadena
`TrajectoryPlan -> ExecuteTrajectory -> TrayAction -> gen_tray ->
camera_pitch/command` es reproducible y fue validada en 693/696, incluida una
reorientacion que evita el primer sector visual pobre y termina normalmente.
La calibracion geometrica amplia y depth 6N quedan como mejoras futuras.

Actualizacion 700/701: el servidor persiste yaw/pitch ordinarios por dron y
epoch para que el cero por defecto de un waypoint no ordene yaw cero. Tres
tramos X+ y tres Z+ bajo GT conservaron yaw cercano a 90 grados y pitch cero;
los STOPs de corredor se confirmaron antes de continuar. La reorientacion de
25 grados sigue siendo una excepcion 6L pendiente de escenario activador.
