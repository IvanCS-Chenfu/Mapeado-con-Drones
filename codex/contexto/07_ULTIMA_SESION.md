# Ultima sesion

## Capitulo 5, prueba 5.6

Se anadio el diagnostico opt-in `debug_fiducial_gt_error=false` en
`StereoSlamNode`. Cuando se activa, reproduce la geometria real de la cara
AprilTag desde los YAML de simulacion, incluido `surface_offset_m`, empareja GT
sellado de `sensor/GT/pose` con el KF y escribe distancia, error de traslacion,
rotacion y skew sin modificar detector, batches, mapa ni control.

La repeticion `c5_6_apriltag_translation_gt_v3` completo con D1/GT, Gazebo,
GUI global y ventana fiducial. Produjo 12 mediciones aceptadas entre 1.188 y
3.126 m, con RMSE de traslacion 0.1001 m, RMSE de rotacion 0.2624 rad y skew
maximo 7 ms. No hubo rechazos por reproyeccion; al alejarse mas, el tag dejo
de decodificarse, manteniendo el comportamiento visual existente. La figura
de traslacion ahora representa ambos errores en dos paneles frente a distancia
GT.

Los resultados estan en `Pruebas/Capítulo 5/5_6_error_apriltag/`. El contrato
nuevo pasa aislado; el contrato fiduciario completo conserva un fallo previo
por divergencia de una trayectoria auxiliar historica. No quedan procesos ROS
2 ni Gazebo de la simulacion.

## Capitulo 5, prueba 5.6 de rotacion

El marcador opt-in `[FID-GT-ERROR]` ahora incorpora `viewing_angle_deg`, el
angulo GT entre el eje optico de la camara y el tag. Se ejecuto D1/GT con tres
giros relativos horarios de `-30 deg` y tres desplazamientos sucesivos de
`0.5 m` en `-X`, a `15 s` por tramo. La trayectoria completo los tres goals y
el giro acumulado de `-90 deg` con Gazebo, GUI global y visualizador fiducial.

La figura de rotacion contiene 29 PnP aceptados: cubren `3.002-35.675 deg`,
con RMSE de traslacion `0.0109 m`, RMSE de rotacion `0.0173 rad` y skew GT
maximo de 9 ms. Tras ese angulo el dron siguio su trayectoria, pero el tag no
se decodifico, por lo que no se inventaron mediciones hasta 90 grados. Los
artefactos quedan en `Pruebas/Capítulo 5/5_6_error_apriltag/`.

## Cierre del Capitulo 5

El usuario dio por concluidas las pruebas del capítulo. La última observación
visual añadió trayectorias de deriva alrededor del edificio y el override de
spawn X de D1, conservador por defecto. La variante inversa aparece en
`(-10,10)` y sigue los objetivos norte-oeste-sur acordados, con diagnósticos
fiduciales y Fase 6 desactivados. Terminó correctamente y no generó métricas;
la evidencia visual queda en `Pruebas/Capítulo 5/5_8_deriva/`.
