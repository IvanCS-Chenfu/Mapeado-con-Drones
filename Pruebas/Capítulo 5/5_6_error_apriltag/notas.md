# 5.6 Error visual AprilTag

## Objetivo

Medir en simulacion el error de la pose `camera_T_tag` estimada por AprilTag
frente a la geometria GT del tag en el instante del KeyFrame.

## Configuracion acordada

- Un dron y fuente de navegacion `GT`.
- `debug_fiducial_visualization=true` para la ventana de la imagen anotada.
- `debug_fiducial_gt_error=true` para registrar solo el diagnostico opt-in.
- Fase 6, mascara fisica de MapPoints y protocolo de perdida ORB desactivados.
- Trayectoria: llegada a `(0,-10,1)` con yaw absoluto `90 deg`, espera de 8 s,
  y alejamiento hasta `(0,-14.5,1)` durante 45 s conservando el yaw.

## Medida

Para cada tag con solucion PnP valida o rechazada exclusivamente por
reproyeccion, el wrapper usa la pose GT mas proxima al timestamp del KF, con
desfase maximo permitido de `0.075 s`. La geometria GT reproduce la pose de la
superficie que crea `fiducial_spawner.py`, incluido
`surface_offset_m=0.001`.

La figura representa distancia GT camara-tag frente a los errores de
traslacion y de rotacion, en dos paneles. Las detecciones aceptadas y las
rechazadas por reproyeccion se distinguen por color.

## Resultado

La repeticion `c5_6_apriltag_translation_gt_v3` completo los seis pasos del
escenario con `success=true` y codigo 0. Gazebo, la GUI global y el
visualizador fiducial estuvieron activos; la guarda de recursos no se activo.

Se registraron 12 mediciones PnP, todas aceptadas, con desfase GT maximo de
`0.007 s`. El rango muestreado fue de `1.188 m` a `3.126 m`. El error medio,
RMSE y maximo de traslacion fueron `0.0381 m`, `0.1001 m` y `0.3448 m`;
para rotacion fueron `0.1216 rad`, `0.2624 rad` y `0.7244 rad`. No hubo
rechazos por reproyeccion: al continuar el alejamiento dejaron de aparecer
tags decodificados, por lo que tampoco se produjo una imagen roja nueva. Esto
confirma la semantica existente de la ventana fiducial.

Artefactos:

- `datos_traslacion.csv`: muestras por KeyFrame con ambos errores.
- `grafica_error_traslacion.png`: errores de traslacion y rotacion frente a
  distancia GT.
- `resumen.json`: valores agregados reproducibles de ambos errores.

## Rotacion acordada

La segunda ejecucion conserva D1, GT y los mismos flags de diagnostico. Tras
llegar a `(0,-10,1,90 deg)`, ejecuta tres giros relativos horarios de `-30 deg`.
En cada uno se desplaza `0.5 m` en `-X`, con una duracion de `15 s`, hasta
acumular `-90 deg`. El diagnostico registra el angulo GT entre el eje optico y
el tag; la figura separa error de traslacion y error de rotacion frente a ese
angulo.

## Resultado de rotacion

La ejecucion `c5_6_apriltag_rotation_gt` termino con `success=true` y codigo
0. Los tres goals horarios relativos terminaron correctamente en `15 s` cada
uno, por lo que el dron completo el giro mecanico acumulado de `-90 deg`.
Gazebo, la GUI global y el visualizador fiducial estuvieron activos y la guarda
de recursos no se activo.

El diagnostico registro 29 muestras PnP, todas aceptadas, con skew GT maximo
de `0.009 s`. El angulo observable entre eje optico y tag abarco de `3.002 deg`
a `35.675 deg`; tras ese rango los KFs continuaron creandose, pero el tag dejo
de decodificarse, de modo que no hay una medida PnP hasta los `90 deg` finales.
En las muestras disponibles, el error medio/RMSE/maximo de traslacion fue
`0.0102 / 0.0109 / 0.0218 m` y el de rotacion
`0.0120 / 0.0173 / 0.0735 rad`, respectivamente. No hubo rechazos por
reproyeccion.

Artefactos:

- `datos_rotacion.csv`: muestra por KeyFrame con angulo GT y ambos errores.
- `grafica_error_rotacion.png`: errores de traslacion y rotacion frente al
  angulo de vision GT.
- `resumen_rotacion.json`: valores agregados reproducibles.
