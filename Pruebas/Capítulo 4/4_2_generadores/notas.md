# Prueba 4.2 - Perfiles de velocidad en un eje

## Configuracion

- Eje representado: `x`.
- Posicion inicial: `0.0 m`.
- Velocidad inicial y final: `0.0 m/s`.
- `v_max_lin`: `0.8 m/s`, tomado de `dron/dron_individual/config/trajectory.yaml`.
- `t_a`: `5.0 s`, tomado de `dron/dron_individual/config/trajectory.yaml`.
- Muestreo exportado: `0.02 s`.
- Generador utilizado: `lib_tray::GenTrayVelTrap` del codigo actual.
- No se utilizo Gazebo, GT, ROS ni el controlador.

## Casos

| Caso | Destino | Tiempo total | Velocidad pico | Clasificacion del generador |
| --- | ---: | ---: | ---: | --- |
| `trapezoidal_x` | `10.0 m` | `17.5 s` | `0.8 m/s` | trapezoidal |
| `triangular_x` | `1.0 m` | `5.0 s` | `0.4 m/s` | triangular |

En ambos casos la aceleracion tiene modulo `0.16 m/s^2`, la posicion final
coincide con el destino y la velocidad final es `0.0 m/s`.

## Evidencias

- `datos/trapezoidal_x.csv`
- `datos/triangular_x.csv`
- `figuras/perfil_trapezoidal_x.png`
- `figuras/perfil_triangular_x.png`
- `datos/cubica_x.csv`
- `figuras/perfil_cubico_x.png`
- `datos/waypoints_cubica.csv`
- `datos/waypoints_cubica_objetivos.csv`
- `figuras/waypoints_cubica_xy.png`
- `datos/waypoints_cubica_blend_1s.csv`
- `datos/waypoints_cubica_blend_1s_objetivos.csv`
- `figuras/waypoints_cubica_blend_1s_xy.png`
- `scripts/generar_perfiles_velocidad.cpp`
- `scripts/generar_grafica_perfiles.py`
- `scripts/generar_grafica_waypoints.py`

## Prueba cubica

- Posicion inicial: `0.0 m`.
- Destino: `10.0 m`.
- Tiempo total: `5.0 s`.
- Velocidad inicial y final: `0.0 m/s`.
- Coeficientes del eje: `(a0, a1, a2, a3) = (0, 0, 1.2, -0.16)`.
- Velocidad maxima observada: `3.0 m/s`.
- Aceleracion inicial: `2.4 m/s^2`.

El generador cubico no utiliza una meseta de velocidad ni impone directamente
`v_max_lin`; con estos datos la velocidad maxima resultante es `3.0 m/s`.

## Prueba cubica por waypoints

Se utilizo `GenTrayPol3Waypoints` con cuatro ejes internos `(x, y, z, yaw)`.
Todos los destinos mantienen `z=1.0 m` y `yaw=90 grados` (`1.57079632679 rad`).
El tiempo de blend C1 fue de `5.0 s` y los tiempos de llegada fueron
acumulados cada `12.0 s`.

| Punto | X (m) | Y (m) | Z (m) | Yaw (grados) | Tiempo (s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| Inicial | 0 | 0 | 1 | 90 | 0 |
| WP 1 | 4 | 0 | 1 | 90 | 12 |
| WP 2 | 7 | 3 | 1 | 90 | 24 |
| WP 3 | 4 | 7 | 1 | 90 | 36 |
| WP 4 | -1 | 5 | 1 | 90 | 48 |
| WP 5 | -3 | 1 | 1 | 90 | 60 |

La trayectoria resultante tiene `9` tramos internos y una duracion total de
`60 s`. La figura `waypoints_cubica_xy.png` muestra la proyeccion X-Y, la
trayectoria generada y los waypoints de destino. El blend amplio hace visibles
las curvas en los cambios de direccion. La comprobacion de las `3001` muestras
confirmo `z=1.0 m` y yaw constante de `90 grados`.

## Comparacion con blend de 1 segundo

Se mantuvieron los mismos waypoints y se repitio la trayectoria con el valor
anterior `waypoint_blend_sec=1.0 s` y tiempos de llegada `6, 12, 18, 24 y
30 s`. La duracion total es `30 s` y la figura se genero en azul para
distinguirla de la ejecucion con `blend=5 s`. En esta version las curvas son
mucho menos pronunciadas y el recorrido se aproxima mas a los segmentos rectos
entre waypoints.

## Conclusion

CONSEGUIDA. El desplazamiento de `10.0 m` produce el tramo central de velocidad
constante esperado en el perfil trapezoidal. El desplazamiento de `1.0 m`, con
los mismos limites, produce un perfil triangular sin meseta. Cada prueba tiene
su propia figura con posicion, velocidad y aceleracion del eje x. La prueba
cubica produce una posicion suave, una velocidad parabolica y una aceleracion
lineal. La prueba por waypoints genera una trayectoria cubica compuesta en el
plano XY con curvas pronunciadas cuando `blend=5 s`, mientras que con `blend=1
s` se aproxima mas a una poligonal. Ambas versiones mantienen constantes la
altura y el yaw. Los CSV conservan los datos utilizados para generar las cinco
figuras.
