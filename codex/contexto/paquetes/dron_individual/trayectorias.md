# Trayectorias en `dron_individual`

## Relación con `lib_tray`

`dron_individual` no calcula directamente todas las ecuaciones de trayectoria. Usa `lib_tray`:

- `GenTrayPol3`
- `GenTrayVelTrap`
- `GenTrayElipse`
- `GenTrayPol3Waypoints`
- `GenTrayVelTrapWaypoints`

`gen_tray.cpp` actúa como action server ROS 2 que envuelve estas clases.

`generar_dron.launch.py` acepta `trajectory_config` para seleccionar el YAML de
parámetros de trayectoria instalado en `config/`. Esto permite variar
`crear.tray.v_max_lin`, `crear.tray.v_max_ang` y `crear.tray.t_a` por ejecución;
el valor nominal es `trajectory.yaml`.

## Action `TrayAction`

Entrada:

- `tipo_trayectoria`:
  - `0`: polinomio cúbico;
  - `1`: velocidad trapezoidal;
  - `2`: elipse.
- `target_pose`.
- tiempos o parámetros `tx`, `ty`, `tz`, `tyaw`.
- flags `absoluto_x/y/z/yaw`.

Feedback:

Por cada eje `x`, `y`, `z`, `yaw` se publica un `Float64MultiArray` de 5 valores:

```text
[posición, velocidad, aceleración, jerk, ratio]
```

Resultado:

- `success`;
- `t_total`.
- `reason`.
- `reason`.

## Uso actual en simulación

La GUI `gui_tray_multi.py` del paquete `simulacion_dron` envía goals a:

```text
/dron_X/AccionTrayectoria
```

Los nodos de control consumen el feedback y mueven físicamente el dron en Gazebo.

## Recomendación para automatizar pruebas

Crear scripts que llamen a la action `TrayAction` directamente, en lugar de depender de la GUI. El script debe parametrizar:

- namespace del dron;
- tipo de trayectoria;
- objetivo XYZ/Yaw;
- flags absoluto/relativo;
- tiempos/velocidades.

Esto permitirá a Codex ejecutar escenarios reproducibles para cada fase.

## Ejecución por waypoints de 6I

`ExecuteMultiWaypoint` fue retirado de `gen_tray`: no quedan quinticas por
tramo, tangentes centrales, primer punto artificial ni temporizador paralelo.

La ruta activa usa `Pol3Waypoints` de `lib_tray` como
perfil físico activo. `TrayAction` mantendrá los modos legacy y transportará
una lista ordenada de destinos W junto con tiempos acumulados, uno por destino.
No se envía un waypoint inicial: `gen_tray` toma el `NavigationState` canónico
al aceptar la action, lo convierte una sola vez mediante `O_T_W` y lo emplea
como origen real. `waypoint_blend_sec=3.0` s llega como parámetro ROS al
generador y se entrega a `GenTrayPol3Waypoints`. Para varios destinos, la
biblioteca redondea cada guía interior con un Pol3 C1 de dos veces esa ventana;
no obliga a cruzar la guía ni exige continuidad de aceleración o jerk. Los
puntos intermedios no son detenciones y el último termina en reposo. Un único
destino conserva el perfil `GenTrayPol3` legacy muestra a muestra.

D* aporta únicamente XYZ. Los waypoints intermedios conservan yaw y pitch de
la pose de entrada; el último puede llevar yaw objetivo cuando la tarea lo
proporcione. El giro y `camera_pitch` para observación pertenecen a 6M. Un goal
directo con una sola pose debe recorrer exactamente el mismo perfil que el
`GenTrayPol3` de siempre, pero a través de la nueva ruta de ejecución.

`task_server` crea el `TrajectoryPlan` y, desde esa misma decisión, lo
despacha al dron y lo publica al GUI con el mismo `trajectory_id`. Los estados
`planned`, `active`, `completed` y `canceled` actualizan ese mismo plan, sin
crear una geometría distinta para el movimiento.

La conversión de yaw world a control conserva una representación angular
continua: cada destino usa el equivalente más cercano al yaw inicial o al
waypoint anterior. No se normaliza aisladamente cada destino a `[-pi,pi]`
antes de construir el Pol3, porque eso convertiría un cruce corto de la frontera
angular en un giro de casi una vuelta completa.

La integración física de cambios de corredor sigue en diagnóstico: la prueba
671 observó cascadas STOP/replan y no acredita aún que la ruta llegue a
`ExecutePol3Waypoints` antes de una degradación dura.

`gen_tray` centraliza ahora los terminales en `FinalizeGoal`: antes de publicar
`succeed`, `abort` o `canceled` comprueba que el handle siga activo y captura
la excepcion de una retirada concurrente. El worker conserva la regla de que
solo su propio hilo finaliza su goal; el guard evita que el apagado de ROS o una
preempcion ya procesada termine el proceso por un segundo resultado.

Para los goals trapezoidales absolutos, `gen_tray` recibe también estados
iniciales con pequeñas velocidades residuales al enlazar waypoints. La
generación en `lib_tray` descarta las componentes contrarias al desplazamiento
o incompatibles con `v_max` antes de calcular las fases del perfil; así el
action server puede completar el siguiente goal sin quedar bloqueado por
tiempos de aceleración negativos.

## STOP interno de 6I

`TrayAction.stop_at_current_pose` es una orden interna, no una orden de misión
expuesta a GUI. Ante un conflicto, `task_server` la envía al dron; al recibirla,
`gen_tray` captura el `NavigationState` canónico, usa pose y yaw actuales como
destino, conserva la velocidad inicial y calcula un Pol3 de frenado durante
`stop_duration_sec` (5.0 s en `trajectory.yaml`). El último feedback fija esa
misma pose con derivadas lineales y angulares nulas.

STOP no devuelve `stop_completed` ni abre un lifecycle paralelo: la trayectoria
local termina su action igual que una ruta normal. El servidor conoce que el
comando activo era STOP y, al recibir su terminal normal con éxito, calcula y
envía la siguiente ruta. Por tanto el controlador nunca conserva un setpoint de
movimiento sin relevo y la transición no depende de que el servidor estime la
pose instantánea del dron.

## Diagnóstico de Fronteras 6I

El goal interno de `TrayAction` transporta `trajectory_id` y su feedback añade
ese identificador junto a `diagnostic_piece_index`; son campos de observación,
no entradas al perfil. Con `debug_f6i_trajectory=true`, `gen_tray` registra
una muestra al inicio de cada pieza y, en fronteras interiores, posición y
velocidad a ambos lados exactos. `control_calcular_fuerzas` registra una vez la
misma identidad/pieza con referencia recibida y pose/velocidad canónicas
actuales. Así se puede separar una curva C1 correcta de un desfase entre action
y control sin inundar los logs a 30 Hz.

## Protocolo secuencial legacy usado en `12R-D4`

Esta sección describe un escenario histórico útil para reproducir deriva en el
servidor monolítico. No es la planificación activa de Fase 3; las pruebas
activas deben venir del archivo `subfase_3T.md` correspondiente.

Para observar deriva y loops no basta con cambiar una vez el YAML. Debe poder
repetirse el mismo escenario N veces y clasificar cada ejecucion con logs + RViz2.

En ese escenario legacy, mover cada dron solo cuando el otro haya terminado su movimiento.
Escenario base recomendado:

1. mover `dron_1` al fiducial 2 con yaw `90 deg`;
2. mover `dron_2` encima de `dron_1`, a `0.3 m` por encima;
3. mover `dron_1` `6 m` en `-x`;
4. mover `dron_2` encima de `dron_1`;
5. mover `dron_1` de nuevo al fiducial;
6. mover `dron_2` encima de `dron_1`.

La clasificacion no debe depender del nombre `tray_prueba_1.yaml` o
`tray_prueba_2.yaml`, sino de la evidencia:

- ejecucion limpia: RViz2 sin deriva clara y logs sin loops destructivos;
- ejecucion con deriva: RViz2 con deriva/ruido/paredes duplicadas y logs con
  loops, decisiones de subnube o tareas/contadores de optimizacion.

Tras cada prueba, Codex debe dar una conclusion preliminar de logs y esperar la
observacion RViz2 del usuario para cerrar la conclusion final.
## Waypoints de orientacion 6M

`TrayAction` acepta yaw y `camera_pitch` por waypoint. `gen_tray` conserva la
trayectoria Pol3 de XYZ/yaw y publica el destino final de pitch como
`camera_pitch/command` con joint `stereo_pitch_joint`. Los marcadores
`F6M-WAYPOINT-CONTRACT` y `F6M-CAMERA-PITCH` son `WARN` para permanecer
visibles con el nivel normal de launch. Localizacion:
`dron/dron_individual/src/control_tray/gen_tray.cpp`, patrones
`ExecutePol3Waypoints|PublishCameraPitch`.
