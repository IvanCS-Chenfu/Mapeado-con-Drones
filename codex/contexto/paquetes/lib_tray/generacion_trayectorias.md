# Generación de trayectorias en `lib_tray`

## Objetivo

Calcular trayectorias por eje para que `gen_tray` publique referencias suaves al controlador del dron.

Cada generador produce arrays de 5 valores por eje:

```text
[posición, velocidad, aceleración, jerk, ratio]
```

## `GenTrayPol3`

Archivo:

`src/generacion/gen_tray_pol3.cpp`

Rol:
- calcula trayectorias con polinomios cúbicos;
- usa condiciones iniciales/finales de posición y velocidad;
- permite tiempos finales por eje;
- permite objetivos absolutos o relativos según eje.

Uso típico:
- movimientos suaves de punto a punto con duración conocida.

## `GenTrayVelTrap`

Archivo:

`src/generacion/gen_tray_veltrap.cpp`

Rol:
- calcula trayectoria con perfil de velocidad trapezoidal;
- usa velocidad máxima y tiempo de aceleración;
- calcula fases de aceleración, velocidad constante y deceleración;
- exporta coeficientes para evaluación temporal.

Uso típico:
- movimientos donde se quiere limitar velocidad lineal/angular máxima.

## Waypoints

Sin modificar esas clases ni su API pública, el paquete incorpora dos
generadores en este mismo paquete:

- `GenTrayPol3Waypoints`;
- `GenTrayVelTrapWaypoints`.

No son dos paquetes ROS ni un tercer perfil geométrico. Cada uno extiende la
lógica de su familia a una secuencia ordenada de destinos. Los waypoints solo
contienen poses; no llevan velocidad, aceleración ni jerk. La implementación
deriva las velocidades intermedias a partir de la geometría y los tiempos para
atravesar los vértices sin detenerse. El estado inicial procede del dron al
aceptar la action y el estado final se detiene en el último destino.

`GenTrayPol3Waypoints` es el perfil activo de 6I. Recibe un tiempo
acumulado desde el origen por cada destino, de manera que destino y tiempo
ocupan la misma posición de sus listas. Con un único destino debe ser
equivalente, muestra a muestra, a `GenTrayPol3` legacy. Esta igualdad es un
test contractual, no una aproximación visual.

Para varios destinos compone aristas Pol3 nominales con velocidad inicial
capturada solo en el origen y velocidad final nula por arista. Cada vértice
interior es una guía, no un punto de paso obligatorio: se recorta la arista
anterior `waypoint_blend_sec` antes de la guía y la siguiente el mismo tiempo
después de iniciarse; un Pol3 de `2 * waypoint_blend_sec` conecta ambas muestras
con la misma pose y velocidad (C1). No exige continuidad de aceleración ni jerk
y el vehículo puede redondear el vértice. El origen y el último destino no se
recortan; el último permanece con velocidad nula. El calendario multi-destino
usa un tiempo común por eje y rechaza ventanas solapadas. El constructor expone
`waypoint_blend_sec=3.0` s por defecto. Un único destino conserva exactamente
los tiempos independientes por eje y la salida de `GenTrayPol3` legacy.

`GenTrayVelTrapWaypoints` conservará la semántica de `GenTrayVelTrap`: no se
le impone un tiempo final. Calcula la duración efectiva de cada tramo a partir
de posición, velocidades, `v_max` y `t_a`, y construye desde ello sus tiempos
acumulados. Por tanto, una lista temporal externa no puede alterar ese perfil.

La limpieza asociada retiró de `gen_tray` el perfil paralelo
`ExecuteMultiWaypoint`: sus quinticas por tramo, tangentes centrales, primer
punto artificial y temporización propia no forman parte de `lib_tray` ni se
mantienen como alternativa.

Tests: `test_waypoint_generators` verifica paridad Pol3 de un destino,
continuidad C1 de los empalmes, rechazo de ventanas solapadas, continuidad por
vértices VelTrap y temporización propia de VelTrap.

Para diagnóstico F6I, `GenTrayPol3Waypoints` expone de solo lectura el número,
índice temporal e inicio de sus tramos (`get_num_tramos`,
`get_indice_tramo`, `get_inicio_tramo`). No altera la curva: permite que
`gen_tray` muestree ambos lados exactos de una frontera y compare pose y
velocidad C1 sin duplicar la composición en otro paquete.

## Proxima integracion 6J/6K

El servidor necesitara samplear la misma curva nominal que ejecuta el dron para
reservar su volumen barrido. Se extraera o expondra una API pura reutilizable de
la composicion vigente de `Pol3Waypoints`; no se implementara otra curva en
`task_server`. El muestreo espacial para reservas empezara con
`reservation_sweep_sample_step_voxels=0.5`, configurable por despliegue. No se
anadiran velocidades ni aceleraciones por waypoint a `TrajectoryPlan`: la
action sucesora espera el terminal ordinario de la anterior y cada lado usa la
misma biblioteca/version de trayectorias. Esta seccion describe el contrato
pendiente, no una API disponible aun.

## `GenTrayElipse`

Archivo:

`src/generacion/gen_tray_elipse.cpp`

Rol:
- genera trayectorias elípticas alrededor de un centro o punto inicial;
- útil para rodear zonas/edificios;
- calcula posición, velocidad y derivadas en función del tiempo.
- usa radios positivos (`abs(rx)`, `abs(ry)`) y avanza `theta` de `theta0` a
  `theta0 + 2*pi`; la API actual no permite elegir el sentido de giro;
- el punto inicial debe estar sobre la elipse para evitar un salto inicial de
  referencia, porque `theta0` se proyecta sobre la curva definida.

Uso típico dentro del proyecto:
- mover drones alrededor de un edificio para mapearlo.

## Relación con Fase 1

La calidad de la trayectoria influye en:

- cantidad de KeyFrames generados;
- estabilidad de ORB-SLAM3;
- probabilidad de loops;
- cobertura del entorno.

## Reglas para modificar

- Mantener interfaz pública usada por `dron_individual`.
- Mantener intactas las interfaces y resultados de los tres perfiles legacy.
- Situar las variantes por waypoints en `lib_tray`, no en `gen_tray` ni en el
  controlador de fuerzas.
- Si se cambia formato de salida, actualizar `TrayAction` y control.
- Probar con los ejecutables de test y con una simulación corta.
