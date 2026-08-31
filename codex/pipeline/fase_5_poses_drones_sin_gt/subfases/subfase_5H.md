# Subfase 5H - Integracion de NavigationState con trayectoria y control

## Estado

`CONSEGUIDA`. Esta subfase conserva exclusivamente el alcance de integracion
original. La estabilizacion posterior pertenece a 5I.

## Objetivo

Hacer que `gen_tray` y el controlador consuman una unica interfaz
`orbslam3_msgs/msg/NavigationState`, sin consultar GT directamente, manteniendo
separados el frame local continuo O y el frame global corregible W.

## Contrato

- `orbslam3_ros2` publica el estado ORB en O y, cuando existe anchor, su pose W.
- `navigation_state_mux` selecciona ORB o el `GT_FALLBACK` temporal de Fase 5.
- `gen_tray` acepta goals relativos en O y convierte una vez los absolutos W->O.
- La fuente se congela durante cada goal. Una perdida ORB puede degradar a GT,
  que queda retenido hasta la siguiente frontera de goal.
- El controlador recibe exactamente la pose y velocidad publicadas por el mux.
- La alineacion de fuentes conserva continuidad en O; no corrige ORB con GT.

## Frames

```text
O_T_B: estado local continuo usado por control
W_T_B: estado global corregible usado para goals absolutos y visualizacion
```

Las correcciones globales no se reinyectan en ORB-SLAM3 ni desplazan un goal ya
aceptado.

## Alcance implementado

- interfaz comun de pose y velocidad;
- gate de validez local/global;
- source lock atomico por goal;
- `GT_FALLBACK` temporal hasta Fase 6;
- handoff y continuidad de frame;
- ejes RViz2 de la pose que entra al controlador.

## Exclusiones

No pertenecen a 5H el predictor a 50 Hz, la correccion `B_T_C`, el estimador
angular, el predictor dinamico, `MIDPOINT_DYNAMIC`, las baterias GT/ORB ni la
evidencia visual. Todo ello se documenta en 5I.

## Validacion

Las pruebas 242-252 validaron handshake, lock, goals y extrinseca. La
integracion inicial revelo inestabilidad al gobernar movimiento con ORB; su
diagnostico, correccion y cierre estan en `subfase_5I.md`.

## Criterio de cierre

5H queda conseguida porque existe una cadena unica
`NavigationState -> gen_tray/control`, con contratos O/W y fallback temporal
definidos. La calidad dinamica de ese estado se valida separadamente en 5I.
