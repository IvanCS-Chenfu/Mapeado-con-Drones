# Prueba 6.5.2 - Revisiones, resultados obsoletos y KFs tardios

## Configuracion

- Un dron con fuente de navegacion GT.
- Ruta: `(0,-10,1,90 deg) -> (-10,-10,1,90 deg) -> (-10,10,1,0 deg) -> (3,10,1,-90 deg)`.
- El ultimo tramo supera en 3 m la region del fiducial norte para que entren
  KFs mientras existe o acaba de terminar la optimizacion.
- Fase 6 y protocolo de perdida ORB desactivados.
- GUI: Gazebo y `multidron_gui`; sin RViz2.

## Instrumentacion y criterio

- Telemetria pasiva `[C6-KF-POSE]` para KFs, timestamps y revisiones.
- Eventos existentes: `[F3Q-OPT-START]`, `[F3Q-OPT-END]`,
  `[F3K-ATOMIC-COMMIT]`, `[F3K-COMMIT-STALE]`,
  `[F3K-FUTURE-KF-PROPAGATE]`, `[F3H-FID-REVALIDATE]`,
  `[F3P-FUSION-RETRY]` y `[F3Q-POST-OPT-LOOPS]`.
- Se ejecuta primero sin retardo artificial. La prueba documentara los eventos
  que ocurran realmente; la ausencia de un tipo de concurrencia no se suplira
  modificando la logica.

## Resultado

La ejecucion natural termino con `[SCENARIO-RUNNER-DONE]` y `[SIM-DONE]` con
codigo 0. El log bruto y su reduccion se conservan como
`datos_brutos/ejecucion_natural.*`.

El segundo fiducial genero un grafo a los `66.483 s` y un commit completo a
los `67.861 s`: revision 41, ventana de 87 KFs, `late_window=0`, `tail=1` y
87 KFs movidos. Se observaron 119 KFs unicos: 106 antes de construir el grafo,
4 durante la ventana de correccion y 9 despues del commit, mientras el dron
continuaba hacia `x=3`.

Despues del commit se registraron seis eventos
`[F3K-FUTURE-KF-PROPAGATE]`, una actualizacion de continuacion, tareas
post-optimizacion y un reintento de fusion. La segunda revalidacion fiducial
quedo `stale` porque el objetivo ya estaba dentro del umbral. No aparecio un
`[F3K-COMMIT-STALE]`; el commit aceptado no estaba obsoleto. Tampoco se
emitieron `[F3Q-OPT-START]` ni `[F3Q-OPT-END]` para esta ruta fiducial, por lo
que la figura usa `[F3I-GRAPH-BUILD]` como inicio efectivo de la correccion.

La figura `figuras/timeline_revisiones_y_kfs_tardios.png` y los CSV de
`datos_procesados/` se generan con
`scripts/procesar_c6_5_2_revisiones_y_concurrencia.py`. No fue necesario usar
ningun retardo artificial de optimizacion.
