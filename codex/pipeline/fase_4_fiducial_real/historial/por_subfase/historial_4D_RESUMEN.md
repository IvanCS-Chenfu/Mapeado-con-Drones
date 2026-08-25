# Historial 4D - Resumen

## Estado agregado

`CONSEGUIDA`.

Servicio remoto, replica de interfaces, cliente con retry, cola acotada, worker,
detector AprilTag 36h11, IPPE_SQUARE y metricas estan implementados y probados.
Los fallos historicos 205/206 demostraron que HighGUI no podia vivir dentro de
`stereo`: primero hubo contaminacion Snap y despues un cierre forzado mato los
wrappers.

La presentacion vive ahora en el proceso ROS independiente
`fiducial_visualizer`; el wrapper solo publica la imagen anotada latest-only.
La prueba 207 completo el escenario y demostro el aislamiento, pero una carrera
de `WND_PROP_VISIBLE` cerraba cada ventana en 3-4 ms. Tras corregirla, la 208
completo la trayectoria: 2 visualizadores READY, 80 publicaciones, 79 SHOW,
17 cierres por timeout, 0 cierres falsos y deltas de ambos wrappers unos 57 s
despues del ultimo cierre. No murio ningun `stereo`.

El usuario acepta el resultado de la 208 y da 4C+4D por concluidas. No repetir
203-208; sus fallos y aprendizajes quedan preservados.

Detalle cronologico: `historial_4D.md`.
