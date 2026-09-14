# Resumen 6H

Estado: PARCIAL. El score mínimo `0.2`, coverage superficial reversible,
portales exclusivos y candidato automático a D* están implementados. La prueba
636 valida con GT el procedimiento correcto de anclaje en fiducial 2
`(0, -10, Z)`, coverage y publicación de rutas candidatas en GUI F7 (2-3
waypoints, 1.660-37.616 ms en los planes aceptados observados). `task_lib` 9/9,
`task_server` 7/7 y GUI 9/9. Falta una prueba integrada que active una entrada
exclusiva y pruebe de extremo a extremo `BLOCKED_BRANCH`, desbloqueo o
`SUPERSEDED`; el desbloqueo actual es un proxy de perímetro local. No se forzará
esa prueba en la casa exterior: su interior `UNKNOWN` tras una barrera
`OCCUPIED` es inaccesible, no una rama. El lifecycle se retomará en una
topología que tenga una expansión conectada real, sin suponer edificios cerrados.
Decisión vigente: conservar sin cambios el `progress` actual, que mide frontera
resuelta y no cobertura superficial o volumétrica real, hasta implementar
coverage volumétrico.

La politica de candidato ya no exige una pared/recorrido exactos: sus valores
2.5 m/5 m son preferencias. La prueba 648 valida que un portal UNKNOWN
alcanzable puede desbloquear el primer movimiento cuando D* lo acepta, sin
relajar ocupacion ni inflacion.

La prueba 649 revelo que incluir el lado FREE como meta podia concentrar la
exploracion en paredes conocidas. La correccion posterior selecciona solo el
extremo UNKNOWN de cada portal alcanzable y mide `novelty_m`; 650 compilo,
paso `task_server` 7/7 y cerro con `SIM-DONE success=true`. Las 339 selecciones
del log son `unknown_portal`, sin meta FREE. Queda parcial porque el proxy no es
coverage volumetrico y D* aun debe descartar muchos portales inflados.

Actualizacion vigente: coverage es ya volumetrico, no el proxy de caras de las
entradas anteriores. Las pruebas 656/657 no cierran la integracion: tras el
anclaje GT, callbacks seriales de mapa retrasan la compuerta mas de 60 s. D* no
es el cuello: el intento posterior responde en 0.450 ms. Falta acordar el
aislamiento de callbacks antes de validar el flujo completo.

La correccion posterior ya esta validada en 659: grupos mapa/control separados
y `MultiThreadedExecutor(2)` responden la compuerta en unos 4 ms mientras un
lote navegable dura 34.141 s. La aplicacion del gate queda diferida a una
frontera segura del worker; D* no fue el cuello (`no_safe_escape` en 0.286 ms).

La prueba 660 valida la política posterior: `no_safe_escape` mantiene la tarea
RUNNING y el mismo destino UNKNOWN, sin `BLOCKED`; D* reintentó esa meta y la
limpió solo cuando la evidencia la invalidó. El valor de 15 s es una espera
mínima: con backlog del worker el reintento medido fue 44.188 s después. La
semántica es correcta; la puntualidad exige, si hiciera falta, trocear lotes
largos de VoxelMapWorker.

La 661 no valida movimiento autónomo: D2, ya anclado en fiducial 2, acumuló
FREE y coverage BC de 0.002 a 0.003, pero no obtuvo corredor FREE suficiente
para su clearance de 1 m. `no_safe_escape` se reintentó sin bloqueo y no hubo
plan/acción de D2. Antes de repetir se debe acordar la precondición de escape.

Actualización 665: la política vigente reduce la distancia mínima de meta
UNKNOWN a 2 m. La ejecución parcial con D1/GT confirma que el selector eligió
metas UNKNOWN de 2.00--2.01 m y que el porcentaje volumétrico pasó de 0.015 a
0.027 antes de ser detenida manualmente. No es prueba de cobertura sostenida ni
de cierre: el usuario pidió parar Gazebo durante la observación.
