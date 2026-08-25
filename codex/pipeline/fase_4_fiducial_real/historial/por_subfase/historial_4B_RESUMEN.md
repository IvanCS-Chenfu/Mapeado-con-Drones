# Historial 4B - resumen

```text
Estado agregado: CONSEGUIDA
Evidencia automatica y visual: conseguida
Pendiente propio de 4B: ninguno
```

El spawner genera AprilTags `DICT_APRILTAG_36h11`, los redetecta antes del
spawn y crea tres objetos estaticos/colisionables con cinco caras activas cada
uno. El escenario 201 espera readiness y completa el cuadrado ±10 con dos
drones a alturas 1.0/1.3 m: 5 pasos de movimiento y 10/10 goals correctos.

Gazebo y RViz2 estuvieron activos; `pipeline_flow`, `system_architecture`, sus
navegadores y la telemetria arquitectonica no arrancaron. El usuario confirmo
que el resultado visual fue perfecto. La prueba 201 queda limitada solo por la
eleccion de una trayectoria distinta de la prueba tipica deseada.

La trayectoria tipica queda ajustada al cuadrado ±10 con paradas en
`(0,±10)` y `(±10,0)`. Su YAML auxiliar y el escenario del paquete son
identicos y el contrato estatico pasa 6/6. Por decision del usuario no se
simula ahora: se verificara junto al bloque 4C+4D.

Estado bueno de cierre: pruebas 201/202, con revision visual perfecta y spawner
limpio. Gazebo conserva el exit 255 conocido posterior al `SIGINT` de cleanup.

Tras 4E+4F, la trayectoria tipica cambia seis transiciones alrededor de
`±180°` a yaw relativo para impedir vueltas de 270/360 grados. Las copias son
identicas, los contratos pasan 19/19 y `simulacion_dron` compila. La prueba 212
no alcanzo ninguno de esos giros: un `LoopTask` anterior fue rechazado por
`commit_pose_store_hard_constraint_violation`, activo el mission gate y dejo
el runner antes de enviar el paso 5. El cambio yaw sigue pendiente de revision
visual y no se considera fallido por esta ejecucion.

Por decision posterior del usuario se elimino completamente el latch
`secondary_blocking_failure_`. Build y suite del servidor pasan. La prueba 213
completa los 17 pasos con 22/22 goals, libera cada backpressure al terminar la
optimizacion y no contiene hard failures reales. Ambos visualizadores producen
74/74 PUB/SHOW y cierran por timeout sin afectar wrappers ni RViz2. El cierre
tecnico es correcto y el usuario da las subfases por concluidas. Sin embargo,
observa derivas no corregidas: la prueba 213 queda marcada para revisarla de
nuevo en 3Q, sin reabrir 4B ni la cadena fiducial.
