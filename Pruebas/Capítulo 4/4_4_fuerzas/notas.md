# Punto 4.4: fuerzas y torques

## Prueba de torque en X

- Escenario: Gazebo GUI, `empty.world`, un dron y YAML con `steps: []`.
- Fuerza publicada: `0 N` durante toda la prueba.
- Torque publicado: `(0, 0, 0) N*m` de 0 a 10 s y `(0.01, 0, 0) N*m` de 10 a 30 s.
- Duracion total de la actuacion: 30 s.
- Launch: `simulacion_dron/launch/fase1_actuacion.launch.py`.
- Publicador: `simulacion_dron/src/fase1_actuacion_publisher.py`.
- Evidencia: `codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_x.reduced.log`.

Resultado: PASS en el segundo arranque. El cambio a `+0.01 N*m` se registro a
los `10.000 s`, el marcador de final a `30.000 s` y el publicador termino
limpiamente. El primer arranque se conserva como incidencia de infraestructura
por puerto de Gazebo ocupado; el script lo reinicio automaticamente.

## Repeticion con mayor torque en X

- Torque aplicado: `(0.02, 0, 0) N*m` entre los 10 y 30 s.
- Fuerza: `0 N` durante toda la prueba.
- Evidencia: `codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_x_02Nm.reduced.log`.

Resultado: PASS funcional en el segundo arranque. El primer arranque encontro
el puerto Gazebo `11345` ocupado y el script reinicio la simulacion. La GUI
puede mostrar «Forzar salida» durante esa colision y el apagado forzado; no se
relaciona con el valor del torque.

## Repeticion con torque de 1.4 N*m en X

- Escenario: Gazebo GUI, `empty.world`, un dron y YAML con `steps: []`.
- Fuerza publicada: `0 N` durante toda la prueba.
- Torque publicado: `(0, 0, 0) N*m` de 0 a 10 s y `(1.4, 0, 0) N*m` de 10 a 30 s.
- Evidencia: `codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_x_140Nm_v2.reduced.log`.

Resultado: PASS de ejecucion. El modelo se inserto correctamente y el cambio a
`+1.4 N*m` se registro a los `10.000 s`. El escenario vacio termino con
`success=true` y Gazebo se cerro limpiamente. La primera ejecucion con el mismo
valor fue INVALID porque el entorno restringido no permitio crear `~/.ros/log`;
no llego a iniciar Gazebo ni a aplicar el torque.

## Torque de 1.5 N*m en X con 20 s de espera

- Escenario: Gazebo GUI, `empty.world`, un dron y YAML con `steps: []`.
- Fuerza publicada: `0 N` durante toda la prueba.
- Tiempo sin torque: 20 s.
- Torque publicado: `(1.5, 0, 0) N*m` durante 20 s, desde los 20 hasta los 40 s.
- Evidencia: `codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_x_150Nm_20_20_v2.reduced.log`.

La primera ejecucion con espera posterior de 5 s fue PARCIAL: registro el
cambio a `+1.5 N*m` a los `20.000 s`, pero el launch se cerro antes del final.
La repeticion con 25 s de espera posterior fue PASS de ejecucion: se registro
`[F1-ACTUATION-DONE] duration=40s` y todos los procesos terminaron limpiamente.

## Torque de 2 N*m en Z

- Tiempo sin torque: 20 s.
- Torque publicado: `(0, 0, 2) N*m` durante 20 s, desde los 20 hasta los 40 s.
- Evidencia: `codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_z_200Nm_20_20.reduced.log`.

Resultado: PASS de ejecucion. El cambio a `+2 N*m` se registro a los
`20.000 s`, el marcador de final a `40.000 s` y el cierre fue limpio.

## Torque de 4 N*m en Z

- Tiempo sin torque: 20 s.
- Torque publicado: `(0, 0, 4) N*m` durante 20 s, desde los 20 hasta los 40 s.
- Evidencia: `codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_z_400Nm_20_20.reduced.log`.

Resultado: PASS de ejecucion. El cambio a `+4 N*m` se registro a los
`20.000 s`, el marcador de final a `40.000 s` y todos los procesos terminaron
limpiamente.
