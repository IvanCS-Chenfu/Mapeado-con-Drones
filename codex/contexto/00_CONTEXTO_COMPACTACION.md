# Contexto de compactacion

## Trabajo activo

Peticion vigente: ejecutar la prueba 4.5 de seguimiento de trayectoria con dos
perfiles, cúbico y trapezoidal, usando un dron, fuente GT, mundo vacío, GUI,
registro sincronizado y 60 s de espera posterior para grabación manual.

Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA
Prueba acordada: dos ejecuciones independientes con los mismos waypoints
absolutos y yaw absoluto; una con `tipo_trayectoria: 0` (cúbica) y otra con
`tipo_trayectoria: 1` (trapezoidal); un solo dron; `navigation_source: gt`;
registro de GT y feedback de trayectoria en CSV; gráficas XY, matriz 3x4 de
GT frente a trayectoria, matriz 3x4 de errores y tabla cuantitativa; GUI de
Gazebo abierta; mundo `empty`; 60 s después de terminar la trayectoria.
Dudas abiertas: ninguna.
Trabajo activo: no

Plan autorizado vigente para 4.5:

- añadir un registrador único al paquete `simulacion_dron` para guardar pose,
  velocidad y aceleración GT y feedback de trayectoria;
- añadir dos YAML de perfil y dos escenarios con los waypoints acordados;
- permitir seleccionar el mundo desde `multi_dron.launch.py`, conservando el
  valor actual por defecto;
- ejecutar cada perfil con `run_simulation.sh`, conservar y reducir sus logs;
- generar CSV sincronizados, figuras y métricas de la tabla en
  `Pruebas/Capítulo 4/4_5_seguimiento/`;
- documentar resultados y cualquier incidencia sin borrar ejecuciones previas.

Validación inicial 4.5: el análisis AST de launch y herramienta offline pasó.
El primer build falló únicamente porque el registrador declaró `float` para los
arrays de feedback, mientras `TrayAction` los define como `double`; se corrigió
el tipo en `fase45_recorder.cpp`. Build repetido: PASS, `simulacion_dron`
compiló e instaló en 14,1 s. Siguiente acción exacta: validar argumentos y
YAML, y después ejecutar el perfil cúbico.

Validación YAML 4.5: los cuatro YAML de configuración/escenario pasan
`yaml.safe_load`. `ros2 launch --show-args` no pudo ejecutarse en el entorno
aislado porque no puede crear el directorio de logs de `~/.ros`; no es un
fallo del código y se reintentará usando el ejecutor de simulación con escritura
habilitada. Siguiente acción exacta: ejecutar `f45_cubica` con GUI, mundo vacío,
un dron, fuente GT, Fase 6/ORB/servidor/fiduciales desactivados, registrador
activo y 60 s de espera posterior.

Resultado `f45_cubica` intento 1: INVALIDO de infraestructura. Los tres
intentos terminaron antes de iniciar Gazebo porque el espacio de `Capítulo 4`
en `mission_profile` no estaba protegido al construir el comando del ejecutor;
el log reducido conserva `malformed launch argument '4/4_5_seguimiento/...`.
Se corrigió `run_simulation.sh` usando quoting shell para esa ruta. Siguiente
acción exacta: repetir el perfil cúbico con la misma configuración.

Resultado `f45_cubica_v2`: `run_simulation.sh` terminó con código 0 y
`success=true`; el escenario cúbico terminó con código 0 y se mantuvo la GUI
durante los 60 s posteriores. El launch recibió SIGINT y cerró sin reintentos.
Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_f45_cubica_v2.log`. Siguiente acción
exacta: reducir el log, comprobar los CSV del registrador y generar los
resultados offline de la prueba cúbica.

Análisis preliminar `f45_cubica_v2`: el reducido confirma nueve goals cúbicos
`success=true`, fuente GT efectiva y escenario finalizado correctamente. El
registrador produjo `gt_pose.csv`, `gt_velocity.csv`, `gt_acceleration.csv` y
`trajectory_feedback.csv` en `Pruebas/Capítulo 4/4_5_seguimiento/datos/cubica`
(~8.100 muestras GT y 2.296 feedback). El proceso combinado `gazebo` informó
exit code 255 al recibir SIGINT, pero `run_simulation.sh` cerró con código 0 y
todos los nodos ROS finalizaron limpiamente; se considera incidencia de cierre
de GUI, no fallo de la prueba física. Siguiente acción exacta: generar las
figuras y métricas cúbicas.

Corrección del analizador 4.5: se detectó y corrigió la interpolación lineal
del yaw en los límites entre goals, sustituyéndola por la muestra de feedback
más cercana para evitar valores angulares intermedios inexistentes. Las
métricas cúbicas se regeneraron con esa corrección: duración común 76,92 s,
RMSE de posición 0,150755 m, MAE 0,118714 m, máximo 0,527657 m y error final
0,132985 m. Siguiente acción exacta: ejecutar el perfil trapezoidal con la
misma ruta, GUI, fuente GT y 60 s posteriores.

Plan autorizado vigente para esta prueba:

- crear un launch de prueba aislado que arranque `empty.world`, el modelo de un
  solo dron, `aplicar_fuerzas_dron` y el publicador de Fase 1;
- no arrancar `control_calcular_fuerzas`, `gen_tray`, ORB-SLAM3 ni el servidor;
- instalar solo el launch y el publicador auxiliares en `simulacion_dron`;
- parametrizar el publicador auxiliar para reutilizarlo sin crear otro nodo;
- ejecutar la simulación mediante `run_simulation.sh` y conservar su log;
- reducir el log y verificar los marcadores de las fases de torque y cierre.

Preparación de torque X: el publicador auxiliar acepta ahora `actuation_mode`,
`force_value` y `torque_x_nm`; el launch expone esos argumentos y conserva el
modo de fuerza anterior por defecto. Para esta prueba se usará
`actuation_mode:=torque_x`, `force_value:=0.0` y `torque_x_nm:=0.01`. Siguiente
acción exacta: compilar `simulacion_dron` y ejecutar la prueba visual de 30 s.

Build torque X: PASS. `build_selected_packages.sh --group simulacion
simulacion_dron` terminó con código 0 en 0.79 s. Checkpoint de ejecución
`f1_4_4A_torque_x`: usar `fase1_actuacion.launch.py` con el perfil vacío,
`actuation_mode:=torque_x`, `force_value:=0.0` y `torque_x_nm:=0.01`; GUI,
un dron, mundo vacío, torque cero de 0-10 s y `+0.01 N·m` de 10-30 s. Criterio:
marcadores de inicio, cambio a torque X, `[F1-ACTUATION-DONE]`, modelo cargado
y cierre limpio.

Resultado `f1_4_4A_torque_x`: `run_simulation.sh` terminó con código 0 y
`success=true` tras un reinicio automático de Gazebo; el primer arranque fue
inválido por muerte temprana, pero el segundo ejecutó el escenario vacío y la
espera posterior. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_x.log`. Siguiente acción
exacta: reducir el log y verificar los marcadores del torque X y el cierre
limpio del segundo intento.

Análisis `f1_4_4A_torque_x`: PASS en el segundo intento. El reducido confirma
que el perfil vacío terminó con código 0, la fuerza fue `0.000 N`, el torque
fue `(0,0,0)` al inicio y cambió a `(0.010,0,0)` exactamente a `10.000 s`.
El publicador emitió `[F1-ACTUATION-DONE]` a `30.000 s` y terminó limpiamente;
el launch cerró después sin error del publicador. El primer intento queda
conservado como incidencia de infraestructura: puerto Gazebo ocupado.

Nueva prueba solicitada: repetir torque X aumentando el valor a `+0.02 N·m`,
manteniendo fuerza `0 N`, torque cero durante 10 s, torque aplicado durante
20 s, un dron, `empty.world`, GUI y cierre a los 30 s. Se reutilizará el launch
y publicador ya parametrizados. Checkpoint `f1_4_4A_torque_x_02Nm`. Siguiente
acción exacta: ejecutar `run_simulation.sh` con `torque_x_nm:=0.02` y conservar
el log.

Resultado `f1_4_4A_torque_x_02Nm`: `run_simulation.sh` terminó con código 0 y
`success=true` tras un reinicio automático de Gazebo; el segundo intento
ejecutó el escenario vacío y completó la espera posterior. Log completo
conservado en `codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_x_02Nm.log`.
Siguiente acción exacta: reducir el log y verificar el torque de `0.02 N·m` y
el cierre limpio.

Análisis `f1_4_4A_torque_x_02Nm`: PASS funcional en el segundo intento. El
reducido confirma fuerza `0.000 N`, torque `(0,0,0)` al inicio y
`(0.020,0,0)` a `10.000 s`, con `[F1-ACTUATION-DONE]` a `30.000 s` y cierre
limpio. El primer intento falló por `Unable to start server [bind: Address
already in use]` en el puerto Gazebo `11345`; el script lo resolvió mediante
reinicio forzado. Diagnóstico vigente: la GUI puede mostrar «Forzar salida» por
la colisión de instancias y el apagado abrupto, no por el torque ni por el
modelo del dron. No queda ningún proceso Gazebo ni puerto `11345` activo.

Corrección autorizada del ejecutor: `run_simulation.sh` debe detectar Gazebo
residual antes de cada intento, distinguirlo de un proceso ajeno que ocupe el
puerto `11345`, esperar la liberación del puerto después de limpiar y repetir
la misma limpieza al terminar una prueba correcta. Se conserva el mecanismo de
reintentos y los logs. Siguiente acción exacta: modificar el script y validar
su sintaxis.

Corrección de `run_simulation.sh` completada: se añadieron detección de
procesos/puerto, limpieza de instancias residuales antes de cada intento,
espera de liberación de `11345` y limpieza final también en ejecuciones
correctas. Si el puerto lo ocupa un proceso ajeno a Gazebo, el script lo
notifica y no lo mata. `bash -n` y `git diff --check` pasan. Checkpoint de
validación `f1_4_4A_cleanup`: repetir el escenario de torque X de `0.02 N·m`
para comprobar un único arranque, cierre normal y puerto libre al terminar.

Resultado `f1_4_4A_cleanup`: `run_simulation.sh` terminó con código 0 y
`success=true` en un único intento. Antes de arrancar detectó y limpió una
instancia residual, confirmó `[SIM-GAZEBO-READY] puerto 11345 libre`, ejecutó
el escenario vacío y, al cerrar, volvió a liberar el puerto. El log completo se
conserva en `codex/archivos_auxiliares/logs/prueba_f1_4_4A_cleanup.log`.
Siguiente acción exacta: reducir el log y comprobar que no hubo reintento ni
errores de la prueba física.

Mejora adicional del cierre: la limpieza de Gazebo usará primero `SIGTERM` y
solo recurrirá a `SIGKILL` si el proceso o el puerto no desaparecen tras la
espera configurada. `bash -n` y `git diff --check` vuelven a pasar. Checkpoint
`f1_4_4A_cleanup_v2`: repetir el mismo escenario para verificar arranque sin
colisión, cierre con `SIM-GAZEBO-TERM` y puerto `11345` libre.

Resultado `f1_4_4A_cleanup_v2`: `run_simulation.sh` terminó con código 0 y
`success=true` en un único intento; el arranque no mostró colisión y el
escenario vacío completó la prueba. El cierre envió `SIGTERM`, pero Gazebo y/o
el puerto siguieron activos durante 10 s y fue necesario el fallback
`SIGKILL`; después se confirmó `[SIM-GAZEBO-READY] puerto 11345 libre`.
Resultado: PARCIAL para el objetivo de eliminar el apagado forzado. La
incidencia está en la respuesta de la GUI/proceso combinado `gazebo`, no en el
publicador ni en la física. Siguiente acción exacta: reducir el log y evaluar
la separación de `gzserver` y `gzclient` para cerrar la GUI de forma aislada.

Corrección en preparación: `fase1_actuacion.launch.py` separa ahora
`gzserver` y `gzclient`, manteniendo la GUI pero permitiendo cerrar servidor y
cliente como procesos independientes. Siguiente acción exacta: recompilar
`simulacion_dron` y repetir la validación de limpieza.

Build de la separación Gazebo: PASS. `build_selected_packages.sh --group
simulacion simulacion_dron` terminó con código 0 en 0.79 s. Checkpoint
`f1_4_4A_cleanup_v3`: repetir el escenario de torque X de `0.02 N·m` para
validar que `gzserver` y `gzclient` arrancan, la prueba termina y el cierre usa
`SIGTERM` sin `SIGKILL`.

Resultado `f1_4_4A_cleanup_v3`: PASS. `run_simulation.sh` terminó con código 0
en un único intento; el escenario vacío y el publicador completaron sus
marcadores, y el launch cerró tras `SIGINT` sin emitir `SIM-GAZEBO-TERM` ni
`SIM-GAZEBO-KILL`. La separación de `gzserver` y `gzclient` evita el bloqueo
del ejecutable combinado `gazebo`. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_cleanup_v3.log`. Siguiente
acción exacta: reducir el log y actualizar documentación del script y launch.

Nueva prueba solicitada: aumentar el torque X a `+0,10 N·m`, manteniendo fuerza
`0 N`, 10 s sin torque, 20 s con torque, un dron, GUI y `empty.world`. El
coeficiente de mezcla `fuerza2torque=0.02` no limita explícitamente el comando;
`0.10 N·m` equivale a una demanda diferencial mayor y debe producir un giro
visible. Checkpoint `f1_4_4A_torque_x_10Nm`. Siguiente acción exacta: ejecutar
la prueba con el launch separado `gzserver`/`gzclient`.

Resultado `f1_4_4A_torque_x_10Nm`: `run_simulation.sh` terminó con código 0 y
`success=true` en un único intento. El escenario vacío completó la prueba y el
launch se cerró sin `SIM-GAZEBO-KILL`; queda pendiente reducir el log para
confirmar el valor y los tiempos del torque. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_x_10Nm.log`.

Corrección de interpretación física: tomando como referencia un eje situado
en el borde de `Y` mínimo del cuerpo, la semidimensión del cuerpo es
`0,10 m` (`fisico.cuerpo.dim = [0.2, 0.2, 0.05]`) y la masa total aproximada
del dron es `1,4 kg`. Por tanto, el torque equivalente del peso respecto a ese
eje es `|tau_x| = 0,10 * (1,4 * 9,81) ~= 1,37 N·m`, con el signo dependiendo de
la convención de ejes. Respecto al centro de masas del dron simétrico, el
torque gravitatorio sigue siendo aproximadamente cero.

Checkpoint de ejecución `f1_4_4A_torque_x_140Nm`: se repetirá la prueba con
`fase1_actuacion.launch.py`, perfil vacío, un dron, `empty.world`, GUI,
`actuation_mode:=torque_x`, `force_value:=0.0` y `torque_x_nm:=1.4`. El
publicador mantendrá torque cero de 0 a 10 s y aplicará `+1,40 N·m` en X de 10
a 30 s; timeout total 30 s y cierre posterior del launch. Siguiente acción
exacta: ejecutar `run_simulation.sh` y conservar su log.

Resultado `f1_4_4A_torque_x_140Nm`: `run_simulation.sh` terminó con código 1 y
`success=false` tras tres intentos (`attempt=0..2`). Gazebo murió durante el
arranque antes de que el publicador pudiera ejecutar la prueba, por lo que el
resultado físico no es evaluable. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_x_140Nm.log`. Siguiente
acción exacta: reducir el log y diagnosticar la primera causa real.

Diagnóstico `f1_4_4A_torque_x_140Nm`: el reducido muestra que `ros2 launch`
terminó antes de iniciar Gazebo porque no pudo crear
`/home/chenfu/.ros/log/...`: `OSError: [Errno 30] Read-only file system`.
Ningún marcador de actuación llegó a ejecutarse. Se repetirá la misma prueba
en un entorno con escritura habilitada para `~/.ros`.

Resultado `f1_4_4A_torque_x_140Nm_v2`: `run_simulation.sh` terminó con código
0 y `success=true`; Gazebo y el escenario arrancaron correctamente, el nodo de
escenario terminó con código 0 y el launch se cerró con SIGINT/SIGTERM. Log
completo conservado en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_x_140Nm_v2.log`.
Log reducido en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_x_140Nm_v2.reduced.log` y
marcadores validados.

Análisis `f1_4_4A_torque_x_140Nm_v2`: PASS de ejecución. El reducido confirma
que el modelo se insertó, la fuerza permaneció en `0 N`, el torque fue nulo al
inicio y cambió exactamente a `(1.400,0,0) N·m` a `10.000 s`; el escenario vacío
terminó con `success=true` y todos los procesos cerraron limpiamente. La
ejecución previa `f1_4_4A_torque_x_140Nm` queda como INVALID de infraestructura
por el entorno sin escritura en `~/.ros`, no por el valor del torque.

Nueva prueba solicitada: repetir torque X con `+1,50 N·m`, fuerza `0 N`,
20 s de espera sin torque y 20 s de aplicación, un dron, `empty.world`, GUI y
cierre tras 40 s de actuación. Se parametrizó el publicador auxiliar con
`idle_duration_sec` y `actuation_duration_sec`; el launch calcula la duración
total automáticamente y conserva los defaults anteriores de 10/20 s.
Siguiente acción exacta: compilar `simulacion_dron` y ejecutar
`f1_4_4A_torque_x_150Nm_20_20`.

Build de la parametrización temporal: PASS. `build_selected_packages.sh
--group simulacion simulacion_dron` terminó con código 0; se instalaron el
publicador y el launch actualizados. Checkpoint de ejecución
`f1_4_4A_torque_x_150Nm_20_20`: usar `actuation_mode:=torque_x`,
`force_value:=0.0`, `torque_x_nm:=1.5`, `idle_duration_sec:=20.0` y
`actuation_duration_sec:=20.0`, con perfil vacío, un dron, `empty.world`, GUI,
timeout 70 s y cierre tras la ventana de 40 s. Siguiente acción exacta:
ejecutar `run_simulation.sh` y conservar el log.

Resultado `f1_4_4A_torque_x_150Nm_20_20`: `run_simulation.sh` terminó con
código 0 y `success=true` en un único intento. El escenario terminó con código
0 y el launch se cerró limpiamente tras la ventana temporal de 40 s. Log
completo conservado en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_x_150Nm_20_20.log`.
Siguiente acción exacta: reducir el log y validar los marcadores temporales.

Análisis provisional `f1_4_4A_torque_x_150Nm_20_20`: PARCIAL. El reducido
confirma el inicio, el reposo de 20 s y el cambio a `(1.500,0,0) N·m` a
`20.000 s`, pero `run_simulation.sh` cerró el launch después de solo 5 s de
espera posterior al escenario vacío; no apareció el marcador
`[F1-ACTUATION-DONE]` de 40 s. Se repetirá con `post-scenario-wait-sec` de 25 s
para dejar completar la actuación.

Resultado `f1_4_4A_torque_x_150Nm_20_20_v2`: `run_simulation.sh` terminó con
código 0 y `success=true` en un único intento, usando 25 s de espera posterior
al escenario para cubrir la ventana completa de actuación. El escenario y el
launch terminaron correctamente. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_x_150Nm_20_20_v2.log`.
Siguiente acción exacta: reducir el log y validar el marcador de finalización.

Análisis `f1_4_4A_torque_x_150Nm_20_20_v2`: PASS de ejecución. El reducido
confirma fuerza `0 N`, torque nulo durante los primeros 20 s, cambio exacto a
`(1.500,0,0) N·m` a `20.000 s`, final de la aplicación a `40.000 s` y marcador
`[F1-ACTUATION-DONE] duration=40s`. El escenario vacío terminó correctamente y
Gazebo se cerró sin errores del publicador. La ejecución
`f1_4_4A_torque_x_150Nm_20_20` queda conservada como PARCIAL porque la espera
posterior de 5 s cerró el launch antes de completar los 20 s de torque.

Nueva prueba final solicitada: aplicar `+2,00 N·m` de torque en Z, manteniendo
fuerza `0 N`, 20 s de espera sin torque y 20 s de aplicación, un dron,
`empty.world`, GUI y perfil vacío. Se amplió el publicador auxiliar con el modo
`actuation_mode:=torque_z` y el parámetro `torque_z_nm`; los modos anteriores
siguen disponibles. Siguiente acción exacta: compilar `simulacion_dron` y
ejecutar `f1_4_4A_torque_z_200Nm_20_20`.

Build pendiente para la prueba final: se modificaron el publicador y el launch
para admitir `torque_z`; el build debe terminar correctamente antes de simular.

Build de torque Z: PASS. `build_selected_packages.sh --group simulacion
simulacion_dron` terminó con código 0 y dejó instalado el modo `torque_z`.
Checkpoint de ejecución `f1_4_4A_torque_z_200Nm_20_20`: usar
`actuation_mode:=torque_z`, `force_value:=0.0`, `torque_z_nm:=2.0`,
`idle_duration_sec:=20.0`, `actuation_duration_sec:=20.0`, perfil vacío, un
dron, `empty.world`, GUI, `post-scenario-wait-sec:=25` y timeout 70 s.
Siguiente acción exacta: ejecutar `run_simulation.sh` y conservar el log.

Resultado `f1_4_4A_torque_z_200Nm_20_20`: `run_simulation.sh` terminó con
código 0 y `success=true` en un único intento. El escenario terminó con código
0 y el launch se cerró limpiamente tras la ventana completa de 40 s. Log
completo conservado en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_z_200Nm_20_20.log`.
Log reducido en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_z_200Nm_20_20.reduced.log`
y marcadores validados.

Análisis `f1_4_4A_torque_z_200Nm_20_20`: PASS de ejecución. El reducido
confirma torque Z nulo durante 20 s, cambio a `(0.000,0,2.000) N·m` a
`20.000 s`, final a `40.000 s` y cierre limpio.

Nueva repetición solicitada: aumentar el torque Z a `+4,00 N·m`, manteniendo
fuerza `0 N`, 20 s de espera, 20 s de aplicación, un dron, `empty.world`, GUI y
perfil vacío. Se reutilizará el modo `torque_z` y los tiempos parametrizados.
Checkpoint `f1_4_4A_torque_z_400Nm_20_20`: usar `torque_z_nm:=4.0`,
`idle_duration_sec:=20.0`, `actuation_duration_sec:=20.0` y
`post-scenario-wait-sec:=25`. Siguiente acción exacta: ejecutar
`run_simulation.sh` y conservar el log.

Resultado `f1_4_4A_torque_z_400Nm_20_20`: `run_simulation.sh` terminó con
código 0 y `success=true` en un único intento. El escenario terminó con código
0 y el launch se cerró limpiamente tras la ventana completa de 40 s. Log
completo conservado en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_z_400Nm_20_20.log`.
Log reducido en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_torque_z_400Nm_20_20.reduced.log`
y marcadores validados.

Análisis `f1_4_4A_torque_z_400Nm_20_20`: PASS de ejecución. El reducido
confirma fuerza `0 N`, torque Z nulo durante los primeros 20 s, cambio exacto a
`(0.000,0,4.000) N·m` a `20.000 s`, final de la aplicación a `40.000 s` y
marcador `[F1-ACTUATION-DONE] duration=40s`. El escenario vacío terminó con
`success=true` y todos los procesos cerraron limpiamente.

Cambios preparados para 4.4A:

- `simulacion_dron/launch/fase1_actuacion.launch.py`: mundo vacío, un dron,
  mezclador y publicador, sin controlador PD ni navegación.
- `simulacion_dron/src/fase1_actuacion_publisher.py`: 0 N durante 10 s,
  15 N durante 20 s y torque nulo.
- `Pruebas/Capítulo 4/configuracion/fase1_4_4_mission_profile.yaml` y
  `fase1_4_4_trayectoria_vacia.yaml`: perfil válido con `steps: []`.
- siguiente acción exacta: compilar `simulacion_dron` y después ejecutar la
  simulación visual de 30 s con Gazebo GUI.

Build 4.4A: PASS. `build_selected_packages.sh --group simulacion
simulacion_dron` terminó con código 0 en 10.6 s. El launch y el publicador ya
están instalados en el workspace de simulación. Siguiente acción exacta:
validar el launch con `--show-args` y ejecutar la simulación visual de 30 s.

Validación previa: YAML de perfil y escenario vacío PASS (`steps: []`). El
`ros2 launch --show-args` fue bloqueado por el sandbox al intentar crear
`~/.ros/log`; se requiere ejecutar la prueba con filesystem y GUI habilitados.
Prueba a ejecutar: `fase1_actuacion.launch.py`, `startup_wait=15 s`,
`post_scenario_wait=15 s`, timeout 30 s, un dron, `empty.world`, fuerza 0 N de
0-10 s y 15 N de 10-30 s, torque nulo. Siguiente acción exacta: ejecutar
`run_simulation.sh` y conservar `prueba_f1_4_4A.log`.

Intento `f1_4_4A`: `run_simulation.sh` terminó con código 1 tras dos intentos;
Gazebo murió durante el arranque antes de ejecutar el escenario y antes de
aplicar la fuerza. Resultado: INVALID por infraestructura, no evaluable como
prueba física. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A.log`. Siguiente acción exacta:
reducir el log y diagnosticar el primer error real.

Diagnóstico del reducido: la ruta del perfil contiene el espacio de `Capítulo
4` y fue añadida al launch sin escapar; ROS emitió `malformed launch argument
'4/configuracion/fase1_4_4_mission_profile.yaml'`. No se inició Gazebo ni el
publicador. Corrección mecánica: repetir con `mission_profile:=` escapado en el
comando de launch y conservar este primer intento sin sobrescribirlo.

Checkpoint de repetición `f1_4_4A_v2`: se ejecutará el mismo escenario y los
mismos tiempos usando `run_simulation.sh`, pasando la ruta de `Capítulo 4`
escapada dentro de `mission_profile:=`. Se espera que Gazebo GUI arranque con
un solo dron en `empty.world`, que el publicador marque 0 N de 0-10 s y 15 N de
10-30 s con torque nulo, y que la prueba termine a los 30 s. Siguiente acción
exacta: ejecutar la simulación visual y conservar su log.

Resultado `f1_4_4A_v2`: `run_simulation.sh` terminó con código 1 tras los dos
intentos configurados. La ruta escapada fue aceptada por ROS, pero Gazebo
murió durante el arranque en ambos intentos, antes de iniciar el escenario y
antes de aplicar la fuerza. Resultado: INVALID por infraestructura, no
evaluable como prueba física. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_v2.log`. Siguiente acción
exacta: reducir el log y diagnosticar el primer error real.

Diagnóstico `f1_4_4A_v2`: Gazebo llegó a cargar `empty.world` y el modelo de
un dron, pero el publicador auxiliar terminó con `TypeError` porque
`RcutilsLogger.warn()` no admite argumentos de formato separados. El script
abortó la simulación al detectar la muerte temprana. Corrección aplicada solo
al nodo de prueba: interpolar el mensaje de cambio de fase antes de llamar al
logger. Siguiente acción exacta: recompilar `simulacion_dron` y repetir la
prueba con un identificador nuevo.

Build correctivo 4.4A: PASS. `build_selected_packages.sh --group simulacion
simulacion_dron` terminó con código 0 en 0.78 s e instaló el publicador
corregido. Siguiente acción exacta: ejecutar la simulación visual con el nuevo
identificador `f1_4_4A_v3`.

Checkpoint de ejecución `f1_4_4A_v3`: repetir `fase1_actuacion.launch.py` con
la ruta de `Capítulo 4` escapada, un dron en `empty.world`, GUI de Gazebo,
0 N durante 10 s, 15 N durante 20 s y torque nulo. Criterio: marcadores del
publicador para inicio, cambio a 15 N y cierre a 30 s; modelo cargado y cierre
limpio del launch. Siguiente acción exacta: ejecutar `run_simulation.sh` y
conservar su log.

Resultado `f1_4_4A_v3`: `run_simulation.sh` terminó con código 0 y
`success=true`. Gazebo arrancó sin reintentos, el `scenario_runner_node`
terminó con código 0 y la espera posterior de 15 s se completó. Log completo
conservado en `codex/archivos_auxiliares/logs/prueba_f1_4_4A_v3.log`.
Siguiente acción exacta: reducir el log y verificar los marcadores del
publicador, el modelo único y el cierre limpio.

Interpretación `f1_4_4A_v3`: la actuación se observó hasta el cambio a 15 N a
los 10.003 s y el escenario fue correcto, pero el cierre del launch se produjo
ligeramente antes del marcador final de 30 s y dejó un `RCLError` del
publicador al recibir SIGINT. No invalida la ventana física observada, pero se
corrige para la repetición solicitada con 14 N: fuerza 14 N de 10-30 s, fase
renombrada y captura limpia del final; el launch esperará 31 s para dejar que
el publicador emita `[F1-ACTUATION-DONE]`.

Nueva prueba solicitada: repetir 4.4A con 14 N, conservando un dron, mundo
vacío, GUI, 0 N durante 10 s, torque nulo y cierre tras la ventana de 30 s.
Cambios preparados en el nodo auxiliar y su launch. Siguiente acción exacta:
compilar `simulacion_dron` y ejecutar la prueba `f1_4_4A_14N`.

Build para 14 N: PASS. `build_selected_packages.sh --group simulacion
simulacion_dron` terminó con código 0 en 0.77 s e instaló el publicador con
14 N y el cierre del launch a los 31 s. Checkpoint de ejecución
`f1_4_4A_14N`: ejecutar `run_simulation.sh` con el perfil vacío, GUI de Gazebo,
un dron, 0 N de 0-10 s, 14 N de 10-30 s y torque nulo. Criterio: marcadores de
inicio, cambio a 14 N y `[F1-ACTUATION-DONE]`, mundo/modelo cargados y cierre
sin errores del publicador.

Resultado `f1_4_4A_14N`: `run_simulation.sh` terminó con código 1 tras los dos
intentos configurados; Gazebo murió durante el arranque antes de poder evaluar
la fuerza de 14 N. Resultado: INVALID por infraestructura, no evaluable como
prueba física. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_f1_4_4A_14N.log`. Siguiente acción
exacta: reducir el log y diagnosticar el primer error real.

Diagnóstico `f1_4_4A_14N`: el publicador no arrancó porque esta versión de ROS
2 Iron no expone `RCLError` en `rclpy.exceptions`; la importación correcta para
capturar el apagado es `rclpy._rclpy_pybind11.RCLError`. Gazebo sí llegó a
cargar `empty.world` y el modelo, pero el script abortó al detectar la muerte
del publicador. Corrección aplicada al nodo auxiliar. Siguiente acción exacta:
recompilar y repetir con 14 N conservando este intento inválido.

Build correctivo de 14 N: PASS. `build_selected_packages.sh --group
simulacion simulacion_dron` terminó con código 0 en 0.78 s. Checkpoint de
ejecución `f1_4_4A_14N_v2`: repetir el mismo perfil y launch con GUI, un dron,
0 N durante 10 s, 14 N durante 20 s, torque nulo y cierre a los 31 s. Criterio:
inicio, cambio a `force_14N`, `[F1-ACTUATION-DONE]`, escenario vacío correcto y
cierre sin errores del publicador.

Resultado `f1_4_4A_14N_v2`: PASS. `run_simulation.sh` terminó con código 0 y
`success=true`; el segundo intento ejecutó el perfil vacío, el publicador
completó la ventana de 14 N y el cierre fue limpio. El log completo y su
reducido se conservan en `codex/archivos_auxiliares/logs/` con el identificador
`prueba_f1_4_4A_14N_v2`.

Resultado de la prueba 4.2: CONSEGUIDA. El exportador auxiliar compilado contra
`gen_tray_veltrap.cpp` produjo `trapezoidal_x.csv` y `triangular_x.csv`. El
generador clasificó el caso de 10 m como trapezoidal (`tf=17.5 s`, pico
`0.8 m/s`) y el de 1 m como triangular (`tf=5.0 s`, pico `0.4 m/s`). La figura
combinada inicial se ha eliminado a petición del usuario y se sustituye por
`perfil_trapezoidal_x.png` y `perfil_triangular_x.png`, cada una con posición,
velocidad y aceleración del único eje x. Como ampliación de la misma prueba,
`gen_tray_pol3.cpp` produjo `cubica_x.csv` y `perfil_cubico_x.png` para 10 m en
5 s, con pico de velocidad `3.0 m/s`. La ampliación por waypoints se repitió
eliminando los tres artefactos anteriores y usando `GenTrayPol3Waypoints` con
`blend=5 s`, cinco destinos y tiempos `12,24,36,48,60 s`; produjo los nuevos
`waypoints_cubica.csv`, `waypoints_cubica_objetivos.csv` y
`waypoints_cubica_xy.png` con curvas claramente pronunciadas. La validación
confirmó z y yaw constantes en todas las muestras. Se conserva esa prueba y se
añadió una ejecución independiente con `blend=1 s`, tiempos `6,12,18,24,30 s`
y color azul, en `waypoints_cubica_blend_1s.csv`,
`waypoints_cubica_blend_1s_objetivos.csv` y
`waypoints_cubica_blend_1s_xy.png`. La comparación confirma el efecto visual
del parámetro. No se modificaron paquetes actuales.
Siguiente acción exacta: revisar con el usuario esta evidencia y acordar la
preparación de la siguiente prueba del Capítulo 4.

Plan histórico anterior:

- crear un perfil YAML separado con `mission_mode`, `navigation_source` y
  `trajectory_file`;
- hacer que `scenario_runner_node` resuelva y ejecute la trayectoria del
  perfil, sin exigir una seleccion independiente desde el script;
- hacer que `trajectory` omita completamente la Fase 6 y que `autonomous`
  respete sus flags funcionales y abra el servidor tras cualquier trayectoria
  completa;
- ejecutar un bootstrap GT monodron y abrir automaticamente la puerta del
  servidor al terminar su ultimo goal;
- conservar `navigation_source` por goal para la prueba GT->ORB;
- usar una mision del servidor con `drones: [1]`;
- actualizar tests y documentacion de los paquetes afectados.

Build `simulacion_dron`: PASS, salida 0, log externo del grupo en
`/home/chenfu/Gazebo/log/simulacion`; incluye launch, runner, perfiles y
escenarios nuevos. Siguiente accion exacta: compilar `task_server` con
`./codex/herramientas/build_selected_packages.sh --group servidor task_server`.
Build `task_server`: PASS, salida 0, log externo del grupo en
`/home/chenfu/Gazebo/log/servidor`; solo conserva warnings deprecated ya
existentes y una advertencia de inicializacion de mensaje. Siguiente accion:
ejecutar contratos dirigidos y revisar el perfil instalado.
Validacion dirigida: `python3 -m py_compile`, `bash -n`, `git diff --check`,
YAML parse y `python3 -m pytest -p no:cacheprovider -q` pasan (`18 passed`).
`ros2 launch --show-args` no pudo arrancar en el sandbox porque ROS intento
crear `~/.ros/log` y despues `/tmp` en un filesystem read-only. El chequeo de
instalacion confirma perfiles, escenarios y la mision monodron instalados.
Siguiente accion exacta: ejecutar el preflight de ROS con permisos escalados y
un directorio de logs temporal escribible.

Preflight ROS: PASS, `ros2 launch simulacion_dron multi_dron.launch.py
--show-args` devuelve los argumentos nuevos y carga los tres workspaces
instalados (`dron`, `servidor`, `simulacion`).

Prueba iniciada y detenida por cambio de requisito: `autonomous_gt_fiducial2`, perfil
`simulacion_dron/config/mission_profiles/autonomous_gt.yaml`, escenario
`simulacion_dron/config/scenarios/autonomous_gt_fiducial2.yaml`, mision
`servidor/task_server/config/mission_house_single_drone.yaml`,
`launch_drone_2:=false`, flags funcionales de Fase 6 activados, GUI desactivada,
timeout 900 s y espera posterior 120 s. El `scenario_runner_node` termino con
codigo 0 y la prueba quedo en la espera posterior; el usuario solicito
detenerla antes de completar esa espera. Resultado experimental: NO
CONCLUIDO; no se clasifica como PASS ni se analizan sus logs.

Acuerdo actualizado por el usuario: el perfil comun incluye tambien la ruta del
YAML de trayectoria. `autonomous` ejecuta toda la trayectoria indicada, aunque
no termine en un fiducial, y solo despues abre el servidor; `trajectory` ejecuta
la misma clase de trayectoria sin iniciar Fase 6. La autorizacion queda
concedida para adaptar launch, runner, script, perfiles, pruebas y
documentacion.

Cambios de esta iteracion completados: `scenario_runner_node` lee
`trajectory_file`, resuelve rutas relativas al perfil, usa la fuente del perfil
como valor por defecto por goal y conserva overrides GT/ORB por goal;
`run_simulation.sh` ya no recibe ni selecciona `--yaml`; los cuatro perfiles
operativos incluyen `trajectory_file`; tests y documentacion estan actualizados.
Validacion estatica: `py_compile`, `bash -n`, YAML parse, `git diff --check` y
los contratos dirigidos pasan (`18 passed`). Siguiente accion exacta: compilar
`simulacion_dron` y validar el ejecutable instalado.

Build `simulacion_dron` tras el cambio de perfil: PASS, salida 0, 29.3 s,
log externo en `/home/chenfu/Gazebo/log/simulacion`. Siguiente accion:
ejecutar preflight ROS y despues las tres simulaciones con perfil, sin
`--yaml` independiente.

Preflight ROS tras instalar perfiles: PASS, `ros2 launch ... --show-args`
expone `mission_profile`, `mission_mode` y `navigation_source`; el perfil y
escenario autonomos existen en install.

Prueba en curso siguiente: `autonomous_gt_profile`, perfil
`simulacion_dron/config/mission_profiles/autonomous_gt.yaml`, trayectoria
seleccionada internamente por `trajectory_file`, mision monodron,
`launch_drone_2:=false`, flags funcionales de Fase 6 activados, GUI desactivada,
timeout 900 s y espera posterior 120 s. Criterio: trayectoria completa,
`[SCENARIO-RUNNER-AUTONOMOUS-HANDOFF]` y actividad posterior del servidor.

Resultado `autonomous_gt_profile`: `run_simulation.sh` termino con codigo 0 y
`success=true` tras un reinicio automatico de Gazebo (intento 0 detecto muerte
temprana; intento 1 completo escenario y espera posterior de 120 s). Siguiente
accion: resultado PASS funcional. El reducido confirma que el perfil selecciono
`autonomous_gt_fiducial2.yaml`, el goal GT de D1 termino correctamente, se
emitio `[SCENARIO-RUNNER-AUTONOMOUS-HANDOFF]` y el servidor produjo actividad
Fase 6 (`F6G`, `F6N`, `F6D` y `F6I`). Queda como incidencia de infraestructura el
reinicio inicial de Gazebo y un `FID-SPAWN-ERROR` del primer intento; el
segundo intento publico readiness y completo la prueba.

Siguiente prueba: `trajectory_gt_profile`, perfil
`simulacion_dron/config/mission_profiles/trajectory_gt.yaml`, trayectoria
seleccionada internamente por `trajectory_file`, misma mision monodron y flags
funcionales de Fase 6 activados para comprobar que `trajectory` los ignora,
timeout 900 s y espera posterior 30 s. Criterio: goals GT completados, sin
handoff y sin procesos `task_server`/`task_manager` de Fase 6.

Resultado `trajectory_gt_profile`: `run_simulation.sh` termino con codigo 0 y
`success=true`, sin reinicio de Gazebo; el scenario runner termino y completo
la espera posterior de 30 s. Siguiente accion exacta: reducir el log y verificar
que no aparecen handoff ni procesos/markers de Fase 6.

Analisis del reducido `trajectory_gt_profile`: los dos goals GT terminaron y no
hubo handoff ni procesos `task_server_node`/`task_manager_node`, pero el dron
registro una inicializacion `F6N-DEPTH-INIT` porque
`depth_observation_enabled` aun recibia el flag bruto. Resultado funcional:
PARCIAL, no se cierra como PASS. Correccion aplicada: profundidad y parada de
profundidad del dron reciben ahora `phase6_enabled`, que es falso en
`trajectory`, y se actualiza el contrato. Siguiente accion: recompilar y
repetir la prueba `trajectory_gt_profile` antes de pasar a GT->ORB.

Build correctivo de `simulacion_dron`: PASS, salida 0, 0.96 s, log externo en
`/home/chenfu/Gazebo/log/simulacion`. Siguiente prueba: repetir
`trajectory_gt_profile` con el mismo perfil y flags, timeout 900 s y espera
posterior 30 s; criterio adicional: ausencia de `F6N-DEPTH-INIT` y de
cualquier nodo/marker de Fase 6.

Resultado `trajectory_gt_profile_v2`: `run_simulation.sh` termino con codigo 0
y `success=true`, sin reinicio de Gazebo; scenario runner y espera posterior
de 30 s completados. Siguiente accion exacta: reducir el log y comprobar la
ausencia de toda actividad Fase 6.

Analisis `trajectory_gt_profile_v2`: PASS. El perfil resolvio
`trajectory_gt.yaml`; ambos goals GT terminaron correctamente, no hubo handoff,
no se lanzaron `task_server_node` ni `task_manager_node`, y el reducido muestra
`F6N-DEPTH-INIT enabled=false`. Los marcadores `F6M-CAMERA-PITCH` pertenecen
al control de pitch de la trayectoria y no implican ejecucion de Fase 6.

Siguiente prueba: `trajectory_gt_orb_profile`, perfil
`simulacion_dron/config/mission_profiles/trajectory_gt_orb.yaml`, trayectoria
seleccionada internamente por `trajectory_file`, mision monodron, flags de Fase
6 activados, timeout 900 s y espera posterior 30 s. Criterio: primer goal GT,
segundo goal ORB, ambos completados, sin handoff ni Fase 6.

Resultado `trajectory_gt_orb_profile`: `run_simulation.sh` termino con codigo 0
y `success=true`, sin reinicio de Gazebo; scenario runner y espera posterior
de 30 s completados. Siguiente accion exacta: reducir el log y comprobar la
secuencia GT->ORB y la ausencia de Fase 6.

Analisis `trajectory_gt_orb_profile`: PASS. El perfil resolvio
`trajectory_gt_orb.yaml`; el primer goal selecciono GT y termino correctamente,
el segundo cambio a ORB y tambien termino correctamente. No hubo handoff,
`task_server_node` ni `task_manager_node`, y `F6N-DEPTH-INIT` registro
`enabled=false`.

Conclusiones agregadas de esta iteracion: `autonomous_gt_profile` PASS
funcional con handoff y actividad Fase 6; `trajectory_gt_profile_v2` PASS con
Fase 6 omitida; `trajectory_gt_orb_profile` PASS con cambio GT->ORB por goal.
La primera ejecucion `trajectory_gt_profile` queda conservada como intento
PARCIAL porque descubrio y motivo el gating adicional de profundidad.
Validacion final especifica: los contratos afectados pasan (`18 passed`),
`py_compile`, `bash -n`, YAML paths y `git diff --check` pasan. La suite amplia
sin `test_pose_metrics.py` obtiene `52 passed, 3 failed` por tres contratos
preexistentes ajenos a esta modificacion: trayectoria auxiliar historica
distinta, dos valores divergentes en perfiles globales y parametros de camara
declarados pero no configurados. La suite completa tampoco puede recolectar
`test_pose_metrics.py` por incompatibilidad externa `matplotlib`/`numpy`
(`numpy 1.21.5`, requiere >=1.23). No se modifican esas zonas.

Estado de esta iteracion: cambios funcionales completados, compilados y
probados con las tres simulaciones acordadas. No queda trabajo activo de esta
subtarea; los logs completos y reducidos de las pruebas se conservan en
`codex/archivos_auxiliares/logs/` para la limpieza posterior solicitada por el
usuario.

Nueva prueba solicitada: trayectoria `trajectory + GT` monodron con cinco
waypoints y yaw fijo 90 grados: `[-10,-10,1]`, `[0,-10,1]`, `[10,-10,1]`,
`[10,-10,3]`, `[-10,-10,3]`. Se crea el perfil
`simulacion_dron/config/mission_profiles/trajectory_gt_wall.yaml` y su
trayectoria asociada. GUI Gazebo, mission GUI y multidron GUI deben permanecer
activadas; espera posterior acordada: 60 s para capturas. Validacion estatica
del nuevo YAML y contratos: PASS (`19 passed`). Siguiente accion exacta:
compilar `simulacion_dron` y ejecutar esta prueba con un solo dron.

Build de `simulacion_dron` para la nueva trayectoria: PASS, salida 0, 1.05 s,
log externo en `/home/chenfu/Gazebo/log/simulacion`. Siguiente prueba:
`trajectory_gt_wall_profile`, perfil
`simulacion_dron/config/mission_profiles/trajectory_gt_wall.yaml`, GUI Gazebo,
mission GUI y multidron GUI activadas, `launch_drone_2:=false`, mision
`mission_house_single_drone.yaml`, timeout 1200 s y espera posterior 60 s.
Criterio: cinco goals GT de D1 completados con yaw 90 grados y cierre limpio.

Nueva variante solicitada antes de ejecutar: conservar los cinco primeros
waypoints de `trajectory_gt_wall` y añadir `[-10,-10,5]` y `[10,-10,5]` como
sexto y septimo goals. Se crea el perfil
`trajectory_gt_wall_high.yaml`; la validacion YAML y los contratos afectados
pasan (`20 passed`). Siguiente accion exacta: recompilar `simulacion_dron` y
ejecutar la variante con las tres GUIs activadas, timeout 1200 s y espera
posterior 60 s.

Nueva prueba acordada: tres cuadrados GT del dron 1 a `z=1`, `z=3` y `z=5`,
con los nueve waypoints de cada cuadrado y la secuencia de yaw absoluto/relativo
indicada por el usuario. Se crea el perfil
`trajectory_gt_square_3_levels.yaml` y un escenario con 27 goals secuenciales.
Validacion YAML y contratos: PASS (`21 passed`). Siguiente accion exacta:
compilar `simulacion_dron` y ejecutar con Gazebo y las tres GUIs activadas,
espera posterior 60 s y timeout 1200 s.

Build de `simulacion_dron` para los tres cuadrados: PASS, salida 0, 1.04 s,
log externo en `/home/chenfu/Gazebo/log/simulacion`. Siguiente prueba:
`trajectory_gt_square_3_levels_profile`, perfil
`simulacion_dron/config/mission_profiles/trajectory_gt_square_3_levels.yaml`,
un solo dron, Gazebo y las tres GUIs activadas, timeout 1200 s y espera
posterior 60 s. Criterio: 27 goals GT completados en los tres niveles y cierre
limpio.

Build de `simulacion_dron` para `trajectory_gt_wall_high`: PASS, salida 0,
0.96 s, log externo en `/home/chenfu/Gazebo/log/simulacion`. Siguiente prueba:
`trajectory_gt_wall_high_profile`, perfil
`simulacion_dron/config/mission_profiles/trajectory_gt_wall_high.yaml`, un
solo dron, Gazebo y las tres GUIs activadas, timeout 1200 s y espera posterior
60 s. Criterio: siete goals GT completados con yaw 90 grados y cierre limpio.

Resultado `trajectory_gt_wall_high_profile`: `run_simulation.sh` termino con
codigo 0 y `success=true`; la trayectoria termino y se mantuvieron Gazebo y las
GUIs durante la espera posterior de 60 s. Siguiente accion exacta: reducir el
log y comprobar los siete resultados GT y la posicion final.

Analisis `trajectory_gt_wall_high_profile`: PASS. El reducido confirma los
siete goals GT de D1, todos con `yaw_deg=90`, en el orden solicitado y con
resultados correctos; el ultimo fue `(10,-10,5)`. `F6N-DEPTH-INIT` quedo en
`enabled=false`, no hubo handoff ni nodos `task_server_node`/`task_manager_node`.
Gazebo anuncio muerte durante la limpieza forzada posterior al cierre limpio
del escenario, pero `run_simulation.sh` termino con `success=true` y codigo 0.
La prueba queda documentada como PASS funcional.

El trabajo experimental F3 queda aparcado y no forma parte de esta limpieza.
No se modifica `codex/archivos_auxiliares` en esta subfase.

Cambios completados en esta limpieza:

- eliminados `.pytest_cache`, `pytest-of-chenfu`, `log`, `.vscode` y el paquete
  legacy `mi_tfg` de la raíz de `src`;
- conservados los tests de CMake y añadido `pytest-of-*/` al `.gitignore`;
- corregido `build_selected_packages.sh` para que `colcon list` use el log
  externo del grupo, igual que `colcon build`;
- actualizados los índices y contratos vigentes que describían `mi_tfg` como
  excepción presente; se conserva su resumen como referencia histórica;
- el `log` externo del workspace padre y `codex/archivos_auxiliares` quedan
  fuera de esta limpieza.

Verificación: `bash -n codex/herramientas/build_selected_packages.sh` y
`git diff --check` pasan. Siguiente acción: limpieza específica de
`codex/archivos_auxiliares` en una conversación separada.

## Reconciliacion con la ultima peticion

- El usuario declara F1 completada. Se consideran recibidos el video GT/Tray y
  las capturas `rqt_graph` monodron/bidron de `f1_doc_visual_007/008`.
- F2 se omite en la campana visual preliminar.
- F3 entra en preparacion. El YAML historico
  `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` no se ejecutara tal cual:
  contiene transiciones explicitas de autoridad ORB propias de pruebas
  posteriores.
- Candidato vigente: `tray_prueba_373.yaml`, que conserva el rodeo completo
  con dos drones, fiduciales y ORB en sombra para mapear, pero fuerza control
  GT en sus 22 goals y no abre fronteras ni activa autoridad ORB.
- El usuario concreta la geometria documental: ambos drones deben visitar los
  cuatro vertices y los cuatro centros de arista del cuadrado
  `[-10,10] x [-10,10]`, en sentidos opuestos y con alturas distintas para no
  colisionar. El yaw no mira al centro: debe conservar rumbo de avance fijo
  durante cada arista y cambiar solo al entrar en el siguiente tramo, igual
  que las pruebas tipicas modernas.
- Se propone un YAML nuevo de documentacion, derivado de la topologia de 373
  pero sin sus barridos extras: dron 1 antihorario y dron 2 horario, alturas
  `1.0 m` y `1.5 m`, parada explicita breve en cada hito. Los yaws y
  `absoluto_yaw` se conservaran del patron validado en `tray_prueba_373.yaml`
  para las aristas equivalentes. Todos los goals conservaran
  `navigation_source: GT`; ORB seguira activo solo como productor de mapas y
  el servidor global permanecera habilitado.
- La ejecucion inicial autorizada empleo RViz para representar `score`. Tras el
  aborto, el usuario pide sustituirlo por el GUI de F7 (`multidron_gui`), que
  consume el campo `score` directamente y aplica su gradiente rojo-amarillo-
  verde, sin efecto en mapa ni control.

## Plan autorizado F3 documental

1. Crear un YAML nuevo con nueve lotes bidron simultaneos: centro sur,
   esquina sur, centro lateral, esquina norte, centro norte y el recorrido
   simetrico de regreso; cada lote tiene parada explicita y conserva los yaws
   y banderas absolutas de `tray_prueba_373.yaml`.
2. Restaurar la configuracion transitoria de RViz y usar `multidron_gui` de F7
   como vista de score, MapPoints, keyframes, drones y fiduciales.
3. Tras una nueva autorizacion, lanzar el perfil F3: Gazebo GUI, GUI F7, bridge
   y navegador `pipeline_flow`, con telemetria F3 detallada desactivada en
   terminal; ORB y servidor global activos para mapear, control GT estricto,
   F6 y el grafo de arquitectura desactivados.
4. Dejar 30 s iniciales para las tres capturas manuales y reducir el log al
   terminar. La presencia de una optimizacion aceptada se observara, pero no
   se forzara ni se modificara 3Q si el runtime la rechaza.

Cambios completados:

- `codex/archivos_auxiliares/trayectorias/f3_documentacion_rodeo_gt_bidron.yaml`:
  nuevo recorrido documental con 18 goals GT, dos alturas y pausas por hito;
- `simulacion_dron/rviz/sparse_global_debug.rviz`: cambio transitorio a score
  de la primera sesion; se restaurara antes de usar el GUI F7 solicitado.

Build F3 documental:

- comando: `./codex/herramientas/build_selected_packages.sh --group simulacion
  simulacion_dron`;
- resultado: `PASS`, salida `0`, un paquete terminado;
- siguiente accion: comprobar el launch instalado, abrir `rqt_graph` con el
  entorno GUI saneado y lanzar la sesion visual documental.

Preflight F3 documental:

- `ros2 launch simulacion_dron multi_dron.launch.py --show-args` pasa con los
  overlays de Dron, Servidor y Simulacion;
- la configuracion RViz instalada contiene `Color Transformer: Intensity`,
  `Channel Name: score` y limites `0/1`;
- el identificador documental `f3_doc_visual_001` no tenia log previo.

Simulacion prevista:

- id documental: `f3_doc_visual_001`;
- YAML: `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/f3_documentacion_rodeo_gt_bidron.yaml`;
- launch: Gazebo, RViz score, `pipeline_flow` y navegador activos; dos drones,
  ORB y servidor global activos para mapeo; control `GT`, `gt_fallback=false`,
  fiduciales activos, F6/GUI de mision/GUI multidron/grafo de arquitectura
  desactivados;
- timeout: `780 s`, espera posterior `15 s`;
- evidencia manual: el usuario graba Gazebo/RViz, captura `rqt_graph` y el
  grafo web durante la espera inicial de 30 s; el mapa sigue recibiendo ORB y
  puede mostrar eventos de score, fusion y optimizacion sin que GT alimente
  estimacion o mapa;
- criterio: los 18 goals GT completan, el servidor/web/RViz arrancan y el log
  contiene publicacion sparse, score y actividad secundaria sin fallo duro.

Resultado preliminar `f3_doc_visual_001`:

- clasificacion: `ABORTED` por el usuario tras el primer fiducial; no se
  completaron los 18 goals ni se usa como evidencia documental final;
- diagnostico ROS live previo al cierre: `/global_mapping/backpressure_active`
  estaba en `true`, `dron_1/AccionTrayectoria` habia perdido su servidor y el
  runner seguia activo, por lo que el siguiente lote no podia progresar;
- el proceso de la simulacion se interrumpio de forma controlada; log completo
  inmutable: `codex/archivos_auxiliares/logs/prueba_f3_doc_visual_001.log`;
- siguiente accion: reducir el log por nodos de dron, backpressure, escenario,
  servidor, errores y shutdown antes de decidir una correccion o repeticion.

Diagnostico `f3_doc_visual_001`:

- el arranque de Gazebo, ORB, servidor global, RViz y `pipeline_flow` fue
  correcto; el navegador quedo en modo `live`;
- los dos primeros goals GT fueron aceptados y ejecutaron
  `[F5H-ATOMIC-GOAL-START]`, pero no llegaron a emitir
  `[SCENARIO-RUNNER-GOAL-RESULT]` antes del aborto;
- durante la detencion el servidor seguia procesando entradas ORB/score y su
  cola primaria alcanzaba al menos `82` pendientes; el topic live de
  backpressure era `true`, por lo que el runner no podia enviar el siguiente
  lote;
- no hay crash previo de servidor ni de Gazebo; el `exit 255` de Gazebo es
  posterior al SIGINT de cierre;
- referencia: `tray_prueba_373.yaml`, con el mismo patron GT/ORB, completo
  `22/22` goals y `SIM-DONE success=true` en `537 s`;
- hipotesis principal: `debug_fase3_logs_terminal=true` imprime telemetria F3
  por cada llegada/commit durante una toma muy cargada y degrada el drenaje.
  El grafo web no necesita ese flag: permanece live con
  `debug_pipeline_flow_web=true`.

Correccion propuesta pendiente de autorizacion:

- repetir el mismo YAML con Gazebo, GUI F7 y web, pero con
  `debug_fase3_logs_terminal=false`; se restaura la configuracion RViz
  transitoria y se inicia `multidron_gui` con retraso breve. No cambia control
  GT, ORB en sombra, fiduciales, servidor, mapa, score ni optimizacion;
- si la repeticion siguiera sin resultados de action, detenerse y discutir un
  segundo diagnostico; no alterar `orb_navigation_prediction_mode`, umbrales
  de backpressure ni runtime 3Q sin acuerdo explicito.

Correccion autorizada para la repeticion:

- el usuario confirma la repeticion con el GUI de F7 en vez de RViz;
- se restaura `sparse_global_debug.rviz` a `RGB8`, pues RViz no interviene en
  esta toma;
- se lanzara `multidron_gui` con un retraso de `12 s`,
  `launch_rviz=false` y `debug_fase3_logs_terminal=false`;
- se preservan Gazebo, el grafo web `pipeline_flow`, dos drones, fiduciales,
  ORB en sombra, servidor global y control GT estricto.

Simulacion en curso:

- id: `f3_doc_visual_002`;
- YAML: `f3_documentacion_rodeo_gt_bidron.yaml`;
- launch: Gazebo GUI y GUI F7 activos, `launch_rviz=false`, `pipeline_flow`
  y su navegador activos, control GT estricto, ORB/servidor/fiduciales/bidron
  activos, F6 y GUI de mision desactivados;
- diagnostico aplicado: `debug_fase3_logs_terminal=false`; espera inicial de
  `30 s`, retraso de GUI F7 de `12 s`, timeout de escenario de `780 s`;
- siguiente accion: registrar la finalizacion, reducir el log y contrastar los
  18 goals con el criterio documental.

Resultado preliminar `f3_doc_visual_002`:

- clasificacion: `FAIL` de arranque; `scenario_runner_node` termino con codigo
  `1` antes de ejecutar la trayectoria;
- causa confirmada: el runner recibio una ruta relativa y no pudo abrir el YAML
  desde su directorio de trabajo (`bad file`); GUI F7 arranco y se cerro de
  forma limpia durante el shutdown;
- la sesion no aporta evidencia visual final ni resultados de goals;
- correccion mecanica autorizada por el acuerdo: relanzar sin cambios
  funcionales, pasando la ruta absoluta del mismo YAML.

Simulacion en curso `f3_doc_visual_003`:

- repite exactamente el perfil de `f3_doc_visual_002`, incluida la GUI F7 y
  `debug_fase3_logs_terminal=false`;
- unica diferencia: `scenario_file` recibe la ruta absoluta
  `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/f3_documentacion_rodeo_gt_bidron.yaml`;
- siguiente accion: reducir el resultado y comprobar los 18 goals GT, actividad
  de mapa/score y disponibilidad de GUI F7/grafo web.

Cambio de alcance solicitado durante `f3_doc_visual_003`:

- el usuario solicita que la misma toma F3 muestre la imagen anotada cuando
  cada dron detecte un AprilTag, para ahorrar una toma adicional;
- la sesion `003` se interrumpio durante el inicio del escenario, sin
  resultados utilizables, pues fue lanzada con
  `debug_fiducial_visualization=false` y no podia generar ese material;
- propuesta confirmada por el usuario: repetir el mismo perfil con
  `debug_fiducial_visualization=true` y
  `debug_fiducial_display_seconds=8.0`. Esto inicia un
  `fiducial_visualizer` aislado por dron, que consume
  `/dron_N/orbslam/fiducial_debug/image`; no altera deteccion, control GT,
  ORB, servidor, mapa ni GUI F7.

Simulacion prevista `f3_doc_visual_004`:

- mismo YAML GT bidron y perfil de `003`, con GUI F7 y `pipeline_flow`;
- ruta absoluta al YAML, `debug_fase3_logs_terminal=false`,
  `debug_fiducial_visualization=true` y retencion de imagen de `8 s`;
- siguiente accion: ejecutar la toma, reducir el log y comprobar los 18 goals
  junto con la evidencia de mapa, score, GUI F7, web y AprilTag.

Resultado preliminar `f3_doc_visual_004`:

- clasificacion: `ABORTED` por el usuario tras comprobar que ambos drones
  llegaron al primer fiducial y no iniciaron el siguiente lote;
- la sesion tenia GUI F7, web e imagenes AprilTag activadas, pero no sirve como
  evidencia final porque no completo el recorrido;
- la primera hipotesis, desactivar la telemetria detallada F3 de terminal, no
  resolvio el bloqueo;
- el shutdown requirio SIGINT, SIGTERM y SIGKILL al quedar el runner bloqueado;
- siguiente accion: reducir el log por escenario, resultado de goals,
  `backpressure`, cola global y servidores de accion antes de proponer otra
  correccion.

Diagnostico confirmado `f3_doc_visual_004`:

- el lote `ambos_al_centro_sur` completo correctamente: ambos goals GT
  devolvieron `success=true` y `t_total=40 s`;
- la imagen AprilTag funciono: los dos `fiducial_visualizer` quedaron READY y
  recibieron/publicaron frames anotados de ambos drones;
- durante la pausa de 10 s posterior, el runner recibio
  `/global_mapping/backpressure_active=true`;
- al iniciar `esquinas_sur_opuestas`, el runner emitio
  `MOVE-GATE-WAIT active=true` y no despacho el siguiente lote antes de la
  interrupcion. No hubo fallo de GT, de acciones ni de GUI;
- por tanto el bloqueo observado es la politica normal de gate F3, no la
  telemetria de terminal, F7 ni el visualizador AprilTag.

Propuesta pendiente de autorizacion:

- anadir al scenario runner una politica YAML opt-in
  `gate_mapping_backpressure: false`, conservando `true` por defecto;
- solo el YAML documental F3 usaria `false`: el servidor seguiria procesando,
  publicando score, optimizaciones, backpressure y grafo web, pero el runner
  no retendria el siguiente goal GT por esa senal;
- riesgo aceptable solo para esta evidencia no formal: puede crecer el backlog
  y alguna optimizacion terminar tarde. Se monitorizara y se abortara ante
  degradacion real; no se cambiara ningun umbral ni la semantica del servidor.

Correccion autorizada e implementada:

- `scenario_runner_node` incorpora `gate_mapping_backpressure`, `true` por
  defecto como parametro ROS y como comportamiento cuando el YAML no lo
  declara;
- `f3_documentacion_rodeo_gt_bidron.yaml` es el unico escenario que fija
  `gate_mapping_backpressure: false`; el servidor conserva score,
  optimizaciones, publicacion de backpressure y grafo web;
- el bypass se registra como `SCENARIO-RUNNER-MOVE-GATE-BYPASS` y queda descrito
  en la documentacion de paquete;
- build previsto: `./codex/herramientas/build_selected_packages.sh --group
  simulacion simulacion_dron`; si pasa, se relanzara la toma con GUI F7, web e
  imagenes AprilTag.

Build del override documental:

- comando: `./codex/herramientas/build_selected_packages.sh --group simulacion
  simulacion_dron`;
- resultado: `PASS`, salida `0`, un paquete terminado;
- siguiente accion: ejecutar `f3_doc_visual_005` con el YAML que declara el
  bypass opt-in y verificar en el log el marcador correspondiente, los 18
  goals GT y la evidencia visual.

Simulacion prevista `f3_doc_visual_005`:

- YAML: `f3_documentacion_rodeo_gt_bidron.yaml` con
  `gate_mapping_backpressure=false`;
- launch: Gazebo, GUI F7, `pipeline_flow`, dos visualizadores AprilTag,
  dos drones, ORB/servidor/fiduciales activos y control GT estricto;
- espera inicial `30 s`, timeout de escenario `780 s`; siguiente accion:
  reducir el log por bypass, goals, mapa y AprilTag tras la ejecucion.

Resultado preliminar `f3_doc_visual_005`:

- clasificacion: `ABORTED` por peticion del usuario; `run_simulation.sh` fue
  interrumpido durante el recorrido y limpio el launch con señales escaladas;
- no se usa como evidencia documental completa hasta reducir y confirmar los
  lotes que alcanzo a ejecutar;
- siguiente accion: conservar el log completo, generar reduccion tematica y
  mantener el trabajo F3 abierto a la espera de la siguiente indicacion del
  usuario.

Analisis `f3_doc_visual_005`:

- el override se aplico: con `backpressure=true`, el runner emitio
  `MOVE-GATE-BYPASS active=true` y despacho los lotes posteriores;
- evidencia positiva: completaron con `success=true`, `t_total=40 s` los
  cuatro lotes GT bidron `centro_sur`, `esquinas_sur`, `centros_laterales`
  y `esquinas_norte` (8 de los 18 goals totales);
- ambos visualizadores AprilTag quedaron READY y mostraron multiples frames
  anotados con tags decodificados;
- la interrupcion ocurrio durante la pausa posterior a las esquinas norte;
  por tanto no hay fallo tecnico del override, pero no existe recorrido F3
  completo ni resultado `SIM-DONE`;
- trabajo F3 sigue abierto a la espera de que el usuario decida si conservar
  esta evidencia parcial, relanzar una toma completa o ajustar el guion.

## Cambio de foco acordado por el usuario

- El proyecto esta casi terminado.
- No se hara Fase 8 ni Fase 9.
- Fase 6 esta casi terminada, pero no se terminara en este chat.
- El objetivo de este chat es ejecutar y documentar pruebas por fase para una
  buena documentacion del TFG.
- El zip `TFG_CAMPANA_FORMAL_F1_F5_v10.zip`, ubicado en la raiz de `src/`,
  define como proceder para la campana formal.
- Antes de esa campana se generara una documentacion visual preliminar de
  F1-F5: un `rqt_graph` por fase, un `rqt_graph` integrado, un video breve por
  fase y capturas de los grafos web aplicables y operativos.
- Esta evidencia sera descriptiva y no sustituye ni consume ejecuciones de las
  158 pruebas formales del zip.

## Guion visual propuesto por el usuario

- F1: un dron en movimiento y grafica temporal GT frente a referencia `Tray`.
- F2: no se grabara video.
- F3: vuelta al edificio, score de MapPoints, optimizaciones y grafo web.
- F4: dron hacia un fiducial, activacion visual AprilTag y error entre GT y la
  pose deducida visualmente del tag.
- F5: aproximacion al fiducial con GT y, tras el anclaje, varios movimientos
  con autoridad ORB.

La comparacion F4 debe hacerse en el mismo frame y timestamp: GT se usara solo
como metrica externa pasiva, transformada a la pose de camara/tag equivalente;
nunca alimentara deteccion, anclaje, control ni optimizacion.

## Preparacion concreta F1 acordada

Entregables previstos, fuera de la campana formal:

- `rqt_graph` con un unico dron F1 y sin ORB, servidor global, fiduciales, F6
  ni GUIs adicionales.
- `rqt_graph` con dos drones F1 y namespaces independientes, bajo el mismo
  perfil aislado.
- video de un dron siguiendo una secuencia corta con una ventana de cuatro
  paneles `GT` frente a `Tray` (`x`, `y`, `z`, `yaw`).

Hallazgos de preparacion:

- `multi_dron.launch.py` ya permite seleccionar `launch_drone_N`, pero inicia
  siempre ORB-SLAM3 y el servidor global; se propone anadir puertas de launch
  con default conservador `true` para propagacion `activar_orbslam` y para
  incluir/omitir el servidor.
- La instrumentacion existente `graficar_GTvsTray` es legacy: publica solo
  error, usa topics globales `/numeric_array` y `/labels_array` y no ofrece
  una comparacion visual GT/Tray por dron. No sirve como evidencia limpia para
  este video.
- Se propone adaptar esa fuente a topics relativos namespaced y crear un
  plotter especifico no bloqueante con cuatro paneles. Solo observa GT y
  feedback de `TrayAction`; no cambia consignas ni control.
- Se preparara un YAML corto para `scenario_runner_node`, con control GT,
  subida segura y desplazamiento corto. El escenario se ejecutara una vez
  para el video y con movimientos independientes de ambos drones para la
  captura de dos namespaces.
- `rqt_graph` esta instalado; no hay `ffmpeg` ni `wf-recorder`, pero GNOME
  esta disponible bajo Wayland. La grabacion se hara manualmente con su
  capturador nativo cuando Codex indique el instante; el usuario confirmara
  `LISTO` tras detenerla.

Archivos previsibles tras autorizacion:

```text
simulacion/simulacion_dron/launch/multi_dron.launch.py
simulacion/simulacion_dron/src/graficar/graficar_GTvsTray.cpp
simulacion/simulacion_dron/src/graficar/<plotter_f1_gt_tray>.py
simulacion/simulacion_dron/CMakeLists.txt
codex/archivos_auxiliares/trayectorias/<escenarios_f1_documentacion>.yaml
codex/contexto/paquetes/simulacion_dron/{00_summary.md,launches.md,graficas_y_gui.md}
```

## Lectura realizada del zip

Documentos raiz leidos:

- `00_README_V10.md`
- `01_ALCANCE_Y_FILOSOFIA.md`
- `04_EVIDENCIAS_LOGS_CAPTURAS.md`
- `05_EJECUCION_CODEX_USUARIO_Y_SEGURIDAD.md`
- `06_PRE1_ENTORNO_Y_REPRODUCIBILIDAD.md`
- `07_EVIDENCIA_HISTORICA_Y_REPRODUCCION.md`
- `08_SUT_HARNESS_Y_INTEGRIDAD.md`
- `09_DECISIONES_GLOBALES_CERRADAS.md`
- `11_TOOLING_VIGENTE_Y_CONFLICTOS_DOCUMENTALES.md`
- `PACKAGE_METADATA.json`
- `plantillas/PLANTILLA_MANIFEST_EJECUCION.json`

Matrices leidas:

- F1: `F1/09_MATRIZ_PRUEBAS.md`, 35 pruebas.
- F2: `F2/14_MATRIZ_PRUEBAS_V4.md`, 24 pruebas.
- F3: `F3/21_MATRIZ_PRUEBAS_V6.md`, 37 pruebas.
- F4: `F4/21_MATRIZ_PRUEBAS_V8.md`, 27 pruebas.
- F5: `F5/26_MATRIZ_PRUEBAS_V10.md`, 35 pruebas.

Total formal definido por el paquete: 158 pruebas. El paquete declara que no se
ha ejecutado ninguna prueba formal todavia.

## Reglas operativas de la campana

- Alcance formal: `PRE.1 + F1-F5 + regresiones transversales F1-F5`.
- F6-F9 quedan fuera del alcance de esta campana; F6 se conserva parcial y no
  se cerrara aqui salvo nueva orden explicita.
- Cada prueba se discute con el usuario antes de modificar o ejecutar.
- Cada intento consume un ID runtime unico `prueba_N`; no se sobrescribe una
  ejecucion cerrada.
- `run_simulation.sh` trunca `prueba_N.log`, por lo que antes de ejecutar debe
  comprobarse o implementarse una reserva de ID.
- Los logs completos son artefactos inmutables: nunca se abren ni se vuelcan
  manualmente; solo se leen reducidos o salidas de parsers controlados.
- Si una prueba necesita evidencia manual, Codex debe explicar capturas/videos
  exactos y esperar `LISTO`.
- Un fallo de infraestructura antes de ejercer el fenomeno estudiado es
  `INVALID`, no fallo funcional del SUT.
- Estados de resultado: `PASS`, `FAIL`, `INVALID`, `INCONCLUSIVE`, `ABORTED`.
- Comportamiento del sistema se registra aparte:
  `NOMINAL`, `EXPECTED_FAILURE`, `UNEXPECTED_FAILURE`.
- La evidencia pesada queda fuera de Git bajo `TFG_EVIDENCIAS_ROOT`
  (default `~/TFG_EVIDENCIAS`) con hashes y manifiestos.
- No se modifica Overleaf ni se hace push sin orden explicita.

## Plataforma formal

Plataforma objetivo declarada por PRE.1:

- Ubuntu 22.04.5 LTS
- ROS 2 Iron
- Gazebo Classic 11.10.2
- C++
- ORB-SLAM3 estereo
- dos camaras funcionales de percepcion
- sin IMU
- sin GPS
- un unico PC

PRE.1 debe generar como minimo `environment.md` y `environment.json`, registrar
herramientas/dependencias y localizar `ORBvoc.txt` con ruta, tamano y SHA256.
Si `ROS_DISTRO` no es `iron`, la ejecucion formal se detiene como configuracion
invalida.

## Estado de campana por fases

- F1: 35 pruebas permanentes definidas.
- F2: 24 runbooks definidos.
- F3: 37 runbooks definidos; 3Q queda reabierta para validacion formal, no como
  requisito global previo de F1.
- F4: 27 runbooks definidos; core 4A-4H, 4I opcional historica fuera de core.
- F5: 35 runbooks definidos; PASS fuerte con ORB, `gt_fallback_enabled=false`.

## Siguiente accion exacta

Autorizacion recibida: usuario confirma "Perfecto, hazlo asi".

Plan autorizado:

1. anadir a `multi_dron.launch.py` las puertas `activar_orbslam` y
   `launch_global_server`, activas por defecto, y propagar la primera a cada
   `generar_dron.launch.py`;
2. sustituir la salida global legacy de `graficar_GTvsTray` por una salida
   namespaced, segura ante datos incompletos, y anadir un plotter pasivo de
   cuatro paneles GT/Tray;
3. crear escenarios cortos para video monodron y actividad bidron;
4. compilar solamente `simulacion_dron`, corregir de forma mecanica si falla;
5. validar el perfil F1 monodron antes de pedir la grabacion y capturas
   manuales, sin usar IDs de la campana formal.

Cambios completados:

- `multi_dron.launch.py`: puertas de aislamiento y nodos opcionales de F1;
- `graficar_GTvsTray.cpp`: salida GT/Tray namespaced sin topics globales;
- `fase1_gt_tray_plotter.py`: cuatro paneles pasivos;
- instalacion CMake y YAMLs de video/bidron.

Siguiente accion exacta: ejecutar
`./codex/herramientas/build_selected_packages.sh --group simulacion simulacion_dron`.

Build F1:

- comando: `./codex/herramientas/build_selected_packages.sh --group simulacion simulacion_dron`;
- resultado: `PASS`, salida `0`, un paquete terminado;
- log completo: `codex/archivos_auxiliares/colcon_build.log` (no inspeccionado,
  pues el build paso);
- siguiente accion: comprobar instalacion y argumentos del launch, luego
  validar arranque monodron aislado sin capturar evidencia aun.

Preflight F1:

- los cuatro argumentos nuevos aparecen en `ros2 launch ... --show-args`;
- el ejecutable C++ y el plotter Python estan instalados;
- un intento de inspeccion previo fue bloqueado por el sandbox al crear un log
  ROS bajo `~/.ros`; al redirigir `ROS_LOG_DIR` al workspace se confirmo que no
  hay fallo del launch.

Simulacion prevista:

- id documental: `f1_doc_smoke_001` (no pertenece a la numeracion formal);
- YAML: `codex/archivos_auxiliares/trayectorias/f1_documentacion_video.yaml`;
- launch: un dron con Gazebo headless y todas las capas F2-F6/visualizadores
  fuera (`activar_orbslam=false`, `launch_global_server=false`,
  `spawn_fiducials=false`, `launch_phase6=false`, `launch_drone_2=false`);
- timeout: 120 s, espera posterior: 3 s;
- criterio: escenario completo, accion GT correcta y ausencia de nodos ORB,
  servidor, fiduciales y F6 en el launch;
- siguiente accion: reducir el log con marcadores `SCENARIO-RUNNER`,
  `SIM-*`, errores y nodos F1.

Resultado de `f1_doc_smoke_001`:

- resultado: `FAIL` tecnico, salida `1`;
- Gazebo/launch llegaron a arrancar, pero `scenario_runner_node` termino con
  codigo `1`;
- log completo inmutable:
  `codex/archivos_auxiliares/logs/prueba_f1_doc_smoke_001.log`;
- no se capturo evidencia visual y este intento no cuenta como prueba formal;
- siguiente accion: reducir ese log y diagnosticar el primer error real antes
  de repetir o cambiar el escenario.

Diagnostico `f1_doc_smoke_001`:

- causa: el runner se ejecuta desde `/home/chenfu/Gazebo` y recibio el YAML
  relativo a `src/`; por eso `YAML::LoadFile` no encontro el archivo;
- el launch aislado fue correcto hasta ese punto: aparecieron exclusivamente
  `gzserver`, `clock`, `generador_URDF`, `navigation_state_mux`, `gen_tray`,
  `control_calcular_fuerzas` y `aplicar_fuerzas_dron` de F1;
- correccion mecanica autorizada: usar ruta absoluta de YAML al invocar el
  runner, sin cambiar comportamiento, escenario ni criterios.

Repeticion prevista:

- id documental nuevo: `f1_doc_smoke_002`;
- mismo perfil y criterios, con
  `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/f1_documentacion_video.yaml`;
- siguiente accion: reducir el log de la repeticion y analizar resultado.

Resultado de `f1_doc_smoke_002`:

- resultado preliminar: `FAIL` tecnico, `scenario_runner_node` devolvio `1`;
- no quedaron procesos `gzserver`, `ros2 launch` ni runner tras el cleanup;
- log completo inmutable:
  `codex/archivos_auxiliares/logs/prueba_f1_doc_smoke_002.log`;
- siguiente accion: reducir y diagnosticar el log antes de otra repeticion.

Diagnostico `f1_doc_smoke_002`:

- YAML leido y runner operativo; el primer goal fue rechazado por
  `reject_no_state`;
- causa: el perfil arrancaba `phase5_navigation_source=orb` aunque ORB estaba
  aislado, y ademas usaba goals absolutos, que requieren frame global de F5;
- correccion mecanica dentro del acuerdo F1: los escenarios ahora usan goals
  relativos y la repeticion arrancara explicitamente en fuente `GT`;
- no se modifica control, mux, politica de goals ni ningun YAML operativo.

Repeticion prevista:

- id documental nuevo: `f1_doc_smoke_003`;
- mismo aislamiento, con `phase5_navigation_source:=gt`, YAML de video
  corregido y ruta absoluta;
- criterio: al menos dos goals relativos GT aceptados/completados, sin ORB,
  servidor, fiduciales ni F6;
- siguiente accion: reducir y analizar el log antes de la sesion visual.

Resultado de `f1_doc_smoke_003`:

- ejecucion terminada y cleanup sin procesos residuales detectados;
- el resultado funcional queda pendiente de reducir el log (la salida de la
  herramienta se interrumpio mientras el escenario seguia activo);
- log completo inmutable:
  `codex/archivos_auxiliares/logs/prueba_f1_doc_smoke_003.log`;
- siguiente accion: reducir y clasificar los goals, el cierre y el aislamiento.

Analisis `f1_doc_smoke_003`:

- resultado: `PASS`; `SIM-DONE success=true`;
- ambos goals relativos GT de `dron_1` completaron (`t_total=5` y `8 s`);
- aislamiento confirmado por el launch: no aparecieron ORB, servidor global,
  fiduciales, F6 ni GUI adicional; solo cadena F1 y mux de navegacion en modo
  GT;
- intento `001` (ruta YAML) y `002` (fuente/absolutos F5) se conservan como
  diagnosticos no formales; el perfil vigente usa ruta absoluta, fuente GT y
  goals relativos.

Simulacion prevista:

- id documental: `f1_doc_smoke_004`;
- YAML: `f1_documentacion_dos_drones.yaml`;
- mismo perfil F1 aislado, dos drones y `phase5_navigation_source:=gt`;
- criterio: dos goals GT simultaneos completos y dos ramas activas, antes de
  solicitar la captura manual `rqt_graph`.

Resultado de `f1_doc_smoke_004`:

- ejecucion terminada y no se detectan procesos residuales;
- resultado funcional pendiente de reducir el log;
- log completo inmutable:
  `codex/archivos_auxiliares/logs/prueba_f1_doc_smoke_004.log`;
- siguiente accion: reducir y clasificar ambos goals antes de la sesion visual.

Analisis `f1_doc_smoke_004`:

- resultado: `PASS`; `SIM-DONE success=true`;
- `dron_1` y `dron_2` completaron de forma simultanea sus goals GT relativos
  en 5 s; las ramas namespaced son independientes;
- el perfil F1 aislado queda tecnicamente listo para capturas manuales.

Preparacion de sesion visual:

- el YAML monodron reserva ahora 25 s de preparacion visual antes del primer
  movimiento;
- se lanzara Gazebo GUI y el plotter GT/Tray, siempre con ORB, servidor,
  fiduciales, F6 y GUIs adicionales desactivados;
- durante esa ventana el usuario abrira `rqt_graph`, ocultara solo
  `/rosout` y `/parameter_events`, capturara el grafo monodron y pulsara
  `Ctrl+Alt+Shift+R` para iniciar el video; confirmara `LISTO` despues de
  detener la grabacion.

Sesion visual monodron autorizada por el usuario con "Perfecto, hazlo":

- id documental: `f1_doc_visual_005`;
- YAML: `f1_documentacion_video.yaml`, ruta absoluta;
- launch: Gazebo GUI y plotter GT/Tray activos; ORB, servidor global,
  fiduciales, F6, RViz2 y GUIs adicionales desactivados;
- preparacion humana: ventana de 25 s antes del movimiento para capturar
  `rqt_graph` y arrancar GNOME ScreenCast con `Ctrl+Alt+Shift+R`;
- criterio: video/captura manual correctamente encuadrados y escenario con dos
  goals GT exitosos;
- siguiente accion: abrir `rqt_graph`, lanzar la sesion y esperar la
  confirmacion manual de captura antes de cerrar/documentar.

Incidencia visual previa al arranque:

- `rqt_graph` fallo antes de abrir por bibliotecas GTK/GIO heredadas de Snap
  VS Code, que cargaban `libpthread` incompatible;
- correccion mecanica: invocar `rqt_graph` con entorno limpio y entregar al
  plotter F1 el mismo `rviz_environment` saneado que ya usa el launch para
  visualizadores;
- no afecta a nodos ROS, aislamiento ni escenario; siguiente accion: reabrir
  `rqt_graph` saneado y lanzar la sesion visual registrada.

Estado de captura manual:

- `rqt_graph` abierto correctamente con entorno limpio y
  `ROS_LOG_DIR=codex/archivos_auxiliares/ros_logs`;
- se inicia `f1_doc_visual_005` con `launch_gazebo_gui=true` y
  `enable_fase1_gt_tray_plot=true`;
- el usuario dispone de 12 s de startup mas 31 s de espera YAML antes del
  primer goal para guardar la captura monodron e iniciar GNOME ScreenCast;
- siguiente accion: esperar la terminacion del escenario, reducir el log y
  esperar confirmacion del usuario sobre video/captura antes de cerrar la
  sesion de `rqt_graph`.

Resultado preliminar `f1_doc_visual_005`:

- `run_simulation.sh` detecto salida temprana de la sesion grafica y ejecuto
  el reintento automatico; tampoco quedaron procesos al finalizar;
- no se clasifica aun como fallo funcional de F1: falta reducir el log para
  separar infraestructura GUI de la simulacion/control;
- log completo inmutable:
  `codex/archivos_auxiliares/logs/prueba_f1_doc_visual_005.log`;
- siguiente accion: reducir por `gzclient`, plotter, Python/Qt y marcadores
  SIM antes de reparar o repetir.

Diagnostico `f1_doc_visual_005`:

- clasificacion: `INVALID` de infraestructura previa al ejercicio funcional;
- causa: `fase1_gt_tray_plotter.py` estaba instalado pero no tenia permiso de
  ejecucion, por lo que ROS rechazo el ejecutable y el launch no llego a crear
  Gazebo;
- `rqt_graph` saneado sigue abierto y no se perdio evidencia funcional;
- correccion mecanica autorizada: marcar el script ejecutable, verificarlo y
  repetir con id documental nuevo sin modificar codigo, escenario o perfil.

Repeticion visual prevista:

- id: `f1_doc_visual_006`;
- mismo YAML, perfil y ventana de preparacion manual;
- siguiente accion: corregir el permiso de ejecucion y lanzar de nuevo.

Preflight de `f1_doc_visual_006`:

- `fase1_gt_tray_plotter.py` es ejecutable tanto en fuente como por el enlace
  instalado;
- `rqt_graph` permanece abierto con entorno saneado;
- se reutiliza el mismo perfil, YAML y criterio de la sesion `005`;
- siguiente accion: lanzar la repeticion y esperar la evidencia manual.

Resultado preliminar `f1_doc_visual_006`:

- el helper detecto un patron de error durante el primer arranque visual y
  reintento automaticamente; no quedan procesos al finalizar;
- log completo inmutable:
  `codex/archivos_auxiliares/logs/prueba_f1_doc_visual_006.log`;
- siguiente accion: reducir por error de Python/Matplotlib/GUI y clasificar la
  infraestructura antes de repetir.

Diagnostico `f1_doc_visual_006`:

- clasificacion: `INVALID` de infraestructura previa al ejercicio funcional;
- causa: el entorno saneado seguia habilitando los paquetes Python del usuario,
  que cargaban `matplotlib` incompatible con el `numpy` del sistema;
- verificacion: con `PYTHONNOUSERSITE=1`, Python usa `numpy 1.21.5` y
  `matplotlib 3.5.1`, ambos compatibles;
- correccion mecanica autorizada: aplicar `PYTHONNOUSERSITE=1` exclusivamente
  al plotter F1, sin afectar a Gazebo, ROS ni a los visualizadores restantes;
- siguiente accion: comprobar la sintaxis del launch y repetir la sesion visual
  con un identificador documental nuevo.

Preflight posterior al arreglo:

- la sintaxis del plotter Python es valida;
- una invocacion de `ros2 launch --show-args` sin superponer el workspace no
  encontro `simulacion_dron`; es un error del entorno de comprobacion, no del
  launch ni de F1;
- el archivo de launch se instala mediante CMake, por lo que se recompilara
  `simulacion_dron` antes de repetir.

Build previsto:

- comando: `./codex/herramientas/build_selected_packages.sh --group simulacion
  simulacion_dron`;
- objetivo: instalar el entorno exclusivo del plotter F1;
- siguiente accion: validar el launch con el workspace superpuesto y lanzar
  `f1_doc_visual_007` si el build termina correctamente.

Resultado del build posterior al arreglo:

- resultado: `PASS`, salida `0`, un paquete terminado (`simulacion_dron`);
- el launch instalado ya contiene el entorno exclusivo del plotter;
- siguiente accion: validar argumentos con los overlays de ROS y simulacion
  superpuestos, despues registrar y lanzar la sesion visual `007`.

Preflight visual posterior al build:

- `ros2 launch simulacion_dron multi_dron.launch.py --show-args` pasa con los
  overlays `dron`, `servidor` y `simulacion` superpuestos;
- las puertas de aislamiento y `enable_fase1_gt_tray_plot` estan presentes;
- la sesion previa no dejo procesos de Gazebo o simulacion residuales.

Simulacion prevista:

- id documental: `f1_doc_visual_007`;
- YAML: `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/f1_documentacion_video.yaml`;
- launch: Gazebo GUI y plotter GT/Tray; un dron; ORB, servidor global,
  fiduciales, F6, RViz2 y GUIs adicionales desactivados; fuente de navegacion
  `gt`;
- tiempo: 12 s de startup, 25 s de preparacion humana y dos goals de 5 y 8 s;
- criterio: ventana de grafica operativa y `SIM-DONE success=true`, dejando al
  usuario capturar `rqt_graph` y grabar el movimiento;
- siguiente accion: lanzar, registrar salida y reducir solo el log resultante.

Resultado preliminar `f1_doc_visual_007`:

- salida del runner: `0`; marcador `SIM-DONE success=true`;
- el escenario completo se ejecuto y el helper realizo su limpieza normal;
- log completo inmutable:
  `codex/archivos_auxiliares/logs/prueba_f1_doc_visual_007.log`;
- siguiente accion: generar un reducido con marcadores de escenario, plotter,
  Python y errores antes de clasificar la sesion visual.

Analisis `f1_doc_visual_007`:

- resultado tecnico: `PASS`; el adaptador y plotter F1 arrancaron y los dos
  goals GT completaron con `success=true` (`5 s` y `8 s`);
- aislamiento confirmado: el comando no habilita ORB, servidor global,
  fiduciales, F6, RViz2 ni GUIs auxiliares;
- la unica traza ocurrio durante el `SIGINT` de cleanup, despues de
  `SIM-DONE`; es un cierre normal de `rclpy`, no un fallo funcional;
- se aplica una correccion mecanica al plotter para silenciar esa traza en las
  proximas sesiones. La captura manual y el video realizados en `007` siguen
  siendo evidencia valida si el usuario los guardo;
- siguiente accion: recompilar el plotter y preparar la sesion bidron para el
  segundo `rqt_graph`.

Resultado del build de cierre del plotter:

- resultado: `PASS`, salida `0`, un paquete terminado (`simulacion_dron`);
- el manejador de cierre ordenado del plotter queda instalado para las sesiones
  posteriores.

Preparacion de captura bidron:

- `f1_documentacion_dos_drones.yaml` incorpora una espera documental de 25 s
  tras los 6 s de arranque y antes de los movimientos simultaneos;
- la modificacion solo amplia el tiempo disponible para capturar el
  `rqt_graph`; no altera goals, fuente GT, control ni namespaces.

Simulacion prevista:

- id documental: `f1_doc_visual_008`;
- YAML: `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/f1_documentacion_dos_drones.yaml`;
- launch: Gazebo GUI, dos drones, fuentes GT y sin plotter/ORB/servidor
  global/fiduciales/F6/RViz2/GUIs adicionales;
- criterio: dos namespaces visibles en `rqt_graph` y dos goals GT simultaneos
  completados;
- siguiente accion: lanzar y permitir la captura manual durante la espera.

Resultado preliminar `f1_doc_visual_008`:

- salida del runner: `0`; marcador `SIM-DONE success=true`;
- la sesion visual bidron termino y se aplico la limpieza normal del launch;
- log completo inmutable:
  `codex/archivos_auxiliares/logs/prueba_f1_doc_visual_008.log`;
- siguiente accion: generar y analizar el log reducido para clasificar los dos
  goals y el aislamiento antes de cerrar esta preparacion F1.

Analisis `f1_doc_visual_008`:

- resultado tecnico: `PASS`; `dron_1` y `dron_2` iniciaron ramas namespaced
  independientes y completaron sus goals GT simultaneos en `5 s`;
- aislamiento confirmado por el comando: sin ORB, servidor global, fiduciales,
  F6, RViz2, plotter ni GUIs auxiliares;
- el escenario ofrecio 25 s de preparacion para la captura manual del
  `rqt_graph` antes del movimiento;
- no se observaron errores Python ni fallos de nodo durante la ejecucion.

Estado agregado de F1 preliminar:

- implementacion y build: `PASS`;
- perfil aislado monodron y bidron: `PASS`;
- video GT frente a `Tray`: tecnicamente ejecutado en `f1_doc_visual_007`;
- capturas manuales esperadas: `rqt_graph` monodron (`007`) y bidron (`008`),
  mas el video de `007`; confirmar con el usuario que se guardaron antes de
  declarar la evidencia manual recibida;
- las ejecuciones `001`, `002`, `005` y `006` se conservan como `INVALID` de
  infraestructura/preparacion; no pertenecen a la campana formal;
- siguiente accion: esperar confirmacion de las tres capturas manuales o
  relanzar solo la sesion que el usuario indique que falto.

Checkpoint de reanudacion: la prueba `trajectory_gt_square_3_levels_profile`
ha terminado. `run_simulation.sh` devolvio `SIM-DONE success=true` y salida
final `0`; el scenario runner devolvio codigo `0` y se completaron los 60 s de
espera posterior con las GUIs abiertas antes de la limpieza. Log completo:
`codex/archivos_auxiliares/logs/prueba_trajectory_gt_square_3_levels_profile.log`.
Siguiente accion exacta: ejecutar `reduce_simulation_log.sh` y leer solo el
reducido para confirmar los 27 goals GT, los tres niveles, la secuencia de yaw
y la ausencia de Fase 6.

Resultado `trajectory_gt_square_3_levels_profile`: `run_simulation.sh`
termino con salida `0`, `SIM-DONE success=true` y limpieza final completada.
El scenario runner devolvio codigo `0` y se mantuvo la espera posterior de 60 s
con Gazebo, mission GUI y multidron GUI activadas. El reducido confirma 27
`GOAL-SEND` y 27 `GOAL-RESULT success=true` para `dron_1`, distribuidos en los
tres cuadrados de `z=1`, `z=3` y `z=5`, con la secuencia de yaw solicitada;
tambien confirma `SCENARIO-RUNNER-DONE success=true`, perfil `trajectory`,
`navigation_source=gt` y `F6N-DEPTH-INIT enabled=false`. No aparecen
`AUTONOMOUS-HANDOFF`, `task_server_node` ni `task_manager_node`. El reducido
conserva un `process has died` de Gazebo durante la limpieza posterior a
`SIM-DONE`; no afecta al resultado funcional. Resultado: PASS. No queda
trabajo activo de esta prueba.

Nueva prueba acordada por el usuario: sustituir la separación vertical de 2 m
por cinco cuadrados GT del dron 1 a `z=1`, `z=2`, `z=3`, `z=4` y `z=5`,
conservando el cuadrado y la secuencia de yaw absoluto/relativo. Se crea el
perfil `trajectory_gt_square_5_levels.yaml` y un escenario secuencial de 45
goals; la prueba estática de YAML y contratos pasa (`22 passed`). Siguiente
acción exacta: compilar `simulacion_dron` y ejecutar la prueba con Gazebo y las
tres GUIs activadas, timeout 1200 s y espera posterior 60 s.

Build de `simulacion_dron` para `trajectory_gt_square_5_levels`: PASS, salida
0, 1.11 s, log externo en `/home/chenfu/Gazebo/log/simulacion`. Siguiente
accion exacta: ejecutar `trajectory_gt_square_5_levels_profile` con un solo
dron, Gazebo y las tres GUIs activadas, timeout 1200 s y espera posterior 60 s.
Criterio: 45 goals GT completados, cinco niveles con salto vertical de 1 m y
cierre limpio.

La ejecucion `trajectory_gt_square_5_levels_profile` fue interrumpida por el
usuario al observar que el dron se habia quedado bloqueado. El wrapper cerro la
sesion y las GUIs con salida `0`, pero aun no se clasifica la prueba: falta
reducir y analizar el log conservado
`codex/archivos_auxiliares/logs/prueba_trajectory_gt_square_5_levels_profile.log`.
Siguiente accion exacta: ejecutar la reduccion focalizada para localizar el
ultimo goal y la causa del bloqueo.

Analisis de la ejecucion interrumpida `trajectory_gt_square_5_levels_profile`:
resultado `PARCIAL/NO CONCLUIDA`. Se enviaron y completaron correctamente 13
goals GT, hasta `(10,0,2)`; cada uno tiene `GOAL-RESULT action_code=4
success=true` y el nodo `gen_tray` registra el inicio valido de cada accion.
El siguiente goal `(10,10,2)` no llego a enviarse. Tras terminar el goal
anterior, `scenario_runner_node` entro en
`MOVE-GATE-WAIT active=true` porque `/global_mapping/backpressure_active`
permanecia activo. Antes ya se observaron esperas de 213.801 s y 60.052 s; la
ultima espera quedo activa unos 595 s hasta la interrupcion. Por tanto, el
dron no se bloqueo ejecutando ese waypoint: quedo detenido esperando la puerta
de backpressure del servidor global, que se activa por la carga de colas,
optimizacion o pendientes secundarios. La linea `STEP-FAILED` es consecuencia
de la interrupcion controlada por el usuario, no de un resultado de accion
fallido. No se modifica aun `gate_mapping_backpressure`, porque desactivarlo
seria una nueva decision sobre el criterio de solape/optimizacion.
Resultado `f45_trapezoidal`: `run_simulation.sh` terminó con código 1 porque
`scenario_runner_node` terminó con código 1; la prueba no se considera válida
hasta analizar el goal fallido. El log completo se conserva en
`codex/archivos_auxiliares/logs/prueba_f45_trapezoidal.log`. Siguiente acción
exacta: reducir el log y localizar el primer error del escenario, conservando
los CSV obtenidos como evidencia de una ejecución fallida.

Diagnóstico `f45_trapezoidal`: el primer goal finalizó correctamente en
`20,595 s`; el segundo comenzó desde `y=-0,013848 m` con velocidad Y no nula
frente a un desplazamiento objetivo de solo `0,013848 m`. La implementación
actual de VelTrap proyecta una velocidad máxima muy pequeña en ese eje y no
resuelve correctamente esa combinación de error residual y velocidad
contraria, por lo que el feedback quedó en `t_act=300 s` y el runner canceló
por timeout. Se conserva la captura fallida en los CSV y el log reducido.
Corrección acordada para la repetición: dividir los nueve goals en pasos
secuenciales y añadir 2 s de asentamiento después de cada waypoint; no se
modifica la geometría ni el código del generador. Siguiente acción exacta:
actualizar solo el escenario trapezoidal y repetirlo en otro directorio de
datos.

Resultado `f45_trapezoidal_v2`: `run_simulation.sh` terminó con código 1; el
asentamiento entre goals no fue suficiente para que el escenario completara
los nueve tramos. El log y los CSV de esta segunda ejecución quedan
conservados en sus rutas `..._v2`. Siguiente acción exacta: reducir el log y
localizar el segundo goal que vuelve a quedar activo antes de decidir si se
ajusta solo el escenario o si la prueba debe documentarse como incidencia del
generador trapezoidal.

Análisis `f45_trapezoidal_v2`: el tramo 1 terminó en `20,362 s`, pero tras los
2 s de espera el tramo 2 aún recibió `v0=(0,0339, 0,0309, 0,0004) m/s` y
quedó activo hasta el timeout de 300 s. Se amplía mecánicamente la espera a
10 s en cada waypoint para permitir el asentamiento del controlador antes del
siguiente perfil. Siguiente acción exacta: repetir como `f45_trapezoidal_v3`
en otro directorio de datos.

Resultado `f45_trapezoidal_v3`: también terminó con código 1 durante el
escenario, pese a las pausas de 10 s. Se conserva como tercera ejecución y no
se considera válida aún. Siguiente acción exacta: reducir el log para
identificar el tramo exacto y cerrar la decisión sobre la prueba trapezoidal.
Preparación de `f45_trapezoidal_v4`: el YAML pasa validación con 18 pasos. En
los nueve goals, los ejes que permanecen constantes usan objetivo relativo
cero y los ejes de desplazamiento mantienen objetivo absoluto; la geometría
nominal y los yaw de los waypoints se conservan. Se usarán 2 s de espera,
fuente GT, un dron, mundo `empty`, GUI, registrador y 60 s posteriores. La
captura se guardará en `datos/trapezoidal_v4`; siguiente acción exacta:
ejecutar la simulación.

Revisión de `f45_trapezoidal_v5`: aunque terminó con código 0, la inspección
visual mostró que los objetivos relativos acumulaban desplazamientos y
deformaban la ruta nominal. Sus CSV, figuras y métricas quedan conservados
como evidencia de workaround no aceptado. Se restauran los nueve goals con
todos los ejes absolutos y se mantiene la corrección de VelTrap ya compilada.
Siguiente acción exacta: ejecutar `f45_trapezoidal_v6` y usar solo esa captura
para la comparación final.
Resultado `f45_trapezoidal_v4`: también terminó con código 1; el ajuste de
objetivos relativos no bastó para completar todos los tramos. Se conserva la
cuarta ejecución y su captura separada. Siguiente acción exacta: reducir el
log y localizar el primer tramo fallido antes de cerrar las métricas.

Diagnóstico `f45_trapezoidal_v4`: el tramo 2 volvió a quedar activo durante
300 s. El feedback llegó a `x=8` pero una coordenada secundaria divergio hasta
`27,21 m`, confirmando tiempos/perfiles inválidos por velocidad residual en un
eje casi estacionario. Se corrige de forma localizada
`dron/lib_tray/src/generacion/gen_tray_veltrap.cpp`: las velocidades inicial y
final se proyectan sobre el sentido del desplazamiento y se ponen a cero si
son contrarias o superan la velocidad máxima del eje. Siguiente acción exacta:
compilar `lib_tray` y `dron_individual`, y repetir la trapezoidal.

Build de la corrección VelTrap: PASS. `lib_tray` compiló en 1,56 s y
`dron_individual` compiló e instaló en 2,43 s. Siguiente acción exacta:
ejecutar `f45_trapezoidal_v5` con el mismo YAML final y guardar la captura en
un directorio nuevo.

Resultado `f45_trapezoidal_v5`: `run_simulation.sh` terminó con código 0 y
`success=true`; los nueve tramos trapezoidales finalizaron, se mantuvo la GUI
durante los 60 s posteriores y el launch cerró con SIGINT. Log completo
conservado en `codex/archivos_auxiliares/logs/prueba_f45_trapezoidal_v5.log`.
Siguiente acción exacta: reducir el log, comprobar los CSV y generar los
resultados offline trapezoidales.

Resultado `f45_trapezoidal_v6`: `run_simulation.sh` terminó y dejó la captura
completa en `Pruebas/Capítulo 4/4_5_seguimiento/datos/trapezoidal_v6` con
pose, velocidad, aceleración GT y feedback de trayectoria. Se usaron los nueve
goals con objetivos absolutos y la corrección VelTrap compilada; queda
pendiente reducir el log y validar que la geometría de la ruta sea la nominal
antes de generar las métricas finales.

Revisión de `f45_trapezoidal_v6`: la captura quedó incompleta; el log solo
contiene el inicio del quinto tramo y no hay nueve resultados de goal. No se
usará para métricas ni figuras finales y se conserva como intento abortado.
Siguiente acción exacta: repetir la misma prueba en `trapezoidal_final` con
timeout de 900 s y registrar la ejecución completa.

Resultado `f45_trapezoidal_final`, intento de arranque 1: `run_simulation.sh`
terminó con código 1 después de agotar tres arranques de Gazebo, sin iniciar
el escenario ni producir una captura válida. Se conserva el log completo;
queda pendiente reducirlo y diagnosticar la incidencia de infraestructura
antes de repetir.

Resultado `f45_trapezoidal_final_escalated`: `run_simulation.sh` terminó con
`success=true` y código 0. Gazebo arrancó con GUI, mundo vacío y un dron; el
escenario trapezoidal completó los nueve segmentos y los 60 s posteriores, y
el registrador dejó la captura en `datos/trapezoidal_final`. El intento previo
queda clasificado como incidencia de permisos de `~/.ros/log`, no como fallo
de la simulación. Siguiente acción exacta: reducir el log, validar los nueve
goals y generar las figuras y métricas finales.

Cierre 4.5: el log reducido confirma nueve goals trapezoidales con
`success=true`, escenario finalizado con código 0 y `run_simulation.sh` con
`success=true`. Se generaron las figuras XY, matrices 3x4 de GT frente a
trayectoria y de errores, CSV sincronizado, JSON y tabla comparativa. La tabla
final queda documentada en `Pruebas/Capítulo 4/4_5_seguimiento/notas.md`.
Builds vigentes: `simulacion_dron`, `lib_tray` y `dron_individual` PASS.
Trabajo activo: no; no quedan acciones técnicas abiertas para esta petición.

Nueva campaña 4.5 autorizada: sustituir las ejecuciones anteriores por cuatro
pruebas con los mismos nueve waypoints y un dron: `cubica_lenta` (duraciones
24/16 s), `cubica_normal` (12/8 s), `trapezoidal_lenta` (v_max nominal y
esperas de 4 s) y `trapezoidal_rapida` (esperas de 2 s y v_max lineal/angular
doblada). Se eliminaron los datos, figuras, resultados y logs específicos de
la campaña anterior. Se añadieron perfiles YAML, argumentos de launch para
`trajectory_v_max_lin`, `trajectory_v_max_ang` y `trajectory_t_a`, y el
procesador ahora genera artefactos por variante y tabla de cuatro columnas.
También se corrigió `GenTrayVelTrap` para usar las velocidades proyectadas en
el cálculo de desplazamiento de aceleración. Siguiente acción exacta: validar
YAML/sintaxis y compilar `lib_tray`, `dron_individual` y `simulacion_dron`.

Build `lib_tray` de la nueva campaña: PASS, código 0. Siguiente acción exacta:
compilar `dron_individual` y `simulacion_dron`.

Build `dron_individual` de la nueva campaña: PASS, código 0. Siguiente acción
exacta: compilar `simulacion_dron` para instalar los launch y el procesador
actualizados.

Build `simulacion_dron` de la nueva campaña: PASS, código 0. Los tres paquetes
seleccionados están instalados. Siguiente acción exacta: ejecutar
`cubica_lenta` con GUI, mundo `empty`, GT, un dron y 60 s posteriores.

Resultado `f45_cubica_lenta`: `run_simulation.sh` terminó con `success=true` y
código 0; el escenario devolvió código 0 y completó la espera posterior de
60 s. Captura conservada en `datos/cubica_lenta`. Siguiente acción exacta:
reducir el log y validar sus nueve goals antes de ejecutar `cubica_normal`.

Resultado `f45_cubica_normal`: `run_simulation.sh` terminó con `success=true` y
código 0; el escenario devolvió código 0 y completó los 60 s posteriores.
Captura conservada en `datos/cubica_normal`. Siguiente acción exacta: reducir
el log y validar sus nueve goals antes de ejecutar `trapezoidal_lenta`.

Resultado `f45_trapezoidal_lenta`: `run_simulation.sh` terminó con
`success=true` y código 0; el escenario devolvió código 0 y completó los 60 s
posteriores. Captura conservada en `datos/trapezoidal_lenta`. Siguiente acción
exacta: reducir el log y validar los nueve goals antes de ejecutar
`trapezoidal_rapida` con v_max lineal/angular duplicada.

Resultado `f45_trapezoidal_rapida`: `run_simulation.sh` terminó con
`success=true` y código 0; el escenario devolvió código 0 y completó los 60 s
posteriores con `trajectory_v_max_lin=1.6` y `trajectory_v_max_ang=1.0`.
Captura conservada en `datos/trapezoidal_rapida`. Siguiente acción exacta:
reducir el log, validar los nueve goals y procesar las cuatro variantes.

Diagnóstico de `f45_trapezoidal_rapida`: aunque launch recibió los argumentos
numéricos, los CSV muestran máximos `x_vel/y_vel` de 0,8 m/s y `yaw_vel` de
0,396 rad/s, iguales a la prueba nominal. Esta ejecución no es válida para
evaluar el doble de velocidad. Se sustituye el override numérico por una
selección explícita de archivo de parámetros `trajectory_fast.yaml` desde el
launch; se conservará la captura solo como intento técnico y se repetirá la
prueba rápida.

Corrección de selección de parámetros: `generar_dron.launch.py` y
`multi_dron.launch.py` aceptan ahora `trajectory_config`, que selecciona el
YAML instalado en `dron_individual/config`; se añadió `trajectory_fast.yaml`
con `v_max_lin=1.6`, `v_max_ang=1.0` y `t_a=5.0`. Siguiente acción exacta:
compilar de nuevo `dron_individual` y `simulacion_dron`, y repetir solo la
trapezoidal rápida en `datos/trapezoidal_rapida`.

Build de la corrección `trajectory_config`: `dron_individual` PASS, código 0.
Siguiente acción exacta: compilar `simulacion_dron` y repetir la captura rápida
con `trajectory_config:=trajectory_fast.yaml`.

Build de `simulacion_dron` tras añadir `trajectory_config`: PASS, código 0.
Siguiente acción exacta: repetir `f45_trapezoidal_rapida` con el YAML rápido y
registrar el resultado antes de volver a procesar las cuatro variantes.

Resultado `f45_trapezoidal_rapida_v2`: `run_simulation.sh` terminó con
`success=true` y código 0; el escenario devolvió código 0 y completó los 60 s
posteriores usando `trajectory_config:=trajectory_fast.yaml`. La captura
sobrescribió `datos/trapezoidal_rapida`; queda pendiente validar sus máximos de
velocidad y reducir el log antes del análisis final.

Validación final de la campaña: `trapezoidal_lenta` alcanzó aproximadamente
0,8 m/s en X/Y y `trapezoidal_rapida` aproximadamente 1,6 m/s; ambas tuvieron
nueve goals correctos. Se procesaron las cuatro variantes con CSV sincronizado,
figuras XY, matrices 3x4, matrices de error, JSON y tabla de cuatro columnas.
Duraciones registradas: `cubica_lenta` 153,260 s, `cubica_normal` 76,950 s,
`trapezoidal_lenta` 174,590 s y `trapezoidal_rapida` 110,480 s. Métricas RMSE
de posición: 0,023118 m, 0,148723 m, 0,025235 m y 0,077503 m respectivamente.
La documentación final queda en `Pruebas/Capítulo 4/4_5_seguimiento/notas.md`.
La campaña queda cerrada; trabajo activo: no.
