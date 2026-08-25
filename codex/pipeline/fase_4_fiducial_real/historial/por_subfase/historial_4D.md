# Historial 4D

## 2026-08-25 - Implementacion y pruebas de componente

- objetivo intentado: configurar y detectar fiduciales en un worker del wrapper;
- archivos modificados: interfaces Dron/Servidor, `fiducial_config_server`, `FiducialDetector`, `StereoSlamNode`, launch/config y arquitectura;
- paquetes compilados: interfaces Dron/Servidor, `orbslam3`, `orbslam3_server`, `dron_individual` y `simulacion_dron`;
- resultado de build: correcto;
- pruebas: detector 3/3 casos, servidor 11/11 y simulacion 10/10;
- evidencia positiva: pose finita, rechazo de ID desconocido, escala por `size_m`, parser y replicas correctos;
- conclusion: PARCIAL hasta validacion runtime completa.

## 2026-08-25 - Prueba 203

- objetivo intentado: trayectoria tipica completa con debug fiducial;
- pruebas Gazebo/replay: tres intentos de launch, todos antes del escenario;
- evidencia negativa: el script instalado por symlink no tenia permiso ejecutable;
- conclusion: NO CONSEGUIDA; fallo de empaquetado corregido con permiso ejecutable.

## 2026-08-25 - Prueba 204

- objetivo intentado: repetir 203 tras corregir el ejecutable;
- evidencia positiva: servicio `READY` con 15 tags, dos clientes `READY` y primer KF exacto;
- evidencia negativa: `scenario_runner` recibio una ruta YAML relativa invalida y no movio drones;
- conclusion: NO CONSEGUIDA como prueba integral; usar ruta absoluta.

## 2026-08-25 - Prueba 205

- objetivo intentado: trayectoria completa con ruta absoluta, Gazebo y RViz2;
- patrones del reducido: `FID-CONFIG|KF-EVENT|FID-TAG|FID-KF|FID-VISUAL|SCENARIO-RUNNER|SIM-|process has died|symbol lookup`;
- evidencia positiva: ambos wrappers detectaron tag 202; z `1.498/1.444 m`, error `0.252/0.206 px`, quality `0.916/0.931`;
- evidencia negativa: HighGUI cargo `/snap/core20/.../libpthread.so.0`, fallo por `__libc_pthread_init` y termino ambos wrappers con exit 127; sin nuevos deltas ni anchors, RViz2 quedo vacio;
- resultado: el usuario detuvo la simulacion a los 214 s durante el paso 8; no es una finalizacion del escenario;
- correccion posterior: entorno Snap saneado para nodos ORB y guarda `DISPLAY`/`WAYLAND_DISPLAY`; rebuild correcto y CTest 1/1;
- conclusion: PARCIAL; deteccion demostrada, correccion visual pendiente de runtime por decision expresa de no repetir;
- siguiente paso recomendado: verificar ventana, cierre a 5 s y continuidad de deltas en una futura simulacion autorizada.

## 2026-08-25 - Prueba 206

- objetivo intentado: revalidar la ventana HighGUI tras sanear el entorno Snap;
- prueba Gazebo: trayectoria tipica con Gazebo/RViz2, ambos grafos apagados y debug visual a 5 s;
- evidencia positiva: las ventanas funcionaron inicialmente segun el usuario; ambos wrappers detectaron repetidamente tags 202/204 y siguieron publicando deltas; tambien se detecto tag 301;
- evidencia negativa: a los 176-178 s ambos wrappers terminaron con exit `-9`; desde ese momento RViz2 dejo de recibir puntos nuevos;
- interpretacion revisada con el usuario: el escritorio mostro `cerrar ventana/esperar` y el usuario eligio `cerrar ventana`; esa accion fuerza `SIGKILL` sobre el proceso propietario de HighGUI y explica exactamente los dos exit `-9`;
- evidencia descartada: no aparece el fallo `libpthread`, no salto la guarda, habia mas de 5.8 GiB disponibles y no hay evidencia OOM;
- resultado: el usuario detuvo la simulacion a los 331 s durante el paso 14; no es un escenario completo;
- conclusion: PARCIAL; el saneamiento Snap funciona, pero un debug HighGUI dentro de `stereo` no aisla un cierre forzado del SLAM;
- siguiente paso recomendado: mover presentacion/temporizador HighGUI a un proceso ROS independiente y dejar en el wrapper solo la produccion opt-in de la imagen anotada.

## 2026-08-25 - Prueba 207

- objetivo intentado: validar el visualizador ROS separado con la trayectoria tipica completa;
- cambios previos: `stereo` publica `orbslam/fiducial_debug/image` latest-only y el ejecutable independiente `fiducial_visualizer` posee todo HighGUI;
- resultado de build/tests: `orbslam3` y `dron_individual` 1/1; CTest wrapper 1/1 y Simulacion 10/10; guarda 15/15;
- prueba Gazebo: escenario completo, exit 0, `SIM-DONE success=true`, recursos estables y `guard_triggered=false`;
- evidencia positiva: RViz2 y wrappers funcionaron con normalidad segun el usuario; ambos visualizadores READY, tags 202/204 detectados, imagenes publicadas y recibidas;
- evidencia negativa: cada `SHOW` fue seguido 3-4 ms despues por `user_close`; el usuario no vio ventanas;
- diagnostico: `WND_PROP_VISIBLE=0` transitorio justo tras `namedWindow` se interpreto erroneamente como cierre humano;
- conclusion: PARCIAL; aislamiento conseguido, presentacion visual no visible;
- siguiente paso recomendado: exigir una visibilidad confirmada antes de aceptar `user_close` y repetir.

## 2026-08-25 - Prueba 208

- objetivo intentado: repetir integra la 207 tras corregir la carrera de visibilidad;
- resultado de build/tests: rebuild `orbslam3` 1/1, CTest wrapper 1/1, rebuild Simulacion 1/1 y CTest 10/10;
- prueba Gazebo: escenario completo, exit 0, `SIM-DONE success=true`, recursos estables, minimo disponible `5388.2 MiB` y `guard_triggered=false`;
- patrones del reducido: `FID-VISUAL-PUB|FID-VISUALIZER|FID-TAG|PIPE0-WRAPPER-DELTA-PUB|process has died|SIM-*`;
- evidencia positiva: 2 visualizadores READY, 80 publicaciones, 79 eventos SHOW, 17 cierres reales por timeout, 0 `user_close` falsos y 0 desactivaciones; tags 202/204 detectados;
- continuidad: despues del ultimo timeout ambos wrappers siguieron publicando deltas durante unos 57 s; no murio ningun `stereo` y solo aparece el exit 255 conocido de Gazebo en cleanup;
- revision conversada: el usuario acepta el resultado y da por concluidas 4C y 4D;
- conclusion: CONSEGUIDA; cadena tecnica, aislamiento y revision humana aceptados;
- siguiente paso recomendado: preparar el bloque 4E+4F sin repetir 203-208.
