# Historial 3B - resumen

Leer este archivo antes de `historial_3B.md` cuando haya que hablar o trabajar
sobre `3B`.

## Estado vigente

`CONSEGUIDA` el 2026-08-10. La congelación, el runtime vacío y la base visual
incremental quedaron validados automática y visualmente en la prueba 78. La
ampliacion operativa del 2026-08-12 integra la apertura del navegador y mantiene
build/simulacion dentro de las herramientas autorizadas.

## Que se hizo

- `orbslam3_server/legacy2` conserva fuentes, cabeceras, tests, launches y
  metadatos del estado anterior.
- `orbslam3_multi/legacy2` conserva todas las clases y tests anteriores; la
  carpeta `legacy/` previa queda intacta.
- Los MD anteriores se archivaron en los `legacy2` documentales.
- `orbslam3_multi` activo no tiene targets ni codigo.
- `global_map_server` activo solo crea el nodo, hace `spin()` y termina.
- `multi_dron.launch.py` ya no expone ni reenvia parametros legacy del servidor.
- El visualizador anterior se archivó en `simulacion_dron/legacy2`; la base
  activa es live y contiene solo `Wrappers ORB-SLAM3` y `GlobalMapServer`, sin
  aristas ni telemetría funcional.

## Evidencia

- Build incremental y rebuild limpio final: `orbslam3_multi orbslam3_server
  simulacion_dron`, tres paquetes y exit code `0` en ambos.
- Prueba 77 intento 1: fallo mecánico por YAML relativo; artefactos preservados.
- Prueba 77 intento 2: `SIM-DONE ... success=true`, exit code `0`.
- `[F3B-EMPTY-SERVER-INIT]` aparece una vez y `global_map_server` termina
  limpiamente.
- No hay publicaciones, subscriptions ni actividad sparse global del servidor.
- La auditoría de install confirma que no quedan cabeceras, tests, corrector,
  biblioteca ni launch legacy instalados.
- Tests visuales: Pytest `4 passed`, CTest 1/1 passed y render headless correcto
  en escritorio/móvil; `/health` confirma modo live y secuencia 0.
- Prueba 78: ambos drones alcanzaron fiducial 2 en paralelo, los dos goals
  terminaron `success=true`, se completaron 30 s de observación y la prueba
  terminó con exit `0`.
- RViz2 y bridge arrancaron/cerraron limpiamente; no existen outputs globales
  del servidor. El usuario confirmó después que RViz2 y el grafo no mostraron
  actividad global, tal como exigía 3B.

## Aprendizajes

- Una instantánea legacy compilable en paralelo no es necesaria: debe quedar
  inerte para impedir dependencias accidentales.
- Las rutas YAML relativas a `src/` fallan porque `run_simulation.sh` ejecuta
  el runner desde el workspace padre; usar ruta absoluta o relativa a ese cwd.
- En 3B no se recibe ni publica mapa. La recepción comienza en 3C.
- La infraestructura del grafo pertenece a 3B. Cada subfase posterior añade
  sus nodos/aristas/eventos reales; 3U audita y endurece el conjunto.
- Un script instalado con `install(PROGRAMS)` debe conservar permiso ejecutable;
  esta carencia fue detectada antes de la prueba 78 y corregida.
- `pipeline_flow_browser.py` es el helper unico: espera `/health`, abre una
  pestaña y se invoca desde launch. Replay usa `run_simulation.sh
  --without-gazebo`; live conserva healthcheck y reintentos Gazebo.
- Los launches sanean variables Snap/VS Code para que RViz2 no cargue
  bibliotecas GUI incompatibles.

## Siguiente paso

3C-3E se ejecutaron posteriormente. La infraestructura 3B queda como base
operativa de 3F y subfases siguientes.

## Detalle

`historial_3B.md`.
