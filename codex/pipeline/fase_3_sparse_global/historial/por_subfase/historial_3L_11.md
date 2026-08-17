# Historial 3L_11 - Correccion integral y validacion 148-151

## 2026-08-14 - Subfase 3L - Regresion y live final tecnica

- objetivo intentado: validar conjuntamente continuidad futura, control por
  visita, lifecycle web, color por submapa y cierre sin hard failures;
- build/tests: build final de tres paquetes exit 0; 49/49 C++ y 9/9 web. CTest
  no ejecuto por escritura denegada en `build/`; los binarios se ejecutaron
  directamente y todos pasaron;
- replay 149: `CONSEGUIDA`, 496 entradas, 7 tareas, 3 commits, 4 stale, cero
  hard y recursos sanos;
- live 148: `NO CONSEGUIDA`, preservada. La promocion concurrente de KF150
  hizo fallar el commit de KF149 por hard constraint y mantuvo backpressure
  hasta timeout; no fue un problema de memoria;
- replay 150: `CONSEGUIDA`, reproduce 1239 entradas de 148; KF149 pasa a
  control en un commit full, KF150 no se promociona y no hay hard failures;
- live 151: `CONSEGUIDA tecnicamente`, scenario completo exit0/success=true en
  411 s; 6 submapas, 11 tareas, 3 commits, 8 stale, cero hard, pending0,
  `max_active=1` y tres ciclos de backpressure liberados;
- recursos 151: server RSS 179.4 MiB, grupo 1556.4 MiB, MemAvailable minimo
  4650.5 MiB, PSI memoria cero y guard inactivo;
- errores: dos marcadores posteriores a `SIM-DONE` pertenecen al cleanup de
  URDF/Gazebo y no invalidan el escenario;
- conclusion agregada: `PARCIAL`; criterios automaticos 3H-3L conseguidos,
  pendiente la valoracion del usuario en RViz2 y grafo web.

## 2026-08-14 - Revision visual de live 151

- observacion del usuario: `Esta todo perfecto`;
- interpretacion: confirma color perceptible por submapa, continuidad de KFs
  posteriores y lifecycle secundario sin parpadeo;
- conclusion revisada: `CONSEGUIDA`; 3F y 3H-3L quedan cerradas sin borrar la
  live 148 fallida ni sus causas.
