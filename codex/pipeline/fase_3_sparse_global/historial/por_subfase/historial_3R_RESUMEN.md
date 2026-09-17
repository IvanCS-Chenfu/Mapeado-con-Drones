# Historial 3R - resumen

## Estado

```text
PARCIAL: aislamiento confirmado; mascara fisica compilada, testeada y ejecutada
en simulacion conjunta. Solo queda la confirmacion visual del usuario de que
los MapPoints del otro dron se vieron con score cero.
```

## Implementado

- raw = base ORB * distancia * aislamiento + `0.04` por inlier;
- factores configurables, acotados y recuperables, con indice voxel;
- fused = media de todos los miembros + `0.04*N`;
- desde 2026-09-15, un MP maduro sin dos vecinos globales validos a `<=0.30 m` tiene
  score raw duro `0`, aunque reciba inliers; la recuperacion por vecino es
  incremental y un track solo de miembros aislados publica fused `0`;
- desde 2026-09-15, los MapPoints dentro de la esfera fisica de cualquier
  KeyFrame global activo de un dron registrado tambien reciben cero duro. La
  esfera se centra en `base_link`, tiene radio igual a la semidiagonal de las
  dimensiones y se actualiza por vecindad al mover/inutilizar KFs o cambiar el
  registro; no depende de ownership del MapPoint;
- propagacion incremental tras raw/anchors/poses/merges;
- visibilidad sparse solo diagnostica; oclusion para Fase 8;
- builder sin filtro, RViz2 score/rgb rojo-amarillo-verde;
- telemetria vigente `F3R-*` y stats live; las pruebas historicas 192-194
  conservan el prefijo `F3S-*` original.

## Evidencia

- build 3/3; tests 9/9 + 4/4 + 1/1;
- 192 pasa pero conserva backlog primario 45; intento preservado;
- 193 pasa con principal/secundario pending 0 y hard_failed 0;
- 60.524 tracked, 24.969 anchored, 529 isolated, 1 near, 24.195 far;
- score `0/0.1502/1`; 77 commits fused y cero negativos sparse;
- 23.531 puntos publicados con score/rgb; recursos estables.

La prueba 194 valida la recalibracion:

- build 3/3; tests 8/8 + 4/4, multi 9/9, servidor 4/4 y web 1/1;
- `success=true`, colas final cero y `hard_failed=0`;
- 24.977 anchored, 99 near y 11.433 far, media `0.2596`;
- frente a 193: far 24.195->11.433, near 1->99, media 0.1502->0.2596;
- 53 commits fused, cero negativos sparse y 23.564 puntos con score/rgb;
- recursos estables; exit 255 de Gazebo solo durante cleanup.

## Revision visual de 193

- las revisitas elevan correctamente el score de una zona;
- la mayoria de estructura valida queda demasiado baja;
- puntos a menos de 1 m conservan score excesivo;
- con baseline aproximado `0.06 m`, el limite lejano actual es `2.4 m` y el
  cercano `0.20 m`, coherente con `far=24.195` y `near=1`.

## No repetir

- no expandir vecinos por cada alta de un batch;
- no reindexar geometria identica;
- no reintroducir penalizaciones sparse de oclusion antes de Fase 8;
- no interpretar el exit 255 de Gazebo durante cleanup como fallo live.

## Pendiente

El usuario confirma que los scores de 194 han salido perfectos y concluye 3R.
La mala optimizacion final del dron antihorario junto al fiducial 2 se investiga
como incidencia separada 3Q/fiducial y no invalida el scoring.

La prueba 767 se ejecuto correctamente tras corregir su modo de concurrencia a
`simultaneous`: ambos drones recibieron sus dos tramos GT en paralelo y el
escenario termino con `success=true`. El primer arranque de Gazebo murio pronto,
pero el reintento limpio del helper completo la prueba. El log confirma los dos
registros y publicaciones sparse con score en `[0,1]`; la GUI es la evidencia
final pendiente para verificar visualmente los puntos del otro dron con cero.
