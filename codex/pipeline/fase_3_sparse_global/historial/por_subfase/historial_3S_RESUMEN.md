# Historial 3S - resumen

## Estado

```text
CONSEGUIDA: RECALIBRACION TECNICA Y VISUAL CONFIRMADAS
```

## Implementado

- raw = base ORB * distancia * aislamiento + `0.04` por inlier;
- factores configurables, acotados y recuperables, con indice voxel;
- fused = media de todos los miembros + `0.04*N`;
- propagacion incremental tras raw/anchors/poses/merges;
- visibilidad sparse solo diagnostica; oclusion para Fase 8;
- builder sin filtro, RViz2 score/rgb rojo-amarillo-verde;
- telemetria `F3S-*` y stats live.

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

El usuario confirma que los scores de 194 han salido perfectos y concluye 3S.
La mala optimizacion final del dron antihorario junto al fiducial 2 se investiga
como incidencia separada 3Q/fiducial y no invalida el scoring.
