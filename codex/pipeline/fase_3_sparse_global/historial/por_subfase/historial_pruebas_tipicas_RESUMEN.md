# Historial pruebas tipicas - resumen

Leer este archivo antes de `historial_pruebas_tipicas.md` cuando haya que elegir
o razonar sobre trayectorias de regresion.

## Estado vigente

Tema transversal de pruebas reutilizables. No es una subfase funcional.

## Pruebas utiles

- `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`: dos drones rodean el
  edificio en sentidos opuestos, pasan por fiducial 2, fiducial 1 y vuelven a
  fiducial 2. Util para loops, fiduciales, prioridad del worker y regresion
  larga.
- `prueba_tipica_fiducial_2_a_1_dos_lados.yaml`: caso corto fiducial 2 a 1 por
  dos lados. Util para diagnostico de grafo/dry-run sin una simulacion larga.
- `prueba_rodeo_antihorario_un_dron_fid2_fid1_fid2.yaml`: un dron activo y otro
  parado. Util para aislar optimizaciones fiduciales consecutivas.

## Evidencia

- La prueba larga produjo tareas fiduciales y loops inter-dron.
- La prueba corta produjo un caso reproducible de error alto en `drone_2` y
  dumps/HTML para replay offline.
- `prueba_41` valido visualmente dos applies consecutivos con un dron.
- `prueba_43-44` validaron cola tardia y KFs tardios dentro de ventana en el
  runtime historico.
- `prueba_73`: escenario correcto, pero `2059/2741` admisiones se perdieron por
  un limite de cola demasiado pequeno; `PARCIAL`.
- `prueba_74`: timeout y movimiento de poses incorrecto por formular un loop
  relativo como prior absoluto; `NO CONSEGUIDA`.
- `prueba_75`: scenario correcto y prioridad fiducial observada, pero detecta
  un loop cercano no causal que propaga `248` KFs. Solo se ejecutan `66` de
  `561` loops encolados y quedan `496` pendientes en el ultimo inicio;
  `PARCIAL`.
- `prueba_76`: filtro causal activo y cero commits de poses por loop, por lo
  que consigue esa regresion concreta. No valida independencia temporal:
  `84/489` loops llegan a ejecutarse, el pico de cola es `429`, quedan al menos
  `414` en el ultimo inicio y la publicacion alcanza `27.951 s` de retraso;
  conclusion transversal `PARCIAL`.
- En `prueba_75` hubo `13` observaciones de `fid=1`; en `prueba_76`, cero. Esta
  diferencia de entrada explica por que la segunda no creo tareas fiduciales.

## Aprendizajes

- Usar estas pruebas como regresiones antes de inventar YAMLs nuevos.
- Distinguir exito mecanico (`SIM-DONE`) de validacion visual o de criterio de
  subfase.
- El `gazebo exit code 255` posterior a `SIM-DONE success=true` suele ser
  cleanup no bloqueante.
- La ausencia de un accept en una regresion negativa no cierra la calidad de
  accepts positivos; `3Q` permanece parcial.
- Los conteos `84/84` describen starts con end, no el drenaje de toda la cola.
- Un commit de pose aceptado puede verse alterado por commits aceptados
  posteriores; no confundirlo con overwrite raw sin buscar rollback y
  cronologia de publicaciones.

## Detalle

`historial_pruebas_tipicas.md`.
