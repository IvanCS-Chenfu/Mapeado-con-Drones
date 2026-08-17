# Pruebas tipicas de trayectoria

## Proposito

Este documento define trayectorias reutilizables que no pertenecen a una unica subfase. Sirven como pruebas de regresion para varias partes de la Fase 3: anclaje fiducial, revisits, snapshots, publicacion sparse, futura optimizacion y futura fusion.

Los archivos `codex/archivos_auxiliares/trayectorias/tray_prueba_X.yaml` son aliases temporales de ejecucion para `run_simulation.sh`. Cuando una trayectoria sea estable y util para varias subfases, debe conservarse tambien con un nombre semantico de prueba tipica.

Ejemplo de nombres estables:

```text
codex/archivos_auxiliares/trayectorias/prueba_tipica_anclaje_diferencial.yaml
codex/archivos_auxiliares/trayectorias/prueba_tipica_fiducial_2_a_1_dos_lados.yaml
codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
```

Regla practica:

- para ejecutar una subfase, copiar o adaptar una prueba tipica a `tray_prueba_1.yaml`, `tray_prueba_2.yaml`, etc.;
- no perder el nombre semantico de la prueba tipica si la trayectoria queda validada;
- documentar en historial que `tray_prueba_X.yaml` proviene de una prueba tipica.

## Prueba tipica 1 — anclaje diferencial y revisit simple

Nombre sugerido:

```text
prueba_tipica_anclaje_diferencial.yaml
```

Estado:

```text
Materializada en codex/archivos_auxiliares/trayectorias/prueba_tipica_anclaje_diferencial.yaml durante 3H.
```

Idea:

Validar el caso que se ha usado repetidamente en Fase 3 para comprobar anclaje, publicacion sparse, revisits sencillos y diferencias entre submapas.

Secuencia conceptual:

1. `drone_1` va al fiducial 2.
2. `drone_2` se coloca encima de `drone_1`, a distinta altura.
3. `drone_1` se mueve a la izquierda.
4. `drone_2` vuelve a colocarse encima de `drone_1`.
5. `drone_1` vuelve al fiducial 2.
6. `drone_2` vuelve a colocarse encima de `drone_1`.

Uso esperado:

- primera observacion fiducial para anclar;
- segunda observacion del mismo fiducial para medir revisit;
- comparar nubes de ambos drones en RViz2;
- generar `.record` pequeno o medio para replay rapido.

Limitaciones:

- no rodea todo el edificio;
- no fuerza necesariamente loops geometricos ricos;
- puede no cubrir fiducial 1.

## Prueba tipica 2 — rodeo del edificio con dos fiduciales

Nombre sugerido:

```text
prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
```

Idea:

Ambos drones rodean el edificio en sentidos contrarios, pasan por los fiduciales 2 y 1, y miran siempre hacia el edificio. Esta prueba debe ser util para validar revisits, drift acumulado, snapshots, futura optimizacion fiducial y futuros loops/fusion.

Tiempos de movimiento:

```text
tx = 40 s
ty = 40 s
tz = 40 s
tyaw = 13 s
```

Secuencia conceptual:

1. Ambos drones van a la vez al fiducial 2, con distinta altura como en las pruebas anteriores.
2. `drone_1` va a `(-10, -10)` con yaw `90 deg`.
3. `drone_2` va a `(10, -10)` con yaw `90 deg`.
4. `drone_1` va a `(-10, 10)` con yaw `0 deg`.
5. `drone_2` va a `(10, 10)` con yaw `180 deg`.
6. `drone_1` va al fiducial 1 con yaw `-90 deg`.
7. `drone_2` se coloca encima de `drone_1`.
8. `drone_1` va a `(10, 10)` con yaw `-90 deg`.
9. `drone_2` va a `(-10, 10)` con yaw `90 deg`.
10. `drone_1` va a `(10, -10)` con yaw `-180 deg`.
11. `drone_2` va a `(-10, -10)` con yaw `0 deg`.
12. Ambos drones vuelven al fiducial 2 con yaw `90 deg`.

Validación reciente: `prueba_43` usa esta trayectoria para comprobar workers
paralelos y KFs llegados durante el solver. El dron 2 incorporó cuatro
controles fiduciales post-target, refinó siete KFs y dejó uno derivado, sin
poses inválidas ni pérdida de MapPoints.

`prueba_44` reutiliza la misma trayectoria para la carrera complementaria. El
grafo del dron 2 capturó 86 KFs; durante el solver llegaron nueve IDs internos
adicionales. El commit refinó los nueve y el HTML poscommit contiene los 95 KFs
reales del intervalo, con nube estable e `invalid_pose_skipped=0`.

Intencion geometrica:

- `drone_1` y `drone_2` recorren lados opuestos del edificio;
- los yaw se eligen para mirar hacia el edificio durante el rodeo;
- la visita a fiducial 1 debe crear una segunda referencia absoluta;
- la vuelta al fiducial 2 debe producir revisits con drift acumulado.

Precaucion importante:

La posicion exacta del fiducial 1 debe tomarse de la configuracion real de fiduciales del launch o del mapa. No inventar coordenadas si el fiducial 1 no esta configurado todavia. Si la subfase necesita esta prueba y el fiducial 1 aun no existe en configuracion, documentar la limitacion y usar solo la parte validable con fiducial 2.

Estado tras 3H:

```text
Materializada tras 3H como prueba de regresion larga en:
codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
codex/archivos_auxiliares/trayectorias/tray_prueba_3.yaml
```

Nota de ejecucion:

- el launch activo del servidor global debe tener configurados los fiduciales legacy `ids=[1,2]`, `x=[0,0]`, `y=[9,-9]`, `z=[1,1]`, `yaw=[0,0]`, `radius=[2,2]`;
- por defecto conviene ejecutar esta prueba con `rawdb_record_enabled:=false` si queda poco disco, porque la trayectoria larga puede generar un `.record` grande;
- si se quiere conservar dataset para replay futuro, liberar espacio antes y usar un nombre semantico de record.

Validacion del 2026-07-09:

```bash
./codex/herramientas/run_simulation.sh --prueba 3 --launch "ros2 launch simulacion_dron multi_dron.launch.py rawdb_record_enabled:=false" --post-scenario-wait-sec 30 --startup-wait-sec 20 --timeout-sec 1200 --max-gazebo-retries 1
```

Resultado:

- `SCENARIO-RUNNER-DONE scenario='prueba_tipica_rodeo_edificio_dos_fiduciales' success=true`;
- `SIM-DONE prueba=3 success=true`;
- `SIM-EXIT-CODE 0`;
- se genero `codex/archivos_auxiliares/logs/prueba_3.log` y `codex/archivos_auxiliares/logs/prueba_3.reduced.log`;
- no se genero un `.record` nuevo porque se ejecuto con `rawdb_record_enabled:=false`.

Evidencia tecnica:

- el inicio en fiducial 2 produjo revisits `[F1H-FID-OK]` con error bajo;
- el paso por fiducial 1 produjo observaciones `[F1E-FID-KF-ASSOC] fid=1` y tareas `[F1H-FID-TASK-CREATED]`, por ejemplo `task_id=1 fid=1 drone_id=1 kf=211 error_t=0.445654` y `task_id=2 fid=1 drone_id=2 kf=158 error_t=22.743950`;
- la vuelta a fiducial 2 produjo revisits correctos en nuevos epochs, por ejemplo `drone_id=2 epoch=2 kf=245 error_t=0.001731` y `drone_id=1 epoch=3 kf=314 error_t=0.025037`;
- el estado final de tareas fiduciales llego a `total=41 pending=41 confirmed_ok=65 high_error=41 duplicates=0 no_pose=0 revisits=106`;
- no aparecieron `FATAL`, `Segmentation fault`, `Killed` ni `std::bad_alloc`;
- aparece `gazebo ... exit code 255` despues de `SIM-DONE`, durante cleanup, y no se considera fallo funcional.

Validacion concurrente del 2026-07-28 (`prueba_42`):

- el escenario largo y la simulación terminan con código 0;
- las tareas 1 y 2, de submapas distintos, optimizan simultáneamente ventanas
  de 63 y 91 KFs (`peak_active_workers=2`);
- se completan tres applies, todos con `raw_db_modified=false`,
  `hard_fixed_moved=false` e `invalid_pose_skipped=0`;
- se generan HTML 3D y dumps independientes para `task_id=1,2,3`;
- el dron 1 pierde tracking durante la vuelta y llega al fiducial 2 con
  `epoch=3`, por lo que esa llegada ancla un submapa nuevo;
- no aparecen ventanas solapadas, de modo que la espera/recheck del coordinador
  todavía requiere una prueba controlada.

Uso esperado:

- Fase 3H: provocar revisits fiduciales y medir residual absoluto.
- Fases 3I-3L: alimentar tareas de optimizacion fiducial.
- Fases 3N-3Q: generar candidatos de loop/subnube en lados opuestos.
- Fases 3V-3W: prueba integral de regresion con trayectoria mas larga.

## Prueba tipica 3 — fiducial 2 a fiducial 1 por dos lados

Nombre estable:

```text
prueba_tipica_fiducial_2_a_1_dos_lados.yaml
```

Idea:

Ambos drones van primero al fiducial 2 para anclar, recorren lados opuestos del
edificio hasta el fiducial 1 y se detienen alli. Es la variante corta de la
prueba de rodeo cuando interesa inspeccionar solo el error acumulado al llegar
al fiducial 1, el grafo fiducial y la propuesta de optimizacion, sin seguir
alrededor del edificio.

Secuencia conceptual:

1. Espera de arranque de tracking.
2. Ambos drones van simultaneamente al fiducial 2.
3. Pausa corta para permitir anclaje/revisit.
4. `drone_1` va por el lado oeste y `drone_2` por el lado este.
5. Ambos llegan simultaneamente al fiducial 1.
6. Pausa final para dry-run, apply/rollback y export HTML/TSV.

Uso esperado:

- `3I`: reproducir una ventana fiducial live de error alto y guardar
  `f3i_window_task_<task_id>.tsv`.
- `3J`: comparar dry-run, costes y HTML 3D sin repetir Gazebo.
- `3K-3L`: observar apply/rollback en RViz2 y logs post-apply.

Validacion del 2026-07-23:

```bash
./codex/herramientas/run_simulation.sh --prueba 27 --yaml /home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_fiducial_2_a_1_dos_lados.yaml --startup-wait-sec 2 --post-scenario-wait-sec 60 --launch "ros2 launch simulacion_dron multi_dron.launch.py pose_graph_vertex_selection_ratio:=0.30 pose_graph_use_covisibility_edges:=false pose_graph_fiducial_neighborhood_vertex_ratio:=0.20 loop_bow_min_mappoints:=1000000 f1l_debug_animation_enabled:=true f1l_graph_dump_enabled:=true f1l_gt_kf_debug_enabled:=true f1l_gt_kf_debug_max_dt_sec:=1.0"
```

Resultado:

- `SCENARIO-RUNNER-DONE scenario='prueba_tipica_fiducial_2_a_1_dos_lados' success=true`;
- `SIM-DONE prueba=27 success=true`;
- `SIM-EXIT-CODE 0`;
- `drone_1` llega a fiducial 1 con error bajo: `kf=226`,
  `error_t=0.158574`, `error_rot=0.025352`, `decision=OK`;
- `drone_2` crea la tarea grande: `task_id=2`, `fid=1`, `kf=203`,
  `error_t=28.937918`, `error_rot=2.908612`, `error_yaw=2.905030`;
- grafo de `task_id=2`: `window_keyframes=130`, `vertices=44`, `edges=43`;
- dry-run: target `28.937918 m -> 0`, yaw `2.905030 -> 0`, coste
  `69777420.092643 -> 512473.703446`;
- HTML live: `codex/archivos_auxiliares/html/f3l_debug_animation_task_2.html`;
- ventana/grafo: `codex/archivos_auxiliares/repeticiones/f3i_window_task_2.tsv`
  y `codex/archivos_auxiliares/repeticiones/f3l_graph_task_2.tsv`;
- HTML offline reproducido desde TSV:
  `codex/archivos_auxiliares/html/f3l_offline_graph_task_2_prueba_27_3d.html`;
- esa ejecución rechazó el apply por `global_map_check_failed`; después, `3K`
  corrigió la causa de publicación por cobertura de KFs corregidos y
  `prueba_31` validó la misma prueba típica con dos applies aceptados.

## Prueba tipica 4 — un dron antihorario fiducial 2 -> 1 -> 2

Nombre estable:

```text
prueba_rodeo_antihorario_un_dron_fid2_fid1_fid2.yaml
```

`drone_1` permanece parado y `drone_2` completa el rodeo antihorario. Es la
prueba de aislamiento para comprobar dos ventanas consecutivas del mismo
submapa sin mezclar el resultado de otro dron.

Validacion del 2026-07-27 (`prueba_41`):

- escenario y simulacion: `success=true`, `SIM-EXIT-CODE 0`;
- un unico `map_epoch` y dos applies del servidor;
- ORB-SLAM3: `loopClosing` inactivo, sin loop ni merge, covisibilidad nativa
  preservada;
- task 1: target `16.740643 -> 0 m`, media GT
  `5.824808 -> 0.910463 m`;
- task 2: target `2.392760 -> 0 m`, pero media GT
  `0.218745 -> 1.358109 m`;
- publicacion final sin poses invalidas ni MapPoints corregidos sin KF;
- HTML 3D: `f3l_debug_animation_task_1.html` y
  `f3l_debug_animation_task_2.html`.

Validacion visual del 2026-07-28:

- el usuario confirma que ambos applies y el mapa completo se ven perfectamente
  en RViz2;
- no hay KFs ni MapPoints fuera de sitio;
- la primera ventana optimizada permanece estable al completar la segunda.

Estado: escenario y ruta de apply/publicacion `CONSEGUIDOS`. La degradacion de
la metrica GT debug de task 2 queda como observacion no bloqueante para
`3I/3J`.

## Relacion con `tray_prueba_X.yaml`

Las pruebas tipicas no sustituyen los YAML numerados. Los YAML numerados siguen siendo la entrada mecanica de `run_simulation.sh`.

Convencion recomendada:

```text
tray_prueba_1.yaml -> prueba live principal de la subfase actual
tray_prueba_2.yaml -> replay o variante rapida de la subfase actual
tray_prueba_4.yaml -> replay/espera legacy si una subfase lo conserva
```

Si una subfase necesita una prueba tipica:

1. conservar la prueba tipica con nombre semantico;
2. copiar su contenido a `tray_prueba_X.yaml` para ejecutar;
3. en el historial, indicar que `tray_prueba_X.yaml` deriva de la prueba tipica correspondiente.

## Criterio documental

Cuando una prueba tipica se modifique:

- actualizar este documento;
- actualizar `codex/contexto/paquetes/simulacion_dron/simulacion_dron.md` si cambia el uso del runner;
- registrar en el historial de la subfase que version se uso;
- no borrar una prueba tipica estable solo porque `tray_prueba_X.yaml` se reutilice para otra subfase.
