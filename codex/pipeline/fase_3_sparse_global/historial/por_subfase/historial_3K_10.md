# Historial 3K_10 - Continuidad indefinida del ultimo control

## 2026-08-14 - Subfase 3K - KFs posteriores al commit

- objetivo intentado: propagar a todo KF futuro la transformacion del ultimo
  control aceptado sin alterar `RawMapDatabase` ni el anchor inicial;
- archivos modificados: `global_pose_types.hpp`, `global_pose_store.hpp/.cpp`,
  `sparse_global_backend.cpp`, telemetria del servidor y tests;
- implementacion: `ContinuationRecord` versionado por submapa; un commit full
  actualiza poses y continuidad atomicamente. `raw_world_pose` permanece bajo
  el anchor inicial y `world_pose/correction_pose` de IDs posteriores se
  derivan desde el control. Un parcial conserva la continuidad anterior;
- build: primer intento fallo por un lock duplicado introducido durante la
  edicion; correccion mecanica y build final de tres paquetes exit 0;
- tests: primer KF insertado despues del commit, tail tardio, parcial,
  aislamiento de submapas y visita coherente posterior incluidos en 49/49 C++;
- replay 149: 7 tareas, 3 commits, 4 stale y cero hard. Tras los commits,
  `(1,0)` KFs157-163 quedan en 0.183-0.248 m y `(2,2)` KFs268-277 en
  0.019-0.190 m, sin repetir 0.9/4 m;
- live 151: commit principal control187 mueve116 KFs; KFs194-209 posteriores
  quedan en 0.081-0.239 m. Cola final vacia y cero fallos;
- conclusion: `PARCIAL`; continuidad demostrada por tests, dos replays y live,
  pendiente confirmacion visual del usuario de que no reaparecen poses previas.
