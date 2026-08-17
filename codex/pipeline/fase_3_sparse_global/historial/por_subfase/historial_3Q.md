# Historial 3Q

## 2026-08-05 - Integracion relativa y doble correccion de seguridad

- objetivo intentado: integrar la decision de optimizar dentro de `LoopTask`
  sin convertir una medida relativa en un objetivo world ni mover anchors;
- archivos modificados: `pose_graph_problem.hpp`, `pose_graph_builder.*`,
  `optimization_manager.*`, `loop_detector.*`, `global_map_server.cpp` y
  `test_loop_optimization_task.cpp`;
- implementacion: nuevo edge type `LOOP_RELATIVE`, candidato fijo, evaluacion
  de residual relativo, soporte geometrico minimo y commit atomico en
  `GlobalPoseStore`;
- build: `orbslam3_multi orbslam3_server`, codigo `0`;
- tests: `test_loop_optimization_task` y pruebas relacionadas, `PASS`;
- `prueba_74`: timeout y accept incorrecto para
  `query=1:0:221`, `candidate=2:3:192`; se escribieron `31` poses optimizadas y
  se propagaron `92`. La medida relativa se habia tratado como prior world
  absoluto. Conclusion: `NO CONSEGUIDA`;
- primera correccion: construir el grafo con `LOOP_RELATIVE`, mantener el lado
  candidato fijo y medir el error relativo en el optimizador;
- `prueba_75`: scenario `success=true`, pero el analisis del log detecto otro
  caso indebido: `query=2:1:62` eligio `candidate=2:1:75`, un KF posterior del
  mismo submapa con gap `13`, y el accept propago `248` KFs. Conclusion:
  `PARCIAL`;
- segunda correccion: `LoopDetector` descarta candidatos posteriores del mismo
  submapa y reporta `skipped_noncausal_same_submap`; el servidor suprime la
  rama de optimizacion para `near_same_submap` de error alto;
- `prueba_76`: scenario `success=true`; el contador causal aparece con valores
  positivos, los cuatro calculos 3Q carecen de soporte previo y terminan sin
  commit. No hay ningun `F1Q-LOOP-OPT-COMMIT` ni movimiento de poses atribuible
  a loops;
- evidencia negativa o ausente: esta ejecucion no produjo un loop legitimo de
  error alto con soporte suficiente, por lo que no valida un accept positivo
  del nuevo grafo;
- conclusion agregada: `PARCIAL`. La regresion que movia todos los KFs queda
  corregida, mientras el cierre algoritmico de accepts validos sigue abierto;
- siguiente paso recomendado: mantener la prueba 76 como regresion negativa y
  anadir en `3V` un caso controlado de loop positivo con error y soportes
  conocidos.

## 2026-08-17 - Preparacion cerrada de la reimplementacion covisible

- objetivo acordado: rehacer 3Q como optimizacion SE(3) covisible comun para
  loop relativo y fiducial absoluto, reutilizando 3I-3L y sin infraestructura
  secundaria duplicada;
- alcance: grafo persistente ligero de constraints, subgrafo minimo hasta
  autoridades hard, fusiones previas soft, dependencias blandas, covisibilidad
  confirmada, controles base 30 % ampliables, candidato privado, commit
  multi-submapa y fusion 3P directa;
- decision intra/inter: no excluir por dron, submapa o epoch; la geometria y el
  error deciden, conservando filtros causales, dos queries y ambiguedad;
- politica fiducial: un hijo soft dentro de umbral promociona hard sin mover;
  fuera de umbral ejecuta MAX covisible y corta dependencia solo tras accept;
- evidencia: los inliers de la tarea que obtiene el segundo apoyo se mantienen
  en memoria y se reutilizan; un stale fuerza una BAJA fresca que recalcula;
- aceptacion inicial: loop solo `ACCEPT_FULL`; una fusion posterior omitida no
  revierte poses validas; fusiones anteriores se miden pero no se endurecen
  automaticamente;
- scheduling: la optimizacion sigue dentro de la `LoopTask` BAJA y no se
  interrumpe, pero activa `stop_drones` desde branch begin hasta task end,
  incluida la fusion posterior; MAX pendiente empieza despues;
- pruebas acordadas: tests deterministas, replay y diez topologias Gazebo
  naturales sin offset artificial. Si una ejecucion no produce error alto se
  informa y el usuario decide una repeticion. Cuatro casos requieren revision
  RViz2/web;
- archivos modificados: contratos `subfase_3Q*.md` y notas de sucesion en
  3H-3L/3O/3P; no se modifico codigo, launch ni YAML;
- build/pruebas: no ejecutados, porque esta entrada documenta preparacion y no
  implementacion;
- conclusion: acuerdo funcional `CERRADO`; implementacion pendiente de una
  orden explicita posterior;
- siguiente paso recomendado: recibir autorizacion funcional, implementar por
  bloques con tests tempranos y ejecutar la matriz Gazebo de menor a mayor.
