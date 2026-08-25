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

## 2026-08-17 - Implementacion covisible comun autorizada

- objetivo intentado: ejecutar el acuerdo 3Q completo sin cola, worker ni
  solver duplicados;
- archivos modificados: tipos/problema de grafo, builder, solver, validator,
  `GlobalPoseStore`, `LoopPipeline`, `SparseGlobalBackend`, servidor, grafo web
  y tests de dominio/cola/contrato visual;
- implementacion: constraints fiduciales absolutas y loops relativas, ventanas
  multi-submapa, hard fijos, temporal/covisibilidad/constraints previas,
  commit atomico, tails/dependencias soft, fusion 3P directa y lifecycle con
  `stop_drones`;
- build: `orbslam3_multi orbslam3_server simulacion_dron`, correcto;
- tests deterministas: bateria critica 40/40 tras las correcciones sucesivas;
- conclusion: `PARCIAL`; codigo integrado, pendiente validacion Gazebo.

## 2026-08-17 - Prueba 162 - arranque fallido

- objetivo intentado: primera prueba tipica larga con dos fiduciales;
- resultado: `success=false`, exit 1 tras dos intentos de arranque;
- evidencia negativa: `pipeline_flow_bridge` no pudo abrir el puerto ocupado y
  la herramienta clasifico Gazebo como muerto durante startup;
- conclusion: `NO CONSEGUIDA`; fallo de entorno/puerto, sin evidencia 3Q.

## 2026-08-17 - Prueba 163 - primer recorrido integrado

- objetivo intentado: repetir la prueba tipica en puerto libre;
- resultado: scenario/tool exit 0 y `success=true`;
- evidencia: lifecycle secundario completo, RANSAC real y loops same-submap
  tratados como diagnostico, no como anclajes de otro componente;
- aprendizaje: la rama integrada funciona, pero aun faltaba ejercitar y
  endurecer ventanas 3Q grandes;
- conclusion: `PARCIAL`.

## 2026-08-17 - Prueba 164 - grafo covisible demasiado denso

- objetivo intentado: recorrido largo con optimizaciones naturales;
- resultado: ejecucion util para diagnostico;
- evidencia negativa: la covisibilidad nativa promovia demasiadas aristas y
  controles en ventanas grandes, elevando coste y rigidez;
- correccion posterior: limitar a seis aristas nativas fuertes por control sin
  eliminar endpoints obligatorios;
- conclusion: `PARCIAL`.

## 2026-08-17 - Prueba 165 - conflictos fiduciales no recuperados

- objetivo intentado: validar convivencia de loop y optimizacion fiducial;
- resultado: timeout; recursos conservados en artefactos de la prueba;
- evidencia negativa: conflictos de revision fiducial terminaban el intento
  sin volver a encolar una tarea MAX fresca;
- correccion posterior: completar lifecycle y reencolar fiducial tras
  revalidacion, sin preemption ni escritura stale;
- conclusion: `NO CONSEGUIDA`.

## 2026-08-17 - Prueba 166 - commits reales y metrica loop incorrecta

- objetivo intentado: escenario dirigido tras el retry fiducial;
- resultado: `success=true`, recursos estables, cola final cero;
- evidencia positiva: cinco conflictos fiduciales reencolados y commit de 137
  KFs, con movimiento posterior al control;
- evidencia negativa: numerosas optimizaciones loop eran falsos positivos
  porque el error comparaba anchors estaticos en vez de las poses KF actuales;
- correccion posterior: medir relacion world actual contra relacion RANSAC;
- conclusion: `PARCIAL`.

## 2026-08-20 - Prueba 167 - metrica corregida sin loop alto natural

- resultado: `success=true`, recursos estables y `pending=0`;
- evidencia: tres conflictos fiduciales reencolados; el cuarto intento mueve
  140 KFs y propaga 16 KFs del hijo soft;
- ausencia relevante: todos los loops confirmados quedaron en error bajo y no
  hubo optimizacion loop natural tras corregir la metrica;
- conclusion: `PARCIAL`; regresion fiducial positiva, accept 3Q no ejercitado.

## 2026-08-20 - Prueba 168 - accepts naturales en recorrido largo

- resultado: `success=true`, recorrido completo, recursos estables y
  `pending=0`;
- evidencia 3Q: 17 intentos, cinco commits, nueve stale y tres rejects sin
  escritura; un commit multi-submapa continuo hasta fusion directa;
- evidencia fiducial: cinco conflictos recuperados y commit de 148 KFs;
- aspecto negativo: ventanas de 209/212 KFs tardaban alrededor de ocho
  segundos y aun sufrían bastante invalidacion concurrente;
- conclusion: `PARCIAL POSITIVA`; funcionalidad demostrada, rendimiento abierto.

## 2026-08-20 - Prueba 169 - commit loop excesivamente estricto

- escenario: G1 dirigido con `tray_prueba_155.yaml`;
- resultado de herramienta: `success=true`, recursos estables;
- evidencia negativa: 45 intentos 3Q, solo tres commits y 42 stale; ventanas de
  hasta 221 KFs/80 controles y una optimizacion activa al cierre;
- causa: commit exigia revisiones exactas aunque la correccion pudiera
  rebasarse sobre el estado vigente compatible;
- conclusion: `NO CONSEGUIDA`.

## 2026-08-20 - Prueba 170 - rebase compatible efectivo

- resultado: `success=true`, sin hard failures y recursos estables;
- evidencia: seis commits de nueve intentos, incluyendo lotes multi-submapa de
  154, 198 y 222 KFs; B se ancla por loop y el commit fiducial padre mueve 235
  KFs y propaga 92 del hijo;
- evidencia negativa: tras 90 s quedaron 31 tareas 3P pendientes, por lo que no
  se demostro drenaje final;
- conclusion: `PARCIAL`.

## 2026-08-20 - Prueba 171 - solver de ventana grande no termina

- resultado de escenario/herramienta: `success=true`; 300 s de drenaje y
  recursos estables (`server RSS 175.1 MiB`, PSI memoria cero);
- evidencia: backlog 3P baja de 28 a 1, pero la tarea `1000000003779` emite
  `F3Q-OPT-START` y no emite final durante mas de tres minutos;
- diagnostico: coste del solver en 160 iteraciones, no deadlock de cola/fusion;
- correccion posterior: convergencia practica desde iteracion 24 y telemetria
  `graph/solve/validation/commit`;
- conclusion: `NO CONSEGUIDA`.

## 2026-08-20 - Prueba 172 - solver corregido, drenaje corto

- resultado: `success=true`, 680 s, `server RSS 157.7 MiB`, PSI cero;
- evidencia 3Q: diez starts con diez ends; ventanas de 207-212 KFs resuelven en
  1.66-2.32 s con 24 iteraciones y las rutas de 160 en unos 4.0 s; commits
  multi-submapa hasta 205 KFs y B anclado por loop;
- evidencia negativa: el drenaje de 240 s acaba mientras la cola baja 27->23;
  las tareas restantes son retries de fusion, no un solver activo;
- conclusion: `PARCIAL`.

## 2026-08-20 - Prueba 173 - tarea fiducial obsoleta bloquea el gate

- resultado: `success=false`, runner/tool exit 124 tras timeout de 800 s;
  drenaje adicional 360 s, recursos estables y cola finalmente cero;
- causa: una tarea fiducial antigua queda detras del nuevo control y el builder
  devuelve `target_not_after_control`; el servidor lo convierte en
  `F3L-HARD-FAILURE` y deja `blocking_failure=true`;
- correccion posterior: `RevalidateFiducialTask()` clasifica target igual o
  anterior al control vigente como `Stale` antes de construir grafo;
- regresion: `target_not_newer_than_current_control` en backend;
- conclusion: `NO CONSEGUIDA`.

## 2026-08-20 - Prueba 174 - 3Q correcto, backlog 3P reproducido

- resultado: `success=true`, 733 s, recursos estables, sin hard failure;
- evidencia 3Q: ocho starts/ends y ocho commits; ventanas hasta 219 KFs/93
  controles, solve maximo 3.870 s;
- evidencia negativa: cola final 25; 696 intentos de fusion, 63 commits, 316
  stale y 12 rollback. Maximos: 70568 proyecciones, visibilidad 1964.55 ms y
  prepare 2001.86 ms;
- causa: cada hard-outlier reconstruia los mismos depth buffers query/candidate;
  el prepare de segundos chocaba continuamente con deltas;
- conclusion: `PARCIAL`.

## 2026-08-20 - Prueba 175 - cierre tecnico vigente

- objetivo: validar 3Q tras compartir un depth buffer por direccion/region sin
  omitir ninguna evidencia negativa;
- build/tests previos: 3/3 paquetes, 3/3 fusion y bateria critica 40/40;
- resultado: `success=true`, scenario/tool exit 0, 613 s y 180 s de drenaje;
  `server RSS max 146.4 MiB`, group 1330.8 MiB, PSI memoria cero y guard false;
- evidencia 3Q: B se ancla por loop; ocho starts/ends, siete commits iniciales
  y un stale reintentado hasta commit; ventanas de dos/tres submapas, maximo
  174 KFs, 89 controles y solve 1.892 s; cola final `pending=0`;
- evidencia 3P: 155 intentos, 129 commits y 10 stale; maximos 1848
  proyecciones, 57.363 ms de visibilidad y 142.766 ms de prepare;
- invariantes: cero hard failure/excepcion, raw no escrito por secundario,
  lifecycle no preemptivo y fusion directa dentro de la misma BAJA;
- evidencia ausente: no se ejecuto literalmente toda la matriz de diez
  escenarios y RViz2/grafo web de esta tanda no han sido revisados por el
  usuario;
- conclusion: `PARCIAL` a nivel de subfase por visual/matriz pendiente;
  implementacion y validacion tecnica `CONSEGUIDAS`;
- siguiente paso recomendado: revision visual del usuario y, si es correcta,
  cerrar 3Q antes de iniciar la siguiente subfase.

## 2026-08-20 - Prueba 176 intento 1 - invocacion relativa invalida

- objetivo: ejecutar dos rodeos consecutivos para observacion directa del
  usuario;
- resultado operativo: la trayectoria no comenzo porque el runner recibio una
  ruta YAML relativa; `scenario exit=1` y cierre manual de la espera, exit 130;
- artefactos: recursos preservados con sufijo `intento_1`; no se leyeron ni
  analizaron logs;
- conclusion: `NO EJECUTADA`; correccion mecanica mediante ruta absoluta.

## 2026-08-20 - Prueba 176 - rodeo doble para revision del usuario

- trayectoria: `tray_prueba_176.yaml`, 25 pasos, dos vueltas completas y 180 s
  de observacion final;
- resultado operativo: `scenario exit=0`, `success=true`, herramienta exit 0
  y 854 s monitorizados;
- artefactos: log completo preservado sin lectura directa; analisis posterior
  realizado exclusivamente sobre reducciones tematicas de escenario, loop,
  fiducial, lifecycle 3Q y publicaciones;
- observacion visual del usuario: al final de la primera vuelta el dron
  antihorario mostraba paredes dobles; la correccion parecio llegar solo al
  fiducial y no elimino el desdoblamiento. Al final de la segunda vuelta una
  correccion loop grande si resulto visualmente efectiva;
- cronologia objetiva: antes y alrededor del primer regreso a fiducial 2 se
  comprometieron loops para KFs 272, 275 y 274. El primero redujo
  `0.920261 m/0.058172 rad` a `0.069910 m/0.004369 rad`, el segundo
  `0.380217/0.031158` a `0.046925/0.003085` y el tercero
  `0.582500/0.139929` a `0.045976/0.011769`;
- retraso confirmado: desde KF229 y, con mas fuerza, KF244/KF260-KF271 hubo
  geometria aceptada con errores aproximados de `0.88-1.59 m`, pero las tareas
  terminaron `waiting_independent_support` hasta reunir el segundo apoyo;
- decision insuficiente: tras el fiducial, tareas como KF283, KF285, KF286 y
  KF288 mezclaron regiones por debajo de `0.35 m` con otras de
  `0.4048-0.4478 m`. La politica vigente hizo que cualquier region fusionable
  dominase toda la tarea, por lo que no optimizo las regiones de error alto;
- fiducial/publicacion: la optimizacion fiducial de `(2,2)` comprometio la
  ventana y llevo el target a error cero. El siguiente principal publico y
  recalculo 121 KFs/7878 MPs; por tanto, no fue un fallo de dirty sets ni de
  `GlobalMapBuilder`. Sin embargo, `PoseGraphBuilder::Build()` solo recibe el
  snapshot de ese submapa: sus aristas covisibles son intra-ventana y no
  reconcilian conjuntamente otros submapas o paredes ya relacionadas;
- segunda vuelta: la tarea del KF496 fue una optimizacion loop real de dos
  submapas y 210 KFs; redujo `3.506835 m/0.305806 rad` a
  `0.087014 m/0.007946 rad`, movio 211 KFs y fue publicada. Coincide con la
  mejora visual observada. La posterior del KF518 resolvio pero quedo stale por
  conflicto de revision;
- diagnostico: la hipotesis del usuario es sustancialmente correcta, aunque no
  basta con bajar un unico umbral. Intervienen el umbral de fusion de `0.35 m`,
  el dominio de fusion sobre evidencia mixta, la espera de apoyo independiente,
  una sola arista `CurrentLoop` elegida por inliers y el alcance mono-submapa
  de la rama fiducial;
- conclusion: `NO CONSEGUIDA` para el criterio visual de esta ejecucion. La
  arquitectura y los commits funcionan, pero 3Q permanece `PARCIAL` y requiere
  revisar la decision fusion/optimizacion y ampliar la consistencia comprobada
  por el grafo antes de su cierre.

## 2026-08-20 - Prueba 177 - repeticion larga agotada

- objetivo intentado: repetir exactamente el YAML de 176 tras las primeras
  correcciones de decision y alcance covisible;
- resultado: runner/tool exit 124 por timeout a 900 s, 945 s monitorizados;
- recursos: servidor RSS maximo 190.2 MiB, grupo 1434.6 MiB, PSI memoria cero
  y guarda inactiva;
- evidencia negativa: el recorrido no termino dentro del limite y no sirve
  como cierre funcional ni visual;
- conclusion: `NO CONSEGUIDA`.

## 2026-08-20 - Prueba 178 - continuidad concurrente incompleta

- build/tests: tres builds 3/3 correctos; el primer CTest de la regresion de
  culling termino 8/9 por conservar como control de continuidad un KF ya
  inactivo. Tras corregir el fixture y la continuidad activa, CTest paso 9/9,
  web 9/9, fusion 3/3 y bateria critica 40/40;
- prueba: mismo YAML largo de 176, runner/tool exit 124 a 1100 s;
- recursos: servidor RSS maximo 187.2 MiB, grupo 1445.7 MiB, PSI memoria cero
  y guarda inactiva;
- evidencia: los grafos fiduciales convergian, pero los commits
  multi-submapa caducaban cuando un control intermedio era culled durante el
  solve;
- conclusion: `NO CONSEGUIDA`; motivo concurrente localizado.

## 2026-08-20 - Prueba 179 - falso loop estructural observado

- resultado: scenario/tool exit 0, `success=true`, 804 s monitorizados;
- recursos: servidor RSS maximo 326.9 MiB, grupo 1882.7 MiB, PSI memoria cero
  y guarda inactiva;
- observacion visual del usuario: tras el primer fiducial varios KFs quedaron
  en posiciones absurdas; vueltas posteriores repararon gran parte del mapa;
- diagnostico reducido: la tarea `1000000003083` acepto dos regiones de una
  hipotesis de unos 27 m/1.61 rad, llevo el residual actual a
  0.083 m/0.007 rad y movio 22 KFs sin medir degradacion temporal,
  covisible, de fusiones previas o de corredores hard-hard;
- conclusion: `NO CONSEGUIDA`; origina la guarda de perdida, el cierre
  transitivo, los residuales estructurales y la referencia hard-hard.

## 2026-08-21 - Prueba 180 - corredor absoluto provoca tormenta

- cambios/build/tests: continuidad tras perdida, cierre por todas las fusiones,
  validacion estructural, corredor y rerun de todos los KFs movidos integrados;
  cuatro builds 3/3 correctos;
- resultado: runner/tool exit 124 a 1200 s, con 120 s de drenaje;
- evidencia: primera vuelta completa; paso 15 espera 697.771 s. Setenta
  optimizaciones terminan y ninguna compromete: 58 rechazos de corredor, 11
  temporales y uno covisible. El worker procesa 1744 tareas y drena a cero;
- causa: el corredor rechazaba exceso absoluto heredado aunque la propuesta lo
  redujera. No hubo deadlock, hard failure ni presion de memoria;
- correccion: comparar exceso before/after; un exceso previo puede conservarse
  o mejorar, nunca aumentar. Build 3/3, CTest 9/9, servidor 4/4 y web 1/1;
- conclusion: `NO CONSEGUIDA`, con causa y regresion opuesta cerradas.

## 2026-08-21 - Prueba 181 - conflicto de revision redundante

- resultado: runner/tool exit 124 a 1200 s y 120 s de drenaje;
- evidencia: el paso 8 espera mas de 1006 s; 267 inicios, 266 finales, cero
  commits y 266 `revision_conflict` sobre propuestas estructuralmente validas;
- causa: `CommitGraphProposal()` rebasaba bajo `state_commit_mutex_` y aun
  exigia una `pose_revision` anterior al solve, causando stale determinista;
- correccion: commit sobre poses vigentes con `expected_pose_revision=0`,
  manteniendo raw revision/drift, controles, hard y atomicidad. Build 3/3;
  tests directos 14/14, 9/9 y 5/5. El CTest integrado no se inicio por limite
  externo de aprobaciones, no por fallo del proyecto;
- conclusion: `NO CONSEGUIDA`, conflicto redundante eliminado.

## 2026-08-21 - Prueba 182 - motivo de commit aun opaco

- objetivo: escenario corto para aislar el stale sostenido;
- resultado: runner/tool exit 124 a 480 s, 60 s de drenaje;
- evidencia: tres commits iniciales y despues 117 `revision_conflict`; el paso
  8 espera el gate hasta timeout. Recursos estables y guarda inactiva;
- correccion mecanica posterior: `AcceptedPoseBatchResult::detail` distingue
  snapshot, control, raw drift, continuidad, hard y store; build 2/2 y tests
  directos 14/14, 9/9 y 5/5;
- conclusion: `NO CONSEGUIDA`; telemetria preparada para separar la causa.

## 2026-08-21 - Prueba 183 - YAML relativo

- resultado: no ejecutada funcionalmente; runner/tool exit 1 porque el YAML
  relativo no se resolvio desde el workspace del script;
- evidencia: launch arranco y cerro, recursos estables, sin recorrido 3Q;
- conclusion: `NO EJECUTADA`; repeticion mecanica como 184 con ruta absoluta.

## 2026-08-21 - Prueba 184 - controles intermedios caducados

- resultado: runner/tool exit 124 a 480 s, 60 s de drenaje;
- evidencia: 33 finales 3Q, un commit, 32
  `commit_control_missing_or_inactive` y un `commit_control_raw_drift`; primer
  gate liberado tras 34.032 s y segundo bloqueado;
- causa: culling/refinado de controles intermedios durante el solve;
- correccion: rebase selectivo que omite solo intermedios caducados, conserva
  extremos loop/fixed/hard y exige dos controles actuales por submapa. Build
  2/2; tests 14/14, 9/9 y 5/5;
- conclusion: `NO CONSEGUIDA`, precondicionante exacto localizado.

## 2026-08-21 - Prueba 185 - apoyo obligatorio culled

- resultado: `success=true`, scenario/tool exit 0, 485 s incluidos 60 s de
  drenaje;
- evidencia: cinco commits, uno con cinco intermedios omitidos; 78 retries
  terminan por control obligatorio inactivo y el ultimo gate espera 170.536 s;
  quedan 15 tareas y una optimizacion activa;
- correccion: un extremo culled con raw estable puede ser apoyo virtual de
  correccion, sin reactivarse ni escribirse; siguen siendo necesarios dos
  controles activos por submapa. Build 2/2; tests 14/14, 9/9 y 5/5;
- conclusion: `PARCIAL`; recorrido completo pero tormenta stale no resuelta.

## 2026-08-21 - Prueba 186 - realimentacion post-opt

- resultado: runner/tool exit 124 a 480 s, 90 s de drenaje;
- evidencia: desaparecen los stales de control y el primer gate baja a
  3.203 s, pero 78/81 intentos comprometen usando apoyo virtual, mueven de
  media 211.3 KFs y se reencolan entre si; el siguiente gate espera 332 s;
- causa: los reruns destinados a fusion volvian a iniciar optimizaciones;
- correccion: `LoopTaskIntent::{Full,FusionRefresh}`. Los reruns post-opt
  mantienen BoW/RANSAC/fusion/score, pero difieren error alto; `Full` prevalece
  al coalescer. Build 2/2; tests 14/14, 9/9 y cola 6/6;
- conclusion: `NO CONSEGUIDA`, realimentacion identificada y corregida.

## 2026-08-21 - Prueba 187 - validacion corta final

- resultado: `success=true`, scenario/tool exit 0, 342 s incluidos 90 s de
  drenaje;
- evidencia: gates de 5.604 s y 10.109 s; tres optimizaciones/tres commits
  frente a 78 en 186; 16 refresh de error alto diferidos, 1047 tareas
  completadas, ultimo `pending=0`, cero hard failures y errores graves;
- telemetria: el intent efectivo queda en dequeue y `F3Q-OPT-START`; un
  `FusionRefresh` coalescido con un `Full` se ejecuta correctamente como full;
- build final posterior: 2/2 en 17.0 s; tests directos 14/14, 9/9 y 6/6;
- conclusion: `CONSEGUIDA` para el escenario corto.

## 2026-08-21/22 - Prueba 188 - rodeo doble final

- objetivo: repeticion exacta de `tray_prueba_176.yaml`, 25 etapas, dos vueltas,
  timeout 1200 s y 180 s de drenaje;
- resultado: `success=true`, scenario/tool exit 0; 1249 s monitorizados;
- optimizacion loop: 26 propuestas; nueve commits `Full` y 17 rechazos sin
  escritura (16 corredor, uno temporal). Los nueve commits reducen en promedio
  el error de traslacion de 0.469849 a 0.089286 m; maximos: 87 KFs de ventana,
  61 controles, 92 KFs movidos, solve 6765.81 ms, validacion 1.614 ms y commit
  11.873 ms. Cinco commits usan apoyos virtuales culled, sin escribirlos;
- fiduciales: 126 observaciones, 31 revalidaciones (9 ready, 22 stale), nueve
  validaciones `accept_full`, ocho commits atomicos, 1006 KFs movidos en total
  y maximo 289; cero hard failures;
- loop/fusion: dos anchors loop confirmados; 1196 intentos 3P, 995 commits,
  990 fusiones normales y cinco posteriores a optimizacion. Se comprometieron
  84103 pares; hubo 55 stale por dependencias, 136 por score y diez no-op;
- recursos: MemAvailable minimo 5166.6 MiB, servidor RSS maximo 423.4 MiB,
  grupo 2014.6 MiB, ORB 881.2 MiB, CPU sistema 57.46 %, PSI memoria cero y
  guarda inactiva;
- carga residual: gates maximos 109.985 s y 197.754 s por reevaluacion/fusion.
  Tras el drenaje la cola seguia descendiendo de 323 a 310, sin
  `blocking_failure`; el criterio admitia cola drenada o claramente
  descendente;
- tests asociados: un CTest intermedio 8/9 fallo porque tres fixtures no
  incluian su arista `CurrentLoop`; corregidos mecanicamente, CTest
  `orbslam3_multi` paso 9/9, contrato web 1/1 y servidor funcional 4/4. Los
  fallos restantes del CTest global son linters historicos de `legacy2` y
  formato previo, fuera de 3Q;
- evidencia ausente: RViz2 y grafo web de esta ejecucion no han sido revisados
  por el usuario;
- conclusion: validacion automatica `CONSEGUIDA`; conclusion agregada 3Q
  `PARCIAL` solo hasta recibir confirmacion visual de esta prueba.

## 2026-08-22 - Prueba 189 - bloqueo aparente por repetitividad

- resultado: interrumpida manualmente a peticion del usuario; no se reescribe
  como prueba superada;
- diagnostico reducido: no hubo deadlock. El paso 19 espero 358.8 s con
  backpressure porque cinco KFs vecinos de una zona repetitiva lanzaron solves
  consecutivos de 63-70 s y fueron rechazados solo despues por estructura;
- un fiducial MAX espero correctamente a la tarea BAJA no preemptiva y despues
  comprometio, pero genero 520 reruns post-opt y la cola alcanzo 619;
- causa: el rechazo negativo era por pareja/revision exacta y no se extendia a
  una region vecina; los `FusionRefresh` tampoco estaban acotados espacialmente
  y todo mantenimiento contaba para cerrar el mission gate;
- conclusion: `NO CONSEGUIDA`; correccion funcional acordada antes de editar.

## 2026-08-22 - Correccion regional y prueba 191

- cambios: precheck despues de RANSAC y antes del builder para regiones
  protegidas, margen 5 m/20 grados y asimetria si solo un lado es fiable;
  ledger regional revisionado; agrupado temporal y filtro AABB world de
  `FusionRefresh`; pending secundario separado en `critical/maintenance` y
  backpressure basado solo en criticos;
- build: 3/3 (`orbslam3_multi`, `orbslam3_server`, `simulacion_dron`), exit 0;
  regresiones dirigidas 30/30, CTest `orbslam3_multi` 9/9 y servidor funcional
  4/4. El CTest global conserva solo lint historico de `legacy2`;
- intento 190: no ejecutado funcionalmente porque el helper rechazo aliases de
  argumentos antiguos; se repitio sin sobrescribir como 191 con `--yaml` y
  `--timeout-sec`;
- resultado 191: dos vueltas/25 etapas, `success=true`, scenario/tool exit 0,
  1046 s incluidos 180 s de drenaje; cierre `pending=0`, 2104 tareas,
  `hard_failed=0`;
- admision: cinco rechazos protegidos antes del grafo, 42 hits regionales y 18
  refresh sin candidatos espaciales. Trece commits post-opt agrupan 2443 KFs
  movidos en 195 regiones y encolan 185 refresh; mantenimiento puro nunca
  activa backpressure;
- gates: ocho esperas, todas liberadas; maximo 80.272 s frente a los 358.8 s
  sin progreso de 189. Recursos estables: servidor RSS 362.1 MiB, grupo
  1948.8 MiB, PSI memoria cero y guarda inactiva;
- limitacion conservada: 40 solves, nueve commits y 31 rejects. Dos hipotesis
  de 20-22 m tardaron unos 17 s porque solo el candidato era protegido y se
  respeto la asimetria acordada. Otra ventana de 786 KFs tardo 83.44 s: ambos
  lados eran protegidos, pero la relacion directa tenia error casi cero y otra
  region disparo la optimizacion; queda como seleccion/umbral futuro;
- conclusion: correccion del bloqueo 189 `CONSEGUIDA` automaticamente. La
  revision RViz2/grafo web de 191 queda pendiente del usuario.

## 2026-08-22 - Conclusion visual del usuario

- el usuario valora muy positivamente el resultado y decide avanzar a la
  siguiente subfase;
- 3Q queda `A REVISAR`, no cerrada definitivamente, debido al error visual
  comunicado anteriormente y al coste residual observado en optimizaciones;
- si pruebas futuras muestran problemas de loops, dobles paredes,
  optimizaciones o ventanas grandes, se reabrira 3Q desde esta evidencia;
- conclusion agregada: `A REVISAR; ACEPTADA PARA CONTINUAR`.

## 2026-08-22 - Revision posterior de prueba 194 - optimizacion final dron 2

- No es una nueva ejecucion: se revisa tematicamente la prueba 194 tras la
  observacion visual del usuario. No se modifica codigo ni configuracion.
- Tramo: el dron 2, que rodea en sentido antihorario, termina el paso 12 y
  vuelve al fiducial 2 en el paso 13.
- Causa principal probable: `task=1000000005308`, query `(2,1,220)`, acepto
  durante el final del paso 12 una correccion loop de `3.950 m/0.454 rad`.
  Uso una sola arista loop sobre 296 KFs/115 controles/3 submapas, movio 359
  KFs y permitio incremento estructural maximo `0.686 m/0.111 rad`.
- Evidencia de admision 5308: tres RANSAC con ratios `0.278-0.388`, residuales
  medios `0.124-0.147 m`, diez competidores y `ambiguity=true`. El precheck vio
  `query=false,candidate=true`: al estar solo un extremo protegido, conservo la
  optimizacion asimetrica acordada y no aplico el rechazo 5 m/20 grados.
- Amplificacion: `task=1000000005421`, query `(2,1,228)`, fue encolada antes
  del paso 13 y ejecutada mientras el dron avanzaba hacia fiducial 2. Corrigio
  otros `0.780 m/0.078 rad`, movio 362 KFs de una ventana de 303 y alcanzo
  incremento estructural `0.335 m/0.049 rad`, con la misma asimetria protegida.
- Ambas optimizaciones comprometieron poses (`F3Q-LOOP-OPT committed=true`).
  La `LoopTask` termino stale solo en su fusion directa por
  `fusion_dependencies_changed_before_commit`; esa salida no revierte el
  subcommit de pose ya aplicado. Los retries posteriores caducaron por cambios
  de revision.
- El validador actuo conforme a sus limites: permite incrementos de hasta
  2 m/0.70 rad en temporal y 1 m/0.50 rad en covisibilidad. Los maximos medidos
  quedaron dentro; el corredor hard-hard no podia proteger aun el interior del
  tramo porque el segundo apoyo fiducial llego despues.
- Fiducial 2 no origino la deformacion: la visita 6 comienza despues de ambos
  loops. Su primer KF coherente da error `0.067 m/0.032 rad` y promueve control;
  los siguientes quedan entre `0.087-0.252 m` y `0.035 rad`, todos dentro de
  umbral. Esto fija el extremo ya cercano, pero no lanza otra optimizacion que
  redistribuya el interior deformado.
- `1000000005421` fue la ultima de 13 optimizaciones loop de 194; no hubo una
  correccion 3Q posterior antes del cierre. Colas, recursos y scoring no son la
  causa.
- Conclusion: reproduccion conceptual del punto debil ya conservado en 3Q:
  evidencia RANSAC ambigua de un tramo nuevo no protegido puede mover una
  ventana multi-submapa muy grande antes de recibir su segundo hard fiducial.
  La tarea 5308 es el causante dominante mas probable y 5421 la correccion
  secundaria. 3Q permanece `A REVISAR`.
- Punto de reentrada futuro, sin aplicarlo ahora: admision de loops asimetricos,
  ambiguedad/competidores, tamaño de ventana y validacion de deformacion en
  tramos con un solo extremo hard.

## 2026-08-22 - Revision visual de prueba 195 y cierre

- No es una nueva ejecucion: se incorpora la lectura visual comunicada por el
  usuario sobre la prueba 195 ya registrada en el cierre de limpieza.
- RViz2 se vio perfecto. La mala optimizacion final de 194 no se reprodujo;
  las correcciones observadas ocurrieron al alcanzar el fiducial.
- La evidencia no borra el fallo 194 ni demuestra que toda ambiguedad futura
  sea imposible, pero permite cerrar Fase 3 sin aplicar otra correccion.
- Mejora futura documentada, no implementada: exigir dos apoyos independientes
  para candidatos cercanos y un umbral creciente de hasta 8-10 para candidatos
  lejanos/ambiguos, seguido de una unica optimizacion.
- Conclusion agregada revisada: `CONSEGUIDA PARA EL CIERRE DE FASE 3`, con
  mejora futura y punto de reentrada conservados.

## 2026-08-25 - Reentrada futura desde prueba 213 de Fase 4

- No se reejecuta 3Q ni se modifica su runtime. Se incorpora la observacion
  visual del usuario sobre la prueba 213 ya realizada.
- La mision completa 17/17 pasos y 22/22 goals, pero permanecen derivas que el
  usuario esperaba corregir mediante loops.
- El reducido contiene 15 intentos 3Q: seis commits tempranos sobre ventanas de
  30-69 KFs, con 28-155 KFs movidos, y nueve rechazos posteriores sobre
  ventanas de 288-313 KFs.
- Siete propuestas posteriores son rechazadas por
  `hard_corridor_displacement_exceeded`. Reducen claramente el error loop, pero
  crean entre 0.000416 y 0.130115 m de exceso nuevo sobre corredores cuyo exceso
  previo era cero. Dos mas terminan `prior_loop_structure_degraded`.
- Varias hipotesis muestran `ambiguity=true`, hasta 29 competidores y apoyo no
  independiente. Esto impide asumir que todos los rechazos fueran correcciones
  buenas, aunque el resultado visual indica que la politica tampoco corrigio
  suficiente deriva.
- Hipotesis inicial a contrastar: admision ambigua y ventanas grandes combinadas
  con una proteccion de corredor posiblemente sobrerrestrictiva. No se propone
  relajar hard fiducials sin correlacionar antes cada deriva con su loop.
- Conclusion: prueba 213 `A REVISAR DE NUEVO EN 3Q`. Reproducirla, correlacionar
  propuestas con la nube visible y distinguir falsos loops, mala ventana y
  rechazo excesivamente conservador antes de elegir una correccion.
