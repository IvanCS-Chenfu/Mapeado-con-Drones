# Historial 3Q - resumen

## Estado vigente

`CONSEGUIDA PARA EL CIERRE DE FASE 3; MEJORA FUTURA DOCUMENTADA`.

3Q ejecuta `OptimizationEvidence` dentro de la misma `LoopTask` BAJA y reutiliza
builder, solver, validator, store y fusion para fiducial absoluto y loop
relativo. No crea cola, worker ni solver duplicados.

## Implementacion vigente

- grafo SE(3) comun con hard fiduciales fijos, temporal, covisibilidad nativa,
  loops/fusiones server, dependencias soft y ventanas multi-submapa;
- regiones robustas de error alto dominan sobre fusiones de la misma tarea;
  cada `CurrentLoop` queda cubierto y validado antes del commit;
- cierre transitivo por todas las fusiones, sin bonus artificial, y validacion
  de estructura temporal, covisible, fusiones previas y hard;
- corredores hard-hard referidos al ultimo commit fiducial: un exceso heredado
  puede mantenerse o reducirse, pero no crecer;
- un epoch nunca anclado conserva anclaje loop libre; un epoch nuevo nacido
  tras perder tracking anclado debe respetar una envolvente raw por KF;
- commit multi-submapa con rebase sobre estado vigente. Intermedios caducados se
  omiten; extremos culled con raw estable pueden actuar como apoyo virtual sin
  reactivarse ni escribirse; se exigen dos controles activos por submapa;
- todos los KFs movidos se reencolan como `FusionRefresh`: recalculan
  BoW/RANSAC/fusion/score, pero no inician optimizacion recursiva. Las tareas
  normales siguen siendo `Full` y prevalecen al coalescer;
- los refresh se agrupan por region y filtran candidatos por AABB world; sus
  pendientes son mantenimiento y no mantienen por si solos backpressure;
- un precheck anterior al builder rechaza loops >5 m/20 grados cuando ambos
  extremos pertenecen a regiones fiduciales/corredores protegidos. El ledger
  regional revisionado evita repetirlos en KFs vecinos; un solo lado fiable
  conserva la optimizacion asimetrica;
- `stop_drones` cubre grafo, solve, validacion, commit y fusion directa; la
  tarea sigue BAJA, no preemptiva, y el principal continua incorporando raw;
- dirty sets contienen solo KFs activos realmente movidos; raw no se modifica
  y el worker secundario no publica.

## Evidencia final

- build final: 2/2 (`orbslam3_multi`, `orbslam3_server`), exit 0;
- regresiones finales directas: grafos/validacion 14/14, pipeline 9/9 y cola
  6/6; CTest `orbslam3_multi` 9/9, web 1/1 y servidor funcional 4/4;
- regresion de perdida: una hipotesis repetida a unos 190 m no ancla el epoch
  perdido, mientras el primer epoch nunca anclado sigue libre;
- prueba 187 corta: `success=true`, tres optimizaciones/tres commits, 16
  `FusionRefresh` altos diferidos, 1047 tareas, `pending=0`, cero hard failures;
- prueba 188 larga: 25 etapas y dos vueltas, scenario/tool exit 0,
  `success=true`; 26 propuestas loop, nueve commits `Full` y 17 rechazos
  estructurales sin escritura;
- los commits loop 188 reducen el error medio de traslacion de 0.469849 a
  0.089286 m; solve maximo 6765.81 ms y commit maximo 11.873 ms;
- fiduciales 188: nueve accepts full, ocho commits atomicos y cero hard
  failures; dos submapas se anclan por loop;
- fusion 188: 995/1196 commits y 84103 pares comprometidos, incluidos cinco
  commits posteriores a optimizacion;
- recursos 188: MemAvailable minimo 5166.6 MiB, servidor RSS maximo 423.4 MiB,
  grupo 2014.6 MiB, PSI memoria cero y guarda inactiva.
- prueba 191 larga: dos vueltas completas, `success=true`, cola final cero,
  2104 secundarias y cero hard failures; cinco rechazos protegidos, 42 hits del
  ledger y 18 refresh sin candidatos espaciales;
- los gates 191 se liberan siempre y el maximo baja de 358.8 s en la prueba 189
  interrumpida a 80.272 s. Mantenimiento puro nunca activa backpressure;
- recursos 191: servidor RSS maximo 362.1 MiB, grupo 1948.8 MiB, PSI memoria
  cero y guarda inactiva.

## Fallos conservados

- 176 y 179 siguen `NO CONSEGUIDAS` visualmente: 179 acepto una hipotesis de
  unos 27 m sin validar la estructura previa;
- 180 rechazo exceso de corredor heredado; 181 mantuvo una revision redundante;
  182/184 localizaron controles culled; 185 conservo extremos demasiado
  estrictos; 186 creo una realimentacion de 78 commits desde reruns post-opt;
- 187 demuestra que `FusionRefresh` elimina esa realimentacion;
- 189 se conserva `NO CONSEGUIDA`: repetitividad regional genero cinco solves
  de 63-70 s y un gate de 358.8 s antes de la interrupcion;
- 191 conserva una limitacion de coste: 31 rejects post-solver y una ventana de
  786 KFs/83.44 s. No reproduce el bloqueo, pero la seleccion multi-region y
  los umbrales pueden perfeccionarse mas adelante;
- un CTest intermedio 8/9 fallo por fixtures sin `CurrentLoop`; la correccion
  mecanica deja 9/9. Los fallos globales restantes son linters historicos de
  `legacy2` y formato previo ajeno al alcance.
- revision de la prueba 194: el dron 2 antihorario recibe dos commits loop
  consecutivos antes del fiducial 2. El dominante corrige `3.950 m/0.454 rad`,
  mueve 359 KFs y admite `0.686 m` de incremento estructural; el siguiente
  corrige `0.780 m/0.078 rad` sobre 362 KFs. Ambos eran asimetricos
  (`query` no protegida, candidato protegido), ambiguos y con diez competidores.
  El fiducial posterior queda dentro de umbral y no reoptimiza el interior.

## Cierre

La prueba 191 termina con cola vacia y corrige el bloqueo operativo de 189. La
seleccion multi-region aun puede iniciar ventanas grandes cuando la relacion
directa protegida es coherente; queda como perfeccionamiento futuro acordado.
La prueba 195 no reproduce la mala optimizacion final y el usuario confirma que
RViz2 se vio perfecto; las correcciones observadas ocurrieron al alcanzar el
fiducial. 3Q queda conseguida para cerrar Fase 3, sin ocultar 194.

Como mejora futura, un candidato cercano podria requerir dos apoyos
independientes y uno lejano/ambiguo un umbral creciente de hasta 8-10 antes de
una unica optimizacion. No se implementa en este cierre. El punto de reentrada
continua siendo admision asimetrica, ambiguedad, tamaño de ventana y validacion
de deformacion.

Detalle cronologico: `historial_3Q.md`.
