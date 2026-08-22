# Subfase 3Q - Criterios de cierre

## `CONSEGUIDA`

3Q solo puede cerrarse cuando la evidencia conjunta demuestra:

1. `OptimizationEvidence` continua dentro de la misma `LoopTask` BAJA.
2. No existen cola, worker, builder, solver, validator o pose store duplicados.
3. Fiducial y loop usan un `PoseGraphProblem`/solver SE(3) comun.
4. El loop se formula como constraint relativa medida, nunca como prior world
   artificial ni con GT.
5. Loops inter/intra dron/submapa siguen la misma decision geometrica.
6. Dos queries independientes y el control de ambiguedad preceden al solve.
7. La ventana es el subgrafo minimo conectado y no un snapshot global.
8. Dependencias soft y tramos delimitados por hard entran cuando son parte del
   camino necesario.
9. Covisibilidad confirmada participa en optimizaciones loop y fiducial.
10. El 30 % base puede ampliarse para representar constraints fuertes.
11. Todos los hard fiducials conservan pose numericamente invariable.
12. El nuevo loop/fiducial termina dentro de su umbral y baja el coste.
13. Fusiones anteriores exponen residual antes/despues y siguen siendo
    relativas blandas en esta version.
14. Reject/stale no escriben; rollback no deja estado parcial.
15. Stale/rollback encola una BAJA fresca solo tras terminar la tarea.
16. KFs tardios/futuros y tails conservan continuidad por submapa y se detienen
    ante hard posterior.
17. El primer fiducial de un hijo soft promociona o ejecuta MAX covisible segun
    error; nunca hace reanchor parcial transitorio.
18. Tras accept, la fusion 3P usa los inliers vivos sin repetir BoW/RANSAC.
19. `fusion_skipped_after_optimization` conserva una optimizacion valida.
20. `RawMapDatabase` permanece logica y revisionadamente intacta.
21. El secundario solo deja dirty sets; el siguiente principal actualiza la
    vista y publica.
22. Al entrar en 3Q se activa `stop_drones` y se libera al terminar toda la
    rama, incluida fusion, para todos los caminos de salida.
23. La tarea no cambia de prioridad ni se interrumpe; MAX pendiente empieza
    despues y antes del siguiente loop.
24. El flujo principal recibe/procesa datos durante solver/fusion.
25. Tests, replays, matriz Gazebo, recursos y revisiones visuales acordadas son
    coherentes y no dejan deadlock, crash o NaN. Tras el drenaje, la cola debe
    quedar vacia o descender de forma verificable sin `blocking_failure`.
26. Una region de error alto apoyada domina sobre regiones fusionables de la
    misma tarea; el umbral no cambia antes de validar esta politica.
27. Cada loop usa 1-3 regiones coherentes como `CurrentLoop`, deduplicadas y
    con residuales finales individuales.
28. Una unica region discordante permite como maximo un rebuild sin ella; una
    incoherencia posterior rechaza sin escribir.
29. Fiducial expande entre submapas solo por loops/fusiones server confirmados
    y dependencias soft; covisibilidad nativa solo refuerza los ya incluidos.
30. La telemetria explica support, independencia y ambiguedad de cada `HOLD`.
31. Un submapa nunca anclado conserva first anchor loop sin limite espacial,
    mientras un epoch perdido desde estado anclado respeta su envolvente raw.
32. Todas las aristas de fusion server conectadas entran transitivamente sin
    bonus artificial ni truncado por el limite sparse ORB.
33. Un loop que rompe temporal, covisibilidad fuerte, fusion previa, hard o
    corredor se rechaza con cero escrituras aunque sus `CurrentLoop` converjan.
34. Una componente completamente soft puede moverse rigidamente si conserva
    sus relaciones y no contradice autoridades.
35. Los corredores hard-hard usan la ultima referencia fiducial, limitan
    provisionalmente interiores a 5 m/20 grados y no derivan por accepts loop;
    un exceso preexistente no puede aumentar en traslacion ni rotacion.
36. Todo KF movido/propagado por commit loop o fiducial recibe reevaluacion
    BAJA; una fusion previa omite solo esa pareja, no candidatos nuevos.
37. La reevaluacion post-opt usa `FusionRefresh`: conserva fusion y score, pero
    no inicia otra optimizacion. Un `Full` normal prevalece al coalescer.
38. Una hipotesis entre dos regiones estables que contradice mas de `5 m / 20
    grados` su relacion protegida se rechaza antes del builder; no hay dry run.
39. El precheck no bloquea el caso protegido-no fiable: ese lado puede absorber
    la correccion mediante el grafo normal.
40. Un rechazo regional equivalente evita solves repetidos de KFs vecinos y se
    invalida por revisiones/autoridad, sin blacklist permanente por submapa.
41. `FusionRefresh` solo busca fusiones espacialmente plausibles, se agrupa por
    region y no mantiene el mission gate cuando es el unico backlog.

Debe existir al menos un `ACCEPT` loop reproducible y evidencia positiva de las
topologias hard/soft principales. Una prueba Gazebo sin error alto se documenta
como prueba sin ejercicio de 3Q y puede requerir repeticion indicada por el
usuario.

## `PARCIAL`

La conclusion es `PARCIAL` cuando la arquitectura y regresiones son correctas,
pero ocurre cualquiera de estos casos:

- no aparece un accept loop real suficiente en Gazebo;
- falta una de las topologias principales o su replay;
- alguna revision visual acordada no se realizo;
- la fusion posterior no se ejercio aunque la optimizacion si;
- los residuales de fusiones previas indican degradacion relevante aun sin
  romper invariantes;
- rendimiento o backlog requieren mas observacion, aunque drenen;
- la ruta fiducial covisible no sustituye todavia todos los reanchors soft.
- falta ejercicio real de continuidad tras perdida, corredor o reevaluacion
  post-opt aunque sus regresiones unitarias pasen.

Cada prueba conserva su conclusion propia. Una repeticion crea otra entrada y
no borra el intento anterior.

## `NO CONSEGUIDA`

Es `NO CONSEGUIDA` si aparece cualquiera de estos fallos:

- un hard fiducial se mueve;
- se usa GT funcionalmente;
- un loop relativo se transforma en target world absoluto persistente;
- raw se modifica por optimizacion;
- existe estado multi-base visible a medias;
- reject/stale/fallo numerico deja poses o fusion aplicadas;
- una fusion omitida revierte injustificadamente una pose valida;
- el tail atraviesa un hard o usa autoridad anterior;
- el primer fiducial de hijo soft corta dependencia antes de accept;
- la covisibilidad expande sin control todo el mapa o rompe continuidad;
- una region fusionable vuelve a ocultar error alto apoyado;
- el builder usa solo una region cuando existen varias coherentes o encadena
  rebuilds sin limite;
- dos workers secundarios se solapan o una MAX interrumpe la activa;
- `stop_drones` no se activa, se libera antes del task end o queda atascado;
- el secundario publica/espera ACK o bloquea el principal;
- retry infinito, backlog sin politica, deadlock, crash, NaN o corrupcion.
- un epoch perdido se ancla a una zona fisicamente inalcanzable;
- un loop converge localmente pero rompe una fusion/covisibilidad/temporal
  previa o desplaza de forma absurda un corredor hard-hard;
- se cancela toda la busqueda de un KF solo porque una de sus parejas ya estaba
  fusionada.

## Parametros provisionales

Se mantienen como inicio, no como verdad definitiva:

- ratios/umbrales geometricos 3O/3P ya validados;
- dos queries y margen de ambiguedad actual;
- controles base `0.30` y vecindades fiduciales de 3I;
- umbral de fusion de 3P como objetivo final del loop;
- pesos nuevos por edge y robust kernel configurables;
- fusiones anteriores soft, sin hardening automatico.
- corredor hard-hard: maximo interior inicial `5 m` y `20 grados`, decreciente
  hacia los extremos y configurable;
- margenes de continuidad tras perdida configurables y derivados del recorrido
  raw, no de GT ni de una distancia fija por KF.

Las pruebas pueden justificar cambios. No se ajustan con GT y cualquier cambio
funcional material se conversa antes de aplicarlo.

## Documentacion de cierre

Tras implementar y probar, actualizar:

- docs de cada componente modificado en `codex/contexto/paquetes/`;
- historial largo/resumen 3Q e indice;
- estado actual, pipeline de Fase 3 y ultima sesion;
- contratos 3H-3L, 3O y 3P si el runtime sustituto cambia sus fronteras;
- memoria operativa con builds/pruebas/conclusion y trabajo pendiente.

No marcar 3Q realizada solo por build o tests sinteticos. RViz2 y el grafo web
son evidencia obligatoria en los casos representativos acordados.
