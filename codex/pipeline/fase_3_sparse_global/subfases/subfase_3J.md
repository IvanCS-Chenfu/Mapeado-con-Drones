# Subfase 3J - Optimizacion privada del grafo fiducial

## Estado

```text
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: PENDIENTE PARA INTEGRACION POST-LOOP
Ejecucion: SOLVER BASE VALIDADO; BYPASS POST-LOOP PENDIENTE DE INTEGRAR
Dudas abiertas: ninguna
```

Este documento define la implementacion vigente de 3J dentro del flujo conjunto
3H-3L. El solver final validado de `legacy2` es la referencia algoritmica. Las
variantes experimentales o descartadas del legado no deben reaparecer.

## Sucesion acordada en 3Q

3Q reutilizara `OptimizationManager` y su ejecucion privada para un problema
SE(3) comun. Fiducial conserva prior absoluto; loop añade constraint relativa y
permite mover ambos lados segun autoridad. Aristas temporales, covisibles y
fusiones previas participan con pesos configurables/robustos. No se crea un
solver loop separado.

## Objetivo

Resolver el `PoseGraphProblem` inmutable de 3I y devolver una
`OptimizationProposal` privada. 3J no modifica ninguna base de datos, no
publica y no decide por si sola el commit.

```text
PoseGraphBuilder -> OptimizationManager -> OptimizationProposal
```

La optimizacion debe llevar el KF fiducial target a su observacion absoluta sin
mover el control hard anterior y conservando de forma razonable la geometria
relativa temporal de la ventana.

## Referencia `legacy2`

Se recuperaran selectivamente los parametros y el comportamiento que dieron
buen resultado:

- optimizacion SE(3), no una correccion solo de yaw o traslacion;
- primer control fijo;
- target con restriccion absoluta fuerte;
- aristas temporales con transformaciones relativas completas;
- vecindades rigidas inducidas alrededor de los extremos;
- pesos, tolerancias, iteraciones y estrategia final que quedaron validados;
- comprobaciones de convergencia y finitud.

Antes de portar una constante se comprobara que pertenece a la variante final
validada y no a un experimento intermedio. Los valores se expondran como
configuracion estable solo cuando tenga sentido operativo; no se usara GT para
autoajustarlos.

## Entrada

`OptimizationManager` recibe exclusivamente el problema privado de 3I:

```text
task_id
control vertices and initial world poses
fixed/hard vertices
temporal SE(3) constraints
absolute target constraint
PropagationPlan
captured revisions
```

No relee contenedores live mientras resuelve. La frescura se comprobara antes
del commit en 3L/3K.

## Formulacion

La funcion de coste combina:

1. restricciones temporales entre controles, basadas en el movimiento raw;
2. fijacion exacta del primer control y de cualquier hard permitido;
3. observacion absoluta del target;
4. restricciones inducidas de las vecindades protegidas de los extremos.

El target absoluto es una observacion fiducial, no un loop. La verdad terreno
simulada se limita a construir `target_world_T_kf`; no se compara el resto de la
trayectoria con GT durante el solver ni durante la aceptacion.

El solver opera sobre poses locales privadas y conserva rotacion SO(3) valida.
Debe normalizar cuaterniones cuando corresponda y rechazar valores no finitos.

## Extremos y vecindades

- el control inicial nunca se mueve;
- el target puede moverse hasta la pose absoluta observada;
- la vecindad inicial permanece rigidamente ligada al control inicial;
- la vecindad final acompaña de forma coherente al target;
- la deformacion se reparte entre ambas zonas usando las aristas temporales;
- ningun hard fiducial previamente aceptado puede desplazarse.

La optimizacion solo contiene el submapa de la tarea en esta fase. Los tipos
pueden ser compatibles con una futura ventana multi-submapa, pero 3J no debe
adelantar covisibilidad.

Por ello, el primer fiducial directo de un submapa anclado solo por loop no se
resuelve con este solver: se aplica el reanchor rigido absoluto acordado en
3H/3O. Los fiduciales posteriores si usan 3J. La futura variante covisible
sustituira ese reanchor transitorio por una optimizacion conjunta.

## Resultado privado

`OptimizationProposal` incluye:

```text
task_id
solver_status
optimized_control_poses
initial/final objective
iteration_count
target translation/rotation/yaw residuals
hard-vertex deltas
finite/geometry diagnostics
PropagationPlan reference or owned copy
consumed revisions
```

En este contrato, residual significa el error que permanece entre la pose
propuesta del target y `target_world_T_kf`, separado en traslacion, rotacion
completa y yaw. No es una medida contra el GT de toda la trayectoria.

Estados minimos:

| Estado | Significado |
|---|---|
| `CONVERGED` | el solver convergio y la propuesta es finita |
| `MAX_ITERATIONS` | produjo candidato finito sin convergencia plena |
| `NUMERICAL_FAILURE` | NaN, Inf, matriz invalida o fallo del solver |
| `INVALID_PROBLEM` | grafo o restricciones incoherentes |
| `CANCELLED_SHUTDOWN` | cierre ordenado antes del commit |

`CONVERGED` no equivale automaticamente a commit. 3L valida invariantes,
cobertura, revisiones y error final. Un candidato finito parcial puede ser
aceptable para una pasada intermedia; un fallo numerico nunca lo es.

## Ejecucion y concurrencia

- el `SecondaryWorker` conserva la propiedad exclusiva de la tarea activa;
- el solver se ejecuta fuera de locks de bases live;
- el flujo principal puede añadir KFs y publicar mientras se calcula;
- ninguna tarea secundaria empieza hasta que termina todo el flujo activo;
- la parada de drones permanece activa durante la optimizacion;
- no hay publication worker ni `publication_ack`.

Los KFs que llegan mientras se resuelve no se incorporan al problema privado ya
capturado. 3K los detectara y propagara de forma coherente durante el commit.

## Cambios a realizar

- reimplementar o adaptar `OptimizationManager` para consumir
  `PoseGraphProblem` inmutable;
- portar la configuracion final validada de `legacy2` y documentar el origen de
  sus parametros;
- preservar restricciones SE(3), hard vertices y vecindades de extremos;
- devolver `OptimizationProposal` sin efectos laterales;
- añadir estados explicitos, diagnosticos finitos y metricas de convergencia;
- mantener el mismo `task_id` y las revisiones de 3H-3I;
- retirar del contrato activo cualquier dependencia de covisibilidad,
  publicacion, HTML o mutacion directa del store.

Los dumps TSV/HTML pueden existir como diagnostico opcional acotado, desactivado
por defecto. Nunca participan en el commit ni son necesarios para que el runtime
avance.

## Invariantes

- `RawMapDatabase` y `GlobalPoseStore` permanecen intactos durante 3J;
- el primer control y todos los hard tienen delta numericamente cero;
- todas las poses propuestas son SE(3) finitas;
- no se usan locks live durante el solver;
- la propuesta corresponde exactamente a las revisiones capturadas;
- el resultado es determinista dentro de las tolerancias numericas acordadas;
- un error grande no se oculta descartando silenciosamente la tarea.

## Contrato visual

Debe mostrarse el flujo real:

```text
PoseGraphBuilder --graph_ready--> OptimizationManager
OptimizationManager --proposal_ready/status--> Validation
```

Los eventos incluyen `task_id`, controles, aristas, iteraciones, estado,
errores inicial/final y duracion. No transportan poses completas ni datos del
solver.

## Pruebas

- caso identidad y error ya casi nulo;
- traslacion pura, yaw puro, rotacion 3D y correccion combinada;
- trayectoria con movimiento vertical y giros fuertes;
- ventana minima y ventana larga;
- control inicial/hard exactamente inmovil;
- target llevado al objetivo con el comportamiento esperado de `legacy2`;
- propuesta finita y determinista;
- `MAX_ITERATIONS` finito distinguible de fallo duro;
- inyeccion de NaN, problema degenerado y restriccion incoherente;
- el store no cambia antes de 3K;
- el flujo principal progresa durante un solver artificialmente lento;
- replay reproduce estados y resultados dentro de tolerancia.

## Criterios de cierre

3J queda conseguida cuando el solver validado produce propuestas privadas y
finitas para los casos acordados, preserva hard/anchors, no bloquea el flujo
principal y entrega a 3L informacion suficiente para aceptar, refinar o declarar
un fallo duro de forma explicita.

## Fuera de alcance

- covisibilidad, loops y ventanas multi-submapa;
- interpolacion definitiva de todos los KFs;
- commit o rollback visible;
- actualizacion de `GlobalMapBuilder`;
- uso de GT fuera de la observacion fiducial simulada;
- aceptacion basada en el aspecto de RViz2 o del grafo web.
