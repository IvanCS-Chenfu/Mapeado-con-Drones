# Subfase 3Q - Plan de pruebas

## Regla de evidencia

No se introducira un offset artificial para provocar error alto. Cada prueba
Gazebo ejecutara su trayectoria natural y el informe distinguira:

```text
loop no detectado
loop geometrico con error bajo / fusion
loop con error alto sin segundo apoyo
optimizacion reject/stale/rollback
optimizacion accept con/sin fusion posterior
```

Si una prueba no activa 3Q, no se presentara como validacion positiva. Se
informara al usuario con toda su evidencia y el usuario decidira si se repite
con otra trayectoria.

## Build

Compilar mediante la herramienta acordada:

```text
orbslam3_multi
orbslam3_server
simulacion_dron
```

No tocar los paquetes ORB prohibidos.

## Tests deterministas

### Topologia y ventana

1. `LoopWindowBetweenTwoHardFiducials` incluye el tramo completo y fija ambos
   hard.
2. `LoopWindowFromHardToOpenTail` incluye hard -> endpoint y crea continuidad.
3. `SoftChildReachesHardBoundedParentSegment` incluye padre, dependencia e
   hijo.
4. `PreviousFusionDefinesSoftCycle` conserva la arista sin fijar poses world.
5. `MinimalConnectedSubgraphExcludesUnrelatedBranches` evita expansion global.
6. `SameSubmapLoopUsesNormalDecision` no termina por identidad.
7. `CovisibilityPromotesMandatoryControls` puede superar el 30 % base.
8. `DenseCovisibilityDoesNotDominateByCount` valida normalizacion de pesos.

### Solver y validacion

1. `LoopRelativeMovesBothSides` reduce error sin target world artificial.
2. `HardFiducialsNeverMove` exige delta numericamente nulo.
3. `FiducialAbsoluteUsesCovisibility` mantiene la regresion fiducial.
4. `PreviousFusionResidualIsMeasured` expone before/after sin rechazo automatico.
5. `AmbiguousHypothesesDoNotOptimize` termina `HOLD`.
6. `TwoIndependentQueriesRequired` activa solo con segundo apoyo.
7. `FullAcceptOnlyForLoop` no compromete candidato parcial.
8. `InvalidOrNonFiniteProposalDoesNotWrite` conserva todas las bases.

### Commit, concurrencia y fusion

1. `AcceptedLoopPosePatchIsAtomic` no expone submapas parcialmente movidos.
2. `LateWindowAndTailFollowAcceptedControls` cubre KFs concurrentes/futuros.
3. `HardBoundaryStopsTail` no propaga a traves de otro fiducial.
4. `RawDatabaseRemainsUnchanged` compara revisiones/contenido raw.
5. `AcceptedOptimizationReusesRansacEvidence` no repite BoW/RANSAC.
6. `FusionSkippedKeepsAcceptedOptimization` conserva poses correctas.
7. `StaleAndRollbackEnqueueFreshLowTask` termina antes del retry.
8. `SecondaryOptimizationDoesNotPublish` solo deja dirty sets.
9. `LoopOptimizationStopFlagLifecycle` activa al entrar en 3Q y libera en
   accept/reject/stale/rollback/excepcion.
10. `LowErrorFusionDoesNotSetOptimizationStopFlag` separa 3P de 3Q.
11. `PriorityRemainsNonPreemptive` observa A loop activa, F MAX pendiente y B
    loop pendiente: A completa, despues F, despues B.

## Replay

Cada live util debe dejar record/replay suficiente para reproducir:

- constraints seleccionadas y revisiones;
- decision de ventana;
- propuesta y validacion;
- commit y fusion posterior;
- orden secundario y stop flag;
- dirty sets y publicacion en el siguiente principal.

El replay no sustituye todos los lives, pero permite repetir accepts, stale y
rollback sin depender de una nueva deriva.

## Matriz Gazebo obligatoria

Crear YAMLs separados y nombres semanticos. La numeracion real de prueba se
asignara al ejecutar.

### G1 - Padre con dos fiduciales, hijo soft

Un dron recorre fiducial 2 -> edificio -> fiducial 1. Otro no ve fiducial, se
ancla por loop al primero y genera un loop posterior. Debe probar ventana del
padre entre hard, dependencia soft, hijo y continuidad.

### G2 - Padre con un fiducial, hijo soft

El padre solo tiene una autoridad hard y un tail abierto. El hijo se ancla por
loop y un cierre posterior debe distribuir error sin una segunda frontera hard.

### G3 - Ambos submapas con un fiducial

Dos drones adquieren autoridad hard independiente y rodean el edificio hasta
crear un loop. Debe mover interiores/tails sin desplazar los dos hard.

### G4 - Un lado entre dos hard y el otro con un hard

Combina un tramo cerrado por dos fiduciales con otro abierto desde un
fiducial. Verifica varios priors absolutos y una constraint relativa.

### G5 - Loop despues de optimizacion fiducial

Primero debe existir un commit fiducial que cambie poses/continuidad. Despues
un loop de error alto debe construir la ventana desde el estado aceptado, no
desde raw ni desde la autoridad anterior.

### G6 - Loop despues de fusion previa

Un primer loop de error bajo fusiona. Un cierre posterior entre los mismos
tramos activa optimizacion. Se mide cuanto cambia la constraint de fusion
anterior y si la nube sigue coherente.

### G7 - Primer fiducial de hijo soft

Un submapa ya colocado por loop observa su primer fiducial:

- dentro de umbral debe promocionar hard sin reanchor;
- fuera de umbral debe ejecutar MAX covisible y cortar la dependencia solo
  tras accept.

Puede requerir dos recorridos naturales diferentes.

### G8 - Llegadas concurrentes

Mantener un goal suficientemente largo para que entren KFs durante solver y
fusion. Verificar late-window, tail, flujo principal vivo y mission gate sin
enviar el siguiente goal.

### G9 - Repeticion de optimizaciones

Provocar cierres sucesivos. Cada tarea reevalua error/revisiones al dequeue y
usa la ultima continuidad, evitando correcciones repetidas obsoletas.

### G10 - Cadena de tres submapas

A hard, B soft respecto a A y C soft respecto a B. Un loop posterior debe
seleccionar solo la cadena necesaria, preservar hard y mantener coherencia de
dependencias dentro/fuera del grafo.

Las trayectorias existentes, incluida
`prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`, se reutilizaran donde
encajen; se crean nuevas solo para topologias no expresables con ella.

## Revision visual

El usuario revisara obligatoriamente RViz2 y el grafo web en cuatro casos:

1. G1: hijo soft y padre entre dos fiduciales;
2. G3/G4: autoridades hard en ambos lados;
3. G5: loop despues de optimizacion fiducial;
4. G6: loop despues de fusion previa.

El resto conserva validacion automatica, logs y metricas. Si aparece una
anomalia visual, su prueba queda abierta aunque los asserts pasen.

El grafo web debe mostrar una iluminacion continua:

```text
LoopDecision -> PoseGraphBuilder -> OptimizationManager -> Validation
-> GlobalPoseStore -> [FusedLandmarkManager] -> task end
```

No debe parpadear por subetapas ni apagar antes de la fusion posterior.

## Metricas por prueba

Informar siempre:

- KFs, submapas y hard incluidos/excluidos;
- vertices base/adicionales y edges temporal/covis/loop/fusion;
- error principal antes/predicted/accepted;
- coste y residuales por familia;
- delta de hard y de fusiones anteriores;
- KFs optimizados, propagados, late y tail por submapa;
- tracks/MPs/scores afectados por fusion;
- tiempo snapshot/builder/solver/validator/fusion/commit/total;
- memoria, PSI, cola, max worker y backpressure;
- tiempo y transiciones de `stop_drones`;
- primary inputs/publicaciones durante solver;
- stale, rollback, retry y pending final;
- si hubo o no un `ACCEPT` real por loop.

## Reduccion de logs

No leer logs completos. Reducir por prueba con patrones equivalentes a:

```text
F3N-|F3O-|F3P-|F3Q-|F3H-|F3I-|F3J-|F3K-|F3L-
SECONDARY|BACKPRESSURE|STOP|F3F-GLOBALMAP|FLOW-EVENT
ERROR|FATAL|SIM-DONE|SIM-EXIT-CODE
```

Crear sublogs para ventana/solver/commit/recursos si el reducido es grande.

## Secuencia de ejecucion

1. tests unitarios y de integracion;
2. build seleccionado;
3. replay determinista positivo/rechazo/stale;
4. Gazebo de menor a mayor topologia;
5. corregir y repetir sin borrar intentos fallidos;
6. revision visual representativa;
7. regresion fiducial 3H-3L y fusion 3P;
8. conclusion agregada solo con cola drenada y estado coherente.
