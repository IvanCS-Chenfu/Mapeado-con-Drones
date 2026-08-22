# Subfase 3Q - Especificacion funcional

## Objetivo

Implementar un optimizador covisible comun para dos constraints principales:

```text
FIDUCIAL_ABSOLUTE: target world conocido por observacion fiducial
LOOP_RELATIVE:     relacion query-candidate medida por 3O/RANSAC
```

La ruta fiducial sigue siendo MAX y la ruta loop forma parte de la `LoopTask`
BAJA original. No se crea una tarea de optimizacion independiente.

## Precondicion vigente

3P ya entrega `LoopTaskComputation` con regiones, transformacion local,
inliers, residual y revisiones. Para error bajo fusiona. Para error alto hoy
termina como `OptimizationEvidence`; 3Q debe continuar desde esa evidencia.

El grafo fiducial activo 3I-3L es mono-submapa y su solver interpola una
correccion absoluta. 3Q debe generalizarlo sin romper su API ni sus regresiones.

## Grafo persistente de constraints

Se añadira un store ligero de topologia, o representacion equivalente, que no
duplique poses ni datos raw. Las autoridades siguen siendo:

```text
poses/continuidad/hard -> GlobalPoseStore
datos ORB crudos       -> RawMapDatabase
covisibilidad          -> CovisibilityDatabase
tracks/score           -> FusedLandmarkManager / LandmarkScoreManager
```

El grafo registra identidad, procedencia, soporte, medida y revisiones de:

- observaciones fiduciales hard aceptadas;
- controles de continuidad por submapa;
- tramos temporales entre controles/fiduciales;
- dependencias de anchor loop blando;
- loops y fusiones geométricas aceptadas;
- KFs extremos y submapas implicados.

Un nodo fiducial representa una observacion aceptada concreta, no solo el ID
fisico del fiducial. La misma marca puede producir varios nodos en instantes o
submapas distintos.

## Admision de loop

No existe una rama distinta por ser inter/intra dron, inter/intra submapa o
misma `map_epoch`. El conjunto de geometria aceptada decide por error:

```text
error bajo -> fusion 3P
error alto -> candidato 3Q
```

Si una misma tarea contiene regiones de ambos tipos, una region de error alto
con apoyo independiente domina y activa 3Q. La existencia de una region
fusionable no puede hacer retornar fusion para toda la tarea. Solo cuando todas
las regiones robustas son de error bajo se ejecuta fusion directa sin solve.

Se conservan guardas causales:

- no usar vecinos temporales inmediatos como cierre;
- una query antigua no toma un candidato futuro del mismo submapa;
- dos queries independientes y coherentes son obligatorias;
- grupos ambiguos de zonas repetitivas terminan en `HOLD`;
- un grupo dominante puede aportar entre una y tres constraints compatibles y
  geometricamente distintas al grafo;
- las constraints se deduplican por vecindario candidato y deben representar
  una transformacion compatible del mismo par de submapas;
- una region contradictoria o de otra hipotesis no se mezcla aunque tenga
  muchos inliers.

La primera query conserva solo una hipotesis compacta. La tarea que obtiene el
segundo apoyo mantiene en memoria sus inliers y los usa en 3Q/3P. La telemetria
expone support actual/requerido, separacion de query, compatibilidad, apoyo de
la segunda hipotesis y margen de ambiguedad; los umbrales vigentes no cambian
en esta correccion inicial.

### Rechazo previo en regiones protegidas

Una geometria RANSAC localmente valida no debe llegar al builder cuando
contradice de forma extrema dos regiones cuya colocacion global ya esta
respaldada. El precheck se ejecuta despues de RANSAC y antes de construir el
grafo; no realiza dry run ni solver.

Una region es protegida cuando contiene un hard, pertenece a un corredor entre
fiduciales o alcanza esos KFs mediante una expansion temporal/covisible fuerte
acotada. Una region estable puede ademas apoyarse en fusiones/loops server
confirmados. La confianza no se propaga sin limite por toda la componente.

El precheck compone la relacion entre el representante protegido, la query y
la medida RANSAC hacia candidate. Compara esa prediccion con la relacion world
vigente. El margen inicial reutiliza `5 m / 20 grados`, decreciente hacia hard.
Si ambos lados son estables y la incompatibilidad excede el margen, termina en
rechazo temprano con cero escrituras. Si solo un lado es estable, no se rechaza
por distancia: el grafo puede mover asimetricamente el lado no fiable.

Los rechazos se guardan en un ledger regional derivado, separado de
`CovisibilityDatabase`. La clave combina regiones query/candidate, bucket de
transformacion, revisiones geometricas y revision estructural. KFs vecinos con
la misma hipotesis consultan el ledger y terminan antes del builder. Fiduciales,
reanchors y cambios loop/fusion de la componente invalidan su alcance; no es
una lista negra permanente entre submapas completos.

## Seleccion de ventana

La ventana es el subgrafo minimo que conecta los extremos del loop actual con
sus autoridades y constraints anteriores relevantes.

### Fusiones previas

Si ya existe una fusion causal entre los mismos tramos, se incluyen:

```text
A_fusion_anterior ... A_loop_actual
        |                    |
B_fusion_anterior ... B_loop_actual
```

La fusion anterior es una constraint relativa blanda, no un punto world fijo.
Se recorren transitivamente **todas** las aristas `ServerLoopGeometric` de la
componente alcanzada, se incorporan sus KFs y se mide cada residual
antes/despues. No se añade un bonus artificial: la region gana influencia por
sus KFs, aristas y soporte reales. Una componente soft completa puede moverse
rigidamente si conserva esas relaciones y no contradice hard ni corredores.

### Fiduciales y tramos temporales

- un endpoint entre dos hard incorpora el tramo completo entre ambos;
- un endpoint posterior al ultimo hard incorpora hard -> endpoint;
- una dependencia soft se sigue recursivamente hasta autoridad hard;
- si el KF de apoyo soft pertenece a un tramo entre dos fiduciales, entra todo
  ese tramo y tambien el camino del hijo hasta su endpoint;
- varias dependencias soft pueden atravesar varios submapas, pero solo entra el
  camino necesario;
- una frontera hard detiene la propagacion de correcciones posteriores.

Si no existe hard alcanzable se fija una unica raiz de gauge con la mayor
autoridad disponible, sin reinterpretarla como fiducial.

### Covisibilidad y expansion fiducial

La topologia temporal/anchor/loop determina primero los endpoints obligatorios.
Las aristas fuertes de covisibilidad pueden definir caminos internos. Las
fusiones server y dependencias soft se recorren transitivamente hasta cerrar la
componente confirmada; la covisibilidad ORB nativa no expande por si sola hacia
submapas nuevos.

Para expandir entre submapas solo se admiten:

- aristas `ServerLoopGeometric` aceptadas y vigentes;
- loops/fusiones con medida y revisiones validas.
- dependencias de anchor loop blando vigentes.

Una vez incluidos los submapas, la covisibilidad ORB nativa con soporte
suficiente refuerza controles/aristas internos, pero no descubre por si sola
nuevos submapas. Esta regla se aplica tambien a la ruta fiducial: deja de ser
mono-submapa y materializa el subgrafo minimo conectado al tramo fiducial por
esas relaciones confirmadas. Todos los hard alcanzados siguen fijos.

BoW, candidatos no verificados y rechazos no son constraints.

## Vertices y propagacion

Todos los KFs de los intervalos pertenecen conceptualmente a la ventana. El
solver selecciona inicialmente aproximadamente el 30 % como controles y crea
un plan de propagacion para el resto.

Son controles obligatorios:

- fiduciales hard y target fiducial;
- query/candidate de loops activos;
- extremos de fusiones/loops anteriores relevantes;
- controles de dependencias soft;
- KFs incidentes en constraints covisibles que no puedan representarse de
  forma segura mediante propagacion.

Por ello, el 30 % es una densidad base y no un maximo. El muestreo conserva la
cobertura temporal/3D ya validada por 3I.

## Autoridad fiducial de un hijo soft

La politica transitoria 3O se sustituye al activar 3Q:

1. un submapa realmente no anclado conserva el first anchor directo;
2. un submapa ya soft y dentro de umbral promociona el fiducial a hard sin
   mover poses y corta la dependencia soft;
3. si el error es alto, encola MAX, construye una ventana covisible conjunta y
   solo promociona/corta la dependencia tras `ACCEPT`.

La constraint loop que creo el anchor soft no se borra; queda disponible para
el grafo y su linaje.

## Candidato y validacion

El solver trabaja sobre una vista privada y produce:

- poses optimizadas de controles;
- poses propagadas de no controles;
- continuidad propuesta por cada submapa afectado;
- coste global y residual por familia de aristas;
- error del loop/fiducial antes y despues;
- variacion de fusiones anteriores;
- IDs/revisiones consumidos.

Inicialmente solo se acepta un candidato completo que:

- reduce coste y error principal;
- deja todas las constraints `CurrentLoop` incluidas dentro del umbral de
  fusion o el fiducial en su umbral;
- mantiene todos los hard inmoviles;
- conserva finitud y continuidad;
- no viola revisiones ni epochs.

Para loop, la degradacion estructural deja de ser solo telemetria. Se rechaza
con cero escrituras si el candidato satisface las constraints actuales
rompiendo una arista temporal, una covisibilidad fuerte, una fusion/loop previo,
un hard o un corredor hard-hard. La ruta fiducial conserva su autoridad
absoluta obligatoria y no se rechaza por esta guarda especifica de loop.

Si, tras el solve loop, una unica region queda claramente discordante mientras
las demas satisfacen una solucion coherente, se elimina esa region y se permite
un solo rebuild+solve. El segundo resultado se valida completo. No hay mas
rebuilds; una incoherencia restante termina `REJECT` sin escrituras. Con una
sola region inicial no existe esta retirada.

## KFs tardios y futuros

El flujo principal sigue insertando datos durante el solver. En commit:

- KFs tardios dentro de intervalos se interpolan si sus revisiones son
  compatibles;
- el tail abierto de cada submapa se reancla respecto a su ultimo control
  optimizado;
- un hard posterior detiene la propagacion;
- hijos soft dependientes de un KF movido conservan coherencia;
- KFs futuros usan el `ContinuationRecord` aceptado.

## Continuidad tras perdida de tracking

Un submapa que nunca estuvo anclado puede anclarse por loop sin restriccion de
distancia. La guarda solo existe cuando un dron cambia de `map_epoch` y el
submapa anterior estaba anclado.

Se conserva el ultimo KF activo fiable del epoch anterior. Para cada KF del
nuevo epoch se deriva una envolvente individual usando el recorrido y giro raw
acumulados mas margenes configurables. Un anchor loop fuera de esa envolvente
termina diferido/rechazado sin commit ni cache negativa permanente; KFs
posteriores pueden volver a intentarlo. Un fiducial ignora la guarda, reancla
todo el submapa y elimina el estado de perdida.

## Corredores entre fiduciales

Cuando una solucion fiducial deja dos hard consecutivos en un submapa, las
poses aceptadas del tramo forman una referencia persistente. Un loop posterior
puede refinarla, pero provisionalmente ningun KF interior puede separarse mas
de 5 m ni 20 grados de esa referencia; el margen decrece hacia ambos hard y es
cero en los extremos. Los loops aceptados no renuevan la referencia para evitar
deriva acumulativa. Una optimizacion fiducial si puede sustituirla.

La validacion compara el exceso antes y despues. Si la pose vigente ya estaba
fuera de su margen por deriva anterior, la propuesta no se rechaza por ese
estado heredado cuando mantiene o reduce por separado los excesos de traslacion
y rotacion. Se rechaza con cero escrituras cualquier exceso nuevo o creciente.
Esto evita repetir solves que no han causado la desviacion sin debilitar la
proteccion frente a saltos absurdos.

## Fusion posterior

Tras `ACCEPT`, la misma `LoopTask` reutiliza sus inliers y prepara 3P sobre las
poses candidatas. No repite BoW ni RANSAC. Si raw/geometry cambio, el intento es
`STALE` y una tarea fresca recalcula todo.

`FUSION_SKIPPED` no invalida poses correctas. Si hay fusion, tracks,
covisibilidad y score forman parte del commit coordinado y de sus dirty sets.

Ademas, tras commits loop y fiducial, cada KF realmente movido o propagado se
reencola como `LoopTask` BAJA para buscar solapes desde su pose nueva. Una pareja
con fusion server ya vigente se omite, pero la tarea continua con candidatos
nuevos. La fusion directa del loop original sigue reutilizando sus inliers sin
esperar esta reevaluacion.

Los reruns `FusionRefresh` son mantenimiento espacial, no nuevos detectores de
loop global. Agrupan KFs movidos por regiones y solo verifican candidatos cuyas
subnubes world se solapan o quedan dentro de un margen configurable. No hacen
fallback a BoW lejano y nunca optimizan. Las tareas `Full` nacidas de
delta/snapshot conservan busqueda global y prevalecen al coalescer.

Un backlog compuesto solo por `FusionRefresh` no mantiene `stop_drones`; las
optimizaciones activas y el trabajo critico conservan el gate. Los refresh
siguen en la misma cola BAJA y se procesan hasta completar sus fusiones/scores.

## Concurrencia y stop de drones

Al entrar realmente en la rama de optimizacion loop:

```text
loop_optimization_active=true -> stop_drones=true
```

El flag permanece durante grafo, solver, validacion, prepare/commit y fusion
directa posterior. Se libera al terminar la tarea, incluso en reject/stale o
rollback. No se activa por un loop de error bajo que solo fusiona.

La tarea sigue siendo BAJA y no se interrumpe. Una MAX pendiente comienza
despues. El flujo principal continua; solo el mission gate retiene nuevos
goals.

## Prohibiciones

- no GT para loop, pesos, solver o aceptacion;
- no segunda cola, worker, builder, solver, validator o pose store;
- no escritura provisional visible;
- no snapshot completo cuando basta una ventana;
- no modificar `RawMapDatabase`;
- no publicar ni ejecutar `GlobalMapBuilder` desde secundario;
- no offset artificial para forzar accepts Gazebo;
- no ACK visual como condicion de task end.
