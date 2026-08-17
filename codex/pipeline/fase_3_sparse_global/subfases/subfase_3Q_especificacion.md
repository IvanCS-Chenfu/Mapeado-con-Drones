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
misma `map_epoch`. Una geometria aceptada decide solo por error:

```text
error bajo -> fusion 3P
error alto -> candidato 3Q
```

Se conservan guardas causales:

- no usar vecinos temporales inmediatos como cierre;
- una query antigua no toma un candidato futuro del mismo submapa;
- dos queries independientes y coherentes son obligatorias;
- grupos ambiguos de zonas repetitivas terminan en `HOLD`;
- un grupo dominante puede aportar varias constraints compatibles al grafo.

La primera query conserva solo una hipotesis compacta. La tarea que obtiene el
segundo apoyo mantiene en memoria sus inliers y los usa en 3Q/3P.

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

La fusion anterior es inicialmente una constraint relativa blanda, no un
punto world fijo. Se medira su error antes/despues. Si las pruebas demuestran
que una fusion fiable se degrada demasiado, una revision posterior podra
endurecer su relacion relativa, nunca convertir ambos KFs en absolutos.

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

### Covisibilidad

La topologia temporal/anchor/loop determina primero los endpoints obligatorios.
Las aristas fuertes de covisibilidad pueden definir caminos minimos e incorporar
submapas necesarios, pero no se recorre ilimitadamente toda la componente.

Solo se admiten:

- covisibilidad ORB nativa con soporte suficiente;
- aristas `ServerLoopGeometric` aceptadas y vigentes;
- loops/fusiones con medida y revisiones validas.

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
- deja el loop dentro del umbral de fusion o el fiducial en su umbral;
- mantiene todos los hard inmoviles;
- conserva finitud y continuidad;
- no viola revisiones ni epochs.

La degradacion de fusiones previas se registra, pero en esta primera version no
rechaza automaticamente ni las vuelve hard.

## KFs tardios y futuros

El flujo principal sigue insertando datos durante el solver. En commit:

- KFs tardios dentro de intervalos se interpolan si sus revisiones son
  compatibles;
- el tail abierto de cada submapa se reancla respecto a su ultimo control
  optimizado;
- un hard posterior detiene la propagacion;
- hijos soft dependientes de un KF movido conservan coherencia;
- KFs futuros usan el `ContinuationRecord` aceptado.

## Fusion posterior

Tras `ACCEPT`, la misma `LoopTask` reutiliza sus inliers y prepara 3P sobre las
poses candidatas. No repite BoW ni RANSAC. Si raw/geometry cambio, el intento es
`STALE` y una tarea fresca recalcula todo.

`FUSION_SKIPPED` no invalida poses correctas. Si hay fusion, tracks,
covisibilidad y score forman parte del commit coordinado y de sus dirty sets.

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
