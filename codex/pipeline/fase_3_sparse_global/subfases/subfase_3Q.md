# Subfase 3Q - Optimizacion covisible comun por loop y fiducial

## Estado

```text
IMPLEMENTADA; VALIDACION AUTOMATICA CORRECTA; A REVISAR
```

3Q se reabre tras la prueba 213 para corregir dos fallos relacionados:

- loops geometricamente validos que no corrigen una deriva porque la propuesta
  se cancela por tolerancias numericas o por una ventana inadecuada;
- loops ambiguos en zonas repetidas que reciben demasiado alcance y deforman
  centenares de KFs, como ocurrio en la prueba 194.

La prueba 220 valida el comportamiento general y la mejora conservadora de
cascada/recuperacion, pero mantiene un punto residual: una constraint con error
world alto aislado entre candidatos consecutivos ya alineados puede entrar en
`CurrentLoop` y ampliar demasiado el solve. El detalle y la evidencia viven en
el historial. El usuario acepta continuar sin otra correccion inmediata y deja
3Q `A REVISAR`: solo se reabrira si el fallo vuelve a aparecer.

El punto de reentrada no es cambiar la ventana ni dejar de optimizar las tres
relaciones. Las tres regiones compatibles ya entran como `CurrentLoop` y deben
seguir intentando error cero sobre la ventana completa. El fallo pendiente es
de enforcement: `LoopRelative` puede agotar 160 iteraciones y declararse
`Converged`; el validator acepta cada loop bajo `0.25 m/0.15 rad`, exige mejora
solo mediante OR y permite degradacion estructural local amplia. Si reaparece,
se revisaran convergencia/no degradacion por loop y estructura local sin tocar
cascada, recuperacion 1/1 ni fiduciales.

Este documento sustituye el contrato anterior de 3Q. Al implementar no se deben
conservar ramas, parametros, comentarios ni tests que expresen reglas obsoletas.
La implementacion final tendra una unica ruta comun para seleccionar la
ventana, construir el grafo, optimizar, validar y hacer commit, compartida por
optimizaciones de loop y fiducial.

Antes de ejecutar se deben consultar
[`historial_3Q_RESUMEN.md`](../historial/por_subfase/historial_3Q_RESUMEN.md) y
los fragmentos de las pruebas 194 y 213 en
[`historial_3Q.md`](../historial/por_subfase/historial_3Q.md). El historial
conserva los intentos anteriores; este MD define solo el comportamiento vigente.

## Objetivo funcional

Cuando RANSAC encuentra una correlacion geometrica valida entre el KF `query` y
el KF `candidate`, hay dos resultados posibles:

1. Si la pose relativa world ya concuerda suficientemente con la transformacion
   RANSAC, se puede realizar fusion directa.
2. Si RANSAC es bueno pero existe discrepancia de distancia o angulo entre las
   nubes, se construye un grafo local y se optimiza antes de decidir la fusion.

El objetivo matematico del solver es llevar a cero el error de pose de la
constraint `CurrentLoop` respecto a la transformacion RANSAC. Esto no significa
poner ambas camaras en la misma pose ni exigir residual cero a cada pareja de
puntos. Significa que, tras aplicar las poses optimizadas, la transformacion
relativa entre ambas nubes debe coincidir con la estimada por RANSAC.

La correccion se reparte entre los KFs movibles de query, candidate y segmentos
covisibles segun todas las aristas y su autoridad. No se fuerza que una nube se
mueva completamente hacia la otra ni que ambas queden en la media. Los
fiduciales hard son inmoviles permanentemente; los segmentos que alcanzan el
consenso 3/60 se fijan solo durante ese solve para que se mueva el lado query.

## Flujo unico de una tarea

```text
RANSAC valido
  -> clasificar fusion directa u optimizacion
  -> comprobar apoyo temporal y ambiguedad
  -> seleccionar segmentos temporales
  -> expandir por covisibilidad server confirmada
  -> construir grafo SE(3) comun
  -> optimizar CurrentLoop hacia cero
  -> validar hard, estructura, loops previos y KFs ya optimizados
  -> commit atomico de poses si procede
  -> fusionar landmarks solo si cumple el umbral de fusion
  -> conservar PriorLoop cuando haya correccion segura sin fusion
```

Las optimizaciones por loop y por fiducial reutilizan la misma seleccion de
segmentos, expansion covisible, builder, solver, validator y commit. La ruta
fiducial no debe fabricar un loop sintetico para obtener una ventana.

## Segmentos temporales de la ventana

`Anterior` y `siguiente` siempre siguen el orden temporal de creacion de los KFs
dentro de su submapa, no la distancia world ni el ID global.

Para cada extremo `query` y `candidate`:

- incluir desde el KF extremo hasta el fiducial hard anterior de su submapa;
- si esta en un tramo cerrado por un hard anterior y otro posterior, incluir
  tambien hasta el hard siguiente;
- si no existe hard posterior, no inventar un cierre ni incorporar por defecto
  el resto completo del submapa;
- combinar varios intervalos del mismo submapa en un unico batch atomico, sin
  convertirlos en una ventana continua artificial;
- mantener obligatorios los extremos de constraints y los controles hard/fixed.

La seleccion es simetrica para query y candidate. Se elimina el cierre
incondicional por submapa completo de los casos problematicos. Un segmento
hard-hard real puede seguir siendo largo y la expansion incidente puede sumar
varios segmentos; la segmentacion por si sola no constituye un limite de
numero de KFs.

## Expansion por covisibilidad

La ventana se amplia solo mediante relaciones confirmadas por el servidor:

- aristas `ServerLoopGeometric` aceptadas;
- fusiones previas representadas como constraints relativas blandas;
- dependencias soft confirmadas incidentes en un segmento ya seleccionado.

Cada arista solo permite incorporar el intervalo temporal de su otro extremo,
delimitado por sus hard anterior y posterior con las mismas reglas. No autoriza
incorporar todo el submapa ni seguir covisibilidad fuera de ese entorno.

La expansion puede alcanzar un tercer submapa o mas, pero cada salto requiere
una arista server confirmada incidente en la zona seleccionada. La covisibilidad
ORB nativa puede reforzar submapas ya admitidos, pero no abre uno nuevo.

Esta expansion se aplica igual a optimizaciones iniciadas por loop y fiducial,
porque una correccion fiducial tambien afecta zonas fusionadas previamente.

## Consenso y limite de escalabilidad

Cuando al menos tres segmentos independientes comparten fusiones confirmadas
en la mayoria de sus KFs comparables, forman un scaffold de consenso:

- requiere al menos tres submapas/segmentos independientes;
- la cobertura alcanza al menos el 60 % de los KFs comparables y esta
  distribuida por el intervalo, no concentrada en un unico KF;
- repeticiones de la misma pareja o instante no son fuentes independientes;
- el scaffold entra mediante `PriorLoop` y aristas de autoridad alta;
- todos los KFs movibles de esos segmentos se marcan `fixed` durante el solve;
- esa fijacion es privada y temporal: no los convierte en fiduciales hard ni
  persiste tras terminar la propuesta;
- el lado query que llega contra ese scaffold es el que absorbe la correccion.

Asi una estructura respaldada por tres recorridos no se desplaza para acomodar
una observacion nueva. La condensacion tambien evita crecimiento transitivo
ilimitado de la ventana.

## Admision de loops en zonas repetidas

Una region RANSAC aislada no basta cuando el riesgo de falsa asociacion es alto.
El pipeline conserva endpoints canonicos de query y candidate y comprueba que
los apoyos progresan coherentemente en ambos lados.

El apoyo minimo es configurable y usa tres senales simples:

- `2` apoyos independientes sin senales de riesgo;
- `4` apoyos con una senal de riesgo;
- `6` apoyos con dos o mas senales de riesgo.

Las senales generales son:

- asimetria de anclaje;
- ambiguedad por candidatos geometricos competidores;
- correccion grande, inicialmente superior a `1.0 m` o `0.20 rad`.

Los apoyos proceden de KFs sucesivos o cercanos temporalmente, con progresion
compatible de query y candidate. La perdida reciente no aumenta por si sola el
apoyo: aplica la politica de continuidad especifica descrita a continuacion.

La distancia world no aumenta por si sola el apoyo general exigido, pues esa pose puede
contener la deriva que se intenta corregir. Solo participa en la senal de
correccion grande junto con ambiguedad, continuidad y estado de anclaje.

Los KFs reencolados como `FusionRefresh` pueden actualizar BoW, geometria,
scores o fusiones, pero no iniciar otra optimizacion por evidencia de error
alto. Esto no bloquea loops normales originados por nuevos deltas o snapshots.

## Recuperacion inmediata tras perdida

Cuando nace el `map_epoch` inmediatamente posterior a una perdida y el submapa
anterior del mismo dron estaba anclado, el ultimo KF fiable aporta un prior de
continuidad independiente de la geometria RANSAC. Un unico loop puede crear un
anchor soft local solo si se cumplen simultaneamente estas condiciones:

- la opcion esta habilitada por YAML;
- query pertenece a ese epoch reciente, sigue sin anchor y candidate pertenece
  a una componente con autoridad world;
- el recorrido local desde el inicio del epoch no supera `2.0 m`;
- la pose world propuesta para query queda a menos de `0.50 m` y `0.15 rad` de
  la ultima pose fiable anterior a la perdida;
- RANSAC pasa todas las guardas geometricas normales;
- no hay otra region candidata aceptada ni hipotesis competidora;
- no existe correccion grande ni otra senal de riesgo.

Los limites `enabled`, traslacion, rotacion y recorrido maximo son parametros
YAML independientes. Superar `2.0 m` no prohibe anclar: desactiva exclusivamente
la via de un loop y vuelve al apoyo adaptativo normal `2/4/6`.

La constraint nacida con un unico loop se marca provisional. Puede colocar el
submapa recuperado en world y conservar una dependencia soft, pero hasta recibir
un segundo loop independiente o un fiducial no puede:

- propagar autoridad a otros submapas no anclados;
- formar parte de un scaffold de consenso;
- habilitar fusion de landmarks.

Una segunda observacion query-candidate independiente y coherente promociona la
constraint y recupera el comportamiento normal. Un fiducial propio sustituye el
anchor soft y lo convierte en autoridad hard.

## Cascada al aparecer autoridad world

Las constraints relativas aceptadas se conservan aunque ninguno de sus extremos
tenga world. Cada commit que introduzca nueva autoridad world, incluido un primer
fiducial o la sustitucion de un anchor loop por fiducial, debe reevaluar de forma
inmediata la componente conectada:

- recorrer solo constraints confirmadas, nunca provisionales;
- calcular anchors derivados para los submapas conectados aun sin world;
- validar continuidad reciente de cada entrada;
- aplicar todos los anchors derivados en un unico batch atomico;
- reencolar los KFs de la componente afectada para reconciliacion mediante el
  pipeline normal, sin crear otro solver ni otra topologia paralela.

Si la componente ya contiene varias autoridades world incompatibles no se mueve
ningun hard ni se sobrescribe un anchor. La cascada no compromete nada y la
reconciliacion segmentada existente debe resolver o rechazar el conflicto con las
guardas actuales.

## Aristas, autoridad y solver

El grafo SE(3) comun contiene:

- aristas temporales entre KFs consecutivos incluidos;
- covisibilidad y fusiones server confirmadas dentro de los intervalos;
- observaciones fiduciales hard;
- `CurrentLoop` para la hipotesis que dispara la optimizacion;
- `PriorLoop` para loops aceptados y correcciones seguras sin fusion;
- aristas del scaffold de consenso cuando corresponda.

`information_weight` afecta tanto al coste como a la relajacion efectiva de
cada familia. Los pesos se normalizan por familia para que muchas aristas
temporales no anulen una constraint fuerte por simple cantidad y para que
autoridad y soporte gobiernen realmente el reparto de movimiento.

El solver busca error cero de `CurrentLoop`, con objetivo practico de
convergencia independiente de `0.05 m / 0.03 rad`. Este target controla la
parada; no clasifica fusion ni aceptacion. Se elimina todo acoplamiento entre
convergencia y umbrales de commit/fusion.

## Fiduciales, KFs optimizados y corredor protegido

Los fiduciales hard no tienen tolerancia: su pose world no puede cambiar.

Los KFs intermedios de un corredor hard-hard no son fijos ni tienen un limite
absoluto de desplazamiento de `2 cm`. Pueden moverse cuanto requiera el solve,
siempre que los hard permanezcan exactos y no se degraden materialmente las
aristas temporales, de covisibilidad o `PriorLoop`.

El corredor sigue siendo una zona protegida para la admision: una asociacion
con esa zona necesita el apoyo adaptativo correspondiente. No se usa como una
segunda clase de hard dentro del validator.

Cada commit aceptado conserva para sus KFs movidos la pose optimizada que pasa
a ser su referencia protegida. Una optimizacion posterior puede reajustarlos,
pero se rechaza atomicamente si propone para cualquiera de esos KFs un cambio
superior a `5 m` o `20 grados` respecto a esa referencia. Este limite protege
frente a la reencolacion en una copia visualmente repetida del entorno; no se
aplica a KFs nunca optimizados y no fija ajustes menores.

## Histeresis de convergencia, commit y fusion

Los umbrales dejan de compartir parametros y significado:

| Decision | Traslacion | Rotacion | Efecto |
|---|---:|---:|---|
| Convergencia practica | `0.05 m` | `0.03 rad` | Target del solver |
| Fusion directa o post-opt | `0.20 m` | `0.12 rad` | Commit y fusion de landmarks |
| Correccion segura maxima | `0.25 m` | `0.15 rad` | Commit sin fusion |

La decision final es:

- hasta `0.20 m / 0.12 rad`: commit atomico de poses y constraints; fusion de
  landmarks solo si tambien pasan dispersion y evidencia geometrica;
- entre fusion y `0.25 m / 0.15 rad`: commit atomico de la mejora, sin fusionar
  landmarks, y conservar la constraint relativa como `PriorLoop`;
- por encima de `0.25 m / 0.15 rad`: rechazar sin escrituras;
- en cualquier rango: rechazar si se mueve un hard, falta geometria obligatoria
  o se viola materialmente temporal, covisibilidad, fusiones previas, loops
  aceptados o la guarda `5 m / 20 grados` de un KF ya optimizado.

Una fusion posterior omitida no invalida una correccion de poses correcta. Solo
se unifican landmarks cuando la coincidencia final es suficientemente precisa.

## Commit, stale y limpieza

El commit es atomico para todos los intervalos afectados. `RawMapDatabase` no
se modifica. El commit se rebasa sobre poses y snapshot vigentes; un control
auxiliar culled puede omitirse, pero los extremos de constraints y controles
hard/fixed son obligatorios. Deben quedar al menos dos controles activos por
submapa. Si falta alguno, la tarea termina stale sin escrituras y encola una
tarea BAJA fresca deduplicada cuando proceda.

No se permite mantener en paralelo la seleccion antigua por submapa completo,
el loop sintetico fiducial, los umbrales compartidos, la relajacion que ignora
`information_weight` ni la deadband absoluta de `2 cm` sobre KFs intermedios.
Los parametros reemplazados se retiran o migran de forma explicita; no se dejan
aliases silenciosos ni ramas muertas.

## Alcance previsto

La implementacion se concentra en `orbslam3_multi`, `orbslam3_server` y la
configuracion/contratos de `simulacion_dron`. No se modifican ORB-SLAM3,
`orbslam3_msgs`, la semantica raw ni la prioridad/no-preemption de tareas. El
worker secundario no publica.

Componentes previstos:

- `LoopPipeline`: evidencia, secuencias coherentes y apoyo adaptativo;
- `PoseGraphBuilder`/problema: intervalos, expansion y familias de aristas;
- solver: pesos efectivos y convergencia independiente;
- `OptimizationValidator`: hard, estructura, revisitados y umbrales separados;
- `OptimizationManager`/`GlobalPoseStore`: commit multi-intervalo y rollback;
- `SparseGlobalBackend`: seleccion comun para loop y fiducial;
- parametros YAML y telemetria que expliquen HOLD, solve, accept sin fusion,
  fusion y reject.

## Pruebas obligatorias

1. Regresion 194: los loops asimetricos y ambiguos quedan en HOLD con apoyo
   insuficiente; no mueven 359/362 KFs y la ventana permanece acotada.
2. Regresion 213: los KFs intermedios pueden moverse mas de `2 cm`; los hard no
   cambian y no se degradan loops previos.
3. Segmentacion simetrica loop/fiducial, varios intervalos del mismo submapa y
   expansion a un tercero.
4. Consenso de tres segmentos: fija temporalmente sus KFs durante el solve sin
   convertirlos en hard persistentes; se mueve el lado query.
5. Apoyo adaptativo `2/4/6`, progresion query-candidate, competidores y
   `FusionRefresh` sin reoptimizacion fuera de la recuperacion inmediata.
6. Recuperacion reciente cercana con un loop, fallback `2/4/6` al superar sus
   limites y rechazo de candidatos ambiguos o lejanos.
7. Constraint provisional sin fusion, scaffold ni propagacion; promocion tras
   segundo loop independiente o sustitucion por fiducial.
8. Constraint activada sin world seguida de fiducial: cascada atomica inmediata
   y reencolado de la componente sin esperar un tercer loop.
9. `information_weight` cambia la relajacion y el target de convergencia es
   independiente de umbrales de fusion/commit.
10. Los tres resultados: fusion, commit con `PriorLoop` sin fusion y rechazo.
11. KFs previamente optimizados aceptan reajustes menores y rechazan una nueva
   propuesta que supere `5 m` o `20 grados` respecto a su referencia.
12. Builds y suites de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`.
13. Trayectoria tipica equivalente a 219 con Gazebo, RViz2 y logs F3Q reducidos;
   la calidad visual final la revisa el usuario.

## Criterios de cierre

3Q queda `CONSEGUIDA` solo si:

- los hard permanecen exactamente inmoviles;
- loops validos corrigen deriva sin cancelarse por ruido numerico;
- loops ambiguos esperan apoyo independiente suficiente;
- las ventanas quedan delimitadas y no crecen por submapas completos;
- loop y fiducial usan la misma topologia covisible confirmada;
- el solver reduce `CurrentLoop` y los pesos tienen efecto real;
- commit, fusion y rechazo siguen la histeresis y son explicables en logs;
- no quedan rutas o parametros obsoletos compitiendo con el contrato nuevo;
- regresiones, builds, simulacion y revision visual son correctas.

Si compila pero falta evidencia runtime o revision visual, sera `PARCIAL`. Si
reaparece deformacion, se mueve un hard, se fusiona por encima del umbral o una
ventana vuelve a crecer sin limite, sera `NO CONSEGUIDA`.

La validacion funcional se considera parcial mientras la revision visual de la
prueba integrada no este confirmada y los reruns post-opt de una ventana grande
puedan dejar trabajo pendiente al apagar. Esa carga residual no cambia las
reglas del grafo descritas arriba ni autoriza una poda adicional implicita.
