# Subfase 6H - Seleccion por cobertura de fachada y relevo de subROI

## Estado

`PARCIAL`. La U y sus claims reversibles ya existen; falta validar el relevo
`TO_FINISH` y la progresion sostenida de inspeccion con la politica de STOP.

Este contrato sustituye la seleccion anterior de voxeles con score `0.2..0.6`
y su coverage lineal. El codigo que aun conserve esos conceptos es transitorio
y debe retirarse al migrar esta subfase; no se mezclan ambas politicas.

## Objetivo

Recorrer de forma eficiente la fachada exterior de un subROI. El dron no debe
deambular para convertir cualquier voxel en `FREE`: debe avanzar lateralmente,
mantener una distancia de observacion preferida y capturar depth de pared al
llegar a cada destino de inspeccion.

## Geometria de cobertura

Cada subROI asume inicialmente observacion exterior. Se identifica la cara mas
cercana al centro del ROI global como cara abierta. Las otras tres caras forman
una U de cobertura, situada `facade_coverage_offset_voxels=2` voxeles hacia el
interior del subROI. Sus dos brazos laterales llegan hacia la cara abierta; no
existe tramo que cierre la U por esa cara.

La U no es una ruta, una pared ni una restriccion de navegacion. D* no la usa
como obstaculo, reserva, inflacion o waypoint. Es una geometria de observacion
mostrada en GUI al seleccionar la tarea del dron, junto con el subROI.

Cada segmento de la U representa una seccion volumetrica perpendicular a su
direccion local de avance:

- un tramo con avance en X cubre la banda de todos los Y y Z permitidos;
- un tramo con avance en Y cubre la banda de todos los X y Z permitidos;
- segmentos consecutivos se fusionan entre su primer y ultimo plano, sin
  sumar penalizaciones;
- planos diagonales de 45 grados que nacen en las esquinas inferiores y
  apuntan hacia el interior delimitan las bandas: una seccion de la base no
  invade los brazos y una seccion lateral no invade la base.

La construccion usa los ejes locales reales del subROI, no presupone que todas
las fachadas sigan X/Y globales. Si dos caras empatan en distancia al centro
global, se usa un desempate geometrico determinista.

## Estado de cada seccion

Una seccion pendiente tiene coste de cobertura `0`; una seccion activa tiene
coste `10`, sin acumulacion adicional. El estado es derivado y reversible, no
un booleano historico: `VoxelMapBuilder` mantiene claims por fuente depth y KF
en la misma revision que materializa la evidencia. Una seccion queda activa
mientras tenga algun claim:

- un impacto depth frontal directo `OCCUPIED=1` de `vista_pared` o
  `VIEW_ADVANCE` reclama la seccion espacial de la captura y las
  `depth_coverage_neighbor_sections=2` secciones contiguas a cada lado en la
  U. El vecindario cruza esquinas, pero no salta entre los extremos de la cara
  abierta;
- todo voxel depth `OCCUPIED=1` reclama ademas la rebanada U volumetrica que
  contiene. Asi una sola vista puede completar varias secciones;
- en una esquina sin fachada, una `vista_pared` valida que no encuentra
  impacto antes de 4 m pero aporta `FREE` fiable hasta esa distancia marca
  `SIN_FACHADA`; esa seccion se activa, cuesta `10` y cuenta igual que una
  seccion con pared;
- depth insuficiente, invalido o interrumpido por `TRACKING_RISK` deja la
  seccion pendiente. No se infiere `SIN_FACHADA` por ausencia de datos.

Al cambiar la pose de un KF, invalidarse una fuente o retirarse su impacto,
solo se recalculan sus claims. Una seccion vuelve a pendiente si ya no contiene
evidencia propia ni vecina. El score medio de MapPoints no activa por si solo
la U: el valor exacto `1` que interviene aqui es evidencia depth directa de
`vista_pared` o `VIEW_ADVANCE`.

La procedencia se asocia al `task_id` del subROI, no al propietario momentaneo.
Por ello un relevo `TO_FINISH` no fija el coverage historicamente: el nuevo
dron recibe la tarea con los claims que siguen siendo validos y una fuente
antigua todavia puede retirarlos al cambiar su KF.

## Seleccion de objetivo

`PointSelectionWorker` consume un dron con subROI y sin orden normal activa.
Primero actualiza el lifecycle:

- todas las secciones activas: `COMPLETED` y retorno a `TaskAssignmentQueue`;
- llegada al extremo de la U junto a la cara abierta con secciones pendientes:
  `TO_FINISH`, conserva su estado y libera el dron para una tarea mas cercana;
- en otro caso selecciona una seccion pendiente de menor coste.

En la seccion elegida busca un voxel `OCCUPIED` con score `>0.4` que represente
la fachada. Si no existe, crea una sonda geometrica de la seccion para decidir
de manera medible si se trata de una esquina sin fachada. A partir de ese
objetivo visual evalua poses de inspeccion. Su coste combina, sin convertirlos
en vetos de D*:

- coste de seccion de cobertura (`0` pendiente, `10` activa);
- separacion preferida a la fachada, inicialmente
  `facade_preferred_wall_distance_m=4.0`;
- altura cercana al plano medio del subROI;
- desplazamiento cercano a la distancia minima preferida del dron.

La pose de menor coste es la interseccion ideal de estos criterios; si esta
ocupada, inflada, reservada o no es valida, se prueba la siguiente. El yaw de
la orden mira a la fachada segun la normal local de la U; no mira a una sonda
ni a una coordenada arbitraria. El pitch se conserva por ahora salvo orden
local de seguridad visual.

## Secuencia con UNKNOWN

1. Si pose y corredor estricto son `FREE`, se encola `MOVE_AND_CAPTURE`.
   El dron viaja mirando a la fachada y captura depth de pared al llegar.
2. Si la pose elegida es `UNKNOWN`, se emite `LOOK_AND_CAPTURE` hacia el
   objetivo visual. Esta orden solo busca liberar el corredor: no confirma
   ocupacion depth.
3. Tras materializar los deltas, se reevalua la misma pose, no una distinta.
   Si sigue `UNKNOWN`, se busca una pose `FREE` cercana al objetivo visual con
   los mismos costes. Si existe, se planifica un avance intermedio hasta ella,
   captura `VIEW_ADVANCE` mirando al objetivo visual y al terminal se reelige
   desde la pose alcanzada. Sin pose valida se descarta el candidato y se
   reelige.
4. Si D* encuentra UNKNOWN en el corredor, ejecuta solo el prefijo `FREE` hasta
   el ultimo waypoint anterior, realiza `VIEW_ADVANCE` mirando al objetivo
   visual y reencola una nueva seleccion desde la pose real alcanzada solo tras
   materializar su evidence.

En una zona ya mayoritariamente `FREE`, debe predominar `MOVE_AND_CAPTURE`.
Un giro de `LOOK_AND_CAPTURE` solo procede para despejar una pose o corredor
realmente desconocido; nunca sustituye una captura de pared al llegar a una
pose libre.

## Responsabilidades y exclusiones

`PointSelectionWorker` decide subROI, seccion, objetivo visual y pose; no
integra depth, no modifica voxeles y no espera al dron. `VoxelMapBuilder`
materializa la evidencia y actualiza la geometria U en el mismo cambio visible
al planificador. `TrajectoryPlanningWorker` solo decide la ruta segura.

No pertenecen a esta subfase: ocupacion persistente por MapPoints, el control
fisico, la optimizacion fiducial o la nube densa de Fase 8.

## Validacion y exito

Una prueba integrada debe mostrar: avance lateral entre secciones pendientes,
capturas depth de pared tras movimientos FREE, activacion de secciones por
`OCCUPIED=1` o `SIN_FACHADA`, y relevo `TO_FINISH` al alcanzar la cara abierta
sin completar toda la U. No se acepta cobertura lineal heredada ni seleccion
por rango `0.2..0.6`.
