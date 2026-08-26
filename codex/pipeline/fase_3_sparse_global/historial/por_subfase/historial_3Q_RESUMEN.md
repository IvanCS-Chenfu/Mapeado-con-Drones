# Historial 3Q - resumen

## Estado vigente

`A REVISAR`: la correccion reabierta tras la prueba 213 y la mejora conservadora
posterior estan implementadas y validadas. La prueba 220 da un resultado visual
general excelente y demuestra la cascada world pendiente. El usuario acepta
continuar sin otra correccion; 3Q solo se reabrira si vuelve a aparecer el
movimiento incoherente aislado observado en una ventana de 296 KFs.

## Implementacion vigente

- `PoseGraphBuilder::BuildSegmented()` selecciona intervalos temporales
  delimitados por hard para loop y fiducial; no existe loop fiducial sintetico
  ni cierre por submapa completo.
- La expansion a otros submapas exige una relacion server incidente en el
  intervalo. La covisibilidad ORB nativa refuerza, pero no descubre submapas.
- Solo los fiduciales hard son inmoviles permanentemente. Los KFs intermedios
  pueden superar 2 cm de movimiento si pasan estructura y commit.
- Los KFs con pose `LoopOptimized` o `FiducialOptimized` pueden reajustarse; se
  rechaza atomicamente si alguno cambia mas de 5 m o 20 grados.
- Tres segmentos soporte independientes unidos por `ServerLoopGeometric`, con
  cobertura minima 60 %, se fijan solo durante ese solve. Query no pertenece al
  scaffold y la marca no se persiste como hard.
- Apoyo adaptativo 2/4/6 segun perdida/asimetria, ambiguedad y correccion grande,
  con progresion coherente de query y candidate.
- `information_weight` afecta coste y relajacion por familia. Convergencia
  `0.05 m/0.03 rad`, fusion `0.20 m/0.12 rad` y commit seguro
  `0.25 m/0.15 rad` son decisiones independientes.
- Todo commit loop conserva `PriorLoop`; fusionar landmarks requiere ademas
  quedar dentro del umbral de fusion.
- Cuando una constraint activa obtiene autoridad world por un fiducial, se
  ejecuta una cascada acotada a su componente y se reencolan sus endpoints para
  reconciliacion normal.
- Tras perdida reciente puede usarse un unico loop solo con continuidad
  estricta configurable (`0.50 m`, `0.15 rad`, recorrido maximo `2 m`). La
  constraint queda provisional, sin fusion ni propagacion amplia, hasta un
  segundo apoyo independiente o un fiducial. Fuera de esa banda sigue 2/4/6.

## Evidencia reciente

- Prueba 218: escenario correcto, pero 0 commits loop; diez propuestas validas
  se cancelaron por `hard_corridor_displacement_exceeded`. Demostro que la
  deadband de 2 cm convertia incorrectamente los KFs internos en casi-hard.
- Builds posteriores: `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`
  correctos. CTest 9/9, 12/12 y 10/10 respectivamente.
- Prueba 219: 17/17 pasos, 22/22 goals, scenario/tool exit 0 y recursos estables.
  De 30 solves terminados, 22 hacen commit y ocho se descartan justificadamente;
  cinco commits terminan tambien en fusion post-opt.
- Los ocho descartes son: dos intervalos demasiado pequenos, dos sin mejora de
  todas las `CurrentLoop`, dos sobre el commit seguro, uno con coste global
  creciente y uno que degradaba covisibilidad nativa.
- No hay rechazo por 2 cm, movimiento hard, violacion 5 m/20 grados,
  `blocking_failure` ni fatal. Maximo reajuste de pose ya optimizada:
  1.404495 m / 0.181256 rad.
- Revision visual del usuario: una esquina queda completamente corregida; las
  dos derechas corrigen peor y otra esquina solo parcialmente. Esta lectura de
  219 queda superada como estado vigente por la repeticion 220.

- Prueba 220: builds correctos y CTest 9/9, 12/12 y 10/10; 17/17 pasos,
  22/22 goals, `success=true`, exit 0 y recursos estables tras el retry Gazebo.
- La cascada world se ejecuto al llegar el primer fiducial y anclo otro submapa
  conectado. La recuperacion 1/1 rechazo correctamente una continuidad de
  unos 5.6 m y `(1,2)` termino anclado por el fallback normal 6/6.
- Revision visual 220: resultado general excelente; un movimiento deterioro
  algo la esquina superior derecha mirando hacia `+Y`.

## `map_epoch`, anchors y zonas derechas

Submapas observados: dron 1 epochs 0-3; dron 2 epochs 0-1. Los cambios
`(2,1)`, `(1,2)` y `(1,3)` aparecen durante los tramos de la derecha. Las
aristas temporales no cruzan epochs, por lo que la hipotesis visual de menor
autoridad en esas esquinas es consistente con los logs, aunque no demuestra por
si sola causalidad geometrica.

No hubo anchors por loop: todos los `[F3O-LOOP-DONE]` tienen `anchors=0` y no
aparece `[F3O-FID-LOOP-REANCHOR]`. Los cinco anchors creados fueron fiduciales
directos para `(2,0)`, `(1,1)`, `(2,1)`, `(1,2)` y `(1,3)`.

La causa no es una desactivacion. Un anchor loop exige activar una constraint
relativa con apoyo suficiente y que la componente contenga ya un submapa con
autoridad world. `(1,1)` activo constraints 2/2 y 3/2 antes de los primeros
fiduciales, cuando ambos lados seguian sin world. `(2,1)` y `(1,2)` rechazaron
su geometria durante la ventana sin anchor. `(1,3)` necesitaba seis apoyos por
riesgo/ambiguedad y alcanzo cuatro antes de recibir fiducial directo.

La distribucion 3Q tambien es asimetrica: tras el bloque inicial, la mayoria de
los solves usan query `(2,1)`; `(1,2)` no inicia ninguno y `(1,3)` completa uno
antes del cierre. Los grafos pueden incluir 3-4 submapas, pero una query nueva
con menos continuidad no queda tan bien condicionada como el tramo con mas
fusiones previas.

## Defecto residual de 220

`task=1000000005590`, reencolada tras el anchor de `(1,2)`, fue la unica
optimizacion loop de 220 que paso de las ventanas habituales de 49-75 KFs a
tres submapas, 296 KFs y 277 KFs movidos. Para candidatos consecutivos 65/66/67,
las constraints con 65 y 67 ya estaban alineadas casi a cero y solo la de 66
presentaba `1.0118 m/0.1290 rad`. Esto no prueba que solo la pose del 66 fuese
incorrecta. La seleccion incorporo las tres regiones como `CurrentLoop` y no
contrasta ese residual relativo aislado con sus vecinos.

El solve bajo el coste `20247.82 -> 71.47`, pero permitio un incremento
estructural de 0.28883 m y reajustes de 0.32602 m. El validator lo acepto porque
los limites actuales son 2 m temporal, 1 m covisible y 5 m/20 grados para KFs
ya optimizados. La fusion posterior stale no revierte el commit de poses.

Los fiduciales no muestran la misma firma: la tarea superior corrigio un error
absoluto real de 1.183 m mientras el otro dron ya tenia control coherente allí.
La telemetria no conserva el reparto de IDs movidos por submapa, por lo que la
atribucion espacial no es demostrable al 100 %, pero la singularidad de
`1000000005590` frente a todos los demas commits da confianza alta.

El builder ya conserva las tres medidas compatibles como `CurrentLoop` y el
solver intenta llevar todas a `0.05 m/0.03 rad`. El hueco es de enforcement:
al agotar 160 iteraciones devuelve `Converged`; el validator permite hasta
`0.25 m/0.15 rad` por loop y solo exige que mejore alguno. Siguiente paso:
mantener la ventana completa y las tres `CurrentLoop`, registrar errores por
arista y exigir convergencia/no degradacion individual, junto con una guarda
estructural local. No cambiar cascada, recuperacion 1/1 ni fiduciales.

Esta mejora queda documentada, no activa. No ejecutar cambios adicionales de
3Q salvo nueva evidencia runtime del mismo fallo o peticion explicita del
usuario.

Detalle cronologico: `historial_3Q.md`.
