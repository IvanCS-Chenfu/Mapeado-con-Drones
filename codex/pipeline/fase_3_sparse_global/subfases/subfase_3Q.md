# Subfase 3Q - Optimizacion covisible comun por loop y fiducial

## Estado

```text
ACTUAL; REABIERTA PARA DIAGNOSTICAR Y CORREGIR OPTIMIZACIONES
```

## Lectura obligatoria antes de retomar 3Q

La reentrada ha sido solicitada tras cerrar Fase 4. Esto fija 3Q como subfase
actual, pero no autoriza todavía cambios funcionales: primero se debe preparar
el diagnostico, acordar alcance/prueba y recibir autorizacion explicita.

Este contrato no contiene por si solo toda la evidencia necesaria para cambiar
3Q. Antes de planificar o implementar cualquier correccion se debe leer:

- [`historial_3Q_RESUMEN.md`](../historial/por_subfase/historial_3Q_RESUMEN.md),
  que conserva el estado operativo, fallos conocidos y puntos de reentrada;
- [`historial_3Q.md`](../historial/por_subfase/historial_3Q.md), en particular
  la revision posterior de la prueba 194 sobre los dos loops asimetricos y
  ambiguos del dron 2 antes del segundo fiducial hard.

No se debe reinterpretar ni corregir 3Q usando unicamente este MD: las pruebas
fallidas y sus causas forman parte del contrato efectivo.

La prueba larga 191 y la revision visual del usuario ofrecen un resultado muy
bueno y permiten avanzar. La subfase no se considera cerrada definitivamente:
si en pruebas futuras reaparecen loops repetitivos, dobles paredes,
optimizaciones innecesarias o ventanas excesivamente costosas, se volvera a
este contrato para revisar seleccion multi-region, umbrales y admision previa
al solver.

## Reentrada obligatoria desde la prueba 213

La prueba 213 de Fase 4 debe revisarse de nuevo en 3Q. La mision y la cadena
fiducial terminaron, pero quedaron derivas visibles que los loops no corrigieron.
Hubo seis commits tempranos y nueve rechazos posteriores: siete por corredor
hard y dos por degradar loops previos. Las propuestas tardias usaban ventanas
de 288-313 KFs; varias reducian mucho el error loop, pero introducian hasta
0.13 m de exceso de corredor y algunas eran ambiguas, con hasta 29 competidores.

Posible causa a comprobar: seleccion loop ambigua y ventanas grandes combinadas
con un validator de corredor demasiado estricto. Al retomar 3Q se debe reproducir
213 y correlacionar cada deriva con su propuesta antes de ajustar seleccion,
ventana o tolerancia, sin debilitar los fiduciales hard. El detalle numerico
queda en `historial_3Q_RESUMEN.md`.

3Q convierte `OptimizationEvidence` en una rama real de la misma `LoopTask`
BAJA y generaliza la optimizacion fiducial para que ambas rutas consuman un
grafo SE(3) comun con constraints temporales, covisibles, fiduciales y de loop.

```text
LoopTask -> 3N -> 3O -> error alto
         -> stop_drones=true
         -> grafo covisible -> solver -> validacion
         -> commit poses/constraints -> fusion 3P directa opcional
         -> dirty sets -> task end -> stop_drones=false
```

La prioridad de la tarea no cambia y no existe preemption: una tarea fiducial
MAX que llegue durante 3Q espera a que termine la `LoopTask` activa y sera la
siguiente en empezar. El flujo principal continua incorporando datos mientras
el mission gate no envia un nuevo goal.

## Acuerdos principales

- loops inter/intra dron o submapa siguen la misma decision geometrica;
- una optimizacion por loop requiere dos queries independientes coherentes;
- una region robusta de error alto domina sobre regiones fusionables de la
  misma tarea; una mezcla no puede ocultar la necesidad de optimizar;
- una optimizacion incorpora entre una y tres regiones geometricamente
  distintas de la hipotesis coherente dominante, no solo la de mas inliers;
- cada constraint loop se valida al final; puede existir un unico rebuild sin
  una region discordante, nunca un ciclo de solves;
- la ventana es el subgrafo minimo conectado hasta autoridades hard;
- una dependencia soft se sigue incluso a otro submapa y puede incorporar un
  tramo completo delimitado por dos fiduciales;
- fusiones anteriores empiezan como constraints relativas blandas medibles;
- fiduciales hard son los unicos puntos world absolutamente inmoviles;
- covisibilidad confirmada entra tanto en optimizacion loop como fiducial;
- la ruta fiducial expande entre submapas solo por loops/fusiones
  `ServerLoopGeometric` aceptados y dependencias soft; la covisibilidad ORB
  nativa solo refuerza submapas ya incluidos;
- se mantienen inicialmente los umbrales vigentes y se instrumenta por que una
  hipotesis aun no tiene apoyo independiente;
- el 30 % es densidad base de controles, ampliable por constraints fuertes;
- se reutilizan builder, solver, validator, commit y continuidad existentes;
- inicialmente solo se comprometen accepts completos;
- inliers RANSAC viven en la tarea y se reutilizan para fusion posterior;
- una fusion posterior omitida no invalida una optimizacion correcta;
- stale/rollback termina el intento y encola una BAJA fresca deduplicada;
- el commit se rebasa sobre snapshot/poses actuales: un control intermedio que
  haya sido retirado o refinado durante el solve se omite, pero siguen siendo
  obligatorios los extremos de las aristas loop y los controles hard/fixed. Un
  control obligatorio culled puede conservarse como apoyo virtual de
  interpolacion si todavia existe y su pose raw no ha derivado; no se vuelve a
  activar ni se escribe. El lote exige ademas un minimo de dos controles
  realmente activos por submapa; si falta geometria obligatoria o esos apoyos,
  el intento termina stale sin escrituras;
- raw nunca se modifica y el worker secundario nunca publica.
- un submapa nunca anclado conserva anclaje loop sin limite de distancia;
- un nuevo `map_epoch` nacido tras perder un submapa anclado valida cada anchor
  loop contra una envolvente derivada de su trayectoria raw acumulada;
- todas las aristas `ServerLoopGeometric` conectadas se siguen transitivamente
  y entran sin bonus de peso artificial;
- una propuesta loop se rechaza con cero escrituras si mejora los loops nuevos
  rompiendo temporal, covisibilidad, fusiones previas, hard o corredores;
- un tramo entre dos fiduciales conserva la ultima solucion fiducial como
  referencia y admite provisionalmente hasta 5 m/20 grados en su interior,
  con limites menores hacia los extremos; si el estado vigente ya excede ese
  margen, un loop puede conservarlo o reducirlo, pero nunca aumentarlo;
- tras un commit loop o fiducial se reencolan todos los KFs movidos como BAJA
  con intencion `FusionRefresh`: repiten BoW/geometria y pueden fusionar o
  actualizar scores, pero una evidencia de error alto no inicia otra
  optimizacion desde ese rerun. Los loops normales nacidos de delta/snapshot
  mantienen el flujo completo; una pareja ya fusionada se omite sin cancelar
  candidatos nuevos del KF.

## Documentos

- `subfase_3Q_especificacion.md`: topologia, ventanas, autoridad e invariantes.
- `subfase_3Q_implementacion.md`: tipos, componentes, solver, commit y eventos.
- `subfase_3Q_testing.md`: tests, matriz Gazebo, logs y revision visual.
- `subfase_3Q_criterios.md`: exito, parcial, fallo y parametros provisionales.

El acuerdo principal es ejecutable, pero puede ajustarse si las pruebas reales
revelan sobrerigidez, constraints insuficientes, mala seleccion de ventana o
coste excesivo. Cada cambio funcional requerira conversacion y quedara
registrado sin ocultar los intentos anteriores.

## Resultado vigente

- build final 2/2 y regresiones funcionales finales correctas: grafos 14/14,
  pipeline 9/9, cola 6/6, CTest `orbslam3_multi` 9/9, web 1/1 y servidor 4/4;
- 187 valida el escenario corto con tres optimizaciones/tres commits, 16
  `FusionRefresh` altos diferidos, 1047 tareas y `pending=0`;
- 188 repite literalmente las 25 etapas/dos vueltas de 176: scenario/tool
  exit 0, nueve commits loop `Full`, ocho commits fiduciales, dos anchors loop,
  cero hard failures y recursos estables;
- los nueve commits loop reducen el error medio de traslacion de 0.469849 a
  0.089286 m; 17 propuestas incompatibles se rechazan sin escritura;
- 995 fusiones se comprometen en 188. Tras 180 s queda backlog de refresh/fusion
  pero desciende de 323 a 310 y no activa `blocking_failure`;
- 176 y 179 permanecen como fallos visuales historicos; 180-186 conservan los
  intentos que localizaron y corrigieron corredor, revision, culling y
  realimentacion post-opt;
- 191 corrige el bloqueo de 189 y permite continuar, aunque conserva coste en
  ventanas grandes;
- 194 reproduce una mala deformacion visual del dron 2: dos loops asimetricos
  ambiguos movieron 359/362 KFs antes del segundo fiducial hard. El diagnostico
  completo y el punto de reentrada estan en el historial obligatorio.

Conclusion agregada: `CONSEGUIDA PARA EL CIERRE DE FASE 3`. La prueba 194
permanece como fallo historico y punto de aprendizaje; la prueba 195 no
reproduce la mala optimizacion final y el usuario confirma RViz2 perfecto.

La prueba 213 se incorpora como nueva reentrada obligatoria: completa la mision,
pero la calidad loop no queda validada y las derivas observadas exigen revisar
los rechazos posteriores a los corredores fiduciales.

## Posible mejora futura no implementada

Una futura revision puede hacer crecer el apoyo independiente exigido antes de
optimizar segun la magnitud de la correccion propuesta y la ambiguedad o
separacion de las regiones:

- candidato cercano y poco ambiguo: al menos dos detecciones independientes;
- candidato lejano o arriesgado: umbral creciente, incluso 8-10 detecciones;
- los apoyos deben proceder de KFs, instantes o puntos de vista distintos; una
  repeticion correlacionada del mismo candidato no cuenta como nueva evidencia;
- al alcanzar el umbral se ejecuta una unica optimizacion, no 2-10 solves
  consecutivos que puedan reforzar una hipotesis falsa.

La distancia por si sola no debe dominar: una correccion grande o una fachada
repetitiva pueden exigir mas evidencia aunque las poses estimadas esten cerca.
Esta politica queda solo documentada como punto de estudio. No se modifica el
runtime, los umbrales ni la logica de 3Q en el cierre de Fase 3.
