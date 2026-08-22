# 01 - Estado actual del proyecto

Leer primero `00_CONTEXTO_COMPACTACION.md` y
`01_ESTADO_ACTUAL_RESUMEN.md`.

## Estado vivo

```text
Fase actual: Fase 2 - separacion servidor/dron/simulacion
Fase 3: CONSEGUIDA
3B-3Q: CONSEGUIDAS
3R: CONSEGUIDA; RECALIBRACION TECNICA Y CIERRE RVIZ2 CONFIRMADOS
3S: CONSEGUIDA; PERFIL DEBUG Y MODO SILENCIOSO VALIDADOS
3T: CONSEGUIDA; LIMPIEZA, CONFIGURACION Y HANDOFF VALIDADOS
Ultima ejecucion: 196 valida el perfil 3S completamente false
```

## Arquitectura

El flujo principal conserva raw, poses, score, builder incremental y
publicacion serial. El worker secundario global ejecuta, sin preemption,
fiducial MAX, `DatabaseUpdateTask` MEDIA y `LoopTask` BAJA.

La MEDIA compromete un patch de covisibilidad y crea las BAJAS. Cada BAJA
coalesce por huella semantica, revalida geometria exacta, actualiza el indice BoW derivado, agrupa regiones y ejecuta
subnubes/matching/RANSAC. Una fusion compatible continua en 3P; el error alto
entra en grafo/solver/commit 3Q y puede fusionar directamente despues. El
anchor loop confirmado sigue modificando poses mediante un batch atomico.

`GlobalPoseStore` mantiene dependencias blandas padre-hijo para anchors loop.
El movimiento del apoyo propaga rigidamente el hijo. Su primer fiducial directo
reancla todo el submapa, corta la dependencia y queda hard. El
worker secundario solo marca KFs dirty; el siguiente principal reproyecta MPs y
publica.

3R hace al score raw dependiente de base ORB, distancia y aislamiento
recuperables, mas inliers confirmados. El fused score es la media de todos sus
raw miembros mas `0.04*N`. La visibilidad sparse solo diagnostica y el builder
publica todos los puntos con score/rgb. La distancia usa limite cercano fijo
1 m y limite lejano `83.333333*baseline`, con banda neutra 1-5 m para el
baseline actual y caidas cuadraticas acotadas.

## Invariantes

- `submapa=(drone_id,map_epoch)`;
- raw y BoW original son inmutables para loops/optimizacion;
- un unico worker secundario, tarea activa no interrumpible;
- commit atomico, sin estado parcial ante stale/conflicto;
- sin GT para BoW, matching, RANSAC ni anchor loop;
- 3Q no modifica raw, no publica desde secundario y no mueve hard fiducials.

## Validacion

El primer replay 152 completo la entrada pero dejo unas 384 tareas secundarias,
por geometria intra-submapa redundante y loops de unos 2.4 s; se conserva como
`PARCIAL`. Tras acotar subnubes/iteraciones y aprovechar covisibilidad fuerte,
replay 153 procesa 806 tareas y termina vacio, con latencias finales de
0.16-0.18 s y cero fallos duros.

Prueba 157 valida 78 KFs hijos propagados en el mismo commit del padre. La
tipica 156 valida reanchor post-loop de 32 KFs, tres commits fiduciales, cola
final vacia y cero fallos duros. La huella final reduce la carga de 9.20 a 2.18
tareas/KF. ORB crea siete submapas; cuatro quedan anclados y tres diferidos.

## Validacion 3P y punto de reentrada

La prueba 159 aborto el servidor por un track absorbido que permanecia en
`touched_tracks`; se conserva como `NO CONSEGUIDA`. La correccion añade
validacion local, regresion exacta y barrera de excepciones del worker.

La prueba 160 completa 56 commits, cinco stale y un rollback; drena 1116 tareas
sin hard failure y el servidor termina limpio. `GlobalMapBuilder` consume
tracks hasta la ultima publicacion. El usuario confirma que RViz2 y el grafo
web se vieron muy bien.

El ajuste posterior elimina los objetivos temporales de commit/visibilidad.
La prueba 161 completa `56/56` regiones, encola y termina los 19 retries
necesarios, aplica ocho fusiones y cuatro optimizaciones fiduciales full y
cierra `pending=0`/cero hard. El prepare aceptado sube a
`633.852/1087.130 ms` de media/maximo, coste aceptado por ahora porque no
bloquea el escenario ni el drenaje. El usuario da por concluida 3P. Solo pide
reorganizar posteriormente el layout desktop del grafo web para mejorar su
legibilidad, sin señalar defectos de topologia o funcionamiento.

## Validacion 3Q y punto de reentrada

El grafo comun resuelve fiducial absoluto y loop relativo con hard fijos,
temporal, covisibilidad sparse, constraints previas y dependencias soft. El
commit multi-submapa propaga tails/hijos y la misma BAJA termina con fusion 3P.

Las pruebas visuales 176 y 179 se conservan como fallos. La segunda acepto una
hipotesis repetitiva de unos 27 m sin medir la estructura previa. La correccion
incorpora continuidad para epochs perdidos, cierre transitivo por fusiones,
residuales temporales/covisibles, corredores hard-hard y rechazo sin escritura.

Los intentos 180-186 localizaron, sin ocultarlos, exceso de corredor heredado,
revision redundante, culling concurrente de controles y realimentacion post-opt.
El commit actual rebasa sobre poses vigentes, permite apoyos virtuales culled
con raw estable y reencola los KFs movidos como `FusionRefresh`, que conserva
fusion/score pero no puede iniciar otra optimizacion.

La prueba 187 completa el escenario corto con tres commits, 1047 tareas,
`pending=0` y cero hard failures. La 188 repite las 25 etapas/dos vueltas de
176: nueve commits loop, ocho commits fiduciales, dos anchors loop y 995
fusiones. Los loops comprometidos reducen el error medio de 0.469849 a
0.089286 m; 17 propuestas estructuralmente incompatibles se rechazan sin
escritura. Recursos estables y cero hard failures.

La prueba 189 se interrumpe tras un gate de 358.8 s causado por cinco solves
repetitivos de 63-70 s y backlog post-opt. La correccion añade precheck
fiducial/corredor antes del builder, ledger regional revisionado, agrupado y
filtro espacial de `FusionRefresh`, y separa mantenimiento de pendientes
criticos para backpressure.

La prueba 191 completa las dos vueltas, drena 2104 secundarias a `pending=0` y
termina sin hard failures. Registra cinco rechazos previos, 42 hits del ledger
y gate maximo 80.272 s. Permanece una ventana residual de 786 KFs/83.44 s
porque su relacion protegida directa era coherente y otra region disparo el
solve; queda para perfeccionar seleccion/umbrales.

El usuario valora muy positivamente el resultado y decide avanzar. En ese
momento 3Q queda `A REVISAR`; tras la revision visual correcta de 195 se acepta
para el cierre. Si reaparecen loops incorrectos, dobles paredes,
optimizaciones innecesarias o ventanas excesivas, se retomaran seleccion
multi-region, umbrales y admision previa al solver desde 189/191/194. Queda
documentada, sin implementar, evidencia adaptativa de dos apoyos para
candidatos cercanos y hasta 8-10 para candidatos lejanos o ambiguos.

## Auditorias absorbidas de arquitectura y visualizador

La auditoria del runtime confirma que la arquitectura transversal de 3T ya fue
implantada por 3C-3R: un worker principal y uno secundario, autoridades
separadas, propuestas privadas, commits revisionados, dirty sets y publicacion
exclusiva del principal. El usuario considera muy bueno el rendimiento y no
solicita mas cambios de sincronizacion.

3U tambien habia incorporado ya las correcciones que su cabecera antigua daba
por pendientes: conexion SSE desde el presente, reconexion por `Last-Event-ID`,
`state_reset`, drenaje por frame y lifecycle secundario por `flow_id`. El
contrato fuente pasa 9/9 y el usuario confirma que el grafo web es muy bueno y
funciona bien. Ambas subfases quedan `CONSEGUIDAS`; no hubo codigo ni simulacion
nueva durante el cierre.

## Auditorias absorbidas de regresion y rendimiento

El usuario acepta las pruebas 187, 188, 191 y 194 como regresion integral
suficiente: ejercitan flujo principal/secundario, loops, fiduciales, fusion,
score, publicacion, colas, recursos, RViz2 y grafo web. 3V queda `CONSEGUIDA`
por evidencia acumulada, sin una nueva ejecucion monolitica ni A/B de
telemetria.

La politica de rendimiento y robustez vigente mantiene histeresis, separacion
critico/mantenimiento, prioridad fiducial, `FusionRefresh` no recursivo,
coalescing y monitorizacion de recursos. El usuario considera bueno el
resultado y decide no introducir mas limites o metricas. 3W queda `CONSEGUIDA`;
los picos residuales documentados se aceptan y no se ocultan.

## Cierre 3T

Se retiraron las rutas `legacy/`/`legacy2/`, snapshots y contratos absorbidos,
con recuperacion disponible desde el checkpoint `1b96a7a`. La auditoria deja
una unica ruta activa por scheduling, autoridad y publicacion. ADR 0009 define
la propiedad de configuracion y los despliegues servidor/simulacion cargan seis
YAML tematicos cuya igualdad y cobertura estan protegidas por tests.

El build final pasa 3/3 paquetes y los CTest 9/9, 10/10 y 8/8. La prueba 195
termina `success=true`, drena 741 principales y 1262 secundarias a cero,
mantiene `max_active=1`/`hard_failed=0`, compromete 11 loops y publica 23.978
puntos con score/rgb. Recursos estables y guarda inactiva. La validacion visual
humana de 195 fue confirmada por el usuario como correcta.

## Cierre 3S y Fase 3

La nueva 3S incorpora `fase3_debug.yaml` con controles independientes para
RViz2, grafo web, navegador y logs terminales. La prueba 196 termina
`success=true`: cuatro goals correctos, servidor operativo, cero marcadores
`[F3*]` y ningun proceso RViz2, bridge o navegador con los flags a false. Los
errores reales permanecen visibles mediante nivel ROS `error`.

El handoff completo esta en `RESULTADO_FINAL_FASE_3.md`. La Fase 3 queda
`CONSEGUIDA`; 3Q conserva la incidencia 194 y la mejora adaptativa como
referencia futura, sin bloquear el inicio de Fase 2.
