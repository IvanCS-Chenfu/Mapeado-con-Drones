# 00 - Contexto de compactacion

## Checkpoint vigente 2026-08-26

```text
Estado: Fase 4 CONSEGUIDA Y CERRADA con alcance 4A-4H; Fase 3 reabierta en 3Q.
Preparacion 4G+4H: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA y consumida
Prueba acordada: trayectoria tipica con Gazebo/RViz2 y ventanas 5 s, seguida
de smoke web separado.
Dudas abiertas: ninguna.
Resultado: builds correctos; CTest Servidor 150/150 y Simulacion 85/85 sin
fallos; prueba 216 correcta sin ruta GT fiducial, con 52/52 primary y cobertura
de los tres objetos; smoke 217 correcto con ambos grafos live.
Decision de cierre: el usuario aplaza 4I como regresion opcional futura; no
condiciona ni reabre el cierre de Fase 4.
Subfase actual: 3Q, para diagnosticar y corregir las optimizaciones que dejaron
derivas visibles en la prueba 213.
Preparacion 3Q: CERRADA
Acuerdo cerrado 3Q: si
Autorizacion funcional 3Q: CONCEDIDA el 2026-08-26
Prueba acordada 3Q: regresiones deterministas de los casos 194 y 213, tests y
builds seleccionados de `orbslam3_multi`, `orbslam3_server` y
`simulacion_dron`, y repeticion de la trayectoria tipica equivalente a la 213
con Gazebo, RViz2 y logs F3Q; la revision visual final corresponde al usuario.
Dudas abiertas 3Q: ninguna.
Clarificacion conversada 3Q: los fiduciales hard no se mueven; el corredor
hard-hard conserva para cada KF entre el primer y ultimo hard una pose world de
referencia y una tolerancia local modulada por su posicion en el tramo. El
validador actual rechaza si el exceso final sobre esa tolerancia supera al
exceso inicial en mas de `1e-8`, por lo que un exceso nuevo de `0.000416 m`
cancela toda la propuesta aunque los hard sigan exactamente fijos. Pendiente
explicar al usuario el grafo completo y cerrar despues la politica correctiva.
Propuesta funcional del usuario 3Q: construir la ventana por segmentos
temporales delimitados por el hard anterior y, si existe, el siguiente;
expandir a otros submapas solo mediante covisibilidad/fusiones confirmadas cuyos
extremos pertenezcan a esos segmentos; aplicar igual expansion a optimizaciones
fiduciales; condensar como priors un consenso mayoritario de al menos tres
submapas para que absorba la correccion el lado query; exigir secuencias de
loops coherentes al lado no anclado, continuidad espacial reforzada tras
perdida y apoyo creciente con riesgo/distancia. Valoracion provisional: buena
direccion, pero la mayoria debe medir fuentes independientes y cobertura
distribuida, y la distancia no debe decidir sola porque la pose world puede
estar precisamente afectada por deriva.
Confirmaciones posteriores 3Q: la rama 3Q parte de RANSAC valido con residual
bajo y discrepancia world alta; la segmentacion sera simetrica para query y
candidate; se acepta expansion solo por relaciones server confirmadas,
consenso/scaffold independiente y secuencias query-candidate coherentes. El
solver debe mantener todos los hard inmoviles y acercar las nubes segun la
transformacion RANSAC, permitiendo reparto de correccion entre ambos lados de
acuerdo con aristas y autoridad. Continuidad tras perdida y proteccion de zonas
repetidas se conservan. El usuario propone reducir el umbral de fusion.
Dato runtime verificado: hoy `loop_fusion_translation_threshold_m=0.35` y
`loop_fusion_rotation_threshold_rad=0.25` (14.3 grados); el mismo par clasifica
fusion directa y fija el residual final exigido por el validator. Cambiarlo sin
separar ambos usos puede aumentar simultaneamente solves y rechazos.
Auditoria preparatoria detallada 3Q: viable sin tocar ORB-SLAM3 ni mensajes.
`LoopPipeline` hoy acumula solo pose/id query y no comprueba progresion del
candidate; el apoyo adaptativo debe conservar endpoints canonicos de ambos
lados y una secuencia coherente. Regla propuesta configurable: 2 apoyos sin
riesgo, 4 con una senal y 6 con dos o mas; senales simples: asimetria protegida
o perdida reciente, ambiguedad competidora y correccion grande. El cierre por
submapa completo de `SparseGlobalBackend` y `BuildLoop()` debe sustituirse por
intervalos delimitados por hard consecutivos, siguiendo solo aristas server
incidentes; varios intervalos del mismo submapa se combinan en un unico batch
atomico. El fiducial multi-submapa deja de usar un loop sintetico y reutiliza
esa seleccion real. El consenso de tres segmentos independientes no fija
candidate: aporta `PriorLoop` y cobertura, y el solver reparte movimiento;
solo hard queda inmovil. Hallazgo adicional: `information_weight` participa en
el coste pero no en `RelaxEdge`, por lo que debe normalizarse por familia y
afectar la relajacion para que autoridad/soporte gobiernen de verdad. Corredor:
referencias por parejas de hard consecutivos y deadband configurable inicial
`0.02 m / 0.00872665 rad`, sin tolerancia para hard. Fusion directa propuesta
`0.20 m / 0.12 rad`; separar parametro de residual final. Regresiones nuevas:
194 ambiguo/asimetrico queda en HOLD con apoyo insuficiente y ventana acotada;
213 acepta exceso numerico y rechaza exceso material; segmentacion loop y
fiducial, consenso no hard, pesos efectivos y commit multi-intervalo.
Clarificacion de histeresis 3Q: el usuario acepta la opcion recomendada y
pregunta por el objetivo cero. El error de pose `CurrentLoop` debe minimizarse
respecto a la transformacion RANSAC hacia cero; las poses de camara y los pares
de puntos no tienen por que coincidir exactamente por distinto punto de vista y
ruido. El solver actual corta segun fracciones del umbral de aceptacion, por lo
que se propone un target de convergencia propio cercano a cero. Una propuesta
entre el limite de fusion y el maximo seguro puede comprometer una mejora sin
fusionar landmarks; debe conservar la constraint relativa aceptada como
`PriorLoop` para una futura refinacion. Solo se fusionan puntos tras opt si el
error de pose queda dentro de `0.20 m / 0.12 rad` y pasa dispersion/evidencia.
Acuerdo final de umbrales 3Q: objetivo matematico del `CurrentLoop` cero;
convergencia practica propia `0.05 m / 0.03 rad`; fusion directa o post-opt de
landmarks solo hasta `0.20 m / 0.12 rad`; aceptacion maxima de una correccion
segura sin fusion hasta `0.25 m / 0.15 rad`. Entre los dos ultimos limites se
comprometen las poses si pasan las guardas estructurales, se conserva la
constraint como `PriorLoop` y no se fusionan landmarks. Por encima del maximo
seguro, o ante movimiento de hard/infraccion estructural material, se rechaza.
Trabajo activo: ejecucion completa de 3Q autorizada sobre el estado vigente
posterior a Fase 4, usando fiduciales reales y la trayectoria tipica actual.
Plan activo 3Q: localizar simbolos desde docs de paquete; implementar ventana
segmentada/expansion covisible/apoyo adaptativo; separar convergencia, fusion y
commit seguro; hacer efectivos los pesos y la deadband de corredor; retirar
logica obsoleta; ampliar regresiones; compilar, ejecutar suites y repetir la
trayectoria equivalente a 213 con Gazebo/RViz2 y logs reducidos.
Contexto 3Q leido: contrato 3Q, especificacion, implementacion, testing,
criterios, historial 3Q resumen y fragmentos 194/213 del historial largo; docs
de `orbslam3_multi` y `orbslam3_server` localizan `LoopPipeline`,
`PoseGraphBuilder`, `OptimizationValidator` y `SparseGlobalBackend`.
Preparacion Fase 5: CERRADA para ejecutar primero 5A como auditoria y
reconciliacion documental de `Fase_5_preparada_para_Codex.zip`.
Acuerdo cerrado Fase 5: si
Autorizacion funcional Fase 5/5A: CONCEDIDA y consumida el 2026-08-25 para el
alcance exclusivamente documental acordado.
Prueba acordada Fase 5/5A: revision de coherencia documental y
`git diff --check`; sin build, test ni simulacion porque 5A no modificara codigo,
launch, YAML funcional, configuracion ni mensajes.
Dudas abiertas Fase 5/5A: ninguna.
Acuerdo Fase 5/5A: actualizar el pipeline, resumen y contratos 5A-5H para
absorber las decisiones del zip; conservar 5I como nota/stub absorbido en 5H;
crear o actualizar el indice/historial documental sin inventar ejecuciones;
preparar 5B como primera subfase funcional. Se puede avanzar documentalmente
en paralelo con 3Q, pero antes de backend fuerte en 5C/5D se verificara el
cierre final de 3Q y el HEAD vigente.
Contexto Fase 5 leido: README/resumen/pipeline propuestos del zip, subfases
propuestas 5A-5H y nota 5I absorbida; pipeline/resumen/5A vigentes del repo;
docs de `orbslam3_ros2`, `orbslam3_msgs`, `orbslam3_multi`,
`orbslam3_server`, `dron_individual`, `simulacion_dron` y ADR 0002 GT.
Observacion Fase 5: HEAD actual `a44b8b8` difiere del SHA auditado del zip
`1d585059`; el zip cambia decisiones funcionales respecto a los MD vigentes:
`O_T_B` continuo para control, `W_T_B` global corregible aparte, usar ref-KF
real + `Tcr`, rechazar goals absolutos sin `W_T_O`, no imponer smoothing,
usar `GT_FALLBACK` temporal solo en F5 y absorber la antigua 5I en 5H. ADR
0002 debe reconciliarse antes de implementar ese fallback.
Decision GT_FALLBACK confirmada por el usuario: mantenerlo durante toda Fase 5
y hasta que Fase 6 aporte tareas reales de recuperacion. Si ORB-SLAM3 se pierde,
el dron debe poder terminar trayectorias largas como el rodeo del edificio sin
quedar bloqueado. Es una fuente temporal de continuidad de control, visible y
parametrizable; no alimenta el mapa, anchors, optimizacion ni la pose global
final, y su retirada queda como condicion explicita de Fase 6.
Plan ejecutado Fase 5/5A: reconciliar `pipeline_fase_5.md`, su resumen y los
contratos 5A-5H con el zip y el acuerdo conversado; dejar 5I como stub absorbido
en 5H; crear indice e historial de 5A; verificar referencias, estados y
`git diff --check`. No modificar codigo, launch, YAML, configuracion ni mensajes;
no ejecutar build, test o simulacion.
Resultado Fase 5/5A: CONSEGUIDA documentalmente. Pipeline, resumen y contratos
reconciliados; 5I absorbida en 5H; historial creado. Verificacion correcta:
referencias presentes, cero clausulas obsoletas buscadas, contratos menores de
250 lineas, bloques Markdown equilibrados y `git diff --check` correcto.
Trabajo activo Fase 5: no. 5B no esta preparada ni autorizada.
Preparacion por bloques Fase 5B-5H: EN_DEBATE.
Acuerdo cerrado bloques Fase 5: no.
Autorizacion funcional bloques Fase 5: PENDIENTE.
Propuesta de bloques: B1=`5B`; B2=`5C+5D+5E+5F` con checkpoints internos y
puerta humana tras metricas 5F; B3=`5G+5H` con reconciliacion previa de ADR
0002, prueba dirigida de `GT_FALLBACK` y vuelta final multi-dron.
Pruebas propuestas: tests deterministas por capa, builds aislados solo tras
cada bloque coherente y una simulacion integrada por bloque; B2 espera el
cierre de 3Q y termina presentando metricas para aceptacion del usuario; B3
incluye perdida dirigida y trayectoria tipica completa.
Dudas abiertas bloques Fase 5: confirmar la division en tres bloques y el
criterio de una sola simulacion integrada por bloque; concretar al preparar B3
el mecanismo reproducible para provocar `RECENTLY_LOST` sin contaminar el
estimador. No iniciar ejecucion F5 mientras 3Q comparta el worktree activo.
Cambios completados Fase 5/5A: reconciliados pipeline, resumen y contratos
5A-5H; 5I convertida en stub absorbido; creados `historial/INDEX.md`,
`historial_5A.md` y su resumen; sincronizados `PIPELINE_MAESTRO.md`, contexto
minimo, estado actual resumido y ultima sesion. No se modifico codigo, launch,
YAML funcional, configuracion ni mensajes; no hubo build, test o simulacion.
Siguiente accion exacta 3Q: leer los resúmenes y MDs vigentes de los componentes
afectados, localizar los simbolos exactos y registrar los archivos criticos
antes de editar codigo o configuracion.
Archivos criticos localizados 3Q: `loop_pipeline.hpp/.cpp`,
`pose_graph_problem.hpp`, `pose_graph_builder.hpp/.cpp`,
`optimization_manager.hpp/.cpp`, `optimization_validator.hpp/.cpp`,
`sparse_global_backend.hpp/.cpp`, `global_pose_store.hpp/.cpp`,
`global_map_server.cpp`, las dos copias `config/global_map/loop_fusion.yaml` y
tests de `orbslam3_multi`/Servidor/Simulacion. Hallazgos confirmados: expansion
transitiva en `BuildLoop`, relajacion sin peso efectivo, convergencia ligada al
umbral de fusion, validator sin banda separada de commit y corredor antiguo
`5 m / 20 grados` con comparacion de exceso casi exacta.
Siguiente accion exacta 3Q tras localizacion: inspeccionar solo las estructuras
y funciones senaladas, cerrar el diseno mecanico de tipos/parametros y aplicar
el primer bloque de implementacion y regresiones sin tocar ORB-SLAM3 ni msgs.
Bloque 1 implementado 3Q: umbral de fusion reducido a `0.20 m / 0.12 rad`;
convergencia separada `0.05 m / 0.03 rad`; commit seguro hasta
`0.25 m / 0.15 rad`; apoyo adaptativo 2/4/6 con progresion query-candidate,
ambiguedad, anclaje/perdida y correccion grande; `information_weight` afecta
la relajacion normalizado por familia; corredor reemplaza `5 m / 20 grados` y
comparacion `1e-8` por deadband `0.02 m / 0.00872665 rad`; commit seguro fuera
de fusion no fusiona landmarks. YAML Servidor/Simulacion sincronizados y
regresion numerica `0.000416 m` anadida. Aun pendientes ventana segmentada,
persistencia explicita de `PriorLoop`, limpieza final y validacion.
Siguiente accion exacta 3Q: compilar `orbslam3_multi` para detectar errores
mecanicos del bloque 1 antes de abordar la seleccion segmentada.
Build 3Q bloque 1: `orbslam3_multi` 1/1 correcto, exit 0, 58.8 s. Log completo
conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido.
Siguiente accion exacta 3Q: implementar la seleccion por intervalos temporales
hard delimitados y expansion solo por aristas server incidentes en esos
intervalos; adaptar commit atomico y tests antes del siguiente build.
Bloque 2 implementado 3Q: `BuildLoop()` parte solo de endpoints CurrentLoop,
calcula intervalos temporales entre hard consecutivos y expande iterativamente
solo cuando el extremo de una arista `ServerLoopGeometric` o dependencia soft
ya cae dentro de un intervalo seleccionado. Intervalos disjuntos del mismo
submapa no crean aristas temporales ni propagacion a traves del hueco. Cada
submapa conserva un unico batch con lista explicita de intervalos y el commit
solo actualiza KFs dentro de ellos. Eliminado el fixed artificial cuando no hay
hard; el gauge relativo se reparte entre lados movibles.
Siguiente accion exacta 3Q: rebuild aislado de `orbslam3_multi`; si compila,
ejecutar su CTest para localizar regresiones contractuales antes de completar
la ruta fiducial comun y persistencia PriorLoop.
Build 3Q bloque 2: `orbslam3_multi` 1/1 correcto, exit 0, 42.8 s. Log completo
conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido.
Siguiente accion exacta 3Q: ejecutar CTest de
`/home/chenfu/Gazebo/build/servidor/orbslam3_multi`; registrar el resultado
antes de corregir tests o continuar con el bloque funcional restante.
CTest `orbslam3_multi` intento 1: no ejecuto funcionalmente ninguna suite;
0/9 porque el entorno no encontraba
`liborbslam3_msgs__rosidl_generator_c.so` (exit 8). No es evidencia de fallo
del codigo. Correccion operativa mecanica: repetir el mismo CTest tras cargar
`/opt/ros/iron/setup.bash` y `/home/chenfu/Gazebo/install/servidor/setup.bash`.
CTest `orbslam3_multi` intento 2: 8/9 targets correctos; 7/12 casos de
`test_loop_pipeline` pasan y cinco contratos antiguos fallan. Causa: siguen
esperando aceptacion fija tras dos apoyos y rechazo previo `5 m / 20 grados`,
ambos comportamientos sustituidos deliberadamente por apoyo adaptativo 2/4/6.
`test_fiducial_optimization`, incluida la deadband numerica, pasa completo.
Siguiente accion exacta 3Q: reemplazar los cinco tests obsoletos por regresiones
de apoyo adaptativo/continuidad y ventana acotada, sin restaurar el guard viejo;
despues rebuild y CTest.
Tests 3Q actualizados: los casos de anclaje, error alto y perdida reciente
aportan suficientes KFs independientes para los defaults 2/4/6; el antiguo
test de rechazo previo por distancia ahora exige HOLD sin solver y soporte
adaptativo; el asimetrico exige evidencia solo tras reunir apoyo. No se ha
restaurado el guard `5 m / 20 grados`.
Siguiente accion exacta 3Q: rebuild `orbslam3_multi` y repetir CTest con overlay.
Build 3Q tests adaptativos: `orbslam3_multi` 1/1 correcto, exit 0, 4.89 s.
Log completo conservado y no leido. Siguiente accion exacta: CTest completo de
`orbslam3_multi` con overlays ROS/Servidor cargados.
CTest `orbslam3_multi` intento 3: 8/9 targets; el nuevo HOLD adaptativo pasa,
pero cuatro escenarios de pipeline aun no alcanzan/cierran su flujo esperado.
La suite de grafo/validator permanece correcta. Diagnostico siguiente: revisar
la geometria sintetica de progresion candidate y la persistencia de PriorLoop;
no relajar los defaults ni restaurar expectativas obsoletas sin comprobar la
causa exacta.
Diagnostico/correccion intento 3: el fixture usaba BoW identico en todos los
KFs, de modo que candidate quedaba artificialmente fijo aunque query avanzase;
ahora cada pareja temporal comparte una palabra distintiva. Hallazgo funcional
real: el `ServerLoopGeometric` aceptado solo se persistia durante fusion 3P;
ahora todo commit loop de poses aplica inmediatamente su constraint seleccionada
como `PriorLoop`, tambien en la banda segura sin fusion. Si esa persistencia
falla se informa error explicito y nunca se intenta fusionar.
Siguiente accion exacta 3Q: rebuild y CTest completo de `orbslam3_multi`.
Build 3Q regresion segmentada: `orbslam3_multi` 1/1 correcto, exit 0, 9.43 s;
log completo conservado y no leido. Siguiente accion: CTest completo con overlay.
CTest `orbslam3_multi` intento 5: 9/9 targets correctos, 100 %, incluida
regresion de intervalo hard `[4,7]`, deadband numerica, apoyo adaptativo,
continuidad de perdida, pesos y persistencia de PriorLoop. Siguiente accion:
Checkpoint de reanudacion tras compactacion 2026-08-26: la peticion mas
reciente es continuar la ejecucion 3Q ya autorizada; alcance, umbrales, prueba
y dudas permanecen sin cambios. Tras el CTest 9/9 se eliminaron campos y
razones obsoletas (`hard_corridor_alpha`, `protected_region_rejected` y
`waiting_second_independent_query`), pero esa limpieza aun no se ha recompilado.
Siguiente accion exacta 3Q: repetir busquedas estaticas, `git diff --check` y
comparacion de YAML; despues rebuild y CTest de `orbslam3_multi` antes de
completar la ruta fiducial comun, consenso, builds de integracion y simulacion.
Auditoria estatica posterior a limpieza 3Q: no quedan referencias no
documentales a los simbolos y parametros obsoletos buscados; `git diff --check`
correcto y las copias Servidor/Simulacion de `loop_fusion.yaml` son identicas.
Siguiente accion exacta 3Q: rebuild seleccionado de `orbslam3_multi`.
Build 3Q posterior a limpieza: `orbslam3_multi` 1/1 correcto, exit 0, 1 min
2 s. Log completo conservado por la herramienta y no leido. Siguiente accion
exacta 3Q: repetir CTest completo de `orbslam3_multi` con overlays cargados.
CTest 3Q posterior a limpieza: primer lanzamiento dentro del sandbox no fue
funcional porque CTest no pudo escribir `LastTest.log` en `build/`; repetido
con permiso operativo, 9/9 targets correctos, 100 %, 30.40 s. Siguiente accion
exacta 3Q: eliminar la adaptacion de optimizacion fiducial mediante loop
sintetico y reutilizar la seleccion segmentada mediante una entrada comun real;
anadir la regresion correspondiente antes de integrar Servidor.
Ruta fiducial comun 3Q implementada: `PoseGraphBuilder::BuildSegmented()` es la
seleccion compartida; `BuildLoop()` la alimenta con constraints RANSAC
`CurrentLoop` y `BuildExpandedFiducial()` con el target fiducial real, sin
`LoopTaskComputation` ni geometria sintetica. La expansion solo incorpora
`PriorLoop` server/dependencias confirmadas, mantiene intervalos explicitos y
solo hard queda fixed. Regresion multi-submapa reforzada para exigir problema
`FiducialAbsolute`, ventanas segmentadas y cero `loop_edges`. Siguiente accion
exacta 3Q: compilar `orbslam3_multi` y ejecutar su CTest completo.
Build 3Q selector fiducial comun: `orbslam3_multi` 1/1 correcto, exit 0,
39.3 s. Siguiente accion exacta 3Q: CTest completo con overlays para validar
la regresion multi-submapa y detectar cambios de comportamiento del solver.
CTest 3Q selector fiducial comun intento 1: 8/9 targets; todas las suites salvo
un caso pasan. `AnchorsUnanchoredSubmapAfterAdaptiveIndependentSupport` obtiene
una sola ventana y ningun `PriorLoop` al sustituir el anchor soft del hijo por
fiducial hard. El nuevo selector no esta siguiendo en ese fixture la relacion
padre-hijo hasta el segundo submapa. No se cambia el contrato; siguiente accion
exacta: inspeccionar IDs y representacion de esa dependencia, corregir la
incidencia y repetir build/CTest.
Diagnostico selector fiducial: la arista/dependencia del fixture estaba anclada
en un KF posterior al target y fuera de su intervalo; incluirla reproduciria el
cierre transitivo obsoleto. La regresion ahora exige una sola ventana y ningun
prior en ese caso. Se anadio un caso directo donde una arista
`ServerLoopGeometric` toca el intervalo: debe crear dos ventanas segmentadas,
un `PriorLoop` con soporte/peso reales y cero `CurrentLoop`. Siguiente accion:
rebuild y CTest de `orbslam3_multi`.
Build 3Q regresion de incidencia fiducial: `orbslam3_multi` 1/1 correcto,
exit 0, 12.2 s. Siguiente accion: CTest completo con overlays.
CTest 3Q incidencia fiducial: 9/9 targets correctos, 100 %, 29.89 s. La ruta
fiducial comun queda validada con expansion solo por arista incidente y sin
loops sinteticos. Siguiente accion exacta 3Q: hacer explicita y testeable la
regla de consenso de al menos tres segmentos y cobertura 60 % como refuerzo de
`PriorLoop`, sin convertir ningun KF no hard en fixed.
Consenso 3Q implementado: cobertura = KFs activos del intervalo que son
extremos de `PriorLoop` inter-submapa / KFs activos del intervalo. Una
componente de al menos 3 segmentos donde todos cubren al menos 0.60 multiplica
por 2 los pesos prior (cap 60), sin alterar `fixed`. Parametros ROS/YAML
explicitos y copias sincronizadas. Regresion de tres segmentos anadida con
cobertura minima 0.60, pesos 10->20 y solo KF hard fijo. Siguiente accion:
build y CTest `orbslam3_multi`.
Build 3Q consenso: `orbslam3_multi` 1/1 correcto, exit 0, 1 min 5 s.
Siguiente accion: CTest completo con overlays.
CTest 3Q consenso: 9/9 targets correctos, 100 %, 29.25 s. Quedan validados
selector segmentado loop/fiducial, incidencia server, apoyo adaptativo,
histeresis, pesos efectivos, corredor y consenso 3/60 sin hard artificial.
Siguiente accion exacta 3Q: build seleccionado y CTest completo de
`orbslam3_server`, seguido de integracion `simulacion_dron`.
Build integracion 3Q Servidor: `orbslam3_server` 1/1 correcto, exit 0, 20.3 s.
Siguiente accion: CTest del paquete Servidor con overlays.
CTest integracion 3Q Servidor intento 1: 11/12 tests correctos; unico fallo
`uncrustify` por formato de una llamada `std::max` en la carga del apoyo
adaptativo. No hay fallo funcional. Correccion mecanica exacta indicada por el
linter y repeticion de build/CTest.
Build Servidor tras formato: `orbslam3_server` 1/1 correcto, exit 0, 20.0 s.
Siguiente accion: repetir CTest 12 targets.
CTest integracion 3Q Servidor intento 2: 12/12 correctos, 100 %, 5.65 s,
incluidos linters y contratos de configuracion. Siguiente accion exacta: build
seleccionado y CTest completo de `simulacion_dron`.
Build integracion 3Q Simulacion: `simulacion_dron` 1/1 correcto, exit 0,
1.00 s. Siguiente accion: CTest completo del paquete con overlays.
CTest integracion 3Q Simulacion: 10/10 correctos, 100 %, 8.37 s, incluidos
contratos global map/fiducial y linters. Builds y suites seleccionadas quedan
verdes. Siguiente accion exacta 3Q: leer el workflow de simulacion, recuperar
el comando vigente equivalente a la prueba 213 con trayectoria tipica,
fiduciales reales, Gazebo/RViz2 y logs F3Q; registrar la prueba antes de lanzar.
Prueba 218 preparada para 3Q: YAML absoluto
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
launch `ros2 launch simulacion_dron multi_dron.launch.py` con Gazebo GUI=true,
RViz2 sparse=true, mission GUI=false, ambos grafos/navegadores y telemetria web
false, logs F3 terminal=true, visualizacion fiducial=true durante 5.0 s,
`rawdb_record_enabled=false`; fiduciales reales/spawn vigentes y GT fiducial
funcionalmente OFF por el perfil posterior a Fase 4. Startup 20 s, timeout
900 s, post-scenario 30 s, un retry Gazebo y monitor de recursos. Criterio:
runner 17/17 y todos los goals correctos; optimizaciones 3Q que terminen en
commit seguro, HOLD adaptativo o rechazo justificado sin mover hard ni mantener
backpressure; continuidad de wrappers/RViz2 y revision visual final del usuario.
Siguiente accion exacta: ejecutar prueba 218 y registrar exit/success/log antes
de reducir o analizar.
Prueba 218 finalizada: primer startup murio antes del escenario y la herramienta
uso correctamente su unico retry; el segundo intento completo scenario exit 0,
`[SIM-DONE] success=true` y `[SIM-EXIT-CODE] 0`. Duracion operativa resumida
505 s; `guard_triggered=false`, minimo disponible 4900.6 MiB, servidor RSS max
289.6 MiB, grupo RSS max 2642.5 MiB, RViz max 225.0 MiB y web 0 MiB. Log
completo conservado en `codex/archivos_auxiliares/logs/prueba_218.log` y no
leido. Siguiente accion exacta: generar reducciones por runner/goals, decisiones
3Q/solver/commit, invariantes hard/backpressure, fiduciales reales/visualizacion
y errores; analizar solo artefactos reducidos.
Reduccion runner 218: bloque completo de 196 lineas; 17/17 pasos, 22/22 goals
con `success=true`, `SCENARIO-RUNNER-DONE success=true`, scenario/tool exit 0.
Siguiente accion: reducir exclusivamente marcadores 3Q de evidencia, grafo,
solver, validator, commit, prior y backpressure para clasificar cada intento.
Diagnostico reducido 3Q prueba 218: 14 intentos loop, cero commits. Dos fallan
builder por `loop_submap_interval_too_small`; dos convergen pero degradan
covisibilidad nativa; diez convergen a error final aprox. 0.018-0.050 m y
0.0025-0.0108 rad, pero validator rechaza por
`hard_corridor_displacement_exceeded`. En siete rechazos repetidos el exceso
final queda casi constante 0.0266-0.0268 m / 0.00533-0.00537 rad; otros quedan
0.0336-0.0438 m. Fiduciales absolutos si hacen commits, pero no son commits
loop. Backpressure se libera tras cada intento y no hay hard failure. Conclusion
provisional: 3Q NO CONSEGUIDA aun; la deadband se esta aplicando como limite
absoluto casi-hard sobre KFs intermedios y sigue cancelando las optimizaciones.
Siguiente accion exacta: inspeccionar formula builder/validator del corredor y
corregir su semantica para proteger hard exactos y estructura relativa sin
impedir el movimiento necesario de KFs internos; repetir tests y simulacion.
Auditoria formula corredor 218: `RefreshHardCorridorLocked()` guarda como
referencia la pose vigente tras cada commit fiducial; por ello el exceso inicial
de todos los KFs internos es cero. `OptimizationValidator` rechaza cualquier
movimiento posterior superior a la deadband 0.02 m / 0.00872665 rad, de modo
que trata de facto todo el tramo como casi-hard. Esto contradice el objetivo de
deformar el segmento manteniendo solo los fiduciales inmoviles. Aumentar la
deadband solo desplazaria el mismo fallo. Recomendacion: retirar la guarda
absoluta/deadband de commit para KFs internos y conservar el corredor como
senal de riesgo/apoyo adaptativo; proteger con hard exactos, estructura
temporal/covis/prior, coste mejorado y maximo seguro 0.25/0.15. Eliminar campos
y parametros muertos de deadband para no dejar logica obsoleta.
Autorizacion funcional 3Q: SUSPENDIDA por esta decision material detectada en
la prueba 218.
Dudas abiertas 3Q: confirmar sustitucion de la guarda absoluta del corredor
por las guardas estructurales recomendadas y autorizar repetir build/tests y
trayectoria tipica.
Siguiente accion exacta: esperar confirmacion del usuario; no editar codigo ni
repetir simulacion hasta cerrar esta decision.
Clarificacion posterior a prueba 218: el limite de 2 cm sobre KFs intermedios
fue una interpretacion incorrecta y se retirara. Solo los fiduciales hard son
inmoviles permanentemente. Los KFs normales pueden moverse segun CurrentLoop,
aristas temporales, covisibilidad y PriorLoop. Los KFs ya optimizados pueden
reajustarse, pero una nueva optimizacion se rechaza si pretende moverlos mas de
5 m o 20 grados respecto a su pose optimizada protegida. Cuando al menos tres
segmentos independientes tienen fusiones en al menos el 60 % de sus KFs, esos
segmentos se fijan temporalmente durante ese solve y actuan como autoridad; se
mueve el lado query. No se convierten en hard fiduciales permanentes.
Preparacion 3Q tras diagnostico 218: CERRADA.
Acuerdo cerrado 3Q: si.
Autorizacion funcional 3Q: PENDIENTE de una orden explicita para aplicar esta
correccion y repetir build, tests y trayectoria tipica.
Dudas abiertas 3Q: ninguna.
Siguiente accion exacta: tras autorizacion explicita, retirar la deadband
absoluta y su configuracion obsoleta, restaurar la guarda 5 m/20 grados solo
para KFs previamente optimizados, fijar temporalmente el consenso 3/60,
actualizar regresiones y repetir validacion completa y simulacion.
Autorizacion funcional 3Q tras prueba 218: CONCEDIDA el 2026-08-26 para
actualizar contrato y codigo con la politica aclarada, compilar, ejecutar tests
y repetir la trayectoria tipica completa.
Trabajo activo 3Q: retirar la guarda absoluta de 2 cm; implementar proteccion
5 m/20 grados solo respecto a poses previamente optimizadas; fijar de forma
temporal, no hard persistente, los segmentos de consenso 3/60; limpiar campos,
parametros y tests obsoletos; validar con builds, suites y nueva simulacion.
Dudas abiertas 3Q: ninguna.
Checkpoint de reanudacion tras compactacion 2026-08-26: releido fisicamente el
estado obligatorio y reconciliado con la ultima orden del usuario. La
autorizacion sigue CONCEDIDA y el alcance no cambia. El contrato 3Q ya refleja
la politica aclarada; en codigo ya se retiraron los campos de deadband, se
anadio la guarda de KFs previamente optimizados y el marcado temporal del
consenso, pero faltan cerrar la regresion 3/60, buscar restos obsoletos,
compilar, ejecutar suites y repetir la trayectoria tipica.
Siguiente accion exacta tras reanudacion: adaptar el test de consenso para un
query movil y tres segmentos soporte fijados solo durante el solve; despues
realizar auditoria estatica y compilar `orbslam3_multi`.
Auditoria previa al build de correccion 3Q: regresion de consenso adaptada a un
query movil y tres segmentos soporte; solo `ServerLoopGeometric` es elegible
para formar consenso, no las dependencias soft que tambien se representan como
`PriorLoop`. No quedan simbolos de deadband en codigo/configuracion, ambas
copias YAML son identicas y `git diff --check` es correcto.
Build preparado: ejecutar
`./codex/herramientas/build_selected_packages.sh orbslam3_multi` para validar
tipos, builder, validator y regresiones tras retirar 2 cm e introducir
5 m/20 grados y fixed temporal 3/60. Siguiente accion tras terminar: registrar
exit/paquetes/log antes de cualquier diagnostico o CTest.
Build correccion 3Q intento 1: no inicio colcon; exit 2 porque la herramienta
requiere `--group`. No es evidencia del codigo ni genero un fallo de paquete.
Correccion mecanica: ejecutar
`./codex/herramientas/build_selected_packages.sh --group servidor orbslam3_multi`
y registrar inmediatamente su resultado.
Build correccion 3Q intento 2: `orbslam3_multi` fallo, exit 2, 0/1 paquetes,
53.2 s. El unico error mostrado es mecanico en la nueva expectativa del test:
`RawKeyFrameId` no tiene miembro `submap_id`. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido directamente.
Siguiente accion exacta: usar el campo real de `RawKeyFrameId` en esa lambda,
repetir `git diff --check` y relanzar el mismo build.
Build correccion 3Q intento 3: `git diff --check` correcto, pero colcon no
inicio el paquete porque el sandbox impidio crear
`/home/chenfu/Gazebo/log/servidor/build_*` (`Read-only file system`). No es
evidencia del codigo. Siguiente accion exacta: repetir el mismo build con el
permiso operativo preaprobado para la herramienta de compilacion.
Build correccion 3Q intento 4: `orbslam3_multi` 1/1 correcto, exit 0, 15.9 s.
Log completo conservado en `codex/archivos_auxiliares/colcon_build.log` y no
leido. Siguiente accion exacta: ejecutar el CTest completo de
`/home/chenfu/Gazebo/build/servidor/orbslam3_multi` con overlays ROS/Servidor;
registrar el resultado antes de corregir o integrar otros paquetes.
CTest correccion 3Q `orbslam3_multi`: 9/9 targets correctos, 100 %, 29.09 s,
incluidos `test_fiducial_optimization` y `test_loop_pipeline`. Quedan verdes
las regresiones de movimiento intermedio >2 cm, guarda de revisitados
5 m/20 grados, consenso temporal 3/60, ventanas y soporte adaptativo.
Siguiente accion exacta: compilar `orbslam3_server` para validar carga de
parametros y telemetria F3Q; despues ejecutar sus tests completos.
Build integracion correccion 3Q Servidor: `orbslam3_server` 1/1 correcto,
exit 0, 20.8 s. Log completo conservado por la herramienta y no leido.
Siguiente accion exacta: CTest completo de
`/home/chenfu/Gazebo/build/servidor/orbslam3_server` con overlays cargados.
CTest integracion correccion 3Q Servidor: 12/12 targets correctos, 100 %,
6.59 s, incluidos contratos de configuracion y linters. Siguiente accion
exacta: compilar `simulacion_dron` para instalar y validar la copia YAML de la
nueva politica antes de ejecutar su CTest.
Build integracion correccion 3Q Simulacion: `simulacion_dron` 1/1 correcto,
exit 0, 0.97 s. Log completo conservado y no leido. Siguiente accion exacta:
CTest completo de `/home/chenfu/Gazebo/build/simulacion/simulacion_dron` con
los overlays Dron/Servidor/Simulacion cargados.
CTest integracion correccion 3Q Simulacion: 10/10 targets correctos, 100 %,
8.33 s, incluidos `global_map_config_contract`, fiduciales y linters. Builds y
suites seleccionadas quedan verdes. Siguiente accion exacta: releer el workflow
de simulacion, registrar la prueba 219 equivalente a 218 y ejecutarla con
trayectoria tipica, fiduciales reales, Gazebo/RViz2 y logs F3Q.
Prueba 219 preparada para la correccion 3Q: YAML absoluto
`/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
launch `ros2 launch simulacion_dron multi_dron.launch.py` con Gazebo GUI=true,
RViz2 sparse=true, mission GUI=false, grafos web/telemetria web=false, logs F3
terminal=true, visualizacion fiducial=true durante 5.0 s y
`rawdb_record_enabled=false`. Startup 20 s, post-scenario 30 s, timeout 900 s,
un retry Gazebo y monitor de recursos. Criterio: runner 17/17 y goals
correctos; commits loop cuando la propuesta sea valida; ningun rechazo por la
deadband retirada; hard inmoviles; revisitados dentro de 5 m/20 grados;
consenso temporal si aparece; sin backpressure persistente. Siguiente accion:
ejecutar 219 y registrar exit/success/ruta de log antes de reducir o analizar.
Prueba 219 finalizada: scenario exit 0, `[SIM-DONE] success=true` y
`[SIM-EXIT-CODE] 0`; duracion resumida 508 s, `guard_triggered=false`, minimo
disponible 4702.2 MiB, servidor RSS max 265.5 MiB, grupo RSS max 2596.0 MiB y
RViz max 226.6 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_219.log` y no leido. Siguiente accion
exacta: generar reducciones tematicas de runner/goals, decisiones y commits
F3Q, invariantes hard/revisitados/consenso, backpressure y errores; analizar
solo esos artefactos.
Analisis reducido prueba 219: runner 17/17 y 22/22 goals con exito. Hubo 31
inicios F3Q y 30 resultados: 22 commits loop y 8 descartes (2 ventana pequena,
2 sin mejora, 2 sobre umbral seguro, 1 coste global mayor y 1 degradacion de
covisibilidad nativa). Tres commits fusionaron landmarks post-opt. No aparece
la razon obsoleta de 2 cm, ni movimiento hard, violacion de revisitados,
`blocking_failure`, fatal o guarda de recursos. Cinco commits terminaron en
fusion post-opt. Maximo reajuste medido en KFs
ya optimizados: 1.404495 m / 0.181256 rad, bajo 5 m / 0.349066 rad.
Riesgo residual 219: los commits reencolan KFs movidos; `pending` alcanzo 51 y
quedo un solve iniciado al apagar tras la espera final. La funcionalidad de
optimizar loops queda demostrada, pero no se declara eliminada la presion de
cola/escalabilidad. No se cambia unilateralmente la politica de reencolado
porque no forma parte del mecanismo concreto acordado.
Siguiente accion exacta: actualizar docs de paquete e historial 3Q, conservar
218 y 219 como entradas separadas, ejecutar verificaciones documentales finales
y cerrar 3Q como PARCIAL por el riesgo de cola pendiente de decision posterior.
Revision visual del usuario sobre 219: la esquina inferior izquierda mirando
hacia +Y queda completamente corregida; las dos esquinas derechas corrigen peor,
posiblemente por varios cambios de `map_epoch`; el usuario vuelve a mencionar
una esquina inferior izquierda mejorada solo parcialmente, referencia espacial
que debe aclararse porque contradice la primera frase. Solicita correlacionar
estas zonas con optimizaciones canceladas, motivos y anchors por loop.
Siguiente accion exacta: generar reducciones tematicas de 219 para anchors,
epochs, decisiones F3O/F3Q y fiduciales; cruzar la cronologia con el recorrido
sin leer el log completo y actualizar la conclusion viva de la misma prueba.
Correlacion visual/log 219 completada: no hubo anchors loop ni reanchors; los
cinco anchors fueron fiduciales directos en `(2,0)`, `(1,1)`, `(2,1)`, `(1,2)`
y `(1,3)`. Los cambios `(2,1)`, `(1,2)` y `(1,3)` aparecen en tramos derechos,
donde temporal no cruza epochs. Tras el bloque inicial casi todos los solves
usan query `(2,1)`; `(1,2)` no inicia ninguno y `(1,3)` completa uno. Esto es
consistente con la mejora visual desigual, sin demostrar una causa unica.
Documentacion viva actualizada: contrato 3Q, resumen/historial/indice, docs de
componentes, contexto minimo, estado y ultima sesion. Conclusion 219/3Q:
PARCIAL; no queda implementacion funcional activa, pendiente aclaracion visual
de la esquina repetida y decision del usuario sobre cierre o mejora multi-epoch.
Verificacion final documental: `git diff --check` correcto; no quedan
afirmaciones vigentes del exceso de corredor antiguo en docs de componentes.
Los cambios paralelos de Fase 5 permanecen intactos y no se han mezclado con
la implementacion 3Q. Trabajo activo 3Q: no; espera respuesta del usuario sobre
la referencia de esquina y la decision de cierre o nueva preparacion.
Diagnostico conversacional posterior sobre anchors loop 219: el mecanismo no
esta deshabilitado. Requiere activar una constraint con apoyo independiente y
que su componente contenga ya autoridad world. `(1,1)` activo constraints con
2/2 y 3/2 antes de los primeros fiduciales, pero ambos lados estaban aun sin
anchor y `BuildAnchorCascade()` no podia sembrar world. `(2,1)` y `(1,2)`
recibieron geometria rechazada mientras estaban sin anchor y luego fiducial
directo. `(1,3)` acumulo como maximo 4/6 apoyos en una hipotesis riesgosa y no
activo constraint antes de su fiducial. `BuildAnchorCascade()` se invoca al
activar una constraint, no automaticamente al llegar luego un fiducial.
Trabajo activo: no; explicacion documental sincronizada, sin cambios de codigo.
Checkpoint de reanudacion 2026-08-26: contexto fisico releido y reconciliado
con la pregunta vigente. Confirmado en codigo que `BuildAnchorCascade()` recorre
la componente de `active_constraints_` y puede anclar en lote sus submapas no
anclados cuando ya existe autoridad world, pero solo se invoca desde una nueva
activacion de constraint. `ProcessFiducialObservation()` no dispara esa cascada:
si A y B conservan una constraint relativa sin world y A recibe despues un
fiducial, B permanece sin anchor hasta que otro loop compatible vuelva a activar
el procesamiento. Trabajo activo: no; no se ha modificado codigo.
Preparacion mejora conservadora 3Q: CERRADA. Acuerdo cerrado: si.
Autorizacion funcional: CONCEDIDA el 2026-08-26. Dudas abiertas: ninguna.
Objetivo: conservar el comportamiento estable de 3Q y corregir solo dos huecos:
disparar la cascada/reconciliacion de constraints activas al aparecer nueva
autoridad world y permitir recuperacion reciente con un unico loop cuando la
continuidad sea inequivoca. Fuera de esa banda se conserva el apoyo adaptativo
2/4/6; superar 2 m no impide anclar. Parametros iniciales YAML: 0.50 m,
0.15 rad y recorrido maximo 2.0 m. El anchor de un solo loop no habilita fusion,
scaffold ni propagacion amplia hasta segundo loop independiente o fiducial.
Exclusiones: no cambiar solver, validator, umbrales de fusion ni scheduler
general; la deduplicacion adicional de cola queda supeditada a nueva evidencia.
Prueba acordada: unitarias deterministas, builds/CTest afectados y repeticion
de la trayectoria con fiduciales reales equivalente a 219, comparando anchors,
optimizaciones, descartes, map_epoch, pending y revision visual humana.
Riesgos aceptados: anchor soft provisional mal asociado y trabajo adicional al
llegar world; se contienen mediante continuidad estricta, ausencia de
ambiguedad, commit atomico y sin propagacion/fusion provisional.
Plan: actualizar contrato, localizar simbolos y YAML; implementar y probar el
bloque minimo; compilar/CTest; ejecutar simulacion; reducir logs y documentar.
Siguiente accion exacta: leer resúmenes/MDs de componentes y localizar las
rutas de configuracion, commit fiducial, constraints y tests antes de editar.
Contrato 3Q actualizado con la politica acordada: recuperacion reciente en tres
bandas, parametros YAML 0.50 m/0.15 rad/2.0 m, constraint provisional sin
fusion/scaffold/propagacion, fallback 2/4/6 y cascada atomica al aparecer world.
Archivos criticos confirmados: `loop_pipeline.hpp/.cpp` para evidencia y grafo
de constraints; `sparse_global_backend.hpp/.cpp` para continuidad y commits;
`fiducial_types.hpp` y `global_map_server.cpp` para devolver/reencolar cascada;
las dos copias `config/global_map/loop_fusion.yaml`; tests de loop pipeline y
contratos de Servidor/Simulacion. No se tocara solver, validator, fusion ni cola
general. Bloque conservador 3Q implementado: contexto de perdida y cuatro
parametros YAML; via de un loop con continuidad estricta; constraint provisional
que bloquea fusion y cascada hasta confirmacion; promocion por segundo apoyo;
`BuildAnchorCascade()` acotado a componente y sin provisionales; commits
fiduciales directos/optimizados disparan cascada y devuelven endpoints para
reconciliacion normal. Servidor carga ambas copias YAML, reencola endpoints y
expone `[F3O-WORLD-CASCADE]`/`single_recovery`. Regresiones añadidas para
cascada sin tercer loop, recuperacion cercana 1/1 y fallback sobre 2 m. Solver,
validator, fusion y scheduler general no se han modificado. Siguiente accion
exacta: build aislado de `orbslam3_multi`; registrar resultado y corregir solo
errores mecanicos antes de ejecutar CTest.
Build mejora conservadora 3Q intento 1: `orbslam3_multi` 1/1 correcto, exit 0,
1 min 20 s. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido. Siguiente accion exacta:
ejecutar CTest completo de `orbslam3_multi` con overlays ROS/Servidor; registrar
resultado antes de cualquier correccion o build del servidor.
CTest mejora conservadora 3Q `orbslam3_multi`: 9/9 targets correctos, exit 0,
incluido `test_loop_pipeline` con cascada fiducial, anchor 1/1 cercano y fallback
adaptativo sobre 2 m. No hubo regresiones en raw, pose store, optimizacion,
scalability ni fusion. Siguiente accion exacta: compilar `orbslam3_server` para
validar carga de parametros, telemetria y reencolado de reconciliacion.
Build mejora conservadora 3Q `orbslam3_server`: 1/1 correcto, exit 0, 21.3 s;
log completo conservado y no leido. Siguiente accion exacta: ejecutar CTest
completo de `orbslam3_server` con overlays y registrar el resultado.
CTest mejora conservadora 3Q `orbslam3_server`: 12/12 correctos, exit 0;
configuracion, backpressure, tests funcionales y linters pasan. Siguiente accion
exacta: compilar `simulacion_dron` para instalar/validar la replica YAML y luego
ejecutar su CTest completo antes de preparar la simulacion larga.
Build mejora conservadora 3Q `simulacion_dron`: 1/1 correcto, exit 0, 0.71 s;
replica YAML instalada. Siguiente accion exacta: ejecutar CTest completo de
`simulacion_dron` con los tres overlays y registrar el resultado.
CTest mejora conservadora 3Q `simulacion_dron`: 10/10 correctos, exit 0;
`global_map_config_contract`, fiduciales y linters pasan. Builds y suites quedan
verdes: `orbslam3_multi` 9/9, `orbslam3_server` 12/12 y Simulacion 10/10.
Prueba 220 preparada como repeticion exacta de 219: YAML
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
launch `ros2 launch simulacion_dron multi_dron.launch.py` con Gazebo GUI,
RViz2 sparse, mission GUI/web desactivados, logs F3 terminal, visualizacion
fiducial 5 s y raw record off. Startup 20 s, post-scenario 30 s, timeout 900 s,
un retry Gazebo y monitor de recursos. Criterio: 17/17 pasos, 22/22 goals,
success/exit 0; hard inmoviles; sin regresion de commits loop; medir cascadas,
single recovery, anchors, descartes, epochs y pending. La revision visual final
corresponde al usuario. Siguiente accion exacta: ejecutar prueba 220 y registrar
resultado/ruta del log antes de reducir o analizar.
Prueba 220 finalizada: primer arranque Gazebo murio temprano y el retry previsto
arranco correctamente; escenario exit 0, `[SIM-DONE] success=true` y
`[SIM-EXIT-CODE] 0`. Segundo intento: 505 s, `guard_triggered=false`, minimo
MemAvailable 4443.3 MiB, servidor RSS max 273.7 MiB, grupo RSS max 2634.9 MiB
y RViz max 225.9 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_220.log` y no leido. Siguiente accion
exacta: generar reducciones separadas para runner/goals, single recovery y
cascadas, anchors/epochs, F3Q solves/commits/descartes, pending/invariantes y
errores; analizar exclusivamente esos artefactos.
Revision visual humana prueba 220: resultado general excelente; unico problema
observado, una optimizacion/movimiento de KFs sin sentido que deterioro algo la
esquina superior derecha mirando hacia +Y. Esta observacion sustituye cualquier
cierre visual provisional y debe incorporarse a la entrada de la misma prueba.
Trabajo activo: diagnosticar esa correccion cruzando el paso por la esquina con
solves F3Q, submapas/epochs, ventanas, aristas y magnitudes; no cambiar codigo
hasta aislar causa. Siguiente accion exacta: reducir/leer cronologia F3Q del
intervalo entre pasos 7-13 y luego ampliar solo los patrones que falten.
Checkpoint de reanudacion 2026-08-26: archivo releido fisicamente tras la
compactacion y estado reconciliado con la ultima peticion. La prueba 220 no se
repite ni se modifica codigo: se compararan el commit fiducial `task=6`, el
commit loop `task=1000000005590` y las correcciones fiduciales posteriores
usando solo reducciones tematicas, para atribuir la deformacion de la esquina
superior derecha y actualizar la conclusion viva de esa misma prueba.
Diagnostico visual 220 cerrado: causa principal con alta confianza en
`task=1000000005590`, reencolada por `anchor_revision_changed` tras anclar
`(1,2)`. Fue la unica optimizacion loop que salto de ventanas habituales de
49-75 KFs a 3 submapas/296 KFs y movio 277. Los candidatos consecutivos
65/66/67 dieron error world casi `0/1.0118/0 m`; la seleccion incorporo las
tres regiones compatibles no fusionables como `CurrentLoop`, aunque solo la
constraint central discrepaba. Esto no demuestra que la pose del 66 fuese la
unica incorrecta: puede ser la medida regional o una deformacion distribuida.
El validator acepto porque el deterioro estructural maximo
`0.288825 m/0.041285 rad` y el reajuste de optimizados
`0.326020 m/0.054135 rad` quedan bajo 2 m temporal, 1 m covisible y 5 m/20
grados. La fusion posterior stale no revirtio el commit de poses. `task=6`
respondio en cambio a un error fiducial absoluto real de 1.183 m y el dron 2 ya
tenia control coherente en fiducial 1; no presenta la firma aislada 0/1/0.
Conclusion prueba 220 revisada por decision del usuario: `A REVISAR`. Resultado
global excelente y defecto residual aislado aceptado sin otra correccion ahora.
Trabajo funcional activo: no. Continuar pipeline; reabrir 3Q solo si reaparece
el mismo fallo. Punto de reentrada conservado: mantener las tres medidas como
`CurrentLoop` sobre la ventana completa y corregir enforcement, porque
`OptimizationManager` declara `Converged` tras 160 iteraciones aunque no alcance
`0.05 m/0.03 rad`, mientras el validator admite `0.25 m/0.15 rad`, mejora OR y
degradacion estructural local amplia. No asumir que el 66 sea la unica pose
incorrecta ni cambiar cascada, recuperacion 1/1 o fiduciales.
Peticion de cierre vigente: crear un commit unico con el trabajo pendiente
coherente de 3Q y la reconciliacion documental 5A ya realizada, y publicarlo en
GitHub. Siguiente accion exacta: verificar diff/status/branch/remoto, ejecutar
`git diff --check`, incluir los archivos nuevos de historial Fase 5, crear el
commit y hacer push sin reescribir historia ni revertir cambios.
Verificacion previa al commit: rama `main`, HEAD y `origin/main` en `a44b8b8`;
remoto `origin` correcto; `git diff --check` sin errores. El lote contiene 49
archivos: runtime/tests/config/docs 3Q y reconciliacion documental 5A, incluidos
sus tres historiales nuevos. No contiene logs de prueba. Siguiente accion
exacta: stage completo del lote coherente, commit unico descriptivo y push
normal a `origin/main`.
Cierre Git 2026-08-26: commit principal `b29d7a6`
(`Mejora optimizacion 3Q y prepara fase 5`) creado y publicado correctamente en
`origin/main`, avance `a44b8b8..b29d7a6`, sin force. 3Q queda `A REVISAR` por
decision del usuario y solo se reabrira si reaparece el fallo de enforcement de
la ventana 0/1/0 documentado en prueba 220. Trabajo activo: no. Siguiente accion
exacta: ninguna; continuar con el pipeline cuando el usuario lo solicite.
revisar restos obsoletos/contratos de configuracion y compilar
`orbslam3_server`; despues ejecutar sus tests antes de Simulacion.
Build 3Q PriorLoop/progresion: `orbslam3_multi` 1/1 correcto, exit 0, 19.4 s;
log completo conservado y no leido. Siguiente accion: CTest completo con overlay.
CTest `orbslam3_multi` intento 4: 8/9 targets, 9/12 casos de pipeline. Ya pasan
anclaje adaptativo, HOLD repetido y asimetria; quedan tres expectativas legacy:
rigidez exacta del hijo durante reanchor (el nuevo solver reparte correccion),
diagnostico de continuidad sin detalle de razones y prohibicion global de toda
reevaluacion tras aceptar una pareja (debe omitirse la pareja, no candidatos
nuevos). Siguiente accion: ajustar esas expectativas al contrato vigente y
añadir detalle estable para diagnosticar el caso de perdida reciente.
Build 3Q ajuste contratos pipeline: `orbslam3_multi` 1/1 correcto, exit 0,
5.03 s; log completo conservado y no leido. Siguiente accion: ejecutar primero
`test_loop_pipeline` para obtener la razon exacta del caso de perdida reciente.
Test dirigido pipeline: 2/3 correctos; anclaje adaptativo y optimizacion/fusion
pasan. El caso de perdida queda en `waiting_second_independent_query` para sus
ocho tareas y no alcanza la guarda de continuidad porque mezcla dos propiedades
en una sola regresion. Correccion de test: fijar soporte 2/2/2 solo en ese caso
para aislar y verificar la continuidad espacial; la politica 2/4/6 permanece
cubierta por los tests adaptativos separados y por los defaults runtime.
Regresion de ventana anadida: dos submapas con hard en KF 0/4 y loop en KF 7
deben producir exactamente un intervalo `[4,7]` por submapa y ocho KFs totales;
la arista server incidente en KF 5 entra sin recuperar el tramo anterior.
Siguiente accion exacta 3Q: rebuild y CTest completo de `orbslam3_multi`.
Siguiente accion exacta Fase 5: preparar conversadamente 5B como primera
subfase funcional. Antes de 5C/5D verificar el cierre de 3Q y el HEAD vigente.
Cualquier implementacion funcional requiere preparación y autorización propias.
```

## Estado vivo

```text
Estado: Fase 2 CONSEGUIDA; Fase 3 CONSEGUIDA; 4A-4F CONSEGUIDAS
Objetivo vigente: revisar y repetir la trayectoria tipica tras convertir los
giros que cruzan/permanecen en ±180 grados a comandos yaw relativos
Preparacion de repeticion post-4E+4F: CERRADA
Acuerdo de repeticion post-4E+4F cerrado: si
Autorizacion funcional de repeticion post-4E+4F: CONCEDIDA el 2026-08-25
Prueba propuesta: trayectoria tipica completa con Gazebo/RViz2, ambos grafos
web apagados y debug visual fiducial activo durante 5 s
Dudas abiertas: ninguna; confirmados tambien los dos giros de salida desde
+180 a -90 y desde -180 a +90, usando respectivamente +90 y -90 relativos
Preparacion del bloque 4E+4F: CERRADA
Acuerdo del bloque 4E+4F cerrado: si
Autorizacion funcional de 4E+4F: CONCEDIDA y consumida el 2026-08-25
Prueba acordada 4E+4F: unitarias/componentes, builds selectivos y trayectoria
tipica completa con Gazebo, RViz2 y ambos grafos web activos; ventanas
fiduciales desactivadas
Preparacion del bloque 4C+4D: CERRADA
Acuerdo del bloque 4C+4D cerrado: si
Autorizacion funcional de 4C+4D: CONCEDIDA para visualizador separado y prueba 207
Preparacion del bloque 4A+4B: CERRADA
Acuerdo del bloque 4A+4B cerrado: si
Autorizacion funcional de 4A+4B: CONCEDIDA y consumida
Bloques acordados: 4A+4B; 4C+4D; 4E+4F; 4G+4H; 4I
Primer bloque propuesto: 4A+4B
Prueba acordada 4C+4D: unitarias/componentes; trayectoria tipica revisada con
Gazebo y RViz2; pipeline_flow apagado y system_architecture activo; smoke corto
posterior con system_architecture apagado; revision visual humana del debug
fiducial por el usuario, sin transferir imagenes por chat
Dudas abiertas 4E+4F: ninguna; modulo/modo ESP32-CAM aplazado a 4I
Debug de prueba 4E+4F: pipeline_flow=true; system_architecture=true;
debug_architecture_telemetry=true en la comprobacion live 211; Gazebo
GUI=true; RViz2=true; ventanas fiduciales=false
Plan activo: modificar las seis transiciones yaw en las copias auxiliar e
instalada, validar contrato, compilar `simulacion_dron` y repetir la trayectoria
tipica completa con el perfil visual acordado
Trabajo activo: si; repeticion visual post-4E+4F autorizada
Siguiente accion exacta: editar ambos YAML de trayectoria, manteniendo
posiciones absolutas y usando yaw relativo +90/0/+90 para dron 2 y
-90/0/-90 para dron 1 en las transiciones alrededor de ±180 grados.
Cambios de trayectoria aplicados en la copia auxiliar y el escenario de
`simulacion_dron`: seis comandos con `absoluto_yaw=false`; dron 2 usa
+90/0/+90 y dron 1 usa -90/0/-90. Posiciones, tiempos y demas comandos sin
cambios. Siguiente accion exacta: comprobar igualdad de copias, contrato YAML
y `git diff --check`; despues compilar solo `simulacion_dron`.
Validacion previa: copias de trayectoria identicas, exactamente seis
`absoluto_yaw=false`, contratos fiducial/config 19/19 y `git diff --check`
correctos. Paquete a compilar: solo `simulacion_dron`. Siguiente accion exacta:
ejecutar build selectivo y registrar el resultado antes de preparar la prueba.
Build de repeticion visual: `simulacion_dron` 1/1 correcto, exit 0. Log completo
de build conservado y no leido. Prueba siguiente: 212, YAML absoluto
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
Gazebo GUI y RViz2 activos; mission GUI, pipeline_flow, system_architecture,
ambos navegadores y telemetria arquitectonica apagados; visualizacion fiducial
activa con 5.0 s y logs F3 en terminal. Timeout 900 s, post-scenario 30 s y
monitor de recursos. Criterio: escenario completo sin giros largos observados
por el usuario, ventanas visibles/aisladas, wrappers y RViz2 continuos y
sincronizacion 4E+4F sin fallos graves. Siguiente accion exacta: ejecutar 212 y
registrar el resultado antes de reducir o interpretar el log.
Prueba 212: detenida por Codex tras confirmacion visual del usuario de que los
drones permanecian en fiducial 2. El runner completo goals del paso 3 y la
espera de 8 s, inicio el paso 5 pero no envio nuevos goals porque el servidor
habia activado `blocking_failure=true` tras
`commit_pose_store_hard_constraint_violation` en task 1000000000461; el mission
gate mantuvo backpressure activo. No es evidencia de fallo de yaw. Ejecucion
interrumpida con Ctrl-C, shell exit 130; cleanup informa `SIM-EXIT-CODE 0` pero
no hubo `SCENARIO-RUNNER-DONE`. Log completo en
`codex/archivos_auxiliares/logs/prueba_212.log`, no leido. Recursos estables,
guard false y minimo 5213.5 MiB. Autorizacion funcional de repeticion:
SUSPENDIDA ante el fallo duro no previsto. Siguiente accion exacta: reducir
solo la tarea 1000000000461 y sus constraints/anchors para precisar la causa;
no repetir ni alterar el gate/GT/backend sin nuevo acuerdo.
Diagnostico reducido 212: la tarea 1000000000461 era un `LoopTask`, no una
orden de trayectoria ni un batch visual. Confirmo tres regiones RANSAC para
query `(1,0,2)`, construyo un grafo de dos submapas/45 KFs/17 controles y el
solver redujo el error, pero el commit rechazo la propuesta porque violaba una
hard constraint ya existente. La salvaguarda evito escritura, pero el servidor
clasifico el rechazo como `F3L-HARD-FAILURE`, fijo `blocking_failure=true` y
mantuvo el mission gate activo. El runner completo paso 3 y espera 4; en paso 5
no envio goals y finalmente marco failed al recibir Ctrl-C. Los seis giros yaw
modificados aun no se alcanzaron. Causa confirmada: loop incompatible con hard
fiducial + politica de fallo bloqueante; no visualizador, yaw, action server,
recursos ni backpressure por cola. Dudas abiertas: decidir entre corregir la
clasificacion/causa del fallo antes de repetir, o ejecutar una prueba
explicitamente solo visual que ignore el mission gate aceptando ese riesgo.
Acuerdo correctivo posterior a prueba 212: CERRADO; el usuario autoriza eliminar
por completo `secondary_blocking_failure_` y cualquier contribucion persistente
equivalente al backpressure. Los fallos duros conservan log y metricas, pero al
terminar una tarea rechazada o fallida se libera el mission gate; una corrupcion
severa se diagnosticara deteniendo manualmente la simulacion. Autorizacion
funcional: CONCEDIDA el 2026-08-25. Prueba acordada: repetir la trayectoria
tipica completa con Gazebo/RViz2, ambos grafos web apagados y visualizacion de
tags durante 5 s. Dudas abiertas: ninguna. Plan: retirar flag, condicion y
telemetria asociada; mantener `hard_failure` como resultado observable; ajustar
tests y documentacion; compilar `orbslam3_server` y `simulacion_dron`; ejecutar
la nueva prueba y analizar solo logs reducidos. Siguiente accion exacta: leer
docs y tests del servidor, aplicar el cambio minimo y su regresion.
Cambio correctivo aplicado: eliminado `secondary_blocking_failure_`, su escritura
en fallos duros, su condicion en `UpdateBackpressure()` y su campo de log. Los
fallos conservan contador/evento y `[F3L-HARD-FAILURE ... action=continue]`.
Anadido contrato pytest del servidor; resultado directo 2/2 y `diff --check`
correcto. Siguiente accion exacta: compilar `orbslam3_server` y registrar el
resultado antes de ejecutar CTest.
Build `orbslam3_server` intento 1: no iniciado, exit 2 de la herramienta porque
se uso por error `--group Servidor` en vez de `--group servidor`. Correccion
operativa mecanica: repetir con el nombre de grupo valido.
Build `orbslam3_server` intento 2: exit 0, 1/1 paquete correcto en 25.8 s.
Log completo de build conservado y no leido. Siguiente accion exacta: ejecutar
CTest del servidor con el overlay cargado; si pasa, actualizar docs de paquete
y preparar la simulacion visual acordada.
CTest servidor intento 1: no iniciado, exit 2 porque `--log-base` se coloco
despues del subcomando `test`. Correccion mecanica: repetir situando la opcion
global antes de `test`.
CTest servidor intento 2: no iniciado funcionalmente, exit 1 por sandbox
read-only al crear `/home/chenfu/Gazebo/log/servidor/test_*`. Repetir la misma
orden con permiso escalado; no hay evidencia de fallo del paquete.
CTest servidor intento 3: permiso automatico agotado antes de iniciar; reintento
escalado posterior correcto. Suite `orbslam3_server` 12/12 targets, 61 tests,
0 errores y 0 fallos (9 skipped). Documentacion de paquete sincronizada con la
politica sin latch persistente. No se recompila `simulacion_dron`: launch y YAML
instalados no cambiaron desde 212. Siguiente accion exacta: registrar y ejecutar
prueba 213 con la trayectoria tipica, Gazebo/RViz2 activos, ambos grafos web
apagados, debug fiducial 5 s y logs F3 visibles; timeout 900 s.
Prueba 213 preparada: YAML absoluto
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
`launch_gazebo_gui=true`, `launch_mission_gui=false`,
`debug_sparse_global_rviz=true`, ambos grafos/navegadores y telemetria web
apagados, `debug_fase3_logs_terminal=true`,
`debug_fiducial_visualization=true`, duracion 5.0 s. Timeout 900 s,
post-scenario 30 s y monitor de recursos. Criterio: runner completo; un hard
failure, si aparece, debe ir seguido de liberacion del backpressure y nuevos
goals; wrappers/RViz2 continuos y revision visual humana de yaw/ventanas.
Siguiente accion exacta: ejecutar 213 y registrar el resultado antes de reducir.
Prueba 213 finalizada: scenario exit 0, `[SIM-DONE] success=true` y
`[SIM-EXIT-CODE] 0`; duracion total 504 s. Recursos estables,
`guard_triggered=false`, minimo disponible 4996.2 MiB, servidor RSS maximo
269.0 MiB y grupo RSS maximo 2623.7 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_213.log` y no leido. Siguiente accion
exacta: reducir por runner/goals, backpressure, optimizacion/hard failures,
visualizadores, procesos y errores; analizar exclusivamente artefactos reducidos.
Analisis 213: 17/17 pasos, 22/22 goals `success=true`, scenario/tool exit 0.
Varias propuestas loop fueron rechazadas, pero cada optimizacion termino con
`optimization_active=false`, publico backpressure false y permitio continuar;
cero `[F3L-HARD-FAILURE]` reales. Visualizacion: 2 READY, 74 PUB y 74 SHOW,
cierres por timeout, sin muerte de wrappers, visualizadores o RViz2. Gazebo 255
solo en cleanup posterior a `SIM-DONE`. Conclusion tecnica: CONSEGUIDA;
pendiente unicamente la valoracion visual humana del usuario sobre los giros y
las ventanas. Documentacion de paquete, historial 4B, resumen de Fase 4,
contexto minimo, estado resumen y ultima sesion sincronizados. Trabajo activo: no; siguiente
accion exacta: recibir la observacion visual del usuario y actualizar la
conclusion viva si aporta matices.
Verificacion final: `git diff --check` correcto; runtime del servidor sin
referencias a `secondary_blocking_failure_` ni campo `blocking_failure=`. El
diff acotado conserva `secondary_hard_failed_` solo como metrica diagnostica.
Revision visual posterior 213: el usuario da por concluidas las subfases 4A-4F,
pero observa que practicamente no hubo correccion loop util y permanecieron
derivas. Analisis reducido: 15 intentos 3Q, seis commits tempranos con ventanas
30-69 KFs y nueve rechazos posteriores con ventanas 288-313 KFs; siete por
`hard_corridor_displacement_exceeded` y dos por
`prior_loop_structure_degraded`. Varias propuestas rechazadas reducian mucho el
error loop, pero creaban 0.000416-0.130115 m de exceso nuevo de corredor; varias
eran ambiguas, hasta 29 competidores y apoyo no independiente. Hipotesis breve:
combinacion de seleccion ambigua/ventanas grandes y validator de corredor
posiblemente sobrerrestrictivo. Prueba 213 marcada `A REVISAR DE NUEVO EN 3Q`
en contrato e historial 3Q; Fase 4 no se reabre. Trabajo activo: no. Siguiente
accion exacta: preparar 4G+4H cuando lo solicite el usuario; al retomar 3Q,
reproducir 213 y correlacionar derivas visibles con cada propuesta/rechazo.
El contexto de reentrada en `subfase_3Q.md` se ha condensado por peticion del
usuario: sintoma, 6 commits/9 rechazos, causa posible y accion futura; el detalle
permanece en `historial_3Q_RESUMEN.md`.
Preparacion 4G+4H: EN_DEBATE. Acuerdo cerrado: no. Autorizacion funcional:
PENDIENTE. Objetivo provisional: interpretar en Servidor los batches 4F como
objetos fiduciales, aplicar rango/fusion/seleccion y conectar un primary visual
al `FiducialAnchorManager`, primero con GT fiducial funcional OFF y retirandolo
solo tras pasar la cadena. Prueba provisional: unitarias de semantica y backend,
builds/CTests selectivos y trayectoria integral con dos drones/tres objetos,
Gazebo/RViz2 y observabilidad a acordar. Dudas abiertas: semantica visual de
`fiducial_visit_id`; thresholds multicara; formula de calidad/pesos; retencion
acotada para secundarios/no aptos de Fase 6; estrategia controlada para validar
residual alto/rollback; momento exacto de retirar GT y perfil visual de prueba.
Trabajo activo: no, solo preparacion conversada. Siguiente accion exacta:
explicar alcance/riesgos y cerrar estas decisiones con el usuario antes de
modificar codigo, launch, YAML, configuracion, tests o ejecutar builds.
Acuerdos parciales posteriores 4G+4H: todos los KFs con primary fiducial deben
llegar a `FiducialAnchorManager`; el manager compara cada uno con sus umbrales
y descarta funcionalmente los coherentes sin crear optimizacion. La fusion usa
como peso base `max(quality_score, epsilon) * sqrt(tag_area/max_tag_area)`.
Servidor conserva una FIFO diagnostica separada con los ultimos 50 KFs
interpretados por dron, incluidos secundarios/no aptos, para uso futuro en
Fase 6; documentar este contrato en `subfase_6S.md` cuando termine el debate.
El wrapper/ORB-SLAM3 envia cada `camera_T_tag` y sus metricas; no recibe
semantica de objeto ni calcula `camera_T_object` o medias SE(3). El Servidor
resuelve `object_T_tag`, calcula distancia/rango y fusiona, aceptando el pequeno
trafico adicional. No se forzara residual alto ni rollback en 4H: se exigira
regresion equivalente a la ruta GT y se diagnosticaran fallos si aparecen.
Prueba integral aceptada: trayectoria tipica con Gazebo/RViz2, primero GT
fiducial compilado pero OFF; tras pasar la cadena visual se elimina por completo
y se repite, con smoke corto separado para grafos web. Dudas pendientes:
delimitar `fiducial_visit_id` sin omitir KFs ni promover cada KF como control;
cerrar fusion multicara robusta porque `quality_score` solo refleja reproyeccion
individual y no garantiza que una cara incoherente tenga score bajo; confirmar
carga C++ de `fiducial_objects.yaml` y thresholds/configuracion asociados.
Preparacion 4G+4H cerrada tras la ultima revision conversada. Acuerdo cerrado:
si. Autorizacion funcional: PENDIENTE. `fiducial_visit_id` agrupa por
`(drone_id,map_epoch,object_id)` mientras no haya un hueco superior al parametro
configurable inicial de 2.0 s; todos los KFs primary se envian igualmente al
manager y se evaluan. Fusion multicara acordada: conservar todas las caras,
peso base por calidad/area y reponderacion robusta por residual geometrico;
thresholds iniciales configurables de 0.15 m y 15 grados; si no queda solucion
estable, no hay anchor funcional para ese object/KF. El rango inclusivo [1,5]
m se comprueba por cada `camera_T_tag`; cualquiera fuera vuelve no apto al
object completo de ese KF, conservando todas las observaciones. Servidor carga
directamente `fiducial_objects.yaml` con `yaml-cpp`; el servicio 4D no expone
semantica de objetos y wrapper/ORB-SLAM3 no la calculan. FIFO reciente: 50 KFs
interpretados por dron, documentada tambien en `subfase_6S.md`. Prueba larga:
Gazebo, RViz2 y ventanas de tags 5 s; grafos web en smoke separado. Exclusiones:
sin GT fiducial funcional, sin cambios de scoring 3R, sin forzar residual alto,
sin reimplementar manager/optimizer ni modificar poses raw. Riesgos aceptados:
una mayoria geometrica erronea con score alto no es desambiguable sin prior;
se rechaza una fusion sin consenso estable. Criterio: tres objetos y ambos
drones recorren la cadena visual exacta, anchors/revisitas se comportan como la
ruta GT, no hay contaminacion ni bloqueo, y tras el primer pase se elimina GT
fiducial por completo y se repite. Dudas abiertas: ninguna. Trabajo activo: no.
Siguiente accion exacta: esperar autorizacion explicita del usuario para editar
los contratos 4G/4H y despues implementar, compilar y probar el bloque acordado.
Contratos 4G+4H actualizados por autorizacion documental del usuario: el MD
corto 4G y su detalle fijan ownership en Servidor, carga `yaml-cpp`, rango por
tag, fusion robusta ponderada, thresholds 0.15 m/15 grados y FIFO de 50 KFs;
4H fija visita visual configurable de 2.0 s sin omitir ningun KF, perfil de
prueba Gazebo/RViz2/ventanas 5 s, smoke web separado y ausencia de residual o
rollback forzado. `subfase_6S.md` conserva el uso futuro de la FIFO solo como
hint reciente. `git diff --check` correcto. Autorizacion funcional de codigo,
build y simulacion: PENDIENTE. Trabajo activo: no. Siguiente accion exacta:
esperar orden explicita para implementar 4G+4H segun los contratos cerrados.
Ejecucion 4G+4H autorizada explicitamente por el usuario el 2026-08-25.
Preparacion: CERRADA. Acuerdo cerrado: si. Autorizacion funcional: CONCEDIDA.
Dudas abiertas: ninguna. Plan activo: localizar integracion/config; implementar
interpretador robusto, FIFO y visitas; unitarias/builds; regresion visual con GT
fiducial OFF; retirar por completo la ruta GT y repetir; smoke web; documentar.
Prueba acordada: trayectoria tipica con 2 drones/3 objetos, Gazebo/RViz2 y
ventanas de tags 5 s, grafos web apagados; smoke web corto separado. Criterio:
anchors/revisitas visuales por KFs exactos, sin GT funcional, sin doble primary,
contaminacion ni bloqueo; trafico medido; segunda regresion tras borrar GT.
Trabajo activo: si. Siguiente accion exacta: leer docs vigentes de los paquetes
afectados y localizar con `rg` los simbolos de configuracion, handoff 4F,
FiducialAnchorManager y retirada GT antes de editar codigo.
Localizacion 4G+4H completada. Cambio funcional inicial aplicado con GT legacy
aun compilado pero `fiducial_sim_enabled=false`: nuevo
`FiducialObjectInterpreter` carga YAML con yaml-cpp, deriva object_T_tag por
cara, agrupa, aplica rango por tag, fusion robusta SE(3), primary determinista,
visitas 2 s y FIFO 50; handoff 4F crea `visual_fiducial` y llama al manager para
cada KF primary. Anade metricas de batches/tags/bytes/rates, parametros launch
y pruebas unitarias. Siguiente accion exacta: ejecutar comprobaciones estaticas
y unitarias directas disponibles; corregir solo fallos mecanicos y despues
registrar el build selectivo de orbslam3_server/simulacion_dron.
Validacion estatica inicial 4G+4H: 31/31 pytest correctos, launch Python valido,
guarda workspace 15/15 y `git diff --check` correcto. La primera guarda detecto
solo `__pycache__` generados por los propios tests; se limpiaron y la repeticion
pasa. Build siguiente: `orbslam3_server` en grupo servidor para compilar nuevo
interpretador, yaml-cpp, integracion y gtest. Si pasa, ejecutar CTest del
servidor y despues compilar `simulacion_dron`. Siguiente accion exacta:
`./codex/herramientas/build_selected_packages.sh --group servidor orbslam3_server`.
Build 4G+4H `orbslam3_server` intento 1: exit 2, 0/1 paquetes; log completo en
`codex/archivos_auxiliares/colcon_build.log`, no leido. La salida del comando
apunta a asignaciones Eigen con initializer-list restantes en el nuevo test;
se reducira el log para confirmar el primer error real antes de corregir.
Siguiente accion exacta: ejecutar `reduce_build_log.sh` y leer solo el reducido.
Diagnostico reducido build intento 1 confirmado: tres lineas del nuevo gtest
asignaban initializer-list a `Eigen::Isometry3d::translation()` (lineas
136/156/186); no hay error de produccion ni dependencia. Correccion mecanica:
usar `Eigen::Vector3d` en las tres. Siguiente accion exacta: repetir el mismo
build selectivo de `orbslam3_server`.
Build 4G+4H `orbslam3_server` intento 2: exit 0, 1/1 paquete correcto en 4.6 s.
Log completo de build conservado y no leido. Siguiente accion exacta: ejecutar
CTest del paquete con overlay servidor para validar el nuevo gtest y regresion
completa; despues registrar resultado antes de compilar Simulacion.
CTest servidor 4G+4H intento 1: las pruebas funcionales pasan, incluidos los
5/5 casos nuevos de `FiducialObjectInterpreter`; unico fallo `uncrustify` por
tres diferencias mecanicas de formato en el nuevo `.cpp`. Resultado global:
13 targets, 1 fallo de lint, sin error funcional. Siguiente accion exacta:
aplicar exactamente el formato exigido, sustituir texto no ASCII del archivo
nuevo y repetir CTest antes de compilar `simulacion_dron`.
CTest servidor 4G+4H intento 2: 73 tests, 0 errores y 0 fallos (12 skipped);
los 5 casos del interpretador, integracion previa y linters pasan. Siguiente
accion exacta: revisar el diff/integracion arquitectonica pendiente y compilar
`simulacion_dron` antes de preparar la primera regresion larga con GT OFF.
Build 4G+4H `simulacion_dron` intento 1: exit 0, 1/1 paquete correcto. Perfiles,
launch y contratos quedan instalados. Siguiente accion exacta: ejecutar CTest
de Simulacion y las guardas estaticas; si pasan, fijar y lanzar la primera
regresion larga con GT fiducial compilado pero funcionalmente OFF.
CTest Simulacion 4G+4H: 85 tests, 0 errores y 0 fallos (8 skipped). Guarda
workspace intento 1: 14/15 por `__pycache__` generados por CTest; retirados
solo esos caches y el temporal pytest. Guarda intento 2: 15/15. `git diff
--check`: correcto. Siguiente accion exacta: preparar la prueba larga 214 con
trayectoria tipica, Gazebo/RViz2 y ventanas 5 s, grafos web apagados y GT
fiducial funcionalmente OFF; registrar el perfil antes de ejecutarla.
Prueba 214 preparada: YAML absoluto
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`;
Gazebo GUI=true, RViz2=true, mission GUI=false, ambos grafos/navegadores y
telemetria web=false, logs F3 terminal=true, visualizacion fiducial=true durante
5.0 s. El perfil instalado mantiene `fiducial_sim_enabled=false`. Timeout 900 s,
espera posterior 30 s y monitor de recursos. Criterio: escenario completo;
batches visuales exactos interpretados en ambos drones, primary/visitas y
manager sin contaminacion, bloqueo ni uso GT; revision visual humana de RViz2
y ventanas. Siguiente accion exacta: ejecutar 214 y registrar el resultado
antes de reducir o interpretar su log completo.
Prueba 214 finalizada: scenario exit 0, `[SIM-DONE] success=true` y exit global
0; duracion 498 s. Recursos estables, `guard_triggered=false`, minimo disponible
5435.8 MiB, servidor RSS maximo 261.6 MiB y grupo RSS maximo 2601.6 MiB; web
0 MiB como se acordo. Log completo en
`codex/archivos_auxiliares/logs/prueba_214.log`, conservado y no leido.
Siguiente accion exacta: reducir 214 por runner, batches, interpretacion de
objetos/tags, primary/visitas, manager/anchors, GT, visualizadores,
backpressure y errores; leer solo el artefacto reducido.
Analisis parcial reducido 214: cadena visual activa sin GT (`F3E-FID-CONFIG
enabled=false`), 80 batches recibidos/interpretados, 94 tags, 80 primary, cero
no-primary/rejected, tres object_id cubiertos y ambos drones aportando anchors;
manager procesa las 80 observaciones, crea cinco anchors de submapa y el
escenario termina sin bloqueo persistente. Defecto encontrado: batches del
mismo tramo llegan ligeramente fuera de orden y `AssignVisitLocked` abre una
visita nueva siempre que `stamp < last_stamp`; aparecen IDs distintos dentro
de una misma aproximacion. La prueba no se cierra aun. Correccion dentro del
contrato: representar intervalos recientes por clave y asociar cada timestamp
al intervalo situado a <=2 s, ampliando sus extremos; test explicito de llegada
desordenada, rebuild/CTest y nueva regresion antes de retirar GT.
Correccion de visitas aplicada: cada clave conserva intervalos temporales; una
llegada fuera de orden se asigna al intervalo mas cercano si queda dentro de
`fiducial_visual_visit_gap_sec`, ampliando min/max, y solo una separacion real
crea otro ID. Unit test ampliado con orden 10.0, 11.0, 10.5 y 13.1 s. Rebuild
`orbslam3_server`: exit 0, 1/1 paquete correcto en 35.5 s; resultado confirmado
en log reducido porque la primera espera de herramienta vencio antes del cierre.
Siguiente accion exacta: CTest completo del servidor y, si pasa, repetir la
regresion visual con el mismo perfil antes de retirar GT.
CTest servidor tras intervalos: 73 tests, 0 errores y 0 fallos (12 skipped).
Prueba siguiente: 215, repeticion exacta de 214 con mismo YAML, Gazebo/RViz2,
ventanas 5 s, grafos web OFF, recursos monitorizados y GT fiducial funcional
OFF. Criterio adicional: los KFs desordenados de una aproximacion conservan el
mismo visit_id; ninguna visita nueva salvo hueco temporal superior a 2 s.
Siguiente accion exacta: ejecutar 215 y registrar su cierre antes del reducido.
Prueba 215 finalizada: scenario exit 0, `[SIM-DONE] success=true`, exit global
0 y duracion 500 s. Recursos estables, guard false, minimo 5400.5 MiB,
servidor RSS maximo 287.0 MiB y web 0 MiB. Log completo en
`codex/archivos_auxiliares/logs/prueba_215.log`, conservado y no leido.
Siguiente accion exacta: generar reducido tematico 215 y verificar cadena,
cobertura, visitas por timestamps, manager, ausencia GT y fallos antes de
autorizar internamente la retirada fisica del camino GT.
Analisis 215: 73/73 batches interpretados como primary, 83 tags, cero rechazos,
objetos 1/2/3 cubiertos y ambos drones activos; las llegadas desordenadas del
mismo arrival conservan visit_id. Cuatro anchors creados. Derivas reales de
4.64-23.39 m activan el manager; varias tareas existentes terminan
`loop_submap_window_too_small`, con `action=continue` y sin bloqueo persistente.
Se clasifica como evidencia del backend 3H/3L, no fallo de 4G; la cadena visual
entrega exactamente los datos que antes recibia la ruta GT.
Retirada GT fiducial aplicada: eliminados subscription/callback/buffer,
conversion body-camera, parametros/radios legacy, test/header, arista web y
runtime edge. Replays aceptan solo `source=visual_fiducial`; referencias de
tests backend migradas. El servidor ya no carga `calibration_dron.yaml`.
Validacion estatica tras una correccion contractual mecanica: 26/26 pytest.
Siguiente accion exacta: builds/CTests de backend, servidor y Simulacion;
despues repetir trayectoria sin codigo GT y ejecutar smoke web separado.
Builds post-retirada GT: `orbslam3_multi`, `orbslam3_server` y
`simulacion_dron`, 1/1 correctos cada uno. CTest Servidor agregado: 150 tests,
0 errores/fallos (10 skipped); el proceso supero la primera espera, confirmado
por `colcon test-result`. CTest Simulacion: 85 tests, 0 errores/fallos
(8 skipped). Siguiente accion exacta: limpiar solo caches de CTest, guarda y
diff; despues preparar la prueba 216, misma regresion larga ya sin ruta GT.
Guardas post-CTest: dos intentos iniciales detectaron solo caches Python
generados; retirados exclusivamente esos artefactos. Resultado final 15/15 y
`git diff --check` correcto. Prueba 216 preparada con el mismo YAML/perfil de
215: Gazebo/RViz2, ventanas 5 s, grafos web OFF, timeout 900 s, post 30 s y
recursos; el binario/config ya no contienen ruta GT fiducial. Criterio:
resultado equivalente a 215, solo source visual, cobertura y visitas validas,
sin suscripcion GT ni bloqueo persistente. Siguiente accion exacta: ejecutar
216 y registrar el cierre antes de reducir.
Prueba 216 finalizada: scenario exit 0, `[SIM-DONE] success=true`, exit global
0 y duracion 504 s. Guard false, minimo 4617.4 MiB, servidor RSS maximo
272.7 MiB y web 0 MiB. Cleanup necesito SIGTERM/SIGKILL del launch despues de
`SIM-DONE`; se analizara si fue solo cierre de Gazebo/procesos auxiliares. Log
completo en `codex/archivos_auxiliares/logs/prueba_216.log`, no leido.
Siguiente accion exacta: reducido tematico y comprobacion de cadena visual,
ausencia GT, procesos y errores antes del smoke web.
Analisis reducido 216: 52 batches sincronizados y 52 primary; cobertura
object 1/2/3, ambos drones en objetos 1/2 y dron 2 en objeto 3; 52 PUB/SHOW,
cero no-primary/unknown y ninguna referencia runtime GT ni hard failure. Un
anchor inicial y revisitas/optimizaciones visuales observables. Scenario y
herramienta correctos. Gazebo exit 255 y SIGKILL ocurrieron solo durante cleanup
posterior a `SIM-DONE`; el destructor del servidor no alcanzo a imprimir sus
contadores finales, pero toda la mision y espera post finalizaron. Conclusion
funcional de regresion sin GT: CONSEGUIDA. Siguiente accion exacta: smoke 217
con `tray_prueba_211.yaml`, ambos grafos web y telemetria activos, ventanas OFF;
verificar edges visuales actuales y ausencia de la arista GT eliminada.
Smoke 217 finalizado: scenario/tool exit 0, `SIM-DONE success=true`, duracion
105 s, guard false, minimo 5172.7 MiB y web RSS maximo 83.0 MiB. Cierre limpio
con SIGINT. Log completo conservado y no leido. Siguiente accion exacta:
reducir por READY/health/actividad/edges fiduciales/errores; si pasa, actualizar
documentacion de paquete e historiales 4G/4H y ejecutar guardas finales.
Archivos criticos 4E+4F localizados: dos copias de
`orbslam3_msgs/{msg,CMakeLists.txt}`; wrapper
`orbslam3_ros2/src/stereo/stereo-slam-node.{hpp,cpp}`; servidor
`orbslam3_multi/{raw_map_types.hpp,raw_map_database.hpp,raw_map_database.cpp,
sparse_global_backend.hpp, sparse_global_backend.cpp}`;
`orbslam3_server/src/global_map_server.cpp`; perfiles
`config/global_map/runtime.yaml`; grafos y tests de `simulacion_dron`.
APIs confirmadas: worker 4D conserva identidad/metricas; `RawInsertResult`
expone `new_keyframe_ids`; `RawMapDatabase` tiene mutex unico y lookup raw;
`SparseGlobalBackend` devuelve `PrimaryBackendResult`; `WorkerLoop` recibe el
commit raw antes de derivados. No hay dudas funcionales nuevas.
Siguiente accion exacta: aplicar interfaces/publicacion 4E y sidecar de
sincronizacion 4F con pruebas unitarias, antes de cualquier build.
Bloque de cambios 4E+4F aplicado: mensajes replicados; publisher reliable
KeepLast(32) de observaciones validas/ordenadas; helper temporal comun y
camera_T_tag; sidecar pending en RawMapDatabase con capacidad 10 por dron,
FIFO sin TTL, digest, first_arrival_id, validacion y matches en RawInsertResult;
subscription/handoff/logs del servidor; parametro YAML replicado; aristas live
en ambos grafos y pruebas raw/contractuales. Prueba estatica inicial: 38/39;
unico fallo por expectativa textual antigua de EnqueueFiducialJob, corregida
mecanicamente. Replicas msg identicas y git diff --check correcto.
Siguiente accion exacta: repetir contratos y compilar primero orbslam3_msgs
Servidor y Dron de forma aislada.
Contratos 4E+4F tras correccion: 39/39 correctos. Build interfaces Servidor
intento 1: exit 0, orbslam3_msgs 1/1 correcto. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido. Siguiente accion
exacta: compilar orbslam3_msgs de Dron de forma aislada.
Build interfaces Dron intento 1: exit 0, orbslam3_msgs 1/1 correcto. Log
completo conservado y no leido. Siguiente accion exacta: compilar el paquete
wrapper `orbslam3` para validar publisher, transform y mensajes generados.
Build wrapper intento 1: exit 0, orbslam3 1/1 correcto; solo warnings legacy
de ORB-SLAM3/Eigen y cv_bridge. Log completo conservado y no leido. Siguiente
accion exacta: compilar `orbslam3_multi` para validar sidecar y sus unit tests.
Build backend intento 1: exit 0, orbslam3_multi 1/1 correcto. Log completo
conservado y no leido. Siguiente accion exacta: ejecutar CTest del backend;
despues compilar orbslam3_server si las unitarias pasan.
CTest backend intento 1: 0/9 ejecutables arrancan, exit 8; todos fallan antes
de los tests por `liborbslam3_msgs__rosidl_generator_c.so` no localizada. Es
un problema de entorno: el CTest directo no cargo el overlay Servidor, no hay
evidencia de fallo funcional. Siguiente accion exacta: repetir CTest tras
source de `/home/chenfu/Gazebo/install/servidor/setup.bash`.
CTest backend intento 2: 9/9 correctos, incluidas las nuevas pruebas de ambos
ordenes de llegada, FIFO por dron, duplicate/conflict y no reactivacion por
update. Siguiente accion exacta: compilar `orbslam3_server` para validar
subscription, configuracion, logs y handoff fuera del mutex.
Build servidor intento 1: exit 0, orbslam3_server 1/1 correcto. Log completo
conservado y no leido. Siguiente accion exacta: ejecutar CTest del servidor
con el overlay cargado y despues compilar Simulacion.
CTest servidor intento 1: 10/11 correctos; unico fallo `uncrustify` por dos
espacios de sangria en la declaracion de `fiducial_subscriptions_`. Correccion
mecanica aplicada, sin cambio funcional. Siguiente accion exacta: repetir
CTest del servidor.
CTest servidor intento 2: 11/11 correctos. Siguiente accion exacta: compilar
`simulacion_dron` para instalar perfiles, grafos y contratos 4E+4F.
Build Simulacion intento 1: exit 0, simulacion_dron 1/1 correcto. Log completo
conservado y no leido. Siguiente accion exacta: ejecutar CTest de Simulacion y
revisar la guarda de replicas/configuracion antes de preparar la simulacion.
CTest Simulacion: 10/10 correctos; pasan contratos de pipeline_flow,
system_architecture, configuracion global y fiduciales, ademas de linters.
Siguiente accion exacta: ejecutar CTest del wrapper y una verificacion estatica
final; despues preparar el YAML/comando de simulacion tipica acordada.
CTest wrapper: 1/1 correcto. Verificacion final previa: mensajes y perfiles
Servidor/Simulacion identicos; `git diff --check` correcto. Prueba siguiente:
209, usando
`codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`.
Launch: `multi_dron.launch.py` con Gazebo GUI, RViz2, pipeline_flow y
system_architecture activos; ambos navegadores activos; mission GUI y ventanas
fiduciales desactivadas; telemetria arquitectonica derivada del master activa.
Timeout 900 s, post-wait 30 s y monitor de recursos. Patrones previstos:
`FID-BATCH-PUB|FID-SYNC|FID-SERVER-SUBSCRIBED|F3C-RAW-COMMIT|F3P-FLOW-WEB-READY|SYSTEM-ARCHITECTURE|SCENARIO-RUNNER|SIM-|ERROR|FATAL|process has died`.
Siguiente accion exacta: ejecutar simulacion 209 y registrar su resultado antes
de reducir o analizar el log.
Simulacion 209: finalizada con exit 1; `scenario_runner_node` termino con codigo
1 antes de recorrer el escenario. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_209.log` y no leido. Recursos estables,
`guard_triggered=false`, minimo disponible 5897.8 MiB. Siguiente accion exacta:
reducir el log 209 con marcadores de scenario/YAML/launch y diagnosticar el
primer error real antes de decidir si la repeticion conserva el acuerdo.
Diagnostico reducido 209: `[SCENARIO-RUNNER-ERROR] bad file` porque el YAML se
paso como ruta relativa y el scenario se ejecuta desde otro cwd. Launch,
wrappers, configuracion fiducial, RViz2 y ambos bridges arrancaron; cierre
limpio salvo Gazebo 255 conocido. Correccion operativa mecanica: nueva prueba
210 con la misma trayectoria mediante ruta absoluta y
`debug_fase3_logs_terminal=true` para observar los marcadores FID del servidor.
No cambia comportamiento ni criterio acordado. Timeout 900 s, post-wait 30 s,
GUI/RViz2/ambos grafos activos y debug visual fiducial apagado. Siguiente
accion exacta: ejecutar prueba 210 y registrar resultado antes del analisis.
Simulacion 210: finalizada con scenario exit 0, `[SIM-DONE] success=true` y
`SIM-EXIT-CODE 0`. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_210.log` y no leido. Recursos estables,
`guard_triggered=false`, minimo disponible 5085.9 MiB. Siguiente accion exacta:
reducir el log por scenario, publisher, sincronizacion, grafos, procesos y
errores; despues contrastar conteos y criterios 4E+4F.
Analisis reducido 210: trayectoria tipica completa, 17 pasos y ambos drones
correctos; 68 `FID-BATCH-PUB`, 68 altas pending y 68
`FID-SYNC-MATCHED`, sin evictions, duplicates, conflicts ni rejects; 33 batches
del dron 1, 35 del dron 2, 11 multitag y pico pending observado 7/10. Todos los
batches publicados quedaron consumidos. Pipeline flow arranco live con
23 nodos/41 aristas; system architecture arranco visible pero en `mode=static`
porque la ejecucion omitio `debug_architecture_telemetry:=true`. No es un fallo
de codigo, sino una opcion incompleta de la prueba, y queda pendiente verificar
la arista wrapper-servidor en live.
Checkpoint de reanudacion 2026-08-25: archivo releido fisicamente tras
compactacion y reconciliado con la orden `Sigue con las subfases`. Autorizacion
4E+4F permanece CONCEDIDA, dudas funcionales ninguna. Siguiente accion exacta:
leer el workflow local de simulacion/documentacion, preparar un smoke corto que
produzca un fiducial y ejecutarlo con ambos grafos activos y
`debug_architecture_telemetry:=true`; despues documentar y cerrar 4E+4F.
Prueba siguiente 211: YAML
`codex/archivos_auxiliares/trayectorias/tray_prueba_211.yaml`, formado por
readiness, espera de tracking, el primer tramo tipico de ambos drones hasta
`(0,-10)` y espera final de 8 s. Launch con Gazebo GUI, RViz2, pipeline_flow,
system_architecture, ambos navegadores y `debug_architecture_telemetry=true`;
ventanas fiduciales desactivadas y logs F3 en terminal. Timeout 360 s,
post-wait 5 s y monitor de recursos. Criterio enfocado: scenario correcto,
`SYSTEM-ARCH-READY mode=live`, batches y matches reales sin errores graves.
Siguiente accion exacta: ejecutar prueba 211 y registrar su resultado antes de
reducir o analizar el log.
Simulacion 211: finalizada con scenario exit 0, `[SIM-DONE] success=true` y
`SIM-EXIT-CODE 0`. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_211.log` y no leido. Recursos estables,
`guard_triggered=false`, minimo disponible 5532.3 MiB; duracion observada 90 s.
Siguiente accion exacta: reducir 211 por readiness de grafos, batches,
sincronizacion, scenario, procesos y errores; analizar solo el reducido y
decidir el cierre tecnico 4E+4F.
Analisis reducido 211: `SYSTEM-ARCH-READY mode=live` y pipeline flow live con
23 nodos/41 aristas; servidor suscrito a ambos topics con capacidad 10. El
smoke produjo 18 batches reales (8 dron 1, 10 dron 2), todos recibidos pending
y todos matched; pico pending 5/10, sin evictions, duplicates, conflicts ni
rejects. El conteo bruto 19 de `rg -c` incluia la propia linea de patrones del
reductor; la comparacion `(drone,kf)` confirma 18 publicaciones y 18 matches.
Scenario completo y cierre global correctos; solo Gazebo 255 durante cleanup,
incidencia conocida no funcional. Con 210 completa y 211 live, criterios
tecnicos 4E+4F satisfechos. Siguiente accion exacta: actualizar docs de paquetes,
historiales 4E/4F, estado/pipeline y ultima sesion; ejecutar guardas finales y
cerrar trabajo activo.
Cierre documental 4E+4F completado. Docs de paquetes, contratos, estado,
pipeline e historiales 4E/4F sincronizados. Guardas finales: contratos 39/39,
arquitectura workspace 15/15, replicas de mensajes y runtime YAML exactas y
`git diff --check` correcto. Conclusion: 4E y 4F CONSEGUIDAS. Trabajo activo:
no. Siguiente accion solo tras nueva peticion: preparar conversacionalmente
4G+4H; ese bloque no tiene autorizacion funcional.
Archivos criticos 4C+4D: ORB_SLAM3/{System,Tracking};
orbslam3_ros2/{stereo_slam_node,FiducialDetector,CMake}; dos copias de
orbslam3_msgs; orbslam3_server/{fiducial_config_server,launch,config};
simulacion_dron/{multi_dron,system_architecture,tests}; perfil canonico en
orbslam3_server/config/fiducial_objects.yaml
Diseno localizado: spawner Python + readiness Bool reliable/transient-local;
scenario_runner espera `wait_for_bool`; SDF/texturas runtime desde YAML;
perfil semantico replicado y rendering exclusivo de Simulacion
Cambios completados: fiducial_objects replicado, rendering Gazebo, spawner
Python, readiness transient-local, `wait_for_bool`, trayectoria +-10,
fiduciales GT transitorios +-8.5, metadata system_architecture y prueba 201
Pruebas estaticas directas: 38/38 correctas; py_compile y git diff --check correctos
Paquete a compilar: simulacion_dron (unico con codigo/launch/tests modificados)
Build 4A+4B intento 1: exit 0; simulacion_dron 1/1 correcto
Log completo conservado: codex/archivos_auxiliares/colcon_build.log (no leido)
CTest intento 1: no ejecutado, exit 8; sandbox impidio escribir LastTest.log
fuera de src, sin evidencia de fallo funcional
CTest intento 2: 9/10 correctos; unico fallo flake8 por seis lineas >99 y dos
imports no usados en archivos nuevos; tests funcionales 4/4 correctos
Diagnostico: correccion mecanica de estilo, sin cambio funcional
Build 4A+4B intento 2 tras flake8: exit 0; simulacion_dron 1/1 correcto
Log completo: codex/archivos_auxiliares/colcon_build.log (no leido)
CTest intento 3: 10/10 correctos; fiducial_contract, config, arquitectura,
pipeline_flow y seis linters pasan
Guarda completa intento 1: 14/15; unico fallo paths por cuatro __pycache__
generados por CTest/py_compile dentro de simulacion_dron; sin fallo funcional
Guarda completa intento 2: 15/15, cero fallos
Verificacion de install: spawner/config Simulacion instalados; el nuevo perfil
canonico de Servidor requiere rebuild de orbslam3_server para instalarse
Build Servidor intento 1: exit 0; orbslam3_server 1/1 correcto
Log completo: codex/archivos_auxiliares/colcon_build.log (no leido)
CTest orbslam3_server: 10/10 correctos
Install verificado: perfiles Servidor/Simulacion identicos; spawner ejecutable
Prueba siguiente: 201
YAML: codex/archivos_auxiliares/trayectorias/tray_prueba_201.yaml
Launch: multi_dron con Gazebo GUI=true, RViz2=true, mission GUI=false,
pipeline_flow=false, system_architecture=false, navegadores/telemetria web=false
Timeout scenario: 900 s; espera visual final de escenario 30 s; post-wait 30 s
Patrones: FID-TEXTURE|FID-SDF|FID-SPAWN|SCENARIO-RUNNER|SIM-|RViz|ERROR|FATAL
Simulacion 201: finalizada; scenario exit 0, `SIM-DONE success=true` y
`SIM-EXIT-CODE 0`. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_201.log` (no leido).
Reducido 201: 15/15 texturas verificadas, 3/3 SDF, 3/3 spawn, readiness
recibido, 10/10 goals correctos y scenario success=true. RViz2 arranco y cerro
limpio; ambos grafos web y telemetria arquitectonica quedaron desactivados.
Incidencia: al SIGINT final, `fiducial_spawner.py` intenta `rclpy.shutdown()`
cuando el contexto ya esta cerrado y sale 1; no afecta al escenario pero debe
corregirse mecanicamente. Gazebo mantiene su exit 255 conocido tras cleanup.
Correccion de cierre aplicada: `ExternalShutdownException`/`KeyboardInterrupt`
son cierres normales y `rclpy.shutdown()` solo se llama con contexto activo.
Verificacion directa: py_compile correcto; contratos 38/38 correctos. Un intento
pytest previo no ejecuto tests por tres rutas mal nombradas, sin fallo funcional.
Rebuild tras correccion: simulacion_dron 1/1 correcto. CTest: 10/10 correcto.
Prueba siguiente: 202, smoke sin movimiento; espera readiness y 2 s.
YAML: codex/archivos_auxiliares/trayectorias/tray_prueba_202.yaml
Launch: Gazebo GUI=true, RViz2=true, ambos grafos/navegadores y telemetria
arquitectonica=false. Timeout 180 s; post-wait 0 s.
Patrones: FID-SPAWN|SCENARIO-RUNNER|SIM-|rviz2|Traceback|process has died
Simulacion 202: finalizada; scenario exit 0, `SIM-DONE success=true` y
`SIM-EXIT-CODE 0`. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_202.log` (no leido).
Reducido 202: 15 texturas, 3 SDF y 3 spawn correctos; readiness recibido;
spawner y RViz2 terminan limpiamente; sin traceback ni `FID-SPAWN-ERROR`.
Grafos web/telemetria no arrancan. Solo queda el exit 255 conocido de Gazebo
tras SIGINT. El usuario confirma que la revision visual de 201 fue perfecta.
Trayectoria tipica revisada: cuadrado ±10 con paradas en `(0,±10)` y
`(±10,0)`; copia auxiliar y escenario de paquete identicos; contrato 6/6.
Por decision del usuario no se simula esta variante ahora y se verificara con
4C+4D. Documentacion de paquete, estado e historial sincronizada. 4A y 4B
quedan CONSEGUIDAS.
Verificacion de cierre: `test_fiducial_contract.py` 6/6, `git diff --check`
correcto y guarda de workspace 15/15. No se ejecuto build ni simulacion tras
el ajuste de trayectoria, conforme a la peticion del usuario.
Verificacion final: `git diff --check` correcto; guarda de workspace 15/15;
check de longitud termina correctamente con avisos preexistentes y los detalles
largos 4A/4B ya disponen de contratos/resumen cortos.
Checkpoint de reanudacion tras compactacion: releido fisicamente este archivo y
reconciliado con la peticion vigente; se mantiene preparacion 4C+4D sin cambios
funcionales, builds ni simulaciones.
Hallazgo 4C: ya existen LastKeyFrameInfo/GetLastKeyFrameInfo y comparacion
before/after en GrabStereo, pero no evento explicito one-shot; la imagen del
wrapper tampoco es genericamente la imagen efectiva si System rectifica/resize.
Hallazgo 4D: no existe detector/config server; OpenCV 4.5.4 dispone de aruco y
highgui ya es dependencia; se requieren servicio replicado, nodo independiente,
worker acotado y metadata. El usuario acepta API minima 4C; detector SUBPIX,
IPPE_SQUARE y 3 px; rechazo por reproyeccion pero no por rango 1-5 m; score
lineal inicial; retry 1 s, timeout 2 s, cola 4/drop oldest y las dos pruebas.
Nuevo alcance solicitado: debug opcional por wrapper que muestre durante 5 s
el frame procesado con ROI y tag_id al detectar fiduciales, sin bloquear worker.
Acuerdo visual: una ventana por wrapper; el KF mas reciente reemplaza al actual
y reinicia el contador; todos los tags decodificados se muestran, aceptados en
verde y rechazados/desconocidos en rojo con motivo; duracion parametrizable con
default 5.0 s; cierre y fallo headless aislados del SLAM.
Los candidatos OpenCV no decodificados y sin tag_id quedan excluidos por acuerdo.
Ultima auditoria: el wrapper actual carga fx/fy/cx/cy/bf y dimensiones, pero no
un modelo completo de distorsion ni una calibracion reconciliada con rectificacion
o resize interno de System; 4D necesita imagen, K y distorsion efectivos unidos.
Decision cerrada: calibracion propiedad de Dron; System realiza una unica
rectificacion/resize y entrega en el recibo 4C evento, imagen, K, distorsion,
dimensiones y estado efectivos; el detector no vuelve a corregir. El servicio
fiducial no transporta calibracion. Una guarda impide doble rectificacion.
Documentacion preparatoria sincronizada: contratos cortos y detalles largos de
4C/4D reflejan preparacion cerrada, autorizacion pendiente, API/calibracion,
servicio/worker/detector, valores acordados, pruebas y debug visual completo.
Documentacion de ORB_SLAM3 y orbslam3_ros2 actualizada con el estado parcial.
Autorizacion recibida: el usuario ordena realizar modificaciones y pruebas el
2026-08-24 conforme a todo el acuerdo cerrado, sin ampliar el alcance.
Bloque 4C ORB aplicado: Tracking registra/limpia/consume evento one-shot en
StereoInitialization, CreateNewKeyFrame y resets; System::TrackStereo devuelve
recibo opcional con imagen/K/distorsion/dimensiones efectivas y expone guardas
de rectificacion/resize. Getter permanente eliminado. No cambia politica de KF.
Siguiente accion exacta: compilar ORB_SLAM3 nativo con `cmake --build ... -j4`
antes de adaptar el wrapper al nuevo contrato
Build ORB intento 1: no ejecutado; la ruta documental `src/dron/ORB_SLAM3/build`
no existe. Cache real localizada en `/home/chenfu/Gazebo/build/dron/ORB_SLAM3`.
Correccion mecanica: usar ese build dir aislado con paralelismo `-j4`.
Build ORB intento 2: detenido tras primer error real; LoopClosing requiere el
getter interno Tracking::GetLastKeyFrame en tres rutas IMU/merge. Correccion
mecanica: restaurar solo ese getter interno; System/wrapper siguen usando
exclusivamente el recibo one-shot. Repetir build nativo.
Build ORB intento 3: target completo `all` alcanzo y enlazo libORB_SLAM3 al 55%;
se detuvo deliberadamente al compilar ejemplos upstream no necesarios.
Verificacion target exacto `ORB_SLAM3`: exit 0, 100% construido.
Siguiente accion exacta: crear interfaces de configuracion identicas en las dos
copias orbslam3_msgs y preparar fiducial_config_server independiente
Checkpoint de reanudacion 2026-08-25: releido fisicamente tras compactacion y
reconciliado con la ultima orden `sigue`; autorizacion 4C+4D permanece CONCEDIDA,
sin dudas nuevas ni cambio de alcance.
Bloque interfaces/config server aplicado: las dos copias de orbslam3_msgs
incorporan FiducialTagConfig y GetFiducialConfig con definiciones identicas;
los perfiles Servidor/Simulacion incorporan SUBPIX, IPPE_SQUARE y umbral 3 px;
orbslam3_server incorpora el nodo independiente
`/global_mapping/get_fiducial_config`, validacion del perfil, logs y actividad
arquitectonica opcional, integrado en su launch. Pendiente validar/build.
Siguiente accion exacta: validar sintaxis y replicas; despues implementar el
detector, cliente/worker y debug visual no bloqueante en orbslam3_ros2.
Bloque detector/wrapper aplicado inicialmente: FiducialDetector usa
APRILTAG_36H11, SUBPIX e IPPE_SQUARE, conserva metricas y rechaza por geometria
o reproyeccion; StereoSlamNode consume exclusivamente StereoTrackingReceipt,
valida imagen/calibracion, solicita configuracion con retry 1 s/timeout 2 s,
encola hasta 4 con drop-oldest y ejecuta detector/GUI en hilos separados. La
GUI solo nace con flag activo y reemplaza el KF mostrado durante 5 s por defecto.
Validacion previa: replicas msg/srv identicas, Python de nodo/launch compila y
`git diff --check` correcto. Pendiente compilar para corregir API/ABI real.
Siguiente accion exacta: build aislado de orbslam3_msgs Servidor y Dron.
Build 4C+4D interfaces Servidor intento 1: exit 0; orbslam3_msgs 1/1 correcto,
incluyendo FiducialTagConfig y GetFiducialConfig.
Siguiente accion exacta: build aislado de orbslam3_msgs Dron.
Build 4C+4D interfaces Dron intento 1: exit 0; orbslam3_msgs 1/1 correcto.
Las dos replicas generan correctamente. Siguiente accion exacta: build aislado
del wrapper orbslam3 para validar detector, servicio e hilos contra APIs reales.
Build wrapper 4C+4D intento 1: exit 0; orbslam3 1/1 correcto. Solo advertencias
legacy de ORB-SLAM3/Eigen, sin errores nuevos.
Cableado aplicado: flags visuales atraviesan multi_dron -> generar_dron ->
orbslam_use; perfil fiducial de Simulacion llega al config server; grafo y
metadata incorporan arista runtime `fiducial_config_server_to_wrapper`.
Pruebas contractuales directas: fiducial+arquitectura 16/16 correctas.
Test directo parser intento 1: no recolectado; la shell directa no tenia
orbslam3_msgs instalado en PYTHONPATH. No es fallo funcional; repetir dentro
del build/CTest de orbslam3_server con entorno colcon.
Siguiente accion exacta: rebuild orbslam3 con gtest sintetico y ejecutar CTest.
Rebuild wrapper intento 2: exit 0; orbslam3 1/1 correcto con GTest instalado.
CTest wrapper intento 1: 1 test suite ejecutada, 1/3 casos correcto; unknown tag
se conserva rechazado, pero dos casos PnP fallan como invalid_pose_geometry.
Diagnostico: OpenCV produce soluciones IPPE con z positiva; la copia temporal
`convertTo(cv::Mat(cv::Vec3d))` no escribe de forma fiable en el Vec y deja la
traslacion a cero. Correccion mecanica: convertir a Mat persistente y extraer
sus tres escalares; construir Matx33d explicitamente. Rebuild y repetir CTest.
Rebuild wrapper intento 3 tras correccion IPPE: exit 0, 1/1 correcto.
CTest wrapper intento 2: suite 1/1 correcta; sus tres casos (aceptado con pose
finita, unknown rechazado y escala segun size_m) pasan.
Siguiente accion exacta: build y CTest de orbslam3_server con nodo/config test.
Build orbslam3_server intento 1: exit 0, 1/1 correcto.
CTest servidor intento 1: 10/11 correctos; unico fallo
test_fiducial_config_server no recolecta porque el entorno de ament pytest no
expone orbslam3_msgs Python. Linters y cuatro GTests existentes pasan.
Diagnostico/correccion mecanica: extraer `load_fiducial_config` a modulo Python
puro instalado junto al ejecutable; nodo y test importan la misma funcion sin
necesitar interfaces ROS durante la coleccion. Rebuild y repetir CTest.
Rebuild orbslam3_server intento 2: exit 0, 1/1 correcto.
CTest servidor intento 2: 11/11 correctos; parser canonico/empty/duplicados,
cuatro GTests previos y seis linters pasan.
Siguiente accion exacta: validar launch, build dron_individual y
simulacion_dron; ejecutar sus CTests/guardas.
Validacion sintactica de cuatro launch modificados: correcta.
Build dron_individual intento 1: exit 0, 1/1 correcto; launch con flags
fiduciales instalado. Siguiente accion exacta: build simulacion_dron.
Build simulacion_dron intento 1: exit 0, 1/1 correcto.
CTest simulacion intento 1: 9/10 correctos; unico fallo contractual porque la
lista cerrada de debug.yaml aun esperaba siete claves y detecta correctamente
las dos nuevas. Correccion mecanica: incluir visualizacion y duracion, y fijar
sus defaults false/5.0. Rebuild y repetir CTest.
Rebuild simulacion intento 2: exit 0. CTest intento 2: 9/10; la misma prueba
ya reconoce defaults nuevos pero conserva una segunda asercion legacy que exige
que todo valor, incluida la duracion 5.0, sea false. Ajustar esa asercion para
aplicarla solo a flags booleanos y repetir.
CTest simulacion intento 3: 10/10 correctos; cuatro contratos y seis linters.
Estado previo a simulacion: builds de interfaces Dron/Servidor, ORB target,
wrapper, server, dron_individual y simulacion correctos; CTests especificos
wrapper 1/1, server 11/11 y simulacion 10/10.
Siguiente accion exacta: preparar prueba tipica revisada 203 con Gazebo/RViz2,
pipeline_flow off, system_architecture on, telemetria on y debug fiducial on.
Prueba siguiente: 203, trayectoria tipica revisada completa.
YAML: codex/archivos_auxiliares/trayectorias/
prueba_tipica_rodeo_edificio_dos_fiduciales.yaml.
Launch: Gazebo GUI=true, RViz2=true, mission GUI=false, pipeline_flow=false,
system_architecture web/browser=true, architecture telemetry=true,
debug fiducial=true durante 5 s; timeout scenario 900 s, post-wait 30 s.
Patrones de reduccion previstos: FID-CONFIG|KF-EVENT|FID-TAG|FID-KF|FID-VISUAL|
SCENARIO-RUNNER|SIM-|ERROR|FATAL|Traceback|process has died.
Simulacion 203 intento 1: NO EJECUTADA funcionalmente; los tres arranques
(inicial + 2 reintentos) terminaron antes del healthcheck Gazebo y el script
salio 1. No se inicio scenario_runner. Recursos holgados, guard_triggered=false.
Log completo conservado en codex/archivos_auxiliares/logs/prueba_203.log y no
leido. Siguiente accion exacta: reducir 203 buscando excepcion de launch y
primer error causal; corregir mecanicamente y repetir como nueva prueba 204.
Diagnostico reducido 203: causa unica repetida en los tres intentos,
`fiducial_config_server.py` no encontrado como ejecutable libexec. El archivo
si esta instalado por symlink, pero la fuente tiene modo 0664 y symlink-install
lo deja no ejecutable. Correccion mecanica: chmod +x sobre la fuente, verificar
resolucion ROS y repetir como prueba 204 con la misma configuracion.
Permiso corregido y ejecutable instalado verificado con `test -x`.
Prueba siguiente: 204, repeticion integra de 203 (misma trayectoria, launch,
debug y criterios), conservando 203 como fallo de arranque independiente.
Simulacion 204: arranque Gazebo/launch conseguido, wrappers activos; el
scenario_runner termino inmediatamente con exit 1 y la prueba cerro con exit 1
sin ejecutar la trayectoria. Recursos correctos, guard_triggered=false.
Log completo conservado en codex/archivos_auxiliares/logs/prueba_204.log y no
leido. Siguiente accion exacta: reducir 204 por scenario/config/FID y localizar
el primer error causal antes de repetir con nuevo identificador.
Diagnostico reducido 204: scenario_file relativo no existe desde el cwd del
proceso (`bad file`); error de invocacion, no de codigo. Evidencia parcial
positiva: config server READY con 15 tags, ambos wrappers hacen request/READY,
KF inicial one-shot tiene timestamp_delta=0 y el primer KF preconfig se omite
sin buffering; todos los nodos relevantes cierran limpios.
Prueba siguiente: 205, misma prueba completa usando ruta YAML absoluta
`/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/
prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`.
Simulacion 205: interrumpida manualmente por peticion del usuario tras 214 s
(exit de sesion 130; cleanup del script registra SIM-EXIT-CODE 0). No alcanzo
conclusion de escenario y no debe contarse como prueba completa. Recursos
estables, guard_triggered=false. Revision visual del usuario: RViz2 no mostraba
nada; solicita usar el log parcial actual y parar.
Log completo conservado en codex/archivos_auxiliares/logs/prueba_205.log y no
leido. Siguiente accion exacta: reducir 205, diagnosticar deteccion 4C/4D y el
vacio de RViz2; no lanzar otra simulacion sin nueva conversacion/autorizacion.
Diagnostico 205: 4C/4D detectan tag 202 en ambos drones con z=1.498/1.444 m,
error=0.252/0.206 px y quality=0.916/0.931. Justo al activar HighGUI, ambos
wrappers mueren exit 127 por cargar `/snap/core20/.../libpthread.so.0` y faltar
`__libc_pthread_init`; por eso no vuelven a publicar deltas. Sin deltas que
incorporen esos KFs, el adaptador GT rechaza todos los KFs anteriores como
outside_fiducial_radius, no crea anchors y RViz queda vacio. Correccion
mecanica: sanear rutas `/snap/` del entorno de los nodos ORB en launch, como ya
hace RViz2. No repetir simulacion por peticion vigente del usuario.
Correccion Snap aplicada en orbslam_use.launch.py para mono/stereo; launch
compila, contrato directo 7/7, rebuild dron_individual 1/1 y CTest final de
simulacion_dron 10/10. No se relanza simulacion. Estado tecnico: 4C validada en
runtime; 4D detector/config/worker validados y dos poses reales correctas;
debug visual corregido pero pendiente de verificacion runtime tras el crash.
Siguiente accion exacta: actualizar docs de paquetes e historial vivo 4C/4D,
guardas finales y cerrar como PARCIAL sin ocultar pruebas 203-205.
Checkpoint de reanudacion 2026-08-25 tras compactacion: releido fisicamente
este archivo y reconciliado con la ultima peticion. La simulacion 205 permanece
detenida por orden del usuario y no se lanzara otra. Se usara exclusivamente el
log reducido actual para cerrar el diagnostico, completar la guarda headless,
verificarla con build/tests locales y documentar 4C CONSEGUIDA y 4D PARCIAL.
Guarda headless aplicada: `StereoSlamNode` solo crea el hilo HighGUI si existe
`DISPLAY` o `WAYLAND_DISPLAY`; si no, registra el motivo y mantiene SLAM activo.
Build siguiente: `./codex/herramientas/build_selected_packages.sh --group dron orbslam3`.
Motivo: validar la ultima correccion defensiva del wrapper sin nueva simulacion.
Build wrapper final: exit 0; `orbslam3` 1/1 correcto. El stderr contiene solo
warnings legacy de cv_bridge/ORB-SLAM3/Eigen. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido. Siguiente accion:
ejecutar CTest de `build/dron/orbslam3` y despues cerrar documentacion.
CTest wrapper final: 1/1 suite correcta; los tres casos sinteticos del detector
siguen pasando tras la guarda headless. No se ejecuta otra simulacion. Siguiente
accion exacta: sincronizar contratos, historiales, estado y docs de paquetes;
despues ejecutar guardas finales de texto/workspace.
Documentacion 4C/4D sincronizada en contratos, historiales, estado y paquetes.
Guarda workspace intento 1: 13/16 checks correctos; falla solo porque la guarda
clasifica todo `debug_*` como booleano y no contempla la duracion numerica 5.0,
y por `__pycache__` generados por tests. Correccion mecanica: declarar esa clave
numerica en la guarda y retirar exclusivamente caches generadas; repetir.
Guarda workspace intento 2: 15/15 checks correctos; replicas de interfaces y
config, ownership, paths, visualizers y docs coherentes. Siguiente accion:
auditoria textual final, `git diff --check` y cierre operativo sin trabajo activo.
Verificacion final: guarda workspace 15/15, replicas msg/srv exactas,
`fiducial_config_server.py` ejecutable y `git diff --check` correcto. Conclusion
agregada: 4C CONSEGUIDA; 4D PARCIAL porque la correccion HighGUI/Snap/headless
compila y pasa CTest, pero no se verifico en otra simulacion por orden del
usuario. Trabajo activo: no. Siguiente accion: ninguna hasta nueva peticion.
Nueva autorizacion 2026-08-25: el usuario ordena realizar la correccion ya
aplicada y ejecutar su prueba. No hay cambios funcionales adicionales.
Prueba siguiente: 206, repeticion completa de la trayectoria tipica con ruta
YAML absoluta. Gazebo GUI=true, RViz2=true, mission GUI=false, ambos grafos web
y telemetria=false, debug fiducial=true durante 5.0 s. Timeout 900 s,
post-wait 30 s y monitor de recursos activo. Criterios: ventanas visibles y
autocierre/reemplazo, tags anotados, wrappers vivos despues de HighGUI y deltas
posteriores; RViz2 se observa pero 4D aun no publica detecciones visuales al
Servidor. Siguiente accion exacta: ejecutar `run_simulation.sh --prueba 206`.
Simulacion 206: interrumpida manualmente por peticion del usuario tras 331 s;
sesion exit 130 y cleanup `SIM-EXIT-CODE 0`. No alcanzo conclusion del escenario.
Recursos estables, `guard_triggered=false`, minimo disponible 5802.5 MiB. El
usuario confirma que la ventana fiducial funciono al principio, pero RViz2 dejo
de publicar puntos y solicita comprobar si el wrapper volvio a cerrarse. Log
completo conservado en `codex/archivos_auxiliares/logs/prueba_206.log` y no
leido. Siguiente accion exacta: reducir 206 por HighGUI/crash, lifecycle de
wrappers, detecciones, KFs, deltas y publicacion del servidor antes de concluir.
Diagnostico revisado 206: las ventanas funcionaron y los wrappers publicaron
deltas despues de las primeras detecciones. A los 176-178 s ambos `stereo`
terminaron con exit `-9`; no aparece `libpthread`, OOM ni guarda. El usuario
aclara que el escritorio ofrecio `cerrar ventana/esperar` y eligio cerrar. Esa
accion fuerza `SIGKILL` sobre el proceso propietario de HighGUI y explica el
cese de puntos en RViz2. La separacion por hilo no aisla un cierre forzado del
proceso. Propuesta material pendiente de acuerdo: visualizador ROS separado que
reciba la imagen anotada opt-in; si se fuerza su cierre, ORB-SLAM3 sigue vivo.
Acuerdo y autorizacion 2026-08-25: el usuario acepta esa propuesta y ordena
aplicarla y probarla. Contrato cerrado: topic relativo por dron
`orbslam/fiducial_debug/image`, QoS latest-only y solo con debug activo; el
wrapper anota/publica pero no ejecuta HighGUI; un ejecutable
`fiducial_visualizer` separado posee ventana y temporizador de 5 s. Un cierre
normal o forzado del visualizador no puede terminar `stereo`. Prueba 207 repite
la trayectoria tipica con Gazebo/RViz2, grafos off y debug visual on.
Siguiente accion exacta: editar wrapper, nuevo ejecutable, CMake, launch y tests.
Correccion visual separada aplicada: `stereo` ya no contiene HighGUI ni hilo de
ventana; publica imagen anotada latest-only en
`orbslam/fiducial_debug/image`. El nuevo ejecutable
`fiducial_visualizer` posee HighGUI, procesa eventos en su main executor y
cierra por timeout o boton. Launch lo crea solo con debug activo y mantiene el
saneamiento Snap. Contrato actualizado para impedir regresion. `git diff
--check` correcto; pytest directo no disponible en la shell (`command not
found`), se validara por CTest instalado. Build siguiente:
`./codex/herramientas/build_selected_packages.sh --group dron orbslam3`.
Build orbslam3 visual separado intento 1: exit 2, 0/1 paquetes. Log completo
conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido. Primer
error aparente: API Iron `rclcpp::Time` no ofrece `to_msg()`. Siguiente accion:
reducir el log, confirmar el primer error real y aplicar solo la conversion
mecanica de timestamp compatible.
Reducido confirma un unico error funcional: `rclcpp::Time::to_msg` inexistente;
el resto son warnings legacy. Corregido con campos `sec/nanosec` explicitos y
añadido include estandar del nuevo nodo. Siguiente accion: repetir build
aislado de `orbslam3`.
Checkpoint de reanudacion tras compactacion 2026-08-25: releido fisicamente
este archivo y reconciliado con la ultima orden. La autorizacion para el
visualizador separado y la prueba 207 permanece CONCEDIDA, sin dudas nuevas.
Build orbslam3 visual separado intento 2: exit 0, `orbslam3` 1/1 correcto;
solo warnings legacy de cv_bridge/ORB-SLAM3/Eigen. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido. Siguiente accion
exacta: ejecutar CTest de `build/dron/orbslam3`.
CTest wrapper visual intento 1: no ejecutado; la revision automatica del
permiso para escribir en el directorio de build agoto su plazo. No aporta
evidencia funcional. Siguiente accion: unico reintento autorizado de CTest.
CTest wrapper visual intento 2: 1/1 correcto; los tres casos sinteticos del
detector pasan tras aislar HighGUI. Siguiente accion exacta: build aislado de
`dron_individual` para instalar y validar el launch del visualizador separado.
Build dron_individual visual separado intento 1: exit 0, 1/1 correcto; launch
instalado. Log completo conservado en `codex/archivos_auxiliares/colcon_build.log`
y no leido. Siguiente accion exacta: build de `simulacion_dron` para instalar
el contrato actualizado y ejecutar su CTest.
Build simulacion_dron visual separado intento 1: exit 0, 1/1 correcto. Log
completo conservado en `codex/archivos_auxiliares/colcon_build.log` y no leido.
Siguiente accion exacta: ejecutar CTest de `build/simulacion/simulacion_dron`.
CTest simulacion visual separado intento 1: 10/10 correctos, incluido
`fiducial_contract` y los seis linters. Siguiente accion exacta: verificar el
ejecutable instalado, launch/guardas y actualizar documentacion de paquetes
antes de preparar la simulacion 207.
Verificacion install: `fiducial_visualizer` existe y es ejecutable. Guarda
completa intento 1: 14/15 checks correctos; unico fallo `paths` por cinco
`__pycache__` generados durante CTest. Sin fallo funcional. Siguiente accion:
retirar exclusivamente esos caches y repetir la guarda.
Guarda completa intento 2: 15/15 checks correctos tras retirar solo caches
generados. Siguiente accion exacta: actualizar documentacion vigente de
`orbslam3_ros2` y `dron_individual` con el aislamiento implementado; despues
registrar y ejecutar la prueba 207.
Documentacion de paquetes actualizada: publisher debug latest-only en wrapper,
HighGUI exclusivo del ejecutable separado y cierre forzado aislado de `stereo`.
`git diff --check` correcto.
Prueba siguiente: 207, trayectoria tipica completa con YAML absoluto; Gazebo
GUI=true, RViz2=true, mission GUI=false, ambos grafos web/telemetria=false,
debug fiducial=true y display 5.0 s. Timeout 900 s, post-wait 30 s, recursos
monitorizados. Criterios: visualizador muestra/cierra; tags y deltas continúan;
si el visualizador se cierra o muere, ambos `stereo` permanecen activos y RViz2
sigue recibiendo mapa. Siguiente accion exacta: ejecutar prueba 207.
Simulacion 207: finalizada completa; scenario exit 0, `SIM-DONE success=true`
y `SIM-EXIT-CODE 0`. Recursos estables, `guard_triggered=false`, minimo
disponible 5272.9 MiB. Revision visual del usuario: RViz2 y wrappers funcionaron
con normalidad durante toda la prueba, pero no aparecio ninguna ventana con
tags. Conclusion provisional: aislamiento del wrapper conseguido; salida visual
no conseguida. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_207.log` y no leido. Siguiente accion:
reducir por deteccion, publisher, visualizador, deltas y muertes de procesos.
Diagnostico reducido 207: ambos visualizadores arrancan READY; ambos wrappers
detectan tags 202/204 y publican `FID-VISUAL-PUB`; cada visualizador recibe y
registra `SHOW`, pero 3-4 ms despues registra `CLOSE reason=user_close`. Causa:
`WND_PROP_VISIBLE` devuelve 0 transitorio inmediatamente tras `namedWindow` y
el nodo lo interpreta como cierre humano antes de que la ventana sea pintada.
Correccion mecanica dentro del acuerdo: no aceptar `user_close` hasta haber
observado al menos una vez la ventana visible; conservar timeout de 5 s.
Siguiente accion exacta: aplicar guarda de visibilidad, actualizar contrato,
rebuild/CTest y decidir la prueba corta de verificacion.
Guarda de visibilidad aplicada en `fiducial_visualizer`: `user_close` solo se
acepta tras observar visibilidad; el cero transitorio inicial queda ignorado y
el timeout permanece. Contrato y documentacion de paquete actualizados.
Siguiente accion exacta: rebuild aislado de `orbslam3`, CTest y contrato de
Simulacion antes de una nueva prueba runtime.
Rebuild orbslam3 tras carrera de visibilidad: exit 0, 1/1 correcto; solo aviso
legacy de cabecera cv_bridge. Log completo conservado en
`codex/archivos_auxiliares/colcon_build.log` y no leido. Siguiente accion:
CTest wrapper y rebuild/CTest de `simulacion_dron`.
Validacion tras carrera: CTest wrapper 1/1 correcto; build simulacion_dron
1/1 correcto; CTest simulacion 10/10 correcto, incluido contrato que exige la
guarda `window_was_visible_`. Prueba siguiente: 208, repeticion integra de 207
con igual YAML/launch/criterios para comparar solo la correccion visual.
Siguiente accion exacta: ejecutar prueba 208.
Simulacion 208: finalizada completa; scenario exit 0, `SIM-DONE success=true`
y `SIM-EXIT-CODE 0`. Recursos estables, `guard_triggered=false`, minimo
disponible 5388.2 MiB. Log completo conservado en
`codex/archivos_auxiliares/logs/prueba_208.log` y no leido. Revision visual
humana de las ventanas aun no comunicada. Siguiente accion: reducido tematico
de publisher/visualizador/procesos y contraste con observacion del usuario.
Diagnostico reducido 208: 2 READY, 80 publicaciones, 79 SHOW, 17 cierres por
timeout, 0 `user_close`, 0 desactivaciones y tags 202/204. Tras el ultimo
timeout ambos wrappers publican deltas unos 57 s mas; no muere ningun `stereo`,
solo Gazebo exit 255 en cleanup. Historial 207/208, contrato 4D, resumen de
fase, estado y ultima sesion sincronizados. Estado 4D: PARCIAL pendiente
exclusivamente de confirmacion visual humana de la prueba 208.
Guarda final tras CTest/documentacion: intento 1 14/15 solo por caches
generados; caches retirados; intento 2 15/15 correcto. `git diff --check`
correcto. Trabajo activo: si, cierre conversacional 4D. Siguiente accion
exacta: recibir la observacion visual del usuario sobre las ventanas de 208 y
actualizar la misma entrada/conclusion; no repetir simulacion mientras tanto.
Revision final 2026-08-25: el usuario acepta la prueba 208 y da 4C+4D por
concluidas. Historial, contratos, pipeline, estado y ultima sesion actualizados;
4D pasa a CONSEGUIDA sin borrar los intentos previos. Preparacion 4E+4F:
EN_DEBATE. Acuerdo cerrado: no. Autorizacion funcional: PENDIENTE. Prueba
acordada: pendiente de conversacion. Dudas abiertas: revisar contrato, codigo
vigente, sincronizacion batch-KF y pruebas antes de pedir autorizacion.
Trabajo activo: si; solo preparacion 4E+4F, sin cambios funcionales.
Inspeccion preparatoria 4E+4F: el wrapper ya conserva en `FiducialJob` la clave
`drone_id/map_epoch`, el evento exacto con `keyframe_id/source_frame_id/stamp`
y todas las metricas 4D. `OrbKeyFrame` raw conserva id/stamp pero no
`source_frame_id`. El servidor tiene subscriptions reliable KeepLast(10),
`PrimaryWorker` compromete raw antes de derivados y el backend expone
`GetRawKeyFrame(RawKeyFrameId)`. Punto seguro: sincronizador acotado en capa
ROS, callback batch + notificacion post raw-commit, sin contaminar backend ni
activar semantica 4G. La conversion temporal actual usa redondeo en debug y
truncado en `FillKeyFrameMsg`; 4E debe unificarla para igualdad exacta o acordar
tolerancia minima. Dudas abiertas: tags validos publicados, QoS, TTL/capacidad,
politica al llenarse, handoff pre-4G y prueba visual de ambos grafos.
Preparacion estatica completada: contratos 4E/4F, docs de wrapper, servidor,
RawMapDatabase y visualizadores contrastados; fragmentos minimos de interfaces
y puntos de commit inspeccionados. `git diff --check` correcto. Propuesta a
debatir: solo detecciones validas; mensaje sin timestamp duplicado; QoS reliable
KeepLast(32); sincronizador ROS con pending 30 s/capacidad global 512 y rechazo
explicito al llenarse; handoff sin semantica ni persistencia backend; prueba
tipica completa con Gazebo/RViz2 y ambos grafos web activos. Siguiente accion:
recibir decisiones del usuario; no modificar codigo/build/simulacion.
Revision conversada 4F: el usuario prefiere una bandeja fiducial muy pequena
integrada en `RawMapDatabase`, consultada al insertar KFs nuevos, y FIFO
drop-oldest en vez de TTL/capacidad 512. Propuesta reconciliada: sidecar
pending, no mezclarlo con OrbMap raw; reutilizar el mutex interno ya existente;
`unordered_map<RawKeyFrameId,batch>` para lookup O(1) y deque solo para orden
de expulsion; al llegar batch se resuelve inmediatamente si el KF ya existe;
al commit delta/full se consulta un hash por cada KF nuevo. `RawMapDatabase`
devuelve matches en `RawInsertResult` y `GlobalMapServer` hace el handoff fuera
del lock; la base no llama a `FiducialManager`. Sin matrices, PnP ni semantica
bajo mutex. Acordado: solo valid=true, header.stamp canonico, QoS reliable
KeepLast(32), handoff sin 4G y prueba tipica con ambos grafos. Dudas abiertas:
capacidad 10 global o por dron, confirmar FIFO sin TTL y ledger reciente
acotado para duplicate/conflict. Autorizacion funcional sigue PENDIENTE.
Acuerdo adicional: capacidad pending 10 por dron; al emparejar y entregar un
batch se extrae inmediatamente y libera hueco, con independencia de que 4G lo
acepte o descarte; sin TTL; FIFO drop-oldest por dron al insertar el undecimo.
Distincion pendiente explicada: otro KF parecido tiene clave distinta y debe
llegar normalmente a 4G; duplicate/conflict significa exclusivamente una
segunda entrega de la misma clave exacta. Sin marca consumida, al haberse
retirado de pending y existir ya el KF raw, esa reentrega se entregaria otra
vez. Propuesta minima: sidecar por KF con digest del batch consumido, sin
conservar el batch ni usar una lista reciente de 32; igual=DUPLICATE,
distinto=CONFLICT. Duda abierta unica sobre sincronizacion: aceptar esta marca
consumida/digest exacta. Autorizacion funcional sigue PENDIENTE.
Clarificacion sobre consumed: no se necesita un bool
`fiducial_batch_consumed` separado. Deltas y full snapshots solo intentaran
resolver `raw.new_keyframe_ids`; un KF existente/actualizado no reactiva el
handoff. El unico estado recomendado es un digest opcional por clave exacta:
su presencia ya significa consumido y protege solo reentregas del batch ROS,
no revisitas de otros KFs. El pending se elimina al match. Snapshots no son la
razon del digest. Duda abierta: confirmacion final de mantener ese digest
opcional minimo para cumplir DUPLICATE/CONFLICT. Autorizacion PENDIENTE.
Auditoria final preparatoria: `RawInsertResult` ya contiene `arrival_id` para
matches resueltos durante un commit, pero `RawMapDatabase` no conserva hoy el
primer arrival por KF; un batch tardio puede recuperar el KF pero no su commit
raw original. Propuesta: metadato interno `keyframe_first_arrival_id`, sin
modificar OrbKeyFrame, para que todo `SynchronizedFiducialBatch` lleve el
arrival exacto. Capacidad pending debe ser parametro YAML configurable, default
10 por dron; medir current/peak/evicted/matched_immediate/matched_pending.
Validaciones sin decision abierta: topic/drone coherentes, batch no vacio,
tags unicos/ordenados, numeros finitos, quality [0,1], timestamp exacto tras
helper compartido y quaternion normalizado. Full snapshot solo resuelve KFs
clasificados como nuevos; updates no reactivan batches. Preguntas finales:
aceptar parametro default 10 y conservar first_arrival_id interno. Sin cambios
funcionales ni autorizacion todavia.
Checkpoint de reanudacion 2026-08-25: archivo releido fisicamente tras la
compactacion y estado reconciliado con la ultima respuesta del usuario. Quedan
aceptados el parametro YAML `fiducial_pending_capacity_per_drone` con default
10 y el metadato interno `keyframe_first_arrival_id` sin modificar
`OrbKeyFrame`. Dudas tecnicas abiertas: ninguna. Preparacion 4E+4F permanece
EN_DEBATE y acuerdo cerrado: no, solo hasta que el usuario confirme el resumen
final; autorizacion funcional: PENDIENTE. Prueba acordada propuesta: unitarias
y de componentes, builds selectivos y trayectoria tipica completa con Gazebo,
RViz2 y ambos grafos web activos; ventanas fiduciales desactivadas. Siguiente
accion exacta: presentar el acuerdo final y esperar su confirmacion, sin codigo,
build ni simulacion.
Confirmacion documental 2026-08-25: el usuario acepta el resumen final y pide
reflejarlo en los MD de 4E y 4F. Preparacion: CERRADA. Acuerdo cerrado: si.
Autorizacion funcional: PENDIENTE. Dudas abiertas: ninguna. Prueba acordada:
unitarias/componentes, builds selectivos y trayectoria tipica completa con
Gazebo, RViz2 y ambos grafos web activos; ventanas fiduciales desactivadas.
Contratos actualizados con mensaje/QoS, sidecar pending por dron, capacidad YAML
10, FIFO sin TTL, digest exacto, first_arrival_id y handoff fuera del mutex.
No se ha modificado codigo, launch, configuración, ni se ha ejecutado build o
simulacion. Siguiente accion exacta: esperar una orden posterior y explícita de
ejecución del bloque 4E+4F.
```

Documentacion preparatoria 4A+4B sincronizada con el acuerdo conversado:
objetos a ±8.5 m, trayectoria a ±10 m, rango fiducial configurable con perfil
inicial 1-5 m, scoring 3R sin cambios, revision visual a cargo del usuario y
ejecucion posterior por bloques. Tambien quedan documentados los grafos de
4E+4F, la retirada de GT fiducial en 4H y la decision de camara aplazada a 4I.

## Acuerdo ejecutado

```text
mi_tfg se conserva como legacy fuera de los grupos
ORB_SLAM3_MULTI y fase45_sandbox se retiraron
orbslam3_msgs se duplica; Servidor es canonico y Dron replica exacta
ADR 0009 gobierna ownership y replicas YAML
cada build selecciona exactamente un paquete
debug oficial: RViz2, pipeline_flow, system_architecture, navegadores,
telemetria arquitectonica y logs F3 activos
todos los flags conservan default false
```

## Implementacion

- grupos fisicos `dron`, `servidor` y `simulacion` con distribucion 5/3/1;
- bases `build/install/log` independientes por grupo;
- YAML por dominio, `use_sim_time` explicito y Xacro conectado a la masa configurada;
- ORBvoc completo preparado en `build/dron/_phase2_resources` e instalado;
- `pipeline_flow` lazy-gated antes de serializar;
- `system_architecture` declarativo, estatico/live y con topic propio;
- guarda de layout, interfaces, dependencias, config, paths, visualizers y docs.

## VALIDACIÓN FINAL

```text
build: 9/9 invocaciones correctas, un paquete por invocacion
CTest: lib_tray 4/4, orbslam3_multi 9/9, orbslam3_server 10/10,
       simulacion_dron 9/9
rebuild de cierre: simulacion_dron 1/1; CTest 9/9
dron_individual: deuda legacy global de linters; archivos tocados correctos
guardas funcionales: todas correctas
guarda final completa: 15 checks, 0 fallos
layout: capturas 1440x900 y 820x1000 inspeccionadas sin solapes
git diff --check: correcto
```

Prueba 199:

```text
smoke debug-off: 5/5 pasos, 4/4 goals, success=true, exit 0
RViz/web 0 MiB, sin marcadores de debug especifico
guard_triggered=false
```

Prueba 200, regresión equivalente a 198:

```text
vuelta oficial: 14/14 pasos, 20/20 goals, success=true, exit 0
RViz2, ambos bridges y navegadores activos
system_architecture live abre con latest_sequence=1
guard_triggered=false; minimo MemAvailable 3873.8 MiB
bridges, RViz2, wrappers y servidor cierran limpiamente
revision visual humana: confirmada correcta por el usuario el 2026-08-24
```

Despues de `SIM-DONE`, `gui_tray_multi` emitio un traceback de shutdown de
`rclpy` y Gazebo termino con el exit 255 conocido. No afectaron al escenario.
El antiguo `ValueError` de cleanup de `system_architecture_bridge` no se
reprodujo.

## Fuente de verdad

```text
codex/pipeline/fase_2_separacion_paquetes/RESULTADO_FINAL_FASE_2.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
codex/pipeline/fase_2_separacion_paquetes/historial/INDEX.md
codex/archivos_auxiliares/logs/prueba_199.reduced.log
codex/archivos_auxiliares/logs/prueba_200.reduced.log
```

La auditoria contra `main` confirma que el commit publicado es `4424a586`.
La implementacion sigue sus decisiones materiales, pero los contratos locales
de 2C-2G deben reconciliar clausulas definitivas eliminadas durante el cierre.
Plan autorizado: layout de referencia, guarda de posiciones, contratos 2C-2G,
build/test/guardas y capturas desktop/viewport estrecho; sin simulacion larga.
Layout implementado en `graph_layout.js`, cargado por `index.html` y aplicado
por `app.js`. El test contractual comprueba cobertura de paquetes y relaciones
espaciales sin fijar pixeles exactos.
Contratos 2C-2G reconciliados con las decisiones definitivas publicadas; 2C
vuelve a distinguir réplica parcial y `deployment profile` completo. Docs de
`simulacion_dron` actualizados con el layout declarativo.
Build siguiente: `./codex/herramientas/build_selected_packages.sh --group simulacion simulacion_dron`.
Motivo: instalar el nuevo asset web y ejecutar la suite contractual del único
paquete afectado, reutilizando los prefijos ya validados de Dron y Servidor.

Build layout 2026-08-24: exit 0; `simulacion_dron` 1/1 correcto.
Log: `codex/archivos_auxiliares/colcon_build.log`.
Siguiente accion exacta: ejecutar CTest de `simulacion_dron` y las guardas.

CTest intento 1: no ejecutado, exit 8 porque el sandbox impidio escribir
`Testing/Temporary/LastTest.log` bajo `build/simulacion`; no es un fallo de test.
CTest intento 2: 9/9 correctos, incluido `system_architecture_contract` con la
nueva guarda de layout.
Guarda completa intento 1: 14 checks correctos y 1 fallo de paths. CTest genero
dos `__pycache__` dentro de `simulacion_dron`; no hay incumplimiento funcional.
Artefactos retirados. Guarda completa intento 2: 15 checks, 0 fallos.
Siguiente accion exacta: capturas web desktop y viewport estrecho, seguidas de
inspeccion visual.

Servidor estatico intento 1: no arranco; el sandbox bloqueo la apertura del
socket local con `PermissionError`. Siguiente accion: repetir con permiso local.
Servidor estatico intento 2: activo en `127.0.0.1:8765`.
Chrome headless intento 1: exit 133 sin captura bajo sandbox.
Chrome headless con permiso: capturas desktop 1440x900 y estrecha 820x1000
generadas e inspeccionadas; contenedores completos, texto legible y sin solapes.
Servidor temporal detenido limpiamente.
Siguiente accion exacta: cierre documental y verificacion final de coherencia.

Cierre documental aplicado. Guarda posterior a toda la documentacion: 15/15.
Asset `graph_layout.js` instalado en el prefijo aislado de Simulacion. No quedan
estados pendientes vigentes y `git diff --check` es correcto.
Guarda tras el checkpoint final: 15/15, 0 fallos.
Commit principal de cierre creado correctamente en `main`.
Push intento 1: rechazado por `non-fast-forward`; `origin/main` contiene cambios
nuevos y no se usara force push.
Fetch correcto: remoto añade `63d677a` y `4424a58`, ambos documentales.
Rebase completado sobre `origin/main`; 26 conflictos documentales resueltos con
autorización explícita conservando las versiones finales ya reconciliadas.
Guarda posterior al rebase: 15/15, 0 fallos.
Commit principal publicado: `d189b0e` en `origin/main`.
Siguiente accion exacta: ninguna; esperar preparación solicitada de 4A.
Fase 4 queda actual, pero ninguna subfase puede ejecutarse sin preparación y
autorización explícitas.
Guarda tras activar Fase 4: 15/15, 0 fallos.

Actualizacion documental Fase 4 2026-08-24: integrado
`Fase_4_completa_4A_4I_muy_detallada.zip` como contrato preparatorio vigente.
`pipeline_fase_4.md`, `pipeline_fase_4_RESUMEN.md` y subfases activas 4A-4I
quedan reconciliadas con Fase 2 ya cerrada. Las antiguas 4J-4L salen del flujo
activo y se conservan en `subfases/legacy/`. El detalle largo importado de 4A,
4B, 4C, 4D y 4G se conserva en `subfases/detalle/`, dejando los contratos
ejecutables cortos conforme a la politica de documentacion. Verificacion:
`git diff --check` correcto y busqueda de frases obsoletas sin coincidencias.
