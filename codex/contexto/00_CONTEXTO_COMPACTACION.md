# 00 - Contexto de compactacion

## Estado Vivo

```text
Estado: 3B-3P y 3S-3W CONSEGUIDAS; 3Q A REVISAR; 3X PENDIENTE
Objetivo vigente: preparar y ejecutar 3X como limpieza, configuracion y handoff final, manteniendo 3Q A REVISAR
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: PENDIENTE; concedida solo para crear el checkpoint Git previo
Prueba acordada: `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` en Gazebo, ademas de build y regresiones
Dudas abiertas: ninguna
Checkpoint 3X acordado: commit previo limitado a Fase 3 (`codex`, `orbslam3_multi`, `orbslam3_server`, cambios 3X de `simulacion_dron` y `.gitignore`); excluir cambios del usuario en `ORB_SLAM3`, `orbslam3_ros2` y `fase45_sandbox`
Acuerdo 3X: retirar legacy de Fase 3 de la version actual sin reescribir Git; absorber/eliminar 3R y 3T-3W tras migrar sus conclusiones; diagnosticar duplicacion antes de retirar; documentar responsabilidades, funciones complejas e invariantes sin comentarios obvios; crear ADR de configuracion por dominios dron/servidor/simulacion; YAML separados y sincronizados en servidor y simulacion; actualizar launches, manifiestos y resultado final
Metadatos 3X acordados: `GPL-3.0-only`, mantenedor `ivancalvosantos2003@uma.es`, version `0.1.0`; ignorar `fase45_sandbox`; 3Q permanece `A REVISAR` y se mostrara como limitacion pendiente en el resultado final
Siguiente accion exacta: crear y verificar el commit de checkpoint; despues pedir autorizacion funcional explicita para ejecutar la limpieza 3X completa
Conclusion agregada 3T: CONSEGUIDA por auditoria; arquitectura implantada por 3C-3S, rendimiento aceptado por el usuario y sin cambios funcionales adicionales
Conclusion agregada 3U: CONSEGUIDA; contrato web 9/9, SSE live/reconexion y lifecycle vigentes, con cierre visual explicito del usuario
Documentacion cierre 3T/3U: contratos, historiales/resumen/indice, contexto, estado, pipeline y ultima sesion sincronizados; no se modifico codigo ni se ejecuto simulacion nueva
Verificacion cierre 3T/3U: `pipeline_flow_contract` 9/9 en 0.13 s y `git diff --check` correcto
Conclusion agregada 3V: CONSEGUIDA; pruebas 187/188/191/194 aceptadas como regresion integral suficiente, sin nueva ejecucion monolitica ni A/B
Conclusion agregada 3W: CONSEGUIDA; rendimiento/robustez actuales aceptados, incluidos picos residuales documentados, sin mas tuning o stress preventivo
Documentacion cierre 3V/3W: contratos, historiales/resumen/indice, contexto, estado, pipeline y ultima sesion sincronizados; no se modifico codigo ni se ejecuto simulacion nueva
Verificacion cierre 3V/3W: estados activos coherentes y `git diff --check` correcto
Trabajo activo: ninguno; siguiente subfase pendiente 3X
Acuerdo revisado de distancia 3S: para raw anclado, limite cercano fisico fijo `near_limit=1.0 m`; si `d<near_limit`, `distance_factor=max(0.05,(d/near_limit)^2)`; banda neutra desde 1 m hasta `far_limit=max(near_limit,83.333333*baseline)`, que con baseline actual `bf/fx=0.06 m` termina en 5 m; si falta baseline, fallback 5 m; para `d>far_limit`, conservar `distance_factor=max(0.25,(far_limit/d)^2)`. El factor se recalcula y recupera al cambiar posicion/pose/calibracion. La penalizacion cercana pertenece al raw miembro y puede diluirse mediante la media exacta del fused track al incorporar miembros buenos; no existe cap ni castigo permanente fused.
Alcance revisado confirmado: no cambian base ORB, aislamiento, `+0.04` raw por inlier, fused `media(raw)+0.04*N`, oclusion aplazada, ownership, dirty sets, builder sin filtro ni gradiente RViz2. Se amplian regresiones de limites/recuperacion/fusion y se repite la prueba tipica con logs reducidos y validacion RViz2.
Riesgos revisados aceptados: el umbral cercano fisico puede penalizar objetos realmente validos a menos de 1 m; la escala lineal por baseline supone camaras de focal/resolucion y precision de disparidad comparables; la media y el bonus fused pueden recuperar parcialmente un miembro cercano penalizado, de forma intencional.
Decisiones 3S confirmadas: fused landmark usa `clamp(media(scores de todos sus raw MapPoints) + 0.04 * N, 0, 1)` con `N` igual al numero de raw MapPoints miembros, por lo que la primera fusion suma `0.08` y cada miembro posterior `0.04`; se conserva ademas `+0.04` raw por inlier confirmado como doble refuerzo intencional; cada raw MP anclado recibe refinamiento geometrico propio y una actualizacion ORB posterior recalcula su score y propaga incrementalmente el cambio a su fused track; scoring por oclusion aplazado a Fase 8/nube densa y visibilidad sparse solo diagnostica en 3S; corregir telemetria de `F1S-*` a `F3S-*`; RViz2 usa el campo `rgb` derivado de score y debe mostrar el gradiente verde-amarillo-rojo
Alcance acordado 3S: `score_raw = clamp(base_score_orb * factor_distancia * factor_aislamiento + 0.04 * inliers_confirmados, 0, 1)`; factores acotados, configurables y recuperables; distancia con penalizacion para cercania fisicamente sospechosa y degradacion progresiva lejana basada en calibracion; aislamiento global persistente y recuperable mediante indice espacial incremental; raw no anclado conserva base ORB sin refinamiento global; fused score recalculado por cambios de miembros, altas, merges y bajas; `ScoreChangeSet` exacto y dirty IDs sin snapshots completos
Exclusiones acordadas 3S: sin penalizacion numerica por oclusion hasta Fase 8, sin GT, sin worker/cola/prioridad nuevos de score, sin modificar geometria/raw/fusion/optimizacion para ajustar score, sin filtrar puntos en builder y sin bloquear publicacion esperando scoring derivado
Riesgos aceptados 3S: umbrales iniciales de distancia/aislamiento requieren calibracion conservadora; aislamiento provisional puede cambiar al crecer el mapa; score puede saturar por doble refuerzo intencional; una prueba natural puede producir pocas contradicciones visuales y se informara sin inventar evidencia
Plan vigente 3S: esperar autorizacion funcional explicita; ajustar solo la politica/configuracion de distancia y sus regresiones; compilar `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`; repetir tests funcionales; ejecutar la prueba tipica acordada; reducir logs, contrastar stats y cerrar visualmente en RViz2
Bloque de cambios 3S completado: contratos actualizados; `LandmarkScoreManager` incorpora factores recuperables de distancia/aislamiento e indice voxel incremental; backend refresca geometria tras raw/anchors/poses y propaga a fused; fused usa media raw mas `0.04*N`; visibilidad sparse queda diagnostica; parametros, grafo y telemetria `F3S-*` añadidos; regresiones nuevas preparadas
Archivos criticos 3S modificados: `landmark_score_manager`, `fused_landmark_manager`, `raw_map_types/raw_map_database`, `sparse_global_backend`, `global_map_server`, tests de score/fusion y contrato web; no se tocaron cambios del usuario en ORB/wrapper/fase45
Siguiente accion exacta: compilar `orbslam3_multi orbslam3_server simulacion_dron` con la herramienta seleccionada y reducir el log solo si falla
Build 3S intento 1: CONSEGUIDO el 2026-08-22 17:34:19-17:34:58; exit 0, 3/3 paquetes (`orbslam3_multi`, `orbslam3_server`, `simulacion_dron`) en 38.4 s; avisos conocidos de Drake/underlay no bloqueantes
Siguiente accion exacta: ejecutar CTest funcional de `orbslam3_multi`, contrato web y tests funcionales de servidor; corregir cualquier regresion sin cambiar la politica acordada
Tests 3S intento 1: NO CONSEGUIDOS globalmente, 7/9; score manager, fused, backend, raw, fiducial y loop pasan. `test_global_map_builder` conserva expectativa antigua 1.0 para un punto que ahora queda aislado a 0.35; `test_scalability_3g` agota 60 s por expansion de vecinos por cada alta en el primer indice voxel
Correccion mecanica/rendimiento 3S: fixture builder adaptado a la penalizacion acordada; indice agrupa voxels afectados por lote y cuenta ocupacion en 27 voxels por punto, evitando coste cuadratico; baseline cacheado por submapa
Siguiente accion exacta: recompilar los tres paquetes y repetir CTest completo para verificar correccion y escalabilidad
Build 3S intento 2: CONSEGUIDO el 2026-08-22 17:38:44-17:39:21; exit 0, 3/3 paquetes en 36.6 s
Tests 3S intento 2: CONSEGUIDOS; `orbslam3_multi` 9/9 en 25.49 s, incluida escalabilidad en 23.85 s; contrato web 1/1 en 1.46 s; funcionalidad `orbslam3_server` 4/4 en 0.35 s
Prueba preparada 3S: prueba 192, YAML absoluto `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`, launch normal, startup 15 s, timeout 1200 s, drenaje 180 s y monitorizacion de recursos. Criterios: scenario success, telemetria `F3S-*`, raw/fused incremental, negativos sparse cero, puntos aislados/near/far observables, nube completa y recursos estables; inspeccion visual RViz2/grafo pendiente del usuario
Siguiente accion exacta: ejecutar prueba 192 y registrar resultado bruto antes de reducir su log completo
Prueba 192 resultado bruto: completada `success=true`, scenario/tool exit 0, 578 s monitorizados incluidos 180 s de drenaje; log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_192.log`. Recursos estables: `guard_triggered=false`, server RSS max 254.5 MiB, grupo RSS max 1590.3 MiB, memory PSI 0, MemAvailable minima 3939.0 MiB
Siguiente accion exacta: reducir prueba 192 por score 3S, fusion, escenario/cierre, errores y recursos; leer solo artefactos reducidos
Diagnostico prueba 192: politica 3S activa con commits raw incrementales, fusiones reales, propagacion a fused y publicaciones con `score_field=true rgb_field=true`; 120/120 intentos fused muestreados tienen `negative=0` y diagnosticos sparse positivos. Sin errores graves. Limitacion: tras 180 s la cola primaria baja 49->45 pero no drena porque cada input ORB con geometria identica reexpande vecindad; telemetria de stats solo se emitia en ruta replay y no permite cuantificar live aislados/near/far
Correccion de rendimiento/telemetria 3S: una entrada geometrica identica solo reevalua su propio factor (madurez ORB) sin reindexar vecinos; stats `F3S` se emiten cada 25 arrivals live. Formula, umbrales y ownership no cambian
Siguiente accion exacta: recompilar, repetir regresiones y ejecutar prueba 193 exacta para validar backlog y estadisticas live
Build 3S intento 3: CONSEGUIDO el 2026-08-22 17:53:19-17:53:43; exit 0, 3/3 paquetes en 23.5 s
Tests 3S intento 3: CONSEGUIDOS; `orbslam3_multi` 9/9 en 25.51 s, servidor funcional 4/4 en 0.34 s y contrato web 1/1 en 1.22 s
Prueba preparada 3S: 193, repeticion exacta de 192 con YAML tipico, launch normal, timeout 1200 s, drenaje 180 s y recursos; comparar backlog y exigir stats live no triviales con negativos sparse cero
Siguiente accion exacta: ejecutar prueba 193 y registrar resultado bruto antes de reducir
Prueba 193 resultado bruto: completada `success=true`, scenario/tool exit 0, 572 s monitorizados incluidos 180 s de drenaje; log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_193.log`. Recursos estables: `guard_triggered=false`, server RSS max 269.5 MiB, grupo RSS max 1639.2 MiB, memory PSI 0, MemAvailable minima 4445.3 MiB
Diagnostico prueba 193: CONSEGUIDA tecnicamente. Principal 722/pending 0; secundario 1288/pending 0/hard_failed 0. Stats finales tracked=60524, bad=30836, anchored=24969, isolated=529, near=1, far=24195, score min/mean/max=0/0.1502/1. Reduccion fused completa: 166 intentos, 77 commits, 12672 positivos, 6319 diagnosticos, 4528 dirty y cero negativos sparse. Publicacion final 23531 puntos con score/rgb. El exit Gazebo 255 sucede durante cleanup posterior a success
Documentacion 3S: contratos, docs de paquetes, historial cronologico/resumen/indice, estado y ultima sesion sincronizados; 192 permanece como parcial y 193 como correccion conseguida
Conclusion agregada 3S: PARCIAL; implementacion/build/tests/simulacion conseguidos, pendiente confirmacion visual del usuario y decidir si la penalizacion lejana de 24195/24969 anclados separa ruido de estructura valida o requiere recalibracion
Revision visual prueba 193: el usuario confirma que las revisitas elevan correctamente el score de una zona. Tambien observa que la mayoria de puntos validos queda demasiado baja y que puntos a menos de 1 m conservan score excesivo. La configuracion actual explica ambos sintomas: con baseline aproximado de 0.06 m el factor lejano empieza en 2.4 m (`40*baseline`), mientras la cercania solo se penaliza por debajo de 0.20 m. La conclusion tecnica de 193 se conserva, pero su calibracion visual no es valida.
Bloque de recalibracion 3S completado: defaults near `1.0/0.05`, far `83.333333/5.0/0.25`; curva cercana cuadratica; curva lejana cuadratica conservada; servidor sincronizado; regresiones numericas de banda, baseline, fallback, recuperacion y dilucion fused añadidas; docs de subfase y paquetes actualizadas. No se tocaron cambios del usuario en ORB/wrapper/fase45.
Trabajo activo: cambios implementados; pendientes build, regresiones y simulacion
Build recalibracion 3S: CONSEGUIDO el 2026-08-22 18:30:38-18:31:11; exit 0, 3/3 paquetes (`orbslam3_multi`, `orbslam3_server`, `simulacion_dron`) en 32.9 s; avisos conocidos de Drake/underlay no bloqueantes
Tests recalibracion 3S: CONSEGUIDOS; dirigidos `LandmarkScoreManager` 8/8 y `FusedLandmarkManager` 4/4; CTest `orbslam3_multi` 9/9 en 25.15 s, servidor funcional 4/4 en 0.35 s y contrato web 1/1 en 1.24 s. El primer `ctest -N` de servidor no pudo escribir `LastTest.log` por sandbox y no ejecuto pruebas; la repeticion autorizada fue limpia.
Prueba preparada 3S: prueba 194, mismo YAML absoluto `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml`, launch normal, startup 15 s, timeout 1200 s, drenaje 180 s y recursos. Criterios: scenario success, colas drenadas/estables, near/far coherentes con banda 1-5 m, score medio recuperado respecto a 193, cero negativos sparse, nube score/rgb completa y validacion visual posterior.
Prueba 194 resultado bruto: completada `success=true`, scenario/tool exit 0, 582 s monitorizados incluidos 180 s de drenaje; log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_194.log`. Recursos estables: `guard_triggered=false`, server RSS max 248.0 MiB, grupo RSS max 1571.3 MiB, memory PSI 0 y MemAvailable minima 4529.0 MiB.
Diagnostico prueba 194: recalibracion tecnica CONSEGUIDA. Cierre tracked=59271, bad=30641, anchored=24977, isolated=435, near=99, far=11433 y score min/mean/max=0/0.2596/1. Frente a 193, con practicamente los mismos anclados, far baja 24195->11433, near sube 1->99 y media sube 0.1502->0.2596. Hay 139 intentos fused, 53 commits, 14448 positivos, 6434 diagnosticos, 3931 dirty y cero negativos sparse. Publicacion final 23564 puntos con score/rgb. Principal 739/pending0; secundario 1273/pending0/hard_failed0. Unico ERROR: Gazebo exit255 durante cleanup posterior a `SIM-DONE`.
Conclusion agregada 3S: CONSEGUIDA; implementacion, build, regresiones y simulacion de recalibracion conseguidas, y el usuario confirma que los scores visuales han salido perfectos.
Documentacion recalibracion 3S: contratos, docs de `orbslam3_multi`/`orbslam3_server`, historial cronologico/resumen/indice, estado, pipeline y ultima sesion sincronizados con prueba 194; 192/193 permanecen como antecedentes. `prueba_194.reduced.log` final conserva 888 lineas tematicas y el log completo no fue leido. `git diff --check` pasa.
Revision visual prueba 194: el usuario confirma que los scores han salido perfectos y da 3S por concluida. Incidencia separada: el dron que gira antihorario optimizo mal al final, cerca del fiducial 2; investigar causalidad 3Q/fiducial sin cambios funcionales.
Trabajo activo: diagnostico read-only de la optimizacion final defectuosa en prueba 194
Diagnostico optimizacion final prueba 194: el dron antihorario es drone 2/submapa `(2,1)`. `task=1000000005308` sobre query 220 corrige 3.950m/0.454rad con ratios RANSAC 0.278-0.388, ambiguity=true y 10 competidores; usa una arista loop, ventana 296/115 controles/3 submapas, mueve 359 KFs y admite incremento estructural 0.686m/0.111rad. `task=1000000005421` corrige despues 0.780m/0.078rad, mueve 362 KFs de ventana 303 y aumenta estructura 0.335m/0.049rad. En ambos `protected.query=false,candidate=true`, por lo que la politica asimetrica evita el precheck 5m/20deg; el validador los acepta dentro de limites temporal/covisible. El subcommit de pose queda committed aunque la fusion directa termine stale por dependencias. La visita fid2 llega despues: errores 0.067-0.252m y ~0.035rad quedan within_threshold, promueve el primer control pero no reoptimiza el interior. 5421 es la ultima OPT loop; la deformacion persiste. Causa dominante probable 5308, amplificada por 5421; no scoring, recursos ni fiducial malo.
Documentacion diagnostico 194: historial/resumen 3Q, indice, referencias 3S, estado, pipeline y ultima sesion sincronizados; misma ejecucion, no prueba nueva. Reduccion final tematica de 166 lineas; log completo no leido. No se modifico codigo/configuracion.
Trabajo activo: ninguno; 3S CONSEGUIDA y 3Q permanece A REVISAR con punto de reentrada documentado
Contrato 3Q actualizado por peticion del usuario: añade una seccion visible de lectura obligatoria hacia `historial_3Q_RESUMEN.md` y `historial_3Q.md`, destaca la revision de prueba 194 y corrige el cierre obsoleto de 188 a `A REVISAR`. Sin cambios de codigo/configuracion.
Siguiente accion exacta: preparar 3X cuando el usuario lo indique; antes de retomar 3Q leer obligatoriamente su resumen e historial
Alcance acordado 3Q: convertir `OptimizationEvidence` en una rama relativa dentro de la misma `LoopTask` BAJA y unificar el optimizador covisible para loop y fiducial; construir ventanas conjuntas mediante fiduciales hard, tramos temporales, fusiones/loops previos, dependencias blandas y covisibilidad fuerte; mover la componente segun constraints; reutilizar lifecycle/builder/solver/validator/store/fusion 3P; commit breve y dirty sets. Una dependencia soft se sigue e incluye cuando conecta con un submapa delimitado por dos fiduciales hard
Exclusiones acordadas 3Q: sin nueva cola/worker/solver duplicado, sin GT, sin modificar raw, sin publicar desde secundario, sin preemption, sin offset artificial de pruebas y sin excluir loops por ser inter/intra dron o submapa
Criterios acordados 3Q: constraints loop relativas y fiduciales absolutas sobre el grafo comun; hard fiducials inmoviles; fusiones previas como relaciones soft medibles; covisibilidad confirmada en optimizaciones loop y fiducial; densidad base de controles 30 por ciento ampliable por constraints covisibles; dos apoyos independientes; accept completo provisional; fusion posterior opcional sin invalidar una buena pose; stale/rollback reencola BAJA fresca; cero escrituras en reject/stale; continuidad de KFs tardios; accepts positivos reproducibles automaticos/live/visuales. Al entrar en optimizacion dentro de una LoopTask BAJA se activa stop_drones y se mantiene hasta terminar validacion, commit y fusion directa de la rama
Riesgos aceptados 3Q: falso cierre en zonas repetitivas mitigado por dos queries/ambiguedad; degradacion medible de fusion previa soft; sobrerigidez por covisibilidad mitigada por pesos configurables; invalidacion concurrente resuelta con stale/retry; commit multi-base con rollback; coste de ventanas/fusion y posibilidad de que una prueba Gazebo natural no produzca error alto, que se informara sin falsear el resultado
Correccion acordada tras 179: un submapa nunca anclado puede anclarse por loop sin limite de distancia; solo un nuevo epoch nacido tras perder un submapa anclado recibe una envolvente por KF derivada de trayectoria raw acumulada. El loop se rechaza con cero escrituras si satisface constraints nuevas rompiendo temporal/covisibilidad/fusiones previas/hard. Todas las aristas `ServerLoopGeometric` se siguen transitivamente sin bonus artificial. Los tramos entre dos fiduciales conservan como referencia la ultima solucion fiducial, con margen interior provisional de hasta 5 m/20 grados y mayor rigidez cerca de los extremos; los loops no renuevan esa referencia. Tras commits loop o fiducial, todos los KFs movidos se reencolan BAJA; cada pareja ya fusionada se omite sin cancelar la busqueda de candidatos nuevos
Conclusion agregada 3H-3L: CONSEGUIDA; tests, replays 149/150, live 151 y confirmacion visual del usuario validan continuidad RViz2, color por submapa y lifecycle web
Conclusion agregada 3M: CONSEGUIDA; patch MEDIA canonico/versionado y encadenado a loops validados
Conclusion agregada 3N: CONSEGUIDA; indice BoW/regiones/ledger causal y cola drenada validados
Conclusion agregada 3O: CONSEGUIDA; propagacion rigida, reanchor por primer fiducial directo, carga secundaria y visualizacion RViz2/grafo web validadas; 3P/3Q comprobaran integralmente sus evidencias de fusion/optimizacion
Conclusion agregada 3P: CONSEGUIDA por cierre explicito del usuario; fallo 159 conservado, prueba 160 valida correccion/visual y prueba 161 valida retry fresco, visibilidad completa, optimizaciones, drenaje y recursos; pulido layout no reabre funcionalidad
Conclusion agregada 3Q: A REVISAR; la prueba 191 corrige el bloqueo 189 y el usuario valora muy positivamente el resultado, por lo que autoriza avanzar. No se cierra definitivamente debido al error visual anterior y al coste residual; reabrir si reaparecen problemas de loops u optimizaciones
Conclusion agregada 3B: CONSEGUIDA; criterios automaticos y visuales confirmados
Conclusion agregada 3C: CONSEGUIDA; raw, FIFO/worker principal, replay, backpressure, web y ausencia de publishers RViz2 validados
Conclusion agregada 3D: CONSEGUIDA; usuario confirma comportamiento web esperado tras prueba90 y da por bueno el resultado para continuar
Conclusion agregada 3E: CONSEGUIDA; usuario acepta la evidencia tecnica de 2 anchors/61 poses/2 hard y no solicita cambios visuales; first anchor se observara de nuevo en la siguiente prueba
Conclusion agregada 3F: CONSEGUIDA; color por epoch corregido, probado y confirmado visualmente por el usuario en live 151
Conclusion agregada 3G: CONSEGUIDA; semantica snapshot, rendimiento, carga real con tres drones y restauracion visual validados
Hito intermedio 3S: contratos actualizados antes de implementar; superado por el cierre tecnico de prueba 193 registrado arriba
Ultimo hito: el usuario acepta el resultado automatico y visual de 191 para continuar, pero marca 3Q `A REVISAR` por el error anterior y posibles problemas futuros de loops/optimizaciones
Politica operativa acordada: compilar solo con build_selected_packages.sh; ejecutar Gazebo/RViz2/web mediante launches llamados por run_simulation.sh; crear helper de apertura de una pestaña del grafo invocado por el launch; no usar comandos ad hoc para escribir fuera de src
Archivos criticos localizados: `orbslam3_multi/include/orbslam3_multi/{loop_pipeline,pose_graph_problem,pose_graph_builder,optimization_manager,optimization_validator,global_pose_store,sparse_global_backend}.hpp`; sus CPP activos; `orbslam3_server/src/global_map_server.cpp`; `simulacion_dron/web/pipeline_flow/graph_definition.js`; tests C++ de optimizacion/loop/backend y contrato web
Plan vigente: actualizar contrato 3Q; implementar precheck protegido y ledger regional; acotar/agrupar `FusionRefresh` y excluir mantenimiento puro del gate; añadir regresiones; compilar `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`; repetir YAML largo de 189 y analizar reducidos
Build siguiente: CONSEGUIDO, 2026-08-17 13:18:38-13:18:53; exit 0, 3/3 paquetes (`orbslam3_multi`, `orbslam3_server`, `simulacion_dron`); log `codex/archivos_auxiliares/colcon_build.log`; unico aviso visible Drake no bloqueante
Prueba vigente: prueba 166 dirigida completada `success=true`, scenario exit 0, herramienta exit 0 y movimiento posterior al fiducial ejecutado; 60 s de drenaje. Recursos estables (`guard_triggered=false`, server RSS max 178.8 MiB, group RSS max 1353.2 MiB, PSI memoria 0). Log completo preservado y no leido: `codex/archivos_auxiliares/logs/prueba_166.log`. Evidencia: retry fiducial funciona sin hard failure; numerosas optimizaciones loop reales y commits; cola final vacia
Build siguiente: CONSEGUIDO, 2026-08-20 15:03:18-15:03:30; exit 0, 3/3 paquetes (`orbslam3_multi`, `orbslam3_server`, `simulacion_dron`); log `codex/archivos_auxiliares/colcon_build.log`; aviso Drake no bloqueante
Prueba vigente: prueba 167 `success=true`, scenario/tool exit 0, recursos estables y cola final `pending=0`. Tres conflictos fiduciales se reencolaron y el cuarto intento comprometio 140 KFs mas 16 propagados de hijo soft; los anteriores quedaron stale bajo umbral. No hubo optimizacion loop natural: al medir poses KF actuales, todos los loops confirmados fueron fusion/error bajo; las numerosas OPT de 166 eran falsos positivos por anchors estaticos
Prueba vigente: prueba 168 larga `success=true`, recorrido entero, recursos estables y cola final `pending=0`. Hubo 17 intentos 3Q naturales: 5 commits, 9 stale y 3 rejects sin escritura; un commit multi-submap termino con fusion directa. No hubo reoptimizaciones de error casi cero. La ruta fiducial resolvio cinco conflictos por retry y comprometio 148 KFs; backpressure final false
Prueba vigente: prueba 169 G1 dirigida completada `success=true`, scenario/tool exit 0, duracion 506 s. Log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_169.log`. Recursos estables: `guard_triggered=false`, server RSS max 165.5 MiB, group RSS max 1351.9 MiB, PSI memoria 0
Prueba vigente: prueba 170 `success=true`; rebase mejora 3Q de 3/45 a 6/9 commits, con commits multi-submap de 154, 198 y 222 KFs. B se ancla por loop y el commit fiducial padre mueve 235 KFs y propaga 92 del hijo. Sin hard failures y recursos estables. Tras 90 s quedaron 31 tareas fusion 3P pendientes: backlog no drenado, no bloqueo 3Q demostrado
Prueba vigente: prueba 171 repetida con drenaje 300 s completada `success=true`, scenario/tool exit 0, duracion 743 s. Log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_171.log`. Recursos estables: `guard_triggered=false`, server RSS max 175.1 MiB, group RSS max 1390.9 MiB, PSI memoria 0. El backlog de fusion bajo de 28 a 1, pero la tarea `1000000003779` emitio `F3Q-OPT-START` y no termino antes del cierre: es coste excesivo del solver sobre una ventana grande, no deadlock de cola ni fusion
Build siguiente: CONSEGUIDO, 2026-08-20 15:31:43-15:32:16; exit 0, 3/3 paquetes (`orbslam3_multi`, `orbslam3_server`, `simulacion_dron`); 33.3 s; unico aviso visible Drake no bloqueante
Tests siguientes: CONSEGUIDOS; 31/31 C++ (`test_fiducial_optimization` 9, `test_sparse_global_backend` 9, `test_loop_pipeline` 8, `test_secondary_queue` 5) y 9/9 pytest del contrato web; CTest integrado 1/1
Prueba vigente: prueba 172 G1 completada `success=true`, scenario/tool exit 0, duracion monitorizada 680 s y drenaje posterior 240 s. Log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_172.log`. Recursos estables: `guard_triggered=false`, server RSS max 157.7 MiB, group RSS max 1333.2 MiB, PSI memoria 0. Diez `F3Q-OPT-START` tienen diez finales; ventanas de 207-212 KFs resuelven normalmente en 1.66-1.72 s con 24 iteraciones y las dos rutas de 160 iteraciones en ~4.0-4.1 s. B se ancla por loop; hay commits multi-submap hasta 205 KFs. Sin hard failures. La cola final baja 27->23 mediante commits, pero no llega a cero en el tiempo disponible debido a retries 3P por revisiones concurrentes
Prueba vigente: prueba 173 G1 terminada `success=false`, scenario/tool exit 124 por timeout del runner a 800 s; la herramienta mantuvo 360 s de drenaje, duracion monitorizada 1184 s. Log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_173.log`. Recursos estables: `guard_triggered=false`, server RSS max 177.0 MiB, group RSS max 1397.5 MiB, PSI memoria 0. La cola termina `pending=0`, pero una `FiducialOptimizationTask` antigua se reencola tras conflictos, otro commit avanza el control y su build devuelve `target_not_after_control`; el servidor lo convierte en `F3L-HARD-FAILURE`, deja `blocking_failure=true` y el segundo gate nunca se libera
Build siguiente: CONSEGUIDO, 2026-08-20 16:10:19-16:10:33; exit 0, 3/3 paquetes, 14.0 s; aviso Drake no bloqueante
Tests siguientes: CONSEGUIDOS tras correccion stale; 31/31 C++ y 9/9 contrato web, incluida regresion `target_not_newer_than_current_control`
Prueba vigente: prueba 174 G1 completada `success=true`, scenario/tool exit 0, duracion monitorizada 733 s y drenaje posterior 300 s. Log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_174.log`. Recursos estables: `guard_triggered=false`, server RSS max 165.7 MiB, group RSS max 1351.5 MiB, PSI memoria 0. Ocho inicios 3Q tienen ocho commits/finales; maximo 219 KFs, 93 controles, solve 3.870 s en 160 iteraciones y 1.806 s en ventanas grandes de 24 iteraciones. Sin hard failure. La cola final queda en 25 por 317 fusiones stale: visibilidad reconstruye identico depth buffer por outlier y prolonga prepare 1-2 s, facilitando conflicto con cada delta
Build siguiente: CONSEGUIDO, 2026-08-20 20:53:54-20:54:11; exit 0, 3/3 paquetes (`orbslam3_multi`, `orbslam3_server`, `simulacion_dron`), 16.6 s; aviso Drake no bloqueante
Tests siguientes: CONSEGUIDOS; CTest de `orbslam3_multi` 9/9 ejecutables en 3.80 s y contrato web 9/9; regresiones nuevas de tres constraints, batch fiducial multi-submapa y anclaje loop seguido de fiducial pasan
Tests siguientes: NO CONSEGUIDOS tras la primera correccion de 178; CTest 8/9, la nueva regresion de culling alcanza `atomic_batch_conflict` porque el lote conserva como `continuation_control` el KF intermedio ya inactivo; los otros 8 ejecutables pasan
Tests siguientes: CONSEGUIDOS tras corregir continuidad activa; CTest `orbslam3_multi` 9/9 en 3.82 s, incluida la regresion que invalida un control entre solve y commit, y contrato web pytest 9/9 en 0.16 s
Tests siguientes: CONSEGUIDOS; 3/3 `test_fused_landmark_manager` conservan 128/128 evidencias con 2 proyecciones compartidas, mas bateria critica 40/40
Prueba vigente: prueba 175 G1 completada `success=true`, scenario/tool exit 0, duracion monitorizada 613 s y drenaje posterior 180 s. Log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_175.log`. Recursos estables: `guard_triggered=false`, server RSS max 146.4 MiB, group RSS max 1330.8 MiB, PSI memoria 0. B se ancla por loop; aparecen ventanas de 2 y 3 submapas. Ocho OPT start/end: 7 commits iniciales y 1 stale por revision, reintentado y comprometido; max 174 KFs/89 controles, solve max 1.892 s, cola final `pending=0`, sin hard failure/excepcion. Fusiones: 155 intentos, 129 commits, 10 stale; visibilidad max 57.363 ms, prepare max 142.766 ms, proyecciones max 1848. Frente a 174: 696 intentos, 63 commits, 316 stale, visibilidad max 1964.55 ms, prepare max 2001.86 ms, proyecciones max 70568
Prueba vigente: prueba 177, repeticion exacta del YAML 176, termino `success=false` por timeout del runner a 900 s; herramienta exit 124, 945 s monitorizados. Recursos: server RSS max 190.2 MiB, group RSS max 1434.6 MiB, PSI memoria 0 y guarda no activada. Log completo preservado y no leido: `codex/archivos_auxiliares/logs/prueba_177.log`
Build siguiente: CONSEGUIDO, 2026-08-20 21:14:24-21:14:40; exit 0, 3/3 paquetes, 15.3 s; CTest 9/9 en 3.84 s
Build siguiente: CONSEGUIDO, 2026-08-20 21:39:35-21:39:51; exit 0, 3/3 paquetes (`orbslam3_multi`, `orbslam3_server`, `simulacion_dron`), 16.2 s; aviso Drake no bloqueante
Build siguiente: CONSEGUIDO, 2026-08-20 21:40:54-21:41:08; exit 0, 3/3 paquetes (`orbslam3_multi`, `orbslam3_server`, `simulacion_dron`), 13.1 s; continuidad activa corregida tras culling
Prueba vigente: prueba 178, mismo YAML de 176 tras primera correccion, `success=false`, runner/tool exit 124 a 1100 s; server RSS max 187.2 MiB, group RSS max 1445.7 MiB, PSI memoria 0, `guard_triggered=false`. Log completo preservado y no leido: `codex/archivos_auxiliares/logs/prueba_178.log`
Prueba preparada: 179, YAML exacto `codex/archivos_auxiliares/trayectorias/tray_prueba_176.yaml`, launch `ros2 launch simulacion_dron multi_dron.launch.py`, startup 15 s, timeout 1200 s, drenaje 120 s y monitorizacion de recursos
Prueba vigente: prueba 179 completada `success=true`, scenario/tool exit 0, duracion monitorizada 804 s. Recursos estables: `guard_triggered=false`, server RSS max 326.9 MiB, group RSS max 1882.7 MiB, memory PSI 0; log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_179.log`
Interpretacion visual 179: NO CONSEGUIDA. Tras el primer commit fiducial, varias hipotesis loop de ~27 m/~1.61 rad fueron rechazadas, pero la tarea `1000000003083` acepto dos regiones, llevo el residual loop a 0.083 m/0.007 rad y movio 22 KFs sin validar la degradacion de aristas estructurales; RViz2 mostro KFs en posiciones absurdas. Repeticiones posteriores corrigieron el mapa de forma bastante buena mediante nuevos loops/optimizaciones
Build siguiente: CONSEGUIDO, 2026-08-21 00:38:20-00:38:56; exit 0, 3/3 paquetes (`orbslam3_multi`, `orbslam3_server`, `simulacion_dron`), 35.2 s; log `codex/archivos_auxiliares/colcon_build.log`; aviso Drake no bloqueante
Build siguiente: CONSEGUIDO, 2026-08-21 00:40:31-00:40:37; exit 0, 3/3 paquetes, 5.46 s; regresiones nuevas compiladas
Build siguiente: CONSEGUIDO, 2026-08-21 00:42:50-00:42:56; exit 0, 3/3 paquetes (`orbslam3_multi`, `orbslam3_server`, `simulacion_dron`), 5.21 s; unico aviso Drake no bloqueante
Build siguiente: CONSEGUIDO, 2026-08-21 00:46:41-00:47:02; exit 0, 3/3 paquetes, 20.8 s; telemetria y regresion de continuidad compiladas, unico aviso Drake no bloqueante
Prueba preparada: 180, YAML exacto `codex/archivos_auxiliares/trayectorias/tray_prueba_176.yaml`, launch `ros2 launch simulacion_dron multi_dron.launch.py`, startup 15 s, timeout 1200 s, drenaje 120 s y monitorizacion de recursos
Prueba vigente: prueba 180 terminada `success=false`, scenario/tool exit 124 por timeout del runner a 1200 s, con 120 s de drenaje y 1345 s monitorizados; log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_180.log`. Recursos sin guarda: server RSS max 270.1 MiB, grupo RSS max 1696.8 MiB, memory PSI 0, CPU max sistema 56.89 por ciento
Diagnostico prueba 180: primera vuelta completada, pero el paso 15 espero 697.771 s de backpressure y el timeout cancelo el inicio de la segunda. Hubo 70 optimizaciones loop con 70 finales y cero commits: 58 rechazos `hard_corridor_displacement_exceeded`, 11 temporales y uno de covisibilidad; tras un commit fiducial se crearon 191 reruns y se encolaron 184. El worker siguio avanzando, dreno a `pending=0`, proceso 1744 tareas y tuvo cero hard failures; no fue deadlock ni presion de memoria. El corredor rechaza actualmente propuestas por una desviacion absoluta preexistente aunque el loop no la empeore, generando solves redundantes de hasta unos 40 s
Bloque de cambios completado: contrato, validador y telemetria del corredor corregidos para comparar exceso before/after; un exceso heredado solo puede mantenerse o disminuir. Regresion `AllowsImprovementOfPreexistingCorridorExcess` añadida y el caso de nuevo salto sigue cubierto
Build siguiente: CONSEGUIDO, 2026-08-21 01:17:20-01:17:54; exit 0, 3/3 paquetes en 33.4 s tras corregir semantica del corredor; unico aviso Drake no bloqueante
Tests siguientes: CONSEGUIDOS, regresiones opuestas del corredor 2/2: nuevo salto rechazado y exceso heredado decreciente aceptado
Tests siguientes: CONSEGUIDOS, CTest completo `orbslam3_multi` 9/9 en 3.74 s tras la correccion del corredor
Tests siguientes: CONSEGUIDOS, servidor funcional 4/4 en 0.33 s y contrato web 1/1 en 1.03 s
Prueba preparada: 181, repeticion exacta de 180 con `tray_prueba_176.yaml`, launch normal, startup 15 s, timeout 1200 s, drenaje 120 s y monitorizacion de recursos; criterio principal: eliminar espera de backpressure de 697.771 s y obtener accepts loop sin degradacion estructural
Prueba vigente: prueba 181 terminada `success=false`, scenario/tool exit 124 a 1200 s con 120 s de drenaje y 1345 s monitorizados; log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_181.log`. Recursos estables y mejores que 180: server RSS max 190.8 MiB, grupo RSS max 1442.0 MiB, memory PSI 0, guarda inactiva
Diagnostico prueba 181: el paso 8 quedo mas de 1006 s esperando backpressure. Hubo 267 `F3Q-OPT-START`, 266 finales antes del cierre, cero commits y 266 `revision_conflict`; las propuestas ya eran aceptadas por estructura, pero el stale se reencolo sin limite como exige el contrato. `CommitGraphProposal()` relee raw/poses vigentes bajo `state_commit_mutex_` y valida drift raw, pero despues pasa una expectativa de `pose_revision` redundante dentro de la misma seccion serializada, causando conflicto determinista
Bloque de cambios completado: commit loop documentado y ajustado para usar las poses vigentes rebasadas bajo `state_commit_mutex_` con `expected_pose_revision=0`; siguen obligatorios revision/drift raw, controles activos, hard y atomicidad multi-submapa
Build siguiente: CONSEGUIDO, 2026-08-21 01:45:48-01:46:01; exit 0, 3/3 paquetes en 12.3 s tras eliminar expectativa duplicada de revision loop
Tests siguientes: CTest integrado no iniciado porque el entorno rechazo la escalada al alcanzarse el limite de uso de aprobaciones; no es un fallo del proyecto. Binarios directos CONSEGUIDOS: grafos/validacion 14/14, pipeline loop 9/9 y cola secundaria 5/5
Prueba vigente: prueba 182 corta terminada `success=false`, scenario/tool exit 124 a 480 s con 60 s de drenaje y 564 s monitorizados; log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_182.log`. Recursos estables: server RSS max 186.1 MiB, grupo RSS max 1387.0 MiB, memory PSI 0, guarda inactiva
Siguiente accion exacta: inspeccionar exclusivamente los artefactos reducidos/divididos de 182 y localizar el motivo terminal actualizado antes de cualquier nuevo cambio
Diagnostico prueba 182: el paso 8 espero el gate hasta el timeout; hubo tres commits loop iniciales y despues 117 `revision_conflict` con propuestas estructuralmente aceptadas. El conflicto aparece al crecer el solve a 1-3 s, pero el resultado anterior no distingue control inactivo, drift raw u otra precondicion
Bloque de cambios completado: telemetria mecanica de commit añadida mediante `AcceptedPoseBatchResult::detail`; no cambia la politica funcional y permite separar la causa exacta en la siguiente prueba corta
Siguiente accion exacta: compilar paquetes afectados, ejecutar regresiones y repetir el escenario corto 182 con un ID nuevo para diagnosticar el precondicionante concreto
Build siguiente: CONSEGUIDO, 2026-08-21 02:00:30-02:01:07; exit 0, 2/2 paquetes (`orbslam3_multi`, `orbslam3_server`) en 36.2 s; unico aviso Drake no bloqueante
Siguiente accion exacta: ejecutar regresiones funcionales directas de grafos, loop y cola antes de la prueba corta de diagnostico
Tests siguientes: CONSEGUIDOS tras telemetria; grafos/validacion 14/14, pipeline loop 9/9 y cola secundaria 5/5. El primer intento busco el binario de cola en el paquete equivocado y se corrigio mecanicamente sin cambio de codigo
Prueba preparada: 183, repeticion del YAML corto `tray_prueba_182.yaml`, launch normal, startup 15 s, timeout 480 s, drenaje 60 s y monitorizacion de recursos; objetivo diagnostico: contar `commit_*` y confirmar el precondicionante que produce stale sostenido
Siguiente accion exacta: ejecutar prueba 183 y registrar su resultado bruto antes de reducirla
Prueba vigente: prueba 183 intento 1 NO EJECUTADA funcionalmente; runner/tool exit 1 porque el YAML relativo no se resolvio desde el workspace del script. El launch arranco y cerro, recursos estables y guarda inactiva; log preservado y no leido en `codex/archivos_auxiliares/logs/prueba_183.log`
Correccion mecanica: repetir como prueba 184 con ruta YAML absoluta, conservando escenario, tiempos y criterios y sin sobrescribir el artefacto 183
Siguiente accion exacta: ejecutar prueba 184 con `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/tray_prueba_182.yaml`
Prueba vigente: prueba 184 terminada `success=false`, scenario/tool exit 124 a 480 s con 60 s de drenaje y 565 s monitorizados; log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_184.log`. Recursos estables: server RSS max 183.2 MiB, grupo RSS max 1389.2 MiB, memory PSI 0 y guarda inactiva
Siguiente accion exacta: reducir/dividir 184 y contar los motivos `commit_*`, gate, commits y estado final de cola sin abrir el log completo
Diagnostico prueba 184: 33 intentos 3Q terminales en el reducido, con 1 commit, 32 `commit_control_missing_or_inactive` y 1 `commit_control_raw_drift`; el gate se libero una vez tras 34.032 s y quedo esperando de nuevo desde el paso 6. La causa dominante es culling/refinado de controles intermedios durante solves concurrentes, no estructura, store ni recursos
Correccion funcional acordada aplicada al contrato: rebase sobre controles actuales, omitiendo solo intermedios caducados; extremos loop, fixed/hard y al menos dos controles activos por submapa siguen obligatorios
Siguiente accion exacta: implementar el filtrado de controles intermedios en `CommitGraphProposal`, exponer su conteo, compilar y repetir regresiones/prueba corta
Bloque de cambios completado: `CommitGraphProposal` conserva obligatorios extremos `CurrentLoop` y controles fixed, omite solo controles intermedios inactivos o con drift raw excesivo, exige dos apoyos actuales por submapa y expone `rebased_skipped_controls` hasta `[F3Q-LOOP-OPT]`
Siguiente accion exacta: compilar `orbslam3_multi` y `orbslam3_server`, registrar resultado y ejecutar regresiones
Build siguiente: CONSEGUIDO, 2026-08-21 02:15:06-02:15:43; exit 0, 2/2 paquetes en 36.4 s; unico aviso Drake no bloqueante
Siguiente accion exacta: ejecutar regresiones funcionales directas de grafos/validacion, pipeline loop y cola secundaria
Tests siguientes: CONSEGUIDOS tras rebase selectivo; grafos/validacion 14/14, pipeline loop 9/9 y cola secundaria 5/5
Prueba preparada: 185, repeticion exacta funcional de 184 con YAML corto absoluto, launch normal, startup 15 s, timeout 480 s, drenaje 60 s y recursos; criterios: recorrido completo, commits con `rebased_skipped>0`, ausencia de stale sostenido/hard failure y gate liberado
Siguiente accion exacta: ejecutar prueba 185 y registrar resultado bruto antes de reducir
Prueba vigente: prueba 185 completada `success=true`, scenario/tool exit 0, 485 s monitorizados incluyendo 60 s de drenaje; log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_185.log`. Recursos estables: server RSS max 193.0 MiB, grupo RSS max 1395.4 MiB, memory PSI 0, guarda inactiva
Siguiente accion exacta: reducir 185 y verificar gates, commits, `rebased_skipped`, stales requeridos, hard failures y estado final de cola
Diagnostico prueba 185: escenario completo y 5 commits 3Q, uno con 5 controles intermedios omitidos; 78 retries terminaron `commit_required_control_missing_or_inactive` y uno por coste. El ultimo barrido espero 170.536 s; al cierre seguian 15 tareas y una optimizacion activa. El motivo requerido agrupa desaparicion real con mero culling, y el caso observado mantiene el raw disponible: conservar ese control como apoyo virtual evita el storm sin escribir ni reactivar el KF
Correccion funcional acordada aplicada al contrato: extremos loop/fixed culled siguen como apoyo virtual solo si existen y su raw no deriva; el commit exige dos controles activos por submapa y escribe unicamente KFs activos
Siguiente accion exacta: implementar apoyo virtual culled, telemetria separada, compilar y repetir regresiones/prueba corta
Bloque de cambios completado: controles culled con raw estable se conservan como correcciones virtuales, `active_corrections>=2` es obligatorio por submapa, el batch sigue omitiendo KFs inactivos y `[F3Q-LOOP-OPT]` expone `rebased_inactive`
Siguiente accion exacta: compilar paquetes afectados y ejecutar regresiones funcionales
Build siguiente: CONSEGUIDO, 2026-08-21 02:26:58-02:27:35; exit 0, 2/2 paquetes en 35.9 s; unico aviso Drake no bloqueante
Siguiente accion exacta: ejecutar 28 regresiones funcionales directas
Tests siguientes: CONSEGUIDOS tras apoyo virtual culled; grafos/validacion 14/14, pipeline loop 9/9 y cola secundaria 5/5
Prueba preparada: 186, repeticion exacta del escenario corto 185, timeout 480 s, drenaje 90 s y recursos; criterios: success, fuerte reduccion del gate/stale storm, commits con `rebased_inactive>0`, cero hard failures y cola descendente
Siguiente accion exacta: ejecutar prueba 186 y registrar resultado bruto
Prueba vigente: prueba 186 terminada `success=false`, scenario/tool exit 124 a 480 s con 90 s de drenaje y 594 s monitorizados; log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_186.log`. Recursos estables: server RSS max 179.4 MiB, grupo RSS max 1366.5 MiB, memory PSI 0, guarda inactiva
Siguiente accion exacta: reducir 186 por separado para escenario, 3Q, backpressure y cierre, y localizar la nueva causa dominante
Diagnostico prueba 186: primer gate reducido a 3.203 s y desaparecen los stales de control; 78 commits 3Q de 81 intentos, todos ellos con apoyo virtual culled (`rebased_inactive` total 572, max 9), vuelven a mover en promedio 211.3 KFs. Cada commit reencola esos KFs y los reruns destinados a fusion vuelven a optimizar, creando realimentacion; el paso 6 queda 332 s en gate hasta timeout. No hay hard failures ni presion de recursos
Correccion funcional acordada aplicada al contrato: los reruns post-opt son `FusionRefresh`; conservan deteccion/fusion/score pero no pueden lanzar otra optimizacion. Deltas/snapshots y retries normales mantienen intencion `Full`
Siguiente accion exacta: implementar la intencion de tarea, preservar `Full` al coalescer y añadir regresion de error alto no recursivo
Bloque de cambios completado: `LoopTaskIntent::{Full,FusionRefresh}` implementado; reruns post-opt de loop/fiducial son refresh, retries preservan intent, la cola conserva `Full` al coalescer y backend difiere error alto de refresh sin optimizar. Regresiones de backend y cola añadidas
Siguiente accion exacta: compilar paquetes afectados y ejecutar regresiones ampliadas
Build siguiente: CONSEGUIDO, 2026-08-21 02:41:02-02:41:36; exit 0, 2/2 paquetes en 33.7 s; unico aviso Drake no bloqueante
Siguiente accion exacta: ejecutar regresiones ampliadas de grafo, pipeline y cola
Tests siguientes: CONSEGUIDOS; grafos/validacion 14/14, pipeline loop 9/9 incluida evidencia alta `FusionRefresh` no recursiva, cola 6/6 incluida precedencia `Full`
Prueba preparada: 187, repeticion exacta de 186 con escenario corto, timeout 480 s, drenaje 90 s y recursos; criterios: success, ningun commit originado por `FusionRefresh`, gate acotado, cero hard failures y cola drenada o claramente descendente
Siguiente accion exacta: ejecutar prueba 187 y registrar resultado bruto
Prueba vigente: prueba 187 completada `success=true`, scenario/tool exit 0, 342 s monitorizados incluyendo 90 s de drenaje; log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_187.log`. Recursos estables: server RSS max 182.6 MiB, grupo RSS max 1351.2 MiB, memory PSI 0, guarda inactiva
Siguiente accion exacta: dividir/reducir 187 y verificar gates, intents, optimizaciones, refresh diferidos, hard failures y cola final
Diagnostico prueba 187: CONSEGUIDA para el escenario corto. Recorrido completo; gates de 5.604 s y 10.109 s; 3 optimizaciones/3 commits frente a 78 en 186; 16 `FusionRefresh` con error alto diferidos; 1047 tareas completadas y ultimo done `pending=0`; cero hard failures/errores graves. Un ID solicitado como refresh inicio 3Q porque se coalescio con una `Full` pendiente, comportamiento correcto pero ambiguo en el log de enqueue
Bloque de cambios completado: telemetria mecanica añade el `intent` efectivo a `[F3H-SECONDARY-START]` y `[F3Q-OPT-START]`
Siguiente accion exacta: compilar, repetir regresiones y ejecutar la prueba larga exacta 176/179 como validacion integral final
Build siguiente: CONSEGUIDO, 2026-08-21 02:49:33-02:49:50; exit 0, 2/2 paquetes en 17.0 s tras telemetria de intent; unico aviso Drake no bloqueante
Siguiente accion exacta: repetir regresiones funcionales antes de la simulacion larga
Tests siguientes: CONSEGUIDOS tras telemetria final; ejecuciones directas sin flags pasan 14/14, 9/9 y 6/6. El flag `--gtest_brief=1` no esta soportado por esta version y solo mostro ayuda con exit 0; no se contabiliza como ejecucion adicional
Prueba preparada: 188, repeticion exacta del YAML largo `tray_prueba_176.yaml`, launch normal, startup 15 s, timeout 1200 s, drenaje 180 s y recursos; criterios integrales: scenario success, intents efectivos correctos, optimizaciones/fiduciales sin storms, estructura protegida, cero hard failures y cola drenada/descendente
Siguiente accion exacta: ejecutar prueba larga 188 y registrar resultado bruto antes de reducir
Tests siguientes: NO CONSEGUIDOS, CTest `orbslam3_multi` 8/9; `test_loop_pipeline` ya pasa tras adaptar la pareja fusionada, pero fallan las tres regresiones nuevas de `test_fiducial_optimization` porque construyen errores loop sin incluir la correspondiente arista `CurrentLoop` y el validador responde correctamente `relative_loop_error_coverage_mismatch`; es un defecto mecanico del fixture, no del codigo productivo
Tests siguientes: CONSEGUIDOS para 3Q; CTest `orbslam3_multi` 9/9 en 3.67 s, incluidas las tres regresiones estructurales nuevas. `pipeline_flow_contract` paso dentro del CTest de `simulacion_dron`; el CTest global termino 3/7 por deuda historica ajena en `flake8`, `lint_cmake`, `pep257` y `uncrustify`, sin fallos del contrato web ni de codigo modificado por 3Q
Tests siguientes: CONSEGUIDO, `pipeline_flow_contract` aislado 1/1 en 1.02 s
Tests siguientes: CONSEGUIDO, regresion dirigida `RejectsFarRepeatedZoneAfterAnchoredEpochLoss` 1/1 en 34 ms; el epoch perdido queda sin anchor ante una hipotesis repetida a unos 190 m
Tests siguientes: CONSEGUIDOS, CTest completo `orbslam3_multi` 9/9 en 3.73 s tras la telemetria y la regresion de continuidad
Tests siguientes: funcionalidad de `orbslam3_server` CONSEGUIDA 4/4 dentro del CTest global; total 8/10 porque fallan solo linters historicos `flake8` y `uncrustify` sobre `legacy2` y formato previo ajeno al alcance 3Q
Tests siguientes: CONSEGUIDOS, cuatro tests funcionales de `orbslam3_server` aislados 4/4 en 0.35 s
Checkpoint de reanudacion: contexto fisico releido tras compactacion el 2026-08-21; el usuario autorizo expresamente documentacion, implementacion, build, pruebas y conclusion del acuerdo corrector
Checkpoint de reanudacion: contexto fisico releido de nuevo tras compactacion el 2026-08-21; se retoma en el diagnostico reducido de la prueba 182, sin abrir su log completo
Bloque de cambios completado: contratos 3Q actualizados; continuidad de epochs perdidos, cierre transitivo por fusiones, residuales estructurales, corredor hard-hard, reencolado post-opt y omision por pareja fusionada implementados; fixtures estructurales corregidos para incluir su arista `CurrentLoop`
Bloque de cambios completado: `[F3Q-LOOP-OPT]` expone aristas estructurales/corredor e incrementos maximos; `[F3O-LOOP-DONE]` expone el gate de perdida y sus limites. Regresion integrada creada para impedir que un epoch perdido se ancle a una zona repetida a unos 190 m, manteniendo libre el anclaje del primer epoch
Ultima actualizacion: 2026-08-22
Checkpoint de reanudacion: contexto fisico releido el 2026-08-22 tras interrupcion; se retoma la prueba larga 188 ya iniciada. El usuario informa de cambios propios en `fase45_sandbox/`, `ORB_SLAM3/` y wrapper; quedan fuera del alcance 3Q y no se tocaran ni revertiran
Siguiente accion exacta: recuperar la sesion activa de `run_simulation.sh` para prueba 188 y registrar su resultado bruto al terminar
Prueba vigente: prueba 188 ya finalizo durante la interrupcion; no quedan procesos y sus artefactos dejaron de escribirse a las 03:11. La salida terminal del PTY se perdio, por lo que `success`/exit se reconstruiran desde sublogs reducidos sin abrir `prueba_188.log`
Siguiente accion exacta: dividir y reducir prueba 188, leer solo sublogs/CSV y registrar resultado verificable
Prueba vigente: prueba 188 completada `success=true`, scenario/tool exit 0; las 25 etapas y dos vueltas terminaron. Duracion monitorizada 1249 s incluyendo 180 s de drenaje; log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_188.log`. Recursos: server RSS max 423.4 MiB, grupo RSS max 2014.6 MiB, memory PSI 0, guarda inactiva
Diagnostico preliminar prueba 188: 703 tareas secundarias observadas en el tramo de markers completo (604 loop, 95 DB, 4 fid); 9 optimizaciones loop, todas intent `full` y 9 commits; 55 `FusionRefresh` de error alto diferidos; 3 commits fiduciales. Cero hard failures/errores graves. Gates maximos 109.985 s y 197.754 s por backlog de fusion; tras 180 s la cola seguia drenando y el tail baja 323->310, sin `blocking_failure`
Checkpoint de reanudacion: contexto fisico releido el 2026-08-22 tras la nueva compactacion; la prueba 188 ya esta auditada tematicamente y se retoma el cierre documental. Los cambios propios del usuario en `fase45_sandbox/`, `ORB_SLAM3/` y el wrapper siguen fuera de alcance y no se tocaran ni revertiran
Bloque de cambios completado: historial 3Q, contratos, estado, pipeline y documentacion de paquetes sincronizados con las pruebas 177-188; fallos intermedios conservados y estado tecnico marcado conseguido
Prueba preparada: 189, YAML absoluto `codex/archivos_auxiliares/trayectorias/tray_prueba_176.yaml`, launch `ros2 launch simulacion_dron multi_dron.launch.py`, startup 15 s, timeout 1200 s, post-scenario 180 s y recursos; no reducir ni analizar por peticion expresa del usuario
Prueba vigente: 189 interrumpida manualmente por peticion del usuario tras observar un dron detenido; la sesion termino por Ctrl-C (exit terminal 130) y limpio Gazebo/RViz2/web. Log preservado sin lectura ni reduccion en `codex/archivos_auxiliares/logs/prueba_189.log`
Checkpoint de reanudacion: contexto fisico releido tras compactacion el 2026-08-22; se retoma exclusivamente el diagnostico ya reducido de 189, sin cambios de codigo
Diagnostico prueba 189: no hubo deadlock ni agotamiento de recursos. El paso 19 nunca envio goal porque `backpressure=true` durante unos 358.8 s. Cinco hipotesis `Full` consecutivas de KFs vecinos en una zona repetitiva lanzaron solves de 63-70 s y fueron rechazadas solo despues por `temporal_structure_degraded`. El fiducial MAX espero de forma no preemptiva a la tarea activa, despues optimizo y comprometio correctamente, pero genero 520 `FusionRefresh` post-opt y elevo la cola hasta 619. La cola seguia drenando hacia unas 455 tareas al interrumpir; la histeresis high=64/low=16 mantuvo el gate cerrado. La apariencia web continua correspondia a trabajos secuenciales, no a una unica optimizacion colgada
Siguiente accion exacta: explicar este diagnostico y acordar una correccion de admision/deduplicacion de hipotesis repetidas y de semantica de backpressure para mantenimiento post-opt, sin tocar aun el codigo
Conclusion de debate 189: el pipeline no clasifica explicitamente una zona como repetitiva. La cache negativa actual solo recuerda una pareja KF+revisiones que fallo RANSAC/geometria; un rechazo post-solver por degradacion temporal/covisible/prior/hard no se propaga a KFs vecinos. Las parejas exactas con `ServerLoopGeometric` si se excluyen antes de RANSAC y `FusedLandmarkManager` conserva idempotencia, pero una tarea `FusionRefresh` no se cancela completa porque puede encontrar candidatos nuevos. Los refresh de error alto no reoptimizan, aunque todos cuentan en la cola y pueden mantener backpressure
Acuerdo corrector 189 autorizado: precheck tras RANSAC y antes del grafo; regiones protegidas por hard/corredor y vecinos temporal/covisibles acotados; comparar relacion medida frente a world con margen inicial 5 m/20 grados; rechazo temprano solo cuando ambos lados sean estables, conservando optimizacion asimetrica con lado no fiable; `LoopRejectionLedger` regional/revisionado; `FusionRefresh` limitado a proximidad de subnubes, agrupado por region y sin mantener `stop_drones` por backlog de mantenimiento; optimizaciones reales siguen parando drones. Dudas abiertas: ninguna
Siguiente accion exacta: leer contratos tematicos 3Q, estado git y fragmentos precisos de pipeline/cola/backpressure antes de editar documentacion y codigo
Bloque de cambios completado: contratos 3Q ampliados; precheck protegido y ledger regional añadidos al backend; `FusionRefresh` filtra por AABB world de subnubes y se agrupa por region temporal; cola secundaria separa pending critico/mantenimiento y el gate usa solo el critico; telemetria y regresiones dirigidas añadidas. No se tocaron cambios del usuario en ORB/wrapper/fase45
Archivos criticos modificados: `orbslam3_multi/include/orbslam3_multi/{loop_pipeline,sparse_global_backend}.hpp`, `src/{loop_pipeline,sparse_global_backend}.cpp`, tests de loop; `orbslam3_server/include/orbslam3_server/secondary_queue.hpp`, `src/global_map_server.cpp` y test de cola; contratos `subfase_3Q_*`
Siguiente accion exacta: ejecutar comprobacion estatica, compilar `orbslam3_multi orbslam3_server simulacion_dron` con `build_selected_packages.sh` y registrar el resultado antes de pruebas
Build correccion 189: CONSEGUIDO el 2026-08-22 13:16:45-13:17:33; exit 0, 3/3 paquetes (`orbslam3_multi`, `orbslam3_server`, `simulacion_dron`) en 47.0 s; log completo preservado en `codex/archivos_auxiliares/colcon_build.log`, no leido; avisos visibles de Drake/override no bloqueantes
Siguiente accion exacta: ejecutar regresiones dirigidas de loop/backend y cola secundaria, registrar resultados y corregir solo fallos mecanicos antes de la simulacion
Tests correccion 189 intento 1: NO CONSEGUIDOS globalmente, 29/30 pasan. `test_sparse_global_backend` 11/11 y `test_secondary_queue` 7/7; `test_loop_pipeline` 11/12 porque una regresion antigua esperaba `fusion_refresh_optimization_deferred`, pero el nuevo contrato correcto termina antes como `fusion_refresh_no_spatial_candidates`. No es fallo productivo; el fixture debe aceptar la salida espacial temprana acordada
Siguiente accion exacta: adaptar mecanicamente esa expectativa, recompilar paquetes afectados y repetir las tres regresiones
Build correccion 189 intento 2: CONSEGUIDO el 2026-08-22 13:18:15-13:18:23; `orbslam3_multi` 1/1, exit 0, 7.2 s; log completo reemplazado/preservado por la herramienta y no leido; avisos de underlay/Drake no bloqueantes
Siguiente accion exacta: repetir `test_loop_pipeline`, `test_sparse_global_backend` y `test_secondary_queue`, y registrar el total antes de preparar simulacion
Tests correccion 189 intento 2: CONSEGUIDOS, 30/30 (`test_loop_pipeline` 12/12, `test_sparse_global_backend` 11/11, `test_secondary_queue` 7/7). Cubren rechazo protegido sin builder, ledger vecino, lado no fiable permitido, filtro/agrupado refresh y pending mantenimiento separado
Tests integrados correccion 189: `orbslam3_multi` CONSEGUIDO 9/9 en 6.53 s. El CTest global de `orbslam3_server` confirma los tests funcionales visibles y conserva deuda historica de lint en `legacy2`; se aisla la bateria funcional para registrar un resultado limpio sin tocar codigo ajeno.
Checkpoint de reanudacion: contexto fisico releido tras compactacion el 2026-08-22; implementacion, build y 30/30 regresiones dirigidas siguen vigentes.
Tests integrados correccion 189: funcionalidad aislada de `orbslam3_server` CONSEGUIDA 4/4 en 0.44 s; el primer intento sandbox no ejecuto tests porque CTest no pudo escribir `LastTest.log`, y la repeticion autorizada fue limpia.
Prueba preparada: 190, repeticion exacta del YAML largo absoluto `/home/chenfu/Gazebo/src/codex/archivos_auxiliares/trayectorias/tray_prueba_176.yaml`, launch `ros2 launch simulacion_dron multi_dron.launch.py`, startup 15 s, timeout 1200 s, drenaje 180 s y monitorizacion de recursos. Criterios: recorrido completo, rechazo protegido previo al solver, ledger regional, ausencia de storms largos, mantenimiento sin cerrar el gate, cero hard failures y cola drenada o descendente.
Prueba 190: NO EJECUTADA funcionalmente; el helper rechazo antes de lanzar el alias inexistente `--scenario-yaml`. Correccion mecanica: usar las opciones actuales `--yaml` y `--timeout-sec` y repetir sin sobrescribir como prueba 191.
Prueba vigente: 191 completada `success=true`, scenario/tool exit 0 y dos vueltas terminadas; 1046 s monitorizados incluidos 180 s de drenaje. Recursos sin guarda: server RSS max 362.1 MiB, grupo RSS max 1948.8 MiB, memory PSI full/some 0, MemAvailable minima 4829.5 MiB. Log completo preservado y no leido en `codex/archivos_auxiliares/logs/prueba_191.log`.
Diagnostico prueba 191: correccion funcional CONSEGUIDA para el bloqueo 189. Hay 5 rechazos protegidos previos y 42 hits del ledger; 18 refresh terminan sin candidatos espaciales. Los 13 commits loop/fiducial post-opt agrupan 2443 KFs movidos en 195 regiones y encolan 185 refresh. Ningun evento activa backpressure por mantenimiento puro; cierre `pending=0`, 2104 procesadas y `hard_failed=0`. Ocho gates se liberan, maximo 80.272 s, frente a 358.8 s sin progreso en 189.
Limitacion prueba 191: aun hubo 40 solves loop, 9 commits y 31 rejects; dos loops lejanos de unos 20-22 m tardaron unos 17 s porque solo el candidato era protegido y se respeto la asimetria acordada. Una ventana de 786 KFs tardo 83.44 s pese a ambos lados protegidos porque la relacion directa medida tenia error casi cero y otra region llevo a optimizacion; queda como seleccion/umbral futuro, no como fallo del precheck lejano. Recursos estables y recorrido completo.
Documentacion correccion 189: contratos 3Q, docs de `orbslam3_multi`/`orbslam3_server`, historial cronologico/resumen/indice, estado, pipeline y ultima sesion sincronizados con 189 y 191. `git diff --check` pasa. Fallos anteriores y coste residual no se ocultaron.
Conclusion visual 191: el usuario valora muy positivamente el resultado y decide avanzar. 3Q queda `A REVISAR`, con punto de reentrada en seleccion multi-region, umbrales, proteccion regional y admision previa al solver si reaparecen sintomas.
Trabajo activo: ninguno; no hay implementacion ni prueba pendiente en 3Q.
Siguiente accion exacta: preparar la siguiente subfase cuando el usuario la indique, conservando 3Q como punto de reentrada futuro
```

## Acuerdo 3B Ejecutado

- `orbslam3_server/legacy2/` conservara el estado actual de `src/`, `include/`
  y `launch/`, junto con copias renombradas de `CMakeLists.txt` y `package.xml`
  para que Colcon no lo descubra como otro paquete.
- `orbslam3_multi/legacy2/` conservara el estado actual de `src/` e `include/`,
  incluidos tests y ejecutables auxiliares, y copias renombradas de sus archivos
  de paquete. La carpeta `legacy/` existente no se movera ni duplicara.
- Cada snapshot tendra un README/manifiesto que explique origen, fecha,
  contenido y prohibicion de compilarlo desde los targets activos.
- El `orbslam3_multi` activo quedara como paquete `ament_cmake` vacio y
  compilable, sin biblioteca placeholder ni clases de las subfases 3C-3P.
- El `orbslam3_server` activo quedara como un nodo ROS 2 minimo: inicializa,
  permanece vivo y termina limpiamente, sin subscriptions, publishers, timers,
  mapa, fiduciales, loops, backpressure ni ground truth.
- El launch publico del servidor se reducira al nodo vacio. El launch de
  simulacion solo se adaptara donde sea necesario para no pasar argumentos
  obsoletos al servidor.
- En `codex/contexto/paquetes/orbslam3_server/legacy2/` y
  `codex/contexto/paquetes/orbslam3_multi/legacy2/` se archivara una copia de
  los MD actuales. En la raiz quedaran exclusivamente resúmenes y documentos
  reescritos para los archivos activos: resumen del paquete vacio en
  `orbslam3_multi`, y resumen, nodo vacio y launch minimo en el servidor.
- No se modificaran `ORB_SLAM3`, `orbslam3_ros2`, `orbslam3_msgs`, algoritmos ni
  contratos de subfases posteriores; tampoco se mantendra un fallback runtime
  al servidor anterior.
- Validacion ejecutada: build de los tres paquetes afectados y prueba 77 con
  un YAML nuevo de espera corta, sin RViz2/web, comprobando arranque limpio del
  servidor, ausencia de salidas sparse globales y cierre sin crash.

## Implementacion 3B Completada

- Los arboles `src/include/launch` anteriores y sus metadatos quedaron
  congelados en ambos `legacy2`; los cambios no confirmados que habia en disco
  forman parte de la instantanea.
- Los MD anteriores se archivaron en los dos `legacy2` documentales y la raiz
  describe unicamente el estado activo.
- `orbslam3_multi` es ahora un paquete `ament_cmake` sin targets ni codigo.
- `orbslam3_server` compila solo un nodo `rclcpp` vacio y su launch declara solo
  `use_sim_time`.
- `multi_dron.launch.py` ya no declara ni reenvia parametros del servidor
  congelado; conserva controles independientes para RViz2 y visualizador web.
- Se creo `tray_prueba_77.yaml` con un unico `wait` de 5 s y sin movimientos.
- Comprobaciones estaticas: no existen metadatos Colcon activos dentro de
  `legacy2`, no quedan referencias runtime antiguas y ambos launches parsean.
- Build 2026-08-10 13:19:50-13:19:57: comando seleccionado sobre
  `orbslam3_multi orbslam3_server simulacion_dron`, exit code 0, tres paquetes
  finalizados. Log preservado:
  `codex/archivos_auxiliares/logs/colcon_build_3B_incremental.log`. Unica
  advertencia visible: `CMAKE_PREFIX_PATH` contiene `/opt/drake/share/drake`
  inexistente; no afecta al resultado.
- Prueba 77, intento 1, 2026-08-10 13:20:28: YAML
  `codex/archivos_auxiliares/trayectorias/tray_prueba_77.yaml`, launch acordado,
  startup 15 s, timeout 60 s y post-wait 3 s. `scenario_runner_node` devolvio 1,
  la herramienta devolvio 1 y `success=false`; el launch fue cerrado por la
  herramienta. Log completo no leido:
  `codex/archivos_auxiliares/logs/prueba_77_intento_1.log`.
- Diagnostico reducido del intento 1: el servidor emitio
  `[F3B-EMPTY-SERVER-INIT]` y termino limpiamente. El fallo fue exclusivamente
  `Could not load scenario YAML ... bad file`: la ruta relativa se resolvio
  desde el workspace padre. La correccion es mecanica y conserva el acuerdo:
  repetir con ruta absoluta. El error 255 de Gazebo aparece durante el cierre
  forzado posterior al fallo del runner, no como causa inicial.
- Prueba 77, intento 2, 2026-08-10 13:21:57: mismo launch y tiempos, YAML
  absoluto. `scenario_runner_node` devolvio 0, la herramienta devolvio 0 y
  `success=true`. Log completo no leido:
  `codex/archivos_auxiliares/logs/prueba_77.log`.
- Analisis reducido del intento 2: marcador de inicio unico, scenario de wait
  completado, servidor terminado limpio y ausencia de actividad sparse global
  del servidor. Los deltas visibles son de wrappers locales. El exit 255 de
  Gazebo ocurre tras `SIM-DONE`, durante cleanup.
- Auditoria post-build: el install incremental aun contiene symlinks obsoletos
  a cabeceras/tests/corrector/launch antiguo. No se ejecutaron, pero deben
  limpiarse como artefactos generados para que el install refleje el runtime
  activo vacio.
- Rebuild limpio 2026-08-10 13:31:52-13:31:59: se eliminaron solo build/install
  generados de ambos paquetes y se recompilaron junto con `simulacion_dron`;
  exit code 0, tres paquetes finalizados. Log vigente:
  `codex/archivos_auxiliares/logs/colcon_build.log`; el build incremental
  anterior queda en `colcon_build_3B_incremental.log`.
- Auditoria final de install: solo se instala el ejecutable/launch activo del
  servidor; `orbslam3_multi` no instala codigo ni targets. Colcon lista una sola
  instancia de cada paquete y no descubre ningun `legacy2`.

## Ampliacion Visual 3B Ejecutada

- Snapshot completado en
  `simulacion_dron/legacy2/pipeline_flow_visualizer/`: bridge, web completa,
  launch y metadatos CMake/package anteriores.
- Documentacion anterior copiada a
  `codex/contexto/paquetes/simulacion_dron/legacy2/`.
- No se ha creado publisher, subscription ni arista wrapper-servidor.
- Bridge activo: primera conexion SSE empieza en el presente; reconexion usa
  `Last-Event-ID`; un cursor expirado produce `state_reset`; `/health` expone
  modo live y secuencia.
- Frontend activo: dos nodos (`wrappers`, `server`), cero aristas, drenaje por
  `requestAnimationFrame`, conteo de gaps y estado debug consultable.
- Test directo `test_pipeline_flow_contract.py`: `4 passed`.
- Se creo `tray_prueba_78.yaml`: espera de tracking, dos goals simultaneos a
  fiducial 2 y espera visual de 30 s.
- Se creo `CONTRATO_VISUAL_INCREMENTAL.md` y se sincronizaron los contratos
  principales 3C-3X, extensiones 3F/3K y subdocumentos de 3O/3P/3Q/3S.
- 3U pasa a auditoria/hardening final; no es creadora tardia del visualizador.
- Documentacion activa de `simulacion_dron` describe solo la base 3B y enlaza
  el snapshot legacy2.
- Build 2026-08-10 14:02:39-14:02:45: tres paquetes finalizados, exit code 0.
  Log completo no leido: `codex/archivos_auxiliares/logs/colcon_build.log`.
  Aviso no bloqueante: prefijo Drake inexistente.
- Test integrado CMake `pipeline_flow_contract`: 1/1 passed, 0 failed, 1.46 s.
- Validacion runtime aislada: el primer arranque revelo que
  `pipeline_flow_bridge.py` no conservaba permiso ejecutable; se corrigio de
  forma mecanica y el rebuild de `simulacion_dron` termino con exit code 0.
- `/health` en puerto temporal 8878 devolvio `status=ready`, `mode=live`,
  capacidad 512, secuencia 0 y `replay_on_first_connect=false`.
- Capturas headless de escritorio 1440x900 y movil 390x844 verificaron dos
  nodos, cero aristas, SSE conectado, cero eventos/gaps y ausencia de solapes.
- Prueba 78 preparada para ejecucion: launch
  `ros2 launch simulacion_dron multi_dron.launch.py`, YAML absoluto
  `tray_prueba_78.yaml`, startup 15 s, timeout global 360 s y post-wait 10 s;
  se mantienen activos los defaults de RViz2 y visualizador web.
- Prueba 78 ejecutada desde 14:12:44: `scenario_runner_node` devolvio 0, la
  herramienta marco `success=true` y termino con exit code 0. Durante la
  ejecucion integrada `/health` siguio en `ready/live`, secuencia 0 y sin
  replay inicial. Log completo preservado y no leido:
  `codex/archivos_auxiliares/logs/prueba_78.log`.
- Analisis reducido de prueba 78: bridge `mode=live topology=2_nodes_0_edges`,
  RViz2 iniciado y cerrado limpio, servidor vacio iniciado, ambos goals
  paralelos correctos en 22 s, espera visual de 30 s y escenario completo.
  No aparecen `global_sparse_map` ni `global_keyframes`; auditoria estatica del
  servidor activo confirma ausencia de publishers, subscriptions y timers.
  El exit 255 de Gazebo es posterior a `SIM-DONE` y pertenece al cleanup.
- Conclusion tecnica: criterios automaticos conseguidos. El usuario confirmó
  posteriormente grafo sin conexiones/actividad y RViz2 sin resultados
  globales; conclusión final 3B: CONSEGUIDA.
- Verificacion final tras cierre documental: CTest
  `pipeline_flow_contract` repetido sobre el build final, 1/1 passed en 2.03 s.
- Auditoria de procesos: no quedan procesos de Gazebo, RViz2, bridge, servidor
  ni launch asociados a la prueba 78.

## Acuerdo Funcional Vigente

### Flujo principal

- El servidor encola deltas/full snapshots; un unico PrimaryWorker procesa una
  entrada completa hasta publish ROS antes de iniciar la siguiente.
- El servidor orquesta y delega el trabajo en clases de `orbslam3_multi`; no
  implementa algoritmos de mapa, loop, fusion u optimizacion.
- `RawMapDatabase` conserva exclusivamente el estado ORB-SLAM3 crudo y devuelve
  un `ChangeSet`/resultado de insercion; no llama directamente a otras bases.
- `GlobalPoseStore` registra en principal los KFs de submapas anclados. Score y
  covisibilidad se actualizan mediante `DatabaseUpdateTask` secundaria HIGH.
- El primer fiducial de `(drone_id, map_epoch)` crea el anchor. Una revisit
  valida puede crear una tarea fiducial prioritaria.
- Los KFs nuevos de un submapa anclado reciben pose world inmediatamente desde
  el ultimo anchor/campo de correccion aceptado, sin esperar al worker
  secundario.
- `GlobalMapBuilder` conserva nube/mensaje de KFs e indices incrementales;
  acumula IDs dirty de raw, poses, scores y fusion y solo recalcula afectados.
- `GlobalMapBuilder` se ejecuta solo dentro de la siguiente tarea principal.
  Un commit secundario notifica dirty sets, pero no despierta, reconstruye ni
  publica por si mismo.
- El flujo principal nunca espera BoW, matching, RANSAC, fusion, grafo, solver,
  HTML ni al visualizador JavaScript.

### Flujo secundario

- Existe un unico worker persistente y una cola con prioridades MAX fiducial,
  HIGH `DatabaseUpdateTask` y NORMAL `LoopTask`.
- Una tarea activa nunca se interrumpe: se ejecuta de inicio a fin.
- Al terminar la activa se elige fiducial, despues update de bases y despues
  loop; FIFO se conserva dentro de cada prioridad.
- Cada KF nuevo/materialmente modificado elegible crea su propia `LoopTask`;
  nunca existe una tarea loop agregada por delta.
- Cada `LoopTask` abarca BoW, filtros baratos, matching/RANSAC cuando proceda,
  decision entre fusion u optimizacion, ejecucion elegida y commit final.
- Fusion y optimizacion por loop no son tareas separadas ni tienen prioridades
  diferentes: son desenlaces de la misma `LoopTask`.
- Toda tarea calcula sobre snapshots privados fuera de locks y termina tras un
  commit breve, validado y atomico en las bases derivadas correspondientes.
- Una tarea secundaria no publica, no despierta al principal y no espera
  `publication_ack`; solo compromete bases y notifica cambios dirty.
- `RawMapDatabase` nunca se modifica por fusion u optimizacion.

### Backpressure

- El flag es OR de high watermark principal/secundario y optimizacion
  fiducial/loop activa, con histeresis.
- El goal activo termina normalmente; mientras el flag siga activo no se envia
  el siguiente goal.
- No se solicitan snapshots periodicos nuevos; los ya en vuelo se conservan y
  al liberar se pide como maximo uno fresco por dron.

### Sincronizacion

- Las lecturas largas usan snapshots inmutables/versionados, no contenedores
  live protegidos durante todo el calculo.
- Los mutex solo protegen cambios breves de cola o commits de estado.
- Las revisiones capturadas se validan antes del commit; un resultado obsoleto
  se descarta o reprograma de forma acotada, sin retry inmediato infinito.
- La publicacion de KFs y nube usa una misma revision coherente y nunca observa
  un lote de poses parcialmente escrito.

### Visualizador JavaScript

- `3B` es propietaria de la infraestructura web JavaScript independiente de
  RViz2; cada subfase añade sus nodos, aristas y eventos reales, y `3U` audita
  y endurece el resultado completo.
- Los eventos se emiten mediante instrumentacion minima a una cola acotada y no
  bloqueante; perder telemetria es preferible a frenar el pipeline.
- Los eventos transportan metadatos, IDs, revisiones, cantidades, tiempos y
  resultado, nunca nubes, descriptores o payloads pesados completos.
- Principal y fiducial se conservan completos; loops y `DatabaseUpdateTask`
  pueden descartarse solo como flujos completos. Un loop descartado que decide
  optimizar se muestra obligatoriamente desde `OPTIMIZATION_DECIDED`.
- La UI ilumina nodos/aristas en vivo y muestra tooltips de responsabilidad y
  datos transferidos. Debe iniciarse con la simulacion, pero su fallo no afecta
  a ROS, RViz2 ni al mapa.
- Es tiempo real de observacion humana, no tiempo real duro.

### Exclusiones

- No usar GT salvo como origen del fiducial simulado, debug o metricas externas.
- No iniciar fases posteriores ni cambiar BoW/RANSAC/optimizacion fuera de la
  reorganizacion necesaria para cumplir este contrato.
- No ampliar el alcance funcional ni cambiar criterios algoritmicos sin cerrar
  un nuevo acuerdo.

## Nuevo orden de pipeline

- Fase 1: control del dron, marcada como realizada en el nuevo pipeline.
- Fase 2: separacion fisica de paquetes, pendiente hasta cerrar Fase 3.
- Fase 3: mapa sparse global; corresponde a la antigua Fase 1 y es la fase
  actual de esta conversacion.
- Fase 4: fiducial real sin GT funcional.
- Fase 5: pose global de cada dron sin ground truth.
- Fase 6: tareas y trayectorias de mapeo.
- Fase 7: GUI 3D propia de operacion.
- Fase 8: nube densa global multi-dron.
- Fase 9: mejoras avanzadas futura; queda como placeholder y sus subfases se
  realizaran cuando se avance a esa fase.
- `PIPELINE_MAESTRO.md`, `AGENTS.md`, `CODEX_INDEX.yaml`, los cuatro MDs de
  arranque/estado, ADR_0004/ADR_0006 y referencias operativas auxiliares ya
  fueron sincronizados el 2026-08-09.
- Busqueda amplia posterior: no quedan referencias operativas a
  `fase_1_sparse_global`, `pipeline_fase_1_RESUMEN` como sparse antiguo,
  `--fase 1L` ni nombres `f1/F1` en los artefactos existentes de
  `codex/archivos_auxiliares/html`, `logs` y `repeticiones`.
- Se corrigieron referencias conceptuales residuales `1B/1C/1G/1L/1N/1O` a
  `3B/3C/3G/3L/3N/3O` en arquitectura, topics, reglas, mapa de codigo,
  pruebas clave, ADR_0003, docs de paquetes y YAMLs de trayectoria.
- Las ocurrencias restantes `F1*`/`f1*` son marcadores, parametros o comandos
  legacy del runtime/historial, o pertenecen a la Fase 1 nueva de control del
  dron; no son identificadores del pipeline sparse activo.

## Investigacion propuesta de problemas runtime

Observaciones aportadas por el usuario mediante un MD temporal:

- el visualizador web parece representar eventos antiguos y puede quedarse
  varios segundos por detras del backend;
- se observaron tareas de loop para KFs de submapas aun no anclados;
- KFs y MapPoints de submapas anclados tardaron demasiado en aparecer en
  RViz2;
- el worker acumulo backlog elevado y se desconoce la edad real de las tareas;
- una correccion aparentemente aceptada del dron antihorario parecio
  desaparecer al final de una ejecucion;
- otra ejecucion no produjo esa optimizacion, indicando no determinismo que
  puede depender de datos, tracking, revisiones o scheduling.

Resultados del diagnostico:

- loops pre-anchor confirmados: `31/66` starts en prueba 75 y `38/84` en 76;
- backlog confirmado: 75 encola `561` y conserva `496` en el ultimo start; 76
  encola `489`, pico `429`, ultimo valor `414`;
- callback bajo mutex: p95 `5.159/4.251 s`, maximo `12.829/17.881 s` en 75/76;
- request->commit RViz2: maximo `20.283/27.951 s`; la captura live domina y el
  publish ROS consume pocos milisegundos;
- perdida raw de optimizacion no confirmada: en 75 hay tres commits de poses
  aceptados posteriores al primer commit fiducial y no hay rollback;
- diferencia de entrada confirmada: 75 tiene `13` observaciones de `fid=1` y
  76 tiene cero;
- replay web confirmado por diseno: cliente `400 x 110 ms`, pulsos `520 ms` y
  reconexion SSE desde cero con hasta `512` eventos antiguos;
- no se puede separar espera de mutex/copia ni medir edad causal exacta de
  tareas con la instrumentacion actual.

No autorizado todavia: cambiar thresholds, solver, scheduling, semantica raw,
numero de threads o arquitectura. Toda correccion funcional requiere acuerdo
nuevo.

## Prueba Acordada Para La Implementacion Posterior

1. Build de `orbslam3_multi`, `orbslam3_server` y `simulacion_dron`.
2. Tests deterministas de ingesta/poses, prioridad de cola, commits atomicos,
   fusion y publicacion coherente.
3. Prueba larga `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` con
   `multi_dron.launch.py`, RViz2 y visualizador JavaScript activos.
4. Validar que el flujo principal sigue recibiendo/publicando mientras una
   tarea secundaria esta activa; que la prioridad observada es
   `tarea activa -> fiducial pendiente -> loops pendientes`; y que cada commit
   aceptado aparece despues en RViz2 sin bloquear el cierre de la tarea.
5. Validar mediante el diagrama los eventos y tooltips sin que su desconexion o
   saturacion afecte a la simulacion.

## Puertas Permanentes

- Leer fisicamente este archivo antes del resto del contexto en un chat nuevo o
  tras compactacion; el resumen automatico no lo sustituye.
- Una primera orden de ejecutar inicia preparacion. Solo una orden posterior al
  acuerdo cerrado concede autorizacion funcional.
- Tras cada cambio, build, test, simulacion o diagnostico de una tarea larga,
  reemplazar este checkpoint inmediatamente.
- Los logs completos solo alimentan reductores; nunca se abren directamente.
- Cada prueba conserva su propia evidencia y conclusion historica.

## Archivos Relevantes

```text
codex/pipeline/fase_3_sparse_global/subfases/subfase_3F_publicacion_reactiva.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3K_worker_secundario.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3C.md a subfase_3H.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3I.md a subfase_3Q_*.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3S_*.md a subfase_3W.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3X.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3P.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3Q_*.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3U.md
codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_general_RESUMEN.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3P_RESUMEN.md
```
