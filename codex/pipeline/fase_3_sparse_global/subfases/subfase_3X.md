# Subfase 3X - Limpieza final, documentacion y handoff de Fase 3

## Estado

```text
PREPARADA; pendiente de implementar y validar 3C-3W
```

`3X` no es propietaria del redisenho de ejecucion. Su funcion es retirar las
rutas sustituidas por los contratos de `3C-3W`, demostrar que ya no se usan y
dejar un handoff verificable. No se puede marcar como conseguida mientras la
arquitectura acordada siga siendo solo documental.

## Objetivo

Cerrar la Fase 3 con una unica ruta runtime comprensible:

```text
flujo principal
  ingesta -> commit raw -> ChangeSet -> poses/covisibilidad/scores -> publicacion

flujo secundario
  cola priorizada -> un worker -> FiducialOptimizationTask o LoopTask -> commit

observabilidad
  eventos ligeros -> visualizador JavaScript, sin gobernar el pipeline
```

El cierre debe eliminar codigo y configuracion ya reemplazados, conservar la
evidencia historica y documentar con precision que esta activo, que es legacy y
que queda pendiente.

## Dependencias obligatorias

Antes de ejecutar `3X` deben haberse validado:

- el contrato raw y `ChangeSet` de `3C/3G`;
- la autoridad de pose y propagacion de KFs futuros de `3D`;
- el anclaje y las revisitas fiduciales de `3E/3H`;
- la publicacion reactiva y coherente de `3F`;
- el worker secundario de `3K`;
- las etapas algorítmicas `3I-3Q` dentro de sus tareas propietarias;
- covisibilidad y score de `3M/3S`;
- los contratos comunes de `3T`;
- el visualizador incremental creado en `3B` y auditado en `3U`;
- la regresion funcional y de rendimiento de `3V/3W`.

No se borra una ruta antigua solo porque exista una sustituta documentada. La
sustituta debe compilar, pasar sus tests y aparecer en la prueba integrada.

## Inventario de limpieza

### Scheduling y tareas

Localizar y retirar, cuando la nueva ruta este validada:

- creacion de un worker por tarea;
- reservas de ventanas y scheduling de solvers simultaneos;
- colas independientes para fusion, loop y optimizacion fiducial;
- la antigua cola FIFO exclusiva de optimizacion;
- cualquier cola especial para KFs posteriores a una optimizacion;
- estados que mantengan una tarea viva hasta que RViz2 publique;
- esperas de confirmacion visual o backpressure ligado a esa confirmacion;
- reintentos stale inmediatos e ilimitados.

Debe quedar un unico `SecondaryTaskWorker`, con prioridad estable, una tarea
activa como maximo y cierre al terminar el commit o el rechazo.

### Ingesta y bases de datos

Retirar:

- llamadas desde `RawMapDatabase` a propietarios derivados;
- scans globales usados para descubrir de nuevo lo ya descrito por `ChangeSet`;
- escrituras de fusion u optimizacion sobre datos raw;
- accesos largos a contenedores live bajo el mutex del servidor;
- commits por elemento que puedan exponer lotes parciales;
- caches duplicadas sin autoridad ni revision definida.

Debe quedar una autoridad unica por dato, conforme a `subfase_3T.md`, y APIs de
snapshot/commit explicitas.

### Publicacion

Retirar:

- reconstrucciones pesadas dentro de callbacks de ingesta;
- timers duplicados que compitan por publicar el mismo estado;
- publicaciones de KFs y MapPoints construidas con revisiones distintas;
- fallback que coloque MapPoints mediante una correccion de submapa si no hay
  un KF world valido que los soporte;
- gates que detengan el pipeline hasta observar una revision en RViz2.

Debe quedar una solicitud reactiva coalescida y un heartbeat de reconciliacion.
El `GlobalMapBuilder` lee snapshots coherentes, aplica poses y tracks vigentes y
publica KFs y nube como salida del flujo principal.

### Loops, fusion y optimizacion

Retirar:

- callbacks o workers privados dentro de BoW, verificacion, decision o fusion;
- conversiones de cada etapa del loop en tareas independientes;
- una segunda tarea creada despues de decidir fusion u optimizacion;
- clases pasarela sin estado, politica o invariantes propios;
- rutas duplicadas para construir o aplicar el mismo grafo.

Una clase candidata solo se elimina tras verificar todas sus referencias. Si
una clase conserva responsabilidad real, se mantiene y se documenta; `3X` no
autoriza borrados por nombre ni refactors esteticos.

### Configuracion y diagnostico

Retirar o renombrar de forma compatible:

- parametros launch de workers simultaneos, reservas de ventana o espera
  visual que ya no tengan consumidor;
- metricas de estados eliminados;
- topics de debug duplicados;
- scripts, tests y comentarios que describan la ruta sustituida como vigente.

Los nombres historicos pueden permanecer en historiales y resultados de
pruebas anteriores, siempre marcados como evidencia de aquella implementacion.

## Metodo seguro de retirada

Para cada elemento candidato:

1. localizar definicion, usos, parametros, tests y documentacion con `rg`;
2. identificar el contrato nuevo que asume su responsabilidad;
3. demostrar esa ruta mediante test unitario o integrado;
4. eliminar el elemento y sus referencias activas en un cambio acotado;
5. compilar los paquetes afectados;
6. repetir los tests relevantes;
7. registrar que se retiro, por que era redundante y que lo sustituye.

No se modifican ni borran historiales para ocultar intentos anteriores. Los
archivos legacy externos al runtime solo se mueven o eliminan con autorizacion
explicita y despues de actualizar su indice.

## Archivos probables

```text
orbslam3_server/src/global_map_server.cpp
orbslam3_server/launch/global_orb_map_server.launch.py
orbslam3_server/CMakeLists.txt
orbslam3_server/package.xml

orbslam3_multi/include/orbslam3_multi/*.hpp
orbslam3_multi/src/*.cpp
orbslam3_multi/CMakeLists.txt

simulacion_dron/launch/*.py
simulacion_dron/web/pipeline_flow/index.html
simulacion_dron/web/pipeline_flow/styles.css
simulacion_dron/web/pipeline_flow/graph_definition.js
simulacion_dron/web/pipeline_flow/app.js

codex/contexto/paquetes/**
codex/contexto/legacy/FASE_3_LEGACY_INDEX.md
codex/archivos_auxiliares/README.md
```

La lista es orientativa. Solo se tocan archivos confirmados por la auditoria y
necesarios para retirar una ruta sustituida o completar el handoff.

## Handoff del visualizador

El cierre debe dejar:

- assets web locales y versionados;
- topologia y textos editables en `graph_definition.js`;
- logica de conexion, animacion y tooltips en `app.js`;
- arranque opcional desde la simulacion, desactivable por parametro;
- direccion y puerto configurables, limitados por defecto a la maquina local;
- instrucciones de ejecucion y diagnostico sin depender de Internet;
- una prueba donde cerrar o desconectar el navegador no altera ROS ni RViz2.

El visualizador es observabilidad. No envia comandos ni participa en commits,
prioridades o decisiones algorítmicas.

## Documentacion final

Actualizar tras las pruebas, sin anticipar resultados:

- `pipeline_fase_3.md` y su resumen;
- `PIPELINE_MAESTRO.md`;
- `01_ESTADO_ACTUAL.md` y su resumen;
- documentacion vigente de cada paquete tocado;
- `FASE_3_LEGACY_INDEX.md`;
- catalogo de datasets y pruebas;
- historiales de las subfases realmente ejecutadas;
- `RESULTADO_FINAL_FASE_3.md` solo cuando exista evidencia de cierre.

La documentacion de paquete debe describir el codigo que queda, no una mezcla
entre la ruta antigua y la prevista. Los detalles extensos de las ejecuciones
permanecen en los historiales.

## Verificacion

### Revision estatica

- no quedan referencias activas a archivos eliminados;
- cada parametro launch tiene un consumidor o esta documentado como legacy;
- una sola clase es propietaria de cada dato y revision;
- no quedan waits visuales ni rutas de scheduling duplicadas;
- los enlaces Markdown y rutas del visualizador existen;
- `git diff --check` no informa errores.

### Build y tests locales

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server simulacion_dron
```

Ejecutar los tests acordados en `3C-3W`, especialmente cola priorizada, commits
atomicos, fusion, optimizacion, snapshots de publicacion y telemetria saturada.

### Prueba integrada

Ejecutar `prueba_tipica_rodeo_edificio_dos_fiduciales.yaml` con RViz2 y el
visualizador JavaScript activos. Debe observarse:

- ingesta y publicacion continuas durante una tarea secundaria lenta;
- orden `tarea activa -> fiducial pendiente -> loops pendientes`;
- una sola tarea secundaria activa;
- commits visibles posteriormente sin que la tarea espere a RViz2;
- KFs y MapPoints movidos desde el estado nuevo;
- diagrama en vivo coherente y prescindible;
- cierre limpio sin referencias a componentes retirados.

## Criterio de exito

`CONSEGUIDA` solo si:

1. la ruta nueva esta implementada y validada de extremo a extremo;
2. cada ruta retirada tiene sustituto probado;
3. no queda scheduling, publicacion o autoridad duplicada activa;
4. build, tests locales y prueba integrada pasan;
5. RViz2 y el visualizador han sido comprobados;
6. la documentacion y el indice legacy coinciden con el repositorio;
7. el resultado final conserva fallos e intentos historicos;
8. el handoff a Fase 2 no deja trabajo funcional de Fase 3 oculto.

`PARCIAL` si la arquitectura funciona pero queda limpieza o evidencia pendiente.
`NO CONSEGUIDA` si una retirada rompe comportamiento, si siguen dos rutas
activas o si la documentacion afirma resultados no demostrados.

## Limites

- No introducir comportamiento funcional nuevo en `3X`.
- No modificar `ORB_SLAM3`, `orbslam3_ros2` ni `orbslam3_msgs` salvo necesidad
  explicita y acordada.
- No borrar datos de prueba ni evidencia historica.
- No iniciar Fase 2 mientras Fase 3 no cumpla su criterio de cierre.
- En la preparacion documental actual no se modifica codigo ni se ejecutan
  build, tests o simulaciones.

## Limpieza Visual Obligatoria

Aplicar `../CONTRATO_VISUAL_INCREMENTAL.md`. Retirar vertices y aristas junto
con el codigo sustituido, comprobar que no quedan IDs huerfanos y conservar el
snapshot/documentacion legacy. No rediseñar el grafo en esta limpieza.
