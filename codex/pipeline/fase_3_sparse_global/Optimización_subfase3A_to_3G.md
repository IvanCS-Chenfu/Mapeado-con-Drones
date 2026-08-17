# Optimización de rendimiento de las subfases 3A a 3G

<!-- HISTORY: true -->

## 1. Propósito de este documento

Este documento reconstruye de forma autónoma y exhaustiva la optimización de
rendimiento realizada al terminar la subfase 3G. Está pensado para poder abrir
otro chat, leer únicamente este archivo y entender:

- qué problema apareció inicialmente;
- qué hipótesis se investigaron;
- qué mediciones se utilizaron;
- qué cambios se hicieron, en qué paquetes y archivos, y por qué;
- qué se probó después de cada cambio;
- qué intentos no funcionaron o resultaron poco relevantes;
- cuánto mejoró el rendimiento;
- qué contratos funcionales se conservaron;
- qué riesgos y límites siguen existiendo al continuar hacia las fases 4 a 9.

La optimización se realizó durante el cierre de 3G, pero afecta a componentes
creados progresivamente entre 3A y 3G. No constituye una nueva subfase ni cambia
el objetivo funcional acordado para ninguna de ellas.

## 2. Resumen ejecutivo

La prueba funcional inicial de 3G fue correcta, pero durante su ejecución el
ordenador llegó a bloquearse o a responder con mucha dificultad. La causa no
fue un único bug ni un `OOM`: era la suma de varias fuentes de presión sobre la
RAM, la swap, el procesador y los tiempos de ejecución del flujo principal.

Los factores más importantes fueron:

1. Cada proceso ORB-SLAM3 cargaba un vocabulario visual muy grande y mantenía
   cerca de 1 GiB de memoria privada proporcional por dron.
2. El servidor conservaba y copiaba mapas completos en varias etapas del
   procesamiento de deltas y snapshots.
3. El journal de `.record` retenía en RAM todas las entradas aunque también se
   fueran a escribir en disco.
4. El replay cargaba el archivo completo antes de procesarlo y podía inundar la
   cola principal.
5. La actualización de scores y la construcción de la vista global hacían
   muchas búsquedas, copias, locks y transformaciones por entidad.
6. Los snapshots de ambos drones podían competir simultáneamente por memoria y
   CPU.
7. Gazebo, RViz2, el grafo web, dos ORB-SLAM3 y el servidor convivían con unos
   2,3 GiB de memoria externa al experimento ya ocupados por el propio entorno.

La solución no fue eliminar funcionalidad. Se introdujo procesamiento
incremental, acceso batch, streaming de record/replay, una puerta global para
snapshots, caches derivadas, un vocabulario ORB compacto y perfiles de launch
para separar pruebas visuales de pruebas de carga.

Resultados principales:

- benchmark sintético de ocho drones: de unos `351,9 ms` por delta a
  `89,3 ms`, mejora aproximada del `74,6 %`;
- replay de referencia: de `5,91 s` a `2,22 s`, mejora del `62,4 %`;
- PSS de un proceso ORB-SLAM3 en la prueba comparable del vocabulario: de
  `969,6 MiB` a `212,1 MiB`, reducción del `78,1 %`;
- memoria del servidor durante replay: de `123,5 MiB` a `82,0 MiB`, reducción
  del `33,6 %`;
- ejecución final visual ordinaria con dos drones: sin presión PSI, con
  `946,6 MiB` mínimos disponibles y `539,1 MiB` de PSS máximo del grupo medido;
- prueba de carga real con tres drones: tres anchors, 141 poses activas y 7.981
  puntos publicados, sin swap activa durante la parte útil.

El flujo principal, las responsabilidades de las bases de datos, la fórmula de
score, los mensajes ROS y el mecanismo de anchors no cambiaron
semánticamente. Cambió cómo se transportan, comparan, almacenan temporalmente y
proyectan los mismos datos.

## 3. Alcance por subfase

### 3A: contrato arquitectónico

No se reabrió la arquitectura de 3A. Se mantuvieron los invariantes:

- `submap = (drone_id, map_epoch)`;
- `RawMapDatabase` es la autoridad de los datos ORB-SLAM3 crudos;
- `GlobalPoseStore` es la autoridad de poses globales y anchors;
- `GlobalMapBuilder` solo mantiene una vista derivada publicable;
- el servidor orquesta el flujo y no se convierte en una base de datos;
- los fiduciales simulados usan ground truth únicamente para emular la futura
  observación absoluta.

La optimización se diseñó alrededor de estos límites, no cambiándolos.

### 3B: infraestructura de ejecución y observación

Los cambios atribuibles a 3B fueron los perfiles de launch, el escalonado de
arranque, el modo headless para cargas, el monitor de recursos y los controles
necesarios para medir sin bloquear el equipo. RViz2 y el grafo web siguieron
siendo herramientas de validación, no partes del cálculo del mapa.

### 3C: cola, worker principal y record/replay

En 3C se concentraron:

- la puerta global de snapshots;
- la transferencia sin copia adicional desde respuestas ROS;
- el journal incremental a disco;
- el replay por streaming con backpressure;
- las estadísticas de almacenamiento del journal.

La disciplina FIFO y el único `PrimaryWorker` continuaron intactos.

### 3D: poses globales

No se cambió el algoritmo ni la autoridad de `GlobalPoseStore`. Las mejoras del
builder redujeron cuántas veces consulta y transforma sus poses, pero no cómo se
calculan ni cuándo se aceptan.

### 3E: anclaje fiducial

No se cambió el algoritmo de `FiducialAnchorManager`, el primer anchor, el radio
de observación ni el uso de ground truth simulado. Las pruebas finales
verificaron que seguían produciéndose dos anchors con dos drones y tres anchors
con tres drones.

### 3F: scores y vista global

La mayor parte de la reducción de CPU del servidor pertenece a 3F:

- actualización batch de scores;
- snapshot raw específico para el builder;
- dirty sets más compactos para submapas no anclados;
- backfill único al anclar;
- cache persistente de transformaciones `world_T_local`;
- evaluación única por KF en cada actualización.

### 3G: snapshots y reconciliación

Los snapshots conservaron su función de reconciliar pérdidas o cambios, pero
pasaron a:

- ejecutarse de uno en uno a escala global;
- normalizar solo la envolvente necesaria;
- comparar por referencia antes de reemplazar;
- generar índices hash solo donde aportan valor;
- comunicar diffs selectivos a las bases y al builder.

## 4. Situación inicial y síntoma observado

La prueba 98 validó el comportamiento funcional previsto para 3G:

- se descartó deliberadamente un delta;
- un snapshot posterior recuperó el estado perdido;
- el snapshot modificó las bases y marcó dirty, pero no construyó ni publicó;
- el siguiente delta normal consumió esos dirty sets;
- se obtuvieron dos anchors;
- `RawMapDatabase` terminó con 103 KFs y 10.938 MPs;
- `GlobalPoseStore` terminó con 94 poses activas;
- la vista final publicó 6.264 puntos y 94 KFs;
- el record contenía 54 entradas y 12 observaciones fiduciales;
- el worker principal respetó `max_active=1`.

Sin embargo, el ordenador se volvió casi inutilizable durante la simulación.
El resultado funcional era correcto, pero la prueba se clasificó como
operativamente no conseguida.

Las primeras mediciones mostraron aproximadamente:

- unos 13 GiB de memoria ocupada;
- solo 2,2 GiB disponibles antes de entrar en la zona crítica;
- 2 GiB de swap ya ocupada;
- snapshots concretos de unos `4,50 s` y `4,81 s`;
- un delta final de unos `4,10 s`.

No apareció un proceso muerto por el kernel ni evidencia de `OOM`. Tampoco se
identificó el disco como cuello de botella principal. El bloqueo percibido era
compatible con presión combinada de memoria, paginación previa, CPU intensa y
ráfagas de asignación simultáneas.

## 5. Baseline de seguridad

Antes de modificar el rendimiento se guardó el estado funcional exacto en:

```text
codex/archivos_auxiliares/baselines/3g_pre_rendimiento_20260812.tar.gz
```

SHA-256:

```text
ceb64b10611043498d6385e46fc02886a5849e1a722a82cb5f3792748d91bef6
```

La baseline permitió comparar el código anterior con el final y comprobar que
no se había perdido una responsabilidad funcional. También sirve como punto de
restauración o de investigación para reproducir el comportamiento previo.

No se modificaron los fuentes de `ORB_SLAM3`, `orbslam3_ros2` ni
`orbslam3_msgs` durante esta optimización.

## 6. Metodología de investigación

Cada cambio se abordó con este ciclo:

1. formular una hipótesis concreta sobre memoria, CPU o latencia;
2. introducir una modificación acotada;
3. compilar únicamente los paquetes afectados;
4. ejecutar una simulación o replay comparable con guardas de memoria;
5. registrar RSS, PSS, disponibilidad, swap, PSI, CPU y tiempos relevantes;
6. comprobar que el estado funcional final seguía siendo el mismo;
7. conservar también los intentos fallidos o no concluyentes;
8. continuar solo si el cambio aportaba evidencia útil.

Las guardas abortaban una ejecución antes de que el sistema llegara a una zona
de bloqueo. Esto hace que algunas pruebas terminen antes del escenario: no son
fallos funcionales, sino experimentos de seguridad que demostraron que todavía
no había margen suficiente.

Para interpretar memoria se distinguieron:

- **RSS**: páginas residentes contabilizadas por proceso, con posible doble
  conteo de memoria compartida;
- **PSS**: reparte proporcionalmente las páginas compartidas y permite sumar
  procesos con más fidelidad;
- **memoria disponible**: margen estimado del sistema para continuar sin una
  presión severa;
- **swap-in/swap-out**: actividad de paginación, especialmente durante la zona
  útil de la prueba;
- **PSI de memoria**: tiempo durante el que tareas quedan frenadas por presión
  de memoria;
- **CPU de sistema y del grupo**: útil para separar carga real de una falsa
  mejora por terminar antes.

## 7. Diagnóstico técnico de las causas

### 7.1 Copias completas en snapshots y deltas

El camino de recepción, normalización e inserción podía mantener varias
representaciones completas de un mismo mapa. En snapshots de miles de MPs esto
generaba ráfagas grandes de asignación, más tiempo bajo presión y mayor coste de
destrucción posterior.

### 7.2 Journal de record residente

El record guardaba las entradas para escribirlas al final. Una ejecución larga
retenía decenas o cientos de megabytes que ya no eran necesarios para el flujo
live. Esta memoria crecía con la duración de la prueba.

### 7.3 Replay monolítico

El replay cargaba todas las entradas antes de reproducirlas. Además de la copia
en disco, existía otra representación completa en RAM y la cola podía crecer
mucho más rápido que el worker.

### 7.4 Coste por entidad

Scores y builder hacían locks, copias o búsquedas repetidas por KF/MP. El coste
fijo por llamada dominaba al trabajar con miles de entidades, aunque cada
operación aislada pareciera pequeña.

### 7.5 Trabajo inútil en submapas no anclados

Los dirty sets podían acumular miles de IDs que aún no tenían pose world. Hasta
que llegara el anchor, el builder no podía proyectarlos, por lo que transportar
y revisar esos IDs repetidamente no aportaba resultado visible.

### 7.6 Transformaciones repetidas

La pose local de un MP depende de su KF y la pose world de ese KF. Muchos MPs
comparten KF, pero la transformación se recomponía más veces de las necesarias.

### 7.7 Snapshots simultáneos

La exclusión por dron no evitaba que dos drones procesaran dos snapshots
grandes a la vez. En el peor momento se duplicaban las ráfagas de memoria y CPU.

### 7.8 Vocabulario ORB

Cada wrapper cargaba privadamente el vocabulario textual completo. Con dos o
más drones este coste se multiplicaba casi linealmente y era, con diferencia,
la mayor reserva de memoria privada de los procesos ORB.

### 7.9 Coste del entorno visual

Gazebo GUI, RViz2 y Chromium para el grafo web son útiles para validar, pero no
son necesarios en todas las pruebas de rendimiento. Ejecutarlos siempre dejaba
menos margen para observar la escalabilidad del pipeline.

## 8. Instrumentación y herramientas creadas

### 8.1 `run_simulation.sh`

Archivo modificado:

```text
codex/herramientas/run_simulation.sh
```

Se añadieron opciones para activar el monitor y una guarda automática. La
herramienta sigue siendo el único punto normal de entrada a las simulaciones y
continúa encargándose del cierre de procesos.

La guarda observa la memoria disponible y evita prolongar una prueba cuando se
entra en una zona que podría bloquear la máquina. Los umbrales fueron variando
en experimentos concretos; no deben confundirse con requisitos funcionales.

### 8.2 `monitor_simulation_resources.sh`

Archivo nuevo:

```text
codex/herramientas/monitor_simulation_resources.sh
```

Produce un CSV y un resumen por prueba. Registra:

- memoria disponible y swap;
- RSS y PSS del grupo de procesos;
- CPU de sistema y del grupo;
- PSI de memoria;
- desglose de `Pss_Anon`, `Pss_File` y `Pss_Shmem` para ORB-SLAM3;
- máximos por proceso relevante: servidor, ORB, Gazebo, RViz2 y grafo web.

El desglose anon/file fue decisivo: mostró que el vocabulario no era solo
cache de archivo recuperable, sino principalmente heap privado.

### 8.3 `generate_compact_orb_vocabulary.py`

Archivo nuevo:

```text
codex/herramientas/generate_compact_orb_vocabulary.py
```

Genera un vocabulario ORB compacto mediante dos pasadas:

1. selecciona y remapea el árbol hasta la profundidad objetivo;
2. escribe el resultado y valida padres, IDs y words resultantes.

La herramienta evita una edición manual no reproducible del vocabulario.

## 9. Cambios en `orbslam3_server`

### 9.1 Archivos modificados

```text
orbslam3_server/include/orbslam3_server/primary_queue.hpp
orbslam3_server/src/global_map_server.cpp
orbslam3_server/test/test_primary_queue.cpp
```

También se sincronizó la documentación vigente del paquete.

### 9.2 Puerta global de snapshots

Antes podía haber un snapshot activo por dron. Ahora existe una puerta global:

- solo un snapshot puede estar activo en todo el servidor;
- las peticiones pendientes se deduplican por fuente;
- al terminar el snapshot activo se libera la puerta y se selecciona el
  siguiente;
- los deltas siguen entrando en la cola con su orden normal;
- no se crea un segundo worker ni se rompe `max_active=1`.

Motivo: impedir que dos respuestas grandes dupliquen simultáneamente el pico de
memoria. La prueba 102 confirmó el funcionamiento de esta serialización.

### 9.3 Recepción sin copia adicional

La respuesta ROS del mapa se conserva mediante un `shared_ptr` aliasado en vez
de copiar inmediatamente todo su contenido a otra estructura. La vida útil del
mensaje queda ligada al elemento de trabajo que lo necesita.

Esto no vuelve mutable el mensaje ni cambia su contrato; elimina una copia
completa en el punto de recepción. La prueba 103 se utilizó para validarlo.

### 9.4 Record incremental

El servidor inicia y finaliza explícitamente un journal incremental:

- abre un archivo temporal con sufijo `.in_progress`;
- escribe cada entrada conforme se confirma;
- al finalizar correctamente, cierra y renombra al destino definitivo;
- puede desactivar la retención en memoria;
- expone estadísticas para comprobar entradas residentes y bytes escritos.

Así, grabar no obliga a conservar toda la sesión en RAM. La prueba 108 mostró
38 entradas escritas y cero entradas residentes.

### 9.5 Replay por streaming y backpressure

El replay se divide en:

1. lectura ligera de metadatos;
2. apertura del archivo;
3. lectura de una entrada;
4. encolado;
5. espera mediante `WaitUntilPendingBelow(high_watermark_)` si la cola está
   demasiado llena;
6. continuación hasta EOF.

No se construye un vector con todas las entradas. Se conservan compatibilidad y
orden para record v1 y v2. La prueba 114 redujo el máximo pendiente de 23 a 8 y
la memoria máxima del servidor de 123,5 a 82 MiB respecto a la prueba 112.

### 9.6 Efecto sobre el flujo principal

No cambió la secuencia lógica:

```text
wrapper -> GlobalMapServer -> PrimaryQueue -> PrimaryWorker
        -> RawMapDatabase -> bases derivadas -> GlobalMapBuilder -> publish
```

La puerta de snapshots controla admisión; el streaming controla cuántos
elementos esperan; ninguno añade procesamiento concurrente del mapa.

## 10. Cambios en `orbslam3_multi`: `RawMapDatabase`

### 10.1 Archivos modificados

```text
orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp
orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp
orbslam3_multi/src/raw_map_database.cpp
orbslam3_multi/test/test_raw_map_database.cpp
```

### 10.2 Normalización ligera

La envolvente normalizada se crea a partir de metadatos y de los contenedores
que realmente deben transformarse. Se eliminó la copia preventiva de todo el
mapa antes de saber qué había cambiado.

La prueba 104 validó esta línea de trabajo. En ella el journal todavía retenía
146 entradas y unos 100 MiB, lo que permitió separar el ahorro de normalización
del problema de retención.

### 10.3 Comparación antes de reemplazar

Los KFs y MPs existentes se consultan por referencia constante y se comparan
antes de sustituirlos. Solo se crea la nueva copia persistente cuando existe un
cambio real.

Esto es especialmente importante para snapshots: la mayoría de entidades
pueden ser idénticas a la versión raw ya almacenada.

### 10.4 Sets de IDs solo cuando hacen falta

Los sets completos de IDs recibidos se construyen principalmente para
snapshots, porque son necesarios para detectar eliminaciones. En deltas no se
crean estructuras masivas que el contrato incremental no necesita.

Los `unordered_set` reservan capacidad previamente para reducir realocaciones.
La prueba 106 bajó el microbenchmark de inserción de unos `8,113 ms/1000` a
`6,462 ms/1000` frente al punto inmediatamente anterior.

### 10.5 Diff de asociaciones con índices hash

Las asociaciones KF-MP se comparan mediante dos índices hash temporales. El
resultado se ordena y deduplica para mantener una salida determinista.

La optimización evita búsquedas cuadráticas, pero no cambia qué asociación se
considera añadida, modificada o eliminada.

### 10.6 Modos del journal

Se añadieron tres modos explícitos:

- `InMemory`: conserva entradas, útil para tests pequeños o consumidores que
  necesitan obtener el vector completo;
- `Disabled`: no conserva journal;
- `Incremental`: escribe a disco y no retiene el payload completo.

APIs asociadas:

```text
StartIncrementalRecord(...)
FinalizeIncrementalRecord(...)
DisableJournalRetention(...)
GetJournalStorageStats(...)
ReadRecordMetadata(...)
StreamRecordEntries(...)
```

El archivo incremental usa `.in_progress` para no presentar como válido un
record cortado a mitad de escritura.

### 10.7 APIs batch y snapshots especializados

Se añadieron consultas orientadas a consumidores concretos:

```text
GetMapPointScoreInputs(...)
GetBuilderSnapshot(...)
GetActiveSubmapEntityIds(...)
```

Estas APIs no crean nuevas autoridades. Devuelven vistas/copias compactas y
coherentes bajo un único lock, evitando miles de llamadas unitarias.

### 10.8 Garantía de autoridad raw

`RawMapDatabase` sigue conteniendo los datos recibidos de ORB-SLAM3. Anchors,
optimizaciones, fusión futura o poses world no reescriben esta base. Los cambios
solo reducen el coste de insertar, comparar, consultar y grabar.

## 11. Cambios en `LandmarkScoreManager`

### 11.1 Archivos afectados

Las declaraciones e implementación del manager y sus tests dentro de
`orbslam3_multi` se actualizaron para ofrecer procesamiento batch. La fórmula
de score no cambió.

### 11.2 Entrada batch

Para cada MP se obtiene en una sola consulta compacta:

```text
observations_count
found_ratio
descriptor_valid
is_bad
```

El manager calcula todas las actualizaciones bajo una adquisición breve de su
lock, en vez de bloquear y consultar por MP.

### 11.3 Fórmula preservada

```text
score = clamp(
    0.55 * observations_component
  + 0.35 * found_ratio_component
  + 0.10 * descriptor_component,
  0.0,
  1.0)
```

Un MP marcado `bad` conserva score cero.

Se distingue entre:

- entrada inspeccionada o actualizada internamente;
- cambio material del score visible para el builder.

Solo un cambio material marca el MP dirty para reconstrucción. En la prueba
110 se observaron 243 entradas tratadas y únicamente 9 cambios materiales, lo
que confirmó que antes se propagaba demasiado trabajo derivado.

## 12. Cambios en `GlobalMapBuilder`

### 12.1 Archivos modificados

```text
orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp
orbslam3_multi/src/global_map_builder.cpp
orbslam3_multi/test/test_global_map_builder.cpp
```

### 12.2 Dirty compacto para submapas no anclados

Si un submapa no tiene anchor, el builder no acumula todos sus KFs y MPs como
IDs pendientes. Mantiene una marca compacta por submapa. No puede proyectarlos
todavía y repetir su inspección no produce salida world.

### 12.3 Backfill único al anclar

Cuando aparece el primer anchor, el builder consulta una sola vez todos los IDs
activos del submapa mediante `GetActiveSubmapEntityIds(...)`. Este backfill
incluye MPs aunque no estuvieran enumerados en el dirty original.

La prueba 112 verificó backfills de `32 KFs/3057 MPs` y `43 KFs/3176 MPs`, y el
estado final coincidió con el esperado.

### 12.4 `RawBuilderSnapshot`

`GetBuilderSnapshot(...)` devuelve solo los campos raw requeridos para construir
la vista. Se obtiene bajo un único lock y evita repetir getters por entidad.

En la prueba 117 el coste del builder sintético bajó aproximadamente:

- dos drones: de 720 a 592 ms, `-17,7 %`;
- cuatro drones: de 1.428 a 1.201 ms, `-15,9 %`;
- ocho drones: mediana de 1.550,8 ms frente a 1.920,6 ms, `-19,3 %`.

### 12.5 Cache persistente `world_T_local`

El builder conserva por KF la transformación de proyección necesaria. Se
invalida cuando:

- cambia la pose local del KF;
- cambia su pose world;
- el KF se elimina;
- cambia el submapa o deja de ser utilizable.

En una actualización, cada KF se valida como máximo una vez y todos sus MPs
reutilizan la transformación. No se cachea una autoridad nueva: es una cache
derivada y descartable.

Las pruebas 118 y 119 midieron repeticiones del builder de `594,5`, `603,1`,
`613,4`, `553,2` y `397,8 ms`, con mediana `594,5 ms`. Frente a `1.550,8 ms`,
la mejora fue del `61,7 %`. El ciclo completo quedó en `692,1 ms` para ocho
drones, unos `86,5 ms` por dron.

### 12.6 Reglas funcionales preservadas

- El builder consulta `RawMapDatabase`, `GlobalPoseStore` y scores.
- No calcula anchors ni persiste poses optimizadas.
- No escribe en las bases de autoridad.
- Un snapshot solo marca dirty; la siguiente entrada principal normal consume
  el trabajo y publica.
- Se conserva `fallback_submap=0` para los datos compatibles que lo requieren.

## 13. Cambios en el backend y benchmarks

Archivos afectados:

```text
orbslam3_multi/include/orbslam3_multi/sparse_global_backend.hpp
orbslam3_multi/src/sparse_global_backend.cpp
orbslam3_multi/test/test_scalability_3g.cpp
orbslam3_multi/CMakeLists.txt
```

El backend expone las nuevas operaciones de record y consultas batch sin
duplicar lógica de las bases. Se creó un benchmark determinista de 2, 4 y 8
drones para medir la misma carga sintética en cada iteración.

Este benchmark no sustituye a Gazebo: sirve para atribuir cambios al backend
sin ruido de física, GUI o tracking. Las pruebas reales finales siguieron
siendo obligatorias.

## 14. Cambios en `dron_individual`

### 14.1 Archivos modificados

```text
dron_individual/launch/orbslam_use.launch.py
dron_individual/launch/generar_dron.launch.py
dron_individual/config/hardware.yaml
dron_individual/config/orbslam/orbslam_stereo.yaml
dron_individual/config/orbslam/orbslam_mono.yaml
dron_individual/config/orbslam/vocabulary/ORBvoc_L5.txt
```

### 14.2 Arenas del allocator

`orbslam_use.launch.py` establece:

```text
MALLOC_ARENA_MAX=2
```

La hipótesis era reducir fragmentación y memoria retenida por múltiples arenas.
La prueba 123 mostró un efecto pequeño: PSS ORB de `973,5` a `969,0 MiB`, cerca
del `0,5 %`. Se mantuvo porque es inocuo y reduce crecimiento potencial con
muchos threads, pero no se considera la solución principal.

### 14.3 Vocabulario configurable

`generar_dron.launch.py` acepta `orb_vocabulary_path`, propagado hasta cada
wrapper. Esto permite seleccionar vocabulario por perfil sin modificar código.

### 14.4 Vocabulario L5 compacto

Se generó:

```text
dron_individual/config/orbslam/vocabulary/ORBvoc_L5.txt
```

Características:

```text
111078 nodos
99969 palabras
15768063 bytes
SHA-256 8390760637f5f41f50c62d575c62810e3bedd1f2508a6cbc926e23fdea5de053
```

El vocabulario completo tiene unos `145250924 bytes`; su árbol k=10 y nivel 6
contiene del orden de 970.994 hojas de profundidad 6. L5 conserva el árbol hasta
el nivel anterior y reduce drásticamente el heap privado.

La prueba 127 pasó de `969,6 MiB` de PSS ORB a `212,1 MiB`, `-78,1 %`. El
componente anónimo pasó de `940,7` a `175,6 MiB`, confirmando que el ahorro era
memoria privada real.

El launch multi-dron usa L5 por defecto. El launch individual conserva L6 para
disponer del vocabulario completo cuando se necesite comparar calidad de
relocalización o loop closure.

### 14.5 Perfil de cámara

Se redujo el perfil multi-dron a:

```text
20 Hz
480 x 360
ORBextractor.nFeatures = 900
```

Intrínsecos coherentes:

```text
fx = fy = 286.02185016085167
cx = 240.5
cy = 180.5
bf = 16.303245459168547
baseline = 0.057
```

No se cambió solo el YAML de ORB: la cámara simulada y sus intrínsecos se
actualizaron conjuntamente para evitar una geometría incoherente. La prueba 132
mostró PSS ORB de `207,3` a `186,8 MiB`, reducción adicional del `9,9 %`, y CPU
de sistema de `38,3 %` a `29,3 %`.

El coste es una posible pérdida de detalle y de robustez visual. Por eso se
validaron anchors, tracking, número de KFs/MPs y publicación final.

## 15. Cambios en `simulacion_dron`

### 15.1 Archivos modificados

```text
simulacion_dron/launch/multi_dron.launch.py
simulacion_dron/src/generar_URDF/generador_URDF.cpp
simulacion_dron/urdf/dron_plugins.xacro
simulacion_dron/config/sim_dron.yaml
codex/archivos_auxiliares/trayectorias/tray_prueba_137.yaml
```

### 15.2 Perfiles de launch

Se añadieron argumentos:

```text
launch_gazebo_gui
launch_mission_gui
drone_start_stagger_sec
orb_vocabulary_path
```

Cuando `launch_gazebo_gui=false` se usa `gzserver` sin cliente gráfico. RViz2 y
el grafo web mantienen sus controles existentes. Así existen dos perfiles:

- visual ordinario para concluir subfases con el usuario;
- headless o selectivo para medir carga y escalabilidad.

### 15.3 Arranque escalonado

Los drones arrancan mediante `TimerAction` con retrasos `0`, `8`, `16` segundos
y sucesivos según `drone_start_stagger_sec`.

Esto evita que todos los procesos carguen el vocabulario, inicialicen cámaras y
reserven heap en el mismo instante. La prueba 134 confirmó más margen de
arranque con el valor por defecto de 8 s.

### 15.4 Resolución coherente en Gazebo

`generador_URDF.cpp` declara, lee y reenvía ancho y alto de cámara.
`dron_plugins.xacro` usa esos argumentos en la imagen simulada. Esto fue
necesario para que la reducción 480x360 fuera real, no solo una expectativa del
consumidor ORB.

### 15.5 Número de drones

`sim_dron.yaml` se cambió temporalmente a tres drones para la prueba de carga
137. Al finalizar se restauró el valor ordinario:

```text
dron.number: 2
```

La trayectoria nueva de la prueba 137 define objetivos para los tres drones y
no sustituye las trayectorias normales.

## 16. Cronología completa de pruebas y decisiones

### Prueba 98: detección del problema

**Objetivo:** validación funcional completa de 3G con pérdida deliberada de un
delta y recuperación mediante snapshot.

**Resultado funcional:** conseguido. Se recuperó el delta, el snapshot no
publicó por sí solo, el siguiente delta consumió los dirty sets y el estado
final fue coherente: dos anchors, 103 KFs raw, 10.938 MPs raw, 94 poses activas,
6.264 puntos publicados, 94 KFs publicados, 54 entradas de record y 12
observaciones fiduciales. `max_active=1` confirmó un único worker principal.

**Resultado operativo:** no conseguido. El usuario observó bloqueos severos.
Los snapshots tardaron aproximadamente `4,50` y `4,81 s`; el delta final,
`4,10 s`. Había unos 2,2 GiB disponibles y 2 GiB de swap ocupada.

**Decisión:** conservar la semántica de 3G y abrir una investigación de
rendimiento incremental.

### Prueba 99: baseline de replay

**Objetivo:** comprobar que el record de la prueba 98 podía reproducirse y
servir como carga determinista sin depender de Gazebo.

**Resultado:** replay correcto. Se confirmó que record/replay era una vía útil
para comparar optimizaciones del backend y aislar el coste del servidor.

### Prueba 100: primera guarda

**Cambio/hipótesis:** ejecutar con guarda muy temprana para comprobar que el
monitor podía detener la prueba antes de bloquear el equipo.

**Resultado:** abortó a los `4 s`, con unos `975 MiB` disponibles y swap llena.

**Conclusión:** la protección funcionaba; todavía no era seguro ejecutar el
escenario completo en el estado inicial.

### Prueba 101: quitar interfaces visuales

**Cambio/hipótesis:** desactivar GUI para medir cuánto del problema procedía de
RViz2, Gazebo visual o web.

**Resultado:** guarda a los `6 s`; mínimo disponible `738,7 MiB`. Máximos
aproximados: servidor `35,5 MiB`, ORB `612,8 MiB`, mientras que el entorno
externo al grupo medido ya ocupaba unos `2058 MiB`.

**Conclusión:** las GUIs consumen margen, pero el coste ORB y la memoria ya
ocupada fuera de la prueba seguían siendo dominantes.

### Prueba 102: puerta global de snapshots

**Cambio:** serialización global de snapshots y deduplicación de peticiones
pendientes.

**Resultado:** guarda configurada en `512 MiB`, activada a los `10 s`; mínimo
`337,5 MiB`. RSS aproximado del grupo `1397,6 MiB`, ORB `1006,8 MiB`, servidor
`35,3 MiB`.

**Conclusión:** se demostró que nunca había dos snapshots activos a la vez. El
cambio eliminó el pico concurrente, aunque por sí solo no solucionaba la gran
memoria residente de ORB.

### Prueba 103: recepción zero-copy

**Primer intento:** configuración no válida de Gazebo; terminó con código 255
antes de producir una comparación útil. Se conservó como intento fallido y no
se atribuyó ninguna mejora.

**Segundo intento:** validación del `shared_ptr` aliasado para conservar la
respuesta ROS sin copia completa adicional.

**Resultado:** guarda a los `102 s`; mínimo disponible `485,8 MiB`; grupo
`1771 MiB`; servidor `191,7 MiB`; ORB `1197,6 MiB`; memoria externa aproximada
`2098,8 MiB`.

**Conclusión:** el ownership y la vida útil zero-copy fueron correctos. El
servidor todavía crecía por el journal y otras copias internas.

### Prueba 104: normalización ligera

**Cambio:** evitar clonar todo el mapa para crear la representación normalizada.

**Resultado:** guarda a los `90 s`; mínimo `508,1 MiB`; servidor `171,7 MiB`;
grupo `1654,2 MiB`. El journal mantenía 146 entradas y unos 100 MiB.

**Conclusión:** la normalización ligera era válida, pero la prueba hizo visible
que la retención del journal pasaba a ser una causa importante del crecimiento
del servidor.

### Prueba 105: comparación por referencia y primera micro-medición

**Cambio:** comparar entidades existentes por referencia constante antes de
reemplazarlas.

**Resultado:** guarda a los `100 s`; mínimo `494,8 MiB`; servidor `198,1 MiB`;
grupo `1692 MiB`; ORB `1147 MiB`. Había 162 entradas residentes y unos 130 MiB
de journal. Microbenchmark: `8,113 ms/1000` frente a `8,173 ms/1000`.

**Conclusión:** la mejora aislada era pequeña en ese benchmark y quedaba
ocultada por el journal. Se mantuvo por reducir copias de snapshots sin alterar
el contrato.

### Prueba 106: sets de IDs selectivos

**Cambio:** no construir sets completos de entidades recibidas en deltas y
reservar capacidad para snapshots.

**Resultado:** guarda a los `62 s`; mínimo `465,5 MiB`. El microbenchmark bajó
de `8,113` a `6,462 ms/1000`.

**Conclusión:** reducción medible del coste de inserción, especialmente en el
camino frecuente de deltas.

### Prueba 107: diff hash de asociaciones

**Primer intento:** Gazebo falló antes de la fase útil; no se usó para evaluar
el cambio.

**Segundo intento:** guarda a los `72 s`; mínimo `497,2 MiB`; ORB
`1212,6 MiB`; servidor `195,8 MiB`. Microbenchmark `7,154 ms/1000`.

**Conclusión:** el índice hash elimina riesgo de comportamiento cuadrático y es
más escalable, aunque este caso sintético fue algo peor que la prueba 106 por el
coste fijo de construir índices. Se mantuvo para snapshots y asociaciones
grandes, con orden y deduplicación deterministas.

### Prueba 108: record incremental

**Cambio:** escritura incremental con `.in_progress` y journal sin payload
residente.

**Resultado:** guarda a los `47 s`; 38 de 38 entradas escritas; cero entradas
residentes; archivo de `66.109.814 bytes`; 36 entradas de mapa y 17
observaciones fiduciales según los contadores del experimento. Máximo del
servidor `72,1 MiB`, frente al rango anterior de `165-198 MiB`.

**Conclusión:** cambio decisivo para sesiones largas. La memoria del servidor
dejó de crecer linealmente con la duración de la grabación.

### Prueba 109: replay del record incremental

**Objetivo:** comprobar legibilidad y equivalencia funcional del nuevo archivo.

**Resultado:** 36/36 entradas procesadas; dos submapas, 87 KFs, 9.815 MPs, dos
anchors, 87 poses; vista de 6.395 puntos y 87 KFs. Mínimo disponible
`1988,1 MiB`; PSI de memoria cero.

**Conclusión:** el record incremental no perdió información y el replay era
funcionalmente equivalente.

### Prueba 110: filtrado material de scores

**Cambio:** entrada y actualización batch, separando trabajo inspeccionado de
cambios de score materialmente visibles.

**Resultado:** 243 entradas internas y solo 9 dirty relevantes. Guarda a los
`11 s`, mínimo `480,3 MiB`; el escenario no llegó a ejecutarse por completo.

**Conclusión:** se confirmó que propagar todos los MPs al builder era trabajo
innecesario. La prueba live no era suficiente para cuantificar rendimiento, por
lo que se continuó con replay y benchmarks.

### Prueba 111: test focal inicial

**Objetivo:** proteger contratos unitarios tras los cambios batch.

**Resultado:** `19/19` tests focales superados. La prueba Gazebo se detuvo por
la guarda antes de un delta relevante.

**Conclusión:** regresión funcional cubierta; medición live inconcluyente.

### Prueba 112: dirty compacto y backfill de anchors

**Cambio:** diferir el trabajo de submapas no anclados y realizar un backfill
completo una sola vez al anclar.

**Resultado:** replay de 54 entradas. Antes del anchor no se calculó la vista
35 veces de forma inútil. Los backfills fueron `32 KFs/3057 MPs` y
`43 KFs/3176 MPs`. El estado final coincidió con la referencia. Servidor máximo
`123,5 MiB`; cola pendiente máxima 23.

**Conclusión:** se preservó la completitud al anclar y se eliminó trabajo previo
sin resultado world. La cola y la carga monolítica del replay quedaron como
siguiente objetivo.

### Prueba 113: preparación del replay streaming

**Objetivo:** ampliar cobertura focal de lectura/escritura y streaming.

**Resultado:** `25/25` tests focales superados. Gazebo alcanzó la guarda a los
`9 s` antes de la zona relevante.

**Conclusión:** contratos protegidos; sin medición live atribuible.

### Prueba 114: replay streaming con watermark

**Cambio:** leer y encolar una entrada cada vez, frenando el productor cuando
la cola alcanza el high watermark.

**Resultado:** mismo estado funcional que la prueba 112. Pendiente máximo de 8
frente a 23. Servidor máximo de `82,0 MiB` frente a `123,5 MiB`, reducción del
`33,6 %`.

**Conclusión:** el replay dejó de cargar toda la sesión y de inundar la cola.

### Prueba 115: primer benchmark de escalabilidad

**Objetivo:** obtener una referencia determinista del backend para 2, 4 y 8
drones.

**Resultados iniciales:**

| Drones | Memoria incremental | Tiempo aproximado por dron/delta |
|---:|---:|---:|
| 2 | 22,1 MiB | 353,7 ms |
| 4 | 33,7 MiB | 341,5 ms |
| 8 | 56,4 MiB | 351,9 ms |

La ejecución Gazebo terminó por guarda a los `10 s`; ORB alcanzó
`1003,7 MiB` y el servidor `35,4 MiB`.

**Conclusión:** la memoria propia del backend escalaba razonablemente, pero el
coste por delta era demasiado alto para fases posteriores y ORB seguía
dominando la prueba real.

### Prueba 116: atribución del coste score/builder

**Cambio:** scores batch y medición detallada del benchmark.

**Resultado:** inserción más score `66,4 ms`; builder `1920,6 ms`, equivalente
al `96,7 %` del total aproximado de `1987 ms` del caso agregado medido.
Gazebo no produjo una comparación concluyente.

**Conclusión:** el cuello de CPU del backend estaba claramente en
`GlobalMapBuilder`, no en la fórmula de scores ni en la inserción raw.

### Prueba 117: `RawBuilderSnapshot`

**Cambio:** consulta batch especializada para el builder bajo un único lock.

**Resultado:**

| Drones | Antes | Después | Mejora |
|---:|---:|---:|---:|
| 2 | 720 ms | 592 ms | 17,7 % |
| 4 | 1428 ms | 1201 ms | 15,9 % |
| 8 | 1920,6 ms | 1550,8 ms (mediana) | 19,3 % |

**Conclusión:** las consultas unitarias eran costosas, pero todavía se repetían
transformaciones de KF para muchos MPs.

### Prueba 118 y prueba 119: cache de proyección por KF

**Cambio:** cache persistente de `world_T_local`, invalidación selectiva y
validación máxima una vez por KF en cada actualización.

**Resultado:** repeticiones de builder de `594,5`, `603,1`, `613,4`, `553,2` y
`397,8 ms`; mediana `594,5 ms`, un `61,7 %` menor que `1550,8 ms`. Ciclo
completo de ocho drones `692,1 ms`, `86,5 ms` por dron. El replay pasó de
`5,91 s` a `2,22 s`, mejora del `62,4 %`, con solo unos `1,4 MiB` adicionales
por la cache.

**Conclusión:** fue la optimización principal de CPU del servidor. La cache es
pequeña, derivada y correctamente invalidada.

### Prueba 120: primer perfil de launch

**Cambio:** argumentos de GUI/headless y configuración de prueba.

**Resultado:** la ruta YAML relativa falló; el launch y sus argumentos eran
válidos. Mínimo disponible `525,1 MiB` antes del fallo.

**Conclusión:** error mecánico de ruta, no del perfil. Se corrigió usando ruta
absoluta en pruebas posteriores.

### Prueba 121: Gazebo headless

**Cambio:** ejecutar `gzserver` sin cliente visual.

**Resultado:** guarda a los `46 s`; mínimo `507,3 MiB`; ORB `1102,8 MiB`.

**Conclusión:** headless libera margen gráfico, pero no resuelve la memoria
privada de los vocabularios ORB.

### Prueba 122: medición PSS y desglose por proceso

**Cambio:** ampliar el monitor para medir PSS.

**Resultado:** grupo RSS/PSS `1731,7/1444,4 MiB`; ORB RSS/PSS
`1063,8/974,9 MiB`; Gazebo `340,4 MiB`; servidor `18,1 MiB`.

**Conclusión:** el doble conteo de RSS no explicaba el problema: ORB seguía
teniendo casi 1 GiB de PSS real.

### Prueba 123: limitar arenas glibc

**Cambio:** `MALLOC_ARENA_MAX=2`.

**Resultado:** PSS ORB de `973,5` a `969,0 MiB`, reducción aproximada del
`0,5 %`.

**Conclusión:** hipótesis secundaria. Se conserva como endurecimiento menor,
pero no se le atribuye una mejora sustancial.

### Prueba 124: intento con puerto ocupado

**Resultado:** la prueba no arrancó correctamente porque el puerto requerido
ya estaba ocupado.

**Conclusión:** intento inválido; se limpiaron procesos y se repitió. No se
extrajeron cifras de rendimiento de este intento.

### Prueba 125: perfil headless válido

**Resultado:** CPU de sistema de `35,7 %` a `27,8 %`, reducción del `22,1 %`;
CPU del grupo de `367,9 %` a `306,9 %`, reducción del `16,6 %`.

**Conclusión:** el perfil headless es útil para pruebas automáticas y de carga,
pero la validación visual seguirá usando RViz2, grafo web y, cuando corresponde,
Gazebo GUI.

### Prueba 126: naturaleza de la memoria ORB

**Cambio:** desglose `Pss_Anon`, `Pss_File` y `Pss_Shmem`.

**Resultado:** PSS ORB `969,6 MiB`, de los cuales `940,7 MiB` eran anónimos,
`34,8 MiB` de archivo y `1,4 MiB` compartidos.

**Conclusión:** la mayor parte era heap privado generado al cargar el
vocabulario. El kernel no podía recuperarlo como simple page cache.

### Prueba 127: vocabulario ORB L5

**Cambio:** vocabulario compacto reproducible y ruta configurable.

**Resultado:** PSS ORB de `969,6` a `212,1 MiB`, reducción del `78,1 %`;
memoria anónima de `940,7` a `175,6 MiB`.

**Conclusión:** fue la optimización principal de RAM. Se mantuvo L6 disponible
para comparar calidad y se exigieron pruebas reales de tracking y anchors.

### Prueba 128: ejecución headless completa con L5

**Objetivo:** comprobar una sesión larga después del cambio de vocabulario.

**Resultado:** `172 s`; mínimo disponible `965,3 MiB`; PSI de memoria máximo
`0,18`; grupo `896,1 MiB`; ORB `357 MiB`; servidor `87,7 MiB`. Funcionalidad
correcta.

**Conclusión:** la ejecución completa dejó de entrar en la zona de bloqueo. El
margen todavía debía comprobarse con todas las interfaces visuales.

### Prueba 129: prueba visual corta con L5

**Resultado:** mínimo `725,7 MiB`; grupo `554,4 MiB`; ORB `209,6 MiB`; RViz2
`124,3 MiB`; web `49,9 MiB`.

**Conclusión:** los tres observadores podían coexistir, aunque una prueba corta
no demostraba el margen al final de la misión.

### Prueba 130: ejecución visual completa antes del ajuste de cámara

**Resultado:** `176 s`; mínimo disponible `556,5 MiB`; grupo `739,2 MiB`; ORB
`366,6 MiB`; servidor `85,8 MiB`; RViz2 `124,7 MiB`; web `50,6 MiB`.

**Conclusión:** funcional, pero demasiado cerca del umbral de seguridad para
pensar en más drones y fases futuras. Se decidió reducir además la carga visual
y de extracción ORB de forma coherente.

### Prueba 131: dos intentos inválidos de parámetros de cámara

**Cambio intentado:** parametrizar ancho y alto de la cámara simulada.

**Resultado:** ambos intentos fallaron con `ParameterNotDeclared` por no haber
declarado/reencaminado completamente los parámetros en el generador URDF.

**Conclusión:** no se aceptó una configuración parcial. Se corrigieron
declaración, lectura, reenvío y propiedades xacro antes de volver a medir.

### Prueba 132: cámara 480x360 y 900 features

**Cambio:** cámara Gazebo, YAML hardware e intrínsecos ORB coherentes; 20 Hz y
900 features.

**Resultado:** PSS ORB de `207,3` a `186,8 MiB`, `-9,9 %`; CPU de sistema de
`38,3 %` a `29,3 %`, `-23,5 %`; CPU del grupo de `248,2 %` a `194,0 %`,
`-21,8 %`.

**Conclusión:** mejora adicional real sin introducir una discrepancia entre
imagen simulada y calibración.

### Prueba 133: ejecución visual completa optimizada

**Resultado:** `176 s`; mínimo disponible `612,3 MiB`; PSI cero; grupo
`640,7 MiB`; ORB `273,8 MiB`; servidor `58,7 MiB`; RViz2 `127,1 MiB`; web
`41,4 MiB`. Estado funcional: dos submapas, dos anchors, 90 KFs, 9.787 MPs y
174/174 elementos relevantes procesados según los contadores de la prueba.

**Conclusión:** se recuperó margen y se mantuvo el resultado funcional con el
perfil visual completo.

### Prueba 134: arranque escalonado por defecto

**Cambio:** retraso de 8 s entre arranques de drones.

**Resultado:** prueba visual corta; mínimo disponible `948,9 MiB`; grupo
`541,2 MiB`; ORB `188,3 MiB`; RViz2 `109,2 MiB`; web `49,8 MiB`.

**Conclusión:** reduce el pico de inicialización y es especialmente útil al
crecer a tres o más drones.

### Prueba 135: smoke test de tres drones

**Cambio:** `dron.number=3` temporalmente.

**Resultado:** mínimo disponible `1124 MiB`; PSS del grupo `837,4 MiB`; ORB
`289,7 MiB`; Gazebo `369,9 MiB`; servidor `18,9 MiB`. El tercer dron produjo
tracking y deltas. El escenario corto no exigía anchors.

**Conclusión:** la infraestructura y los tópicos escalan a un tercer dron sin
bloqueo ni crecimiento explosivo.

### Prueba 136: restauración temporal a dos drones

**Objetivo:** verificar que el experimento de tres drones no había cambiado el
perfil ordinario.

**Resultado:** prueba visual; mínimo disponible `975,6 MiB`; PSS del grupo
`541,1 MiB`; PSI cero.

**Conclusión:** restauración correcta.

### Regresión automática final

Antes de la carga final se ejecutó build de los cuatro paquetes afectados, con
resultado `4/4` correcto.

Hubo una primera invocación breve de GoogleTest con selección inválida; no se
contabilizó como regresión. La invocación válida obtuvo:

```text
37/37 tests C++ superados
8/8 tests del visualizador web superados
```

Distribución de los 37 tests:

| Zona | Tests |
|---|---:|
| RawMapDatabase | 10 |
| SparseGlobalBackend | 6 |
| FiducialAnchorManager | 3 |
| LandmarkScoreManager | 3 |
| GlobalMapBuilder | 3 |
| Escalabilidad | 3 |
| PrimaryQueue | 6 |
| Ground truth/fiducial | 3 |

Benchmark sintético final:

| Drones | Tiempo por delta/dron | Memoria incremental |
|---:|---:|---:|
| 2 | 84,3 ms | 23,4 MiB |
| 4 | 85,5 ms | 34,4 MiB |
| 8 | 89,3 ms | 57,0 MiB |

La memoria incremental permanece próxima a la referencia inicial, mientras el
tiempo de ocho drones baja de `351,9` a `89,3 ms` por delta/dron.

### Prueba 137: carga real de tres drones

**Objetivo:** no limitarse a un arranque smoke, sino completar una misión real
con tres fuentes simultáneas.

**Configuración:** trayectoria nueva con seis goals y tres drones; interfaces
seleccionadas para priorizar la carga. `dron.number=3` solo durante la prueba.

**Resultado funcional:**

- seis goals completados;
- tres anchors;
- 36 observaciones fiduciales;
- 166 poses totales y 141 activas;
- 7.981 puntos y 141 KFs en la vista final;
- 254 entradas procesadas;
- `max_active=1` conservado.

**Resultado de rendimiento:** duración aproximada `178 s`; mínimo disponible
`878,8 MiB`; PSS máximo del grupo `1041,4 MiB`; ORB `436,4 MiB`; Gazebo
`363,2 MiB`; servidor `82,7 MiB`; PSI máximo `0,18`. Durante la zona activa:
disponible medio `997,6 MiB`, CPU de sistema `33,1 %`, CPU agregada del grupo
`340,8 %`. Hubo 281 páginas de swap-out únicamente durante el arranque y cero
durante la fase activa.

**Conclusión:** el pipeline actual funciona con más de dos drones y mantiene
margen operativo. No demuestra todavía escalabilidad ilimitada ni la carga de
las fases 4 a 9, pero elimina el bloqueo original y aporta una referencia real.

### Prueba 138: restauración final ordinaria

**Objetivo:** dejar el proyecto en su configuración normal, visual y con dos
drones, después de todas las pruebas de carga.

**Resultado:** mínimo disponible `946,6 MiB`; PSS máximo del grupo
`539,1 MiB`; ORB `187,9 MiB`; RViz2 `105,3 MiB`; web `41,5 MiB`; PSI cero.

**Conclusión:** restauración conseguida. El estado entregado no queda en el
perfil experimental de tres drones ni en modo exclusivamente headless.

## 17. Inventario exacto del código de optimización

Esta lista separa los cambios de rendimiento de otros cambios históricos que
puedan seguir apareciendo en `git status` por ser el repositorio un worktree con
trabajo previo.

### `orbslam3_server`

```text
orbslam3_server/include/orbslam3_server/primary_queue.hpp
orbslam3_server/src/global_map_server.cpp
orbslam3_server/test/test_primary_queue.cpp
```

Responsabilidad de los cambios: puerta global de snapshots, tipos y estado de
cola necesarios, recepción sin copia completa, record incremental, replay
streaming y tests de orden/admisión.

### `orbslam3_multi`

```text
orbslam3_multi/CMakeLists.txt
orbslam3_multi/include/orbslam3_multi/global_map_builder.hpp
orbslam3_multi/include/orbslam3_multi/landmark_score_manager.hpp
orbslam3_multi/include/orbslam3_multi/raw_map_database.hpp
orbslam3_multi/include/orbslam3_multi/raw_map_types.hpp
orbslam3_multi/include/orbslam3_multi/sparse_global_backend.hpp
orbslam3_multi/src/global_map_builder.cpp
orbslam3_multi/src/landmark_score_manager.cpp
orbslam3_multi/src/raw_map_database.cpp
orbslam3_multi/src/sparse_global_backend.cpp
orbslam3_multi/test/test_global_map_builder.cpp
orbslam3_multi/test/test_landmark_score_manager.cpp
orbslam3_multi/test/test_raw_map_database.cpp
orbslam3_multi/test/test_scalability_3g.cpp
```

`CMakeLists.txt` registra el benchmark/test nuevo. No se modificaron durante la
optimización los algoritmos de:

```text
orbslam3_multi/src/global_pose_store.cpp
orbslam3_multi/src/fiducial_anchor_manager.cpp
```

Tampoco sus contratos necesitaron una reescritura funcional.

### `dron_individual`

```text
dron_individual/launch/orbslam_use.launch.py
dron_individual/launch/generar_dron.launch.py
dron_individual/config/hardware.yaml
dron_individual/config/orbslam/orbslam_stereo.yaml
dron_individual/config/orbslam/orbslam_mono.yaml
dron_individual/config/orbslam/vocabulary/ORBvoc_L5.txt
```

Los metadatos de instalación existentes hacen disponible el vocabulario desde
el paquete. El vocabulario completo `vocabulary/ORBvoc.txt` no se sustituyó ni
se borró.

### `simulacion_dron`

```text
simulacion_dron/launch/multi_dron.launch.py
simulacion_dron/src/generar_URDF/generador_URDF.cpp
simulacion_dron/urdf/dron_plugins.xacro
simulacion_dron/config/sim_dron.yaml
codex/archivos_auxiliares/trayectorias/tray_prueba_137.yaml
```

`sim_dron.yaml` aparece porque se utilizó para la carga de tres drones y se
restauró al final a dos. El valor final, no el temporal, es el que forma parte
del estado entregado.

### Herramientas

```text
codex/herramientas/run_simulation.sh
codex/herramientas/monitor_simulation_resources.sh
codex/herramientas/generate_compact_orb_vocabulary.py
codex/herramientas/USO_HERRAMIENTAS.md
```

`USO_HERRAMIENTAS.md` documenta cómo usar el monitor, las guardas y el
generador. No se introdujo una herramienta paralela que evite los launches
oficiales.

### Documentación sincronizada durante el trabajo

Además de este informe, el estado se fue registrando en:

```text
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3G.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3G_RESUMEN.md
codex/contexto/paquetes/orbslam3_server/
codex/contexto/paquetes/orbslam3_multi/
codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/simulacion_dron/
codex/contexto/07_ULTIMA_SESION.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
```

El historial largo es la evidencia cronológica. Este documento reorganiza esa
información por causa y componente para facilitar una investigación posterior.

## 18. Comparación cuantitativa antes y después

### 18.1 CPU y latencia del backend

| Métrica | Antes | Después | Cambio |
|---|---:|---:|---:|
| Ocho drones, tiempo por delta/dron | 351,9 ms | 89,3 ms | -74,6 % |
| Replay de referencia | 5,91 s | 2,22 s | -62,4 % |
| Builder ocho drones | 1550,8 ms | 594,5 ms | -61,7 % |
| Cámara, CPU de sistema comparable | 38,3 % | 29,3 % | -23,5 % |
| Cámara, CPU del grupo comparable | 248,2 % | 194,0 % | -21,8 % |

La primera cifra sintética inicial de builder fue `1920,6 ms`; `1550,8 ms` es
el punto inmediatamente anterior a la cache, después de introducir
`RawBuilderSnapshot`. Por eso se muestran ambas referencias en la cronología y
se usa `1550,8` para atribuir correctamente el efecto aislado de la cache.

### 18.2 Memoria

| Métrica | Antes | Después | Cambio |
|---|---:|---:|---:|
| PSS ORB, vocabulario comparable | 969,6 MiB | 212,1 MiB | -78,1 % |
| Heap anónimo ORB | 940,7 MiB | 175,6 MiB | -81,3 % |
| Servidor en replay comparable | 123,5 MiB | 82,0 MiB | -33,6 % |
| Journal incremental, entradas residentes | 38 | 0 | -100 % del payload retenido |
| Perfil cámara, PSS ORB comparable | 207,3 MiB | 186,8 MiB | -9,9 % adicional |

No debe sumarse directamente el `78,1 %` del vocabulario con el `9,9 %` de la
cámara: son pasos sucesivos con denominadores distintos.

### 18.3 Escalabilidad sintética final

| Drones | Tiempo final por delta/dron | Memoria incremental final |
|---:|---:|---:|
| 2 | 84,3 ms | 23,4 MiB |
| 4 | 85,5 ms | 34,4 MiB |
| 8 | 89,3 ms | 57,0 MiB |

El tiempo por dron permanece casi plano entre dos y ocho fuentes. Esto no
significa que el tiempo total sea constante: significa que el coste normalizado
por fuente no se degrada de forma acusada en esta carga determinista.

### 18.4 Estado del host en la entrega

La comparación más representativa para el usuario es la prueba visual
ordinaria final, no el benchmark aislado:

```text
dos drones
Gazebo GUI activo
RViz2 activo
grafo web activo
stagger de 8 s por defecto
memoria disponible mínima: 946,6 MiB
PSS máximo del grupo: 539,1 MiB
PSS ORB: 187,9 MiB
PSS RViz2: 105,3 MiB
PSS web: 41,5 MiB
PSI de memoria: 0
```

Frente al bloqueo original, el sistema termina la prueba sin presión de memoria
observable y queda en la configuración normal de dos drones.

## 19. Qué cambió entre clases

### 19.1 Cambios de API interna

Sí se añadieron métodos entre componentes internos. Los principales son:

```text
RawMapDatabase::GetMapPointScoreInputs(...)
RawMapDatabase::GetBuilderSnapshot(...)
RawMapDatabase::GetActiveSubmapEntityIds(...)
RawMapDatabase::ReadRecordMetadata(...)
RawMapDatabase::StreamRecordEntries(...)
RawMapDatabase::StartIncrementalRecord(...)
RawMapDatabase::FinalizeIncrementalRecord(...)
RawMapDatabase::DisableJournalRetention(...)
RawMapDatabase::GetJournalStorageStats(...)
PrimaryQueue::WaitUntilPendingBelow(...)
```

El backend y el servidor exponen o consumen estas operaciones. Son cambios de
eficiencia y de lifecycle; no trasladan autoridad de una clase a otra.

### 19.2 Comunicación que se evitó

- El score manager ya no necesita una cadena de getters por cada MP.
- El builder ya no solicita raw y pose repetidamente por cada punto.
- Un submapa no anclado se representa con una marca compacta en vez de miles de
  IDs dirty.
- El replay no entrega toda la sesión de golpe a la cola.
- El receptor no copia una respuesta completa antes de encolarla.

### 19.3 Comunicación que no cambió

- El wrapper sigue publicando deltas al servidor.
- El snapshot sigue entrando por el mismo flujo principal y se compara contra
  `RawMapDatabase`.
- `RawMapDatabase` informa qué se modificó.
- `GlobalPoseStore` recibe cambios de pose de KFs anclados.
- `LandmarkScoreManager` conserva sus scores derivados.
- `GlobalMapBuilder` recibe dirty sets, consulta autoridades y publica en el
  siguiente flujo principal normal.
- El grafo web sigue observando eventos; no interviene en la ejecución.

## 20. Qué no se modificó funcionalmente

### Mensajes y ROS

No se cambió ningún mensaje de `orbslam3_msgs`, topic público, service público
ni action. No fue necesario recompilar ORB-SLAM3 con una API distinta.

### Cola principal

Sigue existiendo un único worker. No se paralelizó el commit de mapas ni se
permitió que un snapshot publique por su cuenta. Todas las pruebas relevantes
mantuvieron `max_active=1`.

### `RawMapDatabase`

Sigue siendo raw. No se escriben en ella las poses optimizadas ni los anchors.
El snapshot reconcilia datos ORB crudos, igual que antes.

### `GlobalPoseStore`

No se cambió el algoritmo de composición local-world, la persistencia de
anchors ni la selección de poses aceptadas. El builder consulta menos veces,
pero obtiene la misma autoridad.

### `FiducialAnchorManager`

No se alteró la asociación GT-KF, el criterio de primer anchor ni los fiduciales
simulados. Las pruebas con dos y tres drones confirmaron el número esperado de
anchors.

### Scores

La fórmula y el significado del score inicial ORB se conservaron. Solo se
agrupó su cálculo y se evitó propagar valores que no cambiaron materialmente.

### `GlobalMapBuilder`

Continúa siendo una cache/vista derivada. La nueva cache de transformaciones se
puede descartar y reconstruir desde las bases. No se convirtió en autoridad de
poses, MPs ni scores.

### Record/replay

Se mantuvo compatibilidad con record v1 y v2. El record de 3G sigue siendo
delta-only según el acuerdo funcional; la optimización no añadió snapshots al
archivo `.record`.

### Visualización

No se modificó la topología del grafo web para justificar una mejora de
rendimiento. RViz2 sigue recibiendo la misma salida sparse global y la coloración
de score permanece como responsabilidad temporal del servidor hasta la fase 7.

## 21. Costes y riesgos aceptados

### 21.1 Vocabulario L5

Es el cambio con mayor ahorro y también el de mayor riesgo algorítmico. Un
vocabulario menos profundo puede reducir discriminación visual en
relocalización y detección de loops. Durante 3G se validaron tracking, anchors,
deltas y mapas; todavía no se ha validado una campaña extensa de loop closure
de fases posteriores.

Mitigación: L6 se conserva y el path es configurable. En fases 4-6 se debe
comparar L5/L6 si aparecen falsos candidatos, pérdida de recall o problemas de
relocalización.

### 21.2 Resolución y features

480x360, 20 Hz y 900 features reducen carga. Pueden perder puntos pequeños,
texturas lejanas o robustez en movimiento rápido. La configuración es coherente
geométricamente y funcionó en las trayectorias actuales, pero no representa una
garantía universal para todos los mundos.

### 21.3 Serialización de snapshots

Reduce picos a cambio de aumentar el tiempo de espera si muchos drones piden
snapshot simultáneamente. Es una decisión adecuada porque snapshots son
reconciliaciones excepcionales, mientras los deltas son el camino normal. La
cola deduplica peticiones para no acumular snapshots obsoletos.

### 21.4 Caches

Toda cache añade riesgo de invalidación incompleta. Se cubrieron cambios de pose
local, pose world, borrado y anclaje. Los tests y replay compararon el resultado
final, pero las fases de optimización y fusión deberán añadir casos específicos
cuando empiecen a mover KFs/MPs por vías nuevas.

### 21.5 Margen para fases 4 a 9

El objetivo era que 3G no agotara el ordenador antes de añadir loops, fusión,
optimización, densidad y GUI. La situación final aporta margen real, pero no
demuestra de antemano que todas las fases juntas cabrán sin más trabajo.

Las siguientes fases deben respetar estas reglas:

- procesos pesados no deben duplicar mapas completos;
- cálculos secundarios deben usar snapshots privados compactos;
- el commit debe ser breve y selectivo;
- las caches deben invalidarse por IDs/revisiones;
- los records deben escribirse y leerse por streaming;
- una prueba de carga debe poder ejecutarse headless;
- cada nueva fase debe añadir sus métricas a PSS/CPU/PSI y no mirar solo RSS;
- tres drones reales son el mínimo de regresión de escalabilidad cuando cambie
  una ruta compartida.

## 22. Alternativas consideradas y descartadas

### Quitar toda la visualización

No es una solución del producto. Headless se añadió como perfil de medición,
pero las pruebas de cierre siguen necesitando RViz2 y grafo web. Las pruebas 129,
130, 133, 136 y 138 verificaron explícitamente el perfil visual.

### Confiar únicamente en RSS

Se descartó porque suma memoria compartida varias veces. PSS confirmó qué parte
era real y permitió localizar el heap ORB.

### Resolverlo solo con `MALLOC_ARENA_MAX`

La mejora fue de aproximadamente `0,5 %`, insuficiente. Se conservó como ajuste
menor, no como argumento principal del resultado.

### Bajar únicamente la frecuencia

20 Hz redujo CPU, pero no la memoria del vocabulario. Era una mejora útil y
ortogonal, no una solución completa.

### Cargar todo el replay y confiar en la RAM

Se descartó porque el coste crece con la duración. Streaming más watermark
ofrece memoria acotada y conserva el orden.

### Paralelizar el builder o añadir workers

No se hizo. Habría aumentado picos, complejidad y riesgo de revisiones
incoherentes. Primero se eliminó trabajo redundante; el coste final normalizado
quedó casi plano hasta ocho drones sin cambiar el modelo de un worker.

### Reducir estructuras borrando información raw

No se hizo. `RawMapDatabase` conserva la información necesaria. Los snapshots
especializados son copias compactas para consumo, no pérdidas en la autoridad.

## 23. Evidencia y artefactos para otro chat

### Baseline pre-optimización

```text
codex/archivos_auxiliares/baselines/3g_pre_rendimiento_20260812.tar.gz
```

### Historial detallado

```text
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3G.md
```

Contiene entradas cronológicas de todas las pruebas. Su resumen barato está en:

```text
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3G_RESUMEN.md
```

### Logs y recursos

```text
codex/archivos_auxiliares/logs/prueba_<N>.resources.csv
codex/archivos_auxiliares/logs/prueba_<N>.resources.summary
```

Existen artefactos para las pruebas monitorizadas 100-138. Los CSV contienen
la serie temporal; los `.summary` contienen máximos, mínimos y promedios
reducidos. Los logs completos de simulación se conservan en la misma carpeta,
pero deben analizarse mediante las herramientas de reducción, no abrirse
directamente.

### Record de referencia

```text
codex/archivos_auxiliares/repeticiones/rawdb_prueba_98.record
```

### Trayectorias

```text
codex/archivos_auxiliares/trayectorias/tray_prueba_98.yaml
codex/archivos_auxiliares/trayectorias/tray_prueba_99.yaml
codex/archivos_auxiliares/trayectorias/tray_prueba_137.yaml
```

### Herramientas de reproducción

```text
codex/herramientas/build_selected_packages.sh
codex/herramientas/run_simulation.sh
codex/herramientas/monitor_simulation_resources.sh
codex/herramientas/reduce_simulation_log.sh
```

No debe repetirse una simulación pesada sin activar la guarda de recursos si el
host vuelve a tener poca memoria disponible o swap ocupada.

## 24. Cómo interpretar los resultados

Las cifras no proceden todas de la misma configuración. Para evitar conclusiones
incorrectas:

- 100-107 investigan copias y admisión, pero muchas terminan por guarda;
- 108-114 aíslan journal y replay;
- 115-119 aíslan CPU del backend con benchmark/replay;
- 120-126 separan GUI, RSS/PSS, frecuencia y naturaleza del heap;
- 127 compara directamente vocabulario completo frente a L5;
- 128-134 validan integración completa y cámara;
- 135 y 137 prueban tres drones con alcances distintos;
- 138 representa el estado ordinario finalmente entregado.

Una prueba abortada por guarda demuestra el estado del recurso hasta ese punto,
pero no valida la misión completa. Una prueba focal o sintética atribuye mejor
un cambio, pero tampoco sustituye la simulación. La conclusión final utiliza
ambas clases de evidencia.

## 25. Estado final del proyecto

Al terminar la optimización:

- `dron.number=2` está restaurado;
- el launch ordinario abre el perfil visual esperado;
- los drones arrancan escalonados cada 8 s;
- el perfil multi-dron usa el vocabulario L5;
- el vocabulario L6 continúa disponible;
- la cámara multi-dron usa 480x360 a 20 Hz y 900 features;
- el record live se escribe incrementalmente;
- el replay se consume por streaming;
- solo hay un snapshot activo globalmente;
- el worker principal sigue siendo único;
- el builder actualiza únicamente datos afectados y reutiliza transformaciones;
- las bases de datos mantienen sus responsabilidades originales;
- build final: cuatro de cuatro paquetes correctos;
- regresión C++: 37/37;
- regresión web: 8/8;
- simulación de carga real: tres drones conseguida;
- simulación visual final: dos drones conseguida, PSI cero.

La subfase 3G quedó concluida. El problema inicial de bloqueo no se ocultó
reduciendo la duración de la prueba ni eliminando permanentemente las
interfaces: se redujeron las causas estructurales y después se restauró y validó
el uso normal.

## 26. Conclusión

El bloqueo original fue el resultado acumulativo de memoria ORB privada,
duplicación temporal de mapas, journal/replay no acotados y reconstrucción
derivada demasiado costosa. El mayor ahorro de RAM vino del vocabulario L5; el
mayor ahorro de CPU, de convertir el builder en una actualización realmente
incremental con snapshots batch y cache por KF. Record/replay streaming y la
puerta global de snapshots eliminaron crecimientos y picos que empeoraban las
sesiones largas.

El resultado final mantiene el diseño funcional de 3A-3G, funciona con dos
drones en el perfil visual y ha sido sometido a una misión real con tres drones.
Existe margen suficiente para continuar, pero las fases 4-9 deben tratar el
rendimiento como un contrato vivo: medir PSS/CPU/PSI, evitar copias completas y
volver a validar L5 cuando entren relocalización y loops reales.
