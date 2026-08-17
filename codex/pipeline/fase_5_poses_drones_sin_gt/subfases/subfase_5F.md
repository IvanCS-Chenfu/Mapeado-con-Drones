# Subfase 5F — Validación de pose contra GT y experimento de suavizado

## Estado

```text
sin hacer
```

## Objetivo técnico

Medir de forma cuantitativa y visual la calidad de la pose obtenida en 5E y comparar, con el mismo escenario y timestamps, dos salidas:

```text
pose global raw / sin suavizar
pose global suavizada
```

contra Ground Truth utilizado exclusivamente como métrica externa de simulación.

Esta subfase NO decide de antemano que el suavizado deba quedar activo. Su función es producir evidencia sobre:

- error de posición y orientación;
- continuidad y saltos ante correcciones del servidor;
- frecuencia de publicación;
- delay añadido por el estimador y por el suavizado;
- respuesta durante cambios de KF/revisión/optimización.

Después de obtener los resultados, Codex debe detenerse y presentarlos al usuario. La autorización funcional se considera suspendida hasta que el usuario elija explícitamente una de estas opciones:

```text
A. mantener el suavizado probado;
B. modificar el suavizado y repetir;
C. eliminar el suavizado y usar raw;
```

La decisión podrá revisarse más adelante en hardware real comparando de nuevo raw vs smoothed.

La subfase también actúa como puerta de realimentación hacia 5C: si la gráfica demuestra que el error de pose proviene de selección/corrección de KFs aunque 5C hubiese pasado su prueba propia, se reabre 5C y se repiten las subfases afectadas.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5A.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5C.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5D.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5E.md
```

Leer historiales/resúmenes reales de 5C–5E, especialmente:

- frecuencia/delay de `pose_local`;
- ratio de reference/nearest/fallback;
- saltos raw observados tras optimización;
- correcciones/revisiones usadas.

Documentación:

```text
codex/contexto/paquetes/orbslam3_server/global_pose_corrector.md
codex/contexto/paquetes/dron_individual/00_summary.md
codex/contexto/paquetes/simulacion_dron/graficas_y_gui.md
codex/contexto/paquetes/simulacion_dron/launches.md
```

5A debe haber fijado los topics exactos de pose raw/local/global y el mecanismo de gráfica namespaced.

## Diagnóstico de partida

El baseline histórico `global_pose_corrector` ya contiene una implementación de suavizado basada en:

```text
enable_pose_smoothing
publish_smoothed_pose_on_main_topic
pose_smoothing_alpha
max_smoothed_translation_step_m
max_smoothed_yaw_step_deg
reset_smoothing_if_raw_jump_m
reset_smoothing_if_raw_jump_yaw_deg
reset_smoothing_if_no_publish_s
```

y publica raw/smoothed separadas.

Esa implementación es referencia, no decisión final. 5A/5E deben haber determinado si se reutiliza/migra o se implementa el experimento en el nuevo estimador embarcado.

La infraestructura de simulación entregada ya contiene:

```text
graficar.py
graficar_GT.cpp
graficar_GTvsTray.cpp
graficar_tray.cpp
```

con el patrón `Float64MultiArray` + `UInt8MultiArray`. Debe reutilizarse la filosofía sin mezclar datos de drones distintos.

No existe todavía una prueba formal que demuestre que la pose de 5E tiene error suficientemente bajo para sustituir GT en control.

## Archivos permitidos a modificar

5A debe fijar paths definitivos. Baseline probable:

```text
src/dron_individual/src/<estimador_pose>*
src/dron_individual/include/<estimador_pose>*
src/dron_individual/config/*
src/dron_individual/launch/*

src/simulacion_dron/src/graficar/graficar.py
src/simulacion_dron/src/graficar/*pose*         # nuevo adaptador de comparación si hace falta
src/simulacion_dron/CMakeLists.txt
src/simulacion_dron/package.xml
src/simulacion_dron/launch/*
src/simulacion_dron/config/*
src/simulacion_dron/src/control_tray/scenario_runner_node.cpp
codex/archivos_auxiliares/trayectorias/*

codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_5_poses_drones_sin_gt/
```

Si el suavizado histórico se reutiliza desde otro paquete por decisión de 5A, mover/reimplementar solo la parte necesaria al lado Dron; no crear dependencia runtime con el paquete Servidor.

## Archivos prohibidos

```text
ORB_SLAM3/**
src/orbslam3_multi/**              # salvo reabrir 5C después de evidencia y nueva autorización
src/orbslam3_server/**             # salvo reabrir 5C/5D
src/dron_individual/src/control_tray/control_calcular_fuerzas.cpp   # 5H
src/dron_individual/src/control_tray/gen_tray.cpp                   # 5H salvo instrumentación mecánica ya acordada
build/**
install/**
log/**
```

No usar GT dentro del estimador ni para elegir dinámicamente parámetros de suavizado durante la ejecución.

## Funciones, clases o nodos que hay que localizar

Nombres exactos fijados por 5A/5E. Referencias históricas:

```text
GlobalPoseCorrector::InterpolateTransform
GlobalPoseCorrector::LocalPoseCallback
GlobalPoseCorrector::TranslationDistance
GlobalPoseCorrector::YawDifferenceDeg

<nodo estimador de 5E>
<nodo/adaptador de grafica pose estimada vs GT>
graficar.py
plugin_sensor_groundtrurh
```

Topics mínimos a correlacionar:

```text
<pose_global_raw_topic>
<pose_global_smoothed_topic>
sensor/GT/pose
<pose_status/revision topic>
```

Si se compara pose del cuerpo, GT debe corresponder al cuerpo. No comparar pose de cámara estimada contra pose GT del cuerpo sin aplicar extrínseca.

## Cambios requeridos

1. Mantener disponible una salida raw de 5E sin suavizado como referencia obligatoria.

2. Implementar o migrar una rama de suavizado experimental configurable y desconectable. Los parámetros concretos deben partir de la referencia histórica o de 5A, pero no declararse definitivos antes de medir.

3. El suavizado debe actuar principalmente sobre cambios de corrección/marco global y no filtrar innecesariamente el movimiento local físico de alta frecuencia. La implementación debe conservar el timestamp de la observación.

4. Publicar raw y smoothed simultáneamente siempre que sea posible para comparar bajo exactamente las mismas imágenes, deltas y optimizaciones.

5. Crear un adaptador de gráfica específico por dron que registre al menos:

```text
x_GT, x_raw, x_smoothed
y_GT, y_raw, y_smoothed
z_GT, z_raw, z_smoothed
yaw_GT, yaw_raw, yaw_smoothed
```

6. Añadir una vista/serie de errores:

```text
ex_raw, ey_raw, ez_raw
ex_smoothed, ey_smoothed, ez_smoothed
norm_pos_error_raw
norm_pos_error_smoothed
yaw_error_raw
yaw_error_smoothed
```

El error yaw debe normalizarse a la rama angular correcta; no restar ángulos sin wrap.

7. Calcular y guardar para raw y smoothed:

```text
RMSE de posición 3D
MAE de posición 3D
error máximo de posición
RMSE por eje
RMSE/MAE/máximo de yaw u orientación acordada
número de muestras comparadas
ventana temporal utilizada
```

8. Correlacionar por timestamp. Definir una política explícita para emparejar GT y estimación, con tolerancia máxima temporal. No comparar muestras muy separadas solo por orden de llegada.

9. Medir frecuencia y delay. Como mínimo:

```text
pose_local input Hz
raw output Hz
smoothed output Hz
processing delay raw p50/p95
processing delay smoothed p50/p95
diferencia de fase/lag observable en maniobras
```

Si no puede medirse latencia física con precisión, documentar el método y no inventar números.

10. Crear markers agregados equivalentes a:

```text
[F5F-POSE-METRICS] mode=raw rmse_m=... mae_m=... max_m=... yaw_rmse=...
[F5F-POSE-METRICS] mode=smoothed ...
[F5F-LATENCY] raw_p50_ms=... raw_p95_ms=... smoothed_p50_ms=... smoothed_p95_ms=...
[F5F-JUMP] correction_revision=... raw_jump_m=... smoothed_jump_m=...
```

11. Asegurar que GT solo entra en el nodo de métricas/gráfica de Simulación. El estimador Dron debe compilar/funcionar sin `sensor/GT/*`.

12. Ejecutar al menos una maniobra con movimiento suave y otra con cambio global significativo:
   - primer anchor;
   - optimización/revisit que cambie `C_KF`;
   - si es reproducible, cambio de KF/reference.

13. Ejecutar una prueba larga alrededor del edificio/casa. Segmentar métricas por modo de 5E (`reference`, `nearest`, `fallback`) para detectar si el error alto se correlaciona con 5C.

14. Analizar la causa antes de ajustar parámetros. Si raw ya tiene un error sistemático grande, no intentar esconderlo aumentando suavizado; reabrir 5C/5D/5E si la evidencia apunta allí.

15. Tras generar resultados, actualizar `00_CONTEXTO_COMPACTACION.md` con:

```text
Autorizacion funcional: SUSPENDIDA
Dudas abiertas: decision de suavizado pendiente del usuario
```

16. Presentar al usuario una comparación compacta raw vs smoothed con gráficas/métricas y esperar decisión explícita.

17. Solo tras la decisión:
   - si elige raw, dejar suavizado desactivado/eliminado del camino funcional, manteniendo debug si aporta valor;
   - si elige smoothed, fijar parámetros elegidos y repetir prueba de confirmación;
   - si pide mejorar, modificar la estrategia acordada y repetir A/B.

18. No declarar 5F `CONSEGUIDA` antes de la decisión del usuario y de la ejecución de confirmación de la opción elegida.

## Cambios prohibidos

- No usar GT para corregir `C_KF`, resetear pose o elegir en runtime entre raw/smoothed.
- No adoptar suavizado solo porque el código histórico lo tenía activo.
- No aumentar agresivamente el filtrado para esconder un error estructural de 5C.
- No medir solo error y omitir delay.
- No medir solo delay y omitir error.
- No comparar frames distintos cámara/body.
- No cambiar de frame a mitad de una trayectoria local para mejorar artificialmente la gráfica.
- No pasar a 5H únicamente porque la curva “parece buena”; guardar métricas numéricas.
- No decidir por el usuario entre raw y smoothed después del experimento.

## Paquetes a compilar

Según ownership real, baseline:

```bash
./codex/herramientas/build_selected_packages.sh dron_individual simulacion_dron
```

Añadir el paquete real del estimador si no es `dron_individual`.

No recompilar backend salvo que la evidencia obligue formalmente a reabrir 5C/5D/5E.

## Pruebas Gazebo requeridas

### Prueba 1 — Pose estática/hover con anchor

Objetivo: medir ruido/base sin grandes movimientos.

1. obtener anchor;
2. mantener posición/hover durante ventana definida;
3. registrar raw, smoothed y GT;
4. medir error, ruido y delay.

### Prueba 2 — Movimiento relativo/local y adquisición de anchor

1. arrancar sin anchor;
2. hacer maniobra local de búsqueda;
3. adquirir anchor;
4. mantener la trayectoria local hasta terminar según 5B;
5. observar cuándo empieza a existir pose global raw/smoothed;
6. comprobar que la métrica no interpreta la transición de frame como movimiento físico sin etiquetarla.

### Prueba 3 — Optimización durante vuelo

1. volar con pose global;
2. provocar una corrección/optimización significativa y válida;
3. medir salto raw;
4. medir respuesta smoothed;
5. medir lag adicional.

Esta prueba es crítica para decidir si el suavizado ayuda realmente al control.

### Prueba 4 — Maniobras dinámicas

Ejecutar traslación y yaw con cambios suficientemente rápidos para revelar retraso de filtrado.

Comparar raw vs smoothed vs GT sin cambiar parámetros durante la ejecución.

### Prueba 5 — Prueba larga dos drones

Recorrido representativo alrededor del edificio/casa, con Fase 3/4 activas.

Guardar por dron:

```text
metricas raw
metricas smoothed
latencia
modo de KF de 5E
revisiones/correcciones
```

### Prueba 6 — Confirmación tras decisión del usuario

Después de presentar resultados y recibir autorización:

- repetir el escenario representativo con la opción finalmente elegida;
- comprobar que no empeora frecuencia/controlabilidad respecto al A/B;
- registrar la decisión exacta en historial.

## Patrones de reducción de logs

```text
F5F-POSE-METRICS|F5F-LATENCY|F5F-JUMP|F5E-MODE|reference|nearest|fallback|correction_revision|GT|ERROR|FATAL|Segmentation fault|Killed
```

Para las gráficas, conservar archivos/CSV/artefactos de métricas según la infraestructura acordada, sin volcar grandes arrays al contexto.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. raw y smoothed se comparan sobre las mismas muestras/escenario o con una metodología equivalente reproducible;
2. GT se usa solo en Simulación como métrica externa;
3. existen gráficas de posición/orientación y errores;
4. existen RMSE/MAE/máximo y número de muestras;
5. existe medición de frecuencia y delay raw/smoothed;
6. se observa y cuantifica la respuesta ante al menos una corrección global significativa;
7. la prueba larga permite correlacionar error con reference/nearest/fallback;
8. si el error revela un fallo de 5C/5D/5E, esas subfases se reabren en lugar de esconderlo;
9. Codex presenta resultados y se detiene;
10. el usuario toma una decisión explícita sobre suavizado;
11. se ejecuta una prueba de confirmación con la decisión elegida;
12. build/pruebas terminan sin errores graves no explicados;
13. historial y docs quedan actualizados.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: GT entra en el estimador, no se mide delay, las gráficas comparan frames/timestamps incompatibles o Codex adopta una opción sin decisión del usuario.
- `PARCIAL`: se obtienen métricas válidas pero el error/latencia es demasiado alto para continuar y requiere reabrir 5C–5E o mejorar el suavizado.
- `BLOQUEADA`: no existe GT de diagnóstico sincronizable en Simulación o no puede provocarse una corrección global reproducible.

Un error alto no debe maquillarse como `CONSEGUIDA`. Si la instrumentación funciona pero la pose no es suficientemente buena, registrar `PARCIAL` y volver al origen del error.

## Documentación a actualizar

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/paquetes/dron_individual/          # o paquete real del estimador
codex/contexto/paquetes/simulacion_dron/
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/INDEX.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5F.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5F_RESUMEN.md
```

Si se reabre 5C/5D/5E, actualizar sus historiales con nuevas ejecuciones sin borrar resultados anteriores.
