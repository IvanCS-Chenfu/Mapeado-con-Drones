# Subfase 5I — Integración final, adquisición inicial de anchor y regresión multi-dron

## Puerta arquitectónica de cierre de Fase 5

La prueba final multi-dron debe verificar: ninguna dependencia funcional de GT en
control; Server↔Dron solo mediante `orbslam3`; flujos locales correctos hacia control;
`system_architecture` coherente en modo estático/live; herramienta apagada sin coste de
telemetría; cerrar visualizadores no afecta al pipeline.

## Estado

```text
sin hacer
```

## Objetivo técnico

Cerrar la Fase 5 con una integración completa y reproducible en la que los drones arranquen sin pose global, puedan moverse usando pose local, busquen un fiducial para anclarse y, una vez globalizados, ejecuten una prueba larga multi-dron usando pose y velocidad estimadas sin Ground Truth funcional.

La prueba principal no debe empezar artificialmente con el sistema ya anclado. Debe demostrar la cadena completa:

```text
dron en suelo
  -> ORB tracking local
  -> LOCAL_ONLY
  -> despegue/movimiento relativo
  -> búsqueda visual de fiducial
  -> anchor
  -> GLOBAL_VALID
  -> finalización del goal local en su frame original
  -> goals globales posteriores
  -> prueba larga de dos drones alrededor del edificio
```

Además debe incluir regresiones de pérdida de globalización y pérdida/relocalización de ORB para comprobar que los comportamientos de 5B/5G/5H sobreviven a la integración final.

Ground Truth puede registrarse para métricas finales, pero ningún nodo del Dron ni del Servidor puede usarlo para pose, velocidad, control, selección de KFs, recuperación o aceptación de trayectorias.

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
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5B.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5C.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5D.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5E.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5F.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5G.md
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5H.md
```

Leer los resúmenes de historial reales `5A_RESUMEN` … `5H_RESUMEN`. No ejecutar 5I si 5H no está `CONSEGUIDA` o si 5F/5G dejaron una duda de calidad sin aceptación.

Leer también:

```text
codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/orbslam3_ros2/
codex/contexto/paquetes/orbslam3_multi/
codex/contexto/paquetes/orbslam3_server/
codex/contexto/paquetes/simulacion_dron/
```

Leer el contrato vigente de Fase 4 para el detector fiducial real/simulado visual y el mecanismo exacto de anchor/revisit.

## Diagnóstico de partida

Al llegar a 5I deben existir ya, por separado:

- navegación `LOCAL_ONLY` sin anchor;
- semántica de absolutos tratados como relativos cuando no existe global;
- correcciones de KFs desde `GlobalPoseStore`;
- transporte versionado servidor→dron;
- pose local-global a frecuencia ORB;
- decisión raw/smoothed basada en 5F;
- velocidad lineal/angular estimada y aceleración debug opcional;
- recuperación ciega limitada ante `LOCALIZATION_LOST`;
- `gen_tray` y `control_calcular_fuerzas` sin GT funcional.

Lo que falta es demostrar que estas piezas funcionan juntas durante un escenario largo y multi-dron, incluyendo el arranque sin anchor.

La Fase 5 no se considera cerrada si la única prueba exitosa comienza con `world_T_local` ya disponible o si el escenario de búsqueda necesita un script que use GT para saber hacia dónde ir.

## Archivos permitidos a modificar

La integración final debe evitar rediseñar componentes ya validados. Paths finales fijados por 5A. Baseline:

```text
src/simulacion_dron/src/control_tray/scenario_runner_node.cpp
src/simulacion_dron/src/control_tray/*fase5*          # mini programa/escenario de adquisición si se crea separado
src/simulacion_dron/launch/multi_dron.launch.py
src/simulacion_dron/config/*
src/simulacion_dron/CMakeLists.txt
src/simulacion_dron/package.xml
src/simulacion_dron/worlds/*                         # solo colocación/recursos ya definidos por Fase 4

codex/archivos_auxiliares/trayectorias/*
codex/archivos_auxiliares/repeticiones/*             # si la política vigente lo usa para replay

src/dron_individual/launch/*                         # solo ajustes de integración mecánicos
src/orbslam3_server/launch/*                         # solo ajustes de integración mecánicos

codex/contexto/paquetes/simulacion_dron/
codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/orbslam3_server/
codex/pipeline/fase_5_poses_drones_sin_gt/
```

Si una prueba revela un fallo funcional de 5B–5H, suspender 5I y reabrir la subfase propietaria. No corregir algoritmos de pose/velocidad dentro del scenario runner.

## Archivos prohibidos

No modificar durante una simple corrección de integración:

```text
ORB_SLAM3/**
src/orbslam3_multi/**
src/orbslam3_server/src/global_map_server.cpp
src/dron_individual/src/control_tray/control_calcular_fuerzas.cpp
src/dron_individual/src/control_tray/gen_tray.cpp
<estimador de pose/velocidad 5E/5G>
```

salvo que la evidencia obligue formalmente a reabrir la subfase correspondiente y se reciba nueva autorización.

No usar GT para gobernar el mini programa de búsqueda.

## Funciones, clases o nodos que hay que localizar

5A/5H deben haber fijado nombres reales. Baseline:

```text
scenario_runner_node
AccionTrayectoria
<nodo/mini programa de adquisición inicial de anchor>
<localization status topic>
<estimated pose topic>
<estimated velocity topic>
<Fase 4 fiducial detection/status topic>
<anchor/global status topic>
<GlobalMapServer>
<estimador embarcado 5E/5G>
gen_tray
control_calcular_fuerzas
aplicar_fuerzas_dron
```

Topics de GT permitidos solo en nodos de métrica:

```text
sensor/GT/pose
sensor/GT/vel
sensor/GT/acc
```

## Cambios requeridos

1. Crear un mini programa/escenario reproducible de adquisición inicial de anchor. Debe controlar la secuencia mediante estados funcionales, no mediante GT.

2. Estado inicial obligatorio:

```text
submapa sin anchor
ORB tracking válido
localization_state = LOCAL_ONLY
```

3. El mini programa debe esperar explícitamente a que exista pose local válida antes de pedir movimiento. No debe esperar pose global.

4. Ejecutar un despegue/movimiento inicial relativo. La forma exacta debe respetar el control/TrayAction vigente; objetivo de referencia acordado:

```text
subir aproximadamente 1 m de forma relativa
```

5. Tras estabilizarse, ejecutar una maniobra de búsqueda de fiducial basada únicamente en comandos relativos. Estrategia inicial acordada:

```text
giro de 360° / barrido de yaw relativo
```

   Si Fase 4 demuestra que otra secuencia local mínima es más adecuada, 5A/5I puede concretarla sin introducir planificación de Fase 6.

6. Durante la búsqueda, el detector fiducial de Fase 4 debe operar sobre imágenes reales/simuladas de cámara. El mini programa no puede consultar la pose GT del fiducial o del dron para decidir el movimiento.

7. Al detectarse y aceptarse el anchor:
   - `GLOBAL_VALID` puede empezar a existir;
   - si el giro/despegue local sigue activo, terminarlo en local según la regla de 5B;
   - no convertir el goal en curso a world;
   - esperar su `RESULT` o cancelación normal antes de iniciar el primer goal absoluto.

8. Verificar que el estimador recibe correcciones de 5C/5D y publica pose global a frecuencia ORB.

9. Después del anchor y de terminar el goal local, iniciar automáticamente la prueba larga global.

10. Prueba larga principal: dos drones realizan el recorrido representativo alrededor del edificio/casa definido por el proyecto, usando sus namespaces y estimadores propios. No compartir estado entre drones por error.

11. La prueba larga debe ejercer:

```text
varios KFs de referencia
correcciones del servidor
al menos una optimización/revisit si el escenario la produce
movimiento X/Y/Z y yaw
cambio de revisiones
```

12. Mantener GT externo para métricas finales por dron:

```text
RMSE pose
RMSE velocidad
máximo pose
máximo velocidad
tracking lost count/duration
recovery count/success
goal success/failure
frecuencia pose estimada
p50/p95 processing delay
```

13. Incluir una regresión de pérdida global con ORB válido durante un goal world:
   - cancelar goal world;
   - pasar a local;
   - ejecutar movimiento local controlado;
   - recuperar global mediante el mecanismo real de Fase 3/4;
   - siguiente goal puede volver a ser world.

14. Incluir una regresión de pérdida ORB recuperable:
   - provocar zona sin features de forma reproducible;
   - entrar en recovery de 5G;
   - comprobar recuperación de tracking;
   - no reanudar automáticamente el goal cancelado;
   - si Fase 3 reconoce una zona ya globalizada, recuperar global sin exigir necesariamente volver al fiducial.

15. Incluir una regresión de pérdida ORB no recuperable/timeout en una prueba separada o al menos smoke test, para comprobar que el dron no continúa a ciegas indefinidamente.

16. Añadir markers de integración equivalentes a:

```text
[F5I-BOOT] drone=... state=LOCAL_ONLY
[F5I-SEARCH-START] ...
[F5I-ANCHOR-ACQUIRED] drone=... epoch=...
[F5I-LOCAL-GOAL-FINISHED-AFTER-ANCHOR]
[F5I-LONG-START] drones=2
[F5I-LOSS-GLOBAL]
[F5I-LOSS-ORB]
[F5I-RECOVERED]
[F5I-LONG-RESULT] drone=... success=...
[F5I-METRICS] ...
```

17. El mini programa debe fallar de forma explícita si:
   - no aparece pose local;
   - no se encuentra fiducial dentro del tiempo/maniobra acordados;
   - el anchor no se vuelve globalizable;
   - un goal no termina;
   - el estimador deja de publicar;
   - se detecta dependencia funcional de GT.

18. No ocultar una pérdida de anchor reiniciando Gazebo o reposicionando el dron con servicios de simulación durante la misma prueba.

19. Antes de declarar cierre, ejecutar la prueba completa al menos el número de repeticiones que 5A/5I acuerde para detectar fallos no deterministas. Si una única ejecución pasa y otras fallan, no declarar estabilidad.

20. Comparar los resultados contra las métricas de 5F/5G. Si el error empeora significativamente al cerrar el lazo de control, reabrir la subfase responsable y conservar la evidencia de 5I fallida.

21. Verificar estáticamente y en runtime que los nodos funcionales Dron/Servidor no se suscriben a GT, salvo nodos de métricas/Simulación explícitamente etiquetados.

22. Al cerrar la fase, actualizar el estado/pipeline general solo con evidencia real. No marcar Fase 5 cerrada si alguna prueba obligatoria queda `PARCIAL`.

## Cambios prohibidos

- No colocar el dron en la pose correcta mediante GT antes de empezar.
- No iniciar la prueba larga con anchor preinyectado como único caso de éxito.
- No usar GT para decidir dónde está el fiducial o cuándo girar/parar.
- No cambiar una trayectoria local a global a mitad.
- No reanudar automáticamente un goal cancelado tras recovery.
- No usar un único estimador compartido para dos drones.
- No reiniciar el mundo como mecanismo de recuperación dentro de la prueba.
- No modificar algoritmos de 5C–5G desde el scenario runner.
- No relajar umbrales o desactivar comprobaciones solo para que la prueba larga pase.
- No declarar éxito si los goals terminan pero la pose/velocidad divergen de forma inaceptable.

## Paquetes a compilar

Primero comprobar que el grupo Dron sigue compilando aislado:

```bash
./codex/herramientas/build_selected_packages.sh lib_tray dron_individual
```

Después Servidor y Simulación según la estructura de Fase 2. Baseline:

```bash
./codex/herramientas/build_selected_packages.sh orbslam3_multi orbslam3_server
./codex/herramientas/build_selected_packages.sh simulacion_dron
```

Añadir `orbslam3`/interfaces solo si se han tocado en una corrección autorizada.

## Pruebas Gazebo requeridas

### Prueba 1 — Adquisición inicial de anchor desde el suelo

Secuencia obligatoria:

1. arrancar un dron sin anchor;
2. confirmar `LOCAL_ONLY`;
3. esperar pose local;
4. despegar/subir relativamente;
5. realizar búsqueda relativa de fiducial;
6. detectar/aceptar anchor;
7. comprobar `GLOBAL_VALID`;
8. terminar el goal local en curso;
9. ejecutar un goal absoluto corto;
10. registrar métricas externas.

### Prueba 2 — Dos drones: adquisición + prueba larga alrededor del edificio

Ambos drones deben ejecutar su propia adquisición inicial. Si el escenario usa un fiducial cercano común o varios fiduciales, seguir el contrato de Fase 4.

Tras globalizarse:

```text
dron_1 -> recorrido largo A
dron_2 -> recorrido largo B
```

La trayectoria representativa debe corresponder a la prueba típica del proyecto: vuelta completa alrededor del edificio/casa por dos drones, con duración suficiente para generar varios KFs, revisiones y correcciones.

Comando orientativo, a sustituir por el exacto fijado por 5A:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase5_5I_long_2drones \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 30
```

### Prueba 3 — Pérdida global con ORB válido

Durante un goal world:

1. invalidar la relación global por el mecanismo reproducible acordado;
2. comprobar cancelación del goal;
3. mantener pose local;
4. ejecutar un goal relativo;
5. recuperar relación global;
6. comprobar que un nuevo goal world vuelve a ser posible.

### Prueba 4 — Pérdida ORB recuperable

1. volar con tracking;
2. provocar `LOST`;
3. comprobar cancelación normal y recovery ciego;
4. recuperar landmarks;
5. comprobar tracking recuperado;
6. comprobar local/global según Fase 3/4;
7. enviar un goal nuevo.

### Prueba 5 — Pérdida ORB no recuperable

Mantener ausencia de landmarks hasta superar límites de recovery. Comprobar que no existen comandos ciegos indefinidos y que no se usa GT.

### Prueba 6 — Repetición/regresión

Repetir la prueba principal el número de veces acordado en preparación. Registrar cada ejecución cronológicamente, incluso fallos intermitentes.

## Patrones de reducción de logs

### Adquisición/long

```text
F5I-BOOT|F5I-SEARCH-START|F5I-ANCHOR-ACQUIRED|F5I-LOCAL-GOAL-FINISHED-AFTER-ANCHOR|F5I-LONG-START|F5I-LONG-RESULT|F5I-METRICS|F5E-STATS|F5G-VEL-METRICS|GOAL|RESULT|success|ERROR|FATAL|Segmentation fault|Killed
```

### Pérdidas

```text
F5I-LOSS-GLOBAL|F5I-LOSS-ORB|F5I-RECOVERED|F5B-STATE|F5G-RECOVERY|TRACKING|LOST|RELOCAL|GOAL|RESULT|ERROR|FATAL|Segmentation fault|Killed
```

### Ausencia de GT funcional

```text
sensor/GT|F5H-GT-FREE|subscription|subscriber|ERROR|FATAL
```

No interpretar la mera presencia de GT en nodos de métricas como fallo. La evidencia debe distinguir consumidores funcionales de consumidores de Simulación/diagnóstico.

## Criterio de éxito

La subfase y la Fase 5 se consideran `CONSEGUIDAS` solo si:

1. el build aislado de Dron y los builds de Servidor/Simulación pasan;
2. un dron arranca sin anchor y obtiene pose local sin GT;
3. puede despegar/moverse relativamente para buscar un fiducial;
4. obtiene anchor visual y pasa a global;
5. el goal local que estaba activo no cambia de frame al aparecer global;
6. el siguiente goal absoluto se ejecuta con pose/velocidad estimadas;
7. dos drones completan la prueba larga alrededor del edificio usando sus propias estimaciones;
8. la pose estimada mantiene la frecuencia/latencia aceptadas en 5E/5F;
9. la velocidad mantiene la calidad aceptada en 5G;
10. una pérdida global con ORB válido pasa correctamente a local;
11. una pérdida ORB recuperable activa recovery y recupera tracking sin GT;
12. una pérdida no recuperable respeta límites y no navega indefinidamente a ciegas;
13. Fase 3 puede recuperar globalización al reconocer una zona ya conocida cuando aplique, sin exigir siempre otro fiducial;
14. `gen_tray` y `control_calcular_fuerzas` no consumen GT funcionalmente;
15. GT aparece únicamente en métricas externas;
16. no hay errores graves, mezcla de epochs, saltos no explicados o divergencias persistentes;
17. las repeticiones acordadas no revelan un fallo intermitente incompatible con el cierre;
18. historial, estado, pipeline y documentación de paquetes quedan coherentes.

## Criterio de fallo o parcial

- `NO CONSEGUIDA`: no se puede adquirir anchor sin GT, la prueba larga necesita GT funcional, un dron no completa el recorrido, se mezclan frames/epochs o recovery puede seguir a ciegas sin límite.
- `PARCIAL`: la cadena completa funciona pero persisten errores/latencias/fallos intermitentes que impiden usarla como base fiable para Fase 6.
- `BLOQUEADA`: Fase 4 no proporciona un fiducial visual utilizable, Fase 3 no proporciona `GlobalPoseStore`/relocalización requerida o falta una dependencia externa crítica.

Si falla por un componente anterior, reabrir la subfase propietaria y conservar la ejecución fallida de 5I en historial. No editar la conclusión pasada como si la prueba hubiese pasado.

## Documentación a actualizar

Al cierre real:

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/contexto/07_ULTIMA_SESION.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/INDEX.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5I.md
codex/pipeline/fase_5_poses_drones_sin_gt/historial/por_subfase/historial_5I_RESUMEN.md
```

Actualizar también todos los `codex/contexto/paquetes/<paquete>/` modificados durante la integración.

Si Fase 5 queda cerrada, dejar `00_CONTEXTO_COMPACTACION.md` con trabajo activo explícitamente vacío o con la siguiente fase acordada; no iniciar Fase 6 automáticamente.
