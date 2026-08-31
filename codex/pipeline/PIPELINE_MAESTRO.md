# PIPELINE_MAESTRO — Proyecto multi-dron ORB-SLAM3 + ROS 2 + Gazebo

## Invariante global de system_architecture y observabilidad

A partir de este acuerdo, toda subfase futura que altere cualquiera de los elementos
siguientes debe actualizar `system_architecture` en esa misma subfase:
- paquetes o pertenencia a grupo;
- dependencias build/config/deployment entre paquetes;
- topic/service/action/TF relevante;
- relación cross-group;
- réplica de configuración;
- productor/consumidor de un flujo runtime.

La actualización incluye topología declarativa, metadata, telemetría live cuando
corresponda y guardas/tests. La subfase no se considera cerrada si la arquitectura real
y el visualizador divergen.

Los visualizadores web son opcionales y separados. Un debug desactivado debe eliminar
su trabajo específico, no solo ocultar la ventana. Esta regla cubre Fases 4–9 sin
necesidad de añadir texto anticipado en todas sus subfases; se añaden referencias
explícitas únicamente en las subfases de Fase 4/5 que ya sabemos que cambian el grafo.

## Objetivo final

El objetivo final no es solo construir una nube sparse común. El objetivo final es construir una nube densa global de un entorno definido por YAML, usando varios drones, sin ground truth para estimar la pose final ni para colocar puntos en el mapa final.

La nube sparse global es la base necesaria para:

- anclar submapas;
- evitar dobles paredes;
- estimar pose global de cada dron;
- servir como soporte geométrico para la nube densa;
- validar coherencia multi-dron antes de reconstrucción densa.

## Estados posibles

| Estado | Significado |
|---|---|
| `realizado` | Implementado, compilado, probado y documentado. |
| `actual` | Trabajo activo o siguiente punto de entrada. |
| `sin hacer` | Todavía no debe implementarse. |
| `bloqueado` | No puede avanzar por dependencia pendiente. |
| `opcional` | Mejora futura no imprescindible. |
| `legacy` | Plan antiguo o referencia histórica; no es planificación activa. |

## Reglas globales

1. No usar ground truth para mapa sparse, nube densa, pose final, score ni loops.
2. El GT solo puede usarse para fiducial simulado, debug o métricas externas.
3. La unidad correcta es `submapa = (drone_id, map_epoch)`.
4. El mapa publicado debe ser fused, no unión bruta de `MapPoints`.
5. Fiduciales son observaciones absolutas; no son loops.
6. BoW solo genera candidatos; la geometría de subnube confirma o rechaza loops.
7. `orbslam3_server` debe tender a adaptador ROS 2.
8. `orbslam3_multi` debe tender a backend algorítmico.
9. Las poses locales ORB-SLAM3 no se sobrescriben: `RawMapDatabase` conserva raw y `GlobalPoseStore` conserva global.
10. Una subfase solo se marca `realizado` si hay build, pruebas, logs e historial.
11. La ingesta raw, las poses indispensables y la publicacion forman un flujo
    principal que no espera trabajo geometrico o de optimizacion.
12. El trabajo secundario usa una cola priorizada, un unico worker persistente
    y una tarea activa como maximo.
13. Una tarea activa no se interrumpe; las tareas fiduciales preceden a loops
    pendientes y se conserva FIFO dentro de cada prioridad.
14. Una `LoopTask` contiene desde BoW hasta fusion y/o optimizacion; sus etapas
    no crean tareas independientes.
15. Los calculos largos usan snapshots privados. Los mutex se limitan a cambios
    breves de cola y commits validados.

## Preparación y ejecución por Codex

Una primera orden de ejecutar una subfase no autoriza cambios ni pruebas. Codex
debe aplicar primero la puerta de preparación de `AGENTS.md` y
`.agents/skills/ejecutar-fase/SKILL.md`: explicar, debatir, comprobar comprensión
mutua, acordar prueba y criterios, cerrar dudas y recibir autorización posterior.

Si existe un acuerdo previo completo y verificable, no se repiten preguntas y
una orden posterior puede autorizar la ejecución. Cualquier duda funcional,
cambio de alcance, prueba o criterio suspende esa autorización hasta que el
usuario confirme cómo continuar.

Solo con la puerta cerrada, usar el contrato `subfase_*.md` y seguir:

1. registrar acuerdo y autorización;
2. planificar;
3. modificar solo lo permitido;
4. compilar paquetes definidos;
5. reducir y diagnosticar build si falla;
6. ejecutar pruebas Gazebo/replay acordadas;
7. reducir logs antes de leerlos;
8. analizar solo reducidos/sublogs y regenerarlos con otros patrones si faltan
   marcadores; nunca abrir el log completo;
9. actualizar documentación e historial;
10. concluir `CONSEGUIDA`, `NO CONSEGUIDA`, `PARCIAL` o `BLOQUEADA`.

## Fases principales

| Fase | Estado | Nombre | Conclusión breve |
|---|---|---|---|
| 1 | `realizado` | Control del dron en simulación | Cadena Gazebo/Xacro/YAML/control/trayectorias/GUI de simulación documentada como base histórica. |
| 2 | `conseguida` | Separación servidor/dron/simulación | Separación, configuración, visualizadores, guardas y validación final completas. |
| 3 | `actual` | Mapa sparse global multi-dron | Reabierta solo en 3Q para corregir optimizaciones observadas en la prueba 213. |
| 4 | `realizado` | Fiducial real sin ground truth funcional | Cerrada con 4A-4H; 4I queda aplazada como regresión opcional. |
| 5 | `en curso` | Pose global de cada dron sin ground truth | 5A-5E conseguidas; 5F parcial; 5G-5H parciales; X/Y cercana/Z validados y yaw falla en 338. |
| 6 | `sin hacer` | Tareas y trayectorias de mapeo | Generará misiones desde ROI/YAML, replanning, obstáculos locales y reservas dron-dron. |
| 7 | `sin hacer` | GUI 3D propia de operación | GUI C++/Qt/OpenGL independiente de RViz2 y del visualizador web entregado en Fase 3. |
| 8 | `sin hacer` | Nube densa global multi-dron | Reconstrucción dense en servidor a partir de estéreo, poses globales y sparse. |
| 9 | `opcional` | Mejoras avanzadas y robustez | Placeholder futuro; sus subfases se definirán cuando toque avanzar ahí. |

La numeracion refleja el pipeline nuevo completo. Las Fases 2 y 4 están
conseguidas. El cierre previo de Fase 3 se reabre únicamente en 3Q para resolver
los errores de optimización observados en la prueba 213. Fase 4 permanece
cerrada con 4A-4H; 4I queda aplazada. Por petición explícita, 5A se completó
documentalmente en paralelo y 5B quedó validada en la prueba 225. El bloque
5C-5E quedan conseguidas y 5F permanece parcial. 5G-5H ya estan implementadas;
la prueba 266 valida el reanclaje visual SMALL y reduce fuertemente la energia
angular. La 267 prueba anclaje moderate completo, pero vuelve a caer a
`5.56 s` y acumula energia mas deprisa tras el primer anclaje. El gate raw
posterior pasa 46/46 GTests. La prueba 268 valida ese gate, pero un anclaje
moderate plausible dispara `+0.046416 J` y fallback en `0.88 s`. La bateria
269-272 demuestra despues que GT perfecto ya falla al atravesar el predictor
20->50 Hz; 80 ms y el timing realista de 268 lo agravan. La siguiente decision
debe corregir coherencia temporal pose/omega antes de `Delta_target`. E/F/G
273-275 completan y son disipativas al usar omega GT exacta; funcionan tanto
el hold como la extrapolacion SO(3). La causa principal queda aislada en la
derivacion/filtrado de `omega_motion`.
Las pruebas 276-277 sustituyen esa derivacion por un estimador causal de tres
poses y completan dos hovers con pose GT 20 Hz; faltan delay/jitter y ORB real.
La prueba 278 añade 80 ms y falla con clamp `72.2 %` y energia positiva; la
bateria se detiene antes de jitter y ORB real.
282 elimina el clamp sin estabilizar y 284 con aceleracion constante falla aun
antes; ambas quedan solo como diagnostico y 279 permanece detenida.

Los archivos específicos de pipeline de fases futuras son contratos
preparatorios, no autorización de ejecución. Codex debe tratarlos como no
ejecutables hasta que el usuario pida preparar o ejecutar una fase/subfase y se
cierre la puerta de preparación correspondiente.

## Fase 3 entregada

Documento específico:

```text
codex/pipeline/fase_3_sparse_global/pipeline_fase_3.md
```

Subfases activas:

```text
codex/pipeline/fase_3_sparse_global/subfases/subfase_3A.md
...
codex/pipeline/fase_3_sparse_global/subfases/subfase_3T.md
```

La planificacion antigua `12R-*`, `13`, `14` y `15` fue retirada por 3T. Sus
aprendizajes verificables permanecen en historiales y Git.

## Objetivo de Fase 3

Construir un mapa sparse global multi-dron coherente, estable, en frame `world`, con submapas por `(drone_id, map_epoch)`, anchors fiduciales, loops geométricamente verificados, fused landmarks, score centralizado, optimización segura y publicación depurada.

La arquitectura objetivo separa:

```text
flujo principal
  wrapper -> servidor -> PrimaryQueue -> PrimaryWorker
           -> RawMapDatabase -> ChangeSet
           -> GlobalPoseStore / CovisibilityDatabase / LandmarkScoreManager
           -> GlobalMapBuilder -> RViz2

flujo secundario
  cola priorizada -> SecondaryTaskWorker
                   -> FiducialOptimizationTask o LoopTask
                   -> commit breve en bases derivadas

observabilidad
  eventos ligeros -> diagrama JavaScript creado en 3B y ampliado por subfase
```

`RawMapDatabase` conserva datos ORB-SLAM3 crudos. `GlobalPoseStore` conserva
anchors y poses world. `FusedLandmarkManager` y `LandmarkScoreManager` poseen
tracks y score. El servidor coordina transiciones explicitas; ninguna de esas
bases ejecuta calculo secundario durante una escritura raw. 3C crea el flujo
principal y su backpressure basico; 3K añade solo la cola y el worker
secundarios, y la auditoria de rendimiento absorbida valida la politica final.

## Criterio de fin de Fase 3

La Fase 3 termina cuando:

- varios drones construyen un mapa sparse global común;
- no hay dobles paredes persistentes en escenarios normales;
- los submapas por `(drone_id, map_epoch)` se mantienen correctamente;
- fiduciales iniciales anclan submapas;
- segundas visitas a fiducial crean optimización si hay error real;
- loops se validan con subnubes/matching/RANSAC;
- loops buenos fusionan landmarks;
- loops con error alto crean optimización por el pipeline común;
- las poses optimizadas se guardan en `GlobalPoseStore`;
- rollback restaura poses, correcciones, scores y fused tracks afectados;
- la nube publicada usa fused tracks y scores centralizados;
- `global_map_server.cpp` deja de ser el centro algorítmico;
- la ingesta y la publicacion continuan mientras una tarea secundaria esta
  activa;
- existe una sola tarea secundaria activa y se respeta la prioridad acordada;
- cada tarea termina al hacer commit, sin esperar la salida visual;
- existe observabilidad suficiente en logs, RViz2 y el diagrama de flujo para
  separar paquetes y avanzar a fiducial real, pose sin GT, tareas, GUI y dense,
  sin que esa observabilidad gobierne el pipeline.

## Fases posteriores

### Fase 2 — Separación servidor/dron/simulación

Completada el 2026-08-24: grupos físicos de Servidor, Dron y
Simulación, builds aislados, YAML con ownership claro, dos visualizadores
independientes y guardas contra divergencias. Consultar
`fase_2_separacion_paquetes/RESULTADO_FINAL_FASE_2.md`.

### Fase 4 — Fiducial real

Reemplazar el fiducial simulado basado en GT por observaciones visuales de tags
ligadas al KeyFrame exacto. ORB-SLAM3 no detecta fiduciales; el wrapper detecta
tags en la imagen izquierda exacta y el servidor interpreta cubos/tags.

Completada con alcance 4A-4H. La regresión 4I con perfil ESP32-CAM queda
aplazada y requerirá preparación propia si se retoma.

### Fase 5 — Pose global sin GT

Usar ORB-SLAM3 local, correcciones globales de KFs y un estimador embarcado
ligero para publicar pose y velocidad sin `sensor/GT/pose` ni `sensor/GT/vel`
como entradas funcionales del control.

5A queda conseguida documentalmente. 5B implementa y valida el recibo ORB
coherente, `NavigationState`, continuidad `O_T_B` intra-epoch y gate de goals.
El contrato separa `O_T_B` continuo de
`W_T_B` corregible y conserva `GT_FALLBACK` temporal, visible y aislado de
mapa/global hasta que Fase 6 implemente recovery real. La antigua 5I está
absorbida en 5H.

El laboratorio 314-317 valida una `v_hat(t_k)` causal y el estado completo
dinamico bajo timing/jitter sin fallback. Tras corregir la poda para conservar
el predecesor ZOH, 318R2/319R validan cobertura y paridad. La 320 inicial es
invalida por ejecutar `legacy`; corregida la precedencia del launch, 320R usa
`dynamic`, pero el dron no sigue correctamente la aproximacion ORB y llega al
siguiente goal cerca del suelo con velocidad alta antes de perder tracking.
320R2R elimina esa ambiguedad: tras aproximacion GT y handoff limpio, el hover
diverge con tracking `2`, sin fallback ni missing. La Fase 5 permanece parcial;
321B confirma que `p_ORB+v_GT+angular_ORB` es estable, mientras 321AR/321D
divergen con `v_ORB`. MIDPOINT_DYNAMIC sustituye despues la velocidad
THREE_SAMPLE amplificada; 326-329 validan cobertura y precision productiva, y
330/331 completan de forma reproducible un hover ORB real sin fallback ni
tracking no-OK. 332/333 validan X; 334 atraviesa por error el fiducial 2 y
queda invalida por colision. Y debe repetirse alejandose del obstaculo; el STOP
deja Y/Z/yaw/combinacion y la trayectoria representativa pendientes. 334R
evita el obstaculo y conserva tracking, pero no lleva la velocidad ORB a cero
en el hover final; el STOP permanece por fallo funcional de frenado Y.
El diagnostico posterior 334R3R/335R valida Y con mejor cobertura visual y
336/337R valida Z. La bateria se detiene en 338: yaw degrada los errores y
termina en perdida de tracking con GT fallback; 339-343 quedan pendientes.

No ejecutar 5C/5D contra un backend 3Q inestable: verificar primero el cierre
vigente de 3Q. Cada subfase funcional requiere preparación y autorización.

### Fase 6 — Tareas y trayectorias

Generar y coordinar misiones de mapeo sparse desde `tarea_principal.yaml`, ROI,
trayectorias cortas, percepción local de obstáculos y reservas espaciales entre
drones.

No ejecutar ni completar su pipeline específico mientras Fase 3 siga activa,
salvo petición explícita del usuario.

### Fase 7 — GUI propia

Crear una GUI 3D de escritorio en C++/Qt/OpenGL para operación multi-dron. El
visualizador JavaScript entregado en Fase 3 es diagnostico de flujo de datos y no sustituye
esta GUI funcional.

No ejecutar ni completar su pipeline específico mientras Fase 3 siga activa,
salvo petición explícita del usuario.

### Fase 8 — Nube densa global

Construir la nube densa global en servidor usando estéreo, poses globales, mapa
sparse, subnubes dense, occupancy y reintegración tras optimizaciones, sin usar
GT para colocar puntos.

No ejecutar ni completar su pipeline específico mientras Fase 3 siga activa,
salvo petición explícita del usuario.

### Fase 9 — Mejoras avanzadas

Reservada para robustez, nuevos mundos, reflejos, objetos dinámicos, estrés,
evaluación y mejoras que no deben convertirse en requisito oculto de fases
anteriores. La carpeta existe como placeholder; sus subfases se realizarán
cuando el proyecto avance hasta esa fase.

No ejecutar ni completar su pipeline específico mientras Fase 3 siga activa,
salvo petición explícita del usuario.
