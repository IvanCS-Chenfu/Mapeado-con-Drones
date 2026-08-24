# 02 — Reglas técnicas permanentes

<!-- ACUERDOS_CIERRE_F2_2026_08_24_START -->
## Reglas permanentes de grupos, configuración y observabilidad

> **Vigencia:** acuerdo cerrado el 2026-08-24. Este bloque prevalece sobre cualquier
> frase anterior incompatible del mismo documento. No borra ni reescribe evidencia
> histórica; distingue siempre entre estado actual, deuda conocida y arquitectura objetivo.

### Configuración
Distinguir siempre:
1. **ownership semántico**: qué grupo es propietario conceptual del dato;
2. **authority/control**: qué componente decide su valor;
3. **deployment source/profile**: qué copia local carga cada despliegue.

Dron se trata como caja negra de despliegue. Un YAML ROS configura al nodo que lo
carga; no distribuye parámetros a otra máquina. La carga directa de YAML entre grupos
queda prohibida.

La duplicación accidental semántica queda prohibida. Se permiten réplicas declaradas:
- parciales, con lista exacta de claves;
- completas, solo como excepción de perfil de despliegue documentada y con guarda de
  igualdad. El caso vigente es `servidor/orbslam3_server/config/global_map/` ↔
  `simulacion/simulacion_dron/config/global_map/`.

### Reloj
- Dron standalone/real: `use_sim_time=false` por defecto.
- Servidor standalone/real: `use_sim_time=false` por defecto.
- Simulación: override explícito `use_sim_time=true` a todos los nodos que dependan del
  reloj simulado.
Identidad y valores por ejecución deben tener una única autoridad visible en launch.

### Observabilidad web
Los grafos son diagnóstico, nunca dependencia funcional. Debug apagado implica:
sin bridge, sin observers, sin eventos específicos, sin JSON/serialización, sin publish,
sin HTTP/SSE y sin navegador. `pipeline_flow` y `system_architecture` son independientes.

### Futuro entre grupos
- Parámetro controlado por Servidor y consumido por Dron: futuro contrato
  `Dron startup client -> servicio de configuración del Servidor -> Dron`.
- Intrínseco del Dron requerido por Servidor, como `body_T_camera`: futuro contrato
  Dron→Servidor mediante TF/calibración.
No implementar esos transportes en Fase 2 si no existe necesidad funcional actual.
<!-- ACUERDOS_CIERRE_F2_2026_08_24_END -->

## 1. Identidad de submapa

La unidad correcta del sistema es:

```text
submapa = (drone_id, map_epoch)
```

Todo KeyFrame, MapPoint, anchor, loop, fused track, score, corrección, snapshot o tarea de optimización debe conservar:

```text
drone_id
map_epoch
local_id
```

No identificar un mapa local solo con `drone_id`.

## 2. Ground truth

El ground truth solo puede usarse para:

- fiducial simulado;
- debug;
- métricas externas;
- evaluación visual o numérica que no alimente el algoritmo.

No puede usarse para:

- construir mapa sparse;
- decidir loops;
- fusionar landmarks;
- calcular score;
- publicar pose final;
- construir nube densa;
- colocar puntos en el mapa final.

Uso debug permitido desde `3L`:

- mantener una base separada de poses GT por KeyFrame en simulacion, por
  ejemplo `DebugGroundTruthKeyFrameStore`;
- asociar GT por timestamp y loggear error real por KF/ventana antes y despues
  de una optimizacion;
- usar esos logs como evidencia externa para diagnosticar RViz2, daño colateral
  o errores de frames.

Uso debug prohibido:

- usar GT por KF para construir el grafo;
- seleccionar vertices;
- ajustar pesos;
- decidir loops;
- aceptar/rechazar optimizaciones en runtime normal;
- corregir poses;
- publicar mapas.

Si una prueba automatica usa GT por KF como metrica, debe quedar activado de
forma explicita como modo debug/test. El algoritmo debe poder ejecutarse sin
esa informacion.

## 3. Separación raw/global

`RawMapDatabase` guarda datos crudos de ORB-SLAM3:

- KeyFrames;
- MapPoints;
- BoW y FeatureVector;
- descriptores;
- observaciones;
- covisibilidad;
- parent/children;
- loop edges locales;
- poses locales originales;
- deltas;
- full snapshots;
- journal/replay.

`RawMapDatabase` no se modifica por optimización y no borra MapPoints por fusión.

`GlobalPoseStore` guarda estado global:

- anchors por fiducial o loop;
- `world_T_local`;
- poses optimizadas;
- poses propagadas;
- KFs rebasados;
- correcciones heredables para KFs futuros;
- protección de poses optimizadas frente a full snapshots posteriores.

Las poses locales de ORB-SLAM3 nunca deben sobrescribirse para hacerlas globales.

## 4. Fiduciales

Los fiduciales son observaciones absolutas, no loops.

`FiducialAnchorManager` debe:

- recibir del servidor una observación de KF en fiducial;
- anclar el submapa en primera visita;
- medir error en visitas posteriores;
- crear una tarea de optimización si el error supera umbral;
- distinguir KFs realmente fiduciales/hard-fixed de KFs normales de un submapa anclado.

El GT solo simula la observación fiducial mientras no exista detector real.

## 5. Loops

`LoopDetector` solo genera candidatos BoW. BoW no confirma loops.

`SubcloudLoopVerifier` confirma o rechaza con:

- `query_subcloud` desde el KF nuevo;
- `candidate_subcloud` como zona global alrededor del seed candidato, no solo sus puntos;
- matching ORB;
- reducción espacial robusta;
- matching refinado;
- RANSAC;
- error geométrico.

Las decisiones geométricas válidas son:

```text
REJECT
HOLD
FUSION_CANDIDATE
LOOP_OPTIMIZATION_CANDIDATE
```

No reintroducir `LOOP-VIEWMAP-ERROR` como criterio principal.

## 6. Fusión y score

`FusionManager` / `FusedLandmarkManager` fusionan landmarks sin borrar MapPoints raw.

La nube global publicada debe usar fused tracks cuando existan, no una unión bruta de todos los `MapPoints`.

La posición fused debe ser una media ponderada o estimación robusta. El descriptor fused debe ser representativo/medoid; no usar media float normal de ORB.

`LandmarkScoreManager` centraliza todo el scoring. Nadie debe hacer `score += X` fuera de esa clase. Las demás clases emiten eventos semánticos:

- score raw inicial;
- fusión confirmada;
- reobservación;
- expected-but-unmatched;
- rollback de eventos de score;
- cambios de publicación.

## 7. Optimización

`PoseGraphBuilder` construye grafos temporales bajo demanda. No mantener un grafo global permanente.

`OptimizationManager`:

- ejecuta dry-run;
- aplica solo si el resultado es útil;
- escribe poses en `GlobalPoseStore`;
- no toca `RawMapDatabase`;
- sirve para fiduciales y loops;
- debe permitir validación post-apply y rollback.

Después de aplicar, se recalcula el error real. Si la optimización empeora o rompe invariantes, se hace rollback de poses, correcciones, scores y fused tracks afectados.

No aumentar simplemente `max_variables` para arreglar coste. No mover KFs realmente fiduciales/hard-fixed.

## 8. Wrapper, mensajes y ORB_SLAM3

`orbslam3_ros2` y `orbslam3_msgs` son estables.

No rediseñar:

- `OrbMap`;
- `OrbMapPoint`;
- `OrbKeyFrame`;
- `StereoSlamNode`;
- publicación de deltas;
- servicio `GetOrbMap`.

No añadir score global al wrapper.

No modificar `ORB_SLAM3` interno salvo permiso explícito o bug confirmado imposible de corregir en otro paquete.

## 9. Legacy

Las subfases `12R-*`, `13`, `14` y `15` son legacy. Se pueden consultar para aprender de errores previos, pero no son la planificación activa.

No borrar archivos legacy hasta que exista ruta nueva compilada, validada y documentada.

## 10. Build, simulación y logs

Después de modificar código:

1. compilar paquetes seleccionados con `build_selected_packages.sh`;
2. si falla, generar `colcon_build.reduced.log`;
3. corregir el primer error real;
4. ejecutar las simulaciones definidas por la subfase;
5. generar `prueba_X.reduced.log`;
6. analizar solo el reducido; si faltan marcadores, regenerarlo con patrones
   específicos o crear sublogs, sin abrir el completo;
7. actualizar documentación e historial.

No marcar `realizado` sin evidencia de build, simulación y logs.

## 11. Comentarios en código

Toda modificación de código debe comentar lógica no trivial, especialmente:

- fiduciales;
- loops;
- optimización;
- selección de variables;
- propagación;
- fused landmarks;
- scoring;
- rollback;
- callbacks ROS 2.

Los comentarios deben explicar el porqué, no repetir el código.
