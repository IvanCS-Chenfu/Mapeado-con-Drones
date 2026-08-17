# Pipeline Fase 8 — Nube densa global multi-dron

Resumen de entrada:

```text
codex/pipeline/fase_8_nube_densa/pipeline_fase_8_RESUMEN.md
```

## Estado

```text
FASE: SIN HACER
Preparación documental: CERRADA
Acuerdo cerrado: sí
Autorización funcional de ejecución: PENDIENTE
Historial: vacío; no existen ejecuciones reales en este ZIP
Dudas abiertas de arquitectura: ninguna en el contrato actual
```

Este pipeline es un **contrato documental**. No contiene resultados reales de build, simulación ni historial. Las carpetas `historial/` y `historial/por_subfase/` se entregan vacías intencionadamente.

## Objetivo general

Construir en el servidor una reconstrucción densa global multi-dron a partir de cámaras estéreo, manteniéndola geométricamente coherente con el mapa sparse y las poses globales optimizadas sin usar Ground Truth como entrada funcional. La Fase 8 no crea un SLAM denso independiente: las subnubes densas de KeyFrames se expresan en el frame de su propio KF y usan las poses globales aceptadas por Fase 3/5 para colocarse en `world`.

Sparse y dense siguen siendo **dos representaciones distintas**. Se ayudan mutuamente:

- sparse → dense: KFs, poses, loops, fiduciales y optimizaciones colocan/recolocan subnubes;
- dense → sparse: registros geométricos fiables pueden aportar constraints al mismo grafo de poses y superficies densas pueden refinar MapPoints publicados mediante una DB derivada;
- nunca se modifica `RawMapDatabase` ni se devuelven MPs corregidos a ORB-SLAM3.

## Ownership y paquetes acordados

Fase 8 crea dos paquetes de servidor:

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
```

Responsabilidades:

```text
Dron
  cámaras + ORB-SLAM3/wrapper
  -> envía imágenes/identidad/estado
  -> mínimo procesamiento

Servidor
  dense_map_server
    -> ROS 2, subscriptions, publicaciones, coordinación, workers
    -> delega algoritmos

  dense_map_multi
    -> disparity/depth
    -> DenseKeyFrameDatabase
    -> quality/filtros
    -> occupancy
    -> DenseFusionMap
    -> registro dense-dense
    -> análisis de cobertura
    -> recapturas/patches a nivel algorítmico

  orbslam3_multi
    -> RawMapDatabase
    -> GlobalPoseStore
    -> grafo/optimización de poses
    -> GlobalMapBuilder sparse
```

`dense_map_multi` **no** posee un optimizador paralelo de poses. Las medidas dense de 8J se convierten en constraints en 8K y se resuelven con el optimizador vigente de Fase 3. Solo si la implementación real demuestra un acoplamiento que obliga a una refactorización material se volverá a consultar al usuario antes de extraer una librería compartida de pose graph.

## Entrada de imágenes: dos caminos

### A. Mapping sparse normal

```text
camera L/R
  -> StereoSlamNode::GrabStereo
  -> TrackStereo
  -> ORB crea KF_i
  -> wrapper identifica el KF exacto
  -> envía L/R exactas + (drone_id,map_epoch,kf_id)
  -> servidor calcula disparity/depth
  -> DenseSubcloud_i local a KF_i
```

No se asocia por timestamp aproximado ni a un KF cercano. La nube nace directamente del par que produjo el KF.

### B. Tarea dense / comprobación local

```text
topics camera L/R normales
  -> servidor
  -> depth
  -> (a) solo occupancy/seguridad
  -> o (b) DenseHQCapture estacionaria
```

La captura HQ no necesita que ORB cree un KF.

## Fuente de verdad y almacenamiento

```text
DenseKeyFrameDatabase
  clave: (drone_id, map_epoch, kf_id)
  contiene: KF + DenseSubcloud local + metadata/calidad
  NO contiene: imágenes L/R persistentes
```

Las imágenes se consumen y se descartan. Si una nube queda mal, el sistema genera una recaptura/densificación; no se conserva un archivo de imágenes para reprocesar indefinidamente.

La nube global **no** es una concatenación persistente adicional de todos los puntos raw. Se construyen productos derivados que añaden información.

## Dos escalas voxel complementarias

### OccupancyGrid3D gruesa

Cubre el ROI a resolución relativamente grande y almacena información espacial:

```text
UNKNOWN | FREE | OCCUPIED
confidence
evidencia/observaciones
```

Un depth puede actualizar esta grid aunque no cree DenseKF. Los rayos válidos aportan FREE en voxels atravesados y OCCUPIED en la superficie. La información se considera estática acumulativa: no caduca por tiempo, pero nueva evidencia puede reducir confianza de una creencia antigua y cambiar OCCUPIED↔FREE.

### DenseFusionMap fino

Representa superficies/geometría con voxels/bloques sparse asignados solo donde hay observaciones. Puede comenzar como voxel hash ponderado y evolucionar a una integración TSDF-like si las pruebas lo justifican. No se reserva una grid fina de todo el ROI.

## Optimización y reintegración

La posición world de una subnube de KF siempre se deriva de:

```text
p_world = T_world_KF * p_KF
```

En 8E se demuestra que si `T_world_KF` cambia, la subnube visual se mueve. En 8I se resuelve el problema más difícil: retirar/inutilizar la contribución vieja ya integrada en voxels y reintegrar solo KFs/bloques afectados, con revisiones y protección frente a resultados stale.

## Fusión dense y ayuda al sparse

8J registra subnubes y obtiene una medida geométrica relativa entre KFs. No se mueve una nube respecto a su KF para “hacerla encajar”. Si la geometría indica que la relación entre dos vistas debe ser distinta, se crea un candidato:

```text
DenseRelativeConstraintCandidate(KF_A, KF_B, T_A_B, information, quality)
```

8K valida esa medida, la incorpora al grafo de Fase 3 y reutiliza el mismo dry-run/apply/rollback/hard fiducials. `GlobalPoseStore` sigue siendo autoridad.

En paralelo, 8L crea una `SparseDenseCorrectionDatabase` para refinar la posición **publicada** de MPs contra superficies dense sin mutar raw ni ORB local. `GlobalMapBuilder` consulta esta DB al construir la nube sparse derivada.

8M revalida relaciones inter-KF cuando cambian poses. Una relación local MP/surface dentro del mismo KF suele sobrevivir a un rigid transform; una fusión entre KFs distintos puede quedar obsoleta y debe recalcularse/invalidarse.

## Estrategia de captura: híbrida con fallback

La estrategia preferida es una sola misión sparse que genera DenseKF oportunistas mientras los drones se mueven. 8E/8G miden si esas nubes son suficientemente útiles. Si el movimiento produce geometría muy mala, no se inventan MapPoints ni se fuerza un acoplamiento: la arquitectura sigue funcionando con sparse primero y tareas estacionarias dense posteriores.

Después de la primera misión, 8N analiza ROI/calidad y genera **necesidades geométricas**; el TaskManager de Fase 6 asigna drones. No se repite todo el entorno por defecto.

Casos correctivos:

- `BAD_DENSE_KF`: volver a la pose global vigente del KF, capturar parado y sustituir la subnube tras commit atómico;
- zona poco densa/sin KF: capturar parado desde topics, registrar contra mapa y repartir por patches rígidos entre KFs apropiados.

La subnube antigua de un DenseKF se conserva solo mientras se valida la candidata. Tras commit correcto, se elimina si no tiene uso real.

## Planificación y occupancy

La trayectoria global puede consultar sparse + DenseFusionMap + OccupancyGrid3D. La seguridad local sigue usando depth. Si un dron quiere moverse lateralmente hacia una zona no observada, debe usar el comportamiento perceptivo de Fase 6 para orientar/mirar y comprobar profundidad antes de asumir espacio libre. Esos rayos actualizan occupancy aunque no generen nube.

## Multi-dron y GUI desde el principio

No existe una subfase tardía para “hacerlo multi-dron”: la identidad completa está presente desde 8D y 8J registra/fusiona igual intra o inter-dron. Tampoco existe una integración GUI tardía: 8E ya alimenta la capa dense real creada/preparada en Fase 7 y las subfases posteriores añaden diagnóstico.

## Secuencia acordada

| ID | Subfase | Salida principal |
|---|---|---|
| 8A | Calibración, frames y backend | `dense_map_multi`/`dense_map_server` + escala estéreo válida |
| 8B | KF ↔ L/R exactas | par exacto por `(drone_id,map_epoch,kf_id)` |
| 8C | Disparity/depth/subcloud | `DenseSubcloud` local al KF |
| 8D | DenseKeyFrameDatabase | DB canónica sin imágenes |
| 8E | Baseline end-to-end | todos los KFs dense, GUI y recolocación por pose |
| 8F | Selección DenseKF | menos redundancia sin perder cobertura |
| 8G | Calidad/filtros | disparity/depth/nubes evaluadas y filtradas |
| 8H | Voxels globales | occupancy gruesa + fusión fina sparse/TSDF-like |
| 8I | Reintegración | productos fusionados coherentes tras optimización |
| 8J | Registro dense-dense | medidas relativas fiables entre KFs |
| 8K | Grafo sparse+dense | constraints dense usando optimizador Fase 3 |
| 8L | Dense→MapPoints | DB de correcciones derivadas consumida por GlobalMapBuilder |
| 8M | Revalidación | relaciones inter-KF coherentes tras revisiones |
| 8N | Coverage/quality | necesidades de densificación/recaptura para TaskManager |
| 8O | Captura HQ | varios pares estacionarios desde topics |
| 8P | Integración HQ | recaptura DenseKF + patches para zonas sin KF |
| 8Q | Planificación | sparse+dense+occupancy + depth local |
| 8R | Rendimiento | presupuestos y decisión de compresión con evidencia |
| 8S | Exportación | mapa final + metadata/revisión |
| 8T | Integración final | prueba completa multi-dron y cierre |

## Reglas de pruebas

- GT solo como métrica externa.
- Los logs completos nunca se leen directamente; usar reductores/sublogs.
- Cada subfase debe compilar los paquetes realmente tocados y realizar tests deterministas antes de Gazebo cuando sea posible.
- GUI es evidencia visual importante, pero no sustituye métricas geométricas.
- Si una anomalía procede de una fase anterior, volver a esa fase, corregir y revalidar; no maquillar en Fase 8.
- `CONSEGUIDA` exige build, pruebas, logs y documentación coherentes.

## Criterio de cierre de Fase 8

La fase termina cuando varios drones pueden producir una reconstrucción densa global con escala, continuidad y ocupación coherentes en `world`; las subnubes/voxels se actualizan tras correcciones de poses; dense puede ayudar al sparse sin tocar raw; las zonas deficientes generan tareas correctivas estacionarias; planificación usa occupancy/dense manteniendo depth local; el sistema cumple presupuestos de recursos y exporta un resultado reproducible sin GT funcional.
