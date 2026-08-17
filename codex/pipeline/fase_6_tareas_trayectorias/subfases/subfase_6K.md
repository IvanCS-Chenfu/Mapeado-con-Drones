# Subfase 6K — Percepción estéreo/depth local para seguridad y espacio navegable

## Estado

```text
sin hacer
```

## Dependencia

6G y calibración/cámaras funcionales de Fases anteriores. Puede prepararse en paralelo con 6H–6J.

## Objetivo técnico

Crear en el dron una percepción local y temporal de obstáculos físicos a partir de estéreo/depth que permita decidir si el siguiente tramo es seguro, sin construir ni fusionar todavía la nube densa global de Fase 8.

## Relacion con el buffer sparse de 3P

La subfase 3P define un z-buffer sparse temporal para decidir si un outlier de
loop contradice una observacion esperada. Ese buffer:

- proyecta solo las subnubes acotadas de dos KFs;
- clasifica visible, ocluido, fuera, foreground e incierto;
- tiene un presupuesto corto y puede omitir evidencia negativa;
- no calcula disparity, no persiste depth y no cubre superficies sin features.

Por tanto no es una entrada valida de seguridad. Al preparar 6K se debe releer
el contrato y codigo real de 3P para decidir si conviene reutilizar piezas
puramente geometricas, tipos de clasificacion o telemetria. La autoridad de
obstaculos seguira siendo depth/estereo real, reciente y local al dron.

Si 6K obtiene una interfaz de depth/confianza barata y temporalmente alineada,
podra proponerse en una preparacion posterior enriquecer la visibilidad de 3P.
Ese cambio no se hara desde 6K por defecto: debe volver a Fase 3, medir coste y
frescura y preservar que la fusion no dependa de una fuente de seguridad.

## Comportamiento esperado

La seguridad física no puede depender únicamente de MapPoints ORB. Una pared lisa puede ser un obstáculo real y generar muy pocos landmarks. Por ello el dron debe disponer de una fuente separada:

```text
stereo/depth local
   -> profundidad válida
   -> espacio libre / ocupado / desconocido local
   -> distancia a obstáculos
   -> entrada para LocalPlanner
```

El snapshot contiene experimentos en `dron_individual/src/vision/` con StereoSGBM/WLS, depth y PointCloud2. Se pueden reutilizar ideas, parámetros/calibración o código válido tras auditarlo, pero no promover directamente scripts experimentales/TSDF como arquitectura final.

La representación puede ser nube/voxeles/local occupancy equivalente, siempre acotada en alcance y memoria y asociada a timestamps/frames correctos.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6_RESUMEN.md
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3P_especificacion.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3P_implementacion.md
```

Antes de modificar código, Codex debe leer también el MD vigente de cada paquete afectado en `codex/contexto/paquetes/` y el contrato final de Fase 5 que exista en el workspace real. El snapshot usado para preparar esta Fase 6 no contiene la Fase 5 recién ejecutada por el usuario, por lo que no se deben inventar nombres nuevos de topics, frames o mensajes si Fase 5 ya proporciona un contrato equivalente.



## Diagnóstico de partida

Existen múltiples scripts experimentales de visión, profundidad, nubes y TSDF, pero no un pipeline ROS 2 limpio y documentado que el futuro `control_trayectorias` pueda consumir como autoridad local de obstáculos.

## Invariantes y decisiones cerradas

- Depth/estéreo responde “¿hay obstáculo/espacio navegable?”, no “¿tengo buen tracking?”.
- La representación es local/temporal; no constituye el producto de nube densa global de Fase 8.
- El buffer sparse de 3P no sustituye esta representacion ni permite declarar
  espacio libre para navegacion.
- No filtrar obstáculos por tener o no MapPoints ORB.
- Usar calibración y frames reales; no derivar profundidad de GT.
- Espacio desconocido no debe tratarse automáticamente como libre.
- El pipeline debe ser utilizable tanto en interior como exterior.

## Archivos permitidos a modificar

```text
src/dron/dron_individual/src/vision/
src/dron/dron_individual/include/  # módulo local si se crea
src/dron/dron_individual/config/vision.yaml
src/dron/dron_individual/launch/
src/dron/dron_individual/CMakeLists.txt
src/dron/dron_individual/package.xml
src/dron/orbslam3_msgs/  # solo si se necesita una interfaz compartida, normalmente no
codex/contexto/paquetes/dron_individual/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
src/servidor/orbslam3_multi/
src/servidor/orbslam3_server/  # no enviar toda la percepción al servidor como solución base
ORB_SLAM3/
cualquier implementación global TSDF/Open3D como salida funcional de Fase 6
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
topics estéreo/depth reales posteriores a Fase 2/Fase 5
calibración de cámara usada por ORB-SLAM3
`dron_individual/src/vision/vision.py` y experimentos `test_profundidad.py`/nubes solo como referencia
frame `camera`/`camera_optical_frame` y extrínseca a `base_link`
QoS/timestamps de cámaras en simulación
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Inventariar los experimentos y documentar qué piezas son reutilizables y cuáles se descartan; no copiar ciegamente la lógica de TSDF/dense.
2. Crear un nodo/componente ROS 2 estable que sincronice las entradas estéreo o consuma depth si ya existe una fuente equivalente tras Fase 5.
3. Validar disparidad/profundidad y rechazar NaN, infinito, profundidad negativa/fuera de rango y regiones con baja confianza.
4. Generar una representación local acotada de obstáculos/espacio desconocido en frame documentado, actualizada con timestamps recientes.
5. Publicar una interfaz de consulta/estado para el LocalPlanner: distancia mínima, ocupación local o estructura equivalente; evitar transportar nubes enormes a alta frecuencia si no es necesario.
6. Parametrizar alcance, resolución/voxel, margen y frecuencia en `vision.yaml` o YAML propietario equivalente.
7. Añadir logs acotados `LOCAL-DEPTH`, `LOCAL-OBSTACLE`, `LOCAL-DEPTH-INVALID` y métricas de latencia/frecuencia.
8. Comparar de forma acotada sus clases de validez/oclusion y metricas con las
   de 3P; reutilizar solo abstracciones que no añadan dependencia runtime entre
   planificacion local y fusion sparse del servidor.

## Cambios prohibidos

- No usar solo MapPoints con score alto como mapa de paredes.
- No convertir el z-buffer sparse de 3P en mapa de profundidad o fuente de
  collision checking.
- No guardar/fusionar permanentemente todos los frames de profundidad.
- No convertir esta subfase en la reconstrucción densa de Fase 8.
- No tratar píxeles sin profundidad fiable como espacio libre.
- No usar Ground Truth como entrada funcional para pose, asignación, navegación, obstáculos, autorización de trayectorias, cobertura o criterio de finalización.
- No modificar `ORB_SLAM3` como primera opción; ampliar el wrapper o reutilizar interfaces existentes siempre que sea suficiente.
- No implementar la nube densa global, TSDF/Open3D global ni la reconstrucción final de Fase 8.
- No convertir el servidor en planificador de paredes u obstáculos físicos: esa responsabilidad pertenece al dron.
- No borrar ni reescribir datos raw de ORB-SLAM3 para adaptar la navegación.
- No introducir un modo separado de navegación interior y otro exterior.
- No rellenar historiales con resultados ficticios: se crean únicamente cuando exista una ejecución real.

## Paquetes a compilar

Comando base esperado:

```bash
./codex/herramientas/build_selected_packages.sh dron_individual
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6K   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Obstáculo frontal conocido

En Gazebo, colocar/usar una pared u objeto estático delante del dron y comprobar que la percepción local marca ocupación/distancia coherente sin consultar GT. El dron no necesita todavía navegar alrededor.

### Prueba 2 — Pared con poca textura

Usar una superficie con pocas features ORB si el escenario lo permite. La fuente depth debe seguir detectándola aunque el soporte de MapPoints sea bajo.

### Prueba 3 — Datos inválidos/alcance

Comprobar que zonas fuera de rango o disparity inválida no se convierten en espacio libre silenciosamente y que la representación permanece acotada.

## Patrones de reducción de logs

```text
LOCAL-DEPTH|LOCAL-OBSTACLE|depth|disparity|invalid|latency|PointCloud|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Existe una fuente local de obstáculos sin GT consumible por el planificador.
2. Una pared física se detecta aunque tenga pocos MapPoints.
3. Datos inválidos se manejan como inválidos/desconocidos, no libres.
4. Memoria/frecuencia se mantienen acotadas en una prueba corta.
5. No se crea ninguna reconstrucción densa global persistente.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si depth funciona pero no existe una interfaz limpia para el planificador.
- `PARCIAL` si detecta obstáculos texturados pero falla sistemáticamente en superficies sin features por depender indirectamente de ORB.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Calibración/baseline incorrectos que produzcan escala falsa.
- Latencia de depth que haga planificar con obstáculos antiguos.
- Consumo excesivo de CPU/memoria por una nube local demasiado densa.
- Superficies lisas/reflectantes con depth poco fiable.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6K.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6K_RESUMEN.md
codex/pipeline/fase_6_tareas_trayectorias/pipeline_fase_6_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md              # si cambia el estado real
codex/contexto/paquetes/<paquete_afectado>/
```

Los historiales anteriores **no existen en este ZIP a propósito**. Deben crearse solo tras una ejecución real. La documentación de paquete debe reflejar el estado actual del código, no limitarse a añadir una nota histórica.

## Dudas funcionales de contrato

```text
ninguna
```

Cualquier duda nueva que cambie el comportamiento acordado suspende la autorización funcional y debe discutirse antes de continuar, conforme a `AGENTS.md`.
