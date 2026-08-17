# Subfase 6L — Estado de tracking y soporte visual actual de ORB-SLAM3 para navegación

## Estado

```text
sin hacer
```

## Dependencia

Fase 5 y 6G. Requiere localizar el wrapper real no incluido completamente en el snapshot.

## Objetivo técnico

Proporcionar al `control_trayectorias` información ligera y actual sobre la calidad de tracking y lo que ORB-SLAM3 está viendo para evitar movimientos/orientaciones que puedan hacer perder la localización.

## Comportamiento esperado

El planificador necesita separar dos preguntas:

```text
Depth/6K          -> ¿es físicamente seguro?
ORB tracking/6L   -> ¿mantendré información visual suficiente?
```

La información mínima debe permitir detectar degradación antes de una pérdida completa. Puede incluir estado de tracking, número de observaciones/MapPoints seguidos en el frame, distribución aproximada por imagen, edad/calidad y otros indicadores realmente disponibles sin acoplarse en exceso al núcleo.

`OrbMap`/deltas globales no son un sustituto de “qué puntos estoy viendo ahora”. Si el wrapper ya publica un estado equivalente tras Fase 5, se reutiliza. Si no, se amplía el wrapper. Solo si el wrapper no puede obtenerlo con la API disponible se detiene la ejecución para acordar una modificación mínima de ORB_SLAM3.

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
```

Antes de modificar código, Codex debe leer también el MD vigente de cada paquete afectado en `codex/contexto/paquetes/` y el contrato final de Fase 5 que exista en el workspace real. El snapshot usado para preparar esta Fase 6 no contiene la Fase 5 recién ejecutada por el usuario, por lo que no se deben inventar nombres nuevos de topics, frames o mensajes si Fase 5 ya proporciona un contrato equivalente.



## Diagnóstico de partida

El snapshot del wrapper no está disponible completo. La documentación conocida publica pose local/deltas/mapa, pero esos productos no garantizan información de soporte visual del frame actual. El planificador no debe inferir tracking únicamente por densidad del mapa global.

## Invariantes y decisiones cerradas

- No modificar ORB_SLAM3 si el wrapper puede obtener la información necesaria.
- El estado se refiere al frame/observación actual y lleva timestamp/map_epoch coherentes.
- Pocos MapPoints significan soporte visual pobre; **no** significan “no hay obstáculo”.
- La navegación debe reaccionar a `LOST`/estado equivalente y a degradación configurable.
- No enviar descriptores/imágenes completas si basta un resumen ligero.

## Archivos permitidos a modificar

```text
src/dron/orbslam3_ros2/  # localizar ruta real del wrapper
src/dron/orbslam3_msgs/msg/  # si hace falta contrato compartido/local
src/servidor/orbslam3_msgs/  # réplica canónica si se modifica paquete compartido
src/dron/dron_individual/src/control_tray/
src/dron/dron_individual/config/
codex/contexto/paquetes/orbslam3_ros2/
codex/contexto/paquetes/dron_individual/
codex/contexto/paquetes/orbslam3_msgs/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
ORB_SLAM3/  # salvo bloqueo demostrado y nuevo acuerdo explícito
src/servidor/orbslam3_multi/  # no usar el backend global como sensor de frame actual
src/simulacion/simulacion_dron/
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
`StereoSlamNode`/nodo wrapper real
llamada `TrackStereo` y pose/tracking state disponible tras Fase 5
API para current tracked MapPoints/keypoints/estado; confirmar sin inventar acceso
mensajes actuales de pose/tracking de Fase 5
consumidor `control_trayectorias` 6G
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Inventariar qué métricas de tracking expone ya el wrapper y documentar su semántica/timestamps.
2. Definir el conjunto mínimo de indicadores: estado de tracking y soporte visual actual; incluir distribución espacial si puede obtenerse de forma razonable y útil para 6M.
3. Publicar o entregar internamente estos indicadores al `control_trayectorias` con `drone_id`, `map_epoch` y timestamp coherentes cuando corresponda.
4. Evitar copiar mapas/descriptores masivos para esta decisión; preferir conteos, scores agregados o IDs/posiciones estrictamente necesarios.
5. Parametrizar umbrales de advertencia/degradación en el grupo Dron; no hardcodear valores definitivos sin pruebas.
6. Añadir logs por transición `SLAM-VISUAL-STATE good/degraded/lost` y métricas agregadas sin emitir una línea por feature.
7. Si la API pública del wrapper no permite acceder a soporte actual, documentar el bloqueo y pedir autorización antes de tocar ORB_SLAM3; no inventar una estimación falsa.

## Cambios prohibidos

- No concluir que una dirección está libre porque bajen los MapPoints.
- No usar GT para decidir si tracking es correcto.
- No publicar imágenes completas a alta frecuencia solo para contar features en otro nodo si puede hacerse localmente de forma más eficiente.
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
./codex/herramientas/build_selected_packages.sh orbslam3_ros2 dron_individual orbslam3_msgs
```

Si la distribución física creada por Fase 2 requiere el helper por grupos, usar la herramienta vigente equivalente y registrar el comando exacto en historial. Añadir dependencias reales solo si el build demuestra que son necesarias.

## Pruebas Gazebo requeridas

Preparación común:

- Si la prueba necesita una secuencia reproducible, usar/crear un YAML de escenario dentro de `src/simulacion/simulacion_dron/config/scenarios/fase_6/` (ruta final a confirmar contra Fase 2), no en el grupo Dron/Servidor.
- No precalcular en el scenario runner la autonomía que precisamente se está validando; el runner solo prepara condiciones, inyecta eventos o espera resultados.
- Comando base para cualquier prueba Gazebo de esta subfase:

```bash
./codex/herramientas/run_simulation.sh   --prueba fase_6_6L   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Transición de soporte visual

En una secuencia controlada, orientar progresivamente la cámara desde una zona con textura/MapPoints hacia una zona pobre y volver. Deben observarse transiciones coherentes `good -> degraded` sin usar GT.

### Prueba 2 — Tracking perdido/recuperado

Forzar, si es posible de forma segura, un caso donde ORB-SLAM3 reporte pérdida/relocalización. El consumidor debe recibir el estado correcto con epoch/timestamp y no confundirlo con “obstáculo ausente”.

## Patrones de reducción de logs

```text
SLAM-VISUAL-STATE|tracking|tracked|MapPoint|support|degraded|lost|relocal|map_epoch|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. El planificador recibe un estado actual y timestampado de tracking sin GT.
2. Existe al menos una métrica de soporte visual útil además de la pose.
3. La degradación se distingue de espacio libre/obstáculo.
4. No se modifica ORB_SLAM3 salvo necesidad previamente acordada.
5. El tráfico/coste de la interfaz es ligero y acotado.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si solo se expone estado binario tracking/lost y falta una señal preventiva necesaria para 6M.
- `BLOQUEADA` si la API real no permite obtener el soporte requerido sin cambiar ORB_SLAM3 y todavía no existe autorización.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Acoplamiento fuerte a internals de ORB_SLAM3.
- Métrica de conteo engañosa si todos los puntos están concentrados en una esquina de imagen.
- Timestamps desalineados con la pose usada por el planificador.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6L.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6L_RESUMEN.md
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
