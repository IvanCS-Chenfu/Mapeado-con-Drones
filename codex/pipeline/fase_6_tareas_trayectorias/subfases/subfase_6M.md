# Subfase 6M — Planificación de yaw y selección de vistas perceptivamente seguras

## Estado

```text
sin hacer
```

## Dependencia

6K y 6L.

## Objetivo técnico

Hacer que el dron planifique explícitamente hacia dónde mirar durante el movimiento, maximizando información útil y continuidad de tracking sin codificar modos distintos para interior/exterior.

## Comportamiento esperado

El `yaw` no es un subproducto de la dirección de velocidad. Para cada posición/tramo candidato se deben evaluar orientaciones posibles con una jerarquía clara:

```text
1. seguridad física (6K)
2. conservar tracking / soporte visual (6L)
3. observar la superficie/región de cobertura
4. adquirir información nueva y mantener solape
5. eficiencia del movimiento
```

En exterior, las superficies útiles harán que el scorer tienda a mirar hacia el edificio. En interior, tenderá a mirar hacia paredes/estructuras con información. No existe `if interior`/`if exterior`.

Al desplazarse lateralmente cerca de una superficie, el dron puede realizar pequeños giros de inspección hacia el siguiente tramo. Si el soporte visual cae antes de poder mirar más, se detiene el giro y vuelve a una orientación segura. Eso **no** demuestra que el espacio esté vacío; la seguridad la determina depth.

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

Los generadores actuales permiten yaw, pero no existe una política autónoma que seleccione orientación por valor visual. Seguir simplemente el heading de la trayectoria puede hacer que la cámara mire al vacío y pierda ORB-SLAM3.

## Invariantes y decisiones cerradas

- Mismo algoritmo para interiores y exteriores.
- Yaw candidato que compromete tracking por debajo del umbral no puede ganar por ofrecer más novedad.
- Pequeñas inspecciones hacia delante son reversibles y limitadas por soporte visual.
- La ausencia de MapPoints no se usa como detector de espacio libre.
- La orientación se representa en los waypoints de 6H/6I.

## Archivos permitidos a modificar

```text
src/dron/dron_individual/src/control_tray/  # ViewPlanner/scorer propuesto
src/dron/dron_individual/include/
src/dron/dron_individual/config/
src/dron/dron_individual/test/
codex/contexto/paquetes/dron_individual/
```

Las rutas marcadas como propuestas deben confirmarse contra el árbol real antes de crearlas. Si Fase 5 ya contiene un componente equivalente, se amplía/reutiliza en lugar de duplicarlo.

## Archivos prohibidos

```text
ORB_SLAM3/
src/servidor/orbslam3_server/  # el servidor no decide yaw de paredes
src/servidor/orbslam3_multi/
```

Además, no tocar legacy o paquetes ajenos a la subfase como limpieza colateral.

## Funciones, clases, nodos o interfaces que hay que localizar

```text
TaskExecutor/control_trayectorias 6G
salida local de obstáculos 6K
estado visual 6L
waypoint/yaw 6H/6I
cámara/extrínseca y FOV documentados
```

Los nombres nuevos que aparezcan en este documento son nombres de contrato/propuesta. Si el workspace real ya posee una abstracción equivalente, reutilizarla y documentar la correspondencia antes de implementar.

## Cambios requeridos

1. Crear un `ViewPlanner`/scorer o módulo equivalente separado del path planner para evaluar yaw candidatos.
2. Definir métricas disponibles: soporte visual actual/predicho, distribución en imagen cuando exista, observación de superficie de interés, novedad/solape y coste de giro.
3. Aplicar restricciones duras de seguridad/tracking antes de optimizar novedad o eficiencia.
4. Implementar giro de inspección incremental con `max_lookahead_yaw_step`/equivalente parametrizable y aborto si `SLAM-VISUAL-STATE` cae a degradado bajo el criterio acordado.
5. Integrar el yaw escogido en los waypoints propuestos, evitando saltos angulares no ejecutables.
6. Crear una política de fallback: si ninguna nueva orientación es segura perceptivamente, mantener/recuperar la última vista estable y permitir que 6O replantee el movimiento.
7. Añadir logs `VIEW-CANDIDATE`, `VIEW-SELECTED`, `VIEW-ABORT-TRACKING` a baja frecuencia/eventos.

## Cambios prohibidos

- No fijar yaw simplemente al centro geométrico del ROI.
- No clasificar previamente el escenario como interior/exterior.
- No seguir girando para “comprobar si hay algo” cuando cae el tracking.
- No hacer que el valor de novedad compense una colisión o pérdida probable de tracking.
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
./codex/herramientas/run_simulation.sh   --prueba fase_6_6M   --launch "ros2 launch simulacion_dron multi_dron.launch.py"   --post-scenario-wait-sec 20
```

Si la prueba indicada es determinista/unitaria y no necesita Gazebo, no arrancarlo artificialmente; ejecutar el test del paquete y registrar el comando exacto en historial.

### Prueba 1 — Seguimiento lateral de pared

Hacer que el dron se desplace lateralmente junto a una superficie con textura. Debe mantener la cámara mayoritariamente orientada hacia la superficie, no hacia la velocidad por defecto.

### Prueba 2 — Inspección hacia zona pobre

Durante el desplazamiento, permitir un giro incremental hacia una zona con poco soporte visual. El giro debe detenerse/revertirse antes de comprometer tracking; depth sigue siendo la autoridad de obstáculo.

### Prueba 3 — Interior/exterior sin flag

Ejecutar dos escenarios representativos o dos zonas del mismo mundo y comprobar que no se necesita parámetro de modo interior/exterior para orientar la cámara de forma útil.

## Patrones de reducción de logs

```text
VIEW-CANDIDATE|VIEW-SELECTED|VIEW-ABORT|SLAM-VISUAL-STATE|yaw|tracking|support|ERROR|FATAL|Segmentation fault|Killed
```

El log completo se conserva como artefacto y se reduce antes de leerlo, según `AGENTS.md`. Si el reducido no contiene evidencia suficiente, regenerar un reducido con patrones más precisos; no usar el log completo como contexto directo.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Yaw se selecciona explícitamente mediante información perceptiva.
2. El mismo código funciona sin flag interior/exterior.
3. Un giro exploratorio se aborta ante degradación visual.
4. El waypoint ejecutado contiene el yaw seleccionado y es continuo/alcanzable.
5. Seguridad física nunca se infiere de los MapPoints.

Además, el build requerido debe devolver `0`, las pruebas obligatorias deben haberse ejecutado, los marcadores deben aparecer sin errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si yaw funciona en exterior pero depende de una regla que falla en interior.
- `PARCIAL` si se detecta degradación pero el giro no se limita a tiempo.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos.

## Riesgos

- Oscilaciones de yaw por cambiar continuamente de candidato.
- Sobreajuste del scorer a cantidad de MapPoints ignorando distribución.
- Giros demasiado frecuentes que empeoren el control o generen blur.

## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar solo la documentación que corresponda al código tocado, incluyendo:

```text
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6M.md
codex/pipeline/fase_6_tareas_trayectorias/historial/por_subfase/historial_6M_RESUMEN.md
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
