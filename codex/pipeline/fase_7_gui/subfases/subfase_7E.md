# Subfase 7E — Nube sparse, filtro visual y color por score

## Estado

```text
conseguida
```

## Dependencia

7C y 7D conseguidas; Fase 3 debe proporcionar un mapa sparse global validado.

## Objetivo técnico

Representar eficientemente los MapPoints del mapa global y añadir los dos controles de score acordados: umbral exclusivamente visual y color por gradiente rojo→amarillo→verde. La capa no debe modificar el mapa ni el score del backend.

## Comportamiento esperado

`SparseMapLayer` consume una revisión coherente del mapa global. Con `Sparse` desactivado no se dibuja, pero el backend sigue publicando/actualizando. El umbral oculta solo en GUI los puntos de score inferior. El modo de color usa una normalización estable basada en la semántica real del score.

3F genera temporalmente en `GlobalMapServer` un RGB rojo-amarillo-verde para
que RViz2 permita validar la nube antes de existir la GUI. En 7E esa
responsabilidad se transfiere a `SparseMapLayer`: la GUI calcula el color desde
el campo canonico `score` y, una vez validada la ruta, se retira del servidor la
generacion de RGB de presentacion. Esta transferencia no cambia scores,
geometria ni seleccion de puntos.

Debe poder seleccionarse un MapPoint posteriormente en 7H, por lo que conservará una identidad/índice suficiente si el productor la ofrece.

## Contexto obligatorio a leer


```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_7_gui/pipeline_fase_7_RESUMEN.md
codex/pipeline/fase_7_gui/pipeline_fase_7.md
```

Antes de modificar código, leer además los resúmenes/contratos finales **reales** de Fases 3, 4, 5 y 6 que sean productores de los datos usados por esta subfase, y los MD vigentes de cada paquete afectado en `codex/contexto/paquetes/`.

No asumir que los nombres de topics/messages del snapshot documental siguen siendo los definitivos tras ejecutar Fases 3–6. Si ya existe un contrato equivalente, reutilizarlo. No crear una segunda fuente de verdad solo para la GUI.


## Diagnóstico de partida

El contrato rehecho de 3F exige que `/global_sparse_cloud` incluya `score`,
identidad de submapa y un RGB temporal. Antes de implementar 7E se inspecciona
el mensaje real para confirmar el contrato, no para volver a decidir su
ownership. Si falta `score` o identidad, se corrige/reabre Fase 3; la GUI no los
infiere. Si solo falta RGB no es bloqueo porque 7E debe calcularlo desde score.

## Invariantes y decisiones cerradas

- Umbral de score es solo GUI.
- Activar/desactivar sparse no afecta al publisher ni a `GlobalMapBuilder`.
- Gradiente fijo semánticamente: bajo rojo, medio amarillo, alto verde.
- El RGB de 3F es presentacion temporal; en 7E la GUI deriva su propio color
  desde `score` y deja de depender de ese RGB.
- No asumir score `[0,1]` sin comprobarlo.
- No normalizar cada frame por min/max si eso cambia el significado de color.
- Nube recibida debe estar ya en `world` o venir con frame transformable por contrato; no usar GT para colocarla.
- Si la nube real se ve geométricamente mal y el mensaje ya está mal, volver a Fase 3.

## Archivos permitidos a modificar

```text
src/servidor/multidron_gui_lib/src/render/sparse_map_layer.*
src/servidor/multidron_gui_lib/include/multidron_gui_lib/render/sparse_map_layer.*
src/servidor/multidron_gui_lib/src/widgets/score_controls.*
src/servidor/multidron_gui_lib/src/gui_data_model.*
src/servidor/orbslam3_server/               # retirar RGB temporal tras validar la GUI; preservar score/ID
codex/contexto/paquetes/multidron_gui_lib/
codex/contexto/paquetes/multidron_gui/
codex/contexto/paquetes/orbslam3_server/     # si se toca
```

Las rutas nuevas son de contrato. Antes de crearlas, comprobar el árbol posterior a Fases 2–6. Si existe un componente equivalente, reutilizarlo en lugar de duplicarlo.

## Archivos prohibidos

```text
src/servidor/orbslam3_multi/landmark_score_manager.*  # no cambiar algoritmo/threshold funcional
RawMapDatabase/GlobalPoseStore                         # no modificar para visualización
ORB_SLAM3/
```

Además:


- No usar Ground Truth como entrada funcional de GUI, pose, mapa, tareas o trayectorias; GT solo puede usarse como métrica externa de simulación cuando una prueba lo necesite.
- No incrustar ni depender de RViz2.
- No crear una aplicación web ni reutilizar `pipeline_flow` como GUI operativa.
- No ejecutar SLAM, fusión, optimización, planner, obstacle avoidance ni reconstrucción densa dentro del thread gráfico.
- No mandar `TrayAction` directamente desde la GUI para saltarse el sistema de tareas de Fase 6.
- No modificar `ORB_SLAM3` salvo una necesidad separada, demostrada y autorizada; `CAPTURE_SPARSE` no permite forzar KFs.
- No implementar Fase 8 dentro de Fase 7.
- No limpiar legacy o tocar paquetes ajenos como cambio colateral.
- No rellenar historiales con resultados ficticios; se crean únicamente tras ejecuciones reales.


## Funciones, clases, nodos o interfaces que hay que localizar

```text
/global_sparse_cloud real
GlobalMapBuilder / publisher sparse
LandmarkScoreManager y escala real del score
SparseMapLayer
controles `Sparse`, `score threshold`, `color by score`
PointCloud2 fields/estructura GUI real
```

Los nombres de componentes nuevos definidos por este contrato pueden implementarse con una estructura equivalente si existe una razón técnica clara. Los nombres de **interfaces procedentes de fases anteriores** no se inventan: se localizan físicamente primero.

## Cambios requeridos

1. Auditar que la nube sparse real cumple el contrato 3F: score e identidad
   estables, y distinguir esos datos canonicos del RGB temporal.
2. Si faltan score/ID, reabrir/corregir el productor de Fase 3; no inventarlos
   en GUI. No cambiar el algoritmo de score.
3. Convertir una revisión recibida a buffer GPU una sola vez por cambio relevante; evitar `glBegin`/dibujo punto a punto CPU.
4. Implementar toggle `Sparse`.
5. Implementar umbral visual con slider/campo numérico y validación de rango según la escala real.
6. Implementar modo `Color by score` rojo→amarillo→verde y modo normal.
7. Permitir filtro + gradiente simultáneamente.
8. Mantener metadata suficiente para picking futuro sin duplicar innecesariamente toda la nube.
9. Añadir logs `GUI-SPARSE-UPDATE`, `GUI-SCORE-FILTER`, `GUI-SCORE-COLOR` con counts/revision, no con puntos individuales.
10. Verificar que el renderer obtiene sus colores desde `score`, no desde el
    RGB temporal de 3F.
11. Tras validar equivalencia visual y consumidores, retirar de
    `GlobalMapServer` la conversion score->RGB. Mantener `score` e identidad en
    el mensaje; RViz2 deja de ser la autoridad de presentacion de Fase 7.

## Cambios prohibidos

- No cambiar score, geometria o seleccion del productor para simplificar el
  renderer. La retirada del RGB temporal del servidor es la migracion acordada,
  no un cambio del mapa.
- No convertir telemetría descartable/visual en una dependencia del pipeline.
- No realizar una decisión funcional no acordada si durante la integración aparecen varias alternativas razonables; parar y preguntar al usuario.


## Puerta de validación hacia fases anteriores

La GUI es herramienta de observación, no capa de maquillaje. Para cualquier anomalía:

1. capturar primero el valor/mensaje que recibe la GUI y la transformación aplicada;
2. si el dato de entrada ya es incorrecto, detener esta subfase y localizar la fase propietaria;
3. no aplicar offsets, escalados, filtros o estados falsos para que “se vea bien”;
4. si la corrección de origen implica comportamiento funcional, suspender autorización y consultar al usuario conforme a `AGENTS.md`;
5. corregir en la fase de origen, revalidarla y repetir después esta prueba de Fase 7;
6. si el dato recibido es correcto y solo se representa mal, corregir Fase 7.

Ejemplos de ownership: sparse/KFs Fase 3, fiduciales Fase 4, pose/tracking Fase 5, tarea/progreso/trayectoria Fase 6.

Si aparece una duda funcional no acordada por este contrato, Codex debe parar y preguntarle al usuario. Para dron perdido/stale/sin pose nueva válida ya rige la decisión cerrada: conservar última pose válida, mostrar `PERDIDO` y usar representación más transparente.


## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh --group servidor multidron_gui_lib
./codex/herramientas/build_selected_packages.sh --group servidor multidron_gui
./codex/herramientas/build_selected_packages.sh --group servidor orbslam3_server
```

Si no se modifica `orbslam3_server`, compilar solo `multidron_gui_lib` y `multidron_gui`.

Si la separación de Fase 2 utiliza builds por grupo, usar el helper vigente para el grupo Servidor y, cuando haya pruebas de integración, los grupos Dron/Simulación correspondientes. Registrar el comando exacto solo en el historial real.

## Pruebas Gazebo requeridas

### Prueba 1 — Nube sintética con scores conocidos

Crear un dataset de pocos puntos con scores bajo/medio/alto y RGB de entrada
deliberadamente distinto. Verificar que filtro y gradiente se calculan de forma
determinista desde `score`, no copiando el RGB recibido.

### Prueba 2 — Sparse global real

Arrancar el escenario típico de Fase 3/6 y mostrar el sparse real. Activar/desactivar la capa y mover el umbral mientras los drones mapean. El backend debe continuar idéntico.

### Prueba 3 — Puerta visual Fase 3

Comparar mensaje/poses de puntos con lo renderizado. Si aparecen paredes duplicadas, desplazamientos o escalas erróneas, determinar si el dato ya llega incorrecto antes de corregir GUI.

### Prueba 4 — Retirada de RGB temporal

Con la GUI ya validada, retirar la conversion score->RGB del servidor y repetir
la nube real. La GUI debe conservar exactamente el gradiente; `score`, identidad,
geometria y comportamiento del backend no cambian.

No arrancar Gazebo artificialmente para una prueba puramente gráfica/unitaria. Cuando se use simulación, el comando base es:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba fase_7_7E \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py" \
  --post-scenario-wait-sec 20
```

El mecanismo exacto para arrancar `multidron_gui` junto a ese launch se fija en 7A/7B y debe reutilizarse después. La GUI nunca depende de que RViz2 esté abierto.

## Patrones de reducción de logs

```text
GUI-SPARSE-UPDATE|GUI-SCORE-FILTER|GUI-SCORE-COLOR|global_sparse_cloud|score|revision|F1F-GLOBALMAP|F1T-RVIZ|ERROR|FATAL|Segmentation fault|Killed
```

Los logs completos solo alimentan reductores. Si falta evidencia, regenerar el reducido con patrones más precisos; no abrir el log completo directamente.

## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Sparse real se dibuja en `world` de forma coherente.
2. Toggle funciona sin alterar backend.
3. Umbral solo afecta a los puntos visibles.
4. Gradiente representa score estable y puede combinarse con umbral.
5. Rendimiento permite navegar la escena con el sparse real.
6. Cualquier error real de Fase 3 detectado se corrige en Fase 3 y después se revalida.
7. La GUI deriva colores desde `score` y no depende del RGB temporal de 3F.
8. La responsabilidad score->RGB queda retirada de `GlobalMapServer` sin
   perder score/identidad ni alterar consumidores funcionales.

Además, todo build requerido debe devolver `0`, todas las pruebas obligatorias deben ejecutarse, no puede haber errores graves no explicados y la documentación/historial real debe quedar sincronizada.

## Criterio de fallo o parcial

- `PARCIAL` si se dibuja sparse pero no existe score fiable consumible.
- `PARCIAL` si el gradiente cambia de significado entre revisiones por normalización dinámica inadecuada.
- `PARCIAL` si el renderer funciona solo con nubes pequeñas y bloquea con la nube real.

- `NO CONSEGUIDA` si no compila, una prueba obligatoria no se ejecuta, falta evidencia crítica o el comportamiento contradice el objetivo.
- `BLOQUEADA` solo ante dependencia/información externa realmente irresoluble con cambios mínimos o porque una fase anterior debe corregirse antes de continuar.

## Riesgos

- Romper consumidores del `PointCloud2` al añadir campos sin comprobar compatibilidad.
- Duplicar una nube completa en varios formatos/topics sin necesidad.
- Filtrar en servidor y confundir threshold GUI con threshold funcional.
- Culpar al renderer de un mapa global realmente incoherente.

## Documentación a actualizar


Al ejecutar realmente la subfase, actualizar solo documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7E.md
codex/pipeline/fase_7_gui/historial/por_subfase/historial_7E_RESUMEN.md
codex/pipeline/fase_7_gui/pipeline_fase_7_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md              # si cambia el estado real
codex/contexto/paquetes/<paquete_afectado>/
```

Los historiales **no existen en este ZIP a propósito**. Deben crearse solo cuando la subfase se ejecute de verdad. La documentación de paquete debe describir el código actual: ejecutables/nodos, clases, topics/services/actions, parámetros, markers y limitaciones vigentes.


## Dudas funcionales de contrato

```text
ninguna en el acuerdo actual
```

Cualquier duda funcional nueva descubierta durante preparación/ejecución debe presentarse al usuario antes de continuar.
