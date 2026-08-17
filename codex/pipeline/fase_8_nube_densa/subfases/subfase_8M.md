# Subfase 8M — Revalidación de fusiones y correcciones tras optimizaciones


## Estado

```text
sin hacer
```


## Objetivo técnico

Reevaluar únicamente las relaciones que pueden quedar obsoletas cuando cambia una revisión de poses: registros/fusiones entre varios KFs, constraints dense, correcciones MP↔dense y bloques fusionados. Las relaciones puramente locales dentro del mismo KF deben sobrevivir por construcción salvo que cambie el raw/observación que las originó.


## Invariantes y decisiones cerradas

- Una optimización no obliga a recomputar todo: diferenciar relaciones intra-KF de relaciones inter-KF.
- Si dos patches antes coplanares dejan de serlo tras una nueva pose, la relación debe revalidarse/recalcularse o invalidarse.
- Nunca mantener una constraint antigua “porque una vez fue buena” si su revisión/soporte ya no aplica.


## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_8_nube_densa/pipeline_fase_8_RESUMEN.md
codex/pipeline/fase_8_nube_densa/pipeline_fase_8.md
```

Antes de modificar código, leer también los resúmenes/contratos **reales ya ejecutados** de las fases anteriores de las que dependa esta subfase y los MD vigentes de cada paquete afectado en `codex/contexto/paquetes/`. En particular, Fase 8 depende funcionalmente de Fase 3 (sparse global), Fase 5 (poses globales), Fase 6 (tareas/trayectorias/obstáculos) y Fase 7 (GUI). Si el workspace posterior a esas fases usa nombres o rutas distintos a los de este contrato, localizar primero la fuente canónica y reutilizarla; no crear una segunda interfaz solo para satisfacer el nombre documental.



Además, leer específicamente:

Leer mecanismos de revisión/stale de Fase 3 y 8I, además de DBs creadas en 8J–8L.


## Diagnóstico de partida

Aunque puntos dense y MPs locales a un mismo KF se mueven juntos, una fusión `KF_A↔KF_B` puede dejar de ser coherente si A y B cambian de forma distinta. Sin revalidación pueden reaparecer paredes quebradas o constraints obsoletas que intenten deshacer una optimización correcta.


## Archivos permitidos a modificar

```text
src/servidor/dense_map_multi/
src/servidor/dense_map_server/
src/servidor/orbslam3_multi/              # solo estado/revisión/invalidación necesaria
```


Las rutas nuevas indicadas son de **contrato propuesto**. Antes de crearlas, comprobar el árbol real posterior a Fases 2–7 y reutilizar componentes equivalentes si existen.


## Archivos prohibidos

```text
build/                                      # no modificar manualmente
install/                                    # no modificar manualmente
log/                                        # no modificar manualmente salvo limpieza mínima autorizada por herramientas
src/dron/ORB_SLAM3/                         # salvo necesidad explícita, demostrada y nuevo acuerdo
legacy/ o *_antiguo.*                       # no tocar como solución de Fase 8
paquetes ajenos a la subfase                # salvo dependencia real localizada y justificada
```


## Funciones, clases, nodos o interfaces que hay que localizar

```text
DenseRegistrationResult / constraint DB
SparseDenseCorrectionDatabase
DenseReintegrationManager
GlobalPoseStore revision
DenseKeyFrameDatabase revision
```



No inventar nombres de interfaces previas. Si alguno no existe con ese nombre, localizar el componente equivalente mediante documentación y búsqueda estática antes de implementar. Los nombres nuevos propuestos pueden materializarse con una estructura equivalente si conserva el ownership acordado.


## Cambios requeridos

1. Definir dependencias/revisión de cada relación inter-KF: KFs, DenseKF, superficies y pose revision usadas.
2. Al commit de poses, identificar relaciones tocadas por KFs que cambiaron.
3. Recalcular residual/solape de registros dense; mantener, recomputar o invalidar según evidencia actual.
4. Revalidar correcciones MP↔dense si cambia el raw del MP/KF o la superficie fuente; las puramente locales no se invalidan solo por un rigid transform world.
5. Sincronizar invalidaciones con 8I para que el mapa fusionado no siga usando una relación descartada.
6. Evitar bucle infinito optimización→constraint→optimización con cooldown/versionado/criterio de cambio material según diseño real.
7. Añadir markers `DENSE-REVALIDATE`, `DENSE-RELATION-INVALID`, `DENSE-RELATION-KEEP`, `DENSE-RELATION-RECOMPUTE`.


## Cambios prohibidos

- No usar Ground Truth para calcular disparity/depth, colocar la nube densa, fusionar, corregir poses, refinar MapPoints, decidir ocupación o validar online una trayectoria. GT solo puede aparecer como métrica externa de simulación.
- No modificar datos raw de ORB-SLAM3 en `RawMapDatabase`.
- No devolver MapPoints corregidos al ORB-SLAM3 que corre en el dron.
- No ejecutar reconstrucción densa pesada en el dron: el dron se limita a capturar y enviar información.
- No convertir `orbslam3_server` ni `dense_map_server` en un backend algorítmico monolítico; los algoritmos densos pertenecen a `dense_map_multi`.
- No bloquear ingesta sparse, pose, control, GUI o ejecución de tareas mientras se calcula disparity, registro, voxelización, fusión o reintegración.
- No almacenar imágenes L/R permanentemente como parte de `DenseKeyFrameDatabase`; si una zona queda mal, la estrategia acordada es volver a observarla/recapturarla.
- No implementar por adelantado subfases posteriores salvo infraestructura mínima estrictamente necesaria y documentada.
- No limpiar legacy ni cambiar paquetes ajenos como efecto colateral.
- No crear historiales con resultados ficticios. Las carpetas se entregan vacías y los MD de historial nacen solo tras ejecuciones reales.
- No volver a ejecutar indiscriminadamente todos los registros tras cada cambio mínimo.
- No reactivar automáticamente una constraint que el optimizador rechazó.


## Puerta de validación hacia fases anteriores

Fase 8 no puede maquillar fallos de las fases previas. Si durante la preparación o prueba aparece una incoherencia cuyo dato de entrada ya es incorrecto:

1. identificar la fuente propietaria (sparse/KFs Fase 3, poses Fase 5, tareas/obstáculos Fase 6, visualización Fase 7);
2. detener la implementación de Fase 8 que dependa de ese dato;
3. registrar el bloqueo/diagnóstico sin inventar una compensación local;
4. volver a la fase propietaria, corregirla y revalidarla según `AGENTS.md`;
5. repetir después la prueba de Fase 8.

No se aceptan offsets, deformaciones, filtros visuales o copias de estado cuyo único objetivo sea ocultar un error previo.


## Paquetes a compilar

```bash
./codex/herramientas/build_selected_packages.sh dense_map_multi dense_map_server orbslam3_multi
```

Si la separación de Fase 2 está ya ejecutada, respetar sus builds por grupo (`dron`, `servidor`, `simulacion`) y la sincronización de las dos copias de `orbslam3_msgs`. Los comandos listados son el conjunto lógico esperado; el implementador debe usar el helper vigente y registrar en historial el comando real.


## Pruebas Gazebo requeridas

### Prueba 1 — Relación que sigue válida

Optimizar ambos KFs casi rígidamente: la relación debe conservarse sin recomputación innecesaria.

### Prueba 2 — Pared que se quiebra

Mover A/B de forma diferencial mediante una revisión válida y comprobar que el registro/coplanaridad se revalida y no se conserva ciegamente.

### Prueba 3 — Corrección MP local

Mover el KF globalmente; la corrección MP↔surface local permanece válida.

### Prueba 4 — Raw actualizado/reconciliado

Si ORB reexporta un MP/KF cambiado, la corrección derivada dependiente se revisa antes de publicar.

No arrancar Gazebo artificialmente para una prueba puramente unitaria/de componente. Cuando la subfase necesite integración visual, temporal o multi-nodo, usar `run_simulation.sh` y registrar el comando exacto solo en el historial real.


## Patrones de reducción de logs

```text
DENSE-REVALIDATE|DENSE-RELATION|revision|invalid|keep|recompute|DENSE-MP|ERROR|FATAL|Killed
```

Los logs completos solo son entrada de `reduce_*`/`split_*`. Si el reducido no contiene evidencia suficiente, regenerarlo con patrones más específicos o crear un sublog temático; no abrir ni volcar el log completo.


## Criterio de éxito

La subfase se considera `CONSEGUIDA` solo si:

1. Las relaciones inter-KF obsoletas se detectan y no contaminan el estado vigente.
2. Las relaciones intra-KF no se recalculan innecesariamente por un rigid transform global.
3. No aparece un bucle de optimizaciones/constraints sin cambio material.
4. Mapa dense y sparse publicado quedan en la misma revisión coherente tras estabilizar.


## Criterio de fallo o parcial

- `NO CONSEGUIDA` si una constraint/fusión obsoleta permanece activa tras romperse su geometría.
- `PARCIAL` si revalida correctamente pero el coste es excesivo y requiere indexación adicional.


## Riesgos

- Oscilación de constraints cerca de thresholds; usar hysteresis/versionado si la evidencia lo exige.


## Documentación a actualizar

Al ejecutar realmente la subfase, actualizar únicamente documentación respaldada por cambios/evidencia real:

```text
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8M.md
codex/pipeline/fase_8_nube_densa/historial/por_subfase/historial_8M_RESUMEN.md
codex/pipeline/fase_8_nube_densa/pipeline_fase_8_RESUMEN.md
codex/contexto/01_ESTADO_ACTUAL.md                 # si cambia el estado real
codex/contexto/paquetes/<paquete_afectado>/
```

Si se modifica código, el MD del paquete debe reflejar clases/funciones, topics/services/actions, parámetros, markers, ownership, limitaciones y estado de validación vigente. No marcar la subfase como `realizado` sin cumplir su criterio de éxito.


## Dudas funcionales de contrato

```text
ninguna en el acuerdo actual
```

Si durante preparación/ejecución aparece una alternativa funcional material no cubierta por este contrato, suspender la autorización y consultarla al usuario conforme a `AGENTS.md`.
