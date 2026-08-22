# Subfase 3V - Regresion integral de flujos y observabilidad

## Estado

```text
CONSEGUIDA POR EVIDENCIA INTEGRAL ACUMULADA Y ACEPTACION DEL USUARIO
```

No se ejecuta una prueba nueva etiquetada exclusivamente como `3V`. Las
regresiones 187, 188, 191 y 194, los tests funcionales y las revisiones humanas
ya recorren conjuntamente flujo principal, worker secundario serial,
prioridades, loops, fiduciales, fusion, scoring, publicacion, RViz2 y grafo web.
El usuario considera muy buenas estas pruebas y acepta su conjunto como cierre.

Los fallos inducidos y la comparacion A/B de telemetria descritos mas abajo
quedan como protocolo reutilizable ante una regresion, no como trabajo
obligatorio pendiente de esta subfase.

## Objetivo

Validar conjuntamente que el flujo principal permanece vivo, el secundario
respeta prioridad/serializacion, los commits son coherentes y RViz2/diagrama JS
reflejan el estado sin participar en las decisiones.

## Precondiciones

- contratos `3C-3U` implementados;
- sin `PostOptimizationKeyFrameQueue` ni workers por tarea;
- visualizador puede arrancar en modo desktop y headless;
- prueba larga y reductores disponibles.

## Build

```bash
./codex/herramientas/build_selected_packages.sh \
  orbslam3_multi orbslam3_server simulacion_dron
```

Ejecutar todos los tests deterministas añadidos por las subfases afectadas antes
de Gazebo.

## Prueba principal

```text
YAML: codex/archivos_auxiliares/trayectorias/prueba_tipica_rodeo_edificio_dos_fiduciales.yaml
launch: ros2 launch simulacion_dron multi_dron.launch.py
RViz2: activo
pipeline flow visualizer: activo
espera post-escenario: suficiente para observar la tarea final, inicialmente 120 s
```

No ampliar timeouts para ocultar una cola que no drena. Si la ruta no produce
una rama `3Q` real, añadir replay/control sintetico independiente sin GT en la
decision y mantener la prueba live como evidencia separada.

## Secuencias obligatorias

### Flujo principal

```text
delta/snapshot
-> raw commit + ChangeSet
-> pose/covis/score principal
-> publication request
-> KF/nube visibles
```

Debe repetirse mientras el worker esta `ACTIVE`.

### Prioridad

Observar o inducir deterministicamente:

```text
LoopTask A ACTIVE
LoopTask B QUEUED
FiducialTask F QUEUED durante A
```

Orden requerido: `A end -> F start/end -> B start`.

### Loop completo

```text
un task_id
enqueue -> 3N -> 3O -> decision
-> 3P FUSION, o 3Q OPTIMIZATION -> 3P opcional
-> commit -> task end -> publication request
```

### Fiducial

Primer fiducial crea anchor sin tarea. Revisit valida crea tarea prioritaria,
commit de poses y actualizacion del control para KFs futuros.

## Fallos inducidos seguros

1. Cerrar navegador durante la ruta.
2. Saturar cola de telemetria con limite bajo en un test controlado.
3. Detener/desactivar temporalmente consumo secundario y verificar flujo
   principal.
4. Producir candidato stale y comprobar ausencia de retry inmediato.
5. Rechazar un candidato de optimizacion sin estado provisional.

No forzar fallos mediante cambios de GT o alteracion del mapa real.

## Evidencia automatica

- latencias de ingesta, pose y publicacion;
- revisiones raw/pose/covis/fusion/score/derived/publication;
- orden y estado de tareas por prioridad;
- `active_secondary_workers <= 1`;
- profundidad y coalescing de colas;
- commits y bases modificadas;
- KFs/MPs/tracks publicados;
- `fallback_submap_points=0`;
- hard fiducials inmoviles;
- `raw_db_modified=false` tras tareas;
- eventos/drop/reconexion del visualizador;
- estado final y cierre limpio.

## Revision humana

En RViz2 comprobar progresion del mapa, anchors, correcciones y fusion. En la
pagina comprobar que las aristas activadas corresponden a los mismos IDs y
orden de logs reducidos. Una discrepancia se considera fallo de observabilidad,
no se corrige alterando el algoritmo.

## Comparacion A/B de telemetria

Repetir al menos un test corto determinista con telemetria `on/off`. Deben
coincidir decisiones, revisiones y resultados. Medir overhead; no exigir tiempos
byte a byte identicos.

## Logs

El log completo solo alimenta reductores. Patrones iniciales:

```text
F1C-|F1D-|F1E-|F1F-|F1H-|F1K-TASK|F1M-|F1N-|F1O-|F1P-|F1Q-|F1S-|
FLOW-EVENT|BACKPRESSURE|SCENARIO-RUNNER|SIM-DONE|ERROR|FATAL|Killed
```

Crear sublogs por flujo/tarea si el reducido es grande.

## Exito

- build y tests locales pasan;
- escenario completa o cualquier fallo queda diagnosticado como ejecución
  propia, no ocultado;
- flujo principal no se bloquea por tareas;
- prioridad y serializacion son exactas;
- commits no muestran estados parciales;
- resultados secundarios aparecen despues en RViz2;
- visualizador refleja eventos y puede fallar sin impacto;
- shutdown no deja procesos/threads necesarios activos;
- documentacion e historiales quedan sincronizados.

El cierre vigente satisface estos criterios mediante evidencia distribuida. No
se afirma que exista una ejecucion unica que contenga todos los fallos inducidos
ni una comparacion A/B nueva.

## Parcial/fallo

`PARCIAL` si la arquitectura pasa pero falta una rama real, inspeccion humana o
prueba A/B.

Este criterio se conserva para evaluar una futura ejecucion 3V aislada. En el
cierre vigente, la inspeccion humana existe y el usuario acepta expresamente la
evidencia distribuida sin exigir una A/B adicional.

`NO CONSEGUIDA` si se congela ingesta/publicacion, se viola prioridad, hay dos
tareas activas, se espera ACK visual, un commit queda parcial, el visualizador
cambia el resultado o aparecen deadlock/crash/NaN no explicados.

## Regresion Visual Obligatoria

Aplicar `../CONTRATO_VISUAL_INCREMENTAL.md`. No añadir topologia nueva para
hacer pasar la prueba. Validar orden completo de flujos reales, convergencia al
presente, ausencia de replay y coherencia final con RViz2.
