# Subfase 2G — Guardas automáticas, regresión final y cierre de Fase 2

## Estado

```text
SIN HACER
Dependencias: 2A-2F implementadas y con evidencia suficiente
Propósito: convertir la arquitectura acordada en una propiedad verificable
```

## Objetivo técnico

Añadir comprobaciones automáticas que detecten regresiones de estructura,
interfaces, YAML, debug, dependencias y documentación; ejecutar de nuevo los
builds y la prueba oficial; retirar compatibilidad antigua solo cuando su
sustitución esté demostrada; y cerrar la Fase 2 con un handoff reproducible.

La fase no queda cerrada únicamente porque los paquetes estén dentro de tres
carpetas. Debe impedirse que el proyecto vuelva a mezclar grupos o duplicar
configuración silenciosamente.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/09_LOGS_Y_SUBLOGS.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2.md
codex/pipeline/fase_2_separacion_paquetes/subfases/subfase_2A.md a subfase_2F.md
historiales y resúmenes reales de 2A-2F
codex/herramientas/USO_HERRAMIENTAS.md
```

No ejecutar 2G si una subfase anterior conserva una duda funcional abierta.

## Guardas obligatorias

### 1. Ubicación de paquetes

Comprobar que:

```text
dron_individual        -> src/dron/
lib_tray                -> src/dron/
ORB_SLAM3               -> src/dron/
orbslam3_ros2           -> src/dron/
orbslam3_multi          -> src/servidor/
orbslam3_server         -> src/servidor/
simulacion_dron         -> src/simulacion/
```

`codex` permanece en `src/codex/`.

Fallar si un paquete principal reaparece en la raíz de `src/` o en un grupo no
permitido.

### 2. Duplicación controlada de `orbslam3_msgs`

Permitir exactamente:

```text
src/dron/orbslam3_msgs
src/servidor/orbslam3_msgs
```

Rechazar una tercera copia.

Comparar al menos:

- todos los `.msg`;
- todos los `.srv`;
- todos los `.action` futuros;
- `CMakeLists.txt` relevante para generación;
- `package.xml` y versión;
- archivos auxiliares de interfaz.

La comparación puede ignorar solo metadatos explícitamente documentados que no
afecten al contrato. Por defecto debe ser estricta.

Servidor es canónico. El error debe indicar qué archivo diverge y cómo
sincronizarlo.

### 3. Descubrimiento por grupo

Comprobar automáticamente:

```text
colcon list --base-paths src/dron
colcon list --base-paths src/servidor
colcon list --base-paths src/simulacion
```

Fallar si:

- un grupo descubre paquetes de otro;
- un mismo build descubre dos paquetes con el mismo nombre;
- falta un paquete obligatorio;
- Simulación contiene copias completas de paquetes de Dron/Servidor.

### 4. Dependencias prohibidas

Analizar `package.xml`, `CMakeLists.txt`, setup y código para detectar:

**Dron:**

- dependencia de `orbslam3_server` o `orbslam3_multi`;
- dependencia de `simulacion_dron`;
- Gazebo/plugins/modelos cuando no haya consumidor embarcado justificado;
- rutas a `sensor/GT/*` como requisito de despliegue del grupo.

**Servidor:**

- dependencia de `dron_individual` o `lib_tray`;
- dependencia de `simulacion_dron`;
- Gazebo, modelos o plugins;
- acceso directo a YAML de Dron.

**Simulación:**

- copias completas de paquetes externos;
- rutas directas al código fuente de otros grupos;
- integración que no use instalaciones/índice de paquetes.

No prohibir a Simulación depender de Dron y Servidor.

### 5. Rutas y recursos instalados

Rechazar en runtime/configuración vigente:

```text
../../dron
../../servidor
../../simulacion
src/dron/...
src/servidor/...
src/simulacion/...
src/codex/archivos_auxiliares como fuente runtime
```

Comprobar que launch, YAML, worlds, models, URDF, RViz, vocabulario, web y
escenarios están instalados bajo `share/<package>` o una ruta de resultados
configurable.

Distinguir documentación/historial, donde las rutas antiguas pueden aparecer
como evidencia.

### 6. YAML por grupo

Comprobar que:

- cada YAML pertenece a un paquete de su grupo;
- no hay launch que cargue YAML de otro grupo;
- los YAML parsean;
- los nombres de réplica siguen `_<grupo_origen>`;
- las réplicas declaran origen y claves;
- no contienen claves no consumidas cuando pueda verificarse;
- no hay duplicados semánticos declarados dentro del grupo;
- todos los YAML están documentados.

### 7. Coherencia de réplicas parciales

Mantener una definición declarativa de claves replicadas. Ejemplo conceptual:

```yaml
replicas:
  - source: src/dron/dron_individual/config/physical.yaml
    target: src/simulacion/simulacion_dron/config/physical_dron.yaml
    keys:
      - mass_total
      - inertia_total
```

La guarda compara solo esas claves y falla si divergen.

No sincronizar automáticamente sin mostrar el cambio; la copia de destino debe
actualizarse de forma explícita y revisable.

### 8. Masa e inercia

Comprobar dentro de Dron:

- `mass_total` aparece en un único YAML propietario;
- `inertia_total` aparece en un único YAML propietario;
- control/trayectoria no mantienen copias con el mismo significado;
- los launch los pasan a los consumidores;
- las propiedades por enlace de Simulación no se confunden con las globales.

No validar el valor físico ni recalcularlo.

### 9. Debug

Comprobar que:

- cada función tiene su propio flag;
- todos los flags `debug_*` son booleanos;
- todos tienen valor predeterminado `false`;
- cada flag tiene consumidor o está marcado como legacy pendiente;
- procesos web, navegador, RViz, dumps y telemetría no arrancan cuando su flag
  está desactivado;
- la prueba base no habilita debug mediante override oculto.

### 10. Escenarios y resultados

Comprobar que:

- los escenarios runtime están en Simulación;
- el escenario de vuelta al edificio está instalado;
- `codex` no es dependencia runtime para localizarlo;
- los resultados se escriben en `log/`, ruta configurable o temporal;
- no aparecen nuevos artefactos generados dentro de `src/`.

### 11. Documentación

Comprobar:

- rutas vigentes existen;
- todos los paquetes tienen grupo documentado;
- todos los YAML tienen MD o sección de documentación;
- la copia canónica de mensajes está indicada;
- comandos de build usan grupos;
- no hay enlaces Markdown rotos relevantes;
- los MD de fase no contienen resultados ficticios;
- historiales de 2A-2G existen solo tras ejecución.

### 12. Visualizadores

Comprobar:

- `pipeline_flow` y `system_architecture` existen separados;
- ambos se instalan;
- sus flags son independientes;
- ninguno es dependencia funcional del pipeline;
- cerrar bridge/navegador no detiene ROS 2;
- la topología estática del diagrama coincide con el mapa de paquetes.

## Herramienta de guarda

Crear una herramienta principal, por ejemplo:

```text
codex/herramientas/check_workspace_architecture.py
```

o nombre equivalente coherente con el proyecto.

Debe:

- devolver código distinto de cero ante incumplimiento;
- imprimir errores concretos y rutas;
- permitir comprobaciones individuales y ejecución completa;
- no modificar archivos por defecto;
- ser rápida para uso antes de builds largos;
- producir un log reducible;
- documentar sus limitaciones.

Puede apoyarse en archivos de política declarativos dentro de `codex/contexto/`
o `codex/herramientas/`.

## Secuencia de cierre

### 1. Ejecutar guardas estáticas

Antes de build:

```text
ubicación
interfaces duplicadas
manifests/dependencias
rutas
YAML
réplicas
debug
documentación
```

Corregir todos los errores o justificar una excepción aprobada.

### 2. Limpieza final

Borrar de nuevo:

```text
build/
install/
log/
```

No conservar una falsa validación basada en artefactos anteriores.

### 3. Build final por grupos

Usar la estrategia principal de 2B:

```text
Dron paquete a paquete
Servidor paquete a paquete
Simulación como overlay
```

Si durante 2B se adoptó una alternativa, repetir exactamente esa estrategia y
documentar por qué sigue siendo necesaria.

### 4. Tests locales

Ejecutar todos los tests obligatorios de paquetes, configuración, herramientas y
visualizadores.

### 5. Prueba base sin debug

Ejecutar dos drones alrededor del edificio con todos los flags de debug en
`false`.

### 6. Pruebas de visualización

Ejecutar comprobaciones independientes:

- `pipeline_flow`;
- `system_architecture` estático y en vivo;
- desconexión/saturación de ambos.

No es necesario repetir siempre toda la prueba larga si un smoke controlado
prueba un fallo aislado, pero la actividad en vivo del nuevo diagrama debe verse
al menos una vez durante el escenario representativo.

### 7. Repetir guardas

Después de pruebas y documentación, ejecutar de nuevo la herramienta de
arquitectura para detectar archivos generados o flags dejados activos.

### 8. Limpieza de compatibilidad antigua

Eliminar solo después de probar sustitutos:

- rutas antiguas a paquetes en la raíz;
- YAML legacy sin consumidores;
- parámetros de launch duplicados por la nueva configuración;
- scripts que usan un único `install/setup.bash` si ya no son válidos;
- escenarios runtime duplicados bajo `codex`;
- salidas dentro de `src/`;
- flags debug antiguos sin consumidor.

No borrar historiales ni pruebas pasadas.

## Patrones de reducción

### Guardas/build

```text
ARCH-CHECK|PASS|FAIL|duplicate|diverge|forbidden|missing|yaml|debug|Starting >>>|Finished <<<|Failed <<<|ERROR|FATAL
```

### Simulación

```text
SCENARIO-RUNNER|GOAL|RESULT|success|dron_1|dron_2|SYSTEM-ARCH|PIPELINE-FLOW|READY|dropped|ERROR|FATAL|Segmentation fault|Killed|SIM-DONE
```

## Criterio de cierre de la fase

Fase 2 queda `CONSEGUIDA` solo si:

1. las guardas estáticas pasan;
2. Dron compila en aislamiento;
3. Servidor compila en aislamiento;
4. Simulación compila como overlay;
5. las copias de `orbslam3_msgs` son idénticas;
6. YAML, réplicas y debug cumplen la política;
7. todos los tests obligatorios pasan;
8. dos drones completan la vuelta al edificio sin debug;
9. ambos visualizadores se validan por separado;
10. desconectar visualizadores no afecta al sistema;
11. la documentación coincide con el repositorio;
12. no quedan rutas runtime antiguas ni resultados dentro de `src/`;
13. el checkpoint final declara trabajo activo: no;
14. existe handoff claro a Fase 3.

## Criterio de parcial, fallo o bloqueo

`PARCIAL` si el sistema funciona y la prueba pasa, pero falta una guarda o
limpieza no funcional claramente acotada. La Fase 3 no debe asumir que la
separación está cerrada hasta resolverla.

`NO CONSEGUIDA` si una guarda crítica falla, un grupo no compila aislado, las
interfaces divergen, los YAML cruzan grupos o la prueba oficial no termina.

`BLOQUEADA` solo si una dependencia externa imprescindible impide el build o la
prueba y no existe alternativa acordada.

## Handoff final

Dejar documentado:

- árbol definitivo;
- copia canónica y procedimiento de sincronización de mensajes;
- comandos de build y source;
- alternativa de build usada, si no fue la principal;
- mapa de YAML y políticas de edición;
- escenario oficial;
- flags de debug;
- rutas de visualizadores;
- herramienta de guardas;
- limitaciones conocidas;
- siguiente acción de Fase 3.

## Cambios prohibidos

- No rebajar guardas para hacerlas pasar.
- No excluir paquetes fallidos sin acuerdo.
- No sincronizar interfaces ocultando el diff.
- No activar debug por defecto para obtener evidencia.
- No borrar fallos históricos.
- No iniciar trabajo funcional de Fase 3 durante el cierre.
- No declarar `CONSEGUIDA` sin prueba larga y documentación coherente.

## Documentación de cierre

Crear/actualizar al ejecutar:

```text
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2G.md
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2G_RESUMEN.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/01_ESTADO_ACTUAL.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/07_ULTIMA_SESION.md
codex/contexto/paquetes/**
```

No crear un resultado final que oculte intentos fallidos de las subfases.
