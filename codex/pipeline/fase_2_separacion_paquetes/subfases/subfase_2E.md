# Subfase 2E — Actualizar toda la documentación y el contexto de Codex

## Documentación definitiva

El cierre debe explicar de forma coherente:

- ownership, authority y `deployment profile`;
- la excepción declarada de réplica completa `global_map`;
- Dron como caja negra y los contratos futuros de configuración/calibración;
- GT y fiducial simulado como deudas provisionales de Fases 5 y 4;
- `use_sim_time` standalone frente al override de Simulación;
- bootstrap/preflight del `ORBvoc.txt` completo;
- layout `build/install/log/<grupo>` y build de un paquete por invocación;
- `codex/archivos_auxiliares` como evidencia, nunca dependencia funcional;
- reducción obligatoria antes de leer logs completos;
- semántica, independencia y coste desactivado de ambos grafos web.

`system_architecture` y `pipeline_flow` tienen documentación propia y separada.
Los contratos describen comportamiento vigente; los intentos y pruebas viven
en los historiales.

## Estado

```text
CONSEGUIDA
Dependencia: distribución y YAML estabilizados
Resultado: contexto, índices, paquetes, herramientas, ADR e historiales sincronizados
```

## Objetivo técnico

Actualizar el contexto, los índices, los MD de paquetes, las herramientas y las
referencias del pipeline para que la nueva estructura sea la única distribución
vigente descrita por Codex.

La documentación debe dejar claro:

- qué paquetes pertenecen a cada grupo;
- qué grupos se compilan de forma aislada;
- cómo Simulación integra Dron y Servidor;
- por qué existen dos copias de `orbslam3_msgs`;
- cuál es la copia canónica;
- cómo se compilan y se cargan los tres prefijos;
- qué YAML posee cada dato;
- qué parámetros son protegidos, ajustables o de debug;
- cómo se usan réplicas parciales entre grupos;
- dónde viven escenarios y resultados;
- qué rutas y comandos debe usar Codex en las fases posteriores.

## Contexto obligatorio a leer

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/contexto/CODEX_INDEX.yaml
codex/contexto/05_MAPA_PAQUETES.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2.md
codex/contexto/paquetes/**/00_summary.md
codex/herramientas/USO_HERRAMIENTAS.md
```

Leer además el código/launch/YAML final solo cuando la documentación vigente no
permita describir con precisión el estado real.

## Principio documental

Los MD deben describir el repositorio que existe después de 2A-2D, no el plan
previo ni una mezcla entre rutas antiguas y nuevas.

La evidencia cronológica pertenece al historial. Los resúmenes e índices deben
ser breves y suficientes para localizar el detalle.

No duplicar la misma explicación extensa en todos los archivos.

## Archivos obligatorios a revisar

### Arranque e índices

```text
AGENTS.md
codex/contexto/CODEX_INDEX.yaml
codex/contexto/00_BOOTSTRAP_MINIMO.md
codex/contexto/00_LEER_PRIMERO.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
```

Actualizar rutas de lectura, paquetes, fase activa y herramientas si han
cambiado. Mantener `AGENTS.md` compacto; no convertirlo en un manual largo.

### Estado y arquitectura

```text
codex/contexto/01_ESTADO_ACTUAL.md
codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md
codex/contexto/02_REGLAS_TECNICAS.md
codex/contexto/03_ARQUITECTURA_ACTUAL.md
codex/contexto/04_TOPICS_SERVICES_ACTIONS.md
codex/contexto/05_MAPA_PAQUETES.md
codex/contexto/05_MAPA_PAQUETES_SHORT.md
codex/contexto/06_MAPA_CODIGO.md
codex/contexto/07_ULTIMA_SESION.md
```

La arquitectura debe mostrar explícitamente:

```text
Dron
Servidor
Simulación
```

y las dependencias permitidas entre ellos.

### Pipeline

```text
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
codex/pipeline/fase_2_separacion_paquetes/subfases/*.md
```

Actualizar estados solo con evidencia. No marcar subfases realizadas por haber
escrito documentación.

Revisar referencias de fases posteriores que aún apunten a rutas antiguas. No
corregir las incongruencias de numeración internas de la Fase 3 salvo lo
estrictamente necesario para que una ruta exista; su renumeración corresponde a
la preparación específica de Fase 3.

### Herramientas

```text
codex/herramientas/build_selected_packages.sh
codex/herramientas/run_simulation.sh
codex/herramientas/reduce_build_log.sh
codex/herramientas/reduce_simulation_log.sh
codex/herramientas/split_simulation_log.sh
codex/herramientas/find_context.py
codex/herramientas/USO_HERRAMIENTAS.md
```

Documentar:

- bases de build por grupo;
- cómo seleccionar Dron, Servidor o Simulación;
- cómo cargar overlays;
- dónde quedan logs y reducidos;
- nueva ruta de escenarios;
- cómo evitar que se descubran ambas copias de `orbslam3_msgs`;
- estrategia alternativa si la principal no funciona.

## Documentación por paquete

Para cada paquete, revisar `00_summary.md` y los MD específicos.

### Contenido mínimo del resumen

Cada `00_summary.md` debe incluir:

```text
Grupo: dron | servidor | simulacion
Ruta física actual
Nombre ROS 2 declarado
Responsabilidad
Dependencias internas del grupo
Dependencias externas permitidas
Ejecutables/librerías/interfaces principales
Launch principales
YAML propietarios
YAML externos del mismo grupo que consume
Réplicas parciales que posee o consume
Restricciones de build
Prueba de validación vigente
```

### Documentación de YAML

Para cada YAML, documentar una tabla con:

| Campo | Contenido |
|---|---|
| Ruta | Ruta instalada/fuente |
| Propietario | Paquete y grupo |
| Propósito | Responsabilidad exacta |
| Consumidores | Nodos/launch |
| Parámetros importantes | Nombres y significado |
| Tipo de edición | protegido, ajustable, debug o contrato |
| Origen | local o réplica parcial |
| Claves replicadas | Si procede |
| Validación | Build/test que demuestra carga |

Ejemplos de política:

- `mass_total`: protegido; no modificar sin acuerdo explícito;
- `inertia_total`: protegido; no recalcular en Fase 2;
- `control.*.kp/kd`: ajustable solo por una subfase de control autorizada;
- umbrales de loops/fusión/optimización: ajustables solo por la subfase
  algorítmica correspondiente;
- `debug_*`: activables temporalmente para una prueba y devueltos a `false`;
- topics/frames/interfaces: contrato, no cambiar de forma casual.

### Dron

Actualizar al menos:

```text
codex/contexto/paquetes/dron_individual/00_summary.md
codex/contexto/paquetes/dron_individual/control.md
codex/contexto/paquetes/dron_individual/trayectorias.md
codex/contexto/paquetes/dron_individual/launches.md
codex/contexto/paquetes/lib_tray/00_summary.md
codex/contexto/paquetes/orbslam3_ros2/00_summary.md
codex/contexto/paquetes/orbslam3_ros2/stereo_slam_node.md
codex/contexto/paquetes/ORB_SLAM3/00_summary.md
codex/contexto/paquetes/orbslam3_msgs/00_summary.md
```

La documentación de `orbslam3_msgs` debe explicar que existen dos copias
intencionadas y que Servidor es canónica.

### Servidor

Actualizar al menos:

```text
codex/contexto/paquetes/orbslam3_multi/00_summary.md
codex/contexto/paquetes/orbslam3_multi/*.md
codex/contexto/paquetes/orbslam3_server/00_summary.md
codex/contexto/paquetes/orbslam3_server/global_map_server.md
codex/contexto/paquetes/orbslam3_server/launches.md
```

No repetir en cada componente toda la arquitectura de grupo. Enlazar al resumen
cuando sea suficiente.

### Simulación

Actualizar al menos:

```text
codex/contexto/paquetes/simulacion_dron/00_summary.md
codex/contexto/paquetes/simulacion_dron/launches.md
codex/contexto/paquetes/simulacion_dron/modelos.md
codex/contexto/paquetes/simulacion_dron/mundo_gazebo.md
codex/contexto/paquetes/simulacion_dron/scenario_runner_node.md
codex/contexto/paquetes/simulacion_dron/pipeline_flow_visualizer.md
```

Después de 2F, añadir documentación del nuevo `system_architecture`.

## Política permanente para fases posteriores

Añadir una regla explícita en el contexto técnico:

> Toda nueva configuración debe colocarse en el paquete y grupo que posea el
> dato. Un paquete puede cargar YAML de otro paquete del mismo grupo mediante el
> índice instalado. No puede cargar YAML de otro grupo. Si necesita una clave
> remota, crea una réplica parcial local con sufijo `_<grupo_origen>`, declara
> su origen y evita copiar claves no consumidas.

Añadir también:

> No se repiten datos dentro de un grupo. Los launch componen los YAML
> propietarios y entregan a cada nodo los parámetros necesarios.

Y:

> Todo flag de debug es independiente, está en Simulación cuando controla una
> herramienta de integración y queda `false` por defecto.

## Mapa de rutas

Documentar ejemplos claros:

```text
src/dron/dron_individual/...
src/dron/orbslam3_ros2/...
src/servidor/orbslam3_multi/...
src/servidor/orbslam3_server/...
src/simulacion/simulacion_dron/...
```

Actualizar cualquier línea aproximada o ruta de código en los MD de componentes
si el movimiento la ha invalidado.

## Comandos rápidos

Actualizar los comandos del índice para que expresen el grupo. Ejemplos
conceptuales:

```bash
./codex/herramientas/build_dron.sh <paquetes>
./codex/herramientas/build_servidor.sh <paquetes>
./codex/herramientas/build_simulacion.sh <paquetes>
./codex/herramientas/run_simulation.sh --prueba <id> --launch "ros2 launch simulacion_dron multi_dron.launch.py"
```

Si se conserva `build_selected_packages.sh`, debe exigir o inferir el grupo sin
exponer ambos `orbslam3_msgs`.

## Escenarios y evidencia

Actualizar referencias desde la ruta antigua de escenarios hacia:

```text
src/simulacion/simulacion_dron/config/scenarios/
```

o su ruta instalada.

La documentación debe distinguir:

- escenario fuente runtime;
- log completo generado;
- log reducido;
- historial de prueba;
- capturas o artefactos visuales.

No reescribir historiales antiguos para fingir que las pruebas históricas ya
usaban la nueva ruta. Añadir una nota de correspondencia cuando sea necesario.

## Revisión automática de documentación

Ejecutar:

- búsqueda de rutas antiguas;
- búsqueda de `fase_4_separacion` o numeración antigua donde ya no sea válida;
- comprobación de enlaces Markdown;
- comprobación de longitud de documentos;
- `git diff --check`;
- herramienta de búsqueda de contexto sobre cada paquete;
- verificación de que todos los nuevos YAML aparecen en al menos un MD.

Patrones orientativos:

```bash
rg -n "src/(dron_individual|orbslam3_server|simulacion_dron)" src/codex
rg -n "install/setup.bash|build_selected_packages" src/codex
rg -n "hardware.yaml|tray_dron.yaml|sim_dron.yaml" src/codex
rg -n "orbslam3_msgs" src/codex/contexto src/codex/pipeline
```

No sustituir automáticamente toda aparición histórica. Distinguir documentos
vigentes de evidencia antigua.

## Cambios prohibidos

- No borrar historiales anteriores.
- No cambiar conclusiones pasadas por el movimiento de rutas.
- No describir builds o pruebas que no se ejecutaron.
- No mantener rutas antiguas como vigentes “por compatibilidad documental”.
- No duplicar el mismo manual completo en todos los paquetes.
- No ocultar la duplicación intencionada de `orbslam3_msgs`.
- No afirmar que una interfaz es compatible sin la guarda/prueba correspondiente.
- No cambiar numeración interna de Fase 3 fuera de su trabajo específico.

## Verificación funcional de la documentación

Un nuevo chat de Codex debe poder responder, usando primero los MD:

1. ¿Dónde está `orbslam3_server`?
2. ¿Cómo se compila Servidor sin Dron?
3. ¿Por qué hay dos `orbslam3_msgs`?
4. ¿Cuál se modifica primero?
5. ¿Dónde está `mass_total`?
6. ¿Qué YAML puede modificar Codex para activar RViz?
7. ¿Dónde está el número de drones?
8. ¿Cómo carga Simulación parámetros físicos del dron?
9. ¿Dónde está el escenario de vuelta al edificio?
10. ¿Cómo se abre el diagrama arquitectónico?

Si la respuesta exige abrir muchos archivos de código, la documentación no está
completa.

## Criterio de éxito

`CONSEGUIDA` solo si:

1. todos los índices y rutas vigentes coinciden con el repositorio;
2. cada paquete tiene grupo y ruta documentados;
3. cada YAML tiene propósito, consumidores y política de edición;
4. la regla de réplicas parciales está documentada como permanente;
5. la copia canónica de `orbslam3_msgs` está clara;
6. los comandos de build/simulación reflejan los tres grupos;
7. los escenarios y logs se distinguen correctamente;
8. no quedan rutas antiguas activas en documentación vigente;
9. los historiales se conservan;
10. Codex puede localizar el contexto barato sin abrir código.

## Criterio de parcial, fallo o bloqueo

`PARCIAL` si la documentación principal está actualizada pero quedan MD de
detalle con rutas antiguas claramente indexados como pendientes.

`NO CONSEGUIDA` si `AGENTS.md`, índices o resúmenes conducen a rutas inexistentes,
si los YAML no tienen política o si se oculta la dependencia de Simulación.

`BLOQUEADA` solo si un paquete no está disponible y no puede documentarse más
allá de su contrato de ruta.

## Documentación de cierre

Esta subfase modifica la propia documentación y debe además crear/actualizar al
ejecutarse:

```text
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2E.md
codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/historial_2E_RESUMEN.md
codex/contexto/07_ULTIMA_SESION.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
```

El cierre debe indicar expresamente si queda alguna ruta documental pendiente.
