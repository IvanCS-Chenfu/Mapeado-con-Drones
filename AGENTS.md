# AGENTS.md — Arranque compacto para Codex

Este archivo es el punto de entrada ligero del proyecto. No existe una copia
larga activa de `AGENTS.md`: si falta una regla, buscarla en los documentos
específicos de `codex/contexto/`, `codex/pipeline/` o `codex/contexto/decisiones/`.

## Puerta de continuidad obligatoria

Antes de leer otro contexto del proyecto:

1. Leer físicamente `codex/contexto/00_CONTEXTO_COMPACTACION.md` mediante una
   herramienta.
2. Si contiene trabajo activo, reconciliarlo con la petición más reciente del
   usuario antes de actuar.
3. No considerar el resumen automático del chat como sustituto de esa lectura.

En tareas largas, subfases o trabajos con build/simulación, actualizar ese
archivo tras fijar el plan, localizar archivos críticos, completar cambios,
terminar cada build/test/simulación o diagnóstico y antes de una acción larga.
Tras una compactación, releerlo antes de cualquier otra acción y registrar el
checkpoint de reanudación. Al cerrar, dejar explícitamente si queda trabajo
activo.

<!-- CODEX_INDEX_START
codex_index:
  read_order:
    - codex/contexto/00_CONTEXTO_COMPACTACION.md
    - codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
    - codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
    - codex/pipeline/PIPELINE_MAESTRO.md
    - codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
  package_map: codex/contexto/05_MAPA_PAQUETES.md
  packages_dir: codex/contexto/paquetes/
  subfases_dir: codex/pipeline/fase_2_separacion_paquetes/subfases/
  historial_dir: codex/pipeline/fase_2_separacion_paquetes/historial/por_subfase/
  history_summary_suffix: _RESUMEN.md
  tools_dir: codex/herramientas/
  short_summary_name: 00_summary.md
  long_doc_tag: HISTORY: true
quick_commands:
  - "rg -n 'pattern' . --hidden --glob '!build'"
  - "./codex/herramientas/build_selected_packages.sh --group <grupo> <paquete>"
  - "./codex/herramientas/run_simulation.sh --prueba X --launch 'ros2 launch simulacion_dron multi_dron.launch.py'"
CODEX_INDEX_END -->

Regla general: leer primero los resúmenes e índices. Abrir documentos largos
solo cuando falte información concreta para implementar, compilar, simular o
documentar.

## Idioma

- Responder siempre en español.
- Documentar siempre en español.
- Mantener en inglés nombres de paquetes, archivos, clases, funciones,
  variables, topics, services, actions, parámetros ROS, logs y comandos.

## Alcance editable

- Modificar solo archivos dentro de `src/`.
- No modificar manualmente `build/`, `install/` ni `log/`.
- No tocar `ORB_SLAM3`, `orbslam3_ros2` ni `orbslam3_msgs` salvo necesidad
  explícita y justificada.
- No revertir cambios del usuario.

Permisos operativos preaprobados:

- ejecutar `./codex/herramientas/build_selected_packages.sh`;
- ejecutar `./codex/herramientas/run_simulation.sh`;
- limpiar artefactos generados mínimos dentro de `build/`, `install/` o `log`
  si bloquean build/simulación.

Si el sandbox exige escalado para esas herramientas, pedirlo directamente en la
llamada de herramienta.

## Lectura de contexto barata

Antes de actuar, leer en este orden:

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/pipeline/PIPELINE_MAESTRO.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
```

Después, según la tarea:

- compactación/reanudación: tras una compactación del chat, releer físicamente
  `codex/contexto/00_CONTEXTO_COMPACTACION.md` antes de cualquier otra acción,
  reconciliarlo con la última petición y registrar el checkpoint; el resumen
  automático no sustituye esta lectura;
- subfase actual: leer solo `subfase_<ID>.md`;
- historial: leer primero `historial/INDEX.md` y luego
  `historial/por_subfase/historial_<ID>_RESUMEN.md`; abrir
  `historial_<ID>.md` o fragmentos largos solo si falta evidencia concreta;
- paquetes: usar primero `find-context` o `codex/herramientas/find_context.py` para localizar `codex/contexto/paquetes/<paquete>/00_summary.md` y luego abrir solo los MDs necesarios;
- código/scripts: no abrir código como primera fuente de información; consultar
  antes el MD vigente del componente en `codex/contexto/paquetes/` y localizar
  allí la responsabilidad, el símbolo y el patrón estable de búsqueda; saltar
  al código solo si el MD no basta, está obsoleto o hay que inspeccionar el
  fragmento exacto que se va a modificar;
- lectura de código: localizar primero el símbolo o patrón con `rg`, abrir solo
  el rango mínimo necesario y no leer el archivo completo por defecto; si esa
  lectura aporta información que faltaba, completar el MD del componente en el
  mismo turno;
- logs: el log completo solo puede ser entrada de herramientas de reducción;
  Codex nunca lo abre ni vuelca su contenido al contexto. Antes de leer debe
  generar un reducido o sublog temático y leer únicamente ese artefacto. Si
  falta evidencia, vuelve a reducir con patrones nuevos o más precisos.

Abrir documentos largos como `01_ESTADO_ACTUAL.md` o historiales por subfase
solo si el resumen no basta.

## Estado técnico clave

- Objetivo global: nube densa global usando varios drones, sin ground truth para
  mapa final ni pose final.
- Fase activa: cierre conversacional de Fase 2; implementación técnica conseguida.
- Subfase activa actual: consultar `01_ESTADO_ACTUAL_RESUMEN.md`.
- No iniciar otra fase hasta que el usuario confirme la revisión visual de la
  prueba 200 o pida explícitamente cambiar de fase.
- Las subfases `12R-*`, `13`, `14` y `15` son legacy.

Invariantes permanentes:

- `submapa = (drone_id, map_epoch)`;
- `RawMapDatabase` conserva datos ORB-SLAM3 crudos y no se modifica por
  optimización;
- `GlobalPoseStore` conserva anchors, poses globales, poses optimizadas,
  propagadas y rollback;
- los fiduciales son observaciones absolutas, no loops;
- el ground truth solo puede usarse para fiducial simulado, debug o métricas
  externas;
- la nube global publicada debe tender a fused tracks, no unión bruta;
- no aceptar optimizaciones que muevan hard fiducials o rompan invariantes.

## Flujo si el usuario pide ejecutar una subfase

No tratar `subfase_<ID>.md` como especificación completa. Los MDs de subfase
pueden estar incompletos o no explicar todos los matices que el usuario quiere.

### Orden inicial no es autorización

Una primera petición como `haz la subfase X`, `ejecútala`, `realízala completa`
o equivalente nunca autoriza ejecución inmediata. Codex debe detenerse y entrar
en preparación, aunque la orden parezca explícita.

Durante preparación no se modifica código, launch, YAML ni configuración, y no
se inicia build, test o simulación. Codex debe:

1. Leer contexto mínimo, contrato, `historial_<ID>_RESUMEN.md` y docs de los
   paquetes afectados.
2. Explicar objetivo, comportamiento esperado, alcance, exclusiones, archivos
   probables, riesgos, alternativas, prueba y criterios de éxito/fallo.
3. Preguntar lo necesario para comprobar que Codex entiende exactamente lo que
   quiere el usuario y que el usuario entiende y acepta lo que se hará.
4. Cerrar un resumen de acuerdo sin dudas abiertas y pedir confirmación
   explícita antes de ejecutar.

### Excepción por acuerdo previo

No repetir preguntas ya resueltas si la subfase se debatió previamente y existe
un acuerdo completo en la conversación actual o en
`00_CONTEXTO_COMPACTACION.md`. Para considerar el acuerdo cerrado deben constar:
objetivo, comportamiento, alcance, exclusiones, prueba, criterios, riesgos
aceptados y `Dudas abiertas: ninguna`.

Un MD de subfase, historial o resumen automático no demuestra por sí solo que el
usuario conozca y acepte ese acuerdo. Si Codex no puede verificar que la
preparación quedó cerrada, debe preguntar. Con acuerdo previo verificable, una
orden posterior como `ejecútalo según lo hablado` sí sirve como autorización.

Estados obligatorios en `00_CONTEXTO_COMPACTACION.md`:

```text
Preparacion: NO_INICIADA | EN_DEBATE | CERRADA
Acuerdo cerrado: no | si
Autorizacion funcional: PENDIENTE | CONCEDIDA | SUSPENDIDA
Prueba acordada: <prueba o decision explicita de no probar>
Dudas abiertas: <lista o ninguna>
```

Solo tras esa autorización:

1. Registrar en `00_CONTEXTO_COMPACTACION.md` objetivo, autorización, plan y
   siguiente acción exacta.
2. Modificar lo necesario y registrar el bloque de cambios completado.
3. Compilar paquetes seleccionados con builds pequeños si hay paquetes pesados;
   registrar inmediatamente el resultado.
4. Si falla build, ejecutar `reduce_build_log.sh`, diagnosticar el primer error
   real, registrar el diagnóstico y corregir.
5. Antes de una simulación larga, registrar la prueba que se va a ejecutar.
6. Ejecutar las pruebas Gazebo/replay definidas y registrar cada resultado antes
   de analizar o repetir.
7. Reducir logs antes de leerlos; crear sublogs específicos si el reducido
   sigue siendo grande y volver a reducir si falta evidencia. Nunca abrir el
   log completo.
8. Analizar evidencia contra el criterio de éxito y registrar la conclusión.
9. Actualizar documentación compacta, docs de paquete, historial por subfase y
   `historial_<ID>_RESUMEN.md`.
10. Cerrar `00_CONTEXTO_COMPACTACION.md` de forma coherente y responder con
    `CONSEGUIDA`, `NO CONSEGUIDA`, `PARCIAL` o `BLOQUEADA`.

La autorización queda limitada al acuerdo confirmado. Si aparece una duda
funcional, alternativa material, decisión no acordada, cambio de alcance,
prueba o criterio, Codex debe parar antes de actuar, registrar
`Autorizacion funcional: SUSPENDIDA`, explicar la duda y conversar con el
usuario. Solo puede continuar después de cerrar el nuevo acuerdo y recibir una
indicación explícita equivalente a `sigue de esta manera`.

Una corrección puramente mecánica que conserva exactamente el acuerdo, como
corregir una ruta o un error tipográfico, no exige nueva autorización, pero debe
registrarse. Si no está claro si la corrección es mecánica o funcional, parar y
preguntar.

No declarar `CONSEGUIDA` sin build, simulación/logs y documentación coherente.

### Conclusiones vivas por prueba

El cierre técnico no congela la interpretación de una prueba si el usuario
continúa hablando de sus resultados.

- Cada ejecución real, pase o falle, crea una entrada cronológica propia con su
  evidencia y conclusión. Una prueba fallida nunca se borra ni se reescribe
  como si hubiera pasado.
- Una nueva modificación seguida de otra ejecución crea otra entrada; no
  sustituye la prueba anterior. Una subfase puede tener varias pruebas y varias
  conclusiones parciales antes de su conclusión agregada.
- Si la conversación posterior corrige o amplía materialmente la interpretación
  de una prueba ya registrada, Codex debe actualizar directamente la entrada de
  esa misma prueba y su conclusión, sin esperar una orden documental.
- Revisar una conclusión no significa borrar evidencia medida. Conservar el
  resultado objetivo y dejar clara la corrección de conteo, clasificación,
  causalidad o alcance que surgió al hablar con el usuario.
- Cuando la interpretación quede estable, sincronizar
  `historial_<ID>_RESUMEN.md` y, solo si resultan afectados, `INDEX.md`, estado,
  pipeline, última sesión y docs de paquete.
- No cambiar de prueba, subfase o tema ni cerrar una conversación de resultados
  dejando el historial con el análisis anterior.

## Documentación y tokens

Al modificar documentación:

- actualizar primero resúmenes e índices;
- no duplicar la misma narrativa en muchos archivos;
- poner evidencia larga en el historial de la subfase, no en todos los MDs;
- mantener `codex/pipeline/fase_*/subfases/subfase_<ID>.md`
  como contrato ejecutable corto: qué se debe hacer, límites, pruebas y
  criterios; no usarlo como diario de pruebas ni como historial técnico largo;
- mantener `01_ESTADO_ACTUAL_RESUMEN.md` corto;
- mantener `00_CONTEXTO_COMPACTACION.md` como memoria viva breve; actualizarlo
  en hitos de tareas largas y tras compactaciones, reemplazando su estado
  operativo en vez de acumular diario;
- reemplazar `07_ULTIMA_SESION.md` en cada cierre; no añadir texto acumulado al
  final;
- guardar el detalle anterior solo en el historial por subfase;
- mantener `historial_<ID>_RESUMEN.md` como lectura histórica barata de la
  subfase: que se hizo, que salio bien, que salio mal, evidencia vigente y que
  no repetir;
- si un archivo supera mucho las 250 líneas, crear índice/resumen y mover detalle
  a archivo específico o de archivo;
- para historial nuevo, escribir en `historial/por_subfase/historial_<ID>.md` y
  actualizar `historial/por_subfase/historial_<ID>_RESUMEN.md` y
  `historial/INDEX.md`.
- distinguir siempre entre:
  - nueva ejecución: añadir una entrada cronológica;
  - nueva conversación sobre una ejecución existente: modificar su análisis y
    conclusión cuando cambie la interpretación;
  - resumen de subfase: reflejar el estado agregado vigente sin ocultar intentos
    anteriores.

Reglas detalladas:

```text
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
```

## Herramientas principales

```bash
./codex/herramientas/build_selected_packages.sh --group <grupo> <paquete>
./codex/herramientas/reduce_build_log.sh
./codex/herramientas/run_simulation.sh --prueba X --launch "ros2 launch simulacion_dron multi_dron.launch.py"
./codex/herramientas/reduce_simulation_log.sh --prueba X --patterns "<patrones>"
./codex/herramientas/split_simulation_log.sh --prueba X --fase 3L
```

El log completo se conserva como artefacto, pero nunca se lee directamente. Es
solo la entrada de `reduce_*`/`split_*`. No usar `cat`, `sed`, `head`, `tail`,
`less`, `open` ni mostrar directamente la salida de `rg` sobre `*.log`
completos. Tampoco usar una
excepción de "falta un marcador": ampliar patrones, regenerar el reducido y
leer el nuevo reducido.

## Documentación obligatoria tras código

Si se modifica código, launch, configuración o scripts de un paquete, actualizar
también la documentación correspondiente en:

```text
codex/contexto/paquetes/
```

La documentación de paquete debe describir el estado actual del código, no solo
añadir una nota histórica.

Si para entender o modificar una pieza falta información en el MD del componente,
Codex puede leer el código necesario, pero debe completar ese MD con el estado
actual, funciones/clases importantes, relaciones y líneas aproximadas para que la
siguiente consulta no tenga que volver a abrir el código.

Las referencias de cada zona importante deben priorizar este orden:

```text
ruta del archivo -> clase/función/símbolo -> patrón estable para rg -> líneas aproximadas
```

Las líneas son orientativas y no se mantienen como referencias exactas ante
simples desplazamientos. Si se modifica un símbolo, su responsabilidad o su
archivo, se actualiza la referencia del MD afectado; no se revisan todos los MDs
solo porque hayan cambiado números de línea.

## Detalle bajo demanda

La documentación extensa queda distribuida como detalle. Usarla de forma
selectiva:

```text
codex/contexto/01_ESTADO_ACTUAL.md
codex/pipeline/fase_*/historial/por_subfase/
codex/contexto/paquetes/**/<componente>.md
```
