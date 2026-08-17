---
name: actualizar-documentacion
description: Actualiza documentación de contexto, paquetes, historial y estado actual tras build, simulación o revisión conversada de resultados, manteniendo una conclusión por prueba y resúmenes agregados sin ocultar intentos anteriores.
---

Usar al final de una fase, tras una prueba fallida que deba quedar registrada o
cuando se reorganice documentación.

`00_CONTEXTO_COMPACTACION.md` es una excepción operativa: no debe esperar a esta
skill para actualizarse. El agente que planifica, modifica, compila, simula o
diagnostica reemplaza su checkpoint inmediatamente después de cada hito. Esta
skill verifica la coherencia de esos checkpoints y realiza el cierre.

Agente recomendado:

- `curador_documentacion`

## Principio antitokens

Mantener dos capas:

```text
resumen corto -> detalle bajo demanda
```

No copiar la misma evidencia larga en varios MDs. Si un archivo crece demasiado,
crear o actualizar un resumen/índice y mover la evidencia detallada al historial
por subfase o a un archivo de detalle.

Leer antes de documentar:

```text
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/pipeline/fase_3_sparse_global/historial/INDEX.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_<ID>_RESUMEN.md
```

Abrir archivos largos solo si falta información.

## Debe recibir

- resumen de cambios del implementador;
- resultado del build;
- análisis de logs;
- fase/subfase actual;
- historial específico reciente.

## Debe actualizar cuando aplique

- `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`
- `codex/contexto/01_ESTADO_ACTUAL.md`
- `codex/contexto/00_CONTEXTO_COMPACTACION.md`
- `codex/contexto/07_ULTIMA_SESION.md`
- `codex/contexto/paquetes/<paquete>/<paquete>.md`
- `codex/contexto/paquetes/<paquete>/<componente>.md`
- `codex/pipeline/fase_*/historial/INDEX.md`
- `codex/pipeline/fase_*/historial/por_subfase/historial_<ID>_RESUMEN.md`
- `codex/pipeline/fase_*/historial/por_subfase/historial_<ID>.md`
- `codex/pipeline/fase_*/subfases/subfase_*.md`

No recrear un historial completo monolítico. El detalle debe vivir en el
historial por subfase o tema.

## Dividir historiales largos

Si un archivo `historial_<ID>.md` crece demasiado para leerse eficientemente
(por ejemplo, más de 1200-1500 líneas o varias entradas largas), dividirlo en
subhistoriales numerados:

- conservar `historial_<ID>.md` como índice breve y resumen de los subarchivos;
- crear `historial_<ID>_1.md`, `historial_<ID>_2.md`, ... con cortes cronológicos
  claros y entradas completas;
- el archivo principal debe explicar brevemente qué contiene cada subarchivo,
  incluyendo el rango de fechas y el tipo de contenido cubierto;
- usar esta regla para evitar archivos monolíticos y mantener la consulta rápida.

## Dividir especificaciones de subfase

Si un archivo `subfase_<ID>.md` crece demasiado (más de 1000 líneas), dividirlo
en 4 subarchivos temáticos:

- conservar `subfase_<ID>.md` como índice breve que lista los 4 subarchivos;
- crear `subfase_<ID>_especificacion.md`: Estado, Objetivo, Contexto, Diagnóstico, 
  Archivos permitidos/prohibidos (~180-200 líneas);
- crear `subfase_<ID>_implementacion.md`: Cambios requeridos/prohibidos, Funciones/clases 
  a localizar (~400-800 líneas, la más grande);
- crear `subfase_<ID>_testing.md`: Paquetes a compilar, Pruebas Gazebo/replay, Patrones 
  de reducción, Marcadores técnicos (~180-220 líneas);
- crear `subfase_<ID>_criterios.md`: Criterios de éxito/fallo, Documentación a actualizar, 
  Notas de diseño (~130-190 líneas);

El archivo principal debe ser muy breve (5-10 líneas) con título, descripción 
mínima y lista de subarchivos con descripción de contenido.

Si una conversación preparatoria con el usuario aclara cómo debe ejecutarse una
subfase, actualizar el contrato de subfase para reflejar lo acordado. Ese cambio
documental no autoriza por sí mismo cambios funcionales: código, launch, YAML y
configuración se modifican solo tras confirmación explícita de ejecución.

Si en esa conversación se acuerda qué prueba ejecutar tras la modificación,
reflejarlo en el contrato de subfase, historial o resumen correspondiente para
que la validación no quede implícita ni se pierda tras compactaciones.

Si la preparación queda cerrada pero la ejecución se aplaza, no cerrar
`00_CONTEXTO_COMPACTACION.md` como `sin trabajo activo`. Conservar objetivo,
alcance, exclusiones, riesgos, prueba y criterios, junto con:

```text
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: PENDIENTE
Dudas abiertas: ninguna
```

Así el siguiente chat puede omitir preguntas ya resueltas, pero no ejecutar
hasta recibir una orden posterior.

## Documentación obligatoria de paquetes

Si se modifica cualquier archivo de un paquete, actualizar la documentación
correspondiente en `codex/contexto/paquetes/`.

La actualización debe describir el estado actual del código tras la modificación,
no solo añadir una nota cronológica.

Antes de abrir código o scripts para documentarlos, consultar primero
`codex/contexto/paquetes/<paquete>/00_summary.md` y el MD del componente. Si esa
documentación no basta y se lee código, completar el MD del componente para que
la próxima consulta no tenga que releer el código.

Revisar, según aplique:

- archivo general del paquete;
- archivo específico del componente tocado;
- lista de archivos relevantes;
- funciones, clases o nodos principales;
- para cada zona importante, ruta, símbolo y patrón estable de búsqueda con
  `rg`;
- líneas aproximadas de funciones o zonas de edición importantes, tratadas
  únicamente como orientación;
- topics, services, actions y parámetros afectados;
- logs o marcadores usados para validar;
- restricciones técnicas vigentes.

No usar un número de línea exacto como referencia principal. El formato
preferido es:

```text
Archivo -> Símbolo -> Localización con rg -> Líneas aproximadas
```

Un desplazamiento de líneas no obliga a revisar otros MDs. Actualizar la
referencia cuando cambie el símbolo, su responsabilidad o su archivo, y cuando
el componente vuelva a consultarse o modificarse.

La evidencia larga de ejecución va al historial por subfase, no al `.md` del
paquete salvo que sea necesaria para entender el estado vigente.

## Historial por subfase

Rutas:

```text
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_<ID>_RESUMEN.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_<ID>.md
```

Leer primero `historial_<ID>_RESUMEN.md`. Abrir el historial largo o fragmentos
solo si falta evidencia concreta.

Formato mínimo:

```text
## fecha/hora — Subfase <ID> — título corto

- objetivo intentado:
- archivos modificados:
- paquetes compilados:
- resultado de build:
- pruebas Gazebo/replay:
- patrones usados para reducir logs:
- evidencia positiva:
- evidencia negativa o ausente:
- conclusión: CONSEGUIDA / NO CONSEGUIDA / PARCIAL / BLOQUEADA
- siguiente paso recomendado:
```

Después de añadir una entrada:

- actualizar `historial_<ID>_RESUMEN.md` con la consecuencia operativa: estado,
  evidencia clave, aprendizaje, pendiente o estado bueno conocido;
- actualizar `historial/INDEX.md` con una línea corta;
- actualizar `01_ESTADO_ACTUAL_RESUMEN.md` si cambia la subfase o conclusión;
- reemplazar `07_ULTIMA_SESION.md` con un resumen breve de la última sesión.

No añadir sesiones antiguas debajo ni crear copias detalladas. El detalle
permanente debe quedar en el historial por subfase.

## Conclusiones revisables por prueba

Aplicar también esta skill cuando el usuario siga hablando de una prueba ya
documentada y la conversación cambie materialmente su interpretación.

- Cada ejecución real tiene su propia entrada y conclusión, tanto si termina
  `CONSEGUIDA` como `PARCIAL`, `NO CONSEGUIDA` o `BLOQUEADA`.
- Una nueva modificación y una nueva ejecución añaden otra entrada cronológica.
  No borrar ni convertir retroactivamente una prueba fallida en una prueba
  correcta.
- Una conversación sobre la misma ejecución no crea otra prueba. Modificar la
  entrada existente para corregir o ampliar su análisis y conclusión.
- Conservar siempre la evidencia objetiva. Si el primer análisis era erróneo,
  indicar la interpretación revisada sin ocultar qué ocurrió realmente.
- Una subfase puede acumular varias conclusiones de prueba. Su
  `historial_<ID>_RESUMEN.md` expresa la conclusión agregada vigente y referencia
  los fallos o limitaciones que siguen siendo relevantes.
- Actualizar automáticamente, sin esperar una petición documental, cuando la
  conversación produzca una interpretación estable distinta de la registrada.
  Hacerlo en el mismo turno en que Codex reconoce esa corrección y, como máximo,
  antes de cambiar de prueba, subfase o tema.
- Sincronizar `INDEX.md`, estado, pipeline, `07_ULTIMA_SESION.md` y docs de
  paquete solo cuando la revisión cambie lo que esos archivos afirman.

## Logs y sublogs

Si un log reducido sigue siendo grande, crear sublogs específicos antes de
analizarlo en detalle:

```bash
./codex/herramientas/split_simulation_log.sh --prueba X --fase 3L
```

Ver guía:

```text
codex/contexto/09_LOGS_Y_SUBLOGS.md
```

El log completo se conserva, pero no se lee directamente. Si falta un marcador
obligatorio, regenerar el reducido con ese patrón o crear otro sublog antes de
concluir que no existe.

## Reglas

- No inventar resultados no comprobados.
- Comprobar que `00_CONTEXTO_COMPACTACION.md` fue actualizado durante los hitos,
  no solo al cierre; dejarlo como `sin trabajo activo` si no queda pendiente.
- Tras una compactación, releer físicamente esa memoria antes de continuar. El
  resumen automático del chat no sustituye la lectura.
- Si una prueba no se ejecutó, decirlo.
- Si RViz2 no se observó, decirlo.
- No borrar historial anterior de subfase; si un archivo se vuelve redundante,
  eliminarlo solo si su información ya está integrada en índices/historiales por
  subfase.
- No modificar código.
- No marcar una subfase como `realizado` si no hay evidencia suficiente.
- No duplicar grandes bloques entre estado, última sesión, paquete e historial.
