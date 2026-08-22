# 08 — Política de documentación compacta y ahorro de tokens

Objetivo: que Codex conserve todo el contexto importante sin leer miles de
líneas en cada chat.

## Regla principal

Cada tema debe tener dos capas:

```text
resumen corto -> detalle bajo demanda
```

Los resúmenes orientan. Los detalles conservan evidencia.

## Puerta de continuidad

`00_CONTEXTO_COMPACTACION.md` es memoria operativa, no historial. Su lectura
física mediante una herramienta es obligatoria antes del resto del contexto en
todo chat nuevo y justo después de una compactación. El resumen automático del
chat no sustituye esa lectura.

Si contiene trabajo activo, Codex debe reconciliarlo con la petición más
reciente antes de actuar. En tareas largas, subfases o trabajos con
build/simulación, debe reemplazar su checkpoint:

- después de fijar objetivo y plan;
- después de localizar archivos críticos;
- después de cada bloque de cambios;
- inmediatamente después de cada build, test, simulación o diagnóstico;
- antes de iniciar una acción larga;
- después de una compactación y antes de continuar;
- al cierre, indicando si queda trabajo activo.

Actualizar un checkpoint significa editar realmente el archivo. No basta con
recordarlo en el chat, anunciar una actualización futura ni delegarlo al cierre
documental.

## Orden de lectura para futuros chats

1. `AGENTS.md`
2. `00_CONTEXTO_COMPACTACION.md`
3. `CONTEXTO_MINIMO_ACTUAL.md`
4. índice o resumen relevante
5. detalle solo si falta información

No abrir `01_ESTADO_ACTUAL.md` ni historiales de otras subfases por costumbre.
Los logs completos no se abren nunca; solo alimentan reductores.

## Tamaños recomendados

| Tipo de archivo | Tamaño recomendado |
|---|---|
| Bootstrap/resumen | 80-180 líneas |
| Subfase ejecutable | 80-180 líneas |
| Índice | 50-150 líneas |
| Docs de componente | 150-300 líneas si es posible |
| Historial por subfase | Puede ser largo, pero se lee solo cuando toca |
| Archivo completo/legacy | Sin límite, pero no debe ser lectura obligatoria |

Si un `.md` crece demasiado, crear un índice/resumen y mover la evidencia larga a
un archivo específico.

## Historial

No acumular todo en un solo archivo.

Usar:

```text
codex/pipeline/fase_3_sparse_global/historial/INDEX.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_<ID>_RESUMEN.md
codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_<ID>.md
```

`historial_<ID>_RESUMEN.md` es la primera lectura cuando el usuario pida opinar,
argumentar, modificar o recuperar algo de una subfase. Debe resumir que se hizo,
que funciono, que fallo, evidencia vigente, decisiones importantes y que no
conviene repetir. El historial largo conserva la evidencia cronologica.

Una entrada nueva va al historial largo de su subfase. El resumen de esa subfase
se actualiza con la consecuencia operativa, y `INDEX.md` recibe solo una linea o
tabla corta.

## Archivos de subfase

Los archivos de:

```text
codex/pipeline/fase_3_sparse_global/subfases/subfase_<ID>.md
```

son contratos ejecutables, no historiales.

Subfases grandes de Fase 3 (`3O`, `3P`, `3Q`, `3R`) se dividen en subarchivos
temáticos cuando aplica:

- `subfase_<ID>.md`: índice breve (5-10 líneas) que lista los 4 subarchivos.
- `subfase_<ID>_especificacion.md`: Estado, Objetivo, Contexto, Diagnóstico, 
  Archivos permitidos/prohibidos (~180-200 líneas).
- `subfase_<ID>_implementacion.md`: Cambios requeridos/prohibidos, Funciones/clases 
  a localizar (~400-800 líneas).
- `subfase_<ID>_testing.md`: Paquetes a compilar, Pruebas, Patrones, Marcadores 
  (~180-220 líneas).
- `subfase_<ID>_criterios.md`: Criterios de éxito/fallo, Documentación a actualizar, 
  Notas de diseño (~130-190 líneas).

Leer siempre primero el índice principal. Abrir subarchivos solo según necesidad.

Subfases más cortas mantienen un solo archivo `subfase_<ID>.md` bajo 250 líneas.

Deben contener:

- objetivo de la subfase;
- propiedad funcional clara;
- archivos permitidos/prohibidos;
- paquetes a compilar;
- pruebas y logs obligatorios;
- criterios de éxito/fallo.

No deben contener:

- narrativas largas de intentos;
- listas extensas de HTMLs/logs históricos;
- resultados de muchas pruebas descartadas;
- discusiones exploratorias que ya quedaron en el historial.

Si un archivo de subfase supera mucho las 250 líneas, mover el detalle a
`historial/por_subfase/historial_<ID>.md` o a un MD de detalle referenciado, y
dejar en la subfase solo el contrato vigente.

## Protocolo de ejecución de subfases

Los MDs de subfase son contratos iniciales, no especificaciones perfectas. Cuando
el usuario pida ejecutar una subfase o hacer cambios funcionales dentro de una
subfase, Codex debe separar preparación y ejecución.

Una primera orden directa nunca es autorización funcional suficiente, aunque
diga `haz`, `ejecuta` o `realiza`. Codex debe detenerse y preparar antes de
modificar código/configuración o iniciar build, test o simulación.

Preparación obligatoria:

1. leer el contrato de subfase, `historial_<ID>_RESUMEN.md` y docs de paquete;
2. explicar al usuario qué entiende: objetivo, alcance, archivos probables,
   exclusiones, pruebas, riesgos, alternativas, criterios y ambigüedades;
3. preguntar o confirmar qué prueba quiere el usuario ejecutar tras la
   modificación, incluso si la subfase ya propone una prueba por defecto;
4. comprobar que Codex ha entendido y que el usuario conoce y acepta el acuerdo;
5. presentar un resumen final y cerrar todas las dudas;
6. no modificar código, launch, YAML ni configuración durante esta preparación.

No repetir preguntas si la conversación actual o
`00_CONTEXTO_COMPACTACION.md` contiene un acuerdo previo completo y confirmado:
objetivo, comportamiento, alcance, exclusiones, prueba, criterios, riesgos
aceptados y ninguna duda abierta. Un MD de subfase, historial o resumen
automático no prueba por sí solo la aceptación del usuario.

Ejecución autorizada:

- iniciar cambios funcionales solo cuando, después de cerrar la preparación, el
  usuario confirme explícitamente con
  una frase equivalente a `TODO LISTO, REALIZA LA SUBFASE`,
  `con todo lo hablado, ejecútalo` o `me parece bien, sigue`;
- si ya existe acuerdo previo verificable, una orden posterior puede ser esa
  autorización sin repetir el debate;
- limitar la autorización al acuerdo confirmado;
- si durante la ejecución aparece una duda funcional, alternativa material,
  decisión no acordada o zona mal especificada, suspender la autorización,
  parar y preguntar;
- si no hay prueba acordada o el alcance cambia y conviene otra prueba, parar y
  preguntar antes de elegirla por cuenta propia;
- continuar solo tras cerrar el acuerdo revisado y recibir confirmación, por ejemplo
  `ME PARECE BIEN LO DICHO, SIGUE CON LA SUBFASE SEGÚN LO HABLADO`.

Una corrección mecánica que conserva exactamente el acuerdo puede continuar. Si
no está claro si una corrección es mecánica o funcional, parar y preguntar.

Tras la conversación preparatoria, Codex puede actualizar documentación de
subfase para reflejar lo acordado. Esta regla de aprobación estricta aplica a
cambios funcionales; los cambios documentales solicitados explícitamente por el
usuario pueden hacerse directamente.

Si el debate queda cerrado y la ejecución se aplaza, conservar en
`00_CONTEXTO_COMPACTACION.md` la preparación cerrada, el acuerdo completo, la
prueba, las dudas y la autorización pendiente para que el siguiente chat no
repita preguntas ni ejecute sin permiso.

## Estado actual

`01_ESTADO_ACTUAL_RESUMEN.md` debe responder rápido:

- fase actual;
- subfase actual;
- estado/conclusión;
- siguiente paso;
- archivos críticos;
- paquetes a compilar;
- pruebas definidas;
- enlaces a detalle.

`01_ESTADO_ACTUAL.md` puede conservar narrativa larga, pero no debe ser el primer
archivo que se abra.

## Última sesión

`07_ULTIMA_SESION.md` es un handoff efímero. En cada cierre:

1. reemplazar su contenido anterior;
2. dejar solo el resumen de la sesión más reciente;
3. guardar el detalle permanente en el historial por subfase o tema;
4. no crear copias `*_DETALLADA_*` salvo petición explícita.

## Uso de historiales por subfase

Los historiales tienen dos capas:

```text
historial_<ID>_RESUMEN.md -> historial_<ID>.md / fragmentos largos
```

Leer siempre el resumen de historial antes de opinar, argumentar o modificar una
subfase. Esto evita repetir caminos descartados y permite recuperar el ultimo
estado bueno conocido si una regresion futura rompe algo.

Editar ambos niveles cuando se haga trabajo real de una subfase:

1. en `historial_<ID>.md`, anadir la entrada cronologica con modificacion,
   build/test/simulacion, resultados, evidencia y conclusion;
2. en `historial_<ID>_RESUMEN.md`, actualizar solo la consecuencia operativa:
   estado vigente, evidencia nueva, aprendizaje, pendiente o estado bueno
   conocido;
3. en `historial/INDEX.md`, actualizar solo la linea corta si cambia el estado
   o el punto de entrada.

No copiar logs largos ni repetir todo el historial en el resumen. Si el resumen
supera mucho 120 lineas, compactarlo.

### Una conclusión por ejecución

- Cada build/test/simulación ejecutado debe conservar su resultado cronológico,
  incluso cuando falle y una modificación posterior lo resuelva.
- Una nueva ejecución añade una entrada nueva con conclusión propia. No
  sobrescribir el intento anterior.
- La conclusión agregada de la subfase vive en
  `historial_<ID>_RESUMEN.md`; puede apoyarse en varias pruebas con conclusiones
  distintas.

### Revisión conversada de una prueba

El veredicto inicial no cierra documentalmente una prueba si el usuario sigue
hablando de sus resultados.

1. Identificar qué ejecución concreta se está discutiendo.
2. Si cambia materialmente un conteo, clasificación, causa, limitación o
   conclusión, modificar la entrada de esa ejecución sin esperar una orden.
3. Conservar la evidencia objetiva y dejar explícita la interpretación
   corregida; no borrar un fallo real ni fabricar una nueva ejecución.
4. Actualizar el resumen agregado y otros documentos solo si sus afirmaciones
   quedaron desactualizadas.
5. Antes de cambiar de prueba, subfase o tema, comprobar que conversación,
   historial y resumen coinciden.

## Paquetes

Cada documentación de paquete debe describir el estado vigente del código. La
evidencia cronológica larga pertenece al historial, no a todos los MDs de
paquete.

Los MDs de `codex/contexto/paquetes/` son la primera fuente para entender código,
scripts, launch y configuración. Antes de abrir un archivo fuente, Codex debe:

1. localizar el paquete con `find-context` o
   `codex/herramientas/find_context.py`;
2. leer `00_summary.md`;
3. abrir solo el MD del componente afectado;
4. ir al código únicamente si el MD no existe, está desactualizado o no responde
   la duda concreta.

Cada MD de componente debe explicar el estado actual, no los intentos
históricos. Debe incluir, cuando aplique:

- ruta del script/código documentado;
- responsabilidad vigente del archivo;
- clases, funciones y métodos importantes;
- símbolo estable de cada zona importante: clase, función o método;
- patrón corto y estable con el que localizarla mediante `rg`;
- líneas aproximadas de esas funciones o zonas de edición, solo como ayuda
  orientativa;
- entradas y salidas relevantes: topics, services, actions, parámetros, YAML,
  mensajes, ficheros y logs;
- relación con otros scripts, nodos o paquetes;
- invariantes y límites que no deben romperse.

No documentar funciones triviales ni copiar código. El objetivo es que Codex
pueda orientarse, decidir y saltar a una zona concreta sin leer archivos fuente
enteros.

Cuando se lea código porque el MD no bastaba:

1. localizar la zona con `rg` usando el símbolo o un patrón específico;
2. leer solo el rango mínimo necesario, nunca el archivo completo por defecto;
3. completar en el mismo turno el MD del componente con la información vigente
   encontrada;
4. dejar ruta, símbolo, patrón de búsqueda y líneas aproximadas para futuras
   modificaciones;
5. evitar añadir narrativa de investigación o resultados de pruebas.

Formato recomendado para una zona relevante:

```markdown
### <Responsabilidad>

- Archivo: `paquete/ruta/archivo.cpp`
- Símbolo: `Namespace::Clase::metodo`
- Localización: buscar `"Clase::metodo("`
- Líneas aproximadas: `~120-210`
```

La referencia estable es el símbolo o patrón de búsqueda. Las líneas exactas no
son un contrato: pueden desplazarse sin obligar a revisar documentación no
afectada. Se corrigen cuando se vuelve a consultar o modificar el componente, o
cuando cambia el símbolo, su responsabilidad o su archivo.

Cuando se toque un paquete:

1. actualizar el `.md` específico del componente;
2. si el cambio altera el mapa rápido, actualizar `05_MAPA_PAQUETES.md`;
3. no duplicar una entrada larga de historial dentro del paquete.

## Uso de la herramienta de búsqueda rápida

Si el agente no sabe dónde buscar, usar antes:

```bash
python3 codex/herramientas/find_context.py <query>
```

Esto busca primero en `codex/contexto/paquetes/*/00_summary.md` y luego en las rutas clave de `CODEX_INDEX.yaml`.

No usar `--deep` salvo que el resumen corto no ofrezca ninguna coincidencia relevante.

## Logs

Un log completo es un artefacto de entrada para herramientas, no una fuente que
Codex pueda abrir. Toda lectura de contenido exige antes un reducido o sublog:

```bash
./codex/herramientas/reduce_build_log.sh
./codex/herramientas/reduce_simulation_log.sh --prueba 1 --patterns "<patrones>"
```

Se permite consultar tamaño, ruta, fecha o código de salida sin leer contenido.
No se permite `cat`, `sed`, `head`, `tail`, `less`, `open` ni mostrar
directamente resultados de `rg` obtenidos del log completo. Si falta una
evidencia, no se abre el original: se
regenera el reducido con patrones ampliados o se crea un sublog temático.

Si el reducido sigue siendo grande, crear sublogs:

```bash
./codex/herramientas/split_simulation_log.sh --prueba 1 --fase 3L
```

Esto genera sublogs e índice breve con conteos.

## Qué no hacer

- No copiar la misma evidencia larga en `01_ESTADO_ACTUAL.md`,
  `07_ULTIMA_SESION.md`, docs de paquete e historial.
- No obligar a leer todos los ADRs si el resumen basta.
- No leer logs completos, ni siquiera como segunda lectura o por ausencia de un
  marcador en el reducido.
- No mantener copias antiguas si la información ya está integrada en el
  historial por subfase o en un índice activo.

## Regla para cerrar una tarea documental

Al reorganizar documentación, comprobar:

```bash
wc -l AGENTS.md codex/contexto/CONTEXTO_MINIMO_ACTUAL.md codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md
wc -l codex/pipeline/fase_3_sparse_global/historial/historial_fase_3.md codex/pipeline/fase_3_sparse_global/historial/INDEX.md
```

El objetivo no es que todo sea corto, sino que el camino inicial sí lo sea.
