# Historial infraestructura

> Extraido mecanicamente de `historial_fase_3.md`. Leer este archivo antes de abrir otros historiales de subfase.

## 2026-07-12 — Reorganización documental para reducir consumo de tokens

- objetivo intentado: reducir el coste de arranque de nuevos chats de Codex sin
  perder contexto histórico ni técnico.
- archivos modificados/creados:
  - `AGENTS.md`;
  - `codex/contexto/00_BOOTSTRAP_MINIMO.md`;
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`;
  - `codex/contexto/01_ESTADO_ACTUAL.md`;
  - `codex/contexto/06_MAPA_CODIGO.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`;
  - `codex/contexto/09_LOGS_Y_SUBLOGS.md`;
  - `.agents/skills/actualizar-documentacion/SKILL.md`;
  - `codex/herramientas/USO_HERRAMIENTAS.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/INDEX.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/historial_fase_3.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/por_subfase/*.md`.
- archivos eliminados por redundantes tras integrar su información:
  - `codex/contexto/AGENTS_DETALLADO.md`;
  - `codex/contexto/07_ULTIMA_SESION_DETALLADA_2026-07-12.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/archivo/historial_fase_3_completo_2026-07-12.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_formato.md`.
- resultado de build: `NO EJECUTADO`; solo documentación.
- pruebas Gazebo: `NO EJECUTADAS`; solo documentación.
- evidencia positiva:
  - `AGENTS.md` queda como arranque compacto;
  - el estado corto vive en `01_ESTADO_ACTUAL_RESUMEN.md`;
  - el historial operativo queda dividido por subfase/tema, evitando el archivo
    monolítico;
  - `07_ULTIMA_SESION.md` queda definido como archivo reemplazable, no
    acumulativo;
  - la skill `actualizar-documentacion` exige mantener resúmenes, índices y
    sublogs para evitar repetir bloques largos.
- evidencia negativa o ausente:
  - no se modificaron scripts para generar sublogs automáticamente; por ahora la
    guía queda documentada en `09_LOGS_Y_SUBLOGS.md`.
- conclusión: `CONSEGUIDA` para la reorganización documental.
- siguiente paso recomendado:
  - en futuros cambios, leer primero `00_BOOTSTRAP_MINIMO.md` y
    `01_ESTADO_ACTUAL_RESUMEN.md`;
  - añadir historial nuevo solo en `por_subfase/historial_<ID>.md` y actualizar
    `INDEX.md`;
  - crear sublogs si un reducido sigue siendo grande.

## 2026-07-12 — Tres mejoras de arranque compacto y sublogs

- objetivo intentado: reducir otro salto de tokens en chats nuevos y análisis de
  logs.
- archivos creados/modificados:
  - `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md`;
  - `codex/herramientas/split_simulation_log.sh`;
  - `AGENTS.md`;
  - `codex/contexto/00_BOOTSTRAP_MINIMO.md`;
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`;
  - `codex/contexto/09_LOGS_Y_SUBLOGS.md`;
  - `.agents/skills/actualizar-documentacion/SKILL.md`;
  - `codex/herramientas/USO_HERRAMIENTAS.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3.md`.
- resultado de build: `NO EJECUTADO`; documentación y herramienta shell.
- pruebas Gazebo: `NO EJECUTADAS`.
- validación realizada:
  - `bash -n` para `split_simulation_log.sh`;
  - ejecución de `split_simulation_log.sh` sobre `prueba_1.log` existente.
- conclusión: `CONSEGUIDA` para la mejora documental/herramienta.

## 2026-07-03 — Aclaración documental sobre fases futuras y herramientas

- fase y subfase: Fase 3, actualización documental transversal. No es
  ejecución de una subfase funcional.
- objetivo intentado: hacer que un chat nuevo de Codex entienda cómo continuar
  el proyecto sin tocar los pipelines específicos de fases 2 a 6, que el
  usuario quiere completar más adelante.
- archivos modificados:
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/pipeline/PIPELINE_MAESTRO.md`;
  - `codex/herramientas/USO_HERRAMIENTAS.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/historial_fase_3.md`.
- archivos no modificados por instrucción del usuario:
  - `codex/pipeline/fase_2_poses_drones_sin_gt/`;
  - `codex/pipeline/fase_3_gui/`;
  - `codex/pipeline/fase_4_separacion_paquetes/`;
  - `codex/pipeline/fase_5_nube_densa/`;
  - `codex/pipeline/fase_6_mejoras/`.
- cambios realizados:
  - `00_LEER_PRIMERO.md` indica que los pipelines de fases 2 a 6 pueden estar
    vacíos o incompletos y quedan reservados para planificación futura del
    usuario;
  - `PIPELINE_MAESTRO.md` indica que esos pipelines específicos no son
    ejecutables hasta que el usuario los complete o pida trabajar en ellos;
  - `USO_HERRAMIENTAS.md` cambia encabezados de ejemplo `12R-D4` por ejemplos
    genéricos de Fase 3 para evitar confusión con planificación activa.
- paquetes compilados: ninguno.
- resultado de build: `NO EJECUTADO`, porque solo se modificó documentación.
- pruebas Gazebo ejecutadas: ninguna.
- patrones usados para reducir logs: no aplica.
- evidencia positiva encontrada:
  - la documentación transversal ya explica que Fase 3 sigue activa en `3A`-`3X`;
  - los ejemplos de herramientas ya no presentan `12R-D4` como fase de ejemplo
    activa;
  - los pipelines 2-6 quedan protegidos documentalmente para que Codex no los
    modifique sin petición explícita.
- evidencia negativa o ausente:
  - no se ejecutó build;
  - no se ejecutó simulación;
  - no se valida ninguna subfase funcional.
- conclusión: `CONSEGUIDA` para el objetivo documental de esta sesión.
- siguiente paso recomendado: continuar por `subfase_3A.md` cuando se quiera
  ejecutar trabajo funcional de Fase 3.

## 2026-07-28 — Resúmenes de historial por subfase y memoria de compactación

- objetivo intentado:
  - reducir el coste de lectura historica cuando Codex tenga que opinar,
    argumentar o modificar una subfase;
  - separar con claridad los MDs de paquetes, que describen el codigo vigente,
    de los historiales, que describen que se hizo, que funciono, que fallo y
    que no conviene repetir;
  - hacer que Codex recupere estado operativo tras compactaciones sin releer
    documentos largos.
- archivos creados:
  - `codex/contexto/00_CONTEXTO_COMPACTACION.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3A_RESUMEN.md`
    a `historial_3O_RESUMEN.md`;
  - `historial_12R-D4_RESUMEN.md`;
  - `historial_general_RESUMEN.md`;
  - `historial_infraestructura_RESUMEN.md`;
  - `historial_pruebas_tipicas_RESUMEN.md`.
- archivos modificados:
  - `AGENTS.md`;
  - `codex/contexto/00_BOOTSTRAP_MINIMO.md`;
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`;
  - `codex/contexto/CODEX_INDEX.yaml`;
  - `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`;
  - `codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/INDEX.md`;
  - `codex/pipeline/fase_3_sparse_global/subfases/subfase_3L.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3L.md`;
  - `codex/pipeline/fase_3_sparse_global/historial/por_subfase/historial_3L_RESUMEN.md`.
- cambios realizados:
  - el arranque compacto incluye `00_CONTEXTO_COMPACTACION.md`;
  - tras una compactacion, Codex debe leer ese MD antes de continuar;
  - en tareas largas, Codex debe actualizarlo en hitos y reemplazar estado vivo,
    no acumular diario;
  - para una subfase, Codex debe leer `historial/INDEX.md` y luego
    `historial_<ID>_RESUMEN.md` antes de abrir el historial largo;
  - al modificar una subfase, la entrada cronologica va al historial largo y la
    consecuencia operativa al resumen;
  - `INDEX.md` apunta ya a los resúmenes como puerta de entrada.
- paquetes compilados:
  - no aplica; cambio documental.
- pruebas Gazebo/replay:
  - no aplica; cambio documental.
- validacion realizada:
  - conteo de `19` archivos `*_RESUMEN.md`;
  - todos los resúmenes quedan por debajo de `50` lineas cada uno en esta
    creacion inicial.
- conclusion:
  - `CONSEGUIDA` para la reorganizacion documental.
- siguiente paso recomendado:
  - en futuras subfases, actualizar siempre el historial largo y su
    `historial_<ID>_RESUMEN.md`;
  - si un resumen empieza a crecer, compactarlo antes de que supere unas
    `120` lineas.

## 2026-07-28 — Skills alineadas con la politica antitokens

- objetivo intentado:
  hacer que las skills operativas del proyecto apliquen las reglas nuevas sin
  depender solo de `AGENTS.md`.
- skills modificadas:
  - `.agents/skills/actualizar-documentacion/SKILL.md`;
  - `.agents/skills/ejecutar-fase/SKILL.md`;
  - `.agents/skills/find-context/SKILL.md`.
- cambios realizados:
  - `actualizar-documentacion` lee `00_CONTEXTO_COMPACTACION.md` y
    `historial_<ID>_RESUMEN.md`, actualiza resumen de historial junto al
    historial largo y exige documentar lineas aproximadas/estado vigente del
    componente si se leyo codigo;
  - `ejecutar-fase` arranca con contexto minimo, compactacion, resumen de
    pipeline, contrato de subfase, resumen historico y docs de paquete; deja
    historiales largos, pipeline largo y `01_ESTADO_ACTUAL.md` bajo demanda;
  - `find-context` prioriza MDs de paquete para codigo/scripts y resumenes de
    historial para subfases.
- build y pruebas:
  no ejecutadas; cambio documental de skills.
- validacion:
  `quick_validate.py` devuelve `Skill is valid!` para
  `actualizar-documentacion`, `ejecutar-fase` y `find-context`; se corrigio la
  descripcion YAML de `ejecutar-fase` poniendola entre comillas.
- conclusion:
  `CONSEGUIDA` para la alineacion de skills.

## 2026-07-28 — Protocolo de aprobación antes de ejecutar subfases

- objetivo intentado:
  evitar que Codex ejecute una subfase directamente solo por leer
  `subfase_<ID>.md`, ya que esos contratos pueden estar incompletos o no recoger
  todos los matices que quiere el usuario.
- archivos modificados:
  - `AGENTS.md`;
  - `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`;
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/00_BOOTSTRAP_MINIMO.md`;
  - `.agents/skills/ejecutar-fase/SKILL.md`;
  - `.agents/skills/actualizar-documentacion/SKILL.md`;
  - `codex/contexto/00_CONTEXTO_COMPACTACION.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `historial_infraestructura_RESUMEN.md`.
- cambios realizados:
  - una orden tipo "haz/ejecuta/realiza la subfase <ID>" entra en modo
    preparacion: leer contexto, subfase, resumen historico y MDs de paquete,
    explicar lo entendido y preguntar dudas;
  - Codex no puede modificar codigo, launch, YAML ni configuracion hasta recibir
    una autorizacion explicita equivalente a `TODO LISTO, REALIZA LA SUBFASE`;
  - durante una ejecucion autorizada, si aparece una decision importante no
    acordada, Codex debe parar y preguntar antes de seguir;
  - tras la conversacion preparatoria se puede actualizar documentacion de
    subfase para reflejar lo acordado;
  - los cambios puramente documentales pedidos explicitamente mantienen mas
    autonomia.
- build y pruebas:
  no ejecutadas; cambio documental y de skills.
- validacion:
  `quick_validate.py` devuelve `Skill is valid!` para
  `ejecutar-fase` y `actualizar-documentacion`.
- conclusion:
  `CONSEGUIDA` para el protocolo de actuacion.

## 2026-07-28 — Prueba acordada antes o durante modificaciones de subfase

- objetivo intentado:
  hacer que Codex pregunte o confirme que prueba quiere ejecutar el usuario tras
  una modificacion de subfase, en vez de elegirla por inercia desde el contrato
  de subfase.
- archivos modificados:
  - `AGENTS.md`;
  - `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`;
  - `codex/contexto/00_LEER_PRIMERO.md`;
  - `codex/contexto/00_BOOTSTRAP_MINIMO.md`;
  - `.agents/skills/ejecutar-fase/SKILL.md`;
  - `.agents/skills/actualizar-documentacion/SKILL.md`;
  - `codex/contexto/00_CONTEXTO_COMPACTACION.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `historial_infraestructura_RESUMEN.md`.
- cambios realizados:
  - la preparacion de subfase exige preguntar o confirmar la prueba posterior a
    la modificacion;
  - si durante ejecucion autorizada no hay prueba acordada o cambia el alcance,
    Codex debe parar y preguntar antes de continuar;
  - `ejecutar-fase` no debe cerrar preparacion sin prueba acordada o decision
    explicita de no ejecutar prueba;
  - `actualizar-documentacion` debe reflejar la prueba acordada en contrato,
    historial o resumen cuando aplique.
- build y pruebas:
  no ejecutadas; cambio documental y de skills.
- validacion:
  `quick_validate.py` devuelve `Skill is valid!` para `ejecutar-fase` y
  `actualizar-documentacion`; `rg` confirma la regla en arranque, politica,
  skills, ultima sesion e historial de infraestructura.
- conclusion:
  `CONSEGUIDA` para la regla de prueba acordada.

## 2026-07-28 22:44 — Conclusiones vivas por ejecución

- objetivo intentado:
  hacer que cualquier chat conserve una conclusión independiente por cada
  prueba y sincronice automáticamente el historial si la conversación posterior
  cambia la interpretación de sus resultados.
- archivos modificados:
  - `AGENTS.md`;
  - `.agents/skills/ejecutar-fase/SKILL.md`;
  - `.agents/skills/actualizar-documentacion/SKILL.md`;
  - `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`;
  - `codex/contexto/00_CONTEXTO_COMPACTACION.md`;
  - `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`;
  - `codex/contexto/07_ULTIMA_SESION.md`;
  - `historial_infraestructura_RESUMEN.md`.
- reglas añadidas:
  - cada ejecución real crea una entrada y conclusión propias, incluso si falla;
  - una nueva modificación y ejecución añaden otra entrada sin ocultar la
    anterior;
  - la conversación posterior sobre una misma prueba modifica su entrada cuando
    cambia materialmente conteos, causas, clasificación, limitaciones o
    conclusión;
  - esa sincronización es automática y no requiere una orden documental;
  - la evidencia objetiva se conserva y el resumen de subfase muestra la
    conclusión agregada vigente;
  - no se cambia de prueba, subfase o tema con una discrepancia conocida entre
    conversación e historial.
- build y simulación:
  no aplican; cambio exclusivamente documental y de skills.
- validación:
  `quick_validate.py` devuelve `Skill is valid!` para `ejecutar-fase` y
  `actualizar-documentacion`; `git diff --check` no detecta errores.
- conclusión:
  `CONSEGUIDA` para el protocolo de conclusiones vivas por prueba.

## 2026-08-04 - Prohibicion absoluta de leer logs completos

- objetivo intentado:
  impedir que un log completo llegue al contexto de Codex y consuma tokens con
  informacion irrelevante.
- problema corregido:
  las reglas anteriores decian leer primero el reducido, pero permitian abrir el
  completo cuando faltaba un marcador. Esa excepcion anulaba la proteccion.
- regla vigente:
  el log completo se conserva solo como entrada de `reduce_*`/`split_*`; Codex
  lee unicamente reducidos, indices o sublogs. Si falta evidencia, regenera la
  reduccion con patrones nuevos o mas precisos.
- prohibiciones:
  no usar `cat`, `sed`, `head`, `tail`, `less`, `open` ni mostrar directamente
  resultados de `rg` sobre el log completo.
- archivos sincronizados:
  `AGENTS.md`, politica de tokens, guia de logs, bootstrap, pipeline maestro,
  skills de fase/build/simulacion/documentacion y contratos de subfases que
  conservaban excepciones antiguas.
- build y simulacion:
  no aplican; cambio exclusivamente documental y de protocolo.
- conclusion:
  `CONSEGUIDA` para la puerta obligatoria de reduccion previa.
