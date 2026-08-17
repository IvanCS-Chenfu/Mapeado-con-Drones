---
name: ejecutar-fase
description: "Workflow para preparar y ejecutar una fase/subfase del pipeline: primero explicar, preguntar y acordar con el usuario; solo después de autorización explícita implementar, compilar, simular, analizar logs, documentar cada prueba y mantener sus conclusiones sincronizadas durante la conversación posterior."
---

Usa esta skill cuando el usuario pida ejecutar una fase o subfase del pipeline.

Objetivo:
- convertir una orden directa del usuario en una preparacion conversada antes de
  cualquier cambio funcional;
- comprobar tanto que Codex entiende al usuario como que el usuario conoce y
  acepta el alcance, riesgos y validacion;
- ejecutar implementacion, build, simulacion, analisis y documentacion solo tras
  autorizacion explicita del usuario;
- no repetir preguntas si existe un acuerdo previo completo y verificable;
- no dejar la fase en un estado ambiguo;
- concluir con evidencia si la subfase queda `CONSEGUIDA`, `NO CONSEGUIDA`, `PARCIAL` o `BLOQUEADA`.

## Puerta no omisible

Una primera orden directa (`haz`, `ejecuta`, `realiza`, `implementa`) nunca es
autorizacion funcional suficiente. Esta skill debe activarse en modo
preparacion, sin código, launch, YAML, configuracion, build, test ni simulacion.

Solo se omite repetir el debate cuando la conversacion actual o
`00_CONTEXTO_COMPACTACION.md` demuestra un acuerdo previo con objetivo,
comportamiento, alcance, exclusiones, prueba, criterios, riesgos aceptados y
`Dudas abiertas: ninguna`. Un contrato de subfase, historial o resumen
automatico no demuestra por si solo la aceptacion del usuario.

La implementacion solo puede empezar cuando se cumplen simultaneamente:

```text
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA
Prueba acordada: definida o decision explicita de no probar
Dudas abiertas: ninguna
```

Workflow:

1. Preparar sin modificar:
   - `AGENTS.md`
   - `codex/contexto/00_CONTEXTO_COMPACTACION.md`
   - `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`
   - `codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md`
   - `codex/pipeline/PIPELINE_MAESTRO.md`
   - resumen de pipeline de la fase actual
   - subfase actual
   - `historial/INDEX.md`
   - `historial_<ID>_RESUMEN.md` de la subfase
   - `00_summary.md` y MDs de componentes afectados en `codex/contexto/paquetes/`

   Abrir `01_ESTADO_ACTUAL.md`, pipeline largo, decisiones o historiales largos
   solo si los resúmenes no bastan. No modificar código, launch, YAML ni
   configuracion en esta fase.

   Registrar `Preparacion: EN_DEBATE`, `Acuerdo cerrado: no` y
   `Autorizacion funcional: PENDIENTE`. Una orden inicial no cambia esos estados.

2. Explicar y preguntar:
   - resumir al usuario lo que dice la subfase;
   - explicar que entiende Codex como objetivo, alcance, archivos probables,
     paquetes, pruebas, patrones, criterios y riesgos;
   - preguntar o confirmar que prueba quiere el usuario ejecutar tras la
     modificacion, aunque la subfase ya proponga pruebas por defecto;
   - listar ambigüedades o decisiones abiertas;
   - comprobar que el usuario entiende y acepta cambios, exclusiones, riesgos,
     prueba y criterios;
   - hacer preguntas concretas hasta que ambas partes tengan claro como se
     realizara la subfase;
   - presentar un resumen final del acuerdo y pedir confirmacion.

3. Esperar autorización:
   - marcar `Preparacion: CERRADA`, `Acuerdo cerrado: si` y
     `Dudas abiertas: ninguna` solo después de confirmar el resumen;
   - no empezar cambios funcionales hasta un mensaje posterior de autorizacion
     explicita del usuario
     equivalente a `TODO LISTO, REALIZA LA SUBFASE`,
     `con todo lo hablado, ejecútalo` o `me parece bien, sigue`;
   - si ya existe un acuerdo previo verificable, la petición actual puede ser
     ese mensaje posterior y no se repiten preguntas resueltas;
   - al autorizar, registrar `Autorizacion funcional: CONCEDIDA`;
   - si la conversacion aclara el contrato, se puede actualizar documentación de
     subfase antes de ejecutar cuando el usuario lo haya pedido o aceptado;
   - no cerrar la preparacion sin una prueba acordada por el usuario o una
     decision explicita de no ejecutar prueba;
   - cambios puramente documentales solicitados por el usuario no requieren esta
     autorizacion funcional.

4. Usar `planificador_fase` tras la autorización:
   - validar si la fase está bien planteada;
   - definir archivos y funciones a tocar usando primero los MDs de paquete;
   - definir YAMLs de prueba `codex/archivos_auxiliares/tray_prueba_X.yaml`;
   - definir paquetes a compilar;
   - definir patrones para reducir logs de simulación;
   - definir criterio de exito y fallo verificable;
   - conservar la prueba acordada con el usuario como parte del plan; si falta o
     deja de encajar con los cambios, preguntar antes de elegir otra.

   Si la subfase no define archivos, pruebas, patrones o criterio de exito,
   buscar esa informacion primero en `historial_<ID>_RESUMEN.md`, docs de
   paquete y resumen de pipeline. Si sigue faltando informacion critica,
   consultar detalle bajo demanda o preguntar al usuario antes de modificar
   codigo.

   Antes de pasar a implementación, actualizar
   `codex/contexto/00_CONTEXTO_COMPACTACION.md` con objetivo, autorización,
   plan, archivos críticos, prueba acordada y siguiente acción exacta.

5. Usar `implementador_fase`:
   - aplicar cambios mínimos;
   - crear/modificar YAMLs de prueba;
   - al terminar el bloque de cambios, registrar archivos modificados y dejar
     el build como siguiente acción exacta;
   - ejecutar build con `codex/herramientas/build_selected_packages.sh`;
   - inmediatamente después de terminar el build, registrar su resultado antes
     de diagnosticar, corregir o continuar.

   Antes de abrir código, consultar el MD del componente en
   `codex/contexto/paquetes/`. Usar su ruta, símbolo y patrón de búsqueda para
   localizar con `rg` el fragmento exacto que haya que modificar. Si falta
   información y se lee código, abrir solo el rango mínimo necesario y
   completar el MD del componente en el mismo turno; después de modificar el
   código, sincronizar de nuevo ese MD con el estado resultante.

6. Si el build falla:
   - ejecutar `codex/herramientas/reduce_build_log.sh`;
   - usar `diagnosticador_build` para leer `codex/archivos_auxiliares/colcon_build.reduced.log`;
   - si falta contexto, regenerar o ampliar `colcon_build.reduced.log`; nunca
     consultar directamente `colcon_build.log`;
   - registrar el primer error real y la corrección siguiente en
     `00_CONTEXTO_COMPACTACION.md`;
   - corregir directamente solo si es una reparación mecánica que conserva el
     acuerdo; si exige elegir comportamiento, alcance o estrategia, registrar
     `Autorizacion funcional: SUSPENDIDA`, parar y preguntar;
   - devolver el diagnóstico a `implementador_fase`;
   - repetir build.

7. Si el build pasa:
   - registrar la prueba y comando siguientes antes de una simulación larga;
   - ejecutar `codex/herramientas/run_simulation.sh` una vez por cada prueba definida;
   - al terminar cada prueba, registrar inmediatamente `success`, código de
     salida, ruta de log y siguiente acción antes de reducir, analizar o repetir;
   - si el fallo obliga a cambiar prueba, criterio o comportamiento, suspender
     la autorización y preguntar antes de hacerlo;
   - generar logs completos `codex/archivos_auxiliares/prueba_1.log`, `prueba_2.log`, etc.;
   - ejecutar `codex/herramientas/reduce_simulation_log.sh` para generar `prueba_1.reduced.log`, `prueba_2.reduced.log`, etc.

8. Usar `analizador_simulacion_logs`:
   - leer exclusivamente logs reducidos `prueba_X.reduced.log`, índices o
     sublogs temáticos;
   - si faltan marcadores obligatorios, regenerar el reducido con patrones
     específicos; nunca consultar directamente `prueba_X.log`;
   - comprobar `SCENARIO-RUNNER`, envio de goals, resultados `success=true` y cierre de simulacion cuando aplique;
   - comprobar los marcadores tecnicos exigidos por la subfase;
   - comprobar ausencia de errores graves;
   - decidir si las pruebas pasaron;
   - concluir si la fase quedó `CONSEGUIDA`, `NO CONSEGUIDA`, `PARCIAL` o `BLOQUEADA`.
   - registrar esa conclusión y la evidencia mínima antes del cierre
     documental.

9. Usar `curador_documentacion`:
   - actualizar docs de paquetes modificados;
   - actualizar `codex/contexto/paquetes/` para que describa los archivos, funciones, nodos, topics/actions y logs actuales tras la modificacion;
   - actualizar historial largo y `historial_<ID>_RESUMEN.md`;
   - verificar que `00_CONTEXTO_COMPACTACION.md` contiene los checkpoints
     creados durante la ejecución y cerrarlo de forma coherente;
   - actualizar estado actual;
   - ajustar subfase actual o siguiente si procede.

10. Responder al usuario:
   - fase/subfase trabajada;
   - archivos modificados;
   - paquetes compilados;
   - resultado del build;
   - pruebas ejecutadas;
   - resumen de logs;
   - conclusion exacta;
   - documentacion actualizada;
   - siguiente paso recomendado.

11. Si el usuario continúa hablando de los resultados:
   - tratar la conversación como continuación del análisis de la prueba
     concreta, aunque ya se haya enviado un veredicto;
   - si cambia un conteo, causalidad, clasificación, alcance o conclusión,
     modificar automáticamente la entrada de esa prueba y sus resúmenes
     afectados, sin pedir una orden adicional;
   - conservar los datos objetivos y dejar explícita la interpretación
     corregida;
   - no crear una entrada nueva salvo que exista una nueva ejecución real;
   - si hay otra modificación y otra ejecución, añadir una entrada cronológica
     independiente, incluso si la anterior falló;
   - antes de pasar a otra prueba/subfase o cerrar el tema, comprobar que la
     conclusión conversada y el historial coinciden.

Reglas:
- Solo `implementador_fase` modifica código.
- Solo `curador_documentacion` modifica documentación permanente. La memoria
  operativa `00_CONTEXTO_COMPACTACION.md` es una excepción: el agente que
  ejecuta cada hito debe reemplazar su checkpoint inmediatamente.
- Tras una compactación, releer físicamente
  `00_CONTEXTO_COMPACTACION.md` antes de cualquier otra acción. El resumen
  automático del chat no sustituye esa lectura.
- Una peticion inicial de "haz la subfase X" no autoriza cambios funcionales:
  primero explicar, comprobar comprensión mutua y esperar una autorización
  posterior.
- Si existe acuerdo previo completo y verificable, no repetir preguntas; una
  orden posterior puede conceder la autorización.
- Antes o durante modificaciones de una subfase, preguntar/confirmar que prueba
  quiere el usuario ejecutar tras la modificacion.
- La autorización solo cubre el acuerdo confirmado. Ante cualquier duda
  funcional, alternativa material o decisión no acordada, registrar
  `Autorizacion funcional: SUSPENDIDA`, parar y preguntar; continuar solo tras
  cerrar el nuevo acuerdo y recibir confirmación del usuario.
- Una corrección mecánica que conserva exactamente el acuerdo puede continuar y
  debe registrarse. Si no está claro que sea mecánica, parar y preguntar.
- Si durante una ejecucion autorizada la prueba posterior no esta clara o cambia
  el alcance de validacion, parar y preguntar antes de continuar.
- No crear archivos auxiliares extra salvo logs completos `colcon_build.log`/`prueba_X.log`, logs reducidos `colcon_build.reduced.log`/`prueba_X.reduced.log` y YAMLs de prueba, salvo que el usuario lo pida.
- No ejecutar fases futuras antes de cerrar la fase actual.
- No declarar una subfase como conseguida si no se cumple el criterio de exito escrito en la subfase.
- Si el build compila pero los logs no contienen la evidencia esperada, la conclusion debe ser `NO CONSEGUIDA` o `PARCIAL`.
- Si una prueba no se ejecuta, el historial debe decirlo explicitamente.
- Cada prueba ejecutada conserva su propia conclusión histórica. Una repetición
  añade otra entrada y nunca oculta un intento fallido anterior.
- La conclusión de una prueba permanece revisable mientras el usuario hable de
  sus resultados. Toda corrección material acordada se documenta
  automáticamente en esa misma entrada y en su resumen operativo.
- Si RViz2 era parte de la validacion visual pero no fue observado por el usuario, documentarlo como no observado.
- Si se modifica un paquete, no cerrar la ejecucion sin revisar la documentacion correspondiente en `codex/contexto/paquetes/`.
- Si se modifica una subfase, no cerrar la ejecucion sin actualizar
  `historial_<ID>.md` y `historial_<ID>_RESUMEN.md`.
