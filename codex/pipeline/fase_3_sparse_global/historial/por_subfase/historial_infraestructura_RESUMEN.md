# Historial infraestructura - resumen

Leer este archivo antes de `historial_infraestructura.md` cuando haya que hablar
de reglas, herramientas o estructura documental para Codex.

## Estado vigente

Tema documental transversal. Su objetivo es reducir tokens y hacer que nuevos
chats recuperen contexto sin leer documentos largos.

## Que se hizo

- `AGENTS.md` queda como arranque compacto.
- `CONTEXTO_MINIMO_ACTUAL.md` y `pipeline_fase_3_RESUMEN.md` resumen el estado.
- Historiales se dividen por subfase/tema.
- `07_ULTIMA_SESION.md` se define como cierre reemplazable.
- `split_simulation_log.sh` permite crear sublogs cuando un reducido sigue
  siendo grande.
- `00_CONTEXTO_COMPACTACION.md` se usa como memoria viva frente a compactaciones
  durante trabajos largos.
- Los nuevos `historial_<ID>_RESUMEN.md` son la primera lectura historica por
  subfase/tema; el historial largo queda como evidencia cronologica.
- Las skills `actualizar-documentacion`, `ejecutar-fase` y `find-context` ya
  apuntan a compactacion, resumenes de historial y MDs de paquete antes de
  detalle largo/codigo.
- Nueva regla de subfases: una orden tipo "haz la subfase X" inicia preparacion
  conversada, no ejecucion funcional inmediata; codigo/launch/YAML/config solo
  se modifican tras confirmacion explicita del usuario.
- La prueba posterior a una modificacion de subfase debe preguntarse o
  confirmarse con el usuario antes o durante la ejecucion; si no esta acordada o
  cambia el alcance, Codex debe parar y preguntar.
- Cada ejecución conserva una conclusión histórica propia. Una repetición añade
  otra entrada y una conversación posterior que cambia la interpretación
  actualiza automáticamente la entrada de la prueba discutida, sin ocultar
  intentos fallidos ni esperar una orden documental.
- Los logs completos se conservan solo como entrada de reductores. Codex nunca
  los abre: lee exclusivamente reducidos, índices o sublogs; si falta evidencia,
  regenera la reducción con patrones específicos.

## Evidencia

- Cambios documentales y herramienta shell; sin build/simulacion funcional.
- Se valido sintaxis de `split_simulation_log.sh` y ejecucion sobre un log
  existente.
- `quick_validate.py` valido `ejecutar-fase` y `actualizar-documentacion` tras
  añadir la regla de prueba acordada.

## Aprendizajes

- No volver a historiales monoliticos.
- No leer logs completos bajo ninguna circunstancia, tampoco por ausencia de un
  marcador. Volver a reducir sin volcar el original al contexto.
- Mantener resumen corto -> detalle bajo demanda.
- Cuando se modifique una subfase, actualizar el historial largo y tambien su
  resumen operativo.
- Mantener las skills sincronizadas con `AGENTS.md` si cambia la politica de
  lectura barata.
- Si una subfase esta ambigua durante ejecucion autorizada, parar y preguntar
  antes de decidir por cuenta propia.
- No elegir pruebas por inercia: la prueba acordada por el usuario es parte del
  contrato operativo de la subfase.
- No congelar el primer veredicto: antes de cambiar de prueba, subfase o tema,
  conversación, historial y resumen deben reflejar la misma interpretación.

## Detalle

`historial_infraestructura.md`.
