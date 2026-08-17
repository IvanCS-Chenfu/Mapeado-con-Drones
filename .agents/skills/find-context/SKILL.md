---
name: find-context
description: Localiza rápidamente archivos de contexto relevantes usando `CODEX_INDEX.yaml` y resúmenes cortos de paquete antes de que el agente abra documentos largos.
---

Usar cuando el agente no sepa dónde buscar documentación o contexto específico.

Objetivo:
- encontrar primero `00_summary.md` en `codex/contexto/paquetes/`.
- para código/scripts, localizar el MD del componente antes de abrir el archivo
  fuente.
- para subfases/historial, localizar primero `historial_<ID>_RESUMEN.md`.
- si no hay coincidencias, buscar en los archivos prioritarios de `CODEX_INDEX.yaml`.
- evitar abrir MDs largos salvo necesidad real.

Flujo:
1. Ejecutar `python3 codex/herramientas/find_context.py <query>`.
2. Si el resultado lista archivos, abrir solo esos archivos y no otros.
3. Si la consulta es de subfase/historial, abrir
   `codex/pipeline/fase_3_sparse_global/historial/INDEX.md` y luego
   `historial_<ID>_RESUMEN.md`; abrir el historial largo solo si falta evidencia.
4. Si no hay coincidencias y se necesita más detalle, ejecutar con `--deep`.
5. Si el usuario pide un paquete específico, abrir primero `codex/contexto/paquetes/<paquete>/00_summary.md`.
6. Para investigar código, usar primero la ruta, símbolo y patrón de búsqueda
   indicados por el MD del componente.
7. Si el MD no resuelve la duda, localizar con `rg` el símbolo o patrón y abrir
   únicamente el rango mínimo de código necesario.

Reglas:
- esta skill solo busca contexto, no modifica código ni documentación.
- si el resultado incluye menos de 3 coincidencias, el agente puede abrir esos archivos directamente.
- si no hay coincidencias y se usa `--deep`, priorizar archivos con `HISTORY: true` como última opción.
- no usar la búsqueda profunda por defecto para no gastar tokens en MDs largos.
- si tras leer código porque faltaba documentación se obtiene información útil,
  actualizar en el mismo turno el MD del componente en
  `codex/contexto/paquetes/` con ruta, símbolo, patrón estable y líneas
  aproximadas;
- las líneas son orientativas; no revisar todos los MDs por desplazamientos de
  línea si el símbolo y su responsabilidad siguen vigentes.
