# 09 — Logs y sublogs para ahorrar tokens

Los logs completos se conservan, pero Codex no puede leerlos directamente. Solo
las herramientas de reducción pueden usarlos como entrada.

## Flujo recomendado

1. Crear o regenerar el reducido estándar con los patrones mecánicos y técnicos
   necesarios:

```bash
./codex/herramientas/reduce_simulation_log.sh --prueba X --patterns "<patrones>"
```

2. Leer únicamente `prueba_X.reduced.log`.

3. Si falta evidencia, regenerar el reducido con patrones nuevos; no abrir el
   original.

4. Si el reducido sigue grande, crear sublogs por tema y leer solo el índice o
   el sublog necesario.

Comando recomendado:

```bash
./codex/herramientas/split_simulation_log.sh --prueba X --fase 3L
```

## Sublogs sugeridos para Fase 3

```text
prueba_X.scenario.log       SCENARIO-RUNNER|SIM-
prueba_X.errors.log         [ERROR]|[FATAL]|Segmentation fault|Killed|process has died
prueba_X.F3H.log            F1H-  # marcador runtime historico de la antigua Fase 1
prueba_X.F3I.log            F1I-
prueba_X.F3J.log            F1J-
prueba_X.F3K.log            F1K-
prueba_X.F3L.log            F1L-
prueba_X.gt_window.log      F1L-GT-WINDOW-STATS|F1L-GT-COLLATERAL-CHECK
```

## Índice de prueba

La herramienta crea:

```text
codex/archivos_auxiliares/logs/prueba_X.index.md
```

Contenido recomendado:

```text
# Índice prueba X

- comando:
- escenario/YAML:
- resultado mecánico:
- goals enviados:
- goals success:
- errores graves, sin contar marcadores tipo `F1H-FID-POSE-ERROR`:
- marcadores clave y conteos:
- evidencia positiva:
- evidencia negativa:
- conclusión preliminar:
- log completo:
- sublogs:
```

## Reglas

- El log completo nunca se borra por crear sublogs.
- Un sublog no sustituye el análisis: solo reduce tokens.
- Si falta un marcador obligatorio, ampliar patrones y volver a reducir. La
  ausencia solo se concluye después de una reducción que busque explícitamente
  ese marcador.
- Prohibido abrir o volcar el log completo con `cat`, `sed`, `head`, `tail`,
  `less`, `open`, salida directa de `rg` o herramientas equivalentes.
- Los sublogs viven en `codex/archivos_auxiliares/logs/` y pueden regenerarse.
