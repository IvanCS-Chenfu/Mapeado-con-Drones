---
name: ejecutar-simulacion
description: Ejecuta las pruebas de simulación definidas por una fase usando run_simulation.sh y reduce los logs generados.
---

Usar después de que el build haya devuelto `0`.

## Checkpoint obligatorio

Antes de cada simulación, actualizar
`codex/contexto/00_CONTEXTO_COMPACTACION.md` con prueba, YAML, launch, timeout y
siguiente acción. Inmediatamente después de que termine
`run_simulation.sh`, registrar código de salida, `success`, ruta del log y
siguiente acción antes de reducir, analizar o repetir.

No aplazar este checkpoint hasta la documentación final. Si el chat se compacta,
releer físicamente la memoria antes de continuar; el resumen automático no la
sustituye.

Precondiciones:
- Existen YAMLs de prueba en `codex/archivos_auxiliares/tray_prueba_X.yaml`.
- La fase indica cuántas pruebas ejecutar.
- La fase o `planificador_fase` indica qué patrones buscar en cada log.

Comando base por prueba:

```bash
./codex/herramientas/run_simulation.sh \
  --prueba <X> \
  --launch "ros2 launch <paquete> <launch>.launch.py" \
  --post-scenario-wait-sec <segundos>
```

La herramienta debe usar:

```text
codex/archivos_auxiliares/tray_prueba_X.yaml
```

La salida debe ser:

```text
codex/archivos_auxiliares/prueba_X.log
```

Ese archivo es el log completo de la prueba.

Después de cada prueba, reducir el log:

```bash
./codex/herramientas/reduce_simulation_log.sh --prueba <X> --patterns "PATRON1|PATRON2|ERROR|FATAL"
```

La reduccion debe generar:

```text
codex/archivos_auxiliares/prueba_X.reduced.log
```

Reglas:
- No analizar aquí si la prueba pasó. Eso lo hace `analizador_simulacion_logs`.
- No generar informes extra salvo que el usuario lo pida.
- Si una simulación falla por timeout o por action fallida, conservar tanto el log completo como el reducido para análisis.
- El log completo se conserva, pero ningún agente puede leerlo directamente.
- Si el reducido no contiene marcadores suficientes, regenerarlo con patrones
  ampliados o crear un sublog temático y leer solo ese artefacto.
- Cada prueba, incluido un intento fallido, requiere su propio checkpoint de
  resultado.
