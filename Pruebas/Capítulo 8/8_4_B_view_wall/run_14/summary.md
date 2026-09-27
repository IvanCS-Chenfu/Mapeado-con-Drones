# 8.4-B / run_14 - Resumen de evidencia

## Resultado

**CONSEGUIDA.** D1 se ancló con GT en `(0,-10,1,90 grados)` y el workflow experimental `test_view_wall_fixed:1` ejecutó `MOVE_AND_CAPTURE` a `(4,-9,1,90 grados)`.

El comando finalizó con `depth=1`. La evidencia depth generó:

```text
FREE=1
DIRECT_FREE=1
OCCUPIED=1
```

La fuente `depth_occupied:2108:1:0:33` se asoció primero a la sección U `111` y después produjo:

```text
claims=13
active=13
changed=true
```

La tarea `map_section_level_0_AB` contiene 192 secciones U. El progreso derivado del recuento final es `13/192 = 6,77 %`.

## Cadena causal registrada

```text
MOVE_AND_CAPTURE
-> depth=1
-> FREE + DIRECT_FREE + OCCUPIED escritos
-> source pending para sección 111
-> DEPTH_OCCUPIED materializado
-> 13 claims de coverage
```

La actualización no se atribuye al movimiento puro: el claim se emitió después de la escritura y materialización de la fuente `depth_occupied`.

## Condiciones de la ejecución

Para aislar la captura frontal se usaron parámetros experimentales limitados a esta ejecución: objetivo fijo, navegación permitida a través de UNKNOWN, margen extra de obstáculos igual a cero y supresión de STOP de corredor para este workflow. Los valores por defecto del sistema no cambiaron.

El log bruto se conserva centralmente como `codex/archivos_auxiliares/logs/prueba_c8_4_b_view_wall_run_14.log`; la evidencia se consultó mediante su reducción.
