# Mundo Gazebo en `simulacion_dron`

## Mundos actuales

| Archivo | Rol |
|---|---|
| `worlds/empty.world` | Mundo vacío para pruebas básicas |
| `worlds/house_1.world` | Mundo con edificio/casa para mapeo |

La selección se hace en:

```text
simulacion_dron/config/sim_dron.yaml
world.activar: "house_1"
```

## Modelos

`models/house_1/` contiene materiales y texturas del entorno.

Los objetos fiduciales de Fase 4 no estan incrustados en `house_1.world`:
`fiducial_spawner.py` los crea dinamicamente desde los YAML instalados. Esto
mantiene el mundo reutilizable y permite cambiar poses/dimensiones sin duplicar
SDF. `/fiducial_spawn_ready` indica que los tres objetos ya existen.

## Relación con el pipeline maestro

### Fase 1

El mundo debe permitir:
- suficientes features visuales;
- loops/revisitas;
- zonas con posible doble pared si hay deriva;
- fiduciales simulados si están definidos en servidor.

### Fase 6 opcional

Aquí entran mejoras como:
- más mundos;
- reflejos;
- objetos dinámicos;
- iluminación variable;
- pruebas con paredes pobres en textura.

## Reglas

- No cambiar mundo durante una subfase sin documentarlo en el informe.
- Si se cambia `world.activar`, actualizar el historial de la fase.
- Para comparar resultados, usar el mismo mundo y escenario de movimiento.
