# Antigua subfase 5I — Absorbida en 5H

## Estado

```text
NO EJECUTAR COMO SUBFASE INDEPENDIENTE
```

La integración final, adquisición de anchor y regresión multi-dron previstas
originalmente en 5I forman ahora parte de 5H.

## Motivos

- 5H ya posee la integración completa de trayectoria/control y la vuelta final;
- `GT_FALLBACK` permite mantener la misión durante pérdidas en Fase 5;
- recovery, búsqueda activa y reanclaje definitivo sin GT pertenecen a Fase 6;
- mantener 5I repetiría pruebas y responsabilidades de 5H.

## No trasladar desde el contrato antiguo

```text
recovery real sin GT
búsqueda activa compleja de anchor
replanning de misión
ANCHOR_SUBMAP definitivo
```

No borrar historial real si en el futuro existiera alguna ejecución previa de
5I; este archivo solo cierra la ruta ejecutable vigente.
