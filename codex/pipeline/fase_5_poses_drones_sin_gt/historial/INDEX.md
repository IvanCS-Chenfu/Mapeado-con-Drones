# Historial de Fase 5

## Estado

```text
5A: CONSEGUIDA documentalmente
5B: CONSEGUIDA funcionalmente
5C: CONSEGUIDA
5D: CONSEGUIDA
5E: CONSEGUIDA tecnicamente
5F: PARCIAL; analisis de calidad W por revision pendiente
5G-5H: PARCIAL; 256 falla al aceptar un salto angular ORB sin probation temporal
5I: absorbida en 5H
```

## Lectura barata

| Subfase | Resumen | Historial | Conclusión vigente |
|---|---|---|---|
| 5A | [historial_5A_RESUMEN.md](por_subfase/historial_5A_RESUMEN.md) | [historial_5A.md](por_subfase/historial_5A.md) | Auditoría y contratos reconciliados; sin ejecución funcional. |
| 5B | [historial_5B_RESUMEN.md](por_subfase/historial_5B_RESUMEN.md) | [historial_5B.md](por_subfase/historial_5B.md) | Recibo coherente, `O_T_B`, gate y pérdida real validados en prueba 225. |
| 5C | [historial_5C_RESUMEN.md](por_subfase/historial_5C_RESUMEN.md) | [historial_5C.md](por_subfase/historial_5C.md) | Consulta autoritativa y estados validados. |
| 5D | [historial_5D_RESUMEN.md](por_subfase/historial_5D_RESUMEN.md) | [historial_5D.md](por_subfase/historial_5D.md) | Servicio/push dirigido y revisiones naturales validados. |
| 5E | [historial_5E_RESUMEN.md](por_subfase/historial_5E_RESUMEN.md) | [historial_5E.md](por_subfase/historial_5E.md) | O/W separadas y autoridad aplicada; calidad evaluada en 5F. |
| 5F | [historial_5F_RESUMEN.md](por_subfase/historial_5F_RESUMEN.md) | [historial_5F.md](por_subfase/historial_5F.md) | Prueba 234 valida pose/KFs y corrección visual; pendiente eliminar parpadeo entre estados provisional/autoritativo. |
| 5G | [historial_5G_RESUMEN.md](por_subfase/historial_5G_RESUMEN.md) | [historial_5G.md](por_subfase/historial_5G.md) | Fallback y fuente por goal validados en 243; el fallo posterior pertenece a la extrinseca de 5H. |
| 5H | [historial_5H_RESUMEN.md](por_subfase/historial_5H_RESUMEN.md) | [historial_5H.md](por_subfase/historial_5H.md) | 256 confirma handoff continuo, pero acepta 0.125 rad y colapsa antes de validar persistencia. |

No existe historial independiente de 5I porque su contrato fue absorbido en 5H
antes de cualquier ejecución.
