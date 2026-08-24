# Estado actual - resumen

## Situación

```text
Fase 1: realizada
Fase 2: ACTUAL — en cierre técnico
Fase 3: CONSEGUIDA
Fases 4-5: sin hacer; contratos documentales refinados
Prueba 197: smoke debug-off conseguido
Prueba 198: conseguida por validación funcional/visual del usuario
```

## Fase 2

La separación física Dron/Servidor/Simulación y los builds aislados ya existen en el snapshot probado. El cierre pendiente corrige configuración cross-group, ownership/authority/deployment, `use_sim_time`, ORBvoc bootstrap, `usar_veltrap`, masa Xacro, observabilidad web y guardas/documentación.

La prueba 198 se repetirá tras esas correcciones porque validó el snapshot anterior.

## Deudas deliberadamente aplazadas

- Fiducial simulado GT: Fase 4.
- Pose/velocidad GT funcional de `gen_tray` y `control_calcular_fuerzas`: Fase 5/5H.
- Transporte futuro de calibración Dron→Servidor y configuración Server→Dron: documentado, no implementado en Fase 2.

## Observabilidad

`pipeline_flow` y `system_architecture` son independientes. Con debug false no debe existir trabajo específico ni en bridges ni en productores. `system_architecture` solo ilumina tráfico runtime con evidencia directa.

## Fuente de verdad

Leer `00_CONTEXTO_COMPACTACION.md` y `pipeline_fase_2_RESUMEN.md` antes de ejecutar cambios.
