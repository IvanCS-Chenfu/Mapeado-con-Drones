# Estado actual - resumen

## Situacion

```text
Fase 2: CONSEGUIDA
Fase 3: CONSEGUIDA
Fase actual: Fase 4, sin ejecutar
Siguiente subfase: 4A, preparación no iniciada
Trabajo funcional activo: ninguno
Revision visual de prueba 200: confirmada correcta por el usuario
Pendiente de Fase 2: ninguno
Autorización funcional de Fase 4: pendiente
```

## Entrega de Fase 2

- grupos fisicos `dron`, `servidor` y `simulacion`;
- builds y prefijos separados por grupo;
- interfaces duplicadas de forma controlada;
- configuracion por dominio y despliegue segun ADR 0009;
- ORBvoc completo instalado desde un bootstrap fuera de `src`;
- observabilidad lazy-gated segun ADR 0010;
- `system_architecture` estatico/live separado de `pipeline_flow`;
- guardas automaticas de arquitectura y documentacion.

## Validacion

- build: 9/9 paquetes, un paquete por invocacion;
- CTest: `lib_tray` 4/4, `orbslam3_multi` 9/9,
  `orbslam3_server` 10/10 y `simulacion_dron` 9/9;
- prueba 199: debug-off, 5/5 pasos y 4/4 goals;
- prueba 200: debug completo, 14/14 pasos y 20/20 goals;
- RViz2 y ambos web activos; guarda de recursos no disparada;
- `system_architecture_bridge` cierra sin el `ValueError` de prueba 198.
- layout final: CTest 9/9, guarda 15/15 y dos viewports inspeccionados.

## Limitaciones

`dron_individual` conserva deuda legacy global de linters, aunque todos los
archivos tocados pasan comprobaciones focales y rebuild. La prueba 200 presenta
un traceback de cleanup de `gui_tray_multi` y el exit 255 conocido de Gazebo,
ambos posteriores a `SIM-DONE`.

## Referencias

```text
codex/pipeline/fase_2_separacion_paquetes/RESULTADO_FINAL_FASE_2.md
codex/pipeline/fase_2_separacion_paquetes/historial/INDEX.md
codex/contexto/decisiones/ADR_0009_configuracion_por_dominio_y_despliegue.md
codex/contexto/decisiones/ADR_0010_observabilidad_web_debug_coste_cero.md
```
