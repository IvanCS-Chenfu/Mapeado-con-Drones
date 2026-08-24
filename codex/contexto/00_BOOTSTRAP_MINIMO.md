# 00 — Bootstrap mínimo para nuevo chat

## Lectura obligatoria

1. `codex/contexto/00_CONTEXTO_COMPACTACION.md`
2. `codex/contexto/CONTEXTO_MINIMO_ACTUAL.md`
3. `codex/contexto/01_ESTADO_ACTUAL_RESUMEN.md`
4. `codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md`
5. subfase concreta y docs de paquetes afectados.

## Estado

```text
Fase activa: Fase 2 — separación Dron/Servidor/Simulación, en cierre
Fase 3: CONSEGUIDA
Prueba 198: PASADA por validación funcional/visual del usuario
Correcciones finales de Fase 2: documentadas, implementación pendiente de autorización
```

## Invariantes

- `submapa=(drone_id,map_epoch)`.
- Raw ORB-SLAM3 no se sobrescribe por optimización.
- `GlobalPoseStore` es autoridad del estado global.
- GT no es fuente funcional final; control GT actual es deuda legacy hasta 5H y fiducial GT hasta Fase 4.
- Dron es caja negra.
- No carga YAML cross-group.
- Réplicas declaradas: parciales por claves o completas solo como deployment profile explícito; `global_map` es la excepción completa vigente.
- Dron/Servidor standalone `use_sim_time=false`; Simulación `true`.
- Logs completos: solo entrada de reductores.
- Debug web apagado: herramienta dormida también en productores.

## Herramientas

```bash
./codex/herramientas/build_selected_packages.sh --group <grupo> <paquete>
./codex/herramientas/run_simulation.sh --prueba X --launch "ros2 launch simulacion_dron multi_dron.launch.py"
./codex/herramientas/reduce_build_log.sh
./codex/herramientas/reduce_simulation_log.sh --prueba X --patterns "<patrones>"
```
