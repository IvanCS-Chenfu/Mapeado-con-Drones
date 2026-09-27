# Comandos reproducibles

La ejecucion valida es `c6_2_4_loop_backpressure_v3`. Los intentos `v1` y
`v2` se conservan en los logs, pero no se usan como datos experimentales.

```bash
./codex/herramientas/run_simulation.sh \
  --prueba c6_2_4_loop_backpressure_v3 \
  --launch "ros2 launch simulacion_dron multi_dron.launch.py \
    launch_gazebo_gui:=true launch_rviz:=false \
    launch_mission_gui:=false launch_multidron_gui:=true \
    multidron_gui_start_delay_sec:=0.0 launch_phase6:=false \
    orb_loss_protocol_enabled:=false \
    chapter6_queue_telemetry_enabled:=true \
    chapter6_queue_telemetry_period_ms:=500 \
    debug_fase3_logs_terminal:=true \
    debug_fiducial_visualization:=false \
    raw_stats_telemetry_enabled:=false" \
  --mission-profile "/home/chenfu/Gazebo/src/Pruebas/Capítulo 6/6_2_4_loop_y_backpressure/configuracion/mission_profile_c6_2_4.yaml" \
  --startup-wait-sec 18 --timeout-sec 600 --post-scenario-wait-sec 8 --monitor-resources
```

Procesado reproducible:

```bash
python3 "Pruebas/Capítulo 6/scripts/parse_f3_chapter6.py" \
  --log "codex/archivos_auxiliares/logs/prueba_c6_2_4_loop_backpressure_v3.log" \
  --output-dir "Pruebas/Capítulo 6/6_2_4_loop_y_backpressure/datos_procesados"

MPLCONFIGDIR=/tmp/c6_matplotlib PYTHONNOUSERSITE=1 python3 \
  "Pruebas/Capítulo 6/scripts/plot_c6_2_4.py" \
  --data-dir "Pruebas/Capítulo 6/6_2_4_loop_y_backpressure/datos_procesados" \
  --output-dir "Pruebas/Capítulo 6/6_2_4_loop_y_backpressure/figuras"
```
