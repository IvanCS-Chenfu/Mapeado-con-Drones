# Historial 2C

## 2026-08-24 - Configuración por dominio

- `hardware.yaml` y `tray_dron.yaml` se sustituyeron por YAML de dominio;
- se crearon réplicas locales `physical_dron.yaml`, `actuators_dron.yaml` y
  `simulated_sensors.yaml`;
- se retiró `usar_veltrap` y Xacro consume la masa configurada;
- el bootstrap extrae ORBvoc a `build/dron/_phase2_resources` y CMake lo instala;
- guardas de ownership, tipos, réplicas y reloj: correctas;
- conclusión: `CONSEGUIDA`.
