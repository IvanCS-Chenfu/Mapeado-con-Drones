# Subfase 1K - Limpieza y cierre de Fase 1

## Objetivo

Retirar codigo experimental sin uso demostrado, centralizar la telemetria de
Fase 1 bajo `debug_fase_1` y verificar que control, trayectoria y pitch no
sufren regresiones.

## Alcance

- eliminar ejecutables y prototipos no instalados, lanzados, importados ni
  referenciados por el pipeline vigente;
- declarar `debug_fase_1=false` en el perfil del simulador;
- propagarlo a nodos de control y plugins Gazebo de motores, ground truth y
  pitch;
- con el flag desactivado, ocultar `DEBUG/INFO` de Fase 1 y conservar avisos,
  errores y marcadores de resultado del escenario;
- no alterar logs ni comportamiento de Fases 3, 4 o 5.

## Pruebas

1. Tests estaticos y unitarios de los paquetes afectados.
2. Build de `dron_individual` y `simulacion_dron`.
3. Simulacion GT corta con movimiento, yaw y pitch con el flag desactivado.
4. Repeticion con el flag activado para comprobar que reaparece la telemetria.

## Criterio de cierre

Build y tests correctos; ambas simulaciones completan sus goals sin errores
graves; con `false` no aparecen mensajes informativos F1 y con `true` si
aparecen; documentacion sincronizada, commit creado y push a `origin/main`.
