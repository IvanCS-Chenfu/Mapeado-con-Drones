# Contexto minimo actual

Precondicion: leer fisicamente `00_CONTEXTO_COMPACTACION.md` antes de este
archivo y reconciliarlo con la peticion mas reciente.

## Estado

```text
Fase 2: CONSEGUIDA el 2026-08-24
Fase 3: CONSEGUIDA
Fase actual: Fase 4, sin ejecutar
Siguiente subfase: 4A, solo tras preparacion y autorizacion
Revision visual humana de prueba 200: confirmada correcta
Cierre de Fase 2: completo
```

## Arquitectura vigente

```text
src/dron/       -> ORB-SLAM3, wrapper, control, trayectorias e interfaces
src/servidor/   -> backend y servidor de mapa global e interfaces canonicas
src/simulacion/ -> Gazebo, escenarios, integracion y visualizadores
```

Los builds usan bases separadas `build/install/log/{dron,servidor,simulacion}`
y exactamente un paquete por invocacion. `orbslam3_msgs` es canonico en
Servidor y replica exacta en Dron. `mi_tfg` permanece como legacy fuera de los
tres grupos.

## Configuracion y debug

ADR 0009 gobierna ownership y replicas YAML. ADR 0010 exige coste especifico
practicamente nulo cuando `pipeline_flow` o `system_architecture` estan
desactivados. Los siete flags de debug quedan en `false` por defecto.

`pipeline_flow` muestra el flujo interno sparse/global.
`system_architecture` muestra paquetes, grupos e interfaces y recibe actividad
ligera por `/system_architecture/activity`.

## Evidencia de Fase 2

- nueve builds aislados correctos, uno por invocacion;
- CTest: 4/4, 9/9, 10/10 y 9/9 en suites funcionales;
- prueba 199: 5/5 pasos, 4/4 goals y debug especifico dormido;
- prueba 200: 14/14 pasos, 20/20 goals, RViz2 y ambos web activos;
- ambos visualizadores validados por separado; modo live con evento ROS real;
- layout final validado por CTest y capturas desktop/viewport estrecho;
- guardas de layout, interfaces, dependencias, config, paths y visualizers pasan.

La prueba 200 conserva dos incidencias de cleanup posteriores a `SIM-DONE`:
traceback de `gui_tray_multi` y Gazebo 255. Los bridges, RViz2, wrappers y
servidor cerraron limpiamente.

## Lectura siguiente

```text
codex/pipeline/fase_2_separacion_paquetes/RESULTADO_FINAL_FASE_2.md
codex/pipeline/fase_2_separacion_paquetes/historial/INDEX.md
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
codex/contexto/05_MAPA_PAQUETES.md
```
