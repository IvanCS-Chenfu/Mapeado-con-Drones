# Contexto minimo actual

Precondicion: leer fisicamente `00_CONTEXTO_COMPACTACION.md` antes de este
archivo y reconciliarlo con la peticion mas reciente.

## Estado

```text
Fase 2: CONSEGUIDA el 2026-08-24
Fase 3: cierre previo conseguido; reabierta únicamente en 3Q
Fase 4: CONSEGUIDA Y CERRADA con alcance 4A-4H
4A: CONSEGUIDA
4B: CONSEGUIDA
4C: CONSEGUIDA
4D: CONSEGUIDA; prueba 208 aceptada por el usuario
4E: CONSEGUIDA
4F: CONSEGUIDA
4G: CONSEGUIDA
4H: CONSEGUIDA
4I: APLAZADA como regresion opcional futura
Subfase actual: 3Q, reabierta para corregir errores de optimizacion
Preparacion 3Q: no iniciada; sin autorizacion funcional de cambios
Siguiente punto de entrada: preparar 3Q a partir de la prueba 213
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

Nota: Fase 4 fue reconciliada documentalmente desde
`Fase_4_completa_4A_4I_muy_detallada.zip`. Las antiguas 4J-4L quedan legacy,
no ejecutables como subfases activas.

Pruebas 201/202: contrato fiducial y spawn correctos. 4C+4D quedan conseguidas:
la prueba 208 completo la trayectoria con deteccion real, visualizadores
separados, cierres por timeout y wrappers publicando deltas posteriormente.
Los fallos 203-207 se conservan en el historial 4D.

Pruebas 210/211: 68/68 matches en la trayectoria tipica completa y 18/18 en
el smoke con ambos grafos live. Pending pico 7/10, sin expulsiones, duplicados,
conflictos ni rechazos. 4E+4F quedan conseguidas.

Pruebas 214-217: interpretacion visual robusta, visitas fuera de orden,
handoff al manager y retirada total de GT fiducial conseguidos. La 216 completa
la trayectoria sin GT con 52/52 primary y tres objetos; la 217 valida ambos
grafos live. El GT de control/Fase 5 permanece independiente.

Repeticion visual 212: seis yaw relativos aplicados y compilados, pero no
alcanzados. Un `LoopTask` fue rechazado por
`commit_pose_store_hard_constraint_violation`; su clasificacion hard fijo
`blocking_failure=true` y el mission gate impidio enviar el paso 5. Repeticion
suspendida hasta acordar correccion o prueba visual sin gate.

Correccion posterior autorizada: eliminado `secondary_blocking_failure_`; los
fallos siguen siendo observables pero no enclavan el gate. Build y CTest del
servidor correctos. La prueba 213 completa 17/17 pasos y 22/22 goals, libera el
backpressure, registra 74/74 PUB/SHOW fiduciales y termina con exit 0. El usuario
da 4A-4F por concluidas, pero observa derivas no corregidas; 213 queda marcada
para revisarla de nuevo en 3Q.
