# 00 - Contexto de compactación

## Estado vivo

```text
Fecha de checkpoint: 2026-08-24
Fase activa: Fase 2 — separación Dron/Servidor/Simulación
Fase 3: CONSEGUIDA
Estado Fase 2: EN CIERRE TÉCNICO
Acuerdo funcional de correcciones: CERRADO
Autorización actual para código/YAML/launch: PENDIENTE
Trabajo autorizado en este checkpoint: documentación y revisión de GitHub
Dudas funcionales abiertas: ninguna
```

## Evidencia ya disponible

- 2A: estructura física Dron/Servidor/Simulación existente y dos copias de `orbslam3_msgs` validadas como idénticas en el snapshot probado.
- 2B: 9/9 builds aislados terminaron correctamente, un paquete por invocación y bases `build/install/log` por grupo.
- Tests relevantes: `lib_tray` 4/4, `orbslam3_multi` 9/9, `orbslam3_server` 10/10, `simulacion_dron` 9/9; `dron_individual` conserva deuda legacy de lint fuera del alcance funcional, con checks focales y rebuild correctos.
- Prueba 197: smoke con defaults debug false, `success=true`, sin RViz/web y sin telemetría visual observada; cleanup Gazebo 255 posterior a `SIM-DONE` conocido.
- Prueba 198: ejecutada; el usuario confirmó funcionamiento funcional y visual correcto. Su log completo se conserva pero no se lee directamente.

La prueba 198 valida el snapshot **anterior** a las correcciones finales acordadas; después de implementarlas se repetirá una regresión equivalente.

## Correcciones acordadas antes del cierre

### Configuración y despliegue

- Dron = caja negra.
- Separar ownership semántico, autoridad/control y deployment source/profile.
- Prohibir carga directa de YAML entre grupos.
- Permitir réplicas parciales declaradas con claves exactas.
- Permitir réplica completa únicamente como deployment profile declarado; `global_map` Servidor↔Simulación se conserva completo y con igualdad guardada.
- `body_T_camera` es intrínseco Dron; transporte futuro Dron→Servidor mediante TF/calibración, no servicio nuevo en Fase 2.
- Parámetro futuro controlado por Servidor y consumido por Dron: cliente Dron al arrancar → servicio Servidor → valor local.
- Standalone Dron/Servidor: `use_sim_time=false`; Simulación: override `true`.
- Eliminar `usar_veltrap` tras búsqueda global: `TrayAction.tipo_trayectoria` es la autoridad.
- Corregir la masa Xacro para usar la masa configurada, no literal `1.0`.
- Mantener el fiducial actual sin refactorizar; Fase 4 lo sustituirá.
- Mantener GT de control por ahora; Fase 5/5H lo sustituirá.
- ORBvoc normal = vocabulario completo; preparar bootstrap/preflight reproducible desde el recurso versionado. No sustituir silenciosamente por L5.

### Observabilidad

- `pipeline_flow` y `system_architecture` independientes.
- Debug maestro false = sin bridge, navegador, HTTP/SSE, observers, publishers específicos, serialización ni generación de eventos.
- La deuda actual de `pipeline_flow`: el bridge se apaga, pero `GlobalMapServer` todavía genera `/global_mapping/flow_events`; debe corregirse en productores.
- `system_architecture`: paquetes como nodos; capas runtime/build/config/deployment; solo runtime ilumina; evidencia directa; evento desconocido no mapea a ninguna arista.
- `system_architecture` tendrá telemetría ligera propia y no dependerá de `/global_mapping/flow_events`.
- Con web=false queda totalmente dormido; web=true+telemetry=false permite solo grafo estático; web=true+telemetry=true permite live.

### Herramientas

- Logs completos solo para reductores; agentes nunca los abren.
- Artefactos ROS en `build/install/log/<grupo>`.
- `codex/archivos_auxiliares` permitido para evidencias, nunca dependencia runtime.
- `run_simulation.sh`: conservar `setsid`, preparar entorno explícito y usar `bash -c` en vez de `bash -lc` salvo evidencia en contra.

## system_architecture futuro

Regla permanente: cualquier subfase que cambie paquetes, interfaces, relaciones cross-group, réplicas o deployment actualiza topología/metadata/tests y telemetría live si aplica. Referencias explícitas ya añadidas a 4E/4F/4H/4K y 5A/5B/5D/5E/5H/5I. En Fase 5, Servidor↔Dron cruza únicamente por `orbslam3`.

## Próximo paso

1. verificar que este cierre documental queda coherente en GitHub;
2. recibir autorización del usuario;
3. implementar las correcciones acordadas;
4. guardas + builds aislados + tests + smoke debug-off + visualizadores separados + regresión equivalente a 198;
5. actualizar historial/contexto y cerrar Fase 2.
