# Pipeline Fase 2 — Resumen

## Estado

```text
Fase 2: EN CIERRE TÉCNICO
2A: estructura física conseguida en snapshot probado
2B: builds aislados conseguidos en snapshot probado
2C: parcial; correcciones finales de configuración pendientes
2D: prueba 198 pasada; regresión post-cambios pendiente
2E: documentación en cierre
2F: system_architecture existe; corrección semántica/live pendiente
2G: guardas existen; endurecimiento y regresión final pendientes
Autorización actual de implementación: PENDIENTE
Dudas funcionales abiertas: ninguna
```

## Objetivo

Dejar Dron, Servidor y Simulación como despliegues explícitos, aislables y reproducibles, con configuración sin dependencias cross-group ocultas, observabilidad opcional y guardas que impidan regresiones.

## Acuerdos de cierre

- Dron caja negra.
- Separar ownership, authority y deployment profile.
- Carga directa de YAML cross-group prohibida.
- Réplicas parciales declaradas permitidas con claves exactas.
- Réplica completa `global_map` Servidor↔Simulación permitida como deployment profile deliberado y guardado.
- `body_T_camera` pertenece a Dron; transporte futuro por TF/calibración.
- Server→Dron config futura mediante cliente Dron/servicio Servidor, no implementación en Fase 2.
- Dron/Servidor standalone `use_sim_time=false`; Simulación override `true`.
- Eliminar `usar_veltrap`; `TrayAction.tipo_trayectoria` es autoridad.
- Corregir masa Xacro para usar la masa configurada.
- ORBvoc completo con bootstrap/preflight reproducible.
- No refactorizar fiducial actual: Fase 4.
- No retirar GT de control aún: Fase 5/5H.
- `pipeline_flow` y `system_architecture` totalmente dormidos cuando sus debugs están apagados, incluidos productores.
- `system_architecture` muestra realidad actual; paquetes como nodos; solo runtime con evidencia directa ilumina.
- Logs completos solo para reductores.
- Artefactos colcon en `build/install/log/<grupo>`.

## Evidencia

- 9/9 builds aislados correctos en el snapshot probado.
- Tests principales correctos salvo deuda legacy global de lint en `dron_individual`, con checks focales/rebuild correctos.
- Prueba 197: smoke debug-off conseguido.
- Prueba 198: conseguida por validación funcional/visual del usuario.

Después de implementar las correcciones se ejecutará una regresión equivalente a 198.
