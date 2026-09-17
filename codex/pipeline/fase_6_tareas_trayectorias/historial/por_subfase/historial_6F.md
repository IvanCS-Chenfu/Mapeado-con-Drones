# Historial - 6F

## 2026-09-07 - Implementacion y validacion

- `task_manager` consume tareas y publica `TaskReport` para confirmar
  asignaciones, sin ejecutar planes ni emitir `COMPLETED`.
- La deduplicacion se reforzo por `task_id` y `state_revision` y el rebuild
  final de `task_manager` fue correcto.
- Prueba 605: dos confirmaciones locales correctas y movimiento exclusivo del
  escenario de prueba hacia `(0,-10,Z)` con yaw 90 grados.

Conclusion: CONSEGUIDA para la base autonoma acordada.

## 2026-09-16 - Migracion depth desacoplada y fuentes correlacionadas

- objetivo intentado: migrar la integracion depth autonoma a colas, base de
  evidencia y continuaciones por fuente aplicada, sin conectar `InspectFacade`.
- archivos modificados: `evidence_pipeline.{hpp,cpp}`, `task_server_node.cpp`,
  `test_evidence_pipeline.cpp`, contrato 6F y resumen de `task_server`.
- comportamiento: `ReportAutonomousResult` persiste el payload por
  `command_id` antes de encolar `DEPTH_INTEGRATION`; el worker no espera al
  dron ni a la pose KF. Las observaciones quedan locales al KF y una pose tardia
  provoca su materializacion incremental. `vista_pared` produce FREE,
  DIRECT_FREE y OCCUPIED directo; `vista_unknown` solo FREE. La continuacion se
  libera al aplicar todos sus `source_id`.
- paquetes compilados: `task_server`.
- resultado de build: correcto, exit 0; solo avisos deprecados heredados de
  `create_service`.
- pruebas: CTest 9/9 correcto. Los GTests cubren pose tardia, evidencia wall
  FREE/OCCUPIED, evidencia unknown solo FREE y reprocesado de fuente existente.
- evidencia negativa: no se ejecuto Gazebo porque 6D aun no despacha comandos
  autonomos reales; `InspectFacade` queda aislado por acuerdo.
- conclusion: PARCIAL.
- siguiente paso recomendado: conectar runtime local 6D y ejecutar una
  simulacion de resultado depth autonomo extremo a extremo.

## 2026-09-17 - VIEW_ADVANCE para prefijo y respaldo FREE (pruebas 778/779)

- objetivo: impedir que el avance al ultimo FREE anterior a UNKNOWN complete
  sin profundidad y reabra otra inspeccion inmediatamente.
- cambios: `DepthInspectionKind::VIEW_ADVANCE` puede materializar FREE y
  OCCUPIED frontal fiable, sin activar coverage U. Tanto el prefijo de D* como
  `fallback_free_advance` capturan y continúan a `POINT_SELECTION` solo tras
  `sources_applied`.
- build y tests: `task_server` compiló; CTest `9/9`, incluida la regresion de
  evidencia `VIEW_ADVANCE`.
- prueba 778: NO CONSEGUIDA. El respaldo respondía `depth=0` e
  `invalid_depth_job`, porque se enviaba sin captura.
- prueba 779: CONSEGUIDA para la corrección. Los avances respondieron
  `depth=1`, se registraron como `kind=view_advance` y esperaron
  `F6F-DEPTH-SOURCES-APPLIED` antes de reseleccionar.
- límite: STOP sigue produciendo terminales `replaced_by_stop` y las miradas
  UNKNOWN mantienen con frecuencia el destino UNKNOWN.

Conclusión: el contrato de captura/materialización de `VIEW_ADVANCE` queda
CONSEGUIDO; 6F agregado permanece PARCIAL.
