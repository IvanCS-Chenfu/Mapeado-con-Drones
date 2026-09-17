# Subfase 6D - Runtime local, STOP y TRACKING_RISK

## Estado

`PARCIAL`.

## Objetivo

Convertir cada orden aceptada en ejecucion local autonoma del dron, con una
sola orden normal activa y seguridad visual independiente del servidor. El
servidor recibe la aceptacion inmediata y no conserva una espera remota durante
el movimiento, la captura ni una correccion visual.

## Contrato funcional

- `SubmitAutonomousCommand` admite `MOVE_AND_CAPTURE`,
  `LOOK_AND_CAPTURE` y `MOVE_AND_WATCH_FIDUCIAL`. Su respuesta solo confirma
  que el dron guardo la orden correlacionada; no espera su ejecucion.
- Al iniciar la orden, el runtime local toma poses/epoch actuales, ejecuta la
  misma cadena `TrayAction`/`gen_tray` que usa una trayectoria ordinaria y
  nunca crea un controlador paralelo.
- STOP captura pose actual, genera la retencion normal hacia esa pose y
  sustituye la orden normal por generacion. Dos causas concurrentes comparten
  la retencion; solo la primera reemplaza el movimiento fisico.
- `TRACKING_RISK` ejecuta STOP y su correccion local, captura depth y cierra la
  orden como `RESULT_ABORTED`. Si existe depth valido, este se publica igual:
  el estado abortado es diagnostico, no una orden de desechar evidencia.
- `MOVE_AND_WATCH_FIDUCIAL` no detecta ni activa AprilTags. Durante la ruta el
  dron observa el flag expuesto por Fase 4 y devuelve exclusivamente
  `fiducial_seen`; identidad y observacion concreta permanecen en el canal de
  Fase 4.
- Al final, normal, abortado por riesgo o rechazado, el dron llama una unica
  vez a `ReportAutonomousResult` con el mismo `command_id`, pose final, epoch,
  motivo, depth disponible y el flag fiducial.

## Cambios requeridos

- Cola local de comandos, runtime por orden y deduplicacion de terminales.
- Adaptar el STOP visual/depth para que pueda reemplazar tambien una orden
  autonoma, no solo una goal legacy de `ExecuteTrajectory`.
- El ingreso de resultados de `task_server` debe enrutar
  `MOVE_AND_WATCH_FIDUCIAL` a su continuacion de fiducial y permitir que
  `RESULT_ABORTED` con observaciones pase por `DepthIntegrationWorker`.
- Mantener `InspectFacade` legacy aislado: no puede alimentar el nuevo worker
  de depth durante esta migracion.

## Exclusiones

- El dron no decide subROI, coverage, D* ni mapa global.
- Esta subfase no migra todavia los consumidores de `POINT_SELECTION` ni
  `TRAJECTORY_PLANNING`. Por ello la prueba integrada distingue el flujo legacy
  de cobertura de la validacion directa y correlacionada del runtime 6D; no se
  debe afirmar que una orden 6D fue producida por esos workers hasta migrarlos.

## Validacion y exito

STOP de mapa y `TRACKING_RISK` simultaneos producen un solo terminal. Una orden
vieja no completa un workflow de nueva generacion y el dron reporta resultado
aunque el servidor procese otras colas. La prueba acordada mueve D1 con GT a
fiducial 2 `(0,-10,1)`, yaw `90 deg`, y mantiene GUI/Gazebo 180 s: se conserva
el log y solo se reduce el cierre minimo salvo incidencia visual comunicada por
el usuario.

## Cierre actual

El runtime local, el terminal correlacionado y el tratamiento de
`RESULT_ABORTED` con depth valido estan implementados, compilados y cubiertos
por CTest de `task_server`. La prueba 771 recupero el flujo GT legacy al
habilitar depth, pero no genero una orden autonoma ni una captura de fachada:
`POINT_SELECTION` aun no tiene productor. Esta subfase queda parcial hasta que
6H y 6I generen y despachen esas ordenes.
