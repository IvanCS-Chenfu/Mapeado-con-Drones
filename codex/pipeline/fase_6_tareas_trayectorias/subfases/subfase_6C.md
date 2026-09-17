# Subfase 6C - Servicios de comando y resultado desacoplados

## Estado

`PARCIAL`.

Las interfaces y handlers de aceptacion inmediata estan implementados y
compilan en ambos extremos. La ejecucion local de la orden y el envio real de
su resultado se conectaran al runtime en la siguiente migracion.

## Objetivo

Reemplazar para el ciclo autonomo de Fase 6 la relacion larga servidor-dron
por dos servicios breves y correlacionados.

## Contrato funcional

```text
task_server --SubmitAutonomousCommand--> task_manager
task_manager --ReportAutonomousResult--> task_server
```

`SubmitAutonomousCommand` contiene orden, IDs, epoch, objetivo visual, plan si
existe y politica de captura. El handler del dron solo valida, deduplica,
encola localmente y responde `accepted` de inmediato.

`ReportAutonomousResult` contiene el mismo contexto, estado terminal, pose
final, observaciones depth y motivo. El handler del servidor persiste el
resultado y encola el trabajo posterior; no integra voxeles ni calcula D* en
la llamada.

Ordenes normales iniciales: `MOVE_AND_CAPTURE`, `LOOK_AND_CAPTURE` y
`LOOK_FIDUCIAL`. STOP permanece local, asincrono y prioritario.

## Cambios requeridos

- Crear interfaces `mission_msgs` de comando y resultado.
- Implementar confirmacion idempotente de aceptacion y reintento seguro.
- Mantener resultados pendientes localmente si el servicio servidor no existe.
- Conservar el uso interno de `gen_tray`; no obliga a sustituir su action local.

## Exclusiones

- No migrar mandos externos legacy ni el pipeline fiducial previo.
- Un servicio no puede conservar abierta una maniobra larga.

## Validacion y exito

Un comando aceptado no bloquea el worker emisor. Reenvios del mismo comando y
resultado no duplican movimiento ni evidencia. El servidor conserva solo el
ownership minimo del workflow.
