# Resumen 6N

Estado agregado: PARCIAL.

La migracion al barrido lateral de fachada esta implementada y compila. Los
tests dirigidos vigentes son correctos. La prueba 737 valido que una inspeccion
automatica ya no sustituye una trayectoria fisica externa activa. Tambien se
corrigieron la barrera de asignacion durante una interrupcion fiducial y la
propagacion del motivo real de fallo de `InspectFacade`.

La prueba 738, interrumpida por el usuario, confirmo el nuevo motivo
`target_capture_failed:exact_frame_not_buffered` y descubrio un bloqueo por la
cola global `ready_drones_`: D2 puede retener el frente e impedir que D1 avance.
Todavia no se ha validado de extremo a extremo la doble captura, evidencia
FREE, D*, reserva, ejecucion, coverage lineal ni `TO_FINISH`.

Antes de concluir 6N hay que resolver el arbitraje por dron y la conservacion
del frame exacto, aislar D1 en el escenario y repetir la prueba integrada.
