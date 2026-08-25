# Resumen 4H

Estado agregado: **CONSEGUIDA**.

Cada primary visual entra en `FiducialAnchorManager` con KF/arrival exactos,
`world_T_camera_target`, visita visual y `source=visual_fiducial`. La ruta GT
fiducial fue eliminada de codigo, configuracion, subscriptions, replay y
grafos; el GT de control/Fase 5 permanece independiente.

La prueba 216 completa la trayectoria sin GT con 52/52 primary y los tres
objetos. El smoke 217 valida ambos grafos live. Los fallos
`loop_submap_window_too_small` pertenecen al backend y no bloquean la mision.
