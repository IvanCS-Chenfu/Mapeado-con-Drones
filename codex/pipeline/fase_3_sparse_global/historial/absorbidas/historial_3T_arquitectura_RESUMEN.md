# Historial 3T - resumen

## Estado vigente

`CONSEGUIDA` por auditoria tecnica y aceptacion del usuario.

## Evidencia clave

- dos workers persistentes: uno principal y uno secundario;
- publicacion de nube/KFs exclusiva del principal;
- autoridades separadas para raw, poses, covisibilidad, fusion y score;
- calculo secundario sobre propuestas privadas y commits revisionados;
- raw inmutable ante fusion/optimizacion y dirty sets incrementales;
- sin rutas activas de scheduling antiguo ni espera visual funcional;
- builds, tests y simulaciones previas hasta 194 validan progreso, serializacion
  secundaria, atomicidad y cierre de colas.

## Decision de cierre

El usuario considera muy bueno el rendimiento actual y no quiere modificar mas
la sincronizacion. La ausencia de un test monolitico y las mediciones adicionales
de stress quedan en 3V/3W, no como deuda funcional de 3T.

Detalle: `historial_3T.md`.
