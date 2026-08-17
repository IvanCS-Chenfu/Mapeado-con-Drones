# Historial 3Q - resumen

## Estado vigente

`PREPARACION CERRADA; IMPLEMENTACION PENDIENTE DE AUTORIZACION`.

La regresion absoluta/causal anterior se conserva, pero la reimplementacion
acordada ya no fija unilateralmente candidate ni excluye same-submap. Unifica
la optimizacion covisible fiducial/loop sobre el mismo builder, solver,
validator y commit.

## Acuerdo de reimplementacion

- loop relativo y fiducial absoluto usan un grafo SE(3) comun;
- ventana = subgrafo minimo de hard, tramos temporales, dependencias soft,
  loops/fusiones previos y covisibilidad confirmada;
- fusiones previas empiezan soft y se mide su residual;
- 30 % de controles es base ampliable por constraints fuertes;
- dos queries independientes y control de ambiguedad preceden al solver;
- no se diferencia inter/intra dron o submapa para decidir;
- inliers RANSAC viven en la tarea y alimentan fusion 3P tras accept;
- loop compromete inicialmente solo accept completo;
- fusion omitida conserva poses; stale/rollback crea BAJA fresca;
- hard no se mueve, raw no cambia y el secundario no publica;
- `stop_drones` se activa al entrar en optimizacion loop y permanece hasta
  terminar commit/fusion/task, aunque la tarea siga siendo BAJA.

## Validacion acordada

Tests deterministas, replay y diez escenarios Gazebo naturales cubren
autoridades hard/soft, uno/dos fiduciales, fusion previa, optimizacion fiducial
previa, concurrencia, repeticion y cadena de tres submapas. No se fuerza error
alto mediante offset. Cada prueba informa si ejercio realmente 3Q; cuatro casos
requieren revision RViz2/grafo web.

## Evidencia anterior que no se borra

- `prueba_74` movio 31 KFs y propago 92 por tratar un loop como absoluto;
- `prueba_75` acepto indebidamente un candidato posterior cercano y propago
  248 KFs;
- `prueba_76` valido filtros causales, pero no produjo accept positivo;
- esos intentos motivan constraints relativas, filtros causales y la nueva
  matriz de pruebas, pero no prueban la reimplementacion futura.

## Pendiente

Recibir autorizacion funcional posterior, implementar, compilar y ejecutar la
matriz. 3Q no puede marcarse conseguida sin accepts loop reales, regresion
fiducial/covisible, cola drenada y evidencia visual acordada.

Detalle: `historial_3Q.md`.
