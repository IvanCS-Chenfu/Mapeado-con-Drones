# `FiducialAnchorManager`

## Rol

Dominio puro que decide primer anchor, revisita coherente o necesidad de
optimizacion. No depende de ROS, Gazebo ni del origen concreto de la
observacion absoluta.

## Referencias

```text
orbslam3_multi/include/orbslam3_multi/fiducial_types.hpp
  -> FiducialObservation / FiducialProcessResult / FiducialProcessStatus
  -> rg -n "struct Fiducial|enum class FiducialProcessStatus"

orbslam3_multi/include/orbslam3_multi/fiducial_anchor_manager.hpp
  -> FiducialAnchorManager::{Evaluate,AcceptControl,GetLastAcceptedControl}
  -> rg -n "class FiducialAnchorManager|AcceptControl|GetLastAcceptedControl"

orbslam3_multi/src/fiducial_anchor_manager.cpp
  -> calculo de anchor, errores y estado por visita/submapa
  -> rg -n "FiducialAnchorManager::(Evaluate|AcceptControl)"
```

## Contrato

La observacion contiene KF, fiducial, `fiducial_visit_id`, timestamps,
`target_world_T_kf`, fuente y calidad. Para un submapa sin anchor calcula:

```text
world_T_local = target_world_T_kf * inverse(local_T_kf)
```

Si ya existe anchor, compara la pose de `GlobalPoseStore` con el target en
traslacion, rotacion completa y yaw. Dentro de umbral devuelve
`RevisitWithinThreshold`; fuera devuelve `OptimizationRequired` y una tarea
ligera que conserva target, visita, error y revision observada.

El estado separa el ultimo control aceptado de la primera identidad observada
por `(submapa, fiducial_visit_id)`. El primer KF reserva la candidatura aunque
su error requiera optimizacion: si ya esta dentro de umbral se promueve de
inmediato; si no, solo se promueve mediante `AcceptControl()` despues del
commit completo. Ningun KF posterior de esa visita puede adelantarse y
convertirse en hard mientras la tarea del primero calcula.

No deduplica una visita completa: dos KFs distintos se evalúan y pueden crear
dos tareas. La deduplicacion exacta pertenece a `SecondaryTaskQueue`.

`test/test_fiducial_anchor_manager.cpp` cubre primer anchor, revisitas en/bajo
umbral, visitas distintas, validacion de entrada y la carrera primer KF alto
seguido de otro coherente de la misma visita.

## Revision abierta post-3O

El backend registra actualmente el KF de anchor loop mediante `AcceptControl()`.
Por ello un primer fiducial posterior con error alto crea una optimizacion desde
un control blando. Tras live 154 esta politica queda marcada para cambio: el
fiducial absoluto debe sustituir la autoridad loop, no optimizar tomando su KF
como frontera fiable.
