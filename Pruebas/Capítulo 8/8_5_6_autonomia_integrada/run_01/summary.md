# 8.5.6 / run_01 - Resumen

## Resultado

**NO CONSEGUIDA.** El escenario alcanzó el handoff, asignó `map_section_level_0_AB` a D1 y `map_section_level_0_BC` a D2. D1 ejecutó `LOOK_AND_CAPTURE`, comprometió una reserva D* de 195 vóxeles y avanzó, pero `task_server` terminó con `malloc(): unaligned fastbin chunk detected` (exit code `-6`) antes del progreso verificable de D2.

## Diagnóstico y corrección posterior

Los callbacks de evidencia sparse, integración depth, materialización voxel y workflow mutaban `EvidenceDatabase` y estado asociado desde callback groups distintos en un executor multihilo. Se serializaron esos callbacks en `map_callback_group_`, se compiló `task_server` y `run_02` ya no reprodujo el aborto.

Este intento se conserva como evidencia de fallo y no se usa para concluir la prueba.
