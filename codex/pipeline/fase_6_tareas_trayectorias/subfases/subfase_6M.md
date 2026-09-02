# Subfase 6M - Planificacion de observacion yaw, pitch y distancia

## Estado

```text
sin hacer
```

## Dependencia

1J, 6D, 6G-6I y 6L.

## Objetivo tecnico

Elegir como orientar cuerpo y rig para observar superficies, conservar tracking
y refrescar seguridad lateral despues de decidir la ruta XYZ.

## Orden cerrado

```text
D* Lite -> XYZ
coverage/view planner -> yaw + camera_pitch
trajectory generator -> estado continuo
```

Yaw/pitch no amplian el espacio de estados de D*. Para una superficie se busca
eje optico aproximadamente normal y plano de imagen aproximadamente paralelo,
sin sacrificar safety, tracking, clearance ni dinamica.

La distancia preferida es coste suave configurable y `A MEDIR`, no limite. El
planner controla antiguedad/confianza lateral e inserta miradas breves cuando
conviene; esto complementa, nunca sustituye, el depth local.

`camera_pitch` viaja en `TrajectoryPlan`; `dron_individual` ejecuta el joint.
Depth usa `Kref_T_C(current)`, que ya incorpora pitch, y se reintegra al mover
`W_T_KF` sin recalcular profundidad.

## Cambios requeridos

1. Crear candidatos/score de yaw, pitch y distancia sin pesos inventados.
2. Respetar limites fisicos, velocidad del joint y transformadas de 1J.
3. Integrar riesgo visual 6L y permitir cambiar orientacion sin abandonar XYZ.
4. Mantener freshness lateral y politica conservadora si falta observacion.
5. Incluir referencias/derivadas necesarias en plan reproducible W/O.
6. Medir calidad, distancia, lateral refresh y efecto en tracking.

## Limites

No exigir frontalidad perfecta, no controlar Gazebo desde task_manager y no
usar pitch como correccion ad hoc de mapas/fiduciales.

## Pruebas

Pared, suelo/techo, plano inclinado, pasillo, dos rutas con distinta calidad,
mirada lateral, limites de pitch y recuperacion solo cambiando orientacion.
Validar TF/depth/SLAM y movimiento suave con GUI+Gazebo+grafo.

## Criterio de exito

Orientacion y distancia mejoran observacion sin degradar seguridad/tracking;
pitch es fisico, reproducible y coherente con depth/KF.
