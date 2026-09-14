# Historial - 7G

## 2026-09-08 - Tramo habilitado por 6G

- `RosDataBridge` consume el `TrajectoryPlan` autoritativo de
  `/mission/planned_routes`.
- `GuiDataModel::ReplaceTrajectory` conserva un unico plan previsto por dron y
  lo reemplaza al cambiar `trajectory_id`; el bridge registra
  `GUI-PLANNED-ROUTE-UPDATE` sin volcar todos los puntos.
- La layer pasa a llamarse `Plan previsto` para no confundir la polilinea D*
  con la futura trayectoria continua que 6I entregara al ejecutor.
- La prueba 615 confirmo el reemplazo visual de `dstar_1_1` por `dstar_1_2`.

Conclusion: PARCIAL. La GUI no reconstruye desde poses ni inventa curvas, pero
aun no existe una trayectoria fisica continua real que pueda representar.

## 2026-09-10 - Lifecycle de trayectoria ejecutable

- La GUI interpreta `PLANNED`/`ACTIVE`/terminal del `TrajectoryPlan`: registra
  `GUI-TRAJECTORY-UPDATE` para la capa vigente y la limpia con
  `GUI-TRAJECTORY-CLEAR` al completar, cancelar o rechazar.
- Build correcto y CTest 9/9 con overlays ROS completos. En 638 la GUI recibio
  `GUI-TRAJECTORY-UPDATE` para `dstar_1_1` en estado `ACTIVE`, pero la
  preempcion del goal legacy de D1 fallo por doble finalizacion en `gen_tray`.
  El `scenario_runner` termino con codigo 250 y el helper cerro el launch, por
  lo que no hay evidencia visual persistente del tramo en vuelo. 637 fue
  invalida de arranque y 639 de cierre externo.

Conclusion: PARCIAL. El consumidor de lifecycle existe y sus tests pasan; falta
corregir la preempcion de 6I y realizar la prueba Gazebo+F7 de una accion que
complete realmente.

## 2026-09-12 - Solo trayectoria activa en la capa principal (prueba 679)

- El bridge ignora `PLANNED`; la capa principal acepta exclusivamente
  `ACTIVE`. Los terminales hacen clear condicionado por `trajectory_id`, por lo
  que un evento atrasado no puede borrar otra ruta visible.
- Build de `multidron_gui_lib` correcto y CTest 9/9, incluida la regresión de
  identidad en `GuiDataModel`.
- En 679, D1/GT ejecutó coverage y el escenario terminó `SIM-DONE success=true`.
  Candidatos pendientes produjeron `GUI-TRAJECTORY-IGNORE` mientras la ruta
  activa seguía visible; la GUI solo actualizó al recibir `ACTIVE`. Los clears
  duplicados de STOP mostraron `cleared=false` sin borrar una línea nueva.

Conclusión: CONSEGUIDA para la representación de la trayectoria ejecutada. 7G
global conserva estado PARCIAL por los demás alcances de fase.
