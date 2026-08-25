# Historial de Fase 4

| Subfase | Estado agregado | Punto de entrada |
|---|---|---|
| 4A | CONSEGUIDA | `por_subfase/historial_4A_RESUMEN.md` |
| 4B | CONSEGUIDA | `por_subfase/historial_4B_RESUMEN.md` |
| 4C | CONSEGUIDA | `por_subfase/historial_4C_RESUMEN.md` |
| 4D | CONSEGUIDA | `por_subfase/historial_4D_RESUMEN.md` |
| 4E | CONSEGUIDA | `por_subfase/historial_4E_RESUMEN.md` |
| 4F | CONSEGUIDA | `por_subfase/historial_4F_RESUMEN.md` |
| 4G | CONSEGUIDA | `por_subfase/historial_4G_RESUMEN.md` |
| 4H | CONSEGUIDA | `por_subfase/historial_4H_RESUMEN.md` |
| 4I | Aplazada; opcional futura | contrato en `subfases/subfase_4I.md` |

Las pruebas 201/202 cierran 4A+4B. Los intentos 203-207 preservan fallos de
arranque, invocacion y aislamiento HighGUI. La 208 completa el escenario con
visualizador separado, timeouts correctos y wrappers vivos. El usuario acepta
la 208 y cierra 4C+4D.

La 209 conserva el fallo de ruta relativa del bloque 4E+4F. La 210 completa la
trayectoria con 68/68 matches y la 211 confirma ambos grafos live con 18/18
matches adicionales. 4E+4F quedan conseguidas.

La repeticion visual 212 incorpora seis yaw relativos, pero no llega a
ejecutarlos: un loop activa `commit_pose_store_hard_constraint_violation` y el
mission gate bloquea antes del paso 5. El intento se conserva en 4B y el cambio
angular queda pendiente de revision visual.

La correccion posterior elimina el latch persistente de fallos secundarios. La
prueba 213 completa 17/17 pasos y 22/22 goals, con liberacion normal del gate y
74/74 eventos visuales. El usuario da 4A-4F por concluidas, pero observa derivas
no corregidas y remite la calidad de optimizacion de la prueba 213 a una nueva
revision de 3Q.

Las pruebas 214/215 implementan y refinan la interpretacion visual 4G. Tras
retirar completamente GT fiducial, la 216 completa la regresion con los tres
objetos y la 217 valida ambos grafos live. 4G+4H quedan conseguidas.

El usuario cierra la Fase 4 completa con el alcance 4A-4H. La regresion 4I con
perfil ESP32-CAM queda aplazada para otro momento y no condiciona este cierre.
