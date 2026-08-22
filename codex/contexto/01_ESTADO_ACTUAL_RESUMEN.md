# Estado actual - resumen

## Situacion

```text
Fase actual: Fase 2 - separacion servidor/dron/simulacion
Fase 3: CONSEGUIDA
3B-3Q: CONSEGUIDAS
3R: CONSEGUIDA; scoring raw/fused validado tecnica y visualmente
3S: CONSEGUIDA; observabilidad configurable y modo silencioso validado
3T: CONSEGUIDA; limpieza, configuracion y handoff validados
```

## Entrega de Fase 3

- mapa sparse global multi-dron en `world`, organizado por
  `(drone_id, map_epoch)`;
- flujo principal independiente del solver y una cola secundaria priorizada con
  una sola tarea activa;
- fiduciales hard, loops geometricos, optimizacion validada y commit atomico;
- fused landmarks transitivos, score centralizado y publicacion con score/rgb;
- configuracion global separada por despliegue y protegida contra divergencias;
- debug de Fase 3 gobernado por cuatro booleanos en
  `simulacion_dron/config/fase3_debug.yaml`.

## Cierre 3R-3T

3R conserva la base ORB por observacion, aplica factores recuperables de
distancia/aislamiento e inliers y calcula cada fused score como
`media(raw miembros) + 0.04*N`. La banda neutra dependiente del baseline fue
validada visualmente; el usuario confirmo que los scores eran correctos.

3S permite activar de forma independiente RViz2, grafo web, apertura del
navegador y logs terminales. Con `fase3_logs_terminal=false`, el servidor usa
nivel ROS `error`: desaparece la telemetria `[F3*]`, pero se conservan errores
reales. Ningun flag desactiva el mapa ni su procesamiento.

3T retiro legacy y contratos absorbidos, creo el ADR 0009 y los perfiles YAML
por despliegue, actualizo manifests/launches y dejo el handoff reproducible.
Los historiales absorbidos se conservan en `historial/absorbidas/`.

## Evidencia final

- build: 3/3 paquetes;
- CTest: `orbslam3_multi` 9/9, `orbslam3_server` 10/10 y
  `simulacion_dron` 8/8;
- prueba 195: `success=true`, colas 0/0, 11 commits loop, cero hard failures y
  23.978 puntos con score/rgb; RViz2 revisado por el usuario como correcto;
- prueba 196: `success=true`, cinco pasos y cuatro goals correctos, servidor
  activo, cero `[F3*]`, RViz2/web a 0 MiB y sin navegador;
- el unico error de 196 fue el exit 255 conocido de Gazebo durante el cleanup,
  posterior a `SIM-DONE` y sin impacto funcional.

## Mejora futura 3Q

La deformacion de la prueba 194 se conserva en el historial, aunque no se
reprodujo en 195. Se documenta, sin implementar, una politica adaptativa de
evidencia: dos apoyos independientes para candidatos cercanos y hasta 8-10
para candidatos lejanos o ambiguos antes de lanzar una unica optimizacion.
Esta mejora no bloquea el cierre de Fase 3.

## Referencias

```text
codex/pipeline/fase_3_sparse_global/RESULTADO_FINAL_FASE_3.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3S.md
codex/pipeline/fase_3_sparse_global/subfases/subfase_3T.md
codex/pipeline/fase_3_sparse_global/historial/INDEX.md
```
