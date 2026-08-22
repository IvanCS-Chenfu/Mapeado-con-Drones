# 00 - Contexto de compactacion

## Estado vivo

```text
Estado: Fase 3 CONSEGUIDA; Fase 2 actual
Objetivo vigente: ninguno; cierre documental y tecnico completado
Preparacion: CERRADA
Acuerdo cerrado: si
Autorizacion funcional: CONCEDIDA Y CONSUMIDA
Prueba acordada: prueba corta 196 con perfil 3S completamente false
Dudas abiertas: ninguna
Trabajo activo: ninguno
```

## Resultado final

La numeracion activa de cierre es:

```text
3Q optimizacion -> 3R scoring -> 3S debug -> 3T limpieza/handoff
```

- 3R conserva el scoring raw/fused antes identificado como 3S;
- 3S incorpora `simulacion_dron/config/fase3_debug.yaml`;
- 3T es la limpieza/handoff ejecutada inicialmente como 3X;
- los historiales de auditorias provisionales se preservan en
  `historial/absorbidas/`;
- los marcadores de scoring vigentes son `[F3R-*]`; los logs historicos
  192-195 conservan `[F3S-*]`.

3Q queda conseguida para el cierre de Fase 3. La deformacion de la prueba 194
no se borra y permanece en su historial. La prueba 195 no la reproduce y el
usuario confirma el resultado visual correcto. Como mejora futura no
implementada, un candidato cercano podria exigir dos apoyos independientes y
uno lejano o ambiguo hasta 8-10 antes de una unica optimizacion.

## Implementacion 3S

`fase3_debug.yaml` define, todos inicialmente a false:

```text
fase3_rviz2
fase3_grafo_web
fase3_abrir_navegador_web
fase3_logs_terminal
```

`multi_dron.launch.py` valida el YAML, expone overrides homonimos y condiciona
RViz2, bridge y navegador. El navegador requiere tambien el grafo. El launch
del servidor acepta `log_level`; con logs false recibe `error`, por lo que se
ocultan los diagnosticos `[F3*]` sin ocultar errores reales. El procesamiento,
las colas y las publicaciones funcionales no se desactivan.

## Verificacion

- `py_compile` y flake8 de los launches tocados: correctos;
- contratos configuracion/web: 15/15; repeticion final 15/15 en 0.19 s;
- build `orbslam3_multi orbslam3_server simulacion_dron`: 3/3, exit 0;
- CTest: multi 9/9, servidor 10/10 y simulacion 8/8;
- launch instalado: cuatro argumentos 3S presentes con default false;
- prueba 196: `success=true`, cinco pasos y cuatro goals correctos;
- servidor operativo, backpressure observado y cierre limpio;
- cero marcadores `[F3*]` y ningun proceso RViz2/bridge/navegador;
- RSS maximo RViz/web 0.0 MiB, servidor 99.2 MiB y guarda inactiva;
- unico ERROR: exit 255 conocido de Gazebo durante cleanup posterior a
  `SIM-DONE`, sin impacto funcional.
- cierre documental: `git diff --check` correcto y sin referencias activas a
  `subfase_3X`, `historial_3X` ni marcadores actuales `[F3S-*]`.

El log completo `prueba_196.log` se preserva y nunca se leyo directamente. El
analisis uso exclusivamente su reducido y el resumen de recursos.

## Handoff

Fuente de verdad del resultado:

```text
codex/pipeline/fase_3_sparse_global/RESULTADO_FINAL_FASE_3.md
codex/pipeline/fase_3_sparse_global/pipeline_fase_3_RESUMEN.md
codex/pipeline/fase_3_sparse_global/historial/INDEX.md
```

La fase actual es Fase 2, separacion servidor/dron/simulacion. No queda trabajo
activo de Fase 3. No se creo un commit final; existe el checkpoint previo a la
limpieza:

```text
1b96a7a checkpoint: guardar estado de fase 3 antes de limpieza 3X
```

## Cambios ajenos preservados

No tocar ni atribuir a Fase 3 los cambios previos del usuario en:

```text
ORB_SLAM3/include/System.h
ORB_SLAM3/src/System.cc
orbslam3_ros2/src/stereo/stereo-slam-node.cpp
```

`fase45_sandbox/` permanece ignorado y fuera del alcance.
