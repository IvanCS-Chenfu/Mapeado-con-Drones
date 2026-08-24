# 00 - Contexto de compactacion

## Estado vivo

```text
Estado: Fase 2 CONSEGUIDA; Fase 3 CONSEGUIDA; Fase 4 ACTUAL SIN EJECUTAR
Objetivo vigente: ninguno; cierre publicado y Fase 4 activada
Preparacion de Fase 4: NO_INICIADA
Acuerdo de Fase 4 cerrado: no
Autorizacion funcional de Fase 4: PENDIENTE
Prueba acordada de Fase 4: ninguna todavia
Dudas abiertas: ninguna
Trabajo activo: no
Pendiente: preparar 4A solo tras nueva peticion
```

## Acuerdo ejecutado

```text
mi_tfg se conserva como legacy fuera de los grupos
ORB_SLAM3_MULTI y fase45_sandbox se retiraron
orbslam3_msgs se duplica; Servidor es canonico y Dron replica exacta
ADR 0009 gobierna ownership y replicas YAML
cada build selecciona exactamente un paquete
debug oficial: RViz2, pipeline_flow, system_architecture, navegadores,
telemetria arquitectonica y logs F3 activos
todos los flags conservan default false
```

## Implementacion

- grupos fisicos `dron`, `servidor` y `simulacion` con distribucion 5/3/1;
- bases `build/install/log` independientes por grupo;
- YAML por dominio, `use_sim_time` explicito y Xacro conectado a la masa configurada;
- ORBvoc completo preparado en `build/dron/_phase2_resources` e instalado;
- `pipeline_flow` lazy-gated antes de serializar;
- `system_architecture` declarativo, estatico/live y con topic propio;
- guarda de layout, interfaces, dependencias, config, paths, visualizers y docs.

## VALIDACIÓN FINAL

```text
build: 9/9 invocaciones correctas, un paquete por invocacion
CTest: lib_tray 4/4, orbslam3_multi 9/9, orbslam3_server 10/10,
       simulacion_dron 9/9
rebuild de cierre: simulacion_dron 1/1; CTest 9/9
dron_individual: deuda legacy global de linters; archivos tocados correctos
guardas funcionales: todas correctas
guarda final completa: 15 checks, 0 fallos
layout: capturas 1440x900 y 820x1000 inspeccionadas sin solapes
git diff --check: correcto
```

Prueba 199:

```text
smoke debug-off: 5/5 pasos, 4/4 goals, success=true, exit 0
RViz/web 0 MiB, sin marcadores de debug especifico
guard_triggered=false
```

Prueba 200, regresión equivalente a 198:

```text
vuelta oficial: 14/14 pasos, 20/20 goals, success=true, exit 0
RViz2, ambos bridges y navegadores activos
system_architecture live abre con latest_sequence=1
guard_triggered=false; minimo MemAvailable 3873.8 MiB
bridges, RViz2, wrappers y servidor cierran limpiamente
revision visual humana: confirmada correcta por el usuario el 2026-08-24
```

Despues de `SIM-DONE`, `gui_tray_multi` emitio un traceback de shutdown de
`rclpy` y Gazebo termino con el exit 255 conocido. No afectaron al escenario.
El antiguo `ValueError` de cleanup de `system_architecture_bridge` no se
reprodujo.

## Fuente de verdad

```text
codex/pipeline/fase_2_separacion_paquetes/RESULTADO_FINAL_FASE_2.md
codex/pipeline/fase_2_separacion_paquetes/pipeline_fase_2_RESUMEN.md
codex/pipeline/fase_2_separacion_paquetes/historial/INDEX.md
codex/archivos_auxiliares/logs/prueba_199.reduced.log
codex/archivos_auxiliares/logs/prueba_200.reduced.log
```

La auditoria contra `main` confirma que el commit publicado es `4424a586`.
La implementacion sigue sus decisiones materiales, pero los contratos locales
de 2C-2G deben reconciliar clausulas definitivas eliminadas durante el cierre.
Plan autorizado: layout de referencia, guarda de posiciones, contratos 2C-2G,
build/test/guardas y capturas desktop/viewport estrecho; sin simulacion larga.
Layout implementado en `graph_layout.js`, cargado por `index.html` y aplicado
por `app.js`. El test contractual comprueba cobertura de paquetes y relaciones
espaciales sin fijar pixeles exactos.
Contratos 2C-2G reconciliados con las decisiones definitivas publicadas; 2C
vuelve a distinguir réplica parcial y `deployment profile` completo. Docs de
`simulacion_dron` actualizados con el layout declarativo.
Build siguiente: `./codex/herramientas/build_selected_packages.sh --group simulacion simulacion_dron`.
Motivo: instalar el nuevo asset web y ejecutar la suite contractual del único
paquete afectado, reutilizando los prefijos ya validados de Dron y Servidor.

Build layout 2026-08-24: exit 0; `simulacion_dron` 1/1 correcto.
Log: `codex/archivos_auxiliares/colcon_build.log`.
Siguiente accion exacta: ejecutar CTest de `simulacion_dron` y las guardas.

CTest intento 1: no ejecutado, exit 8 porque el sandbox impidio escribir
`Testing/Temporary/LastTest.log` bajo `build/simulacion`; no es un fallo de test.
CTest intento 2: 9/9 correctos, incluido `system_architecture_contract` con la
nueva guarda de layout.
Guarda completa intento 1: 14 checks correctos y 1 fallo de paths. CTest genero
dos `__pycache__` dentro de `simulacion_dron`; no hay incumplimiento funcional.
Artefactos retirados. Guarda completa intento 2: 15 checks, 0 fallos.
Siguiente accion exacta: capturas web desktop y viewport estrecho, seguidas de
inspeccion visual.

Servidor estatico intento 1: no arranco; el sandbox bloqueo la apertura del
socket local con `PermissionError`. Siguiente accion: repetir con permiso local.
Servidor estatico intento 2: activo en `127.0.0.1:8765`.
Chrome headless intento 1: exit 133 sin captura bajo sandbox.
Chrome headless con permiso: capturas desktop 1440x900 y estrecha 820x1000
generadas e inspeccionadas; contenedores completos, texto legible y sin solapes.
Servidor temporal detenido limpiamente.
Siguiente accion exacta: cierre documental y verificacion final de coherencia.

Cierre documental aplicado. Guarda posterior a toda la documentacion: 15/15.
Asset `graph_layout.js` instalado en el prefijo aislado de Simulacion. No quedan
estados pendientes vigentes y `git diff --check` es correcto.
Guarda tras el checkpoint final: 15/15, 0 fallos.
Commit principal de cierre creado correctamente en `main`.
Push intento 1: rechazado por `non-fast-forward`; `origin/main` contiene cambios
nuevos y no se usara force push.
Fetch correcto: remoto añade `63d677a` y `4424a58`, ambos documentales.
Rebase completado sobre `origin/main`; 26 conflictos documentales resueltos con
autorización explícita conservando las versiones finales ya reconciliadas.
Guarda posterior al rebase: 15/15, 0 fallos.
Commit principal publicado: `d189b0e` en `origin/main`.
Siguiente accion exacta: ninguna; esperar preparación solicitada de 4A.
Fase 4 queda actual, pero ninguna subfase puede ejecutarse sin preparación y
autorización explícitas.
Guarda tras activar Fase 4: 15/15, 0 fallos.
