# 01 - Estado actual del proyecto

Leer primero `00_CONTEXTO_COMPACTACION.md` y
`01_ESTADO_ACTUAL_RESUMEN.md`.

## Estado vivo

```text
Fase activa: Fase 3 - mapa sparse global multi-dron
3B-3O: CONSEGUIDAS
3P: CONSEGUIDA; CIERRE FUNCIONAL Y VISUAL CONFIRMADO
Ultimas ejecuciones: 159 fallida, 160 corregida y 161 ajuste de retry/visibilidad
```

## Arquitectura

El flujo principal conserva raw, poses, score, builder incremental y
publicacion serial. El worker secundario global ejecuta, sin preemption,
fiducial MAX, `DatabaseUpdateTask` MEDIA y `LoopTask` BAJA.

La MEDIA compromete un patch de covisibilidad y crea las BAJAS. Cada BAJA
coalesce por huella semantica, revalida geometria exacta, actualiza el indice BoW derivado, agrupa regiones y ejecuta
subnubes/matching/RANSAC. Una fusion compatible continua en 3P con tracks,
covisibilidad server y score; el error alto se reporta para 3Q. El anchor loop
confirmado sigue modificando poses mediante un batch atomico.

`GlobalPoseStore` mantiene dependencias blandas padre-hijo para anchors loop.
El movimiento del apoyo propaga rigidamente el hijo. Su primer fiducial directo
reancla todo el submapa, corta la dependencia y queda hard. El
worker secundario solo marca KFs dirty; el siguiente principal reproyecta MPs y
publica.

## Invariantes

- `submapa=(drone_id,map_epoch)`;
- raw y BoW original son inmutables para loops/optimizacion;
- un unico worker secundario, tarea activa no interrumpible;
- commit atomico, sin estado parcial ante stale/conflicto;
- sin GT para BoW, matching, RANSAC ni anchor loop;
- 3P no aplica optimizacion por loop ni modifica raw/poses.

## Validacion

El primer replay 152 completo la entrada pero dejo unas 384 tareas secundarias,
por geometria intra-submapa redundante y loops de unos 2.4 s; se conserva como
`PARCIAL`. Tras acotar subnubes/iteraciones y aprovechar covisibilidad fuerte,
replay 153 procesa 806 tareas y termina vacio, con latencias finales de
0.16-0.18 s y cero fallos duros.

Prueba 157 valida 78 KFs hijos propagados en el mismo commit del padre. La
tipica 156 valida reanchor post-loop de 32 KFs, tres commits fiduciales, cola
final vacia y cero fallos duros. La huella final reduce la carga de 9.20 a 2.18
tareas/KF. ORB crea siete submapas; cuatro quedan anclados y tres diferidos.

## Validacion 3P y punto de reentrada

La prueba 159 aborto el servidor por un track absorbido que permanecia en
`touched_tracks`; se conserva como `NO CONSEGUIDA`. La correccion añade
validacion local, regresion exacta y barrera de excepciones del worker.

La prueba 160 completa 56 commits, cinco stale y un rollback; drena 1116 tareas
sin hard failure y el servidor termina limpio. `GlobalMapBuilder` consume
tracks hasta la ultima publicacion. El usuario confirma que RViz2 y el grafo
web se vieron muy bien.

El ajuste posterior elimina los objetivos temporales de commit/visibilidad.
La prueba 161 completa `56/56` regiones, encola y termina los 19 retries
necesarios, aplica ocho fusiones y cuatro optimizaciones fiduciales full y
cierra `pending=0`/cero hard. El prepare aceptado sube a
`633.852/1087.130 ms` de media/maximo, coste aceptado por ahora porque no
bloquea el escenario ni el drenaje. El usuario da por concluida 3P. Solo pide
reorganizar posteriormente el layout desktop del grafo web para mejorar su
legibilidad, sin señalar defectos de topologia o funcionamiento.
