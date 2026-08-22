# Resumen historico 3P

Leer `historial_3P.md` para cronologia, metricas y conclusiones de cada prueba.

## Estado agregado

```text
CONSEGUIDA; FUNCIONALIDAD Y REVISION VISUAL ACEPTADAS
Nucleo de fusion: tracks transitivos, score, covis server y rollback activos
Prueba 159: fallida por crash transitivo, conservada
Prueba 160: servidor/cola/builder y revision visual humana validados
Prueba 161: retry fresco y visibilidad completa validados tecnicamente
```

La propiedad vigente de `3P` es la rama de error bajo de una `LoopTask` cuya
geometria ya fue aceptada por 3O. Reutiliza los inliers/outliers de RANSAC para
fusion y score/visibilidad; no posee ingesta, scheduling, BoW, RANSAC,
optimizacion por loop, publicacion ni backpressure.

## Capacidad validada

`3P` consume evidencia `FUSION_CANDIDATE`, conserva `RawMapDatabase`, inserta
covisibilidad geometrica, une `RawMapPointId` en tracks transitivos y publica un
representante por track.

- `prueba_48`: `91` candidatos, `674` tracks y publicacion exacta
  `16259 - 3333 + 674 = 13600`.
- `prueba_49`: `90` candidatos, `691` tracks y publicacion exacta
  `18111 - 3000 + 691 = 15802`.
- Los candidatos de error alto no se fusionan ni crean una optimizacion desde
  `3P`.
- El usuario observo en RViz2 desaparicion/recolocacion de puntos compatible
  con fused tracks.

Estas dos pruebas quedan `CONSEGUIDAS` para la fusion focalizada.

## Regresion larga

`prueba_50-69` revelaron y corrigieron sucesivamente carga excesiva,
reintentos, revisiones demasiado amplias, starvation y publicacion atrasada.
Se conservan todos los intentos en el historial largo. Hitos utiles:

- `prueba_50-53`: se separo calculo geometrico de callbacks y se acoto la carga,
  pero persistieron timeouts y contencion.
- `prueba_58-59`: el runner termino goals sin cancelarlos; el backend seguia
  tardando y aparecieron submapas sin anchor.
- `prueba_62`: granularidad de revisiones y coalescencia mejoraron el drenaje.
- `prueba_68`: la publicacion alcanzo revision monotona final.
- `prueba_69`: el escenario y la revision final terminaron, pero la observacion
  humana mantuvo la conclusion `PARCIAL`.

La ultima prueba midio hasta `30.052 s` de espera de captura por
`live_state_mutex_`, liberacion del control antes del commit visible y `33`
MapPoints fallback sin KF world utilizable.

## Interpretacion vigente

Esos defectos no pertenecen enteros a `3P`:

- ingesta y `ChangeSet`: `3C/3G`;
- poses y KFs futuros: `3D`;
- publicacion: `3F`;
- worker, prioridad y ciclo de vida: `3K`;
- covisibilidad: `3M`;
- BoW y verificacion: `3N/3O`;
- fusion: `3P`;
- optimizacion por loop: `3Q`;
- scores: `3R`;
- contratos y observabilidad: `3T/3U`.

El runtime historico del 2026-08-05 ejecutaba toda la cadena en una `LoopTask` y llamaba a
`3P` al elegir fusion o despues de una optimizacion aceptada. `3P` calcula
sobre snapshots privados, compromete tracks/covisibilidad y solicita
publicacion sin esperar a RViz2. Esa solicitud y sus snapshots amplios son
evidencia del diseño anterior y no forman parte del contrato nuevo.

La preparacion cerrada del 2026-08-16 fijo `LoopPipeline` como autoridad de
decision, `FusionPatch` acotado, tracks transitivos, score por inliers y
visibilidad sparse simetrica, commit logico atomico y dirty sets consumidos solo
por el siguiente `PrimaryInput`. Esa reimplementacion ya esta activa.

## Evidencia runtime nueva

- `prueba_75/76` completan el escenario con el worker unico y publicaciones
  posteriores a `loop_fusion_commit`.
- `prueba_76` mantiene `84/84` tareas y publica hasta el post-run con backlog
  secundario, sin backpressure funcional.
- `test_fused_landmark_manager` continua en PASS.
- `prueba_159` no valida 3P: aunque el runner termino, el servidor aborto en la
  tarea 3043 por acceder con `.at()` a un track absorbido dentro del mismo
  patch. Antes del fallo hubo 21 commits.
- La correccion elimina tracks retirados de `touched_tracks`, valida referencias
  locales y añade una barrera de excepciones del worker y una regresion exacta.
- `prueba_160`: 62 intentos, 56 commits, cinco stale y un rollback correcto;
  1116 secundarias, `pending=0`, cero hard y servidor limpio.
- Los commits suman 3083 pares, tracks `(795,1900,76)`, 2050 miembros ocultos,
  score `+5932/-76` y covisibilidad `+92/~27`.
- `GlobalMapBuilder` consume tracks en 228/383 publicaciones; la final recalcula
  87 tracks con `fusion_revision=56`.
- Build 3/3 y tests 9/9 + 4/4 + 1/1 pasan. MemAvailable minimo 6568.8 MiB,
  PSS maximo servidor 229.7 MiB y guardia inactiva.
- El acuerdo posterior elimina el objetivo de commit de 5 ms sin sustituirlo
  por warning y retira por completo el presupuesto temporal de visibilidad.
- `prueba_161`: 27 intentos, ocho commits, 19 stale incluidos cuatro rollback
  y exactamente 19 retries encolados, iniciados y finalizados. Todos vuelven a
  ejecutar el pipeline fresco; algunos terminan por revalidacion y otros
  alcanzan un commit posterior.
- Visibilidad 161: `56/56` regiones completas en todos los intentos. Los ocho
  commits suman 348 pares, tracks `(258,84,2)`, 575 raw ocultos, score
  `+694/-396`, 11/11 regiones y 203187 proyecciones.
- Coste aceptado por ahora: prepare de commits media/maximo
  `633.852/1087.130 ms`; commit `5.362/10.353 ms`. Pese al aumento, 1162
  secundarias terminan con `pending=0`, cero hard y guard inactivo.
- Cardinalidad final 161: 62718 raw, unos 25249 publicables antes de fusion y
  24930 publicados despues de representar 575 miembros mediante 256 tracks.
  El score registra 1035 actualizaciones, pero no delta final por ID; su media
  teorica queda acotada a `+0.0153..+0.0230` por actualizacion.
- Cuatro optimizaciones fiduciales convergen a error cero y hacen commit full;
  la funcionalidad anterior no regresa.

## Pulido posterior al cierre

- reorganizar el layout desktop del grafo web segun la captura aportada para
  mejorar separacion de vertices, aristas y labels, sin cambiar topologia;
- mantener umbrales geometricos y formulas de score actuales hasta una prueba
  posterior con aproximadamente ocho drones y trayectorias repetidas;
- no repetir ahora la tipica: produjo retries, commits, score negativo completo
  y optimizaciones suficientes.

## Conclusion

`3P` queda `CONSEGUIDA`: el usuario acepta la funcionalidad y da por concluida
la subfase. No queda budget temporal, cada stale/rollback reintenta con estado
fresco, la cola drena y las optimizaciones fiduciales siguen funcionando. El
layout web pendiente es un pulido de legibilidad y no reabre las invariantes.
