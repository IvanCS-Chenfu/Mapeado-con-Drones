# Instrucciones para Codex — Reestructuración, documentación, limpieza y cierre definitivo de Fase 5

## 0. Objetivo

La Fase 5 ya ha quedado funcionalmente `CONSEGUIDA`, pero antes de cerrarla de forma definitiva hay que reorganizar documentación y código para que el resultado final sea comprensible, mantenible y útil como base de Fase 6.

Repositorio de referencia:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones/tree/main
```

Trabajar siempre sobre el workspace local real. Antes de modificar:

```bash
git status
git rev-parse HEAD
git log -1 --oneline
git diff
git diff --cached
```

No resetear cambios locales recientes para igualarlos a GitHub si contienen las últimas modificaciones de Fase 5.

Este trabajo debe conseguir cuatro cosas principales:

1. separar el alcance ORIGINAL de la actual 5H de toda la investigación posterior;
2. crear una nueva subfase que explique todo lo hecho para conseguir que ORB controle correctamente el dron en movimiento;
3. crear una subfase de limpieza/consolidación final;
4. crear `RESULTADO_FASE_5.md` como fotografía oficial del estado final.

Además hay que:

- centralizar los logs de diagnóstico de Fase 5 detrás de un flag de debug;
- conservar la instrumentación de evidencia visual útil;
- dejar a Fase 6 una especificación clara de cómo detectar zonas en degradación ORB, sin implementar aún el flag productivo ni el planificador.

---

# 1. Principio fundamental de documentación

La documentación debe explicar la historia REAL.

No reescribir la fase como si desde el principio se hubiera sabido cuál era la solución correcta.

Debe conservarse la secuencia:

```text
integración
→ aparecen fallos
→ instrumentación
→ aislamiento causal
→ corrección
→ nueva prueba
→ nuevo fallo
→ nueva instrumentación
→ validación progresiva
→ caracterización de la limitación visual
→ cierre
```

Debe quedar claro:

- qué problema apareció;
- qué hipótesis se plantearon;
- qué prueba se diseñó para aislarlo;
- qué resultado salió;
- qué conclusión se obtuvo;
- qué modificación se hizo;
- qué se validó después;
- qué intentos fueron contraproducentes;
- qué cosas no deben repetirse;
- qué responsabilidad pasa a Fase 6.

No inventar resultados, commits, fechas ni pruebas.

Usar documentación, historiales, código y logs reales.

---

# 2. Estructura final de Fase 5

La estructura conceptual debe quedar así:

```text
FASE 5
│
├── 5A ... 5G
│
├── 5H
│   Integración final del estado de navegación con trayectoria/control
│   → alcance ORIGINAL de la antigua 5H
│
├── 5I
│   Estabilización y validación del control ORB en movimiento
│   → todo el trabajo posterior necesario para hacer funcionar trayectoria
│
├── 5J
│   Limpieza, consolidación, debug y preparación para Fase 6
│
└── RESULTADO_FASE_5.md
```

Codex puede adaptar los nombres concretos a la convención del repositorio, pero no la separación conceptual.

---

# 3. Rehacer 5H: dejar únicamente su alcance original

La actual 5H ha absorbido mucha investigación posterior. Debe volver a representar únicamente su propósito original:

> Integrar el estado validado de Fase 5 con `gen_tray` y `control_calcular_fuerzas`, manteniendo continuidad de `O`, semántica correcta de goals, selección de fuente y fallback temporal de Fase 5.

Debe explicar:

## Arquitectura

```text
ORB-SLAM3 OK + anchor
        ↓
NavigationState en O
        ↓
gen_tray
        ↓
control_calcular_fuerzas
```

Temporalmente, si ORB deja de ser consumible:

```text
ORB perdido/no usable
        ↓
GT_FALLBACK
```

GT_FALLBACK nunca debe alimentar:

```text
mapa
KFs
RawMapDatabase
optimización
anchors
ORB-SLAM3
```

## Frames

Dejar muy claros:

```text
W = mundo global
L = mapa local ORB del epoch
O = frame local continuo de control
K = reference KF
C = cámara
B = body
G = Gazebo
```

Especialmente:

```text
O != L
```

Reconstrucción local:

```text
O_T_C = O_T_Kref * inverse(Tcr)
O_T_B = O_T_C * C_T_B
```

Reconstrucción global:

```text
W_T_C = W_T_Kref * inverse(Tcr)
W_T_B = W_T_C * C_T_B
```

Una revisión en `W` no puede mover:

```text
O
velocidad de control
goal O congelado
```

## Goals

Explicar:

```text
goal relativo → se genera en O
```

```text
goal absoluto con W_T_O válida
→ se transforma una vez a O
→ queda congelado en O
```

## Lock de fuente

Documentar el comportamiento final real:

- evitar mezclas GT/ORB dentro de una trayectoria activa;
- controlar la frontera de cambio de fuente;
- permitir ORB→GT ante pérdida real;
- mantener la coherencia del estado inicial del goal.

## Qué sale de 5H

La 5H ya no debe ser el lugar principal para explicar:

```text
20→50 Hz
predictor angular
predictor translacional
inercia
problema de gravedad
MIDPOINT_DYNAMIC
STALE_RAW_HISTORY
ruta de dos fachadas
evidencia visual
```

Todo eso pasa a 5I.

5H puede cerrar con una referencia del estilo:

> La integración inicial reveló problemas de estabilidad en movimiento. Su diagnóstico, corrección y validación se documentan en 5I.

---

# 4. Crear 5I — Estabilización y validación del control ORB en movimiento

Nombre orientativo:

```text
Subfase 5I — Estabilización y validación del estado ORB para control en movimiento
```

Esta debe ser la gran subfase narrativa/técnica que explique TODO lo que ocurrió después de intentar completar la 5H original.

No debe ser una lista de pruebas. Cada bloque debe seguir:

```text
problema
→ hipótesis
→ prueba
→ resultado
→ conclusión
→ modificación
→ validación
```

---

# 5. Historia técnica obligatoria en 5I

## 5.1. Problema de frecuencias ORB/control

Explicar claramente uno de los problemas estructurales iniciales:

```text
ORB-SLAM3 / cámara ≈ 20 Hz
control de vuelo ≈ 50 Hz
```

El problema NO era que “ORB tuviera una frecuencia incorrecta”. El problema era que el controlador necesitaba un estado coherente a 50 Hz mientras las medidas visuales llegaban más lentamente.

Consumir directamente la última pose o derivar velocidades de forma ingenua podía provocar:

```text
hold discontinuo
saltos de velocidad
diferencias finitas ruidosas
pose/velocidad fuera de fase
```

Esto motivó construir una ruta temporal que transformase las medidas visuales en un estado continuo/coherente para control a 50 Hz.

---

## 5.2. Bug de extrínseca cámara/body

Explicar el fallo observado donde aproximadamente:

```text
X world → aparecía como Z control
```

Se aisló que la transformación configurada como:

```text
body_T_camera
```

era realmente la inversa óptica:

```text
camera_T_body
```

La corrección real fue utilizar la `B_T_C` correcta:

```text
RPY = (-90, 0, -90)
```

según la convención vigente del proyecto.

Dejar claro:

- no era culpa del optimizador de Fase 3;
- no era el handoff;
- no debía compensarse en el controlador;
- era un error de frames/extrínseca.

---

## 5.3. Intento de filtrar también actitud GT

Explicar el intento que terminó siendo contraproducente:

```text
filtrar actitud
→ introducir retardo angular
→ error de fase en lazo de torque
→ realimentación inestable
```

Conocimiento final:

> No filtrar la actitud de control introduciendo un retardo arbitrario para esconder ruido visual.

---

# 6. Rama angular

## 6.1. Síntoma

Tras arreglar la extrínseca, la geometría mejoró mucho pero continuaban oscilaciones bajo ORB.

Se observó incoherencia entre:

```text
R
omega
```

---

## 6.2. Desfase temporal

Explicar cómo se comprobó que la medida visual seguía razonablemente a GT pero con retraso, mientras el estado angular usado por el controlador terminaba fuera de fase.

Consecuencia:

```text
tau_er
```

podía inyectar energía en vez de actuar de forma disipativa.

Conceptualmente:

```text
pose angular retrasada/incoherente
+
omega no coherente temporalmente
+
control a 50 Hz
→ torque fuera de fase
```

---

## 6.3. Anclajes visuales SMALL/MODERATE

Explicar:

- por qué reanclar SMALL mejoró mucho el trabajo angular;
- por qué anclar MODERATE completo de forma instantánea volvió a empeorar;
- por qué no se siguió simplemente tocando thresholds.

Conservar como conocimiento histórico:

```text
SMALL anchor → útil
MODERATE full immediate anchor → demasiado brusco
```

---

## 6.4. Pruebas cruzadas GT/ORB para aislar causalidad angular

Explicar la lógica de pruebas como:

```text
R predicha + omega GT
R GT + omega predicha
R GT + omega GT
```

Después se fijaron también `p/v` con GT en algunas baterías para impedir que la translación contaminase el diagnóstico.

Dejar claro:

> GT se utilizó únicamente como instrumento causal/diagnóstico, nunca como solución productiva.

La conclusión histórica fue que:

```text
omega_pred(now)
```

era una causa inmediata de inestabilidad bajo delay/jitter en aquel estado del sistema.

---

## 6.5. Estimador causal de omega

Explicar la transición a un estimador basado en poses aceptadas:

- tres poses;
- velocidades entre midpoints;
- aceleración entre midpoints;
- proyección hasta el timestamp visual;
- sin low-pass arbitrario.

Explicar que esto quedó validado primero en laboratorio con pose perfecta.

---

## 6.6. Predictor dinámico angular

Para compensar:

```text
t_k → now
```

se construyó propagación dinámica usando:

```text
torque body
J
timestamps reales
```

Inicialmente se usó una inercia nominal incorrecta:

```text
diag(1e-4)
```

que provocó predicciones físicamente absurdas.

Después se auditó la planta de Gazebo y se utilizó:

```text
J = diag(
    0.00803107,
    0.00803107,
    0.015805
) kg*m²
```

Con esa J la ruta dinámica angular quedó validada en laboratorio.

Esta historia debe quedar muy clara porque fue un bug físico real, no tuning.

---

# 7. Rama translacional

## 7.1. Aislamiento p/v

Cuando el angular estaba suficientemente controlado para hacer pruebas causales, se utilizaron cruces tipo:

```text
p_ORB + v_GT
p_GT + v_ORB
```

para localizar el problema translacional.

Se observó que la velocidad ORB era un canal especialmente problemático.

---

## 7.2. A_HAT_AMPLIFICATION

Explicar cómo el estimador de tres posiciones producía un `v_mid` razonable, pero la estimación de aceleración/segunda derivada:

```text
a_hat
```

amplificaba ruido visual.

El diagnóstico separó:

```text
A_HAT_AMPLIFICATION
```

y además un error dominante en la propagación dinámica.

---

## 7.3. Bug de gravedad en frame incorrecto

La dinámica translacional estaba en `O`, pero se sumaba directamente:

```text
g = (0, 0, -9.81)
```

como si:

```text
O == W
```

Corrección:

```text
g_W = (0, 0, -9.81)
g_O = O_R_W * g_W
```

`g_O`:

- se obtiene con la primera autoridad global válida;
- se congela durante el `map_epoch`;
- se invalida al cambiar de epoch;
- no se recalcula por una revisión global en W.

Explicar que esto redujo de forma drástica el error de propagación translacional.

---

## 7.4. MIDPOINT_DYNAMIC

Explicar por qué se abandona THREE_SAMPLE productivo en favor de:

```text
MIDPOINT_DYNAMIC
```

Concepto:

```text
dos poses aceptadas
→ v_mid en el midpoint
→ propagación física midpoint → t_k
→ propagación física t_k → now
```

usando:

```text
thrust
R_dynamic
g_O
masa
```

La ventaja es evitar una segunda derivada visual ruidosa.

Dejar claro que `MIDPOINT_DYNAMIC` es la solución productiva actual.

---

# 8. Buffers físicos y causalidad

Explicar la historia de:

```text
torque
thrust
buffers
timestamps
```

incluyendo:

- cold start;
- `EMPTY / MISSING_PREFIX / FULL`;
- seed explícito;
- ZOH;
- poda;
- necesidad de conservar una muestra predecesora para poder propagar causalmente;
- resets visuales que no deben destruir buffers físicos todavía válidos.

---

# 9. Handoff y authority/goal

Explicar los problemas de coordinación alrededor de:

```text
fuente
primer goal
frontera de autoridad
```

incluyendo la confirmación necesaria antes de iniciar la trayectoria.

Dejar claro que finalmente el handoff quedó prácticamente sin salto y dejó de ser sospechoso principal: los fallos aparecían segundos después.

---

# 10. Validación progresiva de movimiento

La narración debe seguir:

```text
hover
→ X
→ Y
→ Z
→ yaw
→ ruta de dos fachadas
```

## Hover

Tras corregir predictor angular, J, `g_O`, MIDPOINT_DYNAMIC y buffers se consiguió hover ORB real reproducible.

Este debe aparecer como un gran hito.

## X

Movimiento X corto validado sin fallback/tracking loss relevante.

## Y

Explicar:

- una prueba invalidada por colisión con un fiducial;
- al alejarse de superficies útiles, el frenado/velocidad lateral empeoraba;
- al acercarse a pared/fiducial, la misma clase de movimiento mejoraba mucho.

Este fue uno de los primeros indicios de dependencia de la geometría visual.

## Z

Movimiento vertical corto validado de forma reproducible.

## Yaw

El yaw sostenido volvió a revelar degradación de `R/omega` y pérdida posterior de tracking.

---

# 11. Ruta de dos fachadas y STALE_RAW_HISTORY

Explicar que en la ruta representativa aparecieron pasos raw enormes.

Inicialmente se sospechó mezcla geométrica de `reference_kf`, pero la auditoría mostró que la entrada raw ya estaba en `O`.

El defecto real era:

```text
STALE_RAW_HISTORY
```

Tras rechazos, el baseline raw podía quedar demasiado antiguo y generar:

```text
raw_dt enorme
raw delta artificial
```

aunque el movimiento real en `O` fuese pequeño.

Corrección:

```text
rebase exclusivo del historial raw
```

cuando la edad deja de ser válida.

Ese rebase:

- no acepta el delta;
- no resetea pose;
- no resetea dinámica;
- no resetea física.

Después `raw_dt` quedó acotado.

Dejar claro:

> Era un bug real, pero no explicaba por sí solo el fallo de trayectoria.

---

# 12. Pulso de validez y fallback

Explicar la instrumentación que mostró un fallback con:

```text
tracking = 2
```

porque fallaban temporalmente:

```text
local_valid=false
local_continuity_valid=false
```

La validez podía recuperarse rápidamente, pero la política de source lock mantenía GT.

Conclusión:

> Ese pulso disparaba el fallback, pero no originaba la inestabilidad, porque la divergencia comenzaba mucho antes.

Por eso no se decidió arreglar el vuelo simplemente añadiendo histéresis al fallback.

---

# 13. Batería 349 — grandes bloques p/v vs R/omega

Explicar muy bien la lógica:

```text
349A:
p/v = ORB
R/omega = GT
```

```text
349B:
p/v = GT
R/omega = ORB
```

El objetivo era averiguar si uno solo de los dos grandes bloques bastaba para reproducir el fallo.

Hubo intentos inválidos por soporte temporal GT insuficiente.

La infraestructura diagnóstica se corrigió únicamente ampliando:

```text
max_gt_alignment_skew:
20 ms → 30 ms
```

manteniendo:

```text
gt_effective_stamp = control_stamp
```

mediante propagación causal y sin introducir retardo artificial.

Finalmente las ramas válidas demostraron conceptualmente:

```text
p/v ORB + angular GT → puede degradarse
```

```text
R/omega ORB + p/v GT → también puede degradarse
```

Inicialmente se clasificó como:

```text
MULTIPLE_INDEPENDENT_ERRORS
```

pero NO detener aquí la narrativa: el diagnóstico de evidencia visual posterior dio una explicación de nivel superior.

---

# 14. Evidencia visual ORB — diagnóstico final

Esta es la parte que permite cerrar Fase 5.

La pregunta correcta no es:

```text
¿ORB jamás se pierde?
```

sino:

```text
¿Fase 5 funciona cuando ORB dispone de evidencia visual suficiente?
```

Documentar la instrumentación realmente añadida. Según lo que exista en el código, explicar métricas como:

```text
número de inliers
inlier ratio
cobertura espacial / grid coverage
puntos con profundidad válida
profundidad
disparidad
distribución de features
tracking
reference KF
```

No afirmar que una métrica existe si finalmente no fue implementada.

No reducir “mala visión” a “pocos puntos”. También importan:

```text
distribución
distancia/geometría
profundidad estéreo
disparidad
calidad de asociaciones
```

---

# 15. Qué se considera degradación ORB

No crear todavía un threshold productivo definitivo.

Una zona/frame entra en degradación cuando de forma persistente se observan tendencias como:

```text
↓ inliers útiles
↓ inlier ratio
↓ cobertura espacial
↓ evidencia estéreo válida
↑ profundidad / puntos más lejanos
↓ disparidad
```

seguidas o acompañadas por aumento de:

```text
error de pose
error de velocidad
error de orientación
error de omega
```

La clave es:

```text
magnitud
+
tendencia temporal
+
persistencia
```

No un frame aislado.

---

# 16. Prueba 351 — GT gobierna, ORB en shadow

Explicar que esta fue la prueba causal importante para distinguir:

```text
mala visión → ORB malo
```

frente a:

```text
mal control → cámara mala → ORB malo
```

En 351:

```text
GT gobierna físicamente
ORB trabaja completo en shadow
```

sobre la ruta problemática.

La degradación en la fachada problemática estuvo precedida por degradación clara de:

```text
inliers
cobertura
profundidad/geometría visual
```

Esto demuestra que el problema visual aparece sin necesidad de que ORB haya desestabilizado primero el dron.

---

# 17. Prueba 352 — ruta favorable shadow

Se creó manualmente una ruta más parecida a lo que Fase 6 deberá generar:

```text
más corta
más lenta
más próxima a pared
más favorable visualmente
```

Primero se probó:

```text
GT + ORB shadow
```

para confirmar la mejora de evidencia.

---

# 18. Pruebas 353–355 — validación final ORB

Después la misma ruta favorable fue gobernada por ORB.

Resultado final:

```text
353 ✅
354 ✅
355 ✅
```

Tres ejecuciones consecutivas correctas.

Dejar claro que la validación final no se basa en una ejecución afortunada.

Según lo registrado:

```text
ORB gobernando
sin fallback posterior relevante
sin tracking loss
sin divergencia
```

Este es el principal criterio experimental para cerrar Fase 5.

---

# 19. Interpretación final de 349

No dejar como conclusión final simplista:

```text
“hay dos bugs independientes p/v y R/omega”
```

La lectura final es:

349 demostró que un bloque degradado de `p/v` puede hacer fallar el control y que un bloque degradado de `R/omega` también puede hacerlo.

Pero 351–355 muestran una causa común de nivel superior:

```text
              evidencia ORB pobre
                    │
             ┌──────┴──────┐
             ↓             ↓
           p/v           R/omega
        se degradan     se degradan
             └──────┬──────┘
                    ↓
                 control
```

Esto explica por qué no se continuó desmontando indefinidamente `p/v/R/omega` cuando la pose visual deja de estar suficientemente condicionada.

---

# 20. Dominio de validez final

Fase 5 NO garantiza:

```text
“si tracking=2, el control siempre será perfecto”
```

Fase 5 SÍ queda validada para:

```text
evidencia visual ORB suficiente
        ↓
pose ORB suficientemente fiable
        ↓
NavigationState coherente
        ↓
control estable
```

La prevención activa de zonas donde la evidencia se degrada pasa a Fase 6.

---

# 21. Crear 5J — Limpieza, consolidación y preparación para Fase 6

Nombre orientativo:

```text
Subfase 5J — Limpieza, consolidación y preparación para Fase 6
```

Objetivo:

> dejar únicamente el código necesario para producto y la instrumentación realmente útil, eliminando basura experimental acumulada durante el diagnóstico.

---

# 22. Auditoría de código

Buscar:

```text
flags de pruebas antiguas
overrides GT temporales
modos experimentales
scripts duplicados
launch args muertos
clases no usadas
código unreachable
TODOs ya resueltos
comentarios obsoletos
tests duplicados
configuración sin consumidores
```

Clasificar cada elemento como:

```text
PRODUCTIVO
TEST ÚTIL
DEBUG ÚTIL
HISTÓRICO/OBSOLETO
```

No borrar automáticamente todo lo diagnóstico.

---

# 23. Qué conservar

Conservar:

- arquitectura productiva final;
- tests que protegen bugs corregidos;
- instrumentación visual útil para Fase 6;
- traces de debugging realmente útiles;
- analizadores útiles para regresiones;
- contratos de frames/timing;
- configuración realmente consumida.

---

# 24. Qué eliminar

Eliminar si ya no es necesario:

- overrides GT específicos de baterías cerradas;
- nodos temporales sin utilidad futura;
- parámetros no consumidos;
- variantes experimentales abandonadas;
- scripts duplicados;
- código muerto.

No perder trazabilidad histórica. Si YAML/logs se conservan como historial, organizarlos en carpetas adecuadas en vez de dejarlos mezclados con producto.

---

# 25. Comentarios del código

Comentar especialmente el PORQUÉ de:

- contratos de frames;
- `g_O` congelada por epoch;
- ZOH y muestra predecesora;
- buffers físicos preservados ante reset visual;
- MIDPOINT_DYNAMIC;
- timestamps/horizontes del predictor;
- diferencia medida visual/base/estado propagado;
- invariantes de `O`;
- condiciones de fallback.

Evitar comentarios que sólo repitan lo que hace una línea.

---

# 26. Flag global de debug de Fase 5

Añadir un flag global siguiendo la convención del repositorio, por ejemplo:

```text
debug_fase_5
```

o:

```text
debug_phase_5
```

Default:

```text
false
```

Debe servir para desactivar centralizadamente la telemetría extensa acumulada en Fase 5.

Ejemplos de logs diagnósticos candidatos:

```text
[F5H-*]
[F5-*]
PHASE-MEASUREMENT
PHASE-PUBLISH
PRODUCTIVE-PREDICT
MIDPOINT-DYNAMIC
LINEAR-MEASUREMENT
CONTROL-DIAG
FALLBACK-CAUSE-TRACE
visual evidence dumps
```

Clasificar los logs reales, no usar esta lista de forma ciega.

---

# 27. No ocultar errores reales

No meter todo bajo el flag.

Clasificación:

```text
TELEMETRÍA / DIAGNÓSTICO
→ bajo debug_fase_5

WARNING FUNCIONAL REAL
→ visible según política adecuada

ERROR / FATAL
→ siempre visible
```

No ocultar, por ejemplo:

```text
NaN
invariante rota
estado imposible
buffer físico realmente inválido
```

---

# 28. Integración con flags existentes

Si existen:

```text
debug_orb_control_state
debug_orb_visual_evidence
debug_architecture_telemetry
```

no crear una jerarquía incoherente.

Diseñar una política clara. Ejemplo conceptual:

```text
debug_fase_5=false
→ subdebug de Fase 5 efectivos = false

debug_fase_5=true
→ cada subdebug puede activarse individualmente
```

Si existe una solución mejor en la arquitectura actual, usarla y documentarla.

---

# 29. Contrato orientativo de evidencia ORB para Fase 6

NO crear todavía el flag productivo:

```text
orb_visual_degraded
```

Su diseño final pertenece a Fase 6.

Pero 5J debe dejar documentado cómo aproximarlo.

Conceptualmente Fase 6 debería poder distinguir:

```text
EVIDENCE_GOOD
EVIDENCE_DEGRADING
EVIDENCE_POOR
```

usando una ventana temporal.

---

# 30. Señales relevantes para Fase 6

Documentar únicamente las métricas realmente disponibles, por ejemplo:

```text
n_tracking_inliers
inlier_ratio
grid_coverage
n_valid_stereo_depth
stereo_depth_ratio
median_depth
median_disparity
tracking_state
```

No inventar campos ausentes.

---

# 31. No reaccionar a un frame aislado

La orientación para Fase 6 debe ser:

```text
ventana de últimos N frames
        ↓
evidencia actual
+
tendencia
+
persistencia
        ↓
GOOD / DEGRADING / POOR
```

No fijar todavía `N` ni thresholds mágicos sin calibración de Fase 6.

---

# 32. No esperar a LOST

Dejar explícito:

> No esperar a `RECENTLY_LOST` o `LOST`.

Puede existir:

```text
tracking=2
```

mientras la estimación ya se degrada demasiado para control.

Fase 6 debe evitar la zona ANTES de la pérdida formal.

---

# 33. Score orientativo para Fase 6

Se puede documentar, sin implementarlo aún:

```text
visual_evidence_score = f(
    inliers,
    inlier_ratio,
    grid_coverage,
    stereo_depth_ratio,
    median_disparity,
    median_depth,
    temporal_trend
)
```

No fijar pesos arbitrarios.

Fase 6 deberá calibrar:

```text
threshold_good
threshold_degrading
threshold_poor
```

con persistencia e histéresis.

---

# 34. Cómo afectará a planificación de Fase 6

Documentar como orientación, NO implementar todavía:

Una zona degradándose podrá aumentar:

```text
coste de trayectoria
```

o motivar:

```text
evitar dirección
cambiar siguiente punto
reducir velocidad
acortar segmento
```

La idea general es:

```text
menos velocidad
segmentos más cortos
mejor evidencia
```

---

# 35. Historiales

Organizar una estructura clara para conservar:

- resumen histórico;
- sesiones;
- baterías;
- pruebas inválidas;
- intentos descartados;
- decisiones técnicas.

No inventar historial.

Si se crean carpetas como:

```text
historial/
pruebas/
diagnosticos/
```

seguir la convención del proyecto.

---

# 36. Documento global de explicación de Fase 5

Además de 5H/5I/5J, crear o consolidar un documento claramente identificable que explique Fase 5 de principio a fin.

Nombre orientativo:

```text
EXPLICACION_FASE_5.md
```

Si la convención actual recomienda integrarlo dentro de `pipeline_fase_5_RESUMEN.md`, puede hacerse allí, pero debe existir una narrativa global clara.

Debe explicar:

```text
qué se quería conseguir
qué arquitectura se eligió
qué problemas aparecieron
cómo se diagnosticaron
qué se corrigió
qué se validó
qué limitaciones quedaron
por qué se cierra
```

---

# 37. Estructura obligatoria del documento de explicación

Debe incluir como mínimo:

## A. Objetivo inicial

Eliminar GT del funcionamiento normal del control y consumir un estado común basado en ORB.

## B. Arquitectura de frames

W/L/O/K/C/B.

## C. Integración inicial

NavigationState, mux, goals, fallback.

## D. Frecuencias

```text
ORB ~20 Hz
control ~50 Hz
```

y necesidad de adaptación temporal.

## E. Extrínseca

Bug `B_T_C`.

## F. Desfase temporal angular

Pose/omega y trabajo de `tau_er`.

## G. Diagnósticos con GT

Por qué se sustituyeron temporalmente p/v o R/omega.

## H. Predictor angular

Estimador causal + propagación dinámica con torque/J.

## I. Problema translacional

p/v y A_HAT_AMPLIFICATION.

## J. Gravedad

`g_O`.

## K. MIDPOINT_DYNAMIC

Solución productiva translacional.

## L. Buffers

Torque/thrust/ZOH/poda.

## M. Hover estable

Primer gran hito.

## N. Trayectorias simples

X/Y/Z/yaw.

## O. Ruta representativa

Dos fachadas/edificio.

## P. STALE_RAW_HISTORY

Bug y solución.

## Q. Pulso de validez

Fallback no causal.

## R. 349 GT/ORB

Aislamiento de grandes bloques.

## S. Evidencia visual

Instrumentación y métricas.

## T. 351

GT + ORB shadow demuestra precedencia visual.

## U. 352–355

Ruta favorable y validación ORB 3/3.

## V. Conclusión

Fase 5 funciona con evidencia suficiente.

## W. Transferencia a Fase 6

Evitar zonas degradadas.

---

# 38. Nivel de detalle del documento de explicación

No hacer un resumen superficial.

Debe permitir que alguien que no haya seguido todas las pruebas entienda:

- qué se rompió;
- por qué;
- cómo se aisló;
- qué solución quedó;
- por qué las decisiones no fueron arbitrarias.

No copiar cientos de logs completos.

Usar:

```text
diagramas ASCII
ecuaciones
tablas de hitos
resultados cuantitativos representativos
referencias a historiales
```

---

# 39. Tabla de hitos recomendada

Crear una tabla similar a:

| Bloque | Problema | Diagnóstico | Corrección | Validación |
|---|---|---|---|---|
| Frames | `B_T_C` invertida | X→Z | extrínseca correcta | 252 |
| Angular | pose/omega fuera de fase | cruces GT | estimator + dynamic predictor | baterías correspondientes |
| Física angular | J incorrecta | predictor explota | J compuesta | 293–298 aprox. según historial real |
| Translación | velocidad ruidosa | cruces p/v | MIDPOINT_DYNAMIC | 326–331 aprox. |
| Gravedad | g aplicada en frame incorrecto | shadow | `g_O` | 324–325 |
| Raw | baseline obsoleto | `raw_dt` gigante | rebase | 344–345 |
| Validez | fallback con tracking 2 | predicate trace | diagnóstico, no parche | 348 |
| Grandes bloques | p/v y R/omega degradan | 349 | buscar causa común | 349 |
| Visión | evidencia pobre | GT+shadow | caracterización | 351 |
| Cierre | ruta favorable | ORB real | sin cambios de estimador | 353–355 |

Verificar los números exactos con los historiales antes de escribirlos.

---

# 40. Crear RESULTADO_FASE_5.md

Este documento NO es otro historial.

Debe ser la fotografía final oficial.

Estructura recomendada:

```text
# Resultado Fase 5

## Estado
CONSEGUIDA

## Objetivo

## Arquitectura final

## Contratos de frames

## Estado final de cada subfase

## Implementación final

## Qué se validó

## Qué NO se exige

## Limitaciones conocidas

## Dominio válido de funcionamiento

## Evidencia ORB

## Pruebas finales

## GT_FALLBACK

## Debug

## Deuda transferida a Fase 6

## Condiciones para reabrir Fase 5
```

---

# 41. Conclusión que debe quedar en RESULTADO_FASE_5

Debe decir, con redacción técnica equivalente:

```text
FASE 5: CONSEGUIDA
```

Alcance:

> La arquitectura de Fase 5 proporciona un estado local continuo y suficientemente coherente para el control del dron cuando ORB-SLAM3 dispone de evidencia visual adecuada. La estabilidad se ha validado en hover, movimientos simples y una ruta corta/lenta/visualmente favorable, incluyendo tres ejecuciones consecutivas finales gobernadas por ORB.

Limitación:

> Las trayectorias largas que atraviesan zonas de evidencia visual pobre pueden degradar tanto p/v como R/omega incluso antes de que ORB pierda formalmente tracking. La detección y evitación de esas regiones pasa a Fase 6.

---

# 42. Qué NO decir

No decir:

```text
“ORB siempre funciona si tracking=2”
```

No decir:

```text
“la vuelta completa al edificio está validada ORB-only”
```

No decir:

```text
“todos los fallos previos eran simplemente pocos puntos”
```

La formulación correcta es:

```text
la arquitectura queda validada dentro del dominio de evidencia visual suficiente
```

---

# 43. GT_FALLBACK

Dejar claro:

- fue una excepción temporal de Fase 5;
- no es la solución final deseada del sistema completo;
- Fase 6 debe eliminar su dependencia según el diseño previsto;
- nunca alimenta mapas/KFs/anchors/optimización.

---

# 44. Condiciones para reabrir Fase 5

Fase 5 debe reabrirse si en Fase 6 aparece:

```text
evidencia visual buena + p/v divergen
```

```text
evidencia visual buena + R/omega divergen
```

```text
W mueve O
```

```text
reference KF introduce discontinuidad
```

```text
NavigationState rompe coherencia
```

No culpar automáticamente a Fase 6.

---

# 45. Builds y tests tras limpieza

Compilar al menos:

```text
orbslam3_ros2
dron_individual
simulacion_dron
```

Ejecutar:

- GTests vigentes;
- mux tests;
- analyzer tests;
- tests de visual evidence;
- `git diff --check`.

No aceptar una limpieza que rompa tests o elimine código necesario.

---

# 46. Regresión funcional post-limpieza

Después de 5J, repetir al menos una ruta favorable ORB equivalente a 353–355.

Preferiblemente una segunda si el coste es razonable.

Objetivo:

```text
limpieza
→ no cambia comportamiento
```

Si cambia:

```text
STOP
```

y localizar qué consolidación/eliminación lo produjo.

---

# 47. Prueba del flag debug

Ejecutar una prueba con:

```text
debug_fase_5=false
```

Comprobar:

```text
no spam diagnóstico F5
```

pero permanecen:

```text
warnings funcionales importantes
errors
fatal
```

Después ejecutar con:

```text
debug_fase_5=true
```

para verificar que la telemetría vuelve a estar disponible.

---

# 48. Qué debe devolver Codex

Al terminar, entregar:

```text
1. commit/estado inicial;
2. diferencias local vs origin/main;

3. estructura final de subfases;

4. archivos creados;
5. archivos movidos;
6. archivos modificados;
7. archivos eliminados;

8. qué queda en 5H;
9. qué pasa a 5I;
10. qué contiene 5J;

11. cómo se preserva el historial;
12. qué narrativa técnica se añadió;

13. código experimental eliminado;
14. código diagnóstico conservado;
15. tests conservados/eliminados y motivo;

16. diseño del flag debug_fase_5;
17. logs bajo el flag;
18. logs siempre visibles;

19. contrato de evidencia para Fase 6;
20. métricas realmente disponibles;
21. propuesta GOOD/DEGRADING/POOR sin thresholds mágicos;

22. contenido/resumen de RESULTADO_FASE_5.md;

23. builds;
24. tests;
25. git diff --check;

26. regresión post-limpieza;
27. prueba debug off;
28. prueba debug on;

29. estado final:
FASE 5 = CONSEGUIDA
```

---

# 49. Criterio de éxito

La intervención queda `CONSEGUIDA` sólo si:

```text
- 5H vuelve a su alcance original;
- 5I explica toda la estabilización posterior;
- 5J limpia y consolida el código;
- los historiales siguen accesibles;
- se elimina basura experimental innecesaria;
- el código que queda está comentado donde corresponde;
- existe un debug global de Fase 5;
- la instrumentación visual útil se conserva;
- Fase 6 recibe un contrato claro de degradación visual;
- existe RESULTADO_FASE_5.md;
- builds/tests pasan;
- la limpieza no cambia el comportamiento;
- Fase 5 queda oficialmente CONSEGUIDA.
```

---

# 50. Principio final

La Fase 5 no debe terminar como cientos de pruebas y parches difíciles de comprender.

Debe terminar como:

```text
arquitectura clara
+
estado ORB de control validado
+
historial técnico trazable
+
implementación limpia
+
dominio de operación explícito
+
interfaz clara hacia Fase 6
```

El documento de explicación debe dejar perfectamente claro por qué se pasó de:

```text
“usar ORB en vez de GT”
```

a necesitar:

```text
adaptación temporal 20→50 Hz
coherencia pose/velocidad
predictor angular
modelo físico de torque/inercia
estimación translacional
g_O
MIDPOINT_DYNAMIC
buffers causales
rebase de raw history
pruebas cruzadas GT/ORB
análisis de evidencia visual
```

hasta llegar finalmente a:

```text
FASE 5 CONSEGUIDA
```

con la limitación visual correctamente transferida a Fase 6.
