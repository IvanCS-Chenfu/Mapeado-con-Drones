# Diagnóstico de evidencia visual ORB — decisión de cierre de Fase 5

## 1. Objetivo

Implementar instrumentación y una batería de pruebas para responder experimentalmente a una única pregunta:

> **Cuando ORB-SLAM3 dispone de evidencia visual suficientemente buena, ¿la Fase 5 transforma esa información en un estado de vuelo continuo, coherente y controlable?**

La prueba larga alrededor del edificio contiene trayectorias largas, zonas lejanas, regiones con poca textura/pocos puntos útiles y una velocidad superior a la prevista posteriormente. En Fase 6 se pretende reducir precisamente esos problemas mediante trayectorias más cortas, menor velocidad y evitación de zonas con poca evidencia ORB.

Por tanto:

- si se demuestra que la degradación visual ocurre primero, después empeoran `p/v/R/omega` y finalmente falla el control, mientras que con evidencia suficiente Fase 5 funciona de forma repetible, **Fase 5 se considerará superada** y la limitación restante pasa a Fase 6;
- si aparece un fallo de estimación/control con evidencia visual buena, **Fase 5 seguirá abierta**.

Este trabajo es principalmente diagnóstico. No seguir realizando cambios funcionales de estimador/control hasta resolver esta cuestión.

---

## 2. Restricciones

Codex está autorizado a:

- inspeccionar toda la implementación actual de Fase 5;
- añadir telemetría de evidencia visual ORB por frame;
- ampliar estructuras/mensajes de debug cuando sea necesario;
- añadir CSV/JSONL y scripts offline;
- añadir tests unitarios/deterministas;
- preparar launch/configuración para las pruebas;
- documentar build, ejecución y análisis;
- analizar los resultados obtenidos por el usuario;
- actualizar los MD de Fase 5 con la conclusión.

No está autorizado en esta intervención a:

- cambiar ganancias del controlador;
- cambiar filtros/umbrales del predictor para “mejorar” el resultado;
- introducir otro predictor;
- cambiar semántica de `O`, `W`, `L`, KFs o goals;
- rehacer el handoff GT/ORB;
- introducir el planificador/recovery de Fase 6;
- usar GT para corregir el estado ORB productivo;
- declarar que el problema es “pocos puntos” mirando solo un contador.

La instrumentación debe estar **apagada por defecto**.

Si aparece un bug inequívoco de Fase 5, parar, documentarlo y proponer la corrección antes de seguir cambiando comportamiento.

---

## 3. Arquitectura que no debe modificarse

Mantener:

```text
ORB-SLAM3 / GT_FALLBACK
          ↓
NavigationState en O
          ↓
gen_tray
          ↓
control_calcular_fuerzas
```

Frames:

```text
W = mundo global corregible
L = mapa local ORB del epoch
O = frame local continuo de control
```

y:

```text
O != L
```

Una revisión/optimización global en `W` no debe mover `O`, introducir velocidad artificial ni deformar un goal ya congelado en `O`.

---

## 4. Requisito fundamental: evidencia del MISMO frame

La evidencia visual debe quedar asociada a la misma llamada de tracking que produjo la pose usada por la navegación.

Conceptualmente:

```text
TrackStereo(frame_t)
        |
        +--> pose / tracking_state
        +--> reference KF + Tcr
        +--> evidencia visual del frame_t
```

Compartir, siempre que exista:

```text
timestamp
frame/sample id
map_epoch
tracking_state
reference_keyframe_id
```

No medir después:

- número total de MapPoints del mapa;
- último frame disponible desde otro callback;
- estado global del mapa como sustituto de la evidencia del frame.

Si actualmente existe un receipt/snapshot asociado a `TrackStereo`, ampliarlo es preferible a realizar lecturas asíncronas.

---

## 5. Qué medir por frame

No basta con contar features. Se necesita una caracterización multidimensional.

### 5.1 Identidad y estado

Registrar:

```text
timestamp_visual
frame_id/sequence si existe
map_epoch
tracking_state
reference_keyframe_valid
reference_keyframe_id

navigation_state_timestamp
state_age
pose_source
local_valid
local_continuity_valid
velocity_valid
```

### 5.2 Features

Si la implementación de ORB lo permite de forma limpia:

```text
n_keypoints_left
n_keypoints_right
n_features_detected
n_features_used_by_tracking
```

### 5.3 MapPoints e inliers

Obtener, según la semántica real de ORB-SLAM3:

```text
n_map_point_matches
n_tracking_inliers
n_tracking_outliers
inlier_ratio
```

No inventar una semántica distinta de la usada internamente por ORB. Documentar exactamente qué significa cada contador.

### 5.4 Evidencia estéreo/profundidad

Especialmente importante para `p/v`:

```text
n_valid_stereo_depth
stereo_depth_ratio
depth_p25
depth_p50
depth_p75
depth_p90
```

Como mínimo, si reducir métricas simplifica mucho:

```text
n_valid_stereo_depth
median_depth
p90_depth o far_point_ratio
```

Hipótesis a comprobar:

```text
puntos lejanos
→ menor disparidad
→ peor precisión de profundidad
→ peor translación
→ peor velocidad derivada
```

### 5.5 Disparidad

Cuando pueda obtenerse correctamente:

```text
disparity = x_left - x_right
```

Registrar, preferentemente:

```text
disparity_p10
disparity_p25
disparity_p50
disparity_p75
disparity_p90
```

o una versión reducida como:

```text
median_disparity
low_disparity_ratio
```

No inventar un threshold de “low disparity” sin estudiar calibración y datos reales.

Si la disparidad no es accesible de forma fiable pero sí la profundidad, documentarlo y usar profundidad.

### 5.6 Distribución espacial

Distinguir:

```text
muchos puntos bien repartidos
```

de:

```text
muchos puntos agrupados en una zona
```

Usar una rejilla sencilla, por ejemplo 4x3 o 4x4, sobre inliers/matches realmente usados por tracking:

```text
occupied_cells
grid_coverage_ratio
```

Opcionalmente:

```text
feature_centroid_x/y
feature_spread_x/y
```

### 5.7 Error de reproyección

Si ORB-SLAM3 lo expone sin una intervención profunda:

```text
reprojection_error_median
reprojection_error_p90
```

Si obtenerlo obliga a alterar demasiado el tracker, dejarlo documentado como no disponible. No repetir optimizaciones solo para esta métrica.

---

## 6. Estado ORB que debe correlacionarse

Para cada muestra disponer de:

```text
raw ORB pose
published O_T_B
linear velocity publicada
angular velocity publicada
tracking_state
reference KF
reference-KF switch
innovación traslacional
innovación angular
corrección aplicada
estado de gates/predictor
```

Reutilizar `debug_orb_control_state` y la infraestructura existente siempre que sea posible.

No crear telemetría duplicada si ya existe.

---

## 7. GT solo como referencia externa

En los experimentos, GT se utilizará para evaluación, nunca para corregir el estado ORB.

Registrar sincronizadamente:

```text
GT pose
GT linear velocity
GT angular velocity
```

y calcular:

```text
position_error
orientation_error
linear_velocity_error
angular_velocity_error
```

Descomponer por ejes cuando sea útil.

Asegurarse de que cada comparación utiliza variables expresadas en frames compatibles.

---

## 8. Telemetría de control

Correlacionar también:

```text
ep
ev
er
ew
F_des
torque solicitado
```

y las métricas de energía que ya existan en los diagnósticos de 5H.

No cambiar el controlador.

---

## 9. Formato de salida

Preferencia:

```text
CSV o JSONL
```

No mezclar silenciosamente datos de distintas frecuencias.

Una organización válida:

```text
orb_visual_evidence.csv
navigation_state_debug.csv
control_debug.csv
gt_reference.csv
```

más un script offline de sincronización por timestamp.

Alternativamente puede generarse un fichero ya sincronizado si la correspondencia temporal es inequívoca.

Debe primar:

1. trazabilidad temporal;
2. bajo impacto en runtime;
3. ausencia de ambigüedad;
4. repetibilidad.

Añadir un flag, nombre orientativo:

```text
debug_orb_visual_evidence
```

apagado por defecto.

Evitar imprimir decenas de líneas por segundo en consola; usar fichero estructurado.

---

## 10. Tests antes de Gazebo

Añadir tests deterministas, al menos para la parte de instrumentación independiente de ORB.

### Grid coverage

```text
puntos en una única celda → cobertura baja
puntos repartidos → cobertura alta
```

### Estadísticas

Probar:

```text
median
percentiles
vectores vacíos
NaN/Inf
profundidades inválidas
```

### Ratios

Validar división por cero y semántica de inliers/outliers.

### Sincronía

Si se amplía un receipt/snapshot:

```text
timestamp pose == timestamp evidencia
reference KF evidencia == reference KF de la muestra
```

No crear tests frágiles que dependan de un número exacto de features de una imagen real.

---

# 11. Experimento principal: GT gobierna y ORB trabaja en shadow

Esta prueba debe hacerse **antes de volver a probar ORB como fuente de control**.

Motivo:

```text
ORB malo
→ control mueve mal el dron
→ cámara empeora
→ ORB empeora
```

puede crear un círculo de realimentación y ocultar qué ocurrió primero.

Ejecutar la trayectoria problemática alrededor del edificio con:

```text
CONTROL REAL = GT
```

mientras ORB-SLAM3 funciona normalmente en shadow y se registra:

```text
evidencia ORB
raw ORB
NavigationState ORB
p/v/R/omega
tracking
reference KFs
GT
```

Pregunta central:

> ¿ORB empieza a degradarse en las mismas zonas aun cuando físicamente el dron está siendo gobernado de forma limpia por GT?

Si sí, aumenta fuertemente la evidencia de una limitación visual.

No limitar el estudio a `RECENTLY_LOST`/`LOST`: interesa detectar degradación mientras ORB todavía afirma `OK`.

---

# 12. Análisis temporal

Generar gráficas sincronizadas, separadas en grupos legibles.

### Evidencia

```text
n_inliers
inlier_ratio
grid_coverage
n_valid_stereo_depth
median_depth
median_disparity
tracking_state
```

### Translación

```text
position_error
linear_velocity_error
```

### Angular

```text
orientation_error
angular_velocity_error
```

### Control

```text
ep/ev
er/ew
force
torque
```

### Eventos

Marcar:

```text
reference KF changes
tracking changes
source changes
W revisions
fallback
inicio/final de goals
```

---

## 13. Precedencia causal

No basta con decir que dos variables son malas simultáneamente.

Patrón compatible con limitación visual:

```text
t0: cae evidencia
t1: aumenta error raw ORB
t2: aumenta error NavigationState
t3: aumenta error/torque de control
t4: eventualmente tracking cae
```

Patrón que implica bug de Fase 5:

```text
t0: evidencia estable/buena
t1: NavigationState salta o deriva
t2: control oscila
t3: debido a la oscilación baja la evidencia
```

No imponer un lag fijo: determinarlo con datos.

---

# 14. Análisis agregado por calidad visual

Además de series temporales, agrupar por cuantiles/bins de:

```text
n_inliers
grid_coverage
median_depth
median_disparity
```

y obtener para cada grupo:

```text
median / p90 position_error
median / p90 linear_velocity_error
median / p90 orientation_error
median / p90 angular_velocity_error
```

Preguntas a responder:

```text
¿el error de velocidad aumenta claramente con baja disparidad?
¿R/omega divergen aun con cobertura/inliers altos?
¿los tramos de evidencia alta se parecen a las pruebas simples estables?
```

No confundir correlación con causalidad: combinar con precedencia temporal y shadow GT.

---

# 15. Separar evidencia translacional y angular

Para `p/v`, estudiar especialmente:

```text
depth
disparity
valid stereo depth ratio
```

Para `R/omega`, estudiar especialmente:

```text
inliers
inlier_ratio
coverage
reprojection error si existe
```

No asumir que una misma métrica explica ambos canales.

Es posible que puntos lejanos degraden mucho más translación que orientación.

---

# 16. Reference KF y revisiones W

Para cada cambio de reference KF:

```text
timestamp
KF old/new
evidencia antes/después
raw pose step
published O pose step
raw/published omega
tracking
```

Separar:

```text
KF switch con evidencia buena
KF switch con evidencia mala
```

Si un cambio de KF produce salto de `O` con evidencia buena:

```text
BUG FASE 5
```

Registrar también revisiones globales `W`.

Comprobar explícitamente:

```text
W revision
→ puede modificar W
→ NO debe modificar O_T_B
→ NO debe crear velocidad de control
→ NO debe mover el goal O congelado
```

Si una revisión W contamina O con evidencia buena, Fase 5 sigue abierta.

---

# 17. Ruta manual representativa de Fase 6

Sin implementar el planificador de Fase 6, preparar manualmente una trayectoria diagnóstica con:

```text
segmentos más cortos
menor velocidad
mayor proximidad a superficies con textura
evitación de las regiones visualmente peores observadas
```

Primero:

```text
GT + ORB shadow
```

para verificar que la evidencia mejora.

Después, solo si se confirma que la evidencia es buena:

```text
ORB control
```

con la arquitectura actual de Fase 5.

No cerrar por una única ejecución afortunada. Como criterio mínimo razonable:

```text
>= 3 ejecuciones ORB consecutivas
```

sin crash, divergencia creciente, fallback inesperado ni tracking loss no explicado.

---

# 18. Baseline empírico

Usar pruebas cortas/simples que ya hayan funcionado como baseline.

Comparar:

```text
evidencia de prueba estable
vs
evidencia de zona problemática
```

y:

```text
errores ORB de prueba estable
vs
errores ORB de zona problemática
```

Evitar crear de entrada un threshold mágico como:

```text
n_inliers < 80 → ORB malo
```

La Fase 6 podrá diseñar posteriormente un score real de evidencia. Aquí solo interesa determinar causalidad.

Se permite crear un `visual_evidence_score` únicamente **offline**, manteniendo siempre visibles las métricas originales y sin usarlo en control/fallback/planificación.

---

# 19. Criterios para declarar FASE 5 SUPERADA

Se podrá cerrar Fase 5 por este asunto si se demuestra de forma convincente todo lo siguiente:

1. En **GT + ORB shadow**, ORB también se degrada en las zonas problemáticas.
2. La degradación de inliers/cobertura/profundidad/disparidad u otra evidencia significativa aparece desde el origen del episodio, antes o junto al inicio del error ORB.
3. En segmentos con evidencia alta no aparece la misma divergencia de `p/v/R/omega`.
4. Los fallos no quedan mejor explicados por:
   - reference-KF switch con evidencia buena;
   - revisión W contaminando O;
   - salto del predictor;
   - incoherencia pose/velocidad;
   - bug temporal;
   - frames/extrínsecos.
5. La ruta corta/lenta/visual favorable mejora claramente la evidencia.
6. Esa ruta funciona gobernada por ORB de manera repetible.
7. No ha sido necesario modificar ganancias, controlador ni arquitectura para conseguirlo.

Conclusión que deberá quedar documentada:

> **La Fase 5 proporciona un estado de control válido y suficientemente estable cuando ORB-SLAM3 dispone de evidencia visual adecuada. Los fallos restantes de la trayectoria larga se originan en regiones de baja observabilidad visual. Su prevención activa corresponde a la Fase 6 mediante trayectorias más cortas, menor velocidad y selección/evitación basada en evidencia.**

---

# 20. Casos que mantienen FASE 5 ABIERTA

Cualquiera de estos resultados obliga a mantenerla abierta:

```text
evidencia buena → p/v divergen

evidencia buena → R/omega divergen

estado ORB empeora primero
→ control oscila
→ solo después baja evidencia

reference KF switch + evidencia buena
→ salto en O

W revision
→ salto en O o goal

raw ORB razonable
→ predictor/NavigationState introduce el error

pose y velocidades publicadas dinámicamente incompatibles

ruta corta/lenta/con buena evidencia
→ sigue fallando bajo ORB
```

En ese caso identificar el **primer componente que rompe el contrato**, no seguir culpando a los puntos ORB.

---

# 21. Informe automático por ejecución

Preparar un script que produzca algo equivalente a:

```text
TEST
----
duración:
fuente de control:

TRACKING
--------
OK:
RECENTLY_LOST:
LOST:

VISUAL EVIDENCE
---------------
inliers median/p10/min:
inlier ratio median/p10:
grid coverage median/p10:
valid stereo depth:
median depth:
median disparity:

ORB ERROR vs GT
---------------
position RMSE/p90/max:
velocity RMSE/p90/max:
orientation RMSE/p90/max:
omega RMSE/p90/max:

EVENTS
------
reference KF switches:
W revisions:
fallbacks:
goals completed:

LOW-EVIDENCE WINDOWS
--------------------
[start, end, evidence cause, error before, error after]

FAILURE WINDOW
--------------
first visual degradation:
first raw ORB degradation:
first NavigationState degradation:
first control degradation:
first tracking degradation:
```

Distinguir `no disponible` de `0`.

Para cada fallo generar además una ventana detallada de aproximadamente 5–10 s previos y una ventana estable comparable.

---

# 22. Archivos prioritarios a inspeccionar

Como mínimo:

```text
dron/orbslam3_ros2/src/stereo/stereo-slam-node.cpp
dron/orbslam3_ros2/src/stereo/stereo-slam-node.hpp

dron/orbslam3_ros2/src/stereo/navigation-state-estimator.cpp
dron/orbslam3_ros2/src/stereo/navigation-state-estimator.hpp

dron/orbslam3_ros2/test/**

dron/dron_individual/src/control_tray/control_calcular_fuerzas.cpp

dron/orbslam3_ros2/config/**
dron/dron_individual/config/**
```

Seguir además la integración real de ORB-SLAM3 usada por `TrackStereo`.

Reutilizar la infraestructura diagnóstica existente de 5H y `debug_orb_control_state`.

No asumir que esta lista agota todos los archivos implicados.

---

# 23. Documentación a actualizar

Al terminar, actualizar sin borrar el historial:

```text
codex/pipeline/fase_5_poses_drones_sin_gt/subfases/subfase_5H.md
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
```

y, si la conclusión final lo requiere:

```text
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5.md
```

Añadir:

```text
motivación
instrumentación
pruebas
resultados
conclusión
deuda transferida a Fase 6 si aplica
```

No inventar resultados de simulación.

---

# 24. Entrega de Codex antes de pedir al usuario ejecutar Gazebo

Codex debe proporcionar:

1. resumen de cambios;
2. archivos modificados;
3. origen y semántica exacta de cada métrica ORB;
4. métricas que no pudieron obtenerse y por qué;
5. tests/build realizados;
6. comandos exactos de compilación;
7. comandos exactos para las pruebas;
8. rutas de salida de telemetría;
9. comando de análisis offline;
10. resultado esperado/criterio de parada de cada prueba.

Localizar y reutilizar scripts/launch existentes; no inventar comandos si el repositorio ya define el flujo correcto.

---

# 25. Orden obligatorio

```text
1. Auditar qué métricas ORB están realmente disponibles
        ↓
2. Implementar telemetría sin cambiar comportamiento
        ↓
3. Build + unit tests
        ↓
4. Prueba simple estable: GT + ORB shadow
        ↓
5. Trayectoria problemática: GT + ORB shadow
        ↓
6. Análisis temporal + bins + eventos
        ↓
7. Ruta corta/lenta/favorable: GT + ORB shadow
        ↓
8. Ruta favorable gobernada por ORB
        ↓
9. Repeticiones
        ↓
10. Decisión Fase 5 / deuda Fase 6
```

No saltar directamente a ORB control.

---

# 26. STOP conditions

Parar y no acumular más pruebas si aparece, con evidencia visual buena:

```text
salto O en cambio KF
predictor introduce omega/velocidad falsa
W mueve O
raw ORB estable pero NavigationState diverge
incoherencia pose/velocity
```

Entregar entonces:

```text
timestamp/frame exacto
entrada al componente
salida del componente
primer punto donde se rompe el contrato
```

y esperar autorización antes de una nueva corrección funcional.

---

# 27. Qué no es una conclusión válida

No aceptar:

```text
"ORB ve pocos puntos"
"la fachada no tiene textura"
"seguramente están lejos"
"tracking se pierde, así que es visual"
"hay muchos puntos, así que ORB está bien"
```

sin series temporales y métricas del mismo frame.

---

# 28. Resultado final obligatorio

El informe debe acabar explícitamente con una de estas dos conclusiones.

## A. FASE 5 SUPERADA

```text
La arquitectura de estimación/control funciona con evidencia ORB suficiente.
Los fallos restantes están causalmente asociados a zonas de observabilidad
visual insuficiente.

La selección activa de trayectorias que mantengan evidencia suficiente
corresponde a Fase 6.
```

Añadir como deuda para Fase 6 el aprovechamiento de las métricas caracterizadas aquí para evaluar/evitar regiones pobres.

No implementar todavía esa lógica de Fase 6.

## B. FASE 5 NO SUPERADA

```text
Existe un fallo de estado/control aun con evidencia visual adecuada.
```

Identificar si el primer problema está en:

```text
raw ORB
reference KF continuity
O reconstruction
predictor p/v
predictor R/omega
timing
pose/velocity coherence
W contamination
frames/extrinsics
otro componente demostrado
```

---

# 29. Principio de cierre

La cuestión no es demostrar que ORB-SLAM3 nunca se pierde.

La cuestión es demostrar que:

```text
evidencia visual adecuada
        ↓
ORB adecuado
        ↓
NavigationState coherente
        ↓
control estable
```

Si eso se cumple de forma repetible, **Fase 5 ha cumplido su responsabilidad**.

La prevención de trayectorias que lleven al dron a regiones visualmente pobres será responsabilidad de **Fase 6**.
