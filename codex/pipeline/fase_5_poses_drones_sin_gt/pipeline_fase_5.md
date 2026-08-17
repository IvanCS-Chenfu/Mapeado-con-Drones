# Pipeline Fase 5 — Poses de los drones sin Ground Truth

Resumen de entrada:

```text
codex/pipeline/fase_5_poses_drones_sin_gt/pipeline_fase_5_RESUMEN.md
```

## Estado

```text
sin hacer
```

## Objetivo general

Proporcionar a cada dron un estado de navegación continuo y utilizable por el control sin Ground Truth, combinando:

- pose local de alta frecuencia de ORB-SLAM3;
- relación global de los KeyFrames mantenida por el servidor en `GlobalPoseStore`;
- correcciones globales versionadas y ligeras servidor→dron;
- un estimador embarcado que no espere al servidor en cada frame;
- velocidad lineal/angular derivada de la evolución local, con filtrado de baja latencia;
- una política explícita para ausencia de anchor, pérdida de globalización y pérdida total de tracking.

La Fase 5 no cambia la autoridad de los datos de la Fase 3:

```text
RawMapDatabase     = verdad cruda recibida de ORB-SLAM3
GlobalPoseStore    = poses world/anchors/correcciones aceptadas
ORB-SLAM3 local    = tracking y mapa local, sin reinyectar correcciones globales
```

## Dependencias de entrada

La fase presupone cerrados o suficientemente estables:

- Fase 3 — mapa sparse global, identidad `(drone_id, map_epoch)`, `RawMapDatabase` y `GlobalPoseStore`;
- Fase 4 — observación fiducial visual y creación/recuperación de anchors sin GT funcional;
- Fase 1 — generación de trayectorias y control del cuadricóptero;
- Fase 2 — separación Dron / Servidor / Simulación y política vigente de `orbslam3_msgs`.

5A debe reconciliar el workspace real posterior a esas fases. Ninguna ruta física de repositorio o copia de interfaz debe asumirse si Fase 2 la cambió.

## Arquitectura objetivo

```text
                         DRON

cámara estéreo
    ↓
ORB-SLAM3 / wrapper
    ├── orbslam/pose_local (L_T_camera, alta frecuencia)
    ├── tracking state / map_epoch / reference KF  [contrato a cerrar en 5A]
    └── orb_map_delta
              │
              │ ROS 2
              ▼
                         SERVIDOR
        RawMapDatabase + GlobalPoseStore
              │
              ├── L_T_KF desde raw
              ├── W_T_KF aceptada/corregida
              └── C_KF = W_T_KF * inv(L_T_KF)
              │
              │ correcciones versionadas, no pose por frame
              ▼
                         DRON
        estimador local-global ligero
              │
              ├── pose local válida siempre que ORB tenga tracking
              ├── pose global si existe corrección/anchor válida
              ├── velocidad lineal/angular estimada
              └── estado LOCAL_ONLY / GLOBAL_VALID / LOCALIZATION_LOST
              │
              ▼
        gen_tray + control_calcular_fuerzas
              │
              ▼
        aplicar_fuerzas_dron -> motores
```

## Ecuación principal

Para un KeyFrame de referencia `KF` del mismo `(drone_id, map_epoch)`:

```text
C_KF = W_T_KF * inverse(L_T_KF)
W_T_camera(t) = C_KF * L_T_camera(t)
```

Después se aplica la extrínseca cámara↔cuerpo acordada para obtener la pose del dron (`base_link`/cuerpo) en el frame correspondiente.

El cálculo por frame en el dron debe ser pequeño: validación de estado + selección de corrección cacheada + multiplicación de transformaciones + publicación. Matching, optimización, selección pesada de candidatos y construcción global permanecen en el servidor/backend.

## Selección del KeyFrame de referencia

Prioridad acordada:

1. 5A investiga si el wrapper puede publicar el `reference_keyframe_id` real usado por ORB-SLAM3 para el frame actual sin modificar de forma problemática el core `ORB_SLAM3`.
2. Si es viable, ese ID será la referencia principal.
3. Si no es viable, el dron usa un conjunto cacheado de KFs corregidos y selecciona un KF cercano/relevante en coordenadas locales, siguiendo una estrategia inicialmente inspirada en el `CORRECTED_KEYFRAME_RELATIVE` existente.
4. 5C no queda congelada: si 5F muestra error sistemático por esta elección, se reabre y se sustituye la estrategia.

No se debe inferir temporalidad de un delta por orden de llegada o por ID: el wrapper puede reenviar KFs antiguos cuando cambia su estado.

## Estados funcionales de pose

### `GLOBAL_VALID`

```text
ORB tracking válido
+
submapa relacionado con world
+
corrección global válida
```

Permite objetivos relativos y absolutos.

### `LOCAL_ONLY`

```text
ORB tracking válido
+
no existe pose global válida
```

La pose local continúa siendo una entrada funcional. Se permiten movimientos relativos.

Si un goal llega con uno o varios `absoluto_* = true`, el valor numérico no se descarta: para cada eje sin referencia global, se aplica exactamente la semántica que ya tendría ese mismo valor con `absoluto_*=false`.

### `LOCALIZATION_LOST`

```text
ORB tracking no válido
+
no existe otra fuente de pose/velocidad independiente
```

No hay IMU ni otra fuente de navegación prevista en esta fase. Por tanto:

- ninguna pose stale se presenta como actual;
- se cancela/interrumpe navegación normal;
- se conserva un buffer corto del último estado válido;
- 5G prueba una maniobra ciega breve destinada solo a volver a una zona con landmarks;
- al recuperar ORB, la maniobra se aborta inmediatamente;
- si no se recupera dentro de límites de seguridad, no se sigue explorando a ciegas.

## Regla de congelación del frame de trayectoria

La interpretación de una trayectoria se fija al aceptarla.

Caso A — se obtiene anchor durante una trayectoria local:

```text
LOCAL_ONLY -> goal relativo/local -> aparece anchor -> GLOBAL_VALID
```

La trayectoria en curso sigue usando pose local hasta terminar. El siguiente goal puede usar `world`.

Caso B — se pierde pose global durante una trayectoria absoluta:

```text
GLOBAL_VALID -> goal world -> se pierde global
```

La trayectoria absoluta se interrumpe. Si ORB sigue válido, el dron pasa a `LOCAL_ONLY`; la nueva maniobra de frenado/continuación segura se expresa en local. No se convierte silenciosamente el objetivo world restante en un objetivo relativo equivalente.

## Correcciones servidor→dron

El servidor no responde a cada frame. Las correcciones se actualizan por eventos:

- llegada/commit de deltas que materializan o cambian KFs;
- aparición de una pose world nueva en `GlobalPoseStore`;
- optimización/propagación que cambia la pose world de un KF relevante;
- recuperación de un submapa previamente globalizado;
- cambios de `map_epoch` que invalidan el cache anterior.

El contrato final debe incluir como mínimo información equivalente a:

```text
drone_id
map_epoch
keyframe_id
map_revision / pose revision
timestamp
L_T_KF o datos suficientes para seleccionar/verificar
W_T_KF o C_KF
estado/validez
```

5A y 5D fijarán si se extienden `MapCorrection` / `CorrectedKeyFrameArray` o se crea una interfaz específica. No se ejecutará 5D con identidad incompleta de epoch/revisión.

## Frecuencia y latencia

La salida estimada no debe tener la frecuencia del servidor. Debe seguir la fuente local:

```text
cada orbslam/pose_local nueva
    -> estimador ligero
    -> nueva pose estimada
```

La última corrección global cacheada se reutiliza hasta que llega una revisión más nueva. Mensajes atrasados o de otro epoch se descartan.

5A mide la frecuencia y latencia de partida. 5E mide el coste añadido por el estimador. 5F compara raw/smoothed y cuantifica el delay del suavizado antes de decidir si se adopta.

## Velocidad y aceleración

La velocidad no se deriva directamente de saltos de pose global. Se estima desde poses locales consecutivas y sus timestamps:

```text
v_local <- delta de traslación local / dt
omega_local <- delta de orientación / dt
```

Después se expresa en el frame requerido por el controlador usando la rotación válida del sistema local/global.

El controlador actual consume pose y `TwistStamped`; su aceleración actual no es una entrada de estado. La aceleración deseada viene del feedback de trayectoria. Aun así, 5G calculará aceleración estimada opcionalmente para diagnóstico si:

```yaml
debug_acc_est: true
```

Con `false`, ese cálculo debe evitarse.

## Suavizado

No se fija una solución por adelantado.

5F debe publicar/medir simultáneamente, cuando sea posible:

```text
pose_raw
pose_smoothed
GT solo diagnóstico
```

y comparar:

- RMSE/MAE/máximo de posición;
- error de yaw/orientación;
- frecuencia;
- latencia de procesamiento;
- saltos ante nuevas correcciones/optimizaciones.

Después Codex debe detenerse y presentar evidencia al usuario. La autorización queda suspendida hasta que el usuario decida:

```text
usar suavizado
mejorarlo y repetir
eliminarlo
```

La selección podrá revisarse otra vez en hardware real.

## Gráficas y métricas

Se reutiliza la filosofía de `simulacion_dron/src/graficar/`:

```text
numeric_array + labels_array -> graficar.py
```

pero 5A debe confirmar si conviene reutilizar esa interfaz o crear topics namespaced específicos para no mezclar drones.

5F: pose estimada vs GT.

5G: velocidad estimada vs GT y, con debug, aceleración estimada vs GT.

GT no puede realimentar ningún estimador ni decidir online qué corrección aceptar.

## Recuperación tras pérdida total

Al perder tracking local:

1. registrar el último estado válido y un buffer corto de estados recientes;
2. cancelar la trayectoria normal;
3. marcar `LOCALIZATION_LOST`;
4. no publicar una pose stale como válida;
5. estimar la dirección de movimiento inmediatamente anterior;
6. probar una maniobra ciega breve en sentido contrario con límites estrictos;
7. comparar al menos:
   - inversión directa/acotada;
   - frenado + inversión;
8. seguir alimentando imágenes a ORB-SLAM3;
9. abortar inmediatamente la maniobra si vuelve tracking;
10. no reanudar automáticamente el goal cancelado;
11. si el nuevo mapa/epoch se relaciona con el mapa global mediante Fase 3, recuperar `GLOBAL_VALID`; en caso contrario quedar en `LOCAL_ONLY`.

La recuperación no pretende navegar a ciegas ni volver exactamente a una pose: intenta regresar a la región visual inmediatamente anterior para recuperar landmarks.

## Subfases y puertas

| Subfase | Puerta de salida |
|---|---|
| `5A` | No quedan incógnitas técnicas de paths/topics/clases que impidan ejecutar 5B–5I; los MD quedan refinados. |
| `5B` | Pose local y estados están definidos; movimientos sin anchor y transiciones de frame se comportan según contrato; baseline/post de pérdidas registrado. |
| `5C` | El backend obtiene correcciones `C_KF` coherentes del mismo submapa y puede revisarlas tras optimización. |
| `5D` | El dron recibe correcciones versionadas sin backlog obsoleto y rechaza epoch/revisión incorrectos. |
| `5E` | La pose estimada sale a frecuencia próxima a `pose_local`, sin round-trip y con modo local/global correcto. |
| `5F` | Existen gráficas/métricas de pose y delay; el usuario ha tomado una decisión explícita sobre suavizado. |
| `5G` | Velocidad lineal/angular sin GT validada; aceleración debug opcional; recuperación ciega caracterizada. |
| `5H` | `gen_tray` y `control_calcular_fuerzas` ya no se suscriben funcionalmente a GT. |
| `5I` | Adquisición de anchor + prueba larga de dos drones + regresiones de pérdida pasan sin GT funcional. |

## Reglas de ejecución

- Cada subfase aplica la puerta de preparación de `AGENTS.md` antes de modificar código.
- 5A puede editar 5B–5I como parte de su objetivo, pero no ejecutar esas subfases.
- Si 5A detecta que una decisión funcional acordada no puede implementarse con la arquitectura real, debe parar y preguntar; no cambiar la intención silenciosamente.
- 5C/5D/5E deben evitar meter matching/optimización en el dron.
- No tocar `ORB_SLAM3` salvo que 5A demuestre necesidad y se reciba autorización explícita; preferir cambios en wrapper.
- No usar GT para resolver pérdidas, escoger KFs, corregir pose o estimar velocidad.
- Los logs completos solo son entrada de reductores; Codex lee reducidos/sublogs.
- Cada ejecución real crea historial por subfase; esta entrega inicial no contiene historial inventado.

## Resultado esperado de la fase

Al finalizar, cada dron dispone de un estado estimado usable en tiempo real:

```text
sin anchor + ORB OK     -> pose local + movimientos relativos
con anchor + ORB OK     -> pose global + relativos/absolutos
pierde global + ORB OK  -> vuelve a local e interrumpe goals world
pierde ORB              -> cancela navegación + recuperación ciega acotada
recupera ORB            -> local; global si Fase 3 recupera relación world
```

El control real deja de depender de Ground Truth y la simulación conserva GT exclusivamente como instrumento de evaluación externa.
