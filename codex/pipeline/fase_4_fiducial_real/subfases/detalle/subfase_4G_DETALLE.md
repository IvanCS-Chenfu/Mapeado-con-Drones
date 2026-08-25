# Detalle largo importado - Subfase 4G

Este archivo conserva el detalle del contrato revisado importado desde `Fase_4_completa_4A_4I_muy_detallada.zip`. El contrato ejecutable corto esta en `../subfase_4G.md`.

# Subfase 4G — Semántica tag→fiducial, zona segura, fusión multicara y selección por KF

## Estado

```text
sin hacer — diseño funcional cerrado
```

## Dependencias

```text
4A: geometría/configuración
4F: batch asociado al KF exacto
```

4G es la subfase que transforma percepción de tags en **una observación fiducial funcional** compatible con el backend existente.

## Principio de ownership

Toda la semántica del objeto permanece en Servidor.

Dron nunca recibe:

```text
object_id
face
object_T_tag
world_T_object
```

El servicio 4D no se amplía para mandar esos datos. El posible ahorro de tráfico no compensa romper esta separación.

## Configuración del Servidor

`orbslam3_server` carga directamente su copia de deployment de
`fiducial_objects.yaml` mediante `yaml-cpp`. El servicio 4D continúa entregando
al Dron solo los datos necesarios para detectar cada tag; no expone
`object_id`, `object_T_tag` ni `world_T_object`.

El Servidor debe poder resolver para cada `tag_id`:

```text
tag_id -> object_id/fiducial_id lógico
         cara
         size/geometría del object
         world_T_object
```

`object_T_tag` se deriva determinísticamente de la geometría de 4A:

- tag centrado;
- orientación interna 0°;
- normal saliente;
- convención fija por `pos_x/neg_x/...`.

No se mantendrá una segunda transformación manual que pueda contradecir dimensiones/cara.

La configuración respeta ADR 0009 y la réplica/deployment validada entre
Servidor y Simulación. El Servidor no lee la copia YAML del grupo Simulación en
runtime: cada deployment usa su copia validada por los tests de consistencia.

## Entrada

```text
KF exacto
observations[] = {
   tag_id,
   camera_T_tag,
   quality_score,
   reprojection_error_px,
   tag_area_px2,
   pose_ambiguity...
}
```

## Paso 1 — Resolver tags conocidos

Cada tag debe mapear a un único fiducial lógico. Un tag desconocido se marca `unknown_tag` y no invalida necesariamente otros grupos válidos del mismo batch.

IDs de tag pueden incluir 0. `object_id/fiducial_id` puede mantenerse positivo para ser compatible con el backend actual.

## Paso 2 — Agrupar por fiducial lógico

Ejemplo:

```text
KF 50:
  101 -> object 1
  102 -> object 1
  105 -> object 1
  201 -> object 2

Groups:
  object 1 = [101,102,105]
  object 2 = [201]
```

Una imagen puede ver hasta tres caras de un cubo; por tanto multi-tag del mismo objeto es un caso normal, no una anomalía.

## Paso 3 — Zona segura configurable

El perfil inicial declara `min_distance_m=1.0` y `max_distance_m=5.0`. El
Servidor posee la semántica y Simulación selecciona el perfil de deployment;
los límites no se derivan del baseline ni cambian el scoring 3R.

Para cada tag se calcula:

```text
d_tag = || translation(camera_T_tag) ||
```

Regla conservadora acordada **por fiducial lógico**:

```text
si todos los tags observados del object están dentro del rango configurado:
    el object puede continuar como anchor candidate

si al menos uno está <1 m o >5 m:
    todo el object queda detected_not_anchor_eligible
```

No se necesita enviar `tag→centro_cubo` al dron solo para medir esta zona.

Un object no elegible:

- no crea anchor;
- no crea optimization task;
- no se descarta del sistema de observación;
- queda disponible para diagnóstico/futuro uso de Fase 6.

Si otro object distinto del mismo KF sí cumple el rango, se evalúa independientemente.

## Paso 4 — Transformar cada cara a una estimación común

Para cada tag conocido, Servidor usa la geometría del object para obtener una pose comparable del mismo fiducial/cámara.

Relaciones a verificar contra frames reales:

```text
camera_T_object_i = camera_T_tag_i * inverse(object_T_tag_i)
world_T_camera_i  = world_T_object * inverse(camera_T_object_i)
```

Equivalentemente puede usarse `world_T_tag * inverse(camera_T_tag)`. La implementación debe elegir una formulación única, con tests de equivalencia y convención.

## Paso 5 — Consistencia multicara

Antes de promediar:

1. todas las poses deben ser finitas;
2. la geometría debe tener handedness/orientación correcta;
3. comparar traslación y rotación entre candidatos del mismo object;
4. medir el residual geométrico de cada cara respecto a la solución común;
5. no permitir que una media oculte un error de configuración/mirror.

Los umbrales son configurables. El perfil inicial usa `0.15 m` para traslación
y `15 grados` para rotación. No se aplica una votación binaria que elimine una
cara solo porque otras dos formen mayoría: todas se conservan y su influencia
se reduce progresivamente según su residual. Si no existe una solución común
estable, el object completo queda no apto para anchor en ese KF.

## Paso 6 — Fusión ponderada

Si hay una sola cara válida, su estimación es el candidato del fiducial.

Si hay varias caras, se crea una pose fusionada robusta. El peso base acordado
es:

```text
w_base_i = max(quality_score_i, epsilon)
           * sqrt(tag_area_px2_i / max_tag_area_px2)
w_i = w_base_i * robust_weight(residual_geometrico_i)
```

Se espera que una cara:

- grande en imagen;
- con bajo error de reproyección;
- poco ambigua;
- poco oblicua;

tenga más peso que una cara lejana/oblicua/pobre.

### Traslación

Puede usar media ponderada normalizada si las observaciones pasaron consistencia.

### Rotación

No promediar roll/pitch/yaw componente a componente. Usar media de quaternions con control de signo o método apropiado de SO(3)/SE(3).

### Calidad global del fiducial

`reprojection_error_px` y `pose_ambiguity` se conservan para diagnóstico y
desempate. `quality_score` por sí solo no demuestra consistencia entre caras,
por lo que no sustituye al residual geométrico.

Calcular `fiducial_quality`/`object_quality` documentada a partir de:

- calidades individuales;
- número de caras coherentes;
- dispersión entre estimaciones;
- peor/mejor error según la fórmula acordada.

La calidad global debe penalizar la dispersión residual y declarar la solución
no estable cuando no quede soporte geométrico suficiente. El kernel robusto y
`epsilon` son detalles de implementación documentados y cubiertos por tests;
no deben alterar los umbrales funcionales acordados.

## Paso 7 — Varios fiduciales en un KF

El Servidor **sí obtiene e interpreta todos** los fiduciales visibles. Sin embargo, el tratamiento existente de Fase 3 está diseñado para un fiducial funcional por KF.

Por tanto 4G selecciona uno.

### Candidatos elegibles

Solo entran en selección los que:

- tienen al menos un tag válido;
- todos sus tags observados cumplen el rango configurado;
- pasan consistencia geométrica;
- producen pose global/cámara válida.

### Política de selección

1. mayor `fiducial_quality` global;
2. desempate por mejor error/consistencia según métrica estable;
3. último desempate determinista por `object_id` para reproducibilidad.

Resultado:

```text
selected_primary_fiducial
```

Los restantes:

```text
detected_secondary
```

No se crean varias observaciones funcionales para `FiducialAnchorManager` en el mismo KF.

## Conservación para Fase 6

Las observaciones secundarias y las detectadas fuera de zona segura pueden servir en Fase 6 para una futura tarea tipo “buscar/acercarse a fiducial”.

Fase 4 **no implementa esa tarea ni decide su API**, pero evita destruir la
información mediante una FIFO en Servidor con los últimos 50 KFs interpretados
por dron. Cada entrada conserva `(drone_id,map_epoch,keyframe_id)`, primary,
secundarios, no aptos y sus motivos. La capacidad es independiente del sidecar
pendiente de 4F. Fase 6 podrá usarla como hint reciente, nunca como confirmación
directa de anchor ni como sustituto de una observación visual vigente.

## Salida funcional de 4G

Un máximo de una observación backend por KF:

```text
FiducialObservation {
  keyframe_id exacto
  fiducial_id/object_id seleccionado
  visit_id visual asignado en 4H
  world_T_camera_target fusionado
  keyframe_stamp
  source = visual_fiducial
  quality = información normalizada
}
```

No aplicar `body_T_camera`: la salida ya es objetivo de cámara.

## Rechazos pertenecientes a 4G

La antigua 4J queda absorbida parcialmente aquí. 4G debe resolver desde el principio:

- `unknown_tag`;
- ID duplicado en configuración;
- group fuera del rango configurado;
- geometría/transform no finita;
- multiface inconsistente;
- multiobject donde ningún candidato es fiable;
- selección determinista.

No implementar primero una versión insegura para “arreglarla después”.

## Tráfico

Se mantiene la decisión de recibir tags individuales y fusionar aquí. 4H medirá el tráfico real. No mover fusión al Dron sin evidencia de cuello de botella.

## Pruebas

### A. Una cara
Debe reconstruir correctamente `world_T_camera_target`.

### B. Dos/tres caras del mismo object
Debe verificar coherencia y producir una sola pose fusionada.

### C. Caras con calidades y residuales distintos
Inyectar una cara perturbada y comprobar que sigue auditada pero pierde peso.
Cubrir también dos caras malas de score bajo frente a una cara buena de score
alto, y un caso sin solución estable que no produzca anchor.

### D. Fuera de rango
Un tag del grupo a 5.2 m y otro a 4.9 m -> object entero no apto para anchor.

### E. Dos objects
Ambos se interpretan; solo uno se selecciona funcionalmente.

### F. Primary por calidad
Crear dos candidatos elegibles con calidad distinta; gana el mejor de forma reproducible.

### G. Secondary preservado
Verificar que el candidato no seleccionado sigue observable/auditable para uso futuro.

### H. Tag desconocido
No afecta a otro object válido del mismo KF.

## Logs

```text
FID-OBJECT-GROUP
FID-OBJECT-RANGE
FID-OBJECT-INCONSISTENT
FID-OBJECT-FUSED
FID-OBJECT-PRIMARY
FID-OBJECT-SECONDARY
FID-OBJECT-UNKNOWN-TAG
```

## Criterio de éxito

- toda semántica permanece en Servidor;
- no se amplía el servicio 4D con geometría de objetos;
- regla de rango configurado aplicada como se acordó;
- fusión multicara ponderada y geométricamente correcta;
- solo un fiducial funcional por KF;
- secundarios preservados;
- salida `world_T_camera_target` lista para backend existente;
- cero GT funcional.
