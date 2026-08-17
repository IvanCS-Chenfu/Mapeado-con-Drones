# Pipeline Fase 4 — Fiducial Real

Resumen de entrada:

```text
codex/pipeline/fase_4_fiducial_real/pipeline_fase_4_RESUMEN.md
```

## Estado

```text
sin hacer
```

No existe evidencia de ejecución de esta fase en el historial. Las carpetas de historial se crean vacías de forma intencionada. Ninguna subfase puede pasar a `realizado` sin build, pruebas, logs y documentación posterior.

## Objetivo general

Sustituir el mecanismo fiducial simulado basado en Ground Truth por fiduciales visuales observables desde la cámara del dron. Cada observación visual debe quedar asociada de forma inequívoca al KeyFrame exacto que utilizó esa imagen, transportar al servidor las marcas vistas y sus poses relativas respecto a la cámara, y permitir que el servidor ancle o corrija submapas en `world` sin aceptar GT como entrada funcional.

La fase debe conservar la arquitectura sparse-global ya existente: los fiduciales son observaciones absolutas, no loops; `RawMapDatabase` mantiene el estado ORB crudo; `GlobalPoseStore` mantiene la autoridad de poses globales/optimizadas; las revisitas fiduciales pueden crear tareas de optimización; una pérdida del detector nunca debe bloquear el tracking ni la publicación del mapa.

## Decisiones ya cerradas para esta fase

1. El fiducial físico será un cubo estático con una marca planar en la cara superior y en las caras laterales habilitadas.
2. Cada cara tendrá un `tag_id` diferente. Varios tags pueden pertenecer al mismo cubo lógico.
3. La primera familia a implementar será `AprilTag 36h11`, usando el diccionario equivalente de OpenCV si la versión disponible lo soporta.
4. ORB-SLAM3 no detectará fiduciales. Solo expondrá al wrapper un evento inequívoco cuando la imagen actual haya originado un nuevo KeyFrame.
5. El wrapper conservará la imagen izquierda exacta usada por ORB-SLAM3 y solo ejecutará el detector cuando esa imagen haya creado un KF.
6. El wrapper no agrupará tags por cubo ni fusionará poses. Para cada tag válido enviará `tag_id + camera_T_tag` y métricas mínimas de calidad.
7. Un mismo KF puede transportar cero, una o muchas observaciones de tag. Si ve varias marcas, todas quedan ligadas al mismo `(drone_id, map_epoch, local_keyframe_id)`.
8. El servidor será responsable de conocer qué tags pertenecen al mismo cubo, calcular la geometría tag-cubo, resolver observaciones redundantes/incoherentes y producir restricciones absolutas de cámara.
9. La asociación funcional nunca se hará por proximidad temporal a otro KF. El timestamp solo se usa como comprobación de coherencia.
10. GT queda prohibido como entrada de anchor o corrección. Solo puede emplearse como métrica externa de simulación/debug.

## Arquitectura objetivo

```text
camera/left + camera/right
        |
        v
ORB_SLAM3_ROS2 wrapper
        |
        +--> ORB-SLAM3::TrackStereo(...)
        |        |
        |        +--> Tracking crea un KF o no
        |                 |
        |                 +--> evento exacto {kf_id, frame_id, timestamp}
        |
        +--> si NO hay KF: no ejecutar detector
        |
        +--> si hay KF:
                imagen izquierda exacta del KF
                        |
                        v
                OpenCV AprilTag detector
                        |
                        +--> tag 101 -> camera_T_tag101
                        +--> tag 102 -> camera_T_tag102
                        +--> tag 405 -> camera_T_tag405
                                |
                                v
                FiducialKeyFrameObservations
                                |
                                v
                         orbslam3_server
                                |
                 asociar al KF exacto en RawMapDatabase
                                |
                 tag_id -> cubo lógico + object_T_tag
                                |
                 world_T_camera objetivo
                                |
                      anchor / revisit
                                |
                   optimización fiducial
```

## Convención de transformaciones

En toda la fase, `A_T_B` significa la transformación que expresa coordenadas del frame `B` en el frame `A`.

Transformaciones principales:

```text
camera_T_tag     medida por visión
object_T_tag     conocida por geometría del cubo
world_T_object   conocida por configuración del mapa fiducial
local_T_camera   pose local del KF publicada por ORB-SLAM3
```

Relaciones esperadas:

```text
camera_T_object = camera_T_tag * inverse(object_T_tag)
world_T_tag     = world_T_object * object_T_tag
world_T_camera  = world_T_tag * inverse(camera_T_tag)
world_T_local   = world_T_camera * inverse(local_T_camera)
```

Antes de implementar estas ecuaciones se debe verificar la convención real `Tcw/Twc`, `camera` frente a `camera_optical_frame` y la orientación de cada tag.

## Configuración distribuida por paquetes

La política de configuración de paquetes se mantiene: un paquete no debe leer directamente YAML de otro grupo solo para reutilizar parámetros.

- Simulación: `simulacion_dron` mantiene su configuración de cubos, poses y texturas necesarias para Gazebo.
- Wrapper/dron: mantiene una configuración reducida con familia, tamaño físico de tag, IDs admitidos y parámetros del detector que realmente necesite.
- Servidor: mantiene una configuración reducida con `object_id`, `tag_id`, `world_T_object` y `object_T_tag` para interpretar las observaciones.

Los valores repetidos críticos, especialmente `tag_id` y tamaño físico, deben validarse mediante tests o scripts de consistencia y no copiarse silenciosamente.

## Subfases

| Subfase | Nombre | Responsabilidad principal | Puerta de salida |
|---|---|---|---|
| `4A` | Configuración de fiduciales multicara | Definir cubos, tags, tamaños, frames y convenciones. | YAML válido, IDs únicos y geometría inequívoca. |
| `4B` | Generación y spawn en Gazebo | Crear cubos estáticos con tags visibles en sus caras. | Los objetos aparecen en las poses configuradas y las texturas son correctas. |
| `4C` | Asociación exacta imagen–KeyFrame | Exponer desde ORB-SLAM3 qué imagen creó cada KF y conservarla en wrapper. | Cada evento identifica de forma determinista el KF de esa llamada a `TrackStereo`. |
| `4D` | Detección y pose de tags en wrapper | Detectar `0..N` tags solo en imágenes de KF y calcular `camera_T_tag`. | IDs y poses válidas con métricas de coste/error. |
| `4E` | Contrato ROS 2 de observaciones | Definir mensaje de un KF con un array de tags observados. | Interface generada, publicable y sin campos GT. |
| `4F` | Sincronización exacta en servidor | Asociar observaciones solo con el KF exacto, incluso si llegan fuera de orden. | No hay asociación por proximidad temporal. |
| `4G` | Interpretación tag–cubo | Resolver qué tags pertenecen al mismo/diferente cubo y producir candidatos absolutos. | Un KF con varias caras/cubos se interpreta coherentemente. |
| `4H` | Primer anchor visual | Reemplazar el anchor GT por `world_T_camera` derivado visualmente. | Primer anchor sin GT y `world_T_local` correcto. |
| `4I` | Revisitas y optimización fiducial | Reutilizar la lógica de residual pequeño/alto y tareas fiduciales. | Revisitas visuales activan la política existente sin tratar fiduciales como loops. |
| `4J` | Rechazos y degradación segura | Cubrir falsos positivos, poses incoherentes, timestamps/epochs y pérdida de detector. | Fallos de visión no rompen SLAM ni mapa. |
| `4K` | Integración multi-dron | Validar varios drones, varios tags y varios cubos en una prueba integral. | Submapas se anclan/corrigen de forma consistente sin GT funcional. |
| `4L` | Cámara real y cierre | Validar la detección con marcas físicas y cámara calibrada; cerrar documentación/regresión. | Evidencia física mínima o bloqueo externo explícito; regresiones verdes. |

## Dependencias

```text
4A -> 4B
4A + 4B + 4C -> 4D
4C + 4D -> 4E
4E -> 4F
4A + 4F -> 4G
4G -> 4H -> 4I -> 4J -> 4K -> 4L
```

`4C` puede prepararse en paralelo con `4A/4B`, pero `4D` necesita tanto una imagen de KF inequívoca como tags físicos/simulados definidos.

## Reglas transversales de implementación

- No introducir detección AprilTag dentro de ORB-SLAM3.
- No enviar imágenes en `OrbMap` ni en `OrbKeyFrame` como solución principal.
- No ejecutar el detector en todos los frames funcionalmente; solo en frames confirmados como KF.
- Usar inicialmente una sola imagen, la izquierda, para pose planar. La derecha queda fuera del baseline obligatorio.
- No agrupar tags por cubo en el wrapper.
- No usar GT para construir `world_T_camera`, `world_T_local`, residual o decisión de optimización.
- No modificar `RawMapDatabase` para guardar estado global corregido.
- No convertir observaciones fiduciales en candidatos de loop.
- No aumentar tolerancias o desactivar validaciones como solución principal a errores de frames o transformaciones.
- Mantener build/test/simulación limitados a los paquetes afectados de cada subfase.

## Regla de ejecución

Estos MD son contratos preparatorios. La primera orden futura para ejecutar una subfase no autoriza cambios funcionales de forma automática. Se debe aplicar el protocolo de preparación y confirmación definido en `AGENTS.md`: leer contexto, explicar alcance, cerrar dudas/prueba y esperar autorización explícita antes de modificar `src/`, compilar o simular.

## Criterio de cierre de Fase 4

La fase se puede marcar como `realizado` solo cuando exista evidencia de que:

1. los cubos y tags aparecen correctamente en Gazebo;
2. ORB-SLAM3 identifica de manera exacta la creación de cada KF sin realizar detección fiducial;
3. el wrapper analiza solo la imagen exacta de ese KF y puede producir `0..N` observaciones;
4. cada observación contiene un `tag_id` y una transformación `camera_T_tag` coherente;
5. el servidor asocia las observaciones únicamente a su KF exacto;
6. el servidor interpreta tags de una o varias caras/cubos sin trasladar esa lógica al wrapper;
7. el primer anchor y las revisitas funcionan sin GT funcional;
8. los fallos del detector no bloquean ORB-SLAM3 ni el transporte de mapas;
9. la prueba multi-dron completa pasa;
10. la validación con cámara real se completa o, si existe una dependencia física externa no disponible, queda explícitamente `BLOQUEADA` sin falsificar evidencia;
11. toda la documentación e historial real quedan actualizados.
