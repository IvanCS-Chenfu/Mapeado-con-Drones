# Pipeline Fase 5 — Poses de los drones sin Ground Truth — RESUMEN

## Frontera cross-group cerrada y obligación de actualizar system_architecture

La frontera de despliegue de Fase 5 queda fijada:
```text
Servidor <-> orbslam3 (wrapper)
```
`dron_individual` no abrirá comunicación directa con Servidor. 5A puede decidir dónde
vive el estimador embarcado y qué contrato local usa `orbslam3` hacia control, pero no
puede romper esa frontera.

GT se mantiene como métrica externa y desaparece del camino funcional en 5H.

Actualizar `system_architecture` explícitamente en 5A, 5B, 5D, 5E, 5H y 5I según los
cambios reales de interfaces. No dibujar conexiones futuras antes de que existan.

## Estado

```text
sin hacer
```

## Objetivo

Sustituir progresivamente `sensor/GT/pose` y `sensor/GT/vel` como entradas funcionales del control por una estimación híbrida local-global:

```text
cámara
  -> ORB-SLAM3
  -> pose local continua en map_local
  -> correcciones de KFs calculadas desde GlobalPoseStore en el servidor
  -> estimador ligero embarcado
  -> pose global cuando el submapa está anclado
  -> velocidad estimada sin GT
  -> gen_tray / control_calcular_fuerzas
```

La pose final debe publicarse a la frecuencia de la pose local de ORB-SLAM3, sin esperar una ida y vuelta al servidor por cada frame.

## Reglas funcionales cerradas

1. `submapa = (drone_id, map_epoch)` sigue siendo una invariante.
2. ORB-SLAM3 conserva su mapa y poses locales; las correcciones globales no se reinyectan en el mapa interno de ORB-SLAM3.
3. El servidor calcula/selecciona información global de KFs y la envía al dron; un estimador embarcado ligero combina esa información con cada nueva `pose_local`.
4. Método preferido: usar el `reference_keyframe_id` real de ORB-SLAM3 si 5A demuestra que puede exponerse sin modificar problemáticamente el core. Fallback: usar KFs corregidos cercanos/relevantes.
5. La transformación conceptual principal es:

```text
C_KF = W_T_KF * inverse(L_T_KF)
W_T_camera(t) = C_KF * L_T_camera(t)
```

6. Si no existe anchor/globalización pero ORB mantiene tracking, la pose local sigue siendo válida y el dron puede moverse relativamente.
7. Si llega un goal con `absoluto_x/y/z/yaw=true` mientras no hay pose global, se conserva el valor del goal pero cada flag absoluto se trata exactamente como si fuese `false`.
8. El frame de una trayectoria se congela al aceptarla:
   - trayectoria iniciada en local sigue en local aunque aparezca un anchor;
   - la siguiente trayectoria ya puede usar `world`.
9. Si se pierde la pose global durante una trayectoria absoluta, esa trayectoria se cancela/interrumpe y el dron pasa a funcionamiento local si ORB sigue válido.
10. Si se pierden simultáneamente pose global y tracking local, no se considera válida ninguna pose. Se cancela la navegación normal y se entra en recuperación ciega experimental, basada en un buffer corto del último estado/movimiento válido, con límites estrictos y sin fingir que existe pose válida.
11. La recuperación ciega intentará volver hacia la zona inmediatamente anterior a la pérdida de landmarks. Se compararán variantes de inversión directa y frenado + inversión antes de fijar la política final.
12. Si ORB recupera tracking, la maniobra ciega se aborta inmediatamente. La trayectoria anterior no se reanuda automáticamente.
13. Si la Fase 3 vuelve a relacionar el submapa con una zona global conocida, se recupera pose global sin exigir necesariamente otro fiducial.
14. El suavizado de pose global no queda decidido de antemano. 5F debe comparar raw vs smoothed, medir error y delay y detenerse para que el usuario elija implementar, modificar o eliminar el suavizado.
15. La velocidad se estima desde el movimiento local de ORB, no derivando directamente una pose global que puede saltar por optimización.
16. Se estima velocidad lineal y angular. La aceleración estimada se calcula solo si `debug_acc_est=true`; por defecto debe poder quedar desactivada y no es entrada funcional del controlador actual.
17. GT solo se usa para métricas y gráficas en simulación, nunca para pose, velocidad, recuperación, selección de KFs o control final.
18. 5C es una estrategia inicial revisable. Si 5F/5G muestran que la selección/corrección de KFs produce error, se reabre 5C y se cambia el método.

## Subfases

| Subfase | Nombre | Salida principal |
|---|---|---|
| `5A` | Investigación y cierre técnico | Código real estudiado y MDs 5B–5I refinados con paths, topics, mensajes y pruebas exactos. |
| `5B` | Pose local, estados y navegación sin anchor | `LOCAL_ONLY` utilizable, semántica absoluta→relativa, frame congelado y gestión inicial de pérdidas. |
| `5C` | Correcciones de KFs desde el servidor | El backend obtiene `C_KF` coherentes desde raw + `GlobalPoseStore` para el mismo submapa. |
| `5D` | Transporte servidor→dron | Correcciones versionadas, acotadas y no obsoletas llegan al dron sin round-trip por frame. |
| `5E` | Estimador embarcado de pose | Cada `pose_local` genera pose estimada a frecuencia ORB usando la última corrección válida. |
| `5F` | Pose vs GT y experimento de suavizado | Gráficas/métricas raw vs smoothed vs GT + delay; decisión explícita del usuario. |
| `5G` | Velocidad, aceleración debug y recuperación ciega | Velocidad lineal/angular sin GT, `debug_acc_est`, métricas vs GT y estrategia de recuperación probada. |
| `5H` | Sustitución final de GT en control | `gen_tray` y `control_calcular_fuerzas` usan estado estimado; no dependen de `sensor/GT/*`. |
| `5I` | Integración y regresión multi-dron | Adquisición inicial de anchor + prueba larga de dos drones + pérdidas/relocalización sin GT funcional. |

## Dependencias

```text
Fase 3 Sparse Global
        +
Fase 4 Fiducial Real
        ↓
5A -> 5B -> 5C -> 5D -> 5E -> 5F -> 5G -> 5H -> 5I
```

- 5F puede obligar a volver a 5C, 5D o 5E.
- 5G puede obligar a volver a 5E/5F si el frame o la continuidad de pose contaminan la velocidad.
- 5H no se ejecuta hasta que el usuario considere suficientemente bajos los errores observados en 5F y 5G.

## Puerta final

La Fase 5 solo se cierra cuando:

- un dron puede arrancar sin anchor y moverse usando únicamente pose local;
- obtiene un anchor visual y empieza a disponer de pose global sin salto funcional de la trayectoria local en curso;
- la pose estimada sigue la frecuencia ORB con delay medido y aceptable;
- la velocidad estimada es utilizable por el controlador;
- GT queda fuera del camino funcional de `gen_tray` y `control_calcular_fuerzas`;
- una pérdida de globalización no inmoviliza al dron si ORB sigue válido;
- una pérdida total entra en recuperación acotada y no reutiliza poses stale como válidas;
- dos drones realizan una prueba larga alrededor del edificio usando pose/velocidad estimadas;
- GT aparece únicamente en gráficas y métricas externas.
