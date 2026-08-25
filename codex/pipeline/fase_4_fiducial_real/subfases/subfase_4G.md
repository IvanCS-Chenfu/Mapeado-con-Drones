# Subfase 4G - Semantica tag->fiducial, zona segura, fusion multicara y seleccion por KF

## Estado

```text
CONSEGUIDA
Preparacion: cerrada
Autorizacion funcional: concedida y consumida
```

## Detalle largo

```text
subfases/detalle/subfase_4G_DETALLE.md
```

## Dependencias

`4A` y `4F` conseguidas.

## Objetivo

Convertir batches exactos de tags en observaciones fiduciales funcionales del
Servidor: resolver `tag_id -> object_id`, aplicar una zona segura configurable
por fiducial
logico, comprobar consistencia multicara, fusionar poses coherentes y seleccionar
un unico fiducial funcional por KF.

## Decisiones activas

- Servidor carga `fiducial_objects.yaml` con `yaml-cpp` y posee
  `object_T_tag`, `world_T_object` y politica de anchor;
- el Dron no agrupa por cubo, no recibe semantica de objeto y envia cada
  `camera_T_tag` con sus metricas visuales;
- detecciones fuera de zona no se destruyen, pero no son aptas para anchor;
- el perfil inicial usa `[1,5] m`, con mínimo y máximo configurables;
- si cualquier tag de un fiducial esta fuera del rango configurado, ese fiducial completo
  queda no apto para anchor en ese KF;
- las observaciones no aptas siguen llegando y se conservan para Fase 6;
- el scoring 3R no cambia en Fase 4; compartir rangos queda como revision futura;
- la fusion conserva todas las caras, no promedia RPY de forma ingenua y usa
  peso base `max(quality_score, epsilon) * sqrt(tag_area/max_tag_area)` mas
  reponderacion robusta por residual geometrico;
- los umbrales multicara iniciales son configurables, con perfil de `0.15 m` y
  `15 grados`; sin una solucion estable el objeto no es apto para anchor;
- si hay varios fiduciales aptos, se elige uno por calidad y desempate
  determinista; los demas quedan diagnosticos;
- el Servidor conserva por dron una FIFO de los ultimos 50 KFs interpretados,
  incluidos primary, secundarios y no aptos con su motivo, para Fase 6.

## Archivos probables al ejecutar

- interpretador fiducial en Servidor;
- acceso a configuracion geometrica de 4A;
- estructuras diagnosticas/resultados;
- tests de consistencia, seleccion y calidad;
- `system_architecture` si cambia relacion o telemetry live.

## Prohibido

- mover semantica tag->cubo al wrapper;
- usar GT para validar o seleccionar candidatos;
- convertir fiduciales en loops;
- aceptar multiples anchors funcionales del mismo KF si el backend de Fase 3 no
  los soporta.

## Pruebas requeridas

Tags conocidos/desconocidos, multiples caras de un fiducial, caras incoherentes,
varios fiduciales en un KF, límites configurados incluidos exactamente 1 y 5 m,
fuera de rango conservado sin anchor, secundarios diagnosticos, calidad y
seleccion determinista, reponderacion robusta y FIFO de 50 KFs por dron.

## Criterio de exito

El Servidor genera un `world_T_camera_target` trazable para un unico fiducial
funcional apto, conserva diagnostico de detecciones secundarias/no aptas y deja
la entrada lista para 4H sin GT funcional.

## Resultado

Implementado y validado en pruebas 214/215: tres objetos, fusion multicara,
visitas tolerantes a llegada fuera de orden y FIFO reciente. Evidencia en
`historial/por_subfase/historial_4G_RESUMEN.md`.
