# Subfase 4I — Regresión con perfil de cámara simulada equivalente a ESP32-CAM

## Estado

```text
APLAZADA — regresión opcional futura, fuera del cierre actual de Fase 4
```

## Dependencia

`4H` conseguida con la cámara baseline de simulación.

## Objetivo

Comprobar que la solución visual no depende de una cámara simulada demasiado favorable. Se sustituirá/configurará la cámara de Gazebo con un perfil cuyas especificaciones correspondan a la **ESP32-CAM barata que se elija como hardware objetivo** y se repetirá la cadena completa de Fase 4.

Esta subfase sustituye la antigua validación física 4L. No requiere aún comprar/imprimir/montar hardware real.

El módulo y el modo exactos de ESP32-CAM se decidirán al preparar 4I; no se
anticipan ni se inventan durante los bloques anteriores.

## Regla fundamental

4I **no es una subfase para parchear el sistema hasta que pase**.

Si al cambiar la cámara aparece un problema:

```text
identificar subfase propietaria
        ↓
volver a 4A/4B/4C/4D/4E/4F/4G/4H
        ↓
corregir con causa y prueba
        ↓
repetir regresiones
        ↓
volver a 4I
```

El historial debe registrar dónde estaba la limitación real.

## Especificaciones de cámara

No inventar valores genéricos “ESP32-CAM”. Antes de implementar se debe fijar el módulo/cámara objetivo real y obtener de datasheet/medición, como mínimo:

```text
resolución utilizada
aspect ratio
FOV horizontal/vertical o focal equivalente
fx, fy, cx, cy derivados/calibrados
frame rate objetivo
modelo de distorsión si se simula
exposición/ruido/blur si Gazebo permite modelarlo de forma razonable
```

Si el hardware objetivo concreto usa más de un modo de resolución, elegir el modo que se pretenda usar en el futuro y documentarlo.

## Perfil de simulación

Crear un perfil claro y seleccionable, por ejemplo conceptualmente:

```text
camera_profile = baseline
camera_profile = esp32cam_target
```

No duplicar parámetros semánticos sin guardas. Respetar ownership de Fase 2: la cámara física/calibración pertenece al Dron; la forma de simularla pertenece al deployment de Simulación. Si se requieren réplicas parciales, declararlas según ADR 0009.

## Qué se repite “subfase a subfase”

### Regresión 4A/4B

La geometría de tags/objetos no cambia. Verificar que con la nueva FOV/resolución los objetos siguen apareciendo donde corresponde y que la prueba no depende de haber movido artificialmente fiduciales.

### Regresión 4C

Confirmar que cambiar resolución/FPS no rompe:

- rectificación/resize;
- imagen exacta de KF;
- evento one-shot;
- timestamps/frame IDs.

### Regresión 4D

Es el punto más sensible:

- tasa de detección;
- tamaño aparente de tag;
- reprojection error;
- ambigüedad IPPE;
- tiempo por KF;
- backlog/drop de cola;
- intrínsecos correctos para la imagen efectiva.

### Regresión 4E

Confirmar que el cambio de cámara no altera identidad ni contrato; puede cambiar tag_count/quality, no la semántica del mensaje.

### Regresión 4F

Confirmar sincronización bajo posibles latencias diferentes de worker.

### Regresión 4G

Reevaluar distribución de `quality_score`, coherencia multicara y la zona segura. El perfil inicial 1–5 m se mantiene, pero si la cámara objetivo demuestra que a cierta distancia la pose deja de ser fiable, cualquier ajuste debe justificarse con datos y quedar en 4D/4G, no como excepción escondida en 4I.

### Regresión 4H

Repetir prueba integral multi-dron sin GT funcional.

## Zona configurada inicialmente a 1–5 m con ESP32-CAM

No se amplía el rango solo porque se detecte un tag a más de 5 m. Detectar ID y estimar una pose suficientemente fiable para un anchor son problemas distintos.

Tampoco se cambia `size_m` para corregir escala. `size_m` sigue siendo el lado físico real/configurado del tag.

Si la cámara no permite pose suficientemente estable en parte de 1–5 m, se documenta y se decide de forma explícita si:

- ajustar detector;
- ajustar tamaño físico futuro de tags;
- reducir la zona válida;
- cambiar modo de cámara;

pero siempre mediante la subfase propietaria y repitiendo 4H.

## Matriz de pruebas recomendada

Para al menos un tag y después escenario completo:

```text
distancias aproximadas: cerca / media / límite superior
vistas: frontal / oblicua
movimiento: estacionario / movimiento normal
casos: 1 tag / multicara / múltiples objetos
```

No es necesario fijar todas las distancias exactas antes de conocer FOV/resolución reales.

## Métricas comparativas baseline vs ESP32

Registrar:

```text
KF totales
KF con detección
ratio detección
reprojection_error p50/p95
quality_score p50/p95
detect_ms p50/p95
worker queue max/drop
batches y bytes
anchors correctos
revisitas/tasks
tracking stability
```

El objetivo es saber **qué empeora y por qué**, no solo obtener un PASS/FAIL.

## Fallos y retorno a subfases

Ejemplos:

```text
no detecta tag pequeño       -> 4D / quizá 4A tamaño futuro
K incorrecta tras resize     -> 4C/4D
worker se acumula            -> 4D
poses multicara divergen     -> 4D/4G
muchos candidatos fuera rango-> 4G / diseño de misión futuro
anchor incorrecto            -> 4G/4H
```

Cada corrección exige volver a ejecutar los tests afectados de cámara baseline para no romper lo que ya funcionaba.

## No incluido

- cámara física real;
- impresión de AprilTags;
- ESP32 transmitiendo vídeo por Wi‑Fi/Bluetooth;
- dron físico;
- diseño de tareas Fase 6;
- pose global del cuerpo Fase 5.

Estos puntos pueden abordarse más adelante sin falsificar que Fase 4 ya realizó validación física.

## Criterio de éxito

1. perfil ESP32-CAM objetivo reproducible y documentado;
2. intrínsecos/configuración coherentes con la imagen entregada al detector;
3. cadena 4C–4H sigue funcionando;
4. prueba integral multi-dron sin GT funcional vuelve a pasar, o cualquier limitación se corrige en su subfase y después pasa;
5. rendimiento/fiabilidad quedan medidos;
6. no se maquilla el resultado cambiando `size_m`, GT o thresholds sin evidencia;
7. documentación final indica claramente que la validación es **simulada con especificaciones ESP32-CAM**, no hardware físico.

## Relación con el cierre de Fase 4

La Fase 4 quedó cerrada por decisión del usuario tras conseguir 4A-4H y validar
la sustitución funcional del fiducial GT por visión en simulación. 4I se
conserva para otro momento como regresión adicional con una cámara objetivo
simulada; no es una deuda ni condiciona el cierre ya aceptado. Una futura
validación física seguirá siendo un trabajo separado.
