# Subfase 5I - Estabilizacion y validacion del estado ORB en movimiento

## Estado

`CONSEGUIDA` dentro del dominio visual validado por las pruebas 352-355.

## Objetivo

Convertir la pose ORB por frame en un estado causal, coherente y suficientemente
actual para un controlador a 50 Hz, sin IMU y sin usar GT en el camino
productivo ORB.

## Historia tecnica consolidada

1. La publicacion ORB a unos 20 Hz producia ZOH y derivadas bruscas frente al
   control a 50 Hz. La estimacion/prediccion se traslado a `orbslam3_ros2`.
2. Se corrigio la composicion de la extrinseca `B_T_C`; la pose visualizada ya
   coincidia, pero el control revelo defectos temporales de R/omega y p/v.
3. Los filtros aplicados a GT empeoraron el baseline y se retiraron: GT normal
   permanece exacto y solo cumple la funcion temporal de fallback.
4. La rama angular se diagnostico mediante anclajes SMALL/MODERATE y cruces GT.
   Se implemento omega causal, extrapolacion angular y predictor dinamico con
   torque e inercia compuesta real del dron.
5. La rama translacional aislo `A_HAT_AMPLIFICATION`. Se corrigio la gravedad
   como `g_O = O_R_W * g_W`, congelada por epoch, y se adopto
   `MIDPOINT_DYNAMIC` para reconstruir velocidad en `t_k` y propagar hasta now.
6. Los buffers de thrust/torque conservan una muestra predecesora ZOH, rechazan
   futuro no causal y sobreviven a resets visuales.
7. El handoff se sincronizo con fronteras de goal para comenzar con error de
   pose y velocidad nulo sin alterar el controlador nominal.
8. Hover, X, Y, Z y yaw se validaron progresivamente. La ruta de dos fachadas
   descubrio `STALE_RAW_HISTORY` y pulsos de validez que provocaban fallback.
9. La bateria 349 demostro defectos independientes suficientes en p/v y
   R/omega ORB; estos cruces fueron diagnosticos y no forman parte del producto.
10. La prueba 351 mostro que la evidencia visual cae antes de la perdida de
    tracking. La ruta favorable 352 fue estable en shadow y 353-355 completo
    tres repeticiones consecutivas con autoridad ORB.

## Implementacion final

- predictor productivo `dynamic` en `StereoSlamNode`;
- publicacion de `NavigationState` a 50 Hz;
- reference KF real y `Tcr` del mismo frame;
- velocidad angular causal y dinamica rigida con J real;
- velocidad lineal `MIDPOINT_DYNAMIC` con gravedad en O;
- buffers fisicos causales y poda con predecesora ZOH;
- resets visuales que preservan actuacion fisica;
- instrumentacion opcional de evidencia visual del mismo frame.

## Dominio validado

ORB funciona para hover y movimientos lentos con textura, profundidad estereo,
inliers y cobertura suficientes. No se promete control ORB robusto cuando la
camara observa una fachada pobre, distante o sin geometria util.

## Pruebas clave

```text
252        extrinseca B_T_C corregida
269-317    aislamiento temporal y predictores angular/translacional
324-329    gravedad O y MIDPOINT_DYNAMIC
330-338    hover y movimientos simples
344-349    reference/raw, fallback y cruces p/v frente a R/omega
350R-352   baseline y causalidad de evidencia visual
353-355    ruta favorable ORB, 3/3 conseguida
```

## Criterio de cierre

Se exige autoridad ORB real, escenario completo, tracking continuo y ausencia
de fallback posterior al handoff en una ruta con evidencia visual suficiente.
No se exige resolver en Fase 5 la planificacion anticipada por calidad visual.
