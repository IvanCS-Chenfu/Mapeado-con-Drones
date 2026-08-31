# Fase 5H — Prueba ORB real con activación limpia únicamente en hover
## Objetivo: separar el fallo de 320R de una activación prematura de ORB durante aproximación/contacto con suelo

## 0. Fuente de verdad

Trabajar sobre el estado ACTUAL del repositorio:

```text
https://github.com/IvanCS-Chenfu/Mapeado-con-Drones
```

Tomar como referencia directa:

```text
historial_5H_RESUMEN.md
prueba_318R2.log
prueba_319R.log
prueba_320.log
prueba_320R.log
```

Estado previo aceptado:

```text
poda ZOH                    VALIDADA
cobertura torque/thrust     VALIDADA
paridad productiva          VALIDADA
318R2                       CONSEGUIDA
319R                        CONSEGUIDA
```

El primer `320` es inválido porque acabó usando:

```text
legacy
```

por precedencia incorrecta del launch.

`320R` sí ejecuta:

```text
orb_navigation_prediction_mode=dynamic
```

y constituye la primera prueba ORB productiva real válida.

Sin embargo, no valida todavía de forma limpia el hover ORB porque ORB gobierna antes de la frontera prevista.

---

# 1. Diagnóstico actual de 320R

La secuencia prevista conceptualmente era:

```text
espera_tracking
        ↓
llegada_gt_fiducial_2
        ↓
espera_anchor
        ↓
activa_orb_en_hover
        ↓
hover_orb
```

Pero en 320R:

> ORB pasa a ser fuente autoritativa antes de terminar la aproximación.

Por tanto, el goal:

```text
llegada_gt_fiducial_2
```

termina siendo ejecutado ya con estado ORB.

Además, aparece una velocidad inicial no estacionaria cercana a:

```text
v0 ≈ (0.0016, -0.2879, -0.0014) m/s
```

al aceptar el primer goal.

Esto es especialmente importante porque el predictor translacional productivo representa dinámica de vuelo libre:

```text
thrust
+
masa
+
gravedad
+
R_dynamic
```

y NO modela el contacto con el suelo.

Si ORB se vuelve autoritativo mientras el dron todavía está apoyado en el suelo o en una fase previa al vuelo, el predictor puede integrar gravedad durante el retraso visual aunque Gazebo esté sosteniendo físicamente el dron mediante contacto.

Eso puede crear un estado lineal válido matemáticamente para vuelo libre pero físicamente incorrecto en contacto.

---

# 2. Evidencia que NO debemos perder

320R no muestra simplemente:

```text
tracking loss
-> caída
```

La divergencia del control empieza antes de la pérdida visual.

Por tanto:

```text
tracking loss
```

no debe tratarse como causa inicial sin más análisis.

También queda demostrado que:

```text
318R2 / 319R
```

funcionan con la misma infraestructura dinámica cuando la entrada de laboratorio está controlada.

Por ello NO tocar todavía:

```text
v_hat(t_k)
omega_hat(t_k)

BodyThrustDynamicPredictor
predictor angular

J
masa
gravedad

buffers
poda ZOH

Kp
Kv
Kr
Kw

SMALL
MODERATE
raw gates
reference KF
W
```

---

# 3. Objetivo de esta nueva iteración

Realizar una prueba ORB real donde:

```text
GT gobierne la aproximación inicial
```

mientras:

```text
ORB trabaja en shadow mode
```

y sólo se permita:

```text
GT -> ORB
```

cuando el dron:

```text
1. ya esté en el aire;
2. haya terminado la aproximación;
3. esté anclado;
4. esté aproximadamente estacionario;
5. ORB lleve un pequeño intervalo produciendo estado consumible.
```

Después ejecutar exclusivamente:

```text
hover ORB
```

La finalidad es responder:

> ¿El estimador productivo ORB dinámico funciona cuando entra en control desde una condición de vuelo estacionaria y no desde contacto/aproximación?

---

# 4. Regla principal — una sola variable nueva

La única modificación funcional autorizada debe ser:

```text
frontera temporal / lógica de activación de ORB en el escenario de prueba
```

No modificar la matemática del estimador.

No ajustar thresholds para mejorar el resultado.

---

# 5. Fuente durante la aproximación

Durante:

```text
espera_tracking
llegada_gt_fiducial_2
espera_anchor
```

forzar que la fuente autoritativa usada por:

```text
gen_tray
control_calcular_fuerzas
```

sea:

```text
GT_FALLBACK / GT diagnóstico
```

según la infraestructura vigente.

ORB debe seguir completamente activo en paralelo:

```text
TrackStereo
KFs
fiduciales
anchor
NavigationState ORB
v_hat
omega_hat
predictor dinámico
```

pero:

```text
NO gobierna el dron todavía.
```

---

# 6. Shadow mode ORB

Mientras GT gobierna, registrar ORB productivo real como fuente sombra.

No crear otro estimador.

Debe observarse exactamente el mismo:

```text
NavigationState ORB dinámico
```

que posteriormente se entregará al mux/control.

Registrar contra GT externo:

```text
p_orb_dynamic(now)
v_orb_dynamic(now)

R_orb_dynamic(now)
omega_orb_dynamic(now)

p_GT(now)
v_GT(now)

R_GT(now)
omega_GT(now)
```

La comparación es exclusivamente diagnóstica.

---

# 7. No usar el contacto con el suelo como validación dinámica

Separar explícitamente:

```text
PRE_FLIGHT / CONTACT
```

de:

```text
AIRBORNE / CONTROL_ACTIVE
```

No evaluar la calidad del predictor de vuelo libre con métricas tomadas cuando el dron está apoyado físicamente en el suelo.

No añadir un modelo de contacto.

La solución productiva final sigue siendo un predictor de vuelo.

---

# 8. Condición de activación de ORB

No permitir el cambio:

```text
GT -> ORB
```

simplemente porque:

```text
tracking == OK
anchor == válido
```

durante la aproximación.

Para ESTA PRUEBA, abrir la frontera únicamente tras:

```text
goal llegada completado
+
anchor disponible
+
hover/posición de activación alcanzada
```

y un pequeño periodo de asentamiento.

---

# 9. Ventana de asentamiento antes del handoff

Antes de autorizar ORB, esperar por ejemplo:

```text
1-2 s
```

o una duración breve justificada por los logs, durante la cual:

```text
GT sigue gobernando
ORB sigue en sombra
```

y comprobar que el dron está aproximadamente estacionario.

No fijar un valor excesivamente estricto.

Registrar al menos:

```text
|v_GT|
|omega_GT|

|v_ORB|
|omega_ORB|

p error
R error
```

La ventana sirve para observar el estado ORB justo antes de entrar en control.

---

# 10. Criterio de estacionariedad

Codex puede reutilizar infraestructura existente o introducir exclusivamente telemetría diagnóstica.

Registrar:

```text
gt_linear_speed
gt_angular_speed

orb_linear_speed
orb_angular_speed
```

No usar GT como parte permanente de la lógica productiva.

En este experimento sí puede usarse GT para confirmar:

```text
el dron está físicamente estacionario antes del handoff
```

porque estamos aislando causalidad.

---

# 11. Handoff GT -> ORB

En el instante acordado:

```text
activa_orb_en_hover
```

permitir:

```text
GT -> ORB
```

una sola vez.

Registrar obligatoriamente:

```text
handoff_stamp

p_GT_before
p_ORB_before

v_GT_before
v_ORB_before

R_GT_before
R_ORB_before

omega_GT_before
omega_ORB_before

position_jump
velocity_jump
rotation_jump
omega_jump
```

Y en el primer tick de control ORB:

```text
ep
ev
er
ew

F_des
tau
```

---

# 12. Muy importante — no regenerar la trayectoria incorrectamente

El hover ORB debe empezar desde el estado aceptado en el handoff.

Mantener el contrato vigente de:

```text
primer feedback t=0
x0/v0/yaw0/yaw_rate0 coherentes
```

No reutilizar una trayectoria previa creada durante la aproximación GT si su frame/estado inicial ya no corresponde.

Si el escenario ya crea un goal nuevo de hover después de activar ORB, conservar ese comportamiento.

---

# 13. Prueba sugerida: 320R2

Repetir el escenario de 320R modificando exclusivamente la frontera de fuente.

Nombre sugerido:

```text
320R2
```

Secuencia explícita:

```text
A. startup
B. espera tracking ORB
C. aproximación al fiducial gobernada por GT
D. espera anchor
E. llegada terminada
F. asentamiento corto con GT
G. ORB en shadow medido
H. activar ORB
I. nuevo goal de hover
J. mantener hover
```

---

# 14. Confirmaciones obligatorias en log

Debe quedar inequívocamente demostrado:

```text
durante la aproximación:
source = GT

antes del handoff:
ORB productivo está activo en sombra

handoff:
GT -> ORB ocurre sólo en la frontera de hover

durante hover:
source = ORB
```

Cualquier entrada ORB autoritativa anterior invalida la prueba.

---

# 15. Criterio mínimo de validez de 320R2

La prueba sólo es válida si:

```text
orb_navigation_prediction_mode = dynamic

aproximación gobernada íntegramente por GT

anchor válido antes del handoff

handoff ocurre en hover

no hay F5H-DYNAMIC-MISSING

torque coverage = FULL
thrust coverage = FULL
```

Si falla alguno:

```text
PRUEBA INVÁLIDA
```

y no interpretar el comportamiento de control.

---

# 16. Métricas antes del handoff

Durante los últimos segundos de shadow:

```text
RMSE p ORB vs GT
RMSE v ORB vs GT

RMSE R ORB vs GT
RMSE omega ORB vs GT

visual age

linear_estimator_mode
angular_estimator_mode

DEGRADED_DT
raw classes

reference KF changes
```

No usar estos RMSE como criterio absoluto de aprobación todavía.

Sirven para contextualizar el handoff.

---

# 17. Métricas después del handoff

Desde:

```text
handoff_stamp
```

registrar:

```text
tiempo gobernado por ORB

ep
ev
er
ew

p/v/R/omega ORB
p/v/R/omega GT

F_des
tau

energía angular

tracking
reference KF
raw class

fallback
fallback cause
```

Separar ventanas:

```text
0-0.1 s
0-0.5 s
0-1 s
1-5 s
resto del hover
```

---

# 18. Interpretación de resultados

## Caso A — 320R2 completa hover ORB

Conclusión:

> El fallo de 320R estaba provocado principalmente por activar ORB demasiado pronto durante aproximación/contacto o desde una condición dinámica inadecuada.

En ese caso repetir exactamente:

```text
321
```

sin cambios.

---

## Caso B — handoff limpio pero ORB empieza a divergir

Si:

```text
aproximación GT correcta
estado estacionario
handoff pequeño
tracking sigue OK
```

pero después:

```text
errores crecen
dron oscila/se desplaza
```

entonces la prueba sí demuestra un fallo real de la ruta ORB productiva.

El siguiente foco deberá ser:

```text
pose ORB real
v_hat(t_k)
omega_hat(t_k)

SMALL/MODERATE
reference KF
dynamic prediction
```

sin volver a culpar contacto/pre-flight.

---

## Caso C — ORB pierde tracking antes de la inestabilidad

Si el dron permanece estable y:

```text
tracking 2 -> non-OK
```

ocurre primero por causa visual:

```text
fallback limpio
```

puede considerarse comportamiento esperado de Fase 5.

Documentar la causa visual.

---

## Caso D — handoff ya tiene salto grande

Si al activar ORB:

```text
position_jump
velocity_jump
rotation_jump
omega_jump
```

son grandes antes de cualquier respuesta del controlador:

> El problema está en el estado ORB acumulado/alineado antes de tomar autoridad.

STOP.

No ejecutar repetición.

---

# 19. Repetición 321

Sólo si 320R2 completa correctamente.

Ejecutar:

```text
321
```

con exactamente:

```text
mismo YAML
mismos gains
misma frontera
mismos estimadores
mismos parámetros
```

Criterio:

```text
320R2 = CONSEGUIDA
321   = CONSEGUIDA
```

para declarar:

```text
HOVER ORB PRODUCTIVO REPRODUCIBLE
```

---

# 20. Qué hacer si 320R2 y 321 funcionan

No pasar directamente al recorrido completo.

Siguiente batería a acordar:

```text
hover
↓
X corto
↓
Y corto
↓
Z corto
↓
yaw lento
↓
curva sencilla
```

con ORB productivo.

Después:

```text
trayectoria representativa
```

---

# 21. Qué NO hacer

No modificar:

```text
predictor angular
predictor translacional

v_hat
omega_hat

J
masa

buffers
ZOH

control gains

SMALL
MODERATE
KF logic
W
```

No implementar otro filtro.

No añadir modelo de contacto.

No usar GT como fuente normal después del handoff salvo:

```text
fallback temporal por pérdida real de tracking
```

---

# 22. Tests automáticos

Añadir sólo los tests necesarios para impedir una regresión de la frontera.

Como mínimo:

```text
1. llegada_gt_fiducial_2 no puede aceptar source=ORB en este escenario.

2. activa_orb_en_hover es la única frontera autorizada GT->ORB.

3. ORB sigue produciendo NavigationState en shadow.

4. el nuevo goal de hover congela su estado inicial usando la fuente ORB ya activada.

5. launch respeta orb_navigation_prediction_mode=dynamic.

6. ninguna configuración YAML vuelve a sobrescribir dynamic con legacy.
```

No convertir esta lógica diagnóstica en una restricción global de producción si sólo pertenece al escenario.

---

# 23. Telemetría recomendada nueva

Añadir marcadores claros:

```text
[F5H-ORB-SHADOW]
source_authority=GT
orb_state_valid=
p_error=
v_error=
r_error=
omega_error=
```

Antes del cambio:

```text
[F5H-ORB-ACTIVATION-READY]
tracking=
anchor=
airborne=
settled=
orb_local_valid=
orb_velocity_valid=
```

En el cambio:

```text
[F5H-ORB-ACTIVATED]
stamp=
goal_boundary=
p_jump=
v_jump=
r_jump=
omega_jump=
```

---

# 24. Qué debe devolver Codex

Al terminar:

```text
Resultado:
CONSEGUIDO / PARCIAL / NO CONSEGUIDO
```

Incluir:

```text
- archivos modificados;

- explicación de por qué 320R activaba ORB antes de tiempo;

- frontera nueva exacta;

- confirmación de que no cambió la matemática del estimador;

- builds;
- GTests;
- analyzer;
- git diff --check;

- prueba 320R2;

- fuente durante aproximación;
- duración shadow;
- anchor antes del handoff;

- p/v/R/omega ORB vs GT justo antes de handoff;

- saltos del handoff;

- tiempo ORB gobernando;

- tracking;
- reference KF;
- fallback y causa;

- resultado hover;

- prueba 321 si corresponde;

- conclusión:
    ACTIVACIÓN PREMATURA CONFIRMADA / DESCARTADA
    HOVER ORB PRODUCTIVO VALIDADO / NO VALIDADO
    HOVER ORB REPRODUCIBLE / NO REPRODUCIBLE

- siguiente paso recomendado.
```

Actualizar:

```text
historial_5H_RESUMEN.md
07_ULTIMA_SESION.md
```

sin borrar historial previo.

---

# 25. Resumen ejecutivo

La infraestructura dinámica ya está validada:

```text
318R2 / 319R
-> sin missing
-> sin fallback
-> paridad correcta
```

320R es la primera ORB real productiva, pero no es una prueba limpia de hover porque ORB empieza a gobernar durante la aproximación.

La siguiente prueba debe hacer:

```text
aproximación:
GT gobierna
ORB observa en shadow

dron llega
anchor válido
dron se estabiliza

        ↓

activar ORB

        ↓

nuevo goal de hover ORB
```

Primero:

```text
320R2
```

Si funciona:

```text
321
```

> No volver a tocar los predictores hasta comprobar ORB real entrando desde una condición de vuelo estacionaria y con una frontera de autoridad limpia.
