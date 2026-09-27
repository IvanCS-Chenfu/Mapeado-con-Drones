# Capitulo 7 - resumen de resultados para 7.8.3

## Trazabilidad y alcance

- La evidencia de 7.8.2 usada en este resumen procede **exclusivamente** de
  `7_8_2_deriva_revisita/run_01`.
- No se emplea ningun dato de `run_02` a `run_15` de 7.8.2. Esos intentos quedan
  fuera del analisis y pueden eliminarse sin afectar a este resumen.
- Las cifras se obtienen de los CSV procesados y, para 7.8.2, del log reducido
  asociado a `run_01`. No se han editado CSV manualmente.

## Resultado de control con ORB

| Ensayo | Perfil | Resultado | RMSE seguimiento | Maximo seguimiento | RMSE ORB-GT alineado |
|---|---|---|---:|---:|---:|
| 7.8.1-A | Cubico | Completado | 0.0934 m | 0.1828 m | 0.0485 m |
| 7.8.1-B | Trapezoidal | Completado | 0.0715 m | 0.1629 m | 0.0363 m |

El perfil trapezoidal presento menores errores de seguimiento y de ORB frente a
GT en esta comparacion. Ambos ensayos mantuvieron fuente ORB en el 100 % de sus
muestras, sin fallback ni muestras de tracking no OK.

## Continuidad y recorrido largo (7.8.2, solo run_01)

- Duracion bajo autoridad ORB: `146.890 s`; muestras sincronizadas: `6.689`.
- Cambios de KF: `156`, repartidos entre `140` KF distintos.
- Fallbacks: `0`; muestras de tracking no OK: `0`.
- Error global W maximo: `0.8330 m`.
- Error local O frente a la referencia de trayectoria en todo el recorrido:
  RMSE `0.1522 m`, maximo `0.4701 m`.
- El log reducido confirma los seis movimientos de la ruta, incluida la
  revisita al fiducial 1 y el tramo posterior. La observacion primaria del
  fiducial 1 aparece a `123.540 s` desde la autoridad ORB, con calidad `0.7212`
  y distancia `1.0330 m`.

Las figuras de esta ejecucion estan en
`7_8_2_deriva_revisita/run_01/processed/`. En particular, las figuras 1 a 3
muestran GT frente a ORB en W y el error local O; la figura 4 hace zoom sobre
la revisita al fiducial 1.

No se observan eventos `F3E`, `F3Q` ni `F3K` en el log reducido de `run_01`.
Por ello no puede afirmarse que hubiera una optimizacion o un commit de
servidor, ni cuantificar un antes/despues de correccion global. Los cambios de
`pose_revision` no se usan como sustituto de esa evidencia porque tambien
ocurren durante cambios de KF.

## Proteccion visual (7.7.2, run_10)

- El riesgo se activo en el sector derecho (`risk_mask=2`), frame `1151`, con
  cero inliers reportados en las ventanas.
- La parada local finalizo `5.030 s` despues del primer evento de riesgo.
- La reorientacion local termino `5.060 s` despues de completar la parada.
- Durante esos eventos se mantuvieron tracking y validez local; la trayectoria
  original quedo inactiva y no se reanudo.

## Limitaciones observadas

- La evidencia de 7.8.2 es parcial respecto al objetivo de optimizacion:
  confirma recorrido, revisita y continuidad, pero no una optimizacion/commit
  por falta de sus marcadores de servidor en el log de `run_01`.
- El error W maximo muestra deriva medible en el recorrido largo, pero no se
  debe atribuir una reduccion concreta de esa deriva a una optimizacion sin la
  evidencia F3 correspondiente.
- Los cambios frecuentes de KF requieren separar cuidadosamente los cambios de
  referencia de las revisiones globales al interpretar la telemetria.

## Conclusion para la memoria

7.8.1 valida el control ORB en trayectorias cortas para ambos perfiles, con
mejor error medido en el perfil trapezoidal. 7.7.2 valida la parada preventiva
y reorientacion por degradacion visual. En 7.8.2/run_01 se valido el recorrido
ORB largo, la revisita al fiducial 1 y la continuidad durante numerosos cambios
de KF; queda como resultado **parcial** la demostracion de correccion global,
porque el commit no se puede confirmar con la evidencia conservada.
