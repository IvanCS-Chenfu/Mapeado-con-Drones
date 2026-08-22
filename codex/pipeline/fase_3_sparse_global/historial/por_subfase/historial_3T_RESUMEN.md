# Historial 3T - resumen

## Estado

```text
CONSEGUIDA
```

3T retiro el legacy vigente y los contratos absorbidos, documento las
responsabilidades activas y centralizo la configuracion operacional en seis
YAML tematicos por despliegue. No cambio algoritmos de mapa ni corrigio 3Q.

## Evidencia

- checkpoint recuperable: `1b96a7a`;
- sin directorios `legacy/`/`legacy2/` ni referencias activas rotas;
- una sola ruta runtime de scheduling, autoridad y publicacion;
- ADR 0009 y copias YAML servidor/simulacion protegidas por tests;
- manifests `0.1.0`, `GPL-3.0-only` e Ivan Calvo Santos;
- build 3/3;
- CTest: 9/9 dominio, 10/10 servidor y 8/8 simulacion;
- contratos YAML/web 14/14 y launches instalados resolviendo;
- Gazebo 195 `success=true`, colas 0/0, `hard_failed=0`, 11 commits loop,
  23.978 puntos con score/rgb y recursos estables.

El usuario confirma posteriormente que RViz2 se vio perfecto en 195.

## Resultado agregado

3T queda `CONSEGUIDA`. La renumeracion final fija 3R scoring, 3S debug y 3T
limpieza. La prueba 196 valida el perfil silencioso y el usuario confirma el
resultado visual correcto de 195. Fase 3 queda concluida; la deformacion 3Q de
194 y su posible mejora se conservan como referencia futura, no como bloqueo.
El resultado operativo y el punto de entrada para Fase 2 estan en
`RESULTADO_FINAL_FASE_3.md`.
