# Mapeado-MultiDrone-Orbslam

<!-- ACUERDOS_CIERRE_F2_2026_08_24_START -->
## Estado del repositorio y alcance del cierre de Fase 2

> **Vigencia:** acuerdo cerrado el 2026-08-24. Este bloque prevalece sobre cualquier
> frase anterior incompatible del mismo documento. No borra ni reescribe evidencia
> histórica; distingue siempre entre estado actual, deuda conocida y arquitectura objetivo.

El repositorio ya está organizado físicamente en los grupos `dron/`, `servidor/` y
`simulacion/`; no debe describirse como un repositorio limitado a la antigua Fase 1.
La Fase 3 sparse global está conseguida y la Fase 2 se encuentra en cierre técnico.

En este checkpoint se documentan primero las correcciones acordadas y se pospone su
implementación. La prueba oficial 198 fue ejecutada y el usuario confirmó
funcionalmente/visualmente que funcionó correctamente. Tras las correcciones de Fase 2
se repetirá una regresión equivalente, porque la prueba 198 valida el snapshot anterior
a esos cambios.

La arquitectura de configuración, observabilidad y aislamiento se rige por ADR 0009 y
ADR 0010.
<!-- ACUERDOS_CIERRE_F2_2026_08_24_END -->

## Alcance del repositorio en GitHub

Este repositorio de GitHub no contiene todos los paquetes del workspace completo del proyecto. Solo se han subido los paquetes y archivos que deben poder editarse desde Codex Web durante la Fase 1.

La documentación de contexto puede mencionar más paquetes de los que aparecen en `src/`. En particular, `codex/contexto/paquetes/` describe el conjunto más amplio de paquetes del proyecto y sirve como referencia para entender qué existe en el entorno completo, aunque no todos esos paquetes estén presentes en este repositorio remoto.

Durante la Fase 1, los cambios de código deben centrarse en los paquetes incluidos aquí, que son los paquetes previstos para edición desde Codex Web en esta etapa. Los paquetes ausentes deben tratarse como dependencias externas o componentes del workspace completo, no como archivos omitidos por error en este repositorio.
