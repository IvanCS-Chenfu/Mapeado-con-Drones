---
name: compilar-paquetes
description: Compila paquetes ROS 2 seleccionados por el agente usando build_selected_packages.sh y gestiona el log de build.
---

Usar cuando haya que compilar después de una modificación.

## Checkpoint obligatorio

Antes de ejecutar el build, `codex/contexto/00_CONTEXTO_COMPACTACION.md` debe
indicar paquetes, motivo y comando o siguiente acción. Inmediatamente después de
que termine, registrar código de salida, paquetes que pasaron/fallaron, ruta del
log y siguiente acción antes de diagnosticar, corregir o ejecutar otra prueba.

No aplazar este checkpoint hasta la documentación final. Si el chat se compacta,
releer físicamente la memoria antes de continuar; el resumen automático no la
sustituye.

Comando base:

```bash
./codex/herramientas/build_selected_packages.sh <paquete_1> <paquete_2> ...
```

Reglas:

1. Los paquetes los decide `planificador_fase` o `implementador_fase`, no la herramienta.
2. El log completo se guarda siempre en:

```text
codex/archivos_auxiliares/colcon_build.log
```

3. Si el comando devuelve `0` y no hace falta inspeccionar contenido, no es
   necesario reducir. Si Codex quiere leer avisos o detalles, debe reducir antes.
4. Si devuelve distinto de `0`, ejecutar:

```bash
./codex/herramientas/reduce_build_log.sh
```

5. Tras reducir, `diagnosticador_build` debe leer:

```text
codex/archivos_auxiliares/colcon_build.reduced.log
```

6. Si falta contexto en el reducido, `diagnosticador_build` debe regenerar o
   ampliar la reducción. El log completo nunca se lee directamente ni se vuelca
   al contexto.

7. `implementador_fase` corrige solo lo que indique `diagnosticador_build`.
8. Repetir hasta compilar o hasta que haya un bloqueo claro.

Cada repetición de build requiere su propio checkpoint de resultado.
