# Subfase 5A — Auditoría técnica y cierre de arquitectura

## Estado

```text
CONSEGUIDA documentalmente el 2026-08-25
implementación funcional de Fase 5: no iniciada
```

## Objetivo

Reconciliar el pipeline y los contratos de Fase 5 con la auditoría contenida en
`Fase_5_preparada_para_Codex.zip`, el código/documentación vigente y las
decisiones aceptadas por el usuario.

5A no implementa pose, transporte, control ni fallback. No modifica código,
launch, YAML funcional, configuración o mensajes, ni requiere build o
simulación.

## Fuentes auditadas

```text
AGENTS.md
codex/contexto/00_CONTEXTO_COMPACTACION.md
codex/contexto/CONTEXTO_MINIMO_ACTUAL.md
codex/contexto/08_POLITICA_TOKENS_DOCUMENTACION.md
codex/pipeline/PIPELINE_MAESTRO.md
pipeline y contratos F5 vigentes
Fase_5_preparada_para_Codex.zip
pipeline/resumen de Fase 4
docs de orbslam3_ros2, orbslam3_msgs y orbslam3_multi
docs de orbslam3_server, dron_individual y simulacion_dron
ADR 0002 sobre uso de GT
```

El ZIP fue preparado sobre `1d585059`; la auditoría se contrastó con el HEAD
vigente `a44b8b8`. Para paths, interfaces y símbolos siempre prevalece el
repositorio real al preparar cada subfase.

## Diagnóstico fijado

- `orbslam/pose_local` expresa pose de cámara en el mapa local y solo se publica
  con tracking válido;
- todavía no expone en una muestra única tracking, `map_epoch`, reference KF y
  `Tcr`;
- `gen_tray` y `control_calcular_fuerzas` consumen actualmente GT pose/vel;
- ORB dispone del reference KF real y conserva `Tcr = C_T_Kref`;
- el último KF creado no equivale al reference KF;
- `RawMapDatabase` y `GlobalPoseStore` ya separan raw y global;
- el `global_pose_corrector` histórico fue retirado y no es código a reactivar;
- `MapCorrection`/`CorrectedKeyFrameArray` son interfaces reservadas cuyo uso
  debe decidirse según el contrato mínimo de 5D.

## Decisiones cerradas

1. El controlador utiliza un frame continuo `O` y consume `O_T_B`.
2. La pose global `W_T_B` se compone aparte y puede saltar por optimización.
3. Se usa reference KF real + `Tcr`, nunca nearest-KF.
4. El estimador por frame vive junto a `orbslam3` en el Dron.
5. El Servidor aporta `W_T_Kref` y revisión, no una pose final por frame.
6. Las consultas de nuevos reference KFs son asíncronas y admiten provisional.
7. Los goals relativos se ejecutan en `O`.
8. Los absolutos requieren `W_T_O`, se convierten al aceptar y se congelan.
9. Un absoluto sin global se rechaza.
10. Una revisión global no cambia el goal activo.
11. La velocidad se deriva de `O_T_B`.
12. Smoothing no es requisito; solo se abre con evidencia de 5F.
13. `RECENTLY_LOST` activa `GT_FALLBACK` temporal y la misión continúa.
14. GT queda excluido de mapa, anchors, optimización y pose global final.
15. Fase 6 elimina el fallback y aporta recovery/reanclaje real.
16. La antigua 5I queda absorbida en 5H.

## Riesgos y puertas

- No asumir `O == L` sin medir continuidad ante Local BA/cambio de KF.
- Tocar `dron/ORB_SLAM3` exige autorización específica si 5B demuestra que la
  API existente no puede exponer reference KF/`Tcr` de forma aditiva.
- Antes de 5C/5D se verifica el cierre vigente de 3Q.
- Antes de 5G se actualiza formalmente el ADR de GT con la excepción temporal,
  parámetro explícito, observabilidad y deuda F6.
- Cada subfase funcional conserva su propia preparación, pruebas y autorización.

## Criterio y verificación

5A queda conseguida si pipeline, resumen y contratos 5A-5H reflejan estas
decisiones; 5I es solo un stub; existe historial documental; no se inventan
builds/simulaciones y pasan las comprobaciones de referencias y
`git diff --check`.

## Siguiente paso

Preparar 5B como primera subfase funcional.
