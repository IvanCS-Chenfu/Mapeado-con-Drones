# Subfase 5C — Backend de pose global autoritativa por KeyFrame

## Estado

```text
sin hacer; no preparar backend fuerte hasta verificar el cierre vigente de 3Q
```

## Objetivo

Crear o reutilizar una API del Servidor que resuelva:

```text
(drone_id, map_epoch, local_kf_id)
    -> W_T_KF
    -> pose_revision
    -> validity/status
```

El Dron usará el reference KF exacto y `Tcr`; 5C no diseña `C_KF`, nearest-KF
ni selección por cercanía.

## Contexto y ámbitos

Leer 5A/5B, el cierre real de 3Q, Fase 4 y docs de `RawMapDatabase`,
`GlobalPoseStore`, `SparseGlobalBackend`, `OptimizationManager` y
`GlobalMapServer`.

Ámbitos permitidos:

```text
servidor/orbslam3_multi/include/**
servidor/orbslam3_multi/src/**
servidor/orbslam3_server/include/**
servidor/orbslam3_server/src/**
tests y documentación
```

No tocar Dron, ORB core, GT ni sobrescribir raw.

## Contrato requerido

- identidad completa `(drone_id, map_epoch, keyframe_id)`;
- `W_T_KF` obtenida desde `GlobalPoseStore` o su API pública;
- revisión reutilizando la semántica autoritativa vigente de Fase 3;
- estados distinguibles `AVAILABLE`, `PENDING/NOT_AVAILABLE_YET` e
  `UNKNOWN/INVALID_EPOCH`;
- punto inequívoco para detectar cambios de pose/revisión que 5D deba publicar.

No publicar arrays completos a todos los drones ni crear otra autoridad global.

## Pruebas acordables

- KF válido;
- epoch incorrecto;
- KF inexistente/pending;
- pose y revisión antes/después de una optimización aceptada;
- inmutabilidad de raw.

Patrones iniciales:

```text
F5C|KF_GLOBAL_QUERY|POSE_REVISION|PENDING|INVALID_EPOCH|RAW|ERROR|FATAL
```

## Criterio de éxito

Identidad completa, pose autoritativa, revisión correcta, cambios detectables,
raw intacto y builds/tests acordados correctos.
