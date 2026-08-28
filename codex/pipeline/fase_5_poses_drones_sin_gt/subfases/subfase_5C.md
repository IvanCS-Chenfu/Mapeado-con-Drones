# Subfase 5C — Backend de pose global autoritativa por KeyFrame

## Estado

```text
CONSEGUIDA; API y estados validados en tests y prueba 230
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

La implementación acordada reutiliza `SparseGlobalBackend::GetGlobalPose()` y
`GlobalPoseRecord::pose_revision`. No crea otra base de datos ni otra revisión.

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

Clasificación acordada:

```text
AVAILABLE          -> registro world activo y versionado
PENDING            -> KF/epoch aún no materializado o todavía sin autoridad
UNKNOWN            -> identidad de dron/KF no admisible
INVALID_EPOCH      -> epoch incompatible con el estado conocido del dron
```

Una ausencia que pueda explicarse por carrera wrapper->Servidor se conserva
como `PENDING`; no se convierte prematuramente en error definitivo.

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

5C se valida como checkpoint interno del bloque; no se considera cierre del
bloque sin transporte 5D, estimador 5E y métricas 5F.
