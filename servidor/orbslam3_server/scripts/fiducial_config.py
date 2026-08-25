from pathlib import Path

import yaml


def load_fiducial_config(path):
    with Path(path).open('r', encoding='utf-8') as stream:
        root = yaml.safe_load(stream) or {}

    schema_version = int(root.get('schema_version', 0))
    family = str(root.get('family', '')).strip()
    detector = root.get('detector') or {}
    corner_refinement = str(detector.get('corner_refinement', '')).strip()
    pose_solver = str(detector.get('pose_solver', '')).strip()
    max_error = float(detector.get('max_reprojection_error_px', 0.0))

    if schema_version <= 0:
        raise ValueError('schema_version debe ser positivo')
    if family != 'APRILTAG_36H11':
        raise ValueError(f'family no soportada: {family!r}')
    if corner_refinement != 'SUBPIX':
        raise ValueError(
            f'corner_refinement no soportado: {corner_refinement!r}')
    if pose_solver != 'IPPE_SQUARE':
        raise ValueError(f'pose_solver no soportado: {pose_solver!r}')
    if max_error <= 0.0:
        raise ValueError('max_reprojection_error_px debe ser positivo')

    tags = {}
    for obj in root.get('objects') or []:
        for face in (obj.get('faces') or {}).values():
            if not face or not bool(face.get('enabled', False)):
                continue
            tag_id = int(face.get('tag_id', -1))
            size_m = float(face.get('size_m', 0.0))
            if tag_id < 0 or size_m <= 0.0:
                raise ValueError(
                    f'tag invalido: tag_id={tag_id} size_m={size_m}')
            if tag_id in tags:
                raise ValueError(f'tag_id duplicado: {tag_id}')
            tags[tag_id] = size_m

    return {
        'schema_version': schema_version,
        'family': family,
        'corner_refinement': corner_refinement,
        'pose_solver': pose_solver,
        'max_reprojection_error_px': max_error,
        'tags': sorted(tags.items()),
    }
