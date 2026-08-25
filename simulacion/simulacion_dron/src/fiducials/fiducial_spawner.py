#!/usr/bin/env python3

import copy
import math
from pathlib import Path
import shutil
import sys
import time
from xml.sax.saxutils import escape

import cv2
import numpy as np
import yaml


FACES = ('pos_x', 'neg_x', 'pos_y', 'neg_y', 'pos_z', 'neg_z')
FAMILY_IDS = {'APRILTAG_36H11': cv2.aruco.DICT_APRILTAG_36H11}


class ContractError(ValueError):
    pass


def _number(value, path, *, positive=False, nonnegative=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContractError(f'{path} debe ser numerico')
    result = float(value)
    if not math.isfinite(result):
        raise ContractError(f'{path} debe ser finito')
    if positive and result <= 0.0:
        raise ContractError(f'{path} debe ser > 0')
    if nonnegative and result < 0.0:
        raise ContractError(f'{path} debe ser >= 0')
    return result


def _vector(value, path, length):
    if not isinstance(value, list) or len(value) != length:
        raise ContractError(f'{path} debe contener {length} valores')
    return [_number(item, f'{path}[{index}]') for index, item in enumerate(value)]


def load_yaml(path):
    with Path(path).open('r', encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ContractError(f'{path} no contiene un mapping YAML')
    return document


def validate_contract(document, *, require_baseline=False):
    if document.get('schema_version') != 1:
        raise ContractError('schema_version debe ser 1')
    family = document.get('family')
    if family not in FAMILY_IDS:
        raise ContractError(f'family no soportada: {family!r}')

    eligibility = document.get('anchor_eligibility')
    if not isinstance(eligibility, dict):
        raise ContractError('anchor_eligibility debe ser un mapping')
    min_distance = _number(
        eligibility.get('min_distance_m'), 'anchor_eligibility.min_distance_m',
        nonnegative=True)
    max_distance = _number(
        eligibility.get('max_distance_m'), 'anchor_eligibility.max_distance_m',
        positive=True)
    if min_distance >= max_distance:
        raise ContractError('min_distance_m debe ser menor que max_distance_m')

    objects = document.get('objects')
    if not isinstance(objects, list):
        raise ContractError('objects debe ser una lista')
    if require_baseline and len(objects) != 3:
        raise ContractError('el deployment baseline debe contener tres objetos')

    dictionary = cv2.aruco.getPredefinedDictionary(FAMILY_IDS[family])
    max_marker_count = len(dictionary.bytesList)
    object_ids = set()
    tag_ids = set()
    normalized = copy.deepcopy(document)

    for object_index, obj in enumerate(objects):
        prefix = f'objects[{object_index}]'
        if not isinstance(obj, dict):
            raise ContractError(f'{prefix} debe ser un mapping')
        object_id = obj.get('object_id')
        if isinstance(object_id, bool) or not isinstance(object_id, int) or object_id <= 0:
            raise ContractError(f'{prefix}.object_id debe ser un entero > 0')
        if object_id in object_ids:
            raise ContractError(f'object_id duplicado: {object_id}')
        object_ids.add(object_id)
        if obj.get('shape') != 'box':
            raise ContractError(f'{prefix}.shape debe ser box')

        size = obj.get('size_m')
        if not isinstance(size, dict) or set(size) != {'x', 'y', 'z'}:
            raise ContractError(f'{prefix}.size_m debe contener x, y, z')
        for axis in ('x', 'y', 'z'):
            _number(size[axis], f'{prefix}.size_m.{axis}', positive=True)

        pose = obj.get('world_T_object')
        if not isinstance(pose, dict):
            raise ContractError(f'{prefix}.world_T_object debe ser un mapping')
        _vector(pose.get('translation_m'), f'{prefix}.world_T_object.translation_m', 3)
        _vector(pose.get('rotation_rpy_deg'), f'{prefix}.world_T_object.rotation_rpy_deg', 3)

        faces = obj.get('faces')
        if not isinstance(faces, dict) or set(faces) != set(FACES):
            raise ContractError(f'{prefix}.faces debe declarar exactamente las seis caras')
        for face_name in FACES:
            face = faces[face_name]
            face_path = f'{prefix}.faces.{face_name}'
            if not isinstance(face, dict) or not isinstance(face.get('enabled'), bool):
                raise ContractError(f'{face_path}.enabled debe ser booleano')
            if not face['enabled']:
                continue
            tag_id = face.get('tag_id')
            if isinstance(tag_id, bool) or not isinstance(tag_id, int) or tag_id < 0:
                raise ContractError(f'{face_path}.tag_id debe ser un entero >= 0')
            if tag_id >= max_marker_count:
                raise ContractError(f'{face_path}.tag_id no existe en {family}: {tag_id}')
            if tag_id in tag_ids:
                raise ContractError(f'tag_id duplicado: {tag_id}')
            tag_ids.add(tag_id)
            _number(face.get('size_m'), f'{face_path}.size_m', positive=True)

    if require_baseline and len(tag_ids) != 15:
        raise ContractError('el deployment baseline debe contener 15 tags habilitados')
    return normalized


def validate_rendering(document):
    if document.get('schema_version') != 1:
        raise ContractError('rendering.schema_version debe ser 1')
    texture_size = document.get('texture_size_px')
    if isinstance(texture_size, bool) or not isinstance(texture_size, int) or texture_size < 64:
        raise ContractError('texture_size_px debe ser un entero >= 64')
    _number(document.get('surface_offset_m'), 'surface_offset_m', nonnegative=True)
    _number(
        document.get('spawn_service_timeout_sec'), 'spawn_service_timeout_sec',
        positive=True)
    _number(
        document.get('spawn_response_timeout_sec'), 'spawn_response_timeout_sec',
        positive=True)
    if (
        not isinstance(document.get('runtime_asset_dir'), str) or
        not document['runtime_asset_dir']
    ):
        raise ContractError('runtime_asset_dir debe ser un string no vacio')
    if (
        not isinstance(document.get('ready_topic'), str) or
        not document['ready_topic'].startswith('/')
    ):
        raise ContractError('ready_topic debe ser un topic absoluto')
    return copy.deepcopy(document)


def enabled_tags(contract):
    for obj in contract['objects']:
        for face_name in FACES:
            face = obj['faces'][face_name]
            if face['enabled']:
                yield obj, face_name, face


def generate_assets(contract, rendering, logger=None):
    root = Path(rendering['runtime_asset_dir']).resolve()
    if root == Path('/') or len(root.parts) < 3:
        raise ContractError('runtime_asset_dir es demasiado amplio')
    if root.exists():
        shutil.rmtree(root)
    textures = root / 'materials' / 'textures'
    scripts = root / 'materials' / 'scripts'
    textures.mkdir(parents=True)
    scripts.mkdir(parents=True)

    dictionary = cv2.aruco.getPredefinedDictionary(FAMILY_IDS[contract['family']])
    size = rendering['texture_size_px']
    material_blocks = []
    for _, _, face in enabled_tags(contract):
        tag_id = face['tag_id']
        image = cv2.aruco.generateImageMarker(dictionary, tag_id, size)
        texture_path = textures / f'tag_{tag_id}.png'
        if not cv2.imwrite(str(texture_path), image):
            raise ContractError(f'no se pudo escribir {texture_path}')

        margin = max(16, size // 4)
        canvas = np.full((size + 2 * margin, size + 2 * margin), 255, dtype=np.uint8)
        canvas[margin:margin + size, margin:margin + size] = image
        corners, ids, _ = cv2.aruco.detectMarkers(canvas, dictionary)
        detected = [] if ids is None else [int(value) for value in ids.flatten()]
        if detected != [tag_id] or len(corners) != 1:
            raise ContractError(f'la textura tag_{tag_id}.png no se redetecta correctamente')
        if logger:
            logger(f'[FID-TEXTURE-VERIFIED] tag_id={tag_id} path={texture_path}')
        material_blocks.append(
            'material Fiducial/Tag{0}\n{{\n  technique\n  {{\n    pass\n'
            '    {{\n      lighting off\n      texture_unit\n      {{\n'
            '        texture tag_{0}.png\n        filtering none\n'
            '      }}\n    }}\n  }}\n}}\n'.format(tag_id))

    (scripts / 'fiducials.material').write_text(
        '\n'.join(material_blocks), encoding='utf-8')
    return root


def _face_pose(face_name, size, offset):
    sx, sy, sz = size
    layouts = {
        'pos_x': ((sx / 2 + offset, 0.0, 0.0), (math.pi / 2, 0.0, math.pi / 2)),
        'neg_x': ((-sx / 2 - offset, 0.0, 0.0), (math.pi / 2, 0.0, -math.pi / 2)),
        'pos_y': ((0.0, sy / 2 + offset, 0.0), (math.pi / 2, 0.0, math.pi)),
        'neg_y': ((0.0, -sy / 2 - offset, 0.0), (math.pi / 2, 0.0, 0.0)),
        'pos_z': ((0.0, 0.0, sz / 2 + offset), (0.0, 0.0, 0.0)),
        'neg_z': ((0.0, 0.0, -sz / 2 - offset), (0.0, math.pi, 0.0)),
    }
    return layouts[face_name]


def _values(values):
    return ' '.join(f'{float(value):.9g}' for value in values)


def build_model_sdf(obj, rendering, assets_root):
    object_id = obj['object_id']
    size = tuple(float(obj['size_m'][axis]) for axis in ('x', 'y', 'z'))
    translation = obj['world_T_object']['translation_m']
    rotation = [math.radians(value) for value in obj['world_T_object']['rotation_rpy_deg']]
    scripts_uri = (Path(assets_root) / 'materials' / 'scripts').resolve().as_uri()
    textures_uri = (Path(assets_root) / 'materials' / 'textures').resolve().as_uri()
    visuals = []
    for face_name in FACES:
        face = obj['faces'][face_name]
        if not face['enabled']:
            continue
        position, rpy = _face_pose(
            face_name, size, rendering['surface_offset_m'])
        tag_id = face['tag_id']
        tag_size = float(face['size_m'])
        visuals.append(f'''
      <visual name="tag_{tag_id}_{face_name}">
        <pose>{_values(position + rpy)}</pose>
        <cast_shadows>false</cast_shadows>
        <geometry>
          <plane><normal>0 0 1</normal><size>{tag_size:.9g} {tag_size:.9g}</size></plane>
        </geometry>
        <material><script>
          <uri>{escape(scripts_uri)}</uri><uri>{escape(textures_uri)}</uri>
          <name>Fiducial/Tag{tag_id}</name>
        </script></material>
      </visual>''')
    pose = _values(tuple(translation) + tuple(rotation))
    box_size = _values(size)
    return f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="fiducial_object_{object_id}">
    <static>true</static>
    <pose>{pose}</pose>
    <link name="body">
      <collision name="body_collision">
        <geometry><box><size>{box_size}</size></box></geometry>
      </collision>
      <visual name="body_visual">
        <geometry><box><size>{box_size}</size></box></geometry>
        <material>
          <ambient>0.85 0.85 0.85 1</ambient>
          <diffuse>0.85 0.85 0.85 1</diffuse>
        </material>
      </visual>{''.join(visuals)}
    </link>
  </model>
</sdf>'''


def spawn_main():
    import rclpy
    from ament_index_python.packages import get_package_share_directory
    from gazebo_msgs.srv import SpawnEntity
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool

    rclpy.init()
    node = Node('fiducial_spawner')
    share = Path(get_package_share_directory('simulacion_dron'))
    node.declare_parameter(
        'config_file', str(share / 'config' / 'fiducial_objects.yaml'))
    node.declare_parameter(
        'render_config_file',
        str(share / 'config' / 'fiducial_rendering.yaml'))
    config_path = node.get_parameter('config_file').value
    rendering_path = node.get_parameter('render_config_file').value

    try:
        contract = validate_contract(load_yaml(config_path), require_baseline=True)
        rendering = validate_rendering(load_yaml(rendering_path))
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        ready_pub = node.create_publisher(Bool, rendering['ready_topic'], qos)
        assets = generate_assets(
            contract, rendering,
            lambda message: node.get_logger().info(message))
        node.get_logger().info(
            '[FID-SPAWN-WAIT] service=/spawn_entity timeout_sec='
            f'{rendering["spawn_service_timeout_sec"]}')
        client = node.create_client(SpawnEntity, '/spawn_entity')
        deadline = time.monotonic() + rendering['spawn_service_timeout_sec']
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                break
        if not client.service_is_ready():
            raise RuntimeError('timeout esperando /spawn_entity')
        node.get_logger().info('[FID-SPAWN-SERVICE-READY] service=/spawn_entity')

        for obj in contract['objects']:
            sdf = build_model_sdf(obj, rendering, assets)
            node.get_logger().info(
                f'[FID-SDF-BUILT] object_id={obj["object_id"]} bytes={len(sdf)}')
            request = SpawnEntity.Request()
            request.name = f'fiducial_object_{obj["object_id"]}'
            request.xml = sdf
            request.robot_namespace = ''
            request.reference_frame = 'world'
            future = client.call_async(request)
            rclpy.spin_until_future_complete(
                node, future,
                timeout_sec=rendering['spawn_response_timeout_sec'])
            if not future.done():
                raise RuntimeError(f'timeout spawneando {request.name}')
            response = future.result()
            if response is None or not response.success:
                status = (
                    '<sin respuesta>' if response is None else
                    response.status_message)
                raise RuntimeError(f'fallo spawneando {request.name}: {status}')
            tag_count = sum(
                1 for face in obj['faces'].values() if face['enabled'])
            node.get_logger().info(
                f'[FID-SPAWN-SUCCESS] object_id={obj["object_id"]} '
                f'name={request.name} tags={tag_count}')

        ready_pub.publish(Bool(data=True))
        node.get_logger().info(
            f'[FID-SPAWN-ALL-DONE] objects={len(contract["objects"])} '
            f'tags=15 ready_topic={rendering["ready_topic"]}')
        rclpy.spin(node)
        return 0
    except (KeyboardInterrupt, ExternalShutdownException):
        return 0
    except Exception as exc:
        node.get_logger().error(f'[FID-SPAWN-ERROR] {exc}')
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(spawn_main())
