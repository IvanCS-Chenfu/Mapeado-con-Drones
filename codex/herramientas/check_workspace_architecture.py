#!/usr/bin/env python3

import argparse
import ast
import filecmp
import json
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


SRC_ROOT = Path(__file__).resolve().parents[2]
POLICY_PATH = SRC_ROOT / 'codex/contexto/workspace_architecture.yaml'
RUNTIME_SUFFIXES = {
    '.cpp', '.cc', '.c', '.hpp', '.h', '.py', '.sh', '.yaml', '.yml',
    '.xml', '.cmake'}


class ArchitectureCheck:
    def __init__(self, policy):
        self.policy = policy
        self.errors = []
        self.passes = []

    def fail(self, check, message):
        self.errors.append((check, message))

    def passed(self, check, message):
        self.passes.append((check, message))

    def run(self, selected):
        checks = {
            'layout': self.check_layout,
            'interfaces': self.check_interfaces,
            'dependencies': self.check_dependencies,
            'config': self.check_config,
            'paths': self.check_runtime_paths,
            'visualizers': self.check_visualizers,
            'docs': self.check_docs,
        }
        names = checks if selected == 'all' else (selected,)
        for name in names:
            checks[name]()
        for check, message in self.passes:
            print(f'[ARCH-CHECK][PASS][{check}] {message}')
        for check, message in self.errors:
            print(f'[ARCH-CHECK][FAIL][{check}] {message}')
        print(
            f'[ARCH-CHECK][SUMMARY] checks={len(self.passes) + len(self.errors)} '
            f'failures={len(self.errors)}')
        return 1 if self.errors else 0

    def check_layout(self):
        expected_by_group = {}
        principal_directories = set()
        for group, definition in self.policy['groups'].items():
            expected = set(definition['packages'].values())
            expected_by_group[group] = expected
            for directory in definition['packages']:
                package_path = SRC_ROOT / group / directory
                principal_directories.add(directory)
                if not package_path.is_dir():
                    self.fail('layout', f'falta {package_path.relative_to(SRC_ROOT)}')
                if directory != 'ORB_SLAM3' and not (package_path / 'package.xml').is_file():
                    self.fail('layout', f'falta package.xml en {package_path}')

        for directory in principal_directories:
            if (SRC_ROOT / directory).exists():
                self.fail('layout', f'paquete principal reaparecio en raiz: {directory}')
        if (SRC_ROOT / 'ORB_SLAM3_MULTI').exists():
            self.fail('layout', 'ORB_SLAM3_MULTI debe permanecer eliminado')
        if (SRC_ROOT / 'fase45_sandbox').exists():
            self.fail('layout', 'fase45_sandbox no pertenece al workspace vigente')

        if shutil.which('colcon') is None:
            self.fail('layout', 'colcon no disponible para verificar descubrimiento')
            return
        for group, expected in expected_by_group.items():
            result = subprocess.run(
                ['colcon', 'list', '--base-paths', str(SRC_ROOT / group), '--names-only'],
                check=False, capture_output=True, text=True)
            if result.returncode != 0:
                self.fail('layout', f'colcon list fallo para {group}: {result.stderr.strip()}')
                continue
            discovered = {line.strip() for line in result.stdout.splitlines() if line.strip()}
            if discovered != expected:
                self.fail('layout', f'{group}: esperados={sorted(expected)} descubiertos={sorted(discovered)}')
            else:
                self.passed('layout', f'{group}: {len(expected)} paquetes correctos')

    @staticmethod
    def tree_files(root):
        return {
            path.relative_to(root): path
            for path in root.rglob('*')
            if path.is_file() and '__pycache__' not in path.parts
        }

    def compare_trees(self, check, source, target):
        source_files = self.tree_files(source)
        target_files = self.tree_files(target)
        if set(source_files) != set(target_files):
            missing = sorted(set(source_files) - set(target_files))
            extra = sorted(set(target_files) - set(source_files))
            self.fail(check, f'{target}: missing={missing} extra={extra}')
            return
        different = [
            str(relative) for relative in source_files
            if not filecmp.cmp(source_files[relative], target_files[relative], shallow=False)]
        if different:
            self.fail(check, f'{target} diverge en {different}')
        else:
            self.passed(check, f'{target.relative_to(SRC_ROOT)} replica exacta')

    def check_interfaces(self):
        source = SRC_ROOT / self.policy['canonical_interfaces']
        for target_path in self.policy['interface_replicas']:
            self.compare_trees('interfaces', source, SRC_ROOT / target_path)
        copies = list(SRC_ROOT.glob('*/orbslam3_msgs'))
        if len(copies) != 2:
            self.fail('interfaces', f'se esperaban 2 copias; encontradas {len(copies)}')
        else:
            self.passed('interfaces', 'exactamente dos copias de orbslam3_msgs')

    @staticmethod
    def manifest_dependencies(path):
        root = ET.parse(path).getroot()
        return {
            element.text.strip() for element in root
            if element.tag.endswith('depend') and element.text}

    def package_group_map(self):
        result = {}
        for group, definition in self.policy['groups'].items():
            for package_name in definition['packages'].values():
                result[package_name] = group
        return result

    def check_dependencies(self):
        prohibited = {
            'dron': {'orbslam3_multi', 'orbslam3_server', 'simulacion_dron', 'gazebo_ros', 'gazebo_msgs'},
            'servidor': {'dron_individual', 'lib_tray', 'simulacion_dron', 'gazebo_ros', 'gazebo_msgs'},
            'simulacion': set(),
        }
        for group, definition in self.policy['groups'].items():
            for directory in definition['packages']:
                manifest = SRC_ROOT / group / directory / 'package.xml'
                if not manifest.is_file():
                    continue
                dependencies = self.manifest_dependencies(manifest)
                forbidden = sorted(dependencies & prohibited[group])
                if forbidden:
                    self.fail('dependencies', f'{manifest}: prohibidas {forbidden}')
            for cmake in (SRC_ROOT / group).rglob('CMakeLists.txt'):
                source = cmake.read_text(encoding='utf-8', errors='replace')
                if group in ('dron', 'servidor') and (
                    'find_package(gazebo_ros' in source or 'find_package(gazebo_msgs' in source
                ):
                    self.fail('dependencies', f'Gazebo prohibido en {cmake}')

        package_groups = self.package_group_map()

        def call_name(node):
            if isinstance(node, ast.Name):
                return node.id
            if isinstance(node, ast.Attribute):
                parts = []
                current = node
                while isinstance(current, ast.Attribute):
                    parts.append(current.attr)
                    current = current.value
                if isinstance(current, ast.Name):
                    parts.append(current.id)
                return '.'.join(reversed(parts))
            return ''

        def foreign_resolvers_in(node, current_group):
            found = set()
            for child in ast.walk(node):
                if not isinstance(child, ast.Call):
                    continue
                name = call_name(child.func)
                if name not in {'FindPackageShare', 'get_package_share_directory'}:
                    continue
                if not child.args or not isinstance(child.args[0], ast.Constant):
                    continue
                package_name = child.args[0].value
                if not isinstance(package_name, str):
                    continue
                package_group = package_groups.get(package_name)
                if package_group is not None and package_group != current_group:
                    found.add(package_name)
            return found

        def python_cross_group_yaml(path, source, current_group):
            try:
                tree = ast.parse(source, filename=str(path))
            except SyntaxError as exc:
                self.fail('dependencies', f'{path}: Python no parseable: {exc}')
                return
            for node in ast.walk(tree):
                if not isinstance(node, ast.Call):
                    continue
                name = call_name(node.func)
                if name not in {'PathJoinSubstitution', 'os.path.join'}:
                    continue
                constants = {
                    child.value for child in ast.walk(node)
                    if isinstance(child, ast.Constant) and isinstance(child.value, str)
                }
                yaml_path = any(
                    value.lower().endswith(('.yaml', '.yml')) for value in constants)
                if 'config' not in constants or not yaml_path:
                    continue
                for package_name in sorted(foreign_resolvers_in(node, current_group)):
                    self.fail(
                        'dependencies',
                        f'{path}: carga YAML config cross-group desde {package_name}')

        def cpp_cross_group_yaml(path, source, current_group):
            for package_name, package_group in package_groups.items():
                if package_group == current_group:
                    continue
                resolver = re.compile(
                    rf"ament_index_cpp::get_package_share_directory\s*\(\s*['\"]"
                    rf"{re.escape(package_name)}['\"]\s*\)")
                for match in resolver.finditer(source):
                    # Sólo la misma sentencia C/C++; nunca una ventana arbitraria
                    # que pueda capturar la construcción de otro path posterior.
                    statement_end = source.find(';', match.end())
                    if statement_end < 0:
                        statement_end = source.find('\n', match.end())
                    if statement_end < 0:
                        statement_end = len(source)
                    statement = source[match.start():statement_end + 1]
                    if re.search(
                        r"/config/[^'\"]+\.ya?ml", statement, re.IGNORECASE):
                        self.fail(
                            'dependencies',
                            f'{path}: carga YAML config cross-group desde {package_name}')

        # La frontera se evalúa por expresión/sentencia. Referenciar el launch o
        # un recurso no-YAML de otro grupo es integración de despliegue válida;
        # cargar directamente su YAML operativo no lo es.
        for group in self.policy['groups']:
            for path in (SRC_ROOT / group).rglob('*'):
                if not path.is_file():
                    continue
                if group == 'dron' and 'ORB_SLAM3' in path.parts:
                    continue
                if 'test' in path.parts:
                    continue
                if path.name != 'CMakeLists.txt' and path.suffix not in RUNTIME_SUFFIXES:
                    continue
                source = path.read_text(encoding='utf-8', errors='replace')
                if path.suffix == '.py':
                    python_cross_group_yaml(path, source, group)
                elif path.suffix in {'.cpp', '.cc', '.c', '.hpp', '.h'}:
                    cpp_cross_group_yaml(path, source, group)
        if not any(check == 'dependencies' for check, _ in self.errors):
            self.passed('dependencies', 'dependencias y YAML cross-group respetan los grupos')

    @staticmethod
    def yaml_document(path):
        source = path.read_text(encoding='utf-8')
        if source.lstrip().startswith('%YAML:'):
            return None
        document = yaml.safe_load(source) or {}
        return document if isinstance(document, dict) else {}

    @classmethod
    def ros_parameters(cls, path):
        document = cls.yaml_document(path)
        if document is None:
            raise ValueError(f'{path} es YAML OpenCV, no YAML de parametros ROS')
        return document['/**']['ros__parameters']

    def check_config(self):
        for replica in self.policy['yaml_replicas']:
            source = SRC_ROOT / replica['source']
            for target_name in replica['targets']:
                target = SRC_ROOT / target_name
                mode = replica['mode']
                if mode == 'tree':
                    self.compare_trees('config', source, target)
                elif mode == 'keys':
                    keys = set(replica['keys'])
                    source_params = self.ros_parameters(source)
                    target_params = self.ros_parameters(target)
                    if set(target_params) != keys:
                        self.fail('config', f'{target}: claves={sorted(target_params)} esperadas={sorted(keys)}')
                    elif {k: source_params.get(k) for k in keys} != target_params:
                        self.fail('config', f'{target} diverge de claves declaradas en {source}')
                    else:
                        self.passed('config', f'{target.relative_to(SRC_ROOT)} replica de claves exactas')
                else:
                    self.fail('config', f'modo de replica desconocido: {mode}')

        # Duplicados semánticos cross-group: el nombre textual sólo es una
        # heurística fiable de ownership cuando la misma clave cruza fronteras de
        # despliegue. Dentro de un mismo grupo puede haber nodos distintos con la
        # misma clave sin compartir semántica. Las réplicas cross-group válidas
        # deben estar declaradas explícitamente en policy.
        if self.policy.get('semantic_duplicate_scope') != 'cross_group_only':
            self.fail('config', 'semantic_duplicate_scope debe ser cross_group_only')
        allowed_duplicate_clusters = []
        for replica in self.policy['yaml_replicas']:
            source = SRC_ROOT / replica['source']
            targets = [SRC_ROOT / name for name in replica['targets']]
            if replica['mode'] == 'keys':
                for key in replica['keys']:
                    allowed_duplicate_clusters.append(
                        (key, {source.resolve(), *(target.resolve() for target in targets)}))
            elif replica['mode'] == 'tree':
                for source_file in source.rglob('*.yaml'):
                    relative = source_file.relative_to(source)
                    params = self.yaml_document(source_file) or {}
                    params = params.get('/**', {}).get('ros__parameters', {})
                    cluster = {source_file.resolve()}
                    cluster.update((target / relative).resolve() for target in targets)
                    for key in params:
                        allowed_duplicate_clusters.append((key, cluster))

        occurrences = {}
        for group in self.policy['groups']:
            for path in (SRC_ROOT / group).rglob('config/**/*.yaml'):
                if not path.is_file():
                    continue
                document = self.yaml_document(path)
                if document is None:
                    continue
                params = document.get('/**', {}).get('ros__parameters', {})
                if not isinstance(params, dict):
                    continue
                for key in params:
                    occurrences.setdefault(key, []).append((group, path.resolve()))
        for key, entries in occurrences.items():
            groups = {group for group, _ in entries}
            if len(groups) < 2:
                continue
            unique_paths = {path for _, path in entries}
            declared = any(
                cluster_key == key and unique_paths.issubset(cluster)
                for cluster_key, cluster in allowed_duplicate_clusters)
            if not declared:
                shown = sorted(
                    f'{group}:{path.relative_to(SRC_ROOT)}' for group, path in entries)
                self.fail(
                    'config',
                    f'duplicado semantico cross-group no declarado {key}: {shown}')

        profile_path = SRC_ROOT / self.policy['debug_profile']
        profile = yaml.safe_load(profile_path.read_text(encoding='utf-8')).get('debug', {})
        expected = set(self.policy['debug_flags'])
        if set(profile) != expected:
            self.fail('config', f'flags debug esperados={sorted(expected)} reales={sorted(profile)}')
        for name, value in profile.items():
            if not isinstance(value, bool) or value is not False:
                self.fail('config', f'{name} debe ser booleano false por defecto')

        required = (
            'dron/dron_individual/config/physical.yaml',
            'dron/dron_individual/config/control.yaml',
            'dron/dron_individual/config/trajectory.yaml',
            'dron/dron_individual/config/actuators.yaml',
            'simulacion/simulacion_dron/config/physical_dron.yaml',
            'simulacion/simulacion_dron/config/actuators_dron.yaml',
            'simulacion/simulacion_dron/config/simulated_sensors.yaml',
        )
        for relative in required:
            if not (SRC_ROOT / relative).is_file():
                self.fail('config', f'falta {relative}')
        for retired in (
            'dron/dron_individual/config/hardware.yaml',
            'dron/dron_individual/config/tray_dron.yaml',
        ):
            if (SRC_ROOT / retired).exists():
                self.fail('config', f'config legacy sigue presente: {retired}')

        for group in self.policy['groups']:
            for path in (SRC_ROOT / group).rglob('*'):
                if (
                    path.is_file() and path.suffix in RUNTIME_SUFFIXES and
                    'test' not in path.parts
                ):
                    source = path.read_text(encoding='utf-8', errors='replace')
                    if 'usar_veltrap' in source:
                        self.fail('config', f'usar_veltrap sigue en runtime: {path}')

        vision = self.ros_parameters(SRC_ROOT / 'dron/dron_individual/config/vision.yaml')
        for key in ('orbslam.activar', 'orbslam.estereo'):
            if not isinstance(vision.get(key), bool):
                self.fail('config', f'{key} debe ser booleano YAML real')

        server_runtime = self.ros_parameters(
            SRC_ROOT / 'servidor/orbslam3_server/config/global_map/runtime.yaml')
        for key in ('use_sim_time', 'drone_count', 'drone_namespace_base'):
            if key in server_runtime:
                self.fail('config', f'{key} no debe competir con autoridad launch en runtime.yaml')

        dron_launch = (SRC_ROOT / 'dron/dron_individual/launch/generar_dron.launch.py').read_text(encoding='utf-8')
        orb_launch = (SRC_ROOT / 'dron/dron_individual/launch/orbslam_use.launch.py').read_text(encoding='utf-8')
        server_launch = (SRC_ROOT / 'servidor/orbslam3_server/launch/global_orb_map_server.launch.py').read_text(encoding='utf-8')
        sim_launch = (SRC_ROOT / 'simulacion/simulacion_dron/launch/multi_dron.launch.py').read_text(encoding='utf-8')
        for source, name in ((dron_launch, 'Dron'), (orb_launch, 'ORB'), (server_launch, 'Server')):
            if "DeclareLaunchArgument('use_sim_time', default_value='false')" not in source:
                self.fail('config', f'{name} standalone no fija use_sim_time=false')
        if "'use_sim_time': 'true'" not in sim_launch or "'use_sim_time': True" not in sim_launch:
            self.fail('config', 'Simulacion no aplica use_sim_time=true explicitamente')
        if 'hardware.yaml' in sim_launch:
            self.fail('config', 'Simulacion sigue cargando hardware.yaml de Dron')
        if 'ORBvoc_L5.txt' in sim_launch:
            self.fail('config', 'Simulacion no puede sustituir ORBvoc completo por L5')
        if "'orb_vocabulary_path', default_value=full_orb_vocabulary" not in sim_launch:
            self.fail('config', 'Simulacion debe conservar orb_vocabulary_path configurable')
        if "'orb_vocabulary_path': LaunchConfiguration('orb_vocabulary_path')" not in sim_launch:
            self.fail('config', 'Simulacion debe propagar orb_vocabulary_path a Dron')

        scenario = SRC_ROOT / self.policy['official_scenario']
        if not scenario.is_file():
            self.fail('config', f'falta escenario oficial {scenario}')
        else:
            yaml.safe_load(scenario.read_text(encoding='utf-8'))
            self.passed('config', 'escenario oficial instalado en Simulacion')
        if not any(check == 'config' for check, _ in self.errors):
            self.passed('config', 'ownership, replicas, reloj y debug coherentes')

    def check_runtime_paths(self):
        forbidden = (
            '/home/chenfu/Gazebo/src/ORB_SLAM3',
            '/home/chenfu/Gazebo/src/dron_individual',
            '/home/chenfu/Gazebo/src/orbslam3_',
            'src/dron/', 'src/servidor/', 'src/simulacion/',
            'codex/archivos_auxiliares',
        )
        for group in self.policy['groups']:
            for path in (SRC_ROOT / group).rglob('*'):
                if not path.is_file() or path.suffix not in RUNTIME_SUFFIXES or 'test' in path.parts:
                    continue
                source = path.read_text(encoding='utf-8', errors='replace')
                for pattern in forbidden:
                    if pattern in source:
                        self.fail('paths', f'{path}: ruta runtime prohibida {pattern}')
        generated = []
        for group in self.policy['groups']:
            root = SRC_ROOT / group
            generated.extend(root.rglob('__pycache__'))
            generated.extend(root.rglob('*.pyc'))
        if generated:
            self.fail('paths', f'artefactos generados dentro de grupos funcionales: {generated[:8]}')
        if not any(check == 'paths' for check, _ in self.errors):
            self.passed('paths', 'sin rutas runtime prohibidas ni artefactos generados')

    def load_graph(self):
        graph_path = SRC_ROOT / self.policy['system_architecture']['graph']
        source = graph_path.read_text(encoding='utf-8').strip()
        prefix = 'window.SYSTEM_ARCHITECTURE = '
        if not source.startswith(prefix):
            raise ValueError('prefijo graph_definition.js invalido')
        return json.loads(source[len(prefix):].removesuffix(';'))

    def load_graph_metadata(self):
        path = SRC_ROOT / self.policy['system_architecture']['metadata']
        source = path.read_text(encoding='utf-8').strip()
        prefix = 'window.SYSTEM_ARCHITECTURE_METADATA = '
        if not source.startswith(prefix):
            raise ValueError('prefijo graph_metadata.js invalido')
        return json.loads(source[len(prefix):].removesuffix(';'))

    def check_visualizers(self):
        package = SRC_ROOT / 'simulacion/simulacion_dron'
        required = (
            'web/pipeline_flow/index.html',
            'web/system_architecture/index.html',
            'web/system_architecture/graph_definition.js',
            'web/system_architecture/graph_metadata.js',
            'src/visualizer/pipeline_flow_bridge.py',
            'src/visualizer/system_architecture_bridge.py',
        )
        for relative in required:
            if not (package / relative).is_file():
                self.fail('visualizers', f'falta {relative}')

        graph = self.load_graph()
        metadata = self.load_graph_metadata()
        expected_packages = {
            (group, package_name)
            for group, definition in self.policy['groups'].items()
            for package_name in definition['packages'].values()
        }
        actual_packages = {
            (node['data']['group'], node['data']['label'])
            for node in graph['nodes'] if node['data']['kind'] == 'package'
        }
        if actual_packages != expected_packages:
            self.fail('visualizers', f'paquetes graph={sorted(actual_packages)} policy={sorted(expected_packages)}')
        package_ids = {
            node['data']['id'] for node in graph['nodes']
            if node['data']['kind'] == 'package'}
        required_node_fields = {
            'path', 'ros_name', 'executables', 'owned_yaml', 'dependencies',
            'cross_group', 'status', 'docs'}
        if set(metadata.get('nodes', {})) != package_ids:
            self.fail('visualizers', 'metadata de paquetes no coincide con el grafo')
        for node_id, values in metadata.get('nodes', {}).items():
            missing = sorted(required_node_fields - set(values))
            if missing:
                self.fail('visualizers', f'{node_id}: metadata incompleta {missing}')

        layers = {'runtime', 'build', 'config', 'deployment'}
        runtime_edges = set(self.policy['system_architecture']['runtime_edges'])
        graph_runtime = set()
        for edge in graph['edges']:
            data = edge['data']
            if data.get('layer') not in layers:
                self.fail('visualizers', f"capa invalida en {data.get('id')}: {data.get('layer')}")
            if not data.get('interface'):
                self.fail('visualizers', f"falta interfaz en {data.get('id')}")
            if data.get('layer') == 'runtime':
                graph_runtime.add(data['id'])
                if data.get('activity_mode') != 'direct':
                    self.fail('visualizers', f"runtime no directo: {data['id']}")
            elif data.get('activity_mode') != 'none':
                self.fail('visualizers', f"arista estatica puede pulsar: {data['id']}")
        if graph_runtime != runtime_edges:
            self.fail('visualizers', f'runtime graph={sorted(graph_runtime)} policy={sorted(runtime_edges)}')
        required_edge_fields = {'message_type', 'namespace', 'qos', 'data_transferred'}
        if set(metadata.get('edges', {})) != runtime_edges:
            self.fail('visualizers', 'metadata runtime no coincide con las aristas declaradas')
        for edge_id, values in metadata.get('edges', {}).items():
            missing = sorted(required_edge_fields - set(values))
            if missing:
                self.fail('visualizers', f'{edge_id}: metadata incompleta {missing}')

        bridge = (package / 'src/visualizer/system_architecture_bridge.py').read_text(encoding='utf-8')
        if '/system_architecture/activity' not in bridge:
            self.fail('visualizers', 'system_architecture no usa su canal propio')
        for forbidden in (
            '/global_mapping/flow_events', 'sensor_msgs.msg import Image',
            'PointCloud2', 'orbslam3_msgs'):
            if forbidden in bridge:
                self.fail('visualizers', f'system_architecture observa trafico prohibido: {forbidden}')
        if "if edge_id not in RUNTIME_EDGES" not in bridge:
            self.fail('visualizers', 'eventos desconocidos no se descartan explicitamente')

        server = (SRC_ROOT / 'servidor/orbslam3_server/src/global_map_server.cpp').read_text(encoding='utf-8')
        if 'if (!pipeline_flow_events_enabled_ || !flow_publisher_)' not in server:
            self.fail('visualizers', 'pipeline_flow producer no queda gated')
        if 'if (pipeline_flow_events_enabled_) {' not in server:
            self.fail('visualizers', 'publisher pipeline_flow se crea aunque este apagado')
        if '#define F2_PIPELINE_FLOW_EVENT' not in server:
            self.fail('visualizers', 'call sites pipeline_flow no estan lazy-gated')
        if server.count('EmitFlowEvent(') != 2 or server.count('EmitSecondaryFlowEvent(') != 2 or server.count('EmitSecondaryLifecycleEvent(') != 2:
            self.fail('visualizers', 'persisten llamadas directas a productores flow fuera de sus definiciones')

        launch = (package / 'launch/multi_dron.launch.py').read_text(encoding='utf-8')
        if "architecture_telemetry_enabled = PythonExpression" not in launch:
            self.fail('visualizers', 'falta gating combinado master+telemetry')
        if "'debug_pipeline_flow_events': LaunchConfiguration('debug_pipeline_flow_web')" not in launch:
            self.fail('visualizers', 'pipeline_flow web no gobierna productor Server')
        if not any(check == 'visualizers' for check, _ in self.errors):
            self.passed('visualizers', 'visualizadores independientes, directos y gated')

    def check_docs(self):
        required_snippets = {
            'codex/contexto/00_CONTEXTO_COMPACTACION.md': ('VALIDACIÓN FINAL', 'regresión equivalente a 198'),
            'codex/contexto/05_MAPA_PAQUETES.md': ('src/dron/', 'src/servidor/', 'src/simulacion/'),
            'codex/contexto/paquetes/simulacion_dron/system_architecture_visualizer.md': ('evidencia directa',),
            'codex/contexto/decisiones/ADR_0010_observabilidad_web_debug_coste_cero.md': ('pipeline_flow', 'system_architecture'),
        }
        for relative, snippets in required_snippets.items():
            path = SRC_ROOT / relative
            if not path.is_file():
                self.fail('docs', f'falta {relative}')
                continue
            source = path.read_text(encoding='utf-8')
            for snippet in snippets:
                if snippet not in source:
                    self.fail('docs', f'{relative} no contiene {snippet}')
        if not any(check == 'docs' for check, _ in self.errors):
            self.passed('docs', 'documentacion refleja validacion final post-implementacion')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--check',
        choices=('all', 'layout', 'interfaces', 'dependencies', 'config', 'paths', 'visualizers', 'docs'),
        default='all')
    args = parser.parse_args()
    policy = yaml.safe_load(POLICY_PATH.read_text(encoding='utf-8'))
    return ArchitectureCheck(policy).run(args.check)


if __name__ == '__main__':
    sys.exit(main())
