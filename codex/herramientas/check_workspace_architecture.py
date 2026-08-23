#!/usr/bin/env python3

import argparse
import filecmp
import json
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
            for directory, package_name in definition['packages'].items():
                package_path = SRC_ROOT / group / directory
                principal_directories.add(directory)
                if not package_path.is_dir():
                    self.fail('layout', f'falta {package_path.relative_to(SRC_ROOT)}')
                if directory != 'ORB_SLAM3' and not (
                        package_path / 'package.xml').is_file():
                    self.fail('layout', f'falta package.xml en {package_path}')

        for directory in principal_directories:
            if (SRC_ROOT / directory).exists():
                self.fail('layout', f'paquete principal reaparecio en raiz: {directory}')
        if (SRC_ROOT / 'ORB_SLAM3_MULTI').exists():
            self.fail('layout', 'ORB_SLAM3_MULTI debe permanecer eliminado')
        for exception in self.policy['root_exceptions']:
            if not (SRC_ROOT / exception).exists():
                self.fail('layout', f'excepcion de raiz ausente: {exception}')

        for group, expected in expected_by_group.items():
            command = [
                'colcon', 'list', '--base-paths', str(SRC_ROOT / group),
                '--names-only']
            result = subprocess.run(
                command, check=False, capture_output=True, text=True)
            if result.returncode != 0:
                self.fail('layout', f'colcon list fallo para {group}: {result.stderr.strip()}')
                continue
            discovered = {line.strip() for line in result.stdout.splitlines()
                          if line.strip()}
            if discovered != expected:
                self.fail(
                    'layout',
                    f'{group}: esperados={sorted(expected)} descubiertos={sorted(discovered)}')
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
            if not filecmp.cmp(
                source_files[relative], target_files[relative], shallow=False)]
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

    def check_dependencies(self):
        prohibited = {
            'dron': {
                'orbslam3_multi', 'orbslam3_server', 'simulacion_dron',
                'gazebo_ros', 'gazebo_msgs'},
            'servidor': {
                'dron_individual', 'lib_tray', 'simulacion_dron',
                'gazebo_ros', 'gazebo_msgs'},
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
                        'find_package(gazebo_ros' in source or
                        'find_package(gazebo_msgs' in source):
                    self.fail('dependencies', f'Gazebo prohibido en {cmake}')
        if not any(check == 'dependencies' for check, _ in self.errors):
            self.passed('dependencies', 'manifests y CMake respetan los grupos')

    @staticmethod
    def ros_parameters(path):
        document = yaml.safe_load(path.read_text(encoding='utf-8'))
        return document['/**']['ros__parameters']

    def check_config(self):
        for replica in self.policy['yaml_replicas']:
            source = SRC_ROOT / replica['source']
            for target_name in replica['targets']:
                target = SRC_ROOT / target_name
                if replica['mode'] == 'tree':
                    self.compare_trees('config', source, target)
                elif self.ros_parameters(source) != self.ros_parameters(target):
                    self.fail('config', f'{target} diverge de {source}')
                else:
                    self.passed('config', f'{target.relative_to(SRC_ROOT)} replica declarada')

        profile_path = SRC_ROOT / self.policy['debug_profile']
        profile = yaml.safe_load(
            profile_path.read_text(encoding='utf-8')).get('debug', {})
        expected = set(self.policy['debug_flags'])
        if set(profile) != expected:
            self.fail('config', f'flags debug esperados={sorted(expected)} reales={sorted(profile)}')
        for name, value in profile.items():
            if not isinstance(value, bool) or value is not False:
                self.fail('config', f'{name} debe ser booleano false por defecto')

        launch = (SRC_ROOT / 'simulacion/simulacion_dron/launch/multi_dron.launch.py').read_text(
            encoding='utf-8')
        for flag in expected:
            if flag not in launch:
                self.fail('config', f'{flag} no tiene consumidor en multi_dron.launch.py')

        physical = self.ros_parameters(
            SRC_ROOT / 'dron/dron_individual/config/physical.yaml')
        if set(physical) != {'fisico.total.masa', 'fisico.total.matriz_inercia'}:
            self.fail('config', 'physical.yaml no es el propietario unico esperado')
        tray = self.ros_parameters(
            SRC_ROOT / 'dron/dron_individual/config/tray_dron.yaml')
        if set(physical) & set(tray):
            self.fail('config', 'tray_dron.yaml duplica propiedades fisicas totales')

        scenario = SRC_ROOT / self.policy['official_scenario']
        if not scenario.is_file():
            self.fail('config', f'falta escenario oficial {scenario}')
        else:
            yaml.safe_load(scenario.read_text(encoding='utf-8'))
            self.passed('config', 'escenario oficial instalado en Simulacion')
        if not any(check == 'config' for check, _ in self.errors):
            self.passed('config', 'YAML, replicas y debug coherentes')

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
                if not path.is_file() or path.suffix not in RUNTIME_SUFFIXES:
                    continue
                if 'test' in path.parts:
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
            self.fail('paths', f'artefactos generados dentro de src: {generated[:8]}')
        if not any(check == 'paths' for check, _ in self.errors):
            self.passed('paths', 'sin rutas runtime prohibidas ni artefactos generados')

    def check_visualizers(self):
        package = SRC_ROOT / 'simulacion/simulacion_dron'
        required = (
            'web/pipeline_flow/index.html',
            'web/system_architecture/index.html',
            'web/system_architecture/graph_definition.js',
            'src/visualizer/pipeline_flow_bridge.py',
            'src/visualizer/system_architecture_bridge.py',
        )
        for relative in required:
            if not (package / relative).is_file():
                self.fail('visualizers', f'falta {relative}')
        graph_path = package / 'web/system_architecture/graph_definition.js'
        if graph_path.is_file():
            source = graph_path.read_text(encoding='utf-8').strip()
            prefix = 'window.SYSTEM_ARCHITECTURE = '
            graph = json.loads(source[len(prefix):].removesuffix(';'))
            packages = [node for node in graph['nodes']
                        if node['data']['kind'] == 'package']
            if len(packages) != 9:
                self.fail('visualizers', f'system_architecture tiene {len(packages)} paquetes')
        if not any(check == 'visualizers' for check, _ in self.errors):
            self.passed('visualizers', 'pipeline_flow y system_architecture separados')

    def check_docs(self):
        required_snippets = {
            'codex/contexto/05_MAPA_PAQUETES.md': ('src/dron/', 'src/servidor/', 'src/simulacion/'),
            'codex/contexto/paquetes/simulacion_dron/00_summary.md': ('system_architecture',),
            'codex/contexto/paquetes/simulacion_dron/launches.md': ('debug.yaml',),
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
            self.passed('docs', 'documentacion operativa contiene rutas y herramientas vigentes')


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--check',
        choices=(
            'all', 'layout', 'interfaces', 'dependencies', 'config',
            'paths', 'visualizers', 'docs'),
        default='all')
    args = parser.parse_args()
    policy = yaml.safe_load(POLICY_PATH.read_text(encoding='utf-8'))
    return ArchitectureCheck(policy).run(args.check)


if __name__ == '__main__':
    sys.exit(main())
