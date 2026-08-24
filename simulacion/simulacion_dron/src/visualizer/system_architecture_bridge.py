#!/usr/bin/env python3

import json
import mimetypes
import os
import threading
from collections import defaultdict, deque
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


RUNTIME_EDGES = {
    'sim_to_orbslam_stereo',
    'sim_to_dron_gt',
    'sim_to_server_fiducial_gt',
    'sim_to_dron_action',
    'dron_to_sim_motors',
    'orbslam_to_server_delta',
    'server_to_orbslam_snapshot_request',
    'orbslam_to_server_snapshot_response',
    'server_to_sim_backpressure',
    'server_to_sim_sparse_map',
    'server_to_sim_pipeline_flow',
}


class EventStore:
    def __init__(self, capacity=512):
        self._events = deque(maxlen=capacity)
        self._condition = threading.Condition()
        self._sequence = 0
        self._capacity = capacity

    def append(self, payload):
        with self._condition:
            self._sequence += 1
            self._events.append((self._sequence, payload))
            self._condition.notify_all()
            return self._sequence

    def start_cursor(self, last_event_id):
        with self._condition:
            latest = self._sequence
            if last_event_id in (None, ''):
                return latest, False
            try:
                requested = int(last_event_id)
            except (TypeError, ValueError):
                return latest, True
            oldest = self._events[0][0] if self._events else latest + 1
            if requested > latest or requested < oldest - 1:
                return latest, True
            return requested, False

    def wait_after(self, sequence, timeout=10.0):
        with self._condition:
            self._condition.wait_for(
                lambda: self._sequence > sequence, timeout=timeout)
            return [event for event in self._events if event[0] > sequence]

    def health(self, telemetry_enabled):
        with self._condition:
            return {
                'status': 'ready',
                'mode': 'live' if telemetry_enabled else 'static',
                'capacity': self._capacity,
                'latest_sequence': self._sequence,
                'telemetry_enabled': telemetry_enabled,
            }


def make_handler(web_root, events, telemetry_enabled):
    class ArchitectureHandler(SimpleHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def end_headers(self):
            path = urlparse(self.path).path
            if path not in ('/events', '/health'):
                self.send_header('Cache-Control', 'no-store, max-age=0')
            super().end_headers()

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == '/events':
                self._serve_events()
                return
            if parsed.path == '/health':
                self._serve_health()
                return
            self.path = '/index.html' if parsed.path == '/' else parsed.path
            super().do_GET()

        def translate_path(self, path):
            relative = Path(urlparse(path).path.lstrip('/'))
            candidate = Path(os.path.abspath(web_root / relative))
            try:
                candidate.relative_to(web_root)
            except ValueError:
                return str(web_root / '__forbidden__')
            return str(candidate)

        def guess_type(self, path):
            return mimetypes.guess_type(path)[0] or 'application/octet-stream'

        def log_message(self, _format, *_args):
            return

        def _serve_health(self):
            payload = json.dumps(events.health(telemetry_enabled)).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Content-Length', str(len(payload)))
            self.end_headers()
            self.wfile.write(payload)

        def _serve_events(self):
            if not telemetry_enabled:
                self.send_response(204)
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                return
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            sequence, reset_required = events.start_cursor(
                self.headers.get('Last-Event-ID'))
            try:
                if reset_required:
                    reset = json.dumps({
                        'kind': 'state_reset',
                        'seq': sequence,
                        'reason': 'event_history_unavailable',
                    })
                    frame = (
                        f'id: {sequence}\nevent: state_reset\n'
                        f'data: {reset}\n\n')
                    self.wfile.write(frame.encode('utf-8'))
                    self.wfile.flush()
                while True:
                    pending = events.wait_after(sequence)
                    if not pending:
                        self.wfile.write(b': heartbeat\n\n')
                        self.wfile.flush()
                        continue
                    for sequence, payload in pending:
                        frame = f'id: {sequence}\ndata: {payload}\n\n'
                        self.wfile.write(frame.encode('utf-8'))
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return

    return ArchitectureHandler


class SystemArchitectureBridge(Node):
    def __init__(self):
        super().__init__('system_architecture_bridge')
        self.declare_parameter('port', 8775)
        self.declare_parameter('web_root', '')
        self.declare_parameter('telemetry_enabled', False)

        port = int(self.get_parameter('port').value)
        web_root = Path(self.get_parameter('web_root').value).resolve()
        self._telemetry_enabled = bool(
            self.get_parameter('telemetry_enabled').value)
        if not web_root.is_dir():
            raise RuntimeError(f'web_root invalido: {web_root}')

        self._events = EventStore()
        self._counts = defaultdict(int)
        self._subscription = None
        if self._telemetry_enabled:
            qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=128,
                reliability=QoSReliabilityPolicy.BEST_EFFORT)
            self._subscription = self.create_subscription(
                String,
                '/system_architecture/activity',
                self._on_activity,
                qos)

        handler = make_handler(web_root, self._events, self._telemetry_enabled)
        self._http = ThreadingHTTPServer(('127.0.0.1', port), handler)
        self._http.daemon_threads = True
        self._http_thread = threading.Thread(
            target=self._http.serve_forever,
            name='system-architecture-http',
            daemon=True)
        self._http_thread.start()
        mode = 'live' if self._telemetry_enabled else 'static'
        self.get_logger().warning(
            f'[SYSTEM-ARCH-READY] url=http://127.0.0.1:{port} mode={mode}')

    def _on_activity(self, message):
        try:
            event = json.loads(message.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if event.get('kind') != 'architecture_activity':
            return
        edge_id = str(event.get('edge_id', ''))
        if edge_id not in RUNTIME_EDGES:
            return
        required = ('source', 'interface', 'interface_kind', 'timestamp')
        if any(name not in event for name in required):
            return
        if not isinstance(event.get('drone_id', 0), int):
            return
        self._counts[edge_id] += 1
        event['count'] = self._counts[edge_id]
        event['status'] = 'ok'
        self._events.append(json.dumps(event, separators=(',', ':')))

    def destroy_node(self):
        self._http.shutdown()
        self._http.server_close()
        self._http_thread.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = SystemArchitectureBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
