#!/usr/bin/env python3

import mimetypes
import os
import threading
import time
from collections import deque
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class EventStore:
    def __init__(self, capacity=512):
        self._events = deque(maxlen=capacity)
        self._condition = threading.Condition()
        self._sequence = 0

    def append(self, payload):
        with self._condition:
            self._sequence += 1
            self._events.append((self._sequence, payload))
            self._condition.notify_all()

    def wait_after(self, sequence, timeout=10.0):
        with self._condition:
            self._condition.wait_for(
                lambda: self._sequence > sequence, timeout=timeout)
            return [event for event in self._events if event[0] > sequence]


def make_handler(web_root, events):
    class FlowHandler(SimpleHTTPRequestHandler):
        protocol_version = 'HTTP/1.1'

        def do_GET(self):
            parsed = urlparse(self.path)
            if parsed.path == '/events':
                self._serve_events()
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

        def _serve_events(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-cache')
            self.send_header('Connection', 'keep-alive')
            self.end_headers()
            sequence = 0
            try:
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

    return FlowHandler


class PipelineFlowBridge(Node):
    def __init__(self):
        super().__init__('pipeline_flow_bridge')
        self.declare_parameter('topic', '/global_mapping/flow_events')
        self.declare_parameter('port', 8765)
        self.declare_parameter('web_root', '')
        topic = self.get_parameter('topic').value
        port = int(self.get_parameter('port').value)
        web_root = Path(self.get_parameter('web_root').value).resolve()
        if not web_root.is_dir():
            raise RuntimeError(f'web_root invalido: {web_root}')

        self._events = EventStore()
        self._subscription = self.create_subscription(
            String, topic, self._on_event, 64)
        handler = make_handler(web_root, self._events)
        self._http = ThreadingHTTPServer(('127.0.0.1', port), handler)
        self._http.daemon_threads = True
        self._http_thread = threading.Thread(
            target=self._http.serve_forever,
            name='pipeline-flow-http',
            daemon=True)
        self._http_thread.start()
        self.get_logger().warning(
            f'[F1U-FLOW-WEB-READY] url=http://127.0.0.1:{port} '
            f'topic={topic} web_root={web_root}')

    def _on_event(self, message):
        self._events.append(message.data)

    def destroy_node(self):
        self._http.shutdown()
        self._http.server_close()
        self._http_thread.join(timeout=2.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = PipelineFlowBridge()
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
