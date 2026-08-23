#!/usr/bin/env python3

import argparse
import json
import time
import webbrowser
from urllib.error import URLError
from urllib.request import urlopen


def wait_until_ready(port, timeout):
    health_url = f'http://127.0.0.1:{port}/health'
    deadline = time.monotonic() + timeout
    last_error = 'sin respuesta'

    while time.monotonic() < deadline:
        try:
            with urlopen(health_url, timeout=0.5) as response:
                health = json.loads(response.read().decode('utf-8'))
            if health.get('status') == 'ready':
                return health
            last_error = f'estado={health.get("status", "desconocido")}'
        except (OSError, URLError, ValueError, json.JSONDecodeError) as error:
            last_error = str(error)
        time.sleep(0.1)

    raise RuntimeError(
        f'el bridge no alcanzo ready en {timeout:.1f}s: {last_error}')


def browser_url(port):
    cache_token = time.time_ns()
    return f'http://127.0.0.1:{port}/?fresh={cache_token}'


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, required=True)
    parser.add_argument('--timeout', type=float, default=20.0)
    args, _unknown = parser.parse_known_args()

    try:
        health = wait_until_ready(args.port, args.timeout)
        url = browser_url(args.port)
        opened = webbrowser.open_new_tab(url)
        if not opened:
            raise RuntimeError(f'ningun navegador acepto la URL {url}')

        print(
            '[SYSTEM-ARCH-BROWSER-OPEN] '
            f'url={url} mode={health.get("mode", "unknown")} '
            f'latest_sequence={health.get("latest_sequence", 0)}',
            flush=True)
    except Exception as error:
        print(
            f'[SYSTEM-ARCH-BROWSER-ERROR] port={args.port} error={error}',
            flush=True)
        raise


if __name__ == '__main__':
    main()
