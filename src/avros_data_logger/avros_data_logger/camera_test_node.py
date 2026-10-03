"""Browser-based RGB and thermal camera test without writing data."""

import argparse
import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import cv2


HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>AV ROS Imitation Learning Data Logger (Camera Test)</title>
<style>
body { margin: 0; background: #181818; color: white; font-family: sans-serif; }
header { padding: 12px 16px; background: #282828; font-size: 20px; }
main { display: flex; flex-direction: column; gap: 12px; padding: 12px; }
.panel { position: relative; height: calc(50vh - 58px); min-height: 180px;
         background: black; display: flex; align-items: center;
         justify-content: center; overflow: hidden; }
.panel img { width: 100%; height: 100%; object-fit: contain; }
.label { position: absolute; top: 8px; left: 8px; background: #000b;
         padding: 5px 8px; z-index: 1; }
.unavailable { color: #aaa; font-size: 22px; }
</style>
</head>
<body>
<header>AV ROS Imitation Learning Data Logger (Camera Test)</header>
<main>
  <section class="panel"><span class="label">RGB</span>
    <img id="rgb" alt="RGB camera"><span id="rgb-unavailable" class="unavailable">Unavailable</span>
  </section>
  <section class="panel"><span class="label">Thermal</span>
    <img id="thermal" alt="Thermal camera"><span id="thermal-unavailable" class="unavailable">Unavailable</span>
  </section>
</main>
<script>
function updateFrame(name, available) {
  const image = document.getElementById(name);
  const message = document.getElementById(name + '-unavailable');
  image.style.display = available ? 'block' : 'none';
  message.style.display = available ? 'none' : 'block';
  if (available) image.src = '/frame/' + name + '.jpg?t=' + Date.now();
}
async function refresh() {
  try {
    const response = await fetch('/status?t=' + Date.now());
    const status = await response.json();
    updateFrame('rgb', status.rgb_available);
    updateFrame('thermal', status.thermal_available);
  } catch (error) {
    updateFrame('rgb', false);
    updateFrame('thermal', false);
  }
}
refresh();
setInterval(refresh, 250);
</script>
</body>
</html>"""


class PreviewServer(HTTPServer):
    allow_reuse_address = True

    def __init__(self, address, handler, state):
        self.state = state
        super().__init__(address, handler)


class PreviewHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = self.path.split('?', 1)[0]
        if path == '/':
            self._send(200, 'text/html; charset=utf-8', HTML.encode())
            return
        if path == '/status':
            with self.server.state['lock']:
                body = json.dumps({
                    'rgb_available': self.server.state['rgb'] is not None,
                    'thermal_available': self.server.state['thermal'] is not None,
                }).encode()
            self._send(200, 'application/json', body)
            return
        if path in ('/frame/rgb.jpg', '/frame/thermal.jpg'):
            name = 'rgb' if path.endswith('rgb.jpg') else 'thermal'
            with self.server.state['lock']:
                body = self.server.state[name]
            if body is None:
                self._send(404, 'text/plain', b'Unavailable')
            else:
                self._send(200, 'image/jpeg', body)
            return
        self._send(404, 'text/plain', b'Not found')

    def _send(self, status, content_type, body):
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_args):
        return


def open_camera(index, name):
    camera = cv2.VideoCapture(index)
    if not camera.isOpened():
        camera.release()
        print(f'{name} camera index {index} unavailable')
        return None
    print(f'{name} camera opened at index {index}')
    return camera


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rgb-index', type=int, default=0)
    parser.add_argument('--thermal-index', type=int, default=1)
    parser.add_argument('--port', type=int, default=8081)
    options = parser.parse_args(args)

    state = {'rgb': None, 'thermal': None, 'lock': threading.Lock()}
    rgb_camera = open_camera(options.rgb_index, 'RGB')
    thermal_camera = open_camera(options.thermal_index, 'Thermal')
    server = PreviewServer(('127.0.0.1', options.port), PreviewHandler, state)
    capture_stop = threading.Event()

    def capture_loop():
        while not capture_stop.is_set():
            rgb = thermal = None
            if rgb_camera is not None:
                ok, rgb = rgb_camera.read()
                if not ok:
                    rgb = None
            if thermal_camera is not None:
                ok, thermal = thermal_camera.read()
                if not ok:
                    thermal = None
            with state['lock']:
                state['rgb'] = _encode(rgb)
                state['thermal'] = _encode(thermal)
            capture_stop.wait(0.05)

    thread = threading.Thread(target=capture_loop, daemon=True)
    thread.start()
    print(f'Camera test available at http://127.0.0.1:{options.port}')
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        capture_stop.set()
        server.shutdown()
        server.server_close()
        if rgb_camera is not None:
            rgb_camera.release()
        if thermal_camera is not None:
            thermal_camera.release()


def _encode(image):
    if image is None:
        return None
    ok, buffer = cv2.imencode('.jpg', image)
    return buffer.tobytes() if ok else None


if __name__ == '__main__':
    main()
