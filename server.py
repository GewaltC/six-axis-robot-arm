import http.server
import mimetypes
import os
import socketserver
import sys

PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
ROOT = os.path.dirname(os.path.abspath(__file__))

# Ensure clean MIME types on Windows for ES modules and 3D assets
mimetypes.add_type('application/javascript', '.js')
mimetypes.add_type('application/javascript', '.mjs')
mimetypes.add_type('application/xml', '.urdf')
mimetypes.add_type('model/vnd.collada+xml', '.dae')
mimetypes.add_type('model/gltf-binary', '.glb')
mimetypes.add_type('model/gltf+json', '.gltf')
mimetypes.add_type('model/stl', '.stl')


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=ROOT, **kwargs)

    def end_headers(self):
        self.send_header('Cache-Control', 'no-cache')
        super().end_headers()


if __name__ == '__main__':
    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(('127.0.0.1', PORT), QuietHandler) as httpd:
        print(f'RoboStudio 6-DOF server running at http://127.0.0.1:{PORT}/')
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            pass
