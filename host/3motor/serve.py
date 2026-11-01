"""Local-only launcher. Reopening reuses this console's existing server."""
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.request import urlopen
import os
import webbrowser

ROOT = Path(__file__).resolve().parent
URL = 'http://127.0.0.1:8765/'

class Handler(SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header('Cache-Control', 'no-store')
        super().end_headers()

def main():
    os.chdir(ROOT)
    try:
        server = ThreadingHTTPServer(('127.0.0.1', 8765), Handler)
    except OSError:
        try:
            with urlopen(URL, timeout=1) as response:
                existing = response.read(3000)
            if b'3RRS' not in existing or b'core.js' not in existing:
                raise RuntimeError('Port 8765 is occupied by another application.')
        except Exception as error:
            raise RuntimeError('Cannot start 3RRS console on port 8765.') from error
        print('3RRS console already running: ' + URL, flush=True)
        webbrowser.open(URL)
        return
    print('3RRS console: ' + URL, flush=True)
    webbrowser.open(URL)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()

if __name__ == '__main__':
    main()
