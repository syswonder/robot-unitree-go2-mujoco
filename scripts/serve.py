#!/usr/bin/env python3
"""Serve mutable source without caching while caching large runtime assets."""

import argparse
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import unquote, urlsplit


class DevelopmentRequestHandler(SimpleHTTPRequestHandler):
    def send_head(self):
        """Apply public-asset restrictions to both GET and HEAD requests."""
        parts = Path(unquote(urlsplit(self.path).path)).parts
        if any(part.startswith('.') for part in parts) or (
            len(parts) > 1 and parts[1] not in {'index.html', 'assets', 'src', 'node_modules', 'native.html'}
        ):
            self.send_error(404)
            return None
        return super().send_head()

    def list_directory(self, path):
        self.send_error(404)
        return None

    def end_headers(self) -> None:
        """Cache immutable heavy assets while keeping source refreshable."""
        extension = Path(self.path.split("?", 1)[0]).suffix.lower()
        if extension in {".obj", ".png", ".stl", ".spz", ".wasm", ".onnx"}:
            self.send_header("Cache-Control", "public, max-age=86400")
        else:
            self.send_header("Cache-Control", "no-cache, must-revalidate")
        super().end_headers()


def main() -> None:
    """Run the local threaded application asset server."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5173)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.bind, args.port), DevelopmentRequestHandler)
    print(f"Serving MuJoCo Robonix on http://{args.bind}:{args.port}", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
