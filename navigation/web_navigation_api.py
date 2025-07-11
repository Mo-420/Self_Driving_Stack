#!/usr/bin/env python3
"""web_navigation_api.py
Phase-1 Navigation API Server
-----------------------------
A lightweight Flask + Socket.IO server that:
• receives destination coordinates from the web-app,
• exposes start/stop/pause endpoints,
• broadcasts real-time robot status (position, navigation state) every 0.5 s.
The server simply forwards commands to a shared `NavigationController` singleton
(imported from `navigation_controller.py`).

Run with:
    python3 web_navigation_api.py --host 0.0.0.0 --port 5000
"""
from __future__ import annotations

import argparse
import json
import threading
import time
from typing import Dict, Any

# Third-party imports (optionally installed in dev env). We import lazily to
# keep static analysers happy even if the packages are missing at runtime.
try:
    from flask import Flask, request, jsonify
    from flask_cors import CORS
    from flask_socketio import SocketIO
except ModuleNotFoundError:  # pragma: no cover – allow static type checking
    Flask = object  # type: ignore
    request = jsonify = lambda *a, **k: None  # type: ignore
    CORS = SocketIO = lambda *a, **k: None  # type: ignore

# Local imports (navigation logic)
from .navigation_controller import NavigationController, NavigationState

UPDATE_INTERVAL = 0.5  # seconds

app = Flask(__name__)
CORS(app)  # allow all origins (configurable)
socketio = SocketIO(app, cors_allowed_origins="*")

controller = NavigationController.get_instance()

# ---------------------------------------------------------------------------
# REST endpoints
# ---------------------------------------------------------------------------
@app.route("/api/navigation/start", methods=["POST"])
def api_start_navigation():
    data: Dict[str, Any] = request.get_json(force=True)
    try:
        lat = float(data["lat"])
        lon = float(data["lon"])
    except (KeyError, TypeError, ValueError):
        return jsonify({"status": "error", "msg": "lat/lon missing or invalid"}), 400

    controller.start_navigation(lat, lon)
    return jsonify({"status": "ok"})


@app.route("/api/navigation/stop", methods=["POST"])
def api_stop_navigation():
    controller.stop_navigation()
    return jsonify({"status": "ok"})


@app.route("/api/navigation/pause", methods=["POST"])
def api_pause_navigation():
    controller.pause_navigation()
    return jsonify({"status": "ok"})


@app.route("/api/navigation/status", methods=["GET"])
def api_status():
    return jsonify(controller.get_status())


# ---------------------------------------------------------------------------
# Background broadcaster
# ---------------------------------------------------------------------------

def _broadcast_loop():
    while True:
        socketio.emit("status", controller.get_status())
        time.sleep(UPDATE_INTERVAL)


# ---------------------------------------------------------------------------
# Main entry-point
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=5000)
    args = parser.parse_args()

    # Start broadcaster thread
    threading.Thread(target=_broadcast_loop, daemon=True).start()

    # Run Flask-SocketIO server
    # Note: use eventlet or gevent for production; default werkzeug for dev.
    socketio.run(app, host=args.host, port=args.port) 