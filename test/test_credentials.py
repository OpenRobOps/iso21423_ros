import json
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from iso21423_ros import credentials
from iso21423_ros.config import from_params

UUID = "5f8c1e2a-6b3d-4a9f-8e11-2c7d9a4b1f00"


def test_static_credentials_read_password_file(tmp_path):
    pw = tmp_path / "pw"
    pw.write_text("s3cret\n")
    cfg = from_params({"entity_uuid": UUID, "broker.url": "mqtt://b:1883", "broker.username": "u",
                       "broker.password_file": str(pw), "broker.tls": True})
    c = credentials.resolve(cfg)
    assert (c.url, c.username, c.password, c.tls) == ("mqtt://b:1883", "u", "s3cret", True)


def test_static_without_password_file():
    cfg = from_params({"entity_uuid": UUID, "broker.url": "mqtt://b:1883"})
    assert credentials.resolve(cfg).password == ""


def test_oro_provider_retries_until_200(tmp_path, logcap):
    seen, codes = [], [403, 500, 200]

    class H(BaseHTTPRequestHandler):
        def do_POST(self):
            body = self.rfile.read(int(self.headers["Content-Length"]))
            seen.append((self.path, json.loads(body), self.headers["Content-Type"]))
            code = codes.pop(0)
            self.send_response(code)
            self.end_headers()
            self.wfile.write(json.dumps({"hostname": "broker.example", "port": 1883,
                                         "protocol": "mqtt://", "username": "gen-u",
                                         "password": "gen-pw"}).encode() if code == 200 else b"nope")

        def log_message(self, *a):
            pass

    srv = HTTPServer(("127.0.0.1", 0), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    key = tmp_path / "key"
    key.write_text("  API-KEY-123 \n")
    cfg = from_params({"entity_uuid": UUID, "credentials_provider": "openrobops",
                       "oro.url": f"http://127.0.0.1:{srv.server_port}", "oro.api_key_file": str(key)})
    sleeps = []
    try:
        c = credentials.resolve(cfg, sleep=sleeps.append)
    finally:
        srv.shutdown()
    assert (c.url, c.username, c.password, c.tls) == ("mqtt://broker.example:1883", "gen-u", "gen-pw", False)
    assert sleeps == [10, 10]
    assert len(seen) == 3
    assert seen[0] == ("/iso_mqtt_config", {"apiKey": "API-KEY-123", "entityUuid": UUID},
                       "application/json")
    assert "not admitted" in logcap.text
    assert "API-KEY-123" not in logcap.text and "gen-pw" not in logcap.text


def test_oro_unreachable_retries(tmp_path):
    key = tmp_path / "key"
    key.write_text("k")
    cfg = from_params({"entity_uuid": UUID, "credentials_provider": "openrobops",
                       "oro.url": "http://127.0.0.1:9", "oro.api_key_file": str(key)})

    class Stop(Exception):
        pass

    def sleep(s):
        raise Stop

    with pytest.raises(Stop):
        credentials.resolve(cfg, sleep=sleep)
