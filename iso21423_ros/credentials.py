"""Broker credentials: static (broker.* params) or fetched from OpenRobOps (/iso_mqtt_config)."""
import http.client
import json
import logging
import threading
import urllib.error
import urllib.request
from dataclasses import dataclass

from iso21423 import Iso21423Error

log = logging.getLogger(__name__)

RETRY_S = 10


@dataclass
class BrokerCredentials:
    url: str
    username: str
    password: str
    tls: dict | None  # as Client.connect(tls=): None = off, {} = default TLS

    def __repr__(self):  # never leak the password into logs
        return f"BrokerCredentials(url={self.url!r}, username={self.username!r}, tls={self.tls!r})"


def _read(path):
    with open(path) as f:
        return f.read().strip()


def _fetch_oro(cfg, stop, sleep):
    endpoint = f"{cfg.oro.url.rstrip('/')}/iso_mqtt_config"
    body = json.dumps({"apiKey": _read(cfg.oro.api_key_file), "entityUuid": cfg.entity_uuid}).encode()
    while True:
        if stop.is_set():
            raise Iso21423Error("credential fetch stopped")
        req = urllib.request.Request(endpoint, data=body, method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                c = json.load(r)
            # validate the shape; never log response content (it carries the password)
            for k in ("protocol", "hostname", "port", "username", "password"):
                c[k]
            return c
        except urllib.error.HTTPError as e:
            hint = " (robot not admitted: apply the ISO robot config in ORO)" if e.code == 403 else ""
            log.warning("iso_mqtt_config -> %s%s; retrying in %ss", e.code, hint, RETRY_S)
        except (OSError, ValueError, http.client.HTTPException, KeyError, TypeError) as e:
            log.warning("iso_mqtt_config failed (%s); retrying in %ss", type(e).__name__, RETRY_S)
        if sleep:
            sleep(RETRY_S)
        else:
            stop.wait(RETRY_S)


def resolve(cfg, stop: threading.Event | None = None, sleep=None) -> BrokerCredentials:
    """May block (retrying every 10 s) with the openrobops provider; set `stop` to abort it
    (raises Iso21423Error). `sleep` is injectable for tests."""
    if cfg.credentials_provider == "openrobops":
        c = _fetch_oro(cfg, stop or threading.Event(), sleep)
        # ORO's `protocol` already contains "://" (and mqtts:// enables TLS by itself)
        url = f"{c['protocol']}{c['hostname']}:{c['port']}"
        log.info("broker %s as %s", url, c["username"])
        return BrokerCredentials(url, c["username"], c["password"], None)
    pw = _read(cfg.broker.password_file) if cfg.broker.password_file else ""
    return BrokerCredentials(cfg.broker.url, cfg.broker.username, pw, {} if cfg.broker.tls else None)
