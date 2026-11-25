"""Broker credentials: static (broker.* params) or fetched from OpenRobOps (/iso_mqtt_config)."""
import json
import logging
import time
import urllib.error
import urllib.request
from dataclasses import dataclass

log = logging.getLogger(__name__)

RETRY_S = 10


@dataclass
class BrokerCredentials:
    url: str
    username: str
    password: str
    tls: bool

    def __repr__(self):  # never leak the password into logs
        return f"BrokerCredentials(url={self.url!r}, username={self.username!r}, tls={self.tls})"


def _read(path):
    with open(path) as f:
        return f.read().strip()


def _fetch_oro(cfg, sleep):
    endpoint = f"{cfg.oro.url.rstrip('/')}/iso_mqtt_config"
    body = json.dumps({"apiKey": _read(cfg.oro.api_key_file), "entityUuid": cfg.entity_uuid}).encode()
    while True:
        req = urllib.request.Request(endpoint, data=body, method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return json.load(r)
        except urllib.error.HTTPError as e:
            hint = " (robot not admitted: apply the ISO robot config in ORO)" if e.code == 403 else ""
            log.warning("iso_mqtt_config -> %s%s; retrying in %ss", e.code, hint, RETRY_S)
        except (OSError, ValueError) as e:  # unreachable, timeout, bad JSON
            log.warning("iso_mqtt_config failed: %s; retrying in %ss", e, RETRY_S)
        sleep(RETRY_S)


def resolve(cfg, sleep=time.sleep) -> BrokerCredentials:
    """May block (retrying every 10 s) with the openrobops provider. Call from the main thread."""
    if cfg.credentials_provider == "openrobops":
        c = _fetch_oro(cfg, sleep)
        # ORO's `protocol` already contains "://"
        url = f"{c['protocol']}{c['hostname']}:{c['port']}"
        log.info("broker %s as %s", url, c["username"])
        return BrokerCredentials(url, c["username"], c["password"], False)
    pw = _read(cfg.broker.password_file) if cfg.broker.password_file else ""
    return BrokerCredentials(cfg.broker.url, cfg.broker.username, pw, cfg.broker.tls)
