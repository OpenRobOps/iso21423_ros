"""Connector configuration, built from the flat ROS parameter map {"a.b.c": value}."""
import uuid
from dataclasses import dataclass, field

POLICIES = ("abort_new", "queue_after", "queue_replace", "parallel", "priority", "preempt")
PROVIDERS = ("", "openrobops")


@dataclass
class IdentityConfig:
    model: str = ""
    serial_number: str = ""
    height: float = 0.0
    footprint: list = field(default_factory=list)  # [(x, y), ...]
    footprint_from_nav2: bool = False


@dataclass
class BrokerConfig:
    url: str = ""
    username: str = ""
    password_file: str = ""
    tls: bool = False


@dataclass
class OroConfig:
    url: str = ""
    api_key_file: str = ""


@dataclass
class CcsConfig:
    id: str = ""
    map_frame: str = "map"
    reference_points: list = field(default_factory=list)  # [((map_x, map_y), (ccs_x, ccs_y)), ...]


@dataclass
class ConnectorConfig:
    entity_uuid: str
    manufacturer_name: str = ""
    identity: IdentityConfig = field(default_factory=IdentityConfig)
    broker: BrokerConfig = field(default_factory=BrokerConfig)
    credentials_provider: str = ""
    oro: OroConfig = field(default_factory=OroConfig)
    ccs: CcsConfig = field(default_factory=CcsConfig)
    base_frame: str = "base_link"
    mode: str = "MODE_AUTO"
    execution_policy: str = "preempt"
    sources: dict = field(default_factory=dict)
    handlers: dict = field(default_factory=dict)
    extensions: dict = field(default_factory=dict)


def _tuples(params, key, n):
    """Group a flat numeric list into n-tuples."""
    flat = list(params.get(key, []))
    if len(flat) % n:
        raise ValueError(f"{key}: expected a multiple of {n} numbers, got {len(flat)}")
    return [tuple(float(v) for v in flat[i:i + n]) for i in range(0, len(flat), n)]


def _nested(params, prefix):
    """{"sources.odometry.rate_hz": 2} -> {"odometry": {"rate_hz": 2}}"""
    out = {}
    for k, v in params.items():
        if k.startswith(prefix + "."):
            name, _, rest = k[len(prefix) + 1:].partition(".")
            if rest:
                out.setdefault(name, {})[rest] = v
    return out


def from_params(params: dict) -> ConnectorConfig:
    p = params
    raw = p.get("entity_uuid", "")
    try:
        entity = str(uuid.UUID(str(raw)))
    except ValueError:
        raise ValueError(f"entity_uuid: required, must be a UUID (got {raw!r})") from None

    policy = p.get("execution_policy", "preempt")
    if policy not in POLICIES:
        raise ValueError(f"execution_policy: must be one of {', '.join(POLICIES)} (got {policy!r})")
    provider = p.get("credentials_provider", "")
    if provider not in PROVIDERS:
        raise ValueError(f"credentials_provider: must be '' or 'openrobops' (got {provider!r})")

    footprint = _tuples(p, "identity.footprint", 2)
    ref = _tuples(p, "ccs.reference_points", 4)

    return ConnectorConfig(
        entity_uuid=entity,
        manufacturer_name=p.get("manufacturer_name", ""),
        identity=IdentityConfig(
            model=p.get("identity.model", ""),
            serial_number=p.get("identity.serial_number", ""),
            height=float(p.get("identity.height", 0.0)),
            footprint=footprint,
            footprint_from_nav2=bool(p.get("identity.footprint_from_nav2", False)),
        ),
        broker=BrokerConfig(
            url=p.get("broker.url", ""),
            username=p.get("broker.username", ""),
            password_file=p.get("broker.password_file", ""),
            tls=bool(p.get("broker.tls", False)),
        ),
        credentials_provider=provider,
        oro=OroConfig(url=p.get("oro.url", ""), api_key_file=p.get("oro.api_key_file", "")),
        ccs=CcsConfig(
            id=p.get("ccs.id", ""),
            map_frame=p.get("ccs.map_frame", "map"),
            reference_points=[((r[0], r[1]), (r[2], r[3])) for r in ref],
        ),
        base_frame=p.get("base_frame", "base_link"),
        mode=p.get("mode", "MODE_AUTO"),
        execution_policy=policy,
        sources=_nested(p, "sources"),
        handlers=_nested(p, "handlers"),
        extensions=_nested(p, "extensions"),
    )
