import pytest

from iso21423_ros.config import from_params

CCS = "0b6f2a3e-1111-4222-8333-444455556666"
UUID = "7B1A9C3E-1111-4222-8333-444455556666"


def test_flat_params_build_config():
    c = from_params({"entity_uuid": UUID, "ccs.id": CCS,
                     "identity.footprint": [0.3, 0.2, 0.3, -0.2, -0.3, -0.2],
                     "ccs.reference_points": [0, 0, 1, 1, 10, 0, 11, 1, 0, 10, 1, 11],
                     "sources.odometry.rate_hz": 2.0, "handlers.move.action": "/navigate_to_pose"})
    assert c.entity_uuid == "7b1a9c3e-1111-4222-8333-444455556666"
    assert c.identity.footprint == [(0.3, 0.2), (0.3, -0.2), (-0.3, -0.2)]
    assert c.ccs.reference_points[1] == ((10, 0), (11, 1))
    assert c.sources == {"odometry": {"rate_hz": 2.0}} and c.handlers["move"]["action"] == "/navigate_to_pose"
    assert c.execution_policy == "preempt" and c.mode == "MODE_AUTO"
    assert c.base_frame == "base_link" and c.ccs.map_frame == "map" and c.credentials_provider == ""


def test_missing_entity_uuid():
    with pytest.raises(ValueError, match=r"^entity_uuid:"):
        from_params({"ccs.id": CCS})


def test_bad_entity_uuid():
    with pytest.raises(ValueError, match=r"^entity_uuid:"):
        from_params({"entity_uuid": "nope"})


def test_odd_footprint_rejected():
    with pytest.raises(ValueError, match=r"^identity.footprint:"):
        from_params({"entity_uuid": UUID, "identity.footprint": [1.0, 2.0, 3.0]})


def test_reference_points_not_multiple_of_4():
    with pytest.raises(ValueError, match=r"^ccs.reference_points:"):
        from_params({"entity_uuid": UUID, "ccs.reference_points": [0, 0, 1, 1, 2, 2]})


def test_unknown_policy_rejected():
    with pytest.raises(ValueError, match=r"^execution_policy:"):
        from_params({"entity_uuid": UUID, "execution_policy": "yolo"})


def test_unknown_credentials_provider_rejected():
    with pytest.raises(ValueError, match=r"^credentials_provider:"):
        from_params({"entity_uuid": UUID, "credentials_provider": "x"})
