import pytest

from iso21423_ros.hooks import DEFAULT_HANDLERS, DEFAULT_SOURCES, load_class


def test_load_class_errors_name_the_key():
    with pytest.raises(ValueError, match=r"^handlers.move.class: cannot load 'nope:X'"):
        load_class("nope:X", "handlers.move.class")


def test_load_class_ok():
    assert load_class("collections:OrderedDict", "k").__name__ == "OrderedDict"


def test_bad_spec_and_missing_attr():
    for spec in ("no_colon", "collections:Nope"):
        with pytest.raises(ValueError, match=r"^k: cannot load"):
            load_class(spec, "k")


def test_defaults():
    assert set(DEFAULT_SOURCES) == {"odometry", "batteryStatus", "globalPlan", "localTrajectory"}
    assert set(DEFAULT_HANDLERS) == {"move", "dock", "undock", "pauseImr", "resumeImr"}
