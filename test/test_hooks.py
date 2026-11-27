# Copyright 2026 InOrbit, Inc.
#
#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.

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


def test_import_time_failure_names_the_key(tmp_path, monkeypatch):
    (tmp_path / "boom_mod.py").write_text("raise NameError('boom')\n")
    monkeypatch.syspath_prepend(str(tmp_path))
    with pytest.raises(ValueError, match=r"^k: cannot load 'boom_mod:X'.*boom"):
        load_class("boom_mod:X", "k")


def test_non_class_rejected():
    with pytest.raises(ValueError, match=r"^k: cannot load 'collections:abc'"):
        load_class("collections:abc", "k")
