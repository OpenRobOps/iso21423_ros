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

import math

from iso21423_ros.ccs import CcsFrame


def test_identity_frame_roundtrip():
    f = CcsFrame("c", [])
    assert f.to_ccs(1, 2) == (1, 2) and f.to_map(1, 2) == (1, 2)
    assert f.location_point(1, 2) == {"ccsId": "c", "x": 1, "y": 2, "z": 0}
    assert f.yaw_to_ccs(0.5) == 0.5 and f.yaw_to_map(0.5) == 0.5


def test_fitted_frame_rotates_and_translates():
    # ccs = Rot90(map) + (5, -2)
    pts = [((0, 0), (5, -2)), ((1, 0), (5, -1)), ((0, 1), (4, -2)), ((2, 3), (2, 0))]
    f = CcsFrame("c", pts)
    x, y = f.to_ccs(2, 1)
    assert (round(x, 9), round(y, 9)) == (4, 0)
    x, y = f.to_map(4, 0)
    assert (round(x, 9), round(y, 9)) == (2, 1)
    assert math.isclose(f.yaw_to_ccs(0.0), math.pi / 2)
    assert math.isclose(f.yaw_to_map(f.yaw_to_ccs(0.3)), 0.3)
