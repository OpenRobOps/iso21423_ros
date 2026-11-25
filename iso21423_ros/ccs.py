"""CCS frame: map-frame <-> ISO CCS rigid transform fitted from reference points."""
from iso21423 import RigidTransform2D, apply_transform, fit_transform, invert_transform, transform_yaw


class CcsFrame:
    def __init__(self, ccs_id, reference_points):
        self.ccs_id = ccs_id
        if len(reference_points) < 3:  # identity frame (map coordinates ARE CCS coordinates)
            self._fwd = RigidTransform2D(0.0, 0.0, 0.0)
        else:
            self._fwd = fit_transform([m for m, _ in reference_points], [c for _, c in reference_points])
        self._inv = invert_transform(self._fwd)

    def to_ccs(self, x, y):
        return apply_transform(self._fwd, (x, y))

    def yaw_to_ccs(self, yaw):
        return transform_yaw(self._fwd, yaw)

    def to_map(self, x, y):
        return apply_transform(self._inv, (x, y))

    def yaw_to_map(self, yaw):
        return transform_yaw(self._inv, yaw)

    def location_point(self, x, y):
        cx, cy = self.to_ccs(x, y)
        return {"ccsId": self.ccs_id, "x": cx, "y": cy, "z": 0}
