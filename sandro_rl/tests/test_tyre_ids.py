"""The tyres found by name in a model with geoms added. With the leg boxes or the CAD hulls
the description's geom ids point at other geoms: until 2026-11-22 the fitted plant's tyre
torsion (--sim tyre_torsion) went to two of those and the tyres kept theirs, in every run
with the boxes or the hulls (the stand-up runs, the one-wheel stand with --boxes)."""

from sandro_rl import evaluate, terrain
from sandro_rl.robot import RobotSpec


def test_the_tyres_are_found_in_models_with_boxes_and_hulls():
    spec = RobotSpec.load()
    names = [spec.model.geom(int(g)).name for g in spec.tyre_geoms]
    assert evaluate.tyre_ids(spec, spec.model) == [int(g) for g in spec.tyre_geoms]
    for hulls in (None, 8):
        m = terrain.build(spec, {}, leg_boxes=True, leg_hulls=hulls)
        ids = evaluate.tyre_ids(spec, m)
        assert [m.geom(i).name for i in ids] == names
        assert ids != [int(g) for g in spec.tyre_geoms]      # why the lookup is by name
