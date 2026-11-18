"""The terrain profile observation (configs/agile_perceptive.yaml) on the CPU, through MJX's
JAX backend: the layout and its mirror, the heights against the terrain, the map's known
flags, the interface's frame, and a blind checkpoint resumed into it."""

import jax
import jax.numpy as jp
import numpy as np
import pytest

from sandro_rl import checkpoint, config as config_mod, train
from sandro_rl.interface import (PROFILE_SCALE, PolicyInterface, profile_known, profile_observation,
                                 profile_travel)
from sandro_rl.robot import RobotSpec

OVERRIDES = ["env.impl=jax", "env.graph_mode=null"]
ROOT = config_mod.CONFIG_DIR.parent


@pytest.fixture(scope="module")
def cfg():
    return config_mod.load("agile_perceptive.yaml", OVERRIDES)


@pytest.fixture(scope="module")
def env(cfg):
    from sandro_rl.env import SandroEnv
    return SandroEnv(cfg)


@pytest.fixture(scope="module")
def states(env):
    keys = jax.random.split(jax.random.key(0), 16)
    return jax.jit(jax.vmap(lambda k, p: env.reset(env.mjx_model, k, p)))(keys, jp.zeros((16, 8)))


def test_layout_and_mirror(env):
    n = env.n_profile // 4
    assert n == 8 and env.actor_layout.size == 26 + 4 * n
    assert env.actor_layout.names[-1] == "profile"
    rng = np.random.default_rng(0)
    obs = rng.normal(size=env.actor_obs_size)
    assert np.allclose(env.mirror_actor_obs(env.mirror_actor_obs(obs)), obs)
    critic = rng.normal(size=env.critic_obs_size)
    assert np.allclose(env.mirror_critic_obs(env.mirror_critic_obs(critic)), critic)
    # The mirror swaps the tracks, heights and flags alike.
    frame = np.zeros(env.actor_layout.size)
    frame[26:26 + n] = 1.0                      # right track heights
    frame[26 + 3 * n:] = 2.0                    # left track flags
    mirrored = env.mirror_actor_obs(np.tile(frame, env.history))[: env.actor_layout.size]
    assert np.all(mirrored[26 + n:26 + 2 * n] == 1.0) and np.all(mirrored[26 + 2 * n:26 + 3 * n] == 2.0)
    assert np.all(mirrored[26:26 + n] == 0.0)


def test_heights_follow_the_terrain_right_track_first(env, states):
    """The profile's heights are the terrain's under the points ahead of each wheel, above
    the floor under that wheel, and the first track is the right wheel's."""
    for i in (0, 5, 11):
        data = jax.tree.map(lambda x: x[i], states.data)
        heights = np.asarray(env._profile_heights(data)).reshape(2, -1)
        _, r_body = env._body_frame(data)
        r_body = np.asarray(r_body)
        heading = np.array([r_body[0, 0], r_body[1, 0], 0.0])
        heading /= np.linalg.norm(heading)
        anchors = np.asarray(data.xanchor)[np.asarray(env.spec.wheel_joint_ids)]
        left = np.array([-heading[1], heading[0], 0.0])
        assert (anchors[0] - anchors[1]) @ left < 0          # the first wheel is on the right
        for w in range(2):
            points = anchors[w] + np.asarray(env.profile_distances)[:, None] * heading
            ground = np.asarray(env.terrain.ground_height(jp.asarray(np.vstack([anchors[w], points])), data))
            np.testing.assert_allclose(heights[w], ground[1:] - ground[0], atol=1e-5)


def test_known_floor_grows_with_the_distance_driven():
    d = np.array([0.10, 0.20, 0.30, 0.45, 0.60, 0.80, 1.05, 1.35])
    np.testing.assert_array_equal(profile_known(d, 0.0, 0.5, 1.2), [0, 0, 0, 0, 1, 1, 1, 0])
    np.testing.assert_array_equal(profile_known(d, 0.25, 0.5, 1.2), [0, 0, 1, 1, 1, 1, 1, 0])
    np.testing.assert_array_equal(profile_known(d, 2.0, 0.5, 2.5), np.ones(8))
    cfg = {"forget_m_per_rad": 0.4, "forget_m_per_s": 0.08, "travel_max_m": 2.0}
    travel = 0.0
    for _ in range(100):                                      # 1 s at 0.5 m/s
        travel = profile_travel(travel, 0.5, 0.0, 0.01, cfg)
    assert travel == pytest.approx(0.42, abs=1e-6)
    for _ in range(100):                                      # then a turn on the spot of 2 rad
        travel = profile_travel(travel, 0.0, 2.0, 0.01, cfg)
    assert travel == 0.0
    assert float(profile_travel(jp.float32(1.99), jp.float32(1.0), jp.float32(0.0), 0.1, cfg, jp)) == 2.0


def test_observation_is_the_nodes_message():
    heights = np.array([0.03, -0.2, 0.4, 0.01] * 4)
    known = np.array([1, 1, 1, 0] * 4)
    obs = profile_observation(heights, known)
    np.testing.assert_allclose(obs[:4], np.array([0.03, -0.15, 0.25, 0.0]) * PROFILE_SCALE)
    np.testing.assert_array_equal(obs[16:], known)


def test_blind_episodes_and_unseen_floor_read_unknown(env, states):
    n = env.n_profile // 4
    profile = np.asarray(states.obs["actor"])[:, 26:26 + 4 * n]
    blind = np.asarray(states.info["profile_blind"]) > 0
    assert blind.any() and not blind.all()
    assert np.all(profile[blind] == 0.0)
    fresh = (~blind) & (np.asarray(states.info["profile_travel"]) == 0.0)
    assert fresh.any()
    near = np.asarray(states.info["profile_near"])
    d = np.asarray(env.profile_distances)
    for i in np.flatnonzero(fresh):
        flags = profile[i, 2 * n:3 * n]
        assert np.all(flags[d < near[i]] == 0.0)
        assert np.all(profile[i, :n][flags == 0.0] == 0.0)


def test_interface_frame_matches_environment(env, cfg):
    iface = PolicyInterface(cfg, RobotSpec.load())
    assert iface.n_profile == env.n_profile
    rng = np.random.default_rng(3)
    q = rng.normal(size=4)
    q /= np.linalg.norm(q)
    sensed = np.concatenate([q, rng.normal(size=3), rng.normal(size=3), np.asarray(env.stand_q) + 0.1,
                             rng.normal(size=6)])
    profile = profile_observation(rng.normal(0.0, 0.03, 16), (rng.uniform(size=16) > 0.3) * 1.0)
    info = {"params": {"mount": jp.eye(3), "gyro_bias": jp.zeros(3), "vel_gain": jp.ones(6)},
            "command_ref": jp.asarray([0.8, 0.0, 0.16, 0.1]), "last_action": jp.zeros(6),
            "profile_obs": jp.asarray(profile, jp.float32)}
    want, _ = env._actor_frame(jp.asarray(sensed, jp.float32), info)
    got = iface.frame(sensed[0:4], sensed[4:7], sensed[7:10], sensed[10:16], sensed[16:22], np.zeros(6),
                      [0.8, 0.0, 0.16, 0.1], profile=profile)
    np.testing.assert_allclose(got, np.asarray(want), rtol=1e-5, atol=1e-5)


def test_a_blind_checkpoint_resumes_with_zero_weight_on_the_profile(env):
    payload, _ = checkpoint.load(ROOT / "policies/agile_terrain_v5")
    wide = train.widen_for_profile(payload, env)
    kernel = np.asarray(wide["params"]["actor"]["params"]["Dense_0"]["kernel"])
    old = np.asarray(payload["params"]["actor"]["params"]["Dense_0"]["kernel"])
    keep, _ = env.blind_indices()
    assert kernel.shape[0] == env.actor_obs_size and len(keep) == old.shape[0]
    np.testing.assert_array_equal(kernel[keep], old)
    rest = np.setdiff1d(np.arange(env.actor_obs_size), keep)
    assert np.all(kernel[rest] == 0.0)
    # Heights are normalised to 2 cm, flags to a half: the resumed count is too large to learn it.
    frame = np.asarray(wide["actor_norm"]["var"])[:58]
    np.testing.assert_allclose(frame[26:42], 0.04, rtol=1e-6)
    np.testing.assert_allclose(frame[42:58], 0.25)
    np.testing.assert_allclose(np.asarray(wide["actor_norm"]["mean"])[42:58], 0.5)
    np.testing.assert_array_equal(np.asarray(wide["actor_norm"]["var"])[keep], np.asarray(payload["actor_norm"]["var"]))
    assert np.asarray(wide["critic_norm"]["mean"]).size == env.critic_obs_size


def test_floor_behind_an_obstacle_is_hidden():
    """A 10 cm box 0.5-0.65 m ahead hides the floor behind it from a camera 0.33 m up; its own
    top and the floor before it stay in view, and flat floor hides nothing."""
    from sandro_rl.interface import profile_occluded
    d = np.array([0.10, 0.20, 0.30, 0.45, 0.60, 0.80, 1.05, 1.35])
    line = np.linspace(0.14, 1.34, 25)
    box = lambda x: np.where((x >= 0.5) & (x <= 0.65), 0.10, 0.0)
    hidden = profile_occluded(d, box(d), line, box(line), 0.33, 0.1)
    # From the camera 0.1 m ahead of the axle the shadow reaches 0.1 + 0.54 * 0.33 / 0.23 = 0.87 m.
    np.testing.assert_array_equal(hidden, [0, 0, 0, 0, 0, 1, 0, 0])
    assert not profile_occluded(d, np.zeros(8), line, np.zeros(25), 0.33, 0.1).any()
    assert np.asarray(profile_occluded(jp.asarray(d), jp.asarray(box(d)), jp.asarray(line), jp.asarray(box(line)),
                                       0.33, 0.1, jp)).tolist() == hidden.tolist()


def test_an_obstacles_shadow_reads_as_the_obstacle():
    """sandro_perception fills the hidden floor behind a tall face with the face's height: a
    0.6 m wall from 0.5 m on reads 0.25 (the face's average), known, at every sample from its
    face to 0.6 m behind it; the shadow of a 3 cm plank stays unknown."""
    from sandro_rl.interface import profile_occluded, profile_shadow
    d = np.array([0.10, 0.20, 0.30, 0.45, 0.60, 0.80, 1.05, 1.35])
    line = np.linspace(0.14, 1.34, 25)
    wall = lambda x: np.where((x >= 0.5) & (x <= 0.7), 0.6, 0.0)
    hidden = profile_occluded(d, wall(d), line, wall(line), 0.33, 0.1)
    heights, filled = profile_shadow(d, wall(d), hidden, line, wall(line), 0.25)
    np.testing.assert_allclose(heights, [0, 0, 0, 0, 0.25, 0.25, 0.25, 0.0])
    np.testing.assert_array_equal(hidden * (1 - filled), [0, 0, 0, 0, 0, 0, 0, 1])
    plank = lambda x: np.where((x >= 0.5) & (x <= 0.65), 0.03, 0.0)
    hidden = profile_occluded(d, plank(d), line, plank(line), 0.33, 0.1)
    assert not profile_shadow(d, plank(d), hidden, line, plank(line), 0.25)[1].any()
