"""Reward-only regression tests: no server, shared memory, models or training.

Run from the repository root: python -m unittest discover -s tests -v
"""
from pathlib import Path
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "R2DRL"))
from robocup2d.agents import Agents
from robocup2d.env import Robocup2dEnv
from robocup2d.protocols import P


class CoachFixture:
    def __init__(self):
        self.goal_flag = 0
        self.position = (0.02, 0.0)
        self.mode = P.coach.GAME_MODE_PLAY_ON

    def goal(self):
        return self.goal_flag

    def ball(self, copy=False):
        return self.position

    def clear_goal_flag(self):
        self.goal_flag = 0

    def gamemode(self):
        return self.mode


def reward_fixture(n=11, use_epv=True):
    # Intentionally bypass constructors: they would launch real processes.
    config = SimpleNamespace(
        n1=n, n2=n, useMaxEpv=use_epv, epv_progress_scale=2.0,
        epv_progress_reward=0.0, step_trainer_noop=False, playon_timeout=1.0,
        terminate_on_goal=False, terminate_on_ball_out=False,
        terminate_on_ball_left_half=False, terminate_on_possession_loss=False,
        reward_on_timeout=0.0, reward_on_draw=0.0,
    )
    agents = Agents.__new__(Agents)
    agents.config = config
    agents.coach = CoachFixture()
    # Here x is a synthetic EPV value, not a physical ball coordinate.
    agents.get_ball_position_epv = lambda position=None: float(
        (agents.coach.position if position is None else position)[0]
    )
    agents._epv_max_value = 1.0
    agents.true_kickable = True
    agents.opponent_true_kickable = False
    agents.ball_owner_team = 1
    agents.set_agent_mask = Mock()
    agents.write_actions = Mock()
    agents.reset_episode_epv()

    env = Robocup2dEnv.__new__(Robocup2dEnv)
    env.config = config
    env.agents = agents
    env.episode_steps = 0
    env.episode_limit = 3000
    env.turn_count = 1
    env.score = [0, 0]
    env._epv_rebase_pending = False
    env._need_restart = False
    env._restart_total = 0
    env._consecutive_restart_count = 0
    env._recent_restart_count = lambda: 0
    env._wait_all_ready_with_timeout = Mock(return_value=True)
    env._is_ball_out_of_bounds = lambda x, y: False
    env._is_ball_left_half = lambda x: False
    env._cache_transition_payload = Mock()
    return env


class GoalResetTests(unittest.TestCase):
    def transition(self, env, value, goal=0, kickable=True, mode=2):
        env.agents.coach.position = (value, 0.0)
        env.agents.coach.goal_flag = goal
        env.agents.coach.mode = mode
        env.agents.true_kickable = kickable
        return env.step([])

    def test_episode_reset_initializes_both_baselines(self):
        env = reward_fixture()
        self.assertEqual(env.agents.initial_episode_epv, 0.02)
        self.assertEqual(env.agents.initial_segment_epv, 0.02)
        self.assertEqual(env.agents.max_episode_epv, 0.02)

    def test_ordinary_positive_increments_and_no_repeat_reward(self):
        env = reward_fixture()
        for value, expected in [(0.05, 0.06), (0.04, 0), (0.05, 0), (0.07, 0.04)]:
            reward, done, info = self.transition(env, value)
            self.assertAlmostEqual(reward, expected)
            self.assertFalse(done)
            self.assertEqual(info["epv_rebased_after_goal"], 0)

    def test_scored_goal_settles_old_max_then_rebases(self):
        env = reward_fixture()
        self.transition(env, 0.4)
        reward, done, info = self.transition(env, 0.1, goal=1)
        self.assertAlmostEqual(reward, 1.0 + (1.0 - 0.4) * 2)
        self.assertAlmostEqual(info["epv_goal_completion_reward"], 1.2)
        self.assertEqual(info["epv_step_reward"], 0)
        self.assertEqual(info["epv_rebased_after_goal"], 1)
        self.assertEqual(info["max_episode_epv"], 0.1)
        self.assertEqual(info["initial_episode_epv"], 0.02)
        self.assertEqual(info["initial_segment_epv"], 0.1)
        self.assertEqual(info["max_epv_improvement"], 0)
        self.assertEqual(env.score, [1, 0])
        self.assertFalse(done)
        self.assertEqual(env.agents.coach.goal(), 0)
        self.assertAlmostEqual(self.transition(env, 0.15)[0], 0.1)

    def test_conceded_goal_cannot_reward_restart_jump(self):
        env = reward_fixture()
        self.transition(env, 0.05)
        reward, done, info = self.transition(env, 0.2, goal=-1)
        self.assertEqual(reward, -1)
        self.assertEqual(info["epv_step_reward"], 0)
        self.assertEqual(info["epv_goal_completion_reward"], 0)
        self.assertEqual(info["max_episode_epv"], 0.2)
        self.assertEqual(env.score, [0, 1])
        self.assertFalse(done)
        self.assertAlmostEqual(self.transition(env, 0.3)[0], 0.2)

    def test_second_goal_has_its_own_completion_reward(self):
        env = reward_fixture()
        self.transition(env, 0.4)
        self.transition(env, 0.1, goal=1)
        self.transition(env, 0.3)
        reward, _, info = self.transition(env, 0.1, goal=1)
        self.assertAlmostEqual(reward, 2.4)
        self.assertAlmostEqual(info["epv_goal_completion_reward"], 1.4)
        self.assertEqual(env.score, [2, 0])

    def test_non_play_on_goal_waits_for_play_on_rebase(self):
        env = reward_fixture()
        self.transition(env, 0.6)
        self.transition(env, 0.0, goal=-1, mode=8)
        self.assertTrue(env._epv_rebase_pending)
        reward, _, info = self.transition(env, 0.03, mode=3)
        self.assertEqual(reward, 0)
        self.assertEqual(info["epv_rebased_after_goal"], 0)
        reward, _, info = self.transition(env, 0.04, mode=2, kickable=False)
        self.assertEqual(reward, 0)
        self.assertEqual(info["max_episode_epv"], 0.04)
        self.assertEqual(info["epv_rebased_after_goal"], 1)
        self.assertFalse(env._epv_rebase_pending)
        self.assertAlmostEqual(self.transition(env, 0.05)[0], 0.02)

    def test_possession_change_does_not_reset_max(self):
        env = reward_fixture()
        self.transition(env, 0.5)
        reward, _, info = self.transition(env, 0.03, kickable=False)
        self.assertEqual(reward, 0)
        self.assertEqual(info["max_episode_epv"], 0.5)
        self.assertEqual(self.transition(env, 0.4)[0], 0)

    def test_epv_disabled_keeps_base_goal_rewards(self):
        for goal in (-1, 1):
            with self.subTest(goal=goal):
                env = reward_fixture(use_epv=False)
                reward, _, info = self.transition(env, 0.1, goal=goal)
                self.assertEqual(reward, goal)
                self.assertEqual(info["epv_goal_completion_reward"], 0)
                self.assertEqual(info["max_episode_epv"], 0.1)

    def test_local_scenario_goal_retains_previous_semantics(self):
        env = reward_fixture(n=3)
        reward, _, info = self.transition(env, 0.1, goal=1)
        self.assertAlmostEqual(reward, 1 + (1 - 0.02) * 2)
        self.assertAlmostEqual(info["epv_step_reward"], 0.16)
        self.assertEqual(info["max_episode_epv"], 1)
        self.assertEqual(info["initial_segment_epv"], 0.02)
        self.assertEqual(info["epv_rebased_after_goal"], 0)

    def test_terminate_on_goal_is_unchanged(self):
        for goal, reason in ((1, "goal_scored"), (-1, "goal_conceded")):
            with self.subTest(goal=goal):
                env = reward_fixture()
                env.config.terminate_on_goal = True
                _, done, info = self.transition(env, 0.1, goal=goal)
                self.assertTrue(done)
                self.assertEqual(info["terminal_reason"], reason)
                self.assertEqual(info["max_episode_epv"], 0.1)

    def test_timeout_on_goal_keeps_reward_and_score(self):
        env = reward_fixture()
        env.episode_limit = 1
        reward, done, info = self.transition(env, 0.1, goal=1)
        self.assertTrue(done)
        self.assertEqual(info["terminal_reason"], "timeout")
        self.assertAlmostEqual(reward, 1 + (1 - 0.02) * 2)
        self.assertEqual(env.score, [1, 0])

    def test_fixed_progress_reward_still_works_after_goal(self):
        env = reward_fixture()
        env.config.epv_progress_reward = 0.25
        self.assertEqual(self.transition(env, 0.5)[0], 0.25)
        self.transition(env, 0.1, goal=-1)
        self.assertEqual(self.transition(env, 0.2)[0], 0.25)

    def test_new_episode_discards_old_segment_max(self):
        env = reward_fixture()
        self.transition(env, 0.9)
        env.agents.coach.position = (0.03, 0)
        env.agents.reset_episode_epv()
        self.assertEqual(env.agents.initial_episode_epv, 0.03)
        self.assertEqual(env.agents.initial_segment_epv, 0.03)
        self.assertEqual(env.agents.max_episode_epv, 0.03)


if __name__ == "__main__":
    unittest.main()
