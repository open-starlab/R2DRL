import unittest
from pathlib import Path
from unittest.mock import patch
from types import SimpleNamespace

from envs.robocup2d.agents import Agents
from envs.robocup2d.env import Robocup2dEnv
from envs.robocup2d.protocols.common import FLAG_DONE, FLAG_IDLE, FLAG_READY
from envs.robocup2d.start_sampler import apply_catalog_launch_profile


class DummyTrainer:
    def __init__(self, flags=FLAG_READY):
        self._flags = tuple(flags)
        self.noop_calls = 0

    def flags(self):
        return self._flags

    def is_idle(self):
        return self._flags in (FLAG_READY, FLAG_DONE)

    def phase(self):
        return 0 if self.is_idle() else 1

    def req_seq(self):
        return 0 if self.is_idle() else 1

    def done_seq(self):
        return 0

    def noop(self):
        self.noop_calls += 1


class PendingIdleTrainer(DummyTrainer):
    def __init__(self, *, req_seq=1, done_seq=0):
        super().__init__(flags=FLAG_IDLE)
        self._req_seq = int(req_seq)
        self._done_seq = int(done_seq)

    def is_idle(self):
        return self._done_seq >= self._req_seq and self.phase() != 1

    def phase(self):
        return 0

    def req_seq(self):
        return self._req_seq

    def done_seq(self):
        return self._done_seq


class AckAfterEmptyActionTrainer(PendingIdleTrainer):
    def __init__(self, player, *, target_seq=1):
        super().__init__(req_seq=target_seq, done_seq=target_seq - 1)
        self.player = player
        self._target_seq = int(target_seq)

    def done_seq(self):
        if self.player.empty_action_calls > 0:
            return self._target_seq
        return self._target_seq - 1


class DummyPlayer:
    def __init__(self, flags=FLAG_READY):
        self._flags = tuple(flags)
        self.default_action_calls = 0
        self.empty_action_calls = 0

    def read_flags(self):
        return self._flags

    def is_playon_ready(self):
        return self.read_flags() == FLAG_READY

    def phase(self):
        return 1 if self.is_playon_ready() else 0

    def obs_seq(self):
        return 0

    def act_seq(self):
        return 0

    def done_seq(self):
        return 0

    def cycle(self):
        return 0

    def take_default_action(self, is_hybrid):
        del is_hybrid
        self.default_action_calls += 1

    def take_empty_action(self, is_hybrid):
        del is_hybrid
        self.empty_action_calls += 1


class SequencePlayer(DummyPlayer):
    def __init__(self, flags_sequence):
        self._flags_sequence = [tuple(flags) for flags in flags_sequence]
        self._idx = 0
        self.default_action_calls = 0
        self.empty_action_calls = 0

    def read_flags(self):
        return self._flags_sequence[min(self._idx, len(self._flags_sequence) - 1)]

    def advance(self):
        if self._idx + 1 < len(self._flags_sequence):
            self._idx += 1

    def cycle(self):
        return self._idx


class DummyCoach:
    def __init__(self, goal=0):
        self._goal = goal
        self.clear_goal_flag_calls = 0

    def goal(self):
        return self._goal

    def clear_goal_flag(self):
        self.clear_goal_flag_calls += 1


class DummyResetAgents:
    def __init__(self, goal=0):
        self.coach = DummyCoach(goal=goal)
        self.reset_default_calls = []
        self.reset_custom_calls = []
        self.set_agent_mask_calls = 0
        self.reset_episode_epv_calls = 0
        self.set_mask_n_calls = []
        self.initial_episode_epv = 0.0
        self.max_episode_epv = 0.0

    def reset_default(self, wait_timeout=None):
        self.reset_default_calls.append(wait_timeout)
        return True

    def reset_custom(self, wait_timeout=None):
        self.reset_custom_calls.append(wait_timeout)
        return True

    def set_mask_n(self, n_control):
        self.set_mask_n_calls.append(int(n_control))
        return True

    def set_agent_mask(self):
        self.set_agent_mask_calls += 1

    def reset_episode_epv(self):
        self.reset_episode_epv_calls += 1


class DummyLog:
    def info(self, *_args, **_kwargs):
        return None


class WaitSignatureTest(unittest.TestCase):
    def _new_agents(self, *, trainer_flags, player_flags):
        agents = Agents.__new__(Agents)
        agents.config = type("Cfg", (), {"n1": len(player_flags), "n2": 0, "team1": "base"})()
        agents.log = None
        agents.freeze_non_controlled = False
        agents.active_right_unums = set()
        agents.agent_mask = [True] * len(player_flags)
        agents.trainer = DummyTrainer(flags=trainer_flags)
        agents.player_list = [DummyPlayer(flags=flags) for flags in player_flags]
        return agents

    def _new_frozen_1v0_agents(self, *, right_flags, active_right=False):
        agents = Agents.__new__(Agents)
        agents.config = type("Cfg", (), {"n1": 1, "n2": 1, "team1": "base"})()
        agents.log = None
        agents.freeze_non_controlled = True
        agents.active_right_unums = {1} if active_right else set()
        agents.agent_mask = [True]
        agents.trainer = DummyTrainer(flags=FLAG_READY)
        agents.player_list = [DummyPlayer(flags=FLAG_READY), DummyPlayer(flags=right_flags)]
        return agents

    def _new_env_for_reset(self, *, turn_count, use_custom_start, goal=0):
        env = Robocup2dEnv.__new__(Robocup2dEnv)
        env.turn_count = int(turn_count)
        env._need_restart = False
        env.runtime = SimpleNamespace(has_live_procs=lambda: True)
        env.config = SimpleNamespace(
            wait_ready_timeout=3.0,
            trainer_ready_timeout_ms=10000.0,
            reset_retries=0,
            use_custom_start=bool(use_custom_start),
        )
        env._last_terminal_reason = "init" if int(turn_count) == 0 else "running"
        env._consecutive_restart_count = 0
        env.last_state = object()
        env.last_obs = object()
        env.last_avail_actions = object()
        env.done = 1
        env.episode_steps = 99
        env.score = [0, 0]
        env.agents = DummyResetAgents(goal=goal)
        env._restart_total = 0
        env._recent_restart_count = lambda: 0
        env._restart_with_backoff = lambda _reason: self.fail("unexpected restart")
        env._cache_transition_payload = lambda: None
        env._sync_control_count = lambda: None
        env._wait_trainer_ready_with_timeout = lambda _timeout, context: True
        env.log = DummyLog()
        return env

    def test_player_read_all_flags_tracks_ready_progress(self):
        agents = self._new_agents(
            trainer_flags=FLAG_READY,
            player_flags=[FLAG_IDLE, FLAG_DONE],
        )

        first = Agents.read_all_flags(agents, include_cycles=True)
        agents.player_list[1]._flags = FLAG_READY
        second = Agents.read_all_flags(agents, include_cycles=True)

        self.assertFalse(first["who"]["player:1"]["ready"])
        self.assertTrue(second["who"]["player:1"]["ready"])

    def test_trainer_read_all_flags_tracks_idle_progress(self):
        agents = self._new_agents(
            trainer_flags=FLAG_IDLE,
            player_flags=[FLAG_READY],
        )

        first = Agents.read_all_flags(agents, include_cycles=True)
        agents.trainer._flags = FLAG_DONE
        second = Agents.read_all_flags(agents, include_cycles=True)

        self.assertFalse(first["trainer"]["idle"])
        self.assertTrue(second["trainer"]["idle"])

    def test_catalog_launch_profile_keeps_frozen_dummy_right_player_for_1v0(self):
        trajectory_path = (
            Path(__file__).resolve().parents[1]
            / "envs"
            / "robocup2d"
            / "trajectories"
            / "scenarioes.npz"
        )
        env_args = {
            "start_catalog_enabled": True,
            "trajectory_path": str(trajectory_path),
            "start_id": "front_goal_1v0",
            "n": 2,
            "init_n": 1,
        }

        profile = apply_catalog_launch_profile(env_args)

        self.assertIsNotNone(profile)
        self.assertEqual(env_args["n"], 1)
        self.assertEqual(env_args["n1"], 1)
        self.assertEqual(env_args["n2"], 1)
        self.assertEqual(env_args["init_n"], 1)
        self.assertEqual(env_args["active_right_unums"], [])
        self.assertTrue(env_args["freeze_non_controlled"])

    def test_wait_all_ready_returns_false_on_timeout_without_ready_entities(self):
        agents = self._new_agents(
            trainer_flags=FLAG_IDLE,
            player_flags=[FLAG_IDLE],
        )

        ticks = iter([0.00, 0.00, 0.06, 0.12, 0.18, 0.24, 0.30, 0.36])
        with patch("envs.robocup2d.agents.time.monotonic", side_effect=lambda: next(ticks)):
            with patch("envs.robocup2d.agents.time.sleep", return_value=None):
                ok = Agents.wait_all_ready(
                    agents,
                    timeout=0.30,
                    poll=0.0,
                    deadlock_window=0.10,
                    context="unit-timeout",
                )

        self.assertFalse(ok)
        self.assertEqual(agents.trainer.noop_calls, 0)
        self.assertEqual(agents.player_list[0].default_action_calls, 0)

    def test_wait_all_ready_returns_true_when_everyone_ready(self):
        agents = self._new_agents(
            trainer_flags=FLAG_READY,
            player_flags=[FLAG_READY],
        )

        ok = Agents.wait_all_ready(
            agents,
            timeout=0.30,
            poll=0.0,
            deadlock_window=0.10,
            context="unit-ready",
        )

        self.assertTrue(ok)

    def test_wait_all_ready_ignores_frozen_inactive_right_player(self):
        agents = self._new_frozen_1v0_agents(
            right_flags=FLAG_IDLE,
            active_right=False,
        )

        ok = Agents.wait_all_ready(
            agents,
            timeout=0.30,
            poll=0.0,
            deadlock_window=0.10,
            context="unit-frozen-right",
        )

        self.assertTrue(ok)
        self.assertEqual(agents.player_list[0].default_action_calls, 0)
        self.assertEqual(agents.player_list[1].default_action_calls, 0)
        self.assertEqual(agents.player_list[0].empty_action_calls, 0)
        self.assertEqual(agents.player_list[1].empty_action_calls, 0)

    def test_wait_all_ready_pumps_ignored_frozen_right_player(self):
        agents = self._new_frozen_1v0_agents(
            right_flags=FLAG_READY,
            active_right=False,
        )
        agents.player_list[0]._flags = FLAG_IDLE

        def fake_sleep(_seconds):
            agents.player_list[0]._flags = FLAG_READY

        with patch("envs.robocup2d.agents.time.sleep", side_effect=fake_sleep):
            ok = Agents.wait_all_ready(
                agents,
                timeout=0.30,
                poll=0.0,
                deadlock_window=0.10,
                context="unit-frozen-right-pump",
            )

        self.assertTrue(ok)
        self.assertEqual(agents.player_list[1].empty_action_calls, 1)

    def test_wait_all_ready_still_requires_active_right_player(self):
        agents = self._new_frozen_1v0_agents(
            right_flags=FLAG_IDLE,
            active_right=True,
        )

        ok = Agents.wait_all_ready(
            agents,
            timeout=0.0,
            poll=0.0,
            deadlock_window=0.10,
            context="unit-active-right",
        )

        self.assertFalse(ok)

    def test_wait_all_ready_can_require_frozen_right_player_for_reset(self):
        agents = self._new_frozen_1v0_agents(
            right_flags=FLAG_IDLE,
            active_right=False,
        )

        ok = Agents.wait_all_ready(
            agents,
            timeout=0.0,
            poll=0.0,
            deadlock_window=0.10,
            context="unit-reset-right",
            wait_for_empty_fallback_players=True,
        )

        self.assertFalse(ok)

    def test_wait_trainer_reset_ack_pumps_ready_players(self):
        player = DummyPlayer(flags=FLAG_READY)
        agents = Agents.__new__(Agents)
        agents.config = SimpleNamespace(
            n1=1,
            n2=0,
            team1="base",
            trainer_ready_timeout_ms=50.0,
            reset_handshake_debug=False,
            reset_handshake_slow_log_seconds=1.0,
        )
        agents.log = None
        agents.player_list = [player]
        agents.trainer = AckAfterEmptyActionTrainer(player, target_seq=1)

        ok = Agents._wait_trainer_reset_ack(
            agents,
            target_seq=1,
            timeout=0.05,
            context="reset_custom_zero",
        )

        self.assertTrue(ok)
        self.assertEqual(player.empty_action_calls, 1)

    def test_reset_players_kicks_after_trainer_ack(self):
        order = []

        class ResetTrainer:
            def reset_players_and_ball(self, ball, left_players, right_players):
                del ball, left_players, right_players
                order.append("submit")
                return 7

            def phase(self):
                return 0

            def req_seq(self):
                return 7

            def done_seq(self):
                return 7

        agents = Agents.__new__(Agents)
        agents.config = SimpleNamespace(team1="base", reset_handshake_debug=False)
        agents.log = None
        agents.trainer = ResetTrainer()
        agents._wait_trainer_reset_ack = lambda seq, timeout, context: order.append("ack") or True
        agents._kick_all_players_after_reset = lambda: order.append("kick")
        agents.wait_all_ready = lambda *, timeout, context: order.append("wait") or True

        ok = Agents._reset_players_and_wait_ready(
            agents,
            ball=object(),
            left_players=[],
            right_players=[],
            timeout=1.0,
            context="unit-reset",
        )

        self.assertTrue(ok)
        self.assertEqual(order, ["submit", "ack", "kick", "wait"])

    def test_wait_all_ready_returns_true_when_flags_progress_to_ready(self):
        agents = Agents.__new__(Agents)
        agents.config = type("Cfg", (), {"n1": 1, "n2": 0, "team1": "base"})()
        agents.log = None
        agents.freeze_non_controlled = False
        agents.trainer = DummyTrainer(flags=FLAG_READY)
        player = SequencePlayer([FLAG_IDLE, FLAG_READY])
        agents.player_list = [player]

        ticks = iter([0.00, 0.00, 0.04, 0.04, 0.08, 0.08])

        def fake_sleep(_seconds):
            player.advance()

        with patch("envs.robocup2d.agents.time.monotonic", side_effect=lambda: next(ticks)):
            with patch("envs.robocup2d.agents.time.sleep", side_effect=fake_sleep):
                ok = Agents.wait_all_ready(
                    agents,
                    timeout=1.0,
                    poll=0.0,
                    deadlock_window=0.10,
                    context="unit-progress",
                )

        self.assertTrue(ok)

    def test_wait_all_ready_rescues_ready_trainer_when_stuck(self):
        agents = self._new_agents(
            trainer_flags=FLAG_READY,
            player_flags=[FLAG_IDLE],
        )

        now = {"t": -0.06}

        def fake_monotonic():
            now["t"] += 0.06
            return now["t"]

        with patch("envs.robocup2d.agents.time.monotonic", side_effect=fake_monotonic):
            with patch("envs.robocup2d.agents.time.sleep", return_value=None):
                ok = Agents.wait_all_ready(
                    agents,
                    timeout=0.30,
                    poll=0.0,
                    rescue_cooldown=0.05,
                    deadlock_window=0.10,
                    context="reset_custom_zero",
                )

        self.assertFalse(ok)
        self.assertEqual(agents.trainer.noop_calls, 0)

    def test_wait_all_ready_returns_true_when_trainer_done_and_players_ready(self):
        agents = self._new_agents(
            trainer_flags=FLAG_DONE,
            player_flags=[FLAG_READY, FLAG_READY],
        )

        ok = Agents.wait_all_ready(
            agents,
            timeout=0.30,
            poll=0.0,
            rescue_cooldown=0.05,
            deadlock_window=0.10,
            context="step turn=1 episode_step=1",
        )

        self.assertTrue(ok)
        self.assertEqual(agents.trainer.noop_calls, 0)
        self.assertEqual(agents.player_list[0].default_action_calls, 0)
        self.assertEqual(agents.player_list[1].default_action_calls, 0)

    def test_wait_all_ready_rescues_ready_trainer_and_ready_player_together(self):
        agents = self._new_agents(
            trainer_flags=FLAG_READY,
            player_flags=[FLAG_READY, FLAG_IDLE],
        )

        now = {"t": -0.06}

        def fake_monotonic():
            now["t"] += 0.06
            return now["t"]

        with patch("envs.robocup2d.agents.time.monotonic", side_effect=fake_monotonic):
            with patch("envs.robocup2d.agents.time.sleep", return_value=None):
                ok = Agents.wait_all_ready(
                    agents,
                    timeout=0.30,
                    poll=0.0,
                    rescue_cooldown=0.05,
                    fast_rescue_window=0.10,
                    deadlock_window=0.10,
                    context="step turn=1 episode_step=2",
                )

        self.assertFalse(ok)
        self.assertEqual(agents.trainer.noop_calls, 0)
        self.assertEqual(agents.player_list[0].default_action_calls, 1)
        self.assertEqual(agents.player_list[1].default_action_calls, 0)

    def test_wait_all_ready_rescues_pending_idle_trainer_request(self):
        agents = self._new_agents(
            trainer_flags=FLAG_IDLE,
            player_flags=[FLAG_READY, FLAG_READY],
        )
        agents.trainer = PendingIdleTrainer(req_seq=11, done_seq=10)

        now = {"t": -0.06}

        def fake_monotonic():
            now["t"] += 0.06
            return now["t"]

        with patch("envs.robocup2d.agents.time.monotonic", side_effect=fake_monotonic):
            with patch("envs.robocup2d.agents.time.sleep", return_value=None):
                ok = Agents.wait_all_ready(
                    agents,
                    timeout=0.30,
                    poll=0.0,
                    rescue_cooldown=0.05,
                    fast_rescue_window=0.10,
                    deadlock_window=0.10,
                    context="reset_custom_zero",
                )

        self.assertFalse(ok)
        self.assertEqual(agents.trainer.noop_calls, 0)
        self.assertEqual(agents.player_list[0].default_action_calls, 1)
        self.assertEqual(agents.player_list[1].default_action_calls, 1)

    def test_reset_goal_restart_precheck_when_no_custom_start(self):
        env = self._new_env_for_reset(turn_count=0, use_custom_start=False, goal=0)
        wait_calls = []

        def fake_wait(timeout, context):
            wait_calls.append((timeout, context))
            return True

        with patch.object(env, "_wait_all_ready_with_timeout", side_effect=fake_wait):
            info = Robocup2dEnv.reset(env)

        self.assertEqual(info["turn_count"], 1)
        self.assertEqual(
            wait_calls,
            [(3.0, "reset_goal_restart_precheck turn=1 attempt=1/1")],
        )
        self.assertEqual(env.agents.reset_default_calls, [])
        self.assertEqual(env.agents.reset_custom_calls, [])
        self.assertEqual(env.agents.coach.clear_goal_flag_calls, 1)
        self.assertEqual(env.agents.set_agent_mask_calls, 1)
        self.assertEqual(env.agents.reset_episode_epv_calls, 1)

    def test_reset_uses_custom_reposition_when_custom_start(self):
        env = self._new_env_for_reset(turn_count=0, use_custom_start=True, goal=0)
        trainer_wait_calls = []

        def fake_trainer_wait(timeout, context):
            trainer_wait_calls.append((timeout, context))
            return True

        env._wait_trainer_ready_with_timeout = fake_trainer_wait
        with patch.object(env, "_wait_all_ready_with_timeout", side_effect=self.fail):
            info = Robocup2dEnv.reset(env)

        self.assertEqual(info["turn_count"], 1)
        self.assertEqual(
            trainer_wait_calls,
            [(10000.0, "reset_trainer_precheck turn=1 attempt=1/1")],
        )
        self.assertEqual(env.agents.reset_default_calls, [])
        self.assertEqual(env.agents.reset_custom_calls, [3.0])
        self.assertEqual(env.agents.coach.clear_goal_flag_calls, 1)
        self.assertEqual(env.agents.set_agent_mask_calls, 1)
        self.assertEqual(env.agents.reset_episode_epv_calls, 1)

    def test_reset_uses_default_reposition_after_running_without_custom_start(self):
        env = self._new_env_for_reset(turn_count=1, use_custom_start=False, goal=0)
        trainer_wait_calls = []

        def fake_trainer_wait(timeout, context):
            trainer_wait_calls.append((timeout, context))
            return True

        env._wait_trainer_ready_with_timeout = fake_trainer_wait
        with patch.object(env, "_wait_all_ready_with_timeout", side_effect=self.fail):
            info = Robocup2dEnv.reset(env)

        self.assertEqual(info["turn_count"], 2)
        self.assertEqual(
            trainer_wait_calls,
            [(10000.0, "reset_trainer_precheck turn=2 attempt=1/1")],
        )
        self.assertEqual(env.agents.reset_default_calls, [3.0])


if __name__ == "__main__":
    unittest.main()
