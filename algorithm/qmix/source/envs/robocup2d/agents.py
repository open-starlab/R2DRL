from __future__ import annotations

from typing import Dict, Tuple, List, Sequence, Set, Optional
import os
import numpy as np
from pathlib import Path
from .protocols import P
from . import ipc
import time

Flags = Tuple[int, int]

class Agents:

    BASE_ACTION_NAMES = {
        0: "tackle",
        1: "shoot",
        2: "intercept",
        3: "advance",
        4: "pass_direct",
        5: "pass_lead",
        6: "pass_through",
        7: "hold",
        8: "catch",
        9: "dribble_up",
        10: "dribble_down",
        11: "dribble_left",
        12: "dribble_right",
        13: "move_up",
        14: "move_down",
        15: "move_left",
        16: "move_right",
        17: "helios",
        18: "wait",
    }
    HYBRID_ACTION_NAMES = {
        0: "turn",
        1: "dash",
        2: "kick",
        3: "catch",
        4: "helios",
        5: "wait",
    }

    def __init__(
        self,
        *,
        config,
        coach_shm_id: str,
        trainer_shm_id: str,
        player_shm_ids: Dict[Tuple[int, int], str],
        log=None,
    ):
        self.log = log
        self.config = config
        self.freeze_non_controlled = bool(
            getattr(self.config, "freeze_non_controlled", False)
        )
        self.active_right_unums: Set[int] = set(
            getattr(self.config, "active_right_unums", [])
        )
        self.coach_shm_id = coach_shm_id
        self.trainer_shm_id = trainer_shm_id
        self.player_shm_ids = dict(player_shm_ids)
        self.agent_mask = np.ones(self.config.n1, dtype=bool)
        self.current_mask_n = self.config.n1
        self.set_mask_n(getattr(self.config, "init_n", self.config.n1))
        self.controlled_n = min(
            int(getattr(self.config, "init_n", self.config.n1)),
            int(self.config.n1),
        )

        self.DEFAULT_BALL = (0.0, 0.0, 0.0, 0.0)
        self.DEFAULT_LEFT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = [
            (-49.4626, -0.0292, 88.8440, 0.0000, 0.0000),
            (-15.1939, -4.4258, 15.1550, 0.0001, 0.0000),
            (-15.1976, 3.0221, -10.8720, 0.0000, 0.0000),
            (-10.0065, -14.7888, 56.2060, 0.0002, -0.0001),
            (-11.5098, 13.4203, -47.9460, 0.0000, 0.0000),
            (-1.5655, 0.0000, -0.5220, 0.1258, 0.0000),
            (-1.0704, -6.6864, 79.3250, 0.0005, -0.0001),
            (-1.0712, 6.4699, -81.9520, 0.0004, 0.0001),
            (-1.2270, -19.3815, 86.2420, 0.0182, -0.0042),
            (-1.4633, 23.3622, -86.7830, 0.0427, 0.0477),
            (-0.3962, 0.0089, -170.9810, -0.0045, 0.0000),
        ]
        self.DEFAULT_RIGHT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = [
            (49.4000, 0.0000, -86.1680, 0.0000, 0.0000),
            (12.1702, 4.5999, -69.0720, 0.0000, 0.0000),
            (12.6534, -3.9485, 73.1320, -0.0001, 0.0000),
            (11.1458, 15.7497, -35.6450, 0.0000, 0.0000),
            (10.6564, -14.3726, 35.8610, -0.0023, -0.0008),
            (12.0067, 0.4165, -88.0130, 0.0000, 0.0000),
            (8.0825, 8.0061, -44.8550, 0.0000, 0.0000),
            (8.0155, -8.2239, 44.5960, 0.0000, 0.0000),
            (2.1734, 12.1163, -10.2030, 0.0000, 0.0000),
            (2.4714, -12.0142, 12.0330, 0.0000, 0.0000),
            (9.1624, 4.7206, -63.0240, 0.0000, 0.0000),
        ]
        self.DEFAULT_BODY_ANGLES = [
            88.8440,
            15.1550,
            -10.8720,
            56.2060,
            -47.9460,
            -0.5220,
            79.3250,
            -81.9520,
            86.2420,
            -86.7830,
            -170.9810,
            -86.1680,
            -69.0720,
            73.1320,
            -35.6450,
            35.8610,
            -88.0130,
            -44.8550,
            44.5960,
            -10.2030,
            12.0330,
            -63.0240,
        ]
        self.DEFAULT_RIGHT_KICKOFF_LEFT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = [
            (-49.4000, 0.0000, -93.8320, 0.0000, 0.0000),
            (-12.1702, 4.5999, -110.9280, 0.0000, 0.0000),
            (-12.6534, -3.9485, 106.8680, 0.0001, 0.0000),
            (-11.1458, 15.7497, -144.3550, 0.0000, 0.0000),
            (-10.6564, -14.3726, 144.1390, 0.0023, -0.0008),
            (-12.0067, 0.4165, -91.9870, 0.0000, 0.0000),
            (-8.0825, 8.0061, -135.1450, 0.0000, 0.0000),
            (-8.0155, -8.2239, 135.4040, 0.0000, 0.0000),
            (-2.1734, 12.1163, -169.7970, 0.0000, 0.0000),
            (-2.4714, -12.0142, 167.9670, 0.0000, 0.0000),
            (-9.1624, 4.7206, -116.9760, 0.0000, 0.0000),
        ]
        self.DEFAULT_RIGHT_KICKOFF_RIGHT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = [
            (49.4626, -0.0292, 91.1560, 0.0000, 0.0000),
            (15.1939, -4.4258, 164.8450, -0.0001, 0.0000),
            (15.1976, 3.0221, -169.1280, 0.0000, 0.0000),
            (10.0065, -14.7888, 123.7940, -0.0002, -0.0001),
            (11.5098, 13.4203, -132.0540, 0.0000, 0.0000),
            (1.5655, 0.0000, -179.4780, -0.1258, 0.0000),
            (1.0704, -6.6864, 100.6750, -0.0005, -0.0001),
            (1.0712, 6.4699, -98.0480, -0.0004, 0.0001),
            (1.2270, -19.3815, 93.7580, -0.0182, -0.0042),
            (1.4633, 23.3622, -93.2170, -0.0427, 0.0477),
            (0.3962, 0.0089, -9.0190, 0.0045, 0.0000),
        ]
        self.DEFAULT_RIGHT_KICKOFF_BODY_ANGLES = [
            -93.8320,
            -110.9280,
            106.8680,
            -144.3550,
            144.1390,
            -91.9870,
            -135.1450,
            135.4040,
            -169.7970,
            167.9670,
            -116.9760,
            91.1560,
            164.8450,
            -169.1280,
            123.7940,
            -132.0540,
            -179.4780,
            100.6750,
            -98.0480,
            93.7580,
            -93.2170,
            -9.0190,
        ]
        self.DEFAULT_3V3_LEFT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = [
            (-49.3706, 0.0199, 88.9400, 0.0000, 0.0000),
            (-15.6612, -4.5942, 15.6520, 0.0000, 0.0000),
            (-0.4939, 0.0773, -9.0620, 0.0664, -0.0116),
        ]
        self.DEFAULT_3V3_RIGHT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = [
            (49.7347, 0.0000, -89.8210, 0.0000, 0.0000),
            (12.6324, 4.4281, -70.9510, 0.0000, 0.0000),
            (13.0318, -4.1272, 72.0180, 0.0000, 0.0000),
        ]
        self.DEFAULT_3V3_BODY_ANGLES = [
            88.9400,
            15.6520,
            -9.0620,
            -89.8210,
            -70.9510,
            72.0180,
        ]
        self.DEFAULT_3V3_RIGHT_KICKOFF_LEFT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = [
            (-49.7347, 0.0000, -90.1790, 0.0000, 0.0000),
            (-12.6324, 4.4281, -109.0490, 0.0000, 0.0000),
            (-13.0318, -4.1272, 107.9820, 0.0000, 0.0000),
        ]
        self.DEFAULT_3V3_RIGHT_KICKOFF_RIGHT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = [
            (49.3706, 0.0199, 91.0600, 0.0000, 0.0000),
            (15.6612, -4.5942, 164.3480, 0.0000, 0.0000),
            (0.4939, 0.0773, -170.9380, -0.0664, -0.0116),
        ]
        self.DEFAULT_3V3_RIGHT_KICKOFF_BODY_ANGLES = [
            -90.1790,
            -109.0490,
            107.9820,
            91.0600,
            164.3480,
            -170.9380,
        ]

        self.CUSTOM_BALL = self.DEFAULT_BALL
        self.CUSTOM_LEFT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = list(
            self.DEFAULT_LEFT_PLAYERS
        )
        self.CUSTOM_RIGHT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = list(
            self.DEFAULT_RIGHT_PLAYERS
        )
        self.CUSTOM_BODY_ANGLES = list(self.DEFAULT_BODY_ANGLES)

        self.CUSTOM_LEFT_PLAYERS, self.CUSTOM_RIGHT_PLAYERS = (
            self._get_custom_players_for_current_config()
        )
        self.CUSTOM_BODY_ANGLES = self._get_custom_body_angles_for_current_config()

        (
            self.coach_shms,
            self.trainer_shms,
            self.player_shms,) = ipc.create_shm_group(
            coach_names=[self.coach_shm_id],
            trainer_names=[self.trainer_shm_id],
            player_names=list(self.player_shm_ids.values()),
            coach_size=P.coach.COACH_SHM_SIZE,
            trainer_size=P.trainer.TRAINER_SHM_SIZE,
            player_size=P.player.PLAYER_SHM_SIZE,
            zero_fill=True,
            log=log,
        )

        self.coach = P.coach.Coach(self.coach_shms[self.coach_shm_id].buf)
        self.trainer = P.trainer.Trainer(self.trainer_shms[self.trainer_shm_id].buf)
        self.players: Dict[Tuple[int, int], P.player.Player] = {}

        for key, shm_id in self.player_shm_ids.items():
            buf = self.player_shms[shm_id].buf
            self.players[key] = P.player.Player(buf)

        # Ordered player list
        self.player_list: List[P.player.Player] = [
            self.players[(team, unum)]
            for team in (1, 2)
            for unum in range(
                1,
                self.config.n1 + 1 if team == 1 else self.config.n2 + 1,
            )
        ]

        if self.config.team1 == "hybrid":
            self.n_actions = P.player.HYBRID_MASK_NUM
        else:
            self.n_actions = P.player.BASE_MASK_NUM
        self.use_action_mask = bool(getattr(self.config, "use_action_mask", True))
        self._no_action_mask_avail = self._make_no_action_mask_avail()

        self._obs_buf = np.empty((len(self.player_list), P.player.STATE_NUM), dtype=np.float32)
        self._mask_buf = np.empty((len(self.player_list), self.n_actions), dtype=np.int32)
        self._team1_obs_buf = np.empty((self.controlled_n, P.player.STATE_NUM), dtype=np.float32)
        self._team1_mask_buf = np.empty((self.controlled_n, self.n_actions), dtype=np.int32)
        if self.config.team1 == "hybrid":
            default_actions = []
            for player in self.player_list[:self.config.n1]:
                action = (
                    player.empty_hybrid_action
                    if self.freeze_non_controlled
                    else player.default_hybrid_action
                )
                default_actions.append(int(action[0]))
        else:
            default_actions = []
            for player in self.player_list[:self.config.n1]:
                action = (
                    player.empty_base_action
                    if self.freeze_non_controlled
                    else player.default_base_action
                )
                default_actions.append(int(action))
        self._team1_default_actions = np.asarray(default_actions, dtype=np.intp)

        self.kickable = False
        self.opponent_kickable = False
        self.true_kickable = False
        self.opponent_true_kickable = False
        self.ball_owner_team = 0
        self.kickable_threshold = 1.085
        self.team1_min_ball_dist = float("inf")
        self.team2_min_ball_dist = float("inf")
        self.nearest_team1_idx = -1
        self.nearest_team2_idx = -1
        self._action_debug_enabled = os.environ.get("ROBOCUP_ACTION_DEBUG", "1").lower() not in (
            "0", "false", "no", "off"
        )
        self._action_debug_limit = int(os.environ.get("ROBOCUP_ACTION_DEBUG_LIMIT", "200"))
        self._action_debug_every = max(1, int(os.environ.get("ROBOCUP_ACTION_DEBUG_EVERY", "1")))
        self._action_debug_count = 0

        # Compute EPV for diagnostics even when EPV reward shaping is disabled.
        self.epv_grid = self._load_epv_grid()
        self.epv_grid_left = np.fliplr(self.epv_grid) if self.epv_grid is not None else None
        if self.epv_grid is not None:
            self._epv_ny, self._epv_nx = self.epv_grid.shape
            self._epv_max_value = float(np.max(self.epv_grid))
            self._epv_field_length = float(self.config.half_length) * 2.0
            self._epv_field_width = float(self.config.half_width) * 2.0
            self._epv_half_length = self._epv_field_length / 2.0
            self._epv_half_width = self._epv_field_width / 2.0
            self._epv_dx = self._epv_field_length / float(self._epv_nx)
            self._epv_dy = self._epv_field_width / float(self._epv_ny)
        else:
            self._epv_ny = self._epv_nx = 0
            self._epv_max_value = 0.0
            self._epv_field_length = self._epv_field_width = 0.0
            self._epv_half_length = self._epv_half_width = 0.0
            self._epv_dx = self._epv_dy = 1.0
        self._epv_last_key = None
        self._epv_last_value = 0.0
        self.current_epv = 0.0
        self.initial_episode_epv = 0.0
        self.max_episode_epv = 0.0

    def _make_no_action_mask_avail(self) -> np.ndarray:
        mask = np.ones(self.n_actions, dtype=np.int32)
        action_names = (
            self.HYBRID_ACTION_NAMES
            if self.config.team1 == "hybrid"
            else self.BASE_ACTION_NAMES
        )
        for action_id, action_name in action_names.items():
            if action_name in ("advance", "helios", "wait"):
                mask[int(action_id)] = 0
        return mask

    def get_player(self, team: int, unum: int) -> P.player.Player:
        return self.players[(team, unum)]

    def all_players(self) -> List[P.player.Player]:
        return self.player_list

    def _right_unum_is_active(self, player_index: int) -> bool:
        right_unum = player_index - self.config.n1 + 1
        return right_unum in self.active_right_unums

    def _player_needs_empty_fallback(self, player_index: int) -> bool:
        if not self.freeze_non_controlled:
            return False
        if player_index < self.config.n1:
            return not bool(self.agent_mask[player_index])
        return not self._right_unum_is_active(player_index)

    def _player_required_for_ready_wait(self, player_index: int) -> bool:
        return not self._player_needs_empty_fallback(player_index)

    def _take_fallback_action(self, player_index: int, player, *, is_hybrid: bool) -> None:
        if self._player_needs_empty_fallback(player_index):
            player.take_empty_action(is_hybrid=is_hybrid)
        else:
            player.take_default_action(is_hybrid=is_hybrid)

    def controlled_agent_count(self) -> int:
        return int(self.controlled_n)

    def active_team1_indices(self) -> np.ndarray:
        active = np.flatnonzero(self.agent_mask)
        expected = self.controlled_agent_count()
        if active.size != expected:
            raise RuntimeError(
                f"active team1 count mismatch: got {active.size}, expected {expected}. "
                "Call set_agent_mask() after syncing init_n before reading obs/actions."
            )
        return active.astype(np.intp, copy=False)

    def request(self, target: str) -> None:
        """
        Send REQUEST signal to:
        - "trainer"
        - "player:team:unum"
        """

        if target == "trainer":
            self.trainer.submit_opcode(P.trainer.OP_NOOP)

        elif target.startswith("player:"):
            try:
                _, team, unum = target.split(":")
                team = int(team)
                unum = int(unum)
            except Exception:
                raise ValueError(
                    "player format must be 'player:team:unum'"
                )

            player = self.get_player(team, unum)
            player.take_default_action(is_hybrid=(self.config.team1 == "hybrid"))

        else:
            raise ValueError(f"Unknown request target: {target}")

    def clear_all_shm_bufs(self) -> None:
        ipc.zero_all_shm_bufs(
            self.coach_shms,
            self.trainer_shms,
            self.player_shms,
        )

    def close(self) -> None:
        """
        Close and unlink shm objects.
        """

        self.clear_all_shm_bufs()

        for shm_dict in (
            self.coach_shms,
            self.trainer_shms,
            self.player_shms,
        ):
            for shm in shm_dict.values():
                try:
                    shm.close()
                except Exception:
                    pass
                try:
                    shm.unlink()
                except Exception:
                    pass

    def _load_epv_grid(self) -> np.ndarray:
        epv_grid_file = (
            getattr(self.config, "epv_grid_file", "EPV_grid.csv")
            or "EPV_grid.csv"
        )
        epv_path = Path(epv_grid_file)
        if not epv_path.is_absolute():
            epv_path = (
                Path(__file__).resolve().parent
                / "LaurieOnTracking"
                / epv_grid_file
            )
        if not epv_path.exists():
            raise FileNotFoundError(f"EPV grid file not found: {epv_path}")
        return np.loadtxt(epv_path, delimiter=",", dtype=np.float32)

    def get_ball_position_epv(
        self,
        position: Tuple[float, float] | None = None,
        attack_direction: int = 1,
    ) -> float:
        if position is None:
            state = self.state(norm=False)
            position = (float(state[0]), float(state[1]))
        if self.epv_grid is None:
            return 0.0

        x, y = position
        if abs(x) > self._epv_half_length or abs(y) > self._epv_half_width:
            return 0.0

        ix = int((x + self._epv_half_length - 1e-4) / self._epv_dx)
        iy = int((y + self._epv_half_width - 1e-4) / self._epv_dy)
        ix = min(max(ix, 0), self._epv_nx - 1)
        iy = min(max(iy, 0), self._epv_ny - 1)
        key = (int(attack_direction), ix, iy)
        if key == self._epv_last_key:
            return self._epv_last_value

        epv = self.epv_grid_left if attack_direction == -1 else self.epv_grid
        value = float(epv[iy, ix])
        self._epv_last_key = key
        self._epv_last_value = value
        return value

    def reset_episode_epv(self) -> float:
        self.current_epv = self.get_ball_position_epv()
        self.initial_episode_epv = self.current_epv
        self.max_episode_epv = self.current_epv
        return self.max_episode_epv

    def update_episode_epv(self, position: Tuple[float, float] | None = None) -> float:
        if self.config.useMaxEpv and not self.true_kickable:
            return 0.0

        prev_max_epv = self.max_episode_epv
        self.current_epv = self.get_ball_position_epv(position=position)
        self.max_episode_epv = max(prev_max_epv, self.current_epv)
        epv_delta = float(self.max_episode_epv - prev_max_epv)
        if (
            not self.config.useMaxEpv
            or epv_delta <= 0.0
        ):
            return 0.0

        progress_reward = float(getattr(self.config, "epv_progress_reward", 0.0))
        if progress_reward > 0.0:
            return progress_reward
        progress_scale = float(getattr(self.config, "epv_progress_scale", 1.0))
        return epv_delta * progress_scale

    def complete_episode_epv(self) -> float:
        if not self.config.useMaxEpv:
            return 0.0

        prev_max_epv = self.max_episode_epv
        target_epv = self._epv_max_value
        if target_epv <= prev_max_epv:
            return 0.0

        self.current_epv = target_epv
        self.max_episode_epv = target_epv
        progress_scale = float(getattr(self.config, "epv_progress_scale", 1.0))
        return float(target_epv - prev_max_epv) * progress_scale

    def state(self, norm: bool = True):
        """
        Return current global state from coach.

        norm=True  -> normalized state
        norm=False -> raw state
        """
        if norm:
            return self.coach.state_norm(half_field_length=self.config.half_length, half_field_width=self.config.half_width)
        else:
            return self.coach.state()

    def obs(self, norm: bool = True):
        """
        Return stacked observations of all players.
        Shape: (n_total_players, STATE_NUM)
        """

        for i, p in enumerate(self.player_list):
            if norm:
                self._obs_buf[i] = p.obs_norm(
                    half_field_length=self.config.half_length,
                    half_field_width=self.config.half_width,
                )
            else:
                self._obs_buf[i] = p.obs()

        return self._obs_buf

    def avail_actions(self):
        if not self.use_action_mask:
            self._mask_buf[:] = self._no_action_mask_avail
            return self._mask_buf

        for i, p in enumerate(self.player_list):
            if self.config.team1 == "hybrid":
                self._mask_buf[i] = p.hybrid_mask()
            else:
                self._mask_buf[i] = p.base_mask()

        return self._mask_buf

    def _should_print_action_debug(self) -> bool:
        if not self._action_debug_enabled:
            return False

        count = self._action_debug_count
        self._action_debug_count += 1

        if self._action_debug_limit >= 0 and count >= self._action_debug_limit:
            return False

        return count % self._action_debug_every == 0

    def _action_debug_line(
        self,
        *,
        kind: str,
        idx: int,
        action: int,
        mask: np.ndarray,
        obs_seq: int,
        u0: Optional[float] = None,
        u1: Optional[float] = None,
    ) -> None:
        if not self._should_print_action_debug():
            return

        state = self.state(norm=False)
        bx = float(state[0])
        by = float(state[1])
        players = state[4:].reshape(22, 6)
        px = float(players[idx, 0])
        py = float(players[idx, 1])
        dist = float(np.hypot(px - bx, py - by))

        valid = np.flatnonzero(np.asarray(mask).reshape(-1) > 0).astype(int).tolist()
        selected_valid = bool(0 <= int(action) < len(mask) and bool(mask[int(action)]))

        if kind == "hybrid":
            name = self.HYBRID_ACTION_NAMES.get(int(action), "unknown")
            params = f" u0={float(u0):.3f} u1={float(u1):.3f}"
        else:
            name = self.BASE_ACTION_NAMES.get(int(action), "unknown")
            params = ""

        print(
            "[ActionDebug] "
            f"team={self.config.team1} idx={idx} unum={idx + 1} "
            f"obs_seq={int(obs_seq)} action={int(action)}:{name}{params} "
            f"selected_valid={int(selected_valid)} valid={valid} "
            f"ball=({bx:.3f},{by:.3f}) self=({px:.3f},{py:.3f}) "
            f"dist={dist:.3f} min_team1_dist={self.team1_min_ball_dist:.3f} "
            f"threshold={self.kickable_threshold:.3f} "
            f"kickable_dist={int(self.kickable)} true_kickable={int(self.true_kickable)} "
            f"owner={int(self.ball_owner_team)} "
            f"opp_min_dist={self.team2_min_ball_dist:.3f} "
            f"opp_kickable_dist={int(self.opponent_kickable)} "
            f"opp_true_kickable={int(self.opponent_true_kickable)}",
            flush=True,
        )

    def write_base_actions(self, actions: np.ndarray):
        n1 = self.config.n1
        actions = np.asarray(actions).reshape(-1)
        active_idx = self.active_team1_indices()
        if actions.shape[0] != active_idx.shape[0]:
            raise ValueError(
                f"base actions must contain {active_idx.shape[0]} active rows, got {actions.shape[0]}"
            )
        action_by_idx = {int(idx): int(actions[row]) for row, idx in enumerate(active_idx)}

        for idx, p in enumerate(self.player_list):
            if idx < n1:
                if idx in action_by_idx:
                    act = action_by_idx[idx]
                elif self.freeze_non_controlled:
                    act = int(p.empty_base_action)
                else:
                    act = int(p.default_base_action)
                # act = int(p.default_base_action)
            else:
                if self.freeze_non_controlled and not self._right_unum_is_active(idx):
                    act = int(p.empty_base_action)
                else:
                    act = int(p.default_base_action)
            obs_seq = int(p.obs_seq())
            if idx in action_by_idx:
                self._action_debug_line(
                    kind="base",
                    idx=idx,
                    action=act,
                    mask=p.base_mask() if self.use_action_mask else self._no_action_mask_avail,
                    obs_seq=obs_seq,
                )
            p.submit_base_action(act, target_obs_seq=obs_seq)

    def write_hybrid_actions(self, actions: np.ndarray):
        n1 = self.config.n1
        actions = np.asarray(actions)
        active_idx = self.active_team1_indices()
        if actions.shape[0] != active_idx.shape[0]:
            raise ValueError(
                f"hybrid actions must contain {active_idx.shape[0]} active rows, got {actions.shape[0]}"
            )
        action_by_idx = {int(idx): actions[row] for row, idx in enumerate(active_idx)}

        for idx, p in enumerate(self.player_list):
            if idx < n1:
                if idx in action_by_idx:
                    action = action_by_idx[idx]
                    a  = int(action[0])
                    u0 = float(action[1])
                    u1 = float(action[2])
                elif self.freeze_non_controlled:
                    a, u0, u1 = p.empty_hybrid_action
                    a = int(a)
                    u0 = float(u0)
                    u1 = float(u1)
                else:
                    a, u0, u1 = p.default_hybrid_action
                    a = int(a)
                    u0 = float(u0)
                    u1 = float(u1)
            else:
                if self.freeze_non_controlled and not self._right_unum_is_active(idx):
                    a, u0, u1 = p.empty_hybrid_action
                else:
                    a, u0, u1 = p.default_hybrid_action
                a = int(a)
                u0 = float(u0)
                u1 = float(u1)

            obs_seq = int(p.obs_seq())
            if idx in action_by_idx:
                self._action_debug_line(
                    kind="hybrid",
                    idx=idx,
                    action=a,
                    u0=u0,
                    u1=u1,
                    mask=p.hybrid_mask() if self.use_action_mask else self._no_action_mask_avail,
                    obs_seq=obs_seq,
                )
            p.submit_hybrid_action(a, u0, u1, target_obs_seq=obs_seq)

    def write_actions(self, actions: np.ndarray):
        if self.config.team1 == "hybrid":
            self.write_hybrid_actions(actions)
        else:
            self.write_base_actions(actions)

    def wait_all_ready(
        self,
        timeout: float = 3600.0,
        poll: float = 0.0005,
        stuck_window: float = 10.0,
        rescue_cooldown: float = 0.2,
        fast_rescue_window: float | None = None,
        deadlock_window: float | None = None,
        context: str = "",
        wait_for_empty_fallback_players: bool = False,
    ):
        if fast_rescue_window is None:
            fast_rescue_window = 0.2
        trainer_deadlock_window = float(
            stuck_window if deadlock_window is None else deadlock_window
        )
        t_end = time.monotonic() + float(timeout)
        now = time.monotonic()
        last_signature = None
        last_change = now
        last_rescue = 0.0
        last_trainer_pending_log = 0.0
        last_empty_fallback_obs_seq: Dict[int, int] = {}

        while True:
            trainer_idle = self.trainer.is_idle()
            trainer_phase = int(self.trainer.phase())
            trainer_req_seq = int(self.trainer.req_seq())
            trainer_done_seq = int(self.trainer.done_seq())
            trainer_pending_not_busy = (
                (not trainer_idle)
                and trainer_phase != P.common.TRAINER_PHASE_BUSY
                and trainer_req_seq > trainer_done_seq
            )
            player_ready = [bool(p.is_playon_ready()) for p in self.player_list]
            if wait_for_empty_fallback_players:
                wait_player_indices = list(range(len(self.player_list)))
            else:
                wait_player_indices = [
                    idx
                    for idx in range(len(self.player_list))
                    if self._player_required_for_ready_wait(idx)
                ]
            wait_player_ready = tuple(player_ready[idx] for idx in wait_player_indices)
            signature = (
                bool(trainer_idle),
                trainer_phase,
                trainer_req_seq,
                trainer_done_seq,
                wait_player_ready,
            )

            if trainer_idle and all(wait_player_ready):
                return True

            now = time.monotonic()
            if signature != last_signature:
                last_signature = signature
                last_change = now

            ready_count = sum(wait_player_ready)
            wait_player_count = len(wait_player_ready)
            all_players_ready = all(wait_player_ready)
            total_ready_count = sum(player_ready)
            total_player_count = len(player_ready)
            stable_for = now - last_change

            is_hybrid = (self.config.team1 == "hybrid")
            for idx, player in enumerate(self.player_list):
                if not self._player_needs_empty_fallback(idx):
                    continue
                if not player_ready[idx]:
                    continue
                obs_seq = int(player.obs_seq())
                if last_empty_fallback_obs_seq.get(idx) == obs_seq:
                    continue
                player.take_empty_action(is_hybrid=is_hybrid)
                last_empty_fallback_obs_seq[idx] = obs_seq

            can_rescue_players = (
                (
                    trainer_idle
                    and ready_count > 0
                    and ready_count < wait_player_count
                )
                or (
                    trainer_pending_not_busy
                    and all_players_ready
                    and ready_count > 0
                )
            )
            can_rescue_players = (
                can_rescue_players
                and stable_for >= float(fast_rescue_window)
                and (now - last_rescue) >= float(rescue_cooldown)
            )
            rescued_players = False
            if can_rescue_players:
                pushed = 0
                is_hybrid = (self.config.team1 == "hybrid")
                for idx in wait_player_indices:
                    is_ready = player_ready[idx]
                    player = self.player_list[idx]
                    if not is_ready:
                        continue
                    self._take_fallback_action(
                        idx,
                        player,
                        is_hybrid=is_hybrid,
                    )
                    pushed += 1
                if pushed > 0:
                    if self.log:
                        self.log.info(
                            "[wait_all_ready] rescue "
                            f"context={context or 'unknown'} "
                            f"stable_for={stable_for:.3f}s "
                            f"pushed_players={pushed} "
                            f"ready_players={ready_count}/{wait_player_count} "
                            f"all_players={total_ready_count}/{total_player_count} "
                            f"trainer_pending={int(trainer_pending_not_busy)} "
                            f"trainer_req_seq={trainer_req_seq} "
                            f"trainer_done_seq={trainer_done_seq}"
                        )
                    last_rescue = now
                    last_change = now
                    rescued_players = True

            if trainer_pending_not_busy and all_players_ready:
                if self.log and (now - last_trainer_pending_log) >= 1.0:
                    self.log.info(
                        "[wait_all_ready] trainer_pending_all_players_ready "
                        f"context={context or 'unknown'} "
                        f"stable_for={stable_for:.3f}s "
                        f"ready_players={ready_count}/{wait_player_count} "
                        f"all_players={total_ready_count}/{total_player_count} "
                        f"trainer_phase={trainer_phase} "
                        f"trainer_req_seq={trainer_req_seq} "
                        f"trainer_done_seq={trainer_done_seq}"
                    )
                    last_trainer_pending_log = now
                if stable_for >= trainer_deadlock_window and not rescued_players:
                    if self.log:
                        self.log.info(
                            "[wait_all_ready] trainer_pending_deadlock "
                            f"context={context or 'unknown'} "
                            f"stable_for={stable_for:.3f}s "
                            f"ready_players={ready_count}/{wait_player_count} "
                            f"all_players={total_ready_count}/{total_player_count} "
                            f"trainer_phase={trainer_phase} "
                            f"trainer_req_seq={trainer_req_seq} "
                            f"trainer_done_seq={trainer_done_seq} "
                            f"players={self.read_all_flags(include_cycles=True)}"
                        )
                    return False

            if now >= t_end:
                if self.log:
                    self.log.info(
                        "[wait_all_ready] timeout "
                        f"trainer={{phase:{trainer_phase}, req_seq:{trainer_req_seq}, done_seq:{trainer_done_seq}}} "
                        f"players={self.read_all_flags(include_cycles=True)}"
                    )
                return False

            time.sleep(float(poll))

    def _trainer_ack_timeout_ms(self, timeout: float) -> int:
        configured = float(
            getattr(
                self.config,
                "trainer_ready_timeout_ms",
                P.trainer.TRAINER_WAIT_DONE_TIMEOUT_MS,
            )
        )
        if configured <= 0.0:
            configured = float(P.trainer.TRAINER_WAIT_DONE_TIMEOUT_MS)
        if timeout is not None and float(timeout) > 0.0:
            configured = min(configured, float(timeout) * 1000.0)
        return max(1, int(configured))

    def _wait_trainer_reset_ack(
        self,
        target_seq: int,
        timeout: float,
        context: str,
    ) -> bool:
        timeout_ms = self._trainer_ack_timeout_ms(timeout)
        t0 = time.monotonic()
        deadline = t0 + timeout_ms / 1000.0
        target_seq = int(target_seq)
        poll_seconds = max(float(P.trainer.TRAINER_POLL_US) / 1_000_000.0, 0.0001)
        is_hybrid = (self.config.team1 == "hybrid")
        pushed_players = 0
        last_pushed_obs_seq: Dict[int, int] = {}
        ok = False

        while time.monotonic() < deadline:
            if int(self.trainer.done_seq()) >= target_seq:
                ok = True
                break

            for idx, player in enumerate(self.player_list):
                if not bool(player.is_playon_ready()):
                    continue
                obs_seq = int(player.obs_seq())
                if last_pushed_obs_seq.get(idx) == obs_seq:
                    continue
                player.take_empty_action(is_hybrid=is_hybrid)
                last_pushed_obs_seq[idx] = obs_seq
                pushed_players += 1

            time.sleep(poll_seconds)

        if not ok:
            ok = int(self.trainer.done_seq()) >= target_seq
        elapsed = time.monotonic() - t0
        debug = bool(getattr(self.config, "reset_handshake_debug", False))
        slow_seconds = float(getattr(self.config, "reset_handshake_slow_log_seconds", 1.0))
        if self.log and (debug or (not ok) or elapsed >= slow_seconds):
            self.log.info(
                "[reset_handshake] trainer_ack "
                f"context={context or 'unknown'} "
                f"ok={int(ok)} "
                f"elapsed={elapsed:.3f}s "
                f"timeout_ms={timeout_ms} "
                f"target_seq={int(target_seq)} "
                f"trainer_phase={int(self.trainer.phase())} "
                f"trainer_req_seq={int(self.trainer.req_seq())} "
                f"trainer_done_seq={int(self.trainer.done_seq())} "
                f"pushed_players={int(pushed_players)}"
            )
        if self.log and (not ok) and pushed_players == 0:
            self.log.warning(
                "[reset_handshake] no_player_progress_before_trainer_ack "
                f"context={context or 'unknown'} "
                f"target_seq={int(target_seq)} "
                f"trainer_phase={int(self.trainer.phase())} "
                f"trainer_req_seq={int(self.trainer.req_seq())} "
                f"trainer_done_seq={int(self.trainer.done_seq())}"
            )
        return bool(ok)

    def _reset_players_and_wait_ready(
        self,
        ball,
        left_players,
        right_players,
        timeout: float,
        context: str,
    ) -> bool:
        seq = self.trainer.reset_players_and_ball(ball, left_players, right_players)
        if self.log and bool(getattr(self.config, "reset_handshake_debug", False)):
            self.log.info(
                "[reset_handshake] submit "
                f"context={context or 'unknown'} "
                f"target_seq={int(seq)} "
                f"trainer_phase={int(self.trainer.phase())} "
                f"trainer_req_seq={int(self.trainer.req_seq())} "
                f"trainer_done_seq={int(self.trainer.done_seq())}"
            )
        self._kick_all_players_after_reset()
        if not self._wait_trainer_reset_ack(seq, timeout, context):
            return False
        return self.wait_all_ready(
            timeout=timeout,
            context=context,
        )

    def _kick_all_players_after_reset(self) -> None:
        """
        After a trainer-driven reset is submitted, force all players to
        consume one fallback action so the synchronized server can advance
        the cycle while the trainer request is being acknowledged.
        """
        is_hybrid = (self.config.team1 == "hybrid")
        for p in self.player_list:
            p.take_empty_action(is_hybrid=is_hybrid)

    def _reset_perception_warmup_cycles(self) -> int:
        cycles = int(getattr(self.config, "reset_perception_warmup_cycles", 3))
        return max(0, cycles)

    def _run_reset_perception_warmup(
        self,
        timeout: float,
        context: str,
        ball,
        left_players,
        right_players,
    ) -> bool:
        cycles = self._reset_perception_warmup_cycles()
        for i in range(cycles):
            if not self._reset_players_and_wait_ready(
                ball,
                left_players,
                right_players,
                timeout,
                context=f"{context}_perception_warmup {i + 1}/{cycles}",
            ):
                return False
        return True

    def reset_default(
        self,
        wait_timeout: float | None = None,
        *,
        kickoff_side: str = "left",
    ) -> bool:
        timeout = 3600.0 if wait_timeout is None else float(wait_timeout)
        default_left_players, default_right_players = self._get_default_players_for_current_config(
            kickoff_side=kickoff_side,
        )
        context = f"reset_default_{kickoff_side}_kickoff"

        if not self._reset_players_and_wait_ready(
            self.DEFAULT_BALL,
            default_left_players,
            default_right_players,
            timeout,
            context=context,
        ):
            return False
        return self._run_reset_perception_warmup(
            timeout,
            context,
            self.DEFAULT_BALL,
            default_left_players,
            default_right_players,
        )

    def reset_custom(self, wait_timeout: float | None = None) -> bool:
        timeout = 3600.0 if wait_timeout is None else float(wait_timeout)
        custom_left_players = self.CUSTOM_LEFT_PLAYERS
        custom_right_players = self.CUSTOM_RIGHT_PLAYERS

        custom_left_players_zero = self.zero_player_velocities(custom_left_players)
        custom_right_players_zero = self.zero_player_velocities(custom_right_players)

        if not self._reset_players_and_wait_ready(
            self.CUSTOM_BALL,
            custom_left_players_zero,
            custom_right_players_zero,
            timeout,
            context="reset_custom_zero",
        ):
            return False

        if not self._reset_players_and_wait_ready(
            self.CUSTOM_BALL,
            custom_left_players,
            custom_right_players,
            timeout,
            context="reset_custom_final",
        ):
            return False
        return self._run_reset_perception_warmup(
            timeout,
            "reset_custom_final",
            self.CUSTOM_BALL,
            custom_left_players,
            custom_right_players,
        )

    def read_all_flags(self, include_cycles: bool = False) -> Dict[str, object]:
        who: Dict[str, object] = {}
        for i, p in enumerate(self.player_list):
            name = f"player:{i}"
            payload = {
                "phase": int(p.phase()),
                "obs_seq": int(p.obs_seq()),
                "act_seq": int(p.act_seq()),
                "done_seq": int(p.done_seq()),
                "ready": bool(p.is_playon_ready()),
            }

            if include_cycles:
                payload["cycle"] = int(p.cycle())
            who[name] = payload

        who["trainer"] = {
            "phase": int(self.trainer.phase()),
            "req_seq": int(self.trainer.req_seq()),
            "done_seq": int(self.trainer.done_seq()),
            "idle": bool(self.trainer.is_idle()),
        }

        return {
            "who": who,
            "trainer": who["trainer"],
        }

    def get_team1_obs(self, norm: bool = True, zero_inactive: bool = True):
        out = self._team1_obs_buf
        active_idx = self.active_team1_indices()
        for row, idx in enumerate(active_idx):
            player = self.player_list[int(idx)]
            if norm:
                out[row] = player.obs_norm(
                    half_field_length=self.config.half_length,
                    half_field_width=self.config.half_width,
                )
            else:
                out[row] = player.obs()

        return out

    def get_team1_avail_actions(self):
        out = self._team1_mask_buf
        active_idx = self.active_team1_indices()
        if not self.use_action_mask:
            out[:] = self._no_action_mask_avail
            return out

        is_hybrid = self.config.team1 == "hybrid"
        for row, idx in enumerate(active_idx):
            player = self.player_list[int(idx)]
            out[row] = player.hybrid_mask() if is_hybrid else player.base_mask()

        return out

    def set_agent_mask(self) -> np.ndarray:
        n = min(int(self.current_mask_n), self.config.n1)

        state = self.state(norm=False)

        bx = float(state[0])
        by = float(state[1])

        players = state[4:].reshape(22, 6)

        team1_slots = players[:11]
        team1_players = team1_slots[:self.config.n1]
        team2_slots = players[11:22]
        team2_players = team2_slots[:self.config.n2]

        px = team1_players[:, 0]
        py = team1_players[:, 1]

        dists = np.sqrt((px - bx) ** 2 + (py - by) ** 2)
        sorted_idx = np.argsort(dists)
        nearest_idx = sorted_idx[:n]

        self.agent_mask[:] = False
        self.agent_mask[nearest_idx] = True

        team1_min_dist = float(dists[sorted_idx[0]]) if len(sorted_idx) > 0 else np.inf
        self.team1_min_ball_dist = team1_min_dist
        self.nearest_team1_idx = int(sorted_idx[0]) if len(sorted_idx) > 0 else -1

        opp_x = team2_players[:, 0]
        opp_y = team2_players[:, 1]
        opp_dists = np.sqrt((opp_x - bx) ** 2 + (opp_y - by) ** 2)
        team2_min_dist = float(np.min(opp_dists)) if opp_dists.size > 0 else np.inf
        self.team2_min_ball_dist = team2_min_dist
        self.nearest_team2_idx = int(np.argmin(opp_dists)) if opp_dists.size > 0 else -1

        self.kickable = bool(team1_min_dist <= self.kickable_threshold)
        self.opponent_kickable = bool(team2_min_dist <= self.kickable_threshold)
        self.true_kickable = any(
            self.players[(1, unum)].is_kickable()
            for unum in range(1, self.config.n1 + 1)
        )
        self.opponent_true_kickable = any(
            self.players[(2, unum)].is_kickable()
            for unum in range(1, self.config.n2 + 1)
        )

        if self.kickable and (not self.opponent_kickable or team1_min_dist <= team2_min_dist):
            self.ball_owner_team = 1
        elif self.opponent_kickable:
            self.ball_owner_team = 2
        else:
            self.ball_owner_team = 0

        return self.agent_mask.copy()

    def set_mask_n(self, n: int) -> int:
        """
        Configure how many team1 agents should be activated
        when set_agent_mask() is called.

        Returns:
            sanitized n
        """
        n1 = int(self.config.n1)

        if n < 0:
            raise ValueError(f"n must be >= 0, got {n}")

        n = min(int(n), n1)
        self.current_mask_n = n
        return self.current_mask_n

    def configure_reset_start(
        self,
        *,
        ball: np.ndarray,
        left_players: np.ndarray,
        right_players: np.ndarray,
        body_angles: np.ndarray,
    ) -> None:
        ball = np.asarray(ball, dtype=np.float32)
        left_players = np.asarray(left_players, dtype=np.float32)
        right_players = np.asarray(right_players, dtype=np.float32)
        body_angles = np.asarray(body_angles, dtype=np.float32)

        if ball.shape not in [(2,), (4,)]:
            raise ValueError(
                f"ball must have shape (2,) or (4,), got {ball.shape}"
            )

        if left_players.ndim != 2 or left_players.shape[1] != 5:
            raise ValueError(
                f"left_players must have shape (n, 5), got {left_players.shape}"
            )
        if right_players.ndim != 2 or right_players.shape[1] != 5:
            raise ValueError(
                f"right_players must have shape (n, 5), got {right_players.shape}"
            )
        if left_players.shape[0] < self.config.n1:
            raise ValueError(
                f"left_players must contain at least {self.config.n1} players, got {left_players.shape[0]}"
            )
        if right_players.shape[0] < self.config.n2:
            raise ValueError(
                f"right_players must contain at least {self.config.n2} players, got {right_players.shape[0]}"
            )

        source_n1 = int(left_players.shape[0])
        source_n2 = int(right_players.shape[0])
        source_angles = source_n1 + source_n2
        if body_angles.shape != (source_angles,):
            raise ValueError(
                f"body_angles must have shape ({source_angles},), got {body_angles.shape}"
            )

        left_players = left_players[:self.config.n1]
        right_players = right_players[:self.config.n2]
        left_angles = body_angles[:source_n1][:self.config.n1]
        right_angles = body_angles[source_n1:source_angles][:self.config.n2]
        body_angles = np.concatenate([left_angles, right_angles], axis=0)
        left_players = left_players.copy()
        right_players = right_players.copy()
        left_players[:, 2] = left_angles
        right_players[:, 2] = right_angles

        self.CUSTOM_BALL = tuple(ball.tolist())
        self.CUSTOM_LEFT_PLAYERS = [tuple(row) for row in left_players.tolist()]
        self.CUSTOM_RIGHT_PLAYERS = [tuple(row) for row in right_players.tolist()]
        self.CUSTOM_BODY_ANGLES = body_angles.copy()

    def zero_player_velocities(self, players):
        return [(x, y, body, 0.0, 0.0) for x, y, body, vx, vy in players]

    def _get_default_players_for_current_config(self, kickoff_side: str = "left"):
        """
        Return benchmark default-reset positions for the requested kickoff side.

        The first natural kickoff is left to the server. Manual default resets use
        these templates so timeout-driven restarts alternate between left and right
        kickoff formations while preserving visual l=yellow/r=blue sides.
        """
        side = str(kickoff_side).strip().lower()
        if side not in ("left", "right"):
            raise ValueError(f"kickoff_side must be 'left' or 'right', got {kickoff_side!r}")

        is_3v3 = self.config.n1 == 3 and self.config.n2 == 3
        if is_3v3:
            if side == "right":
                return (
                    list(self.DEFAULT_3V3_RIGHT_KICKOFF_LEFT_PLAYERS),
                    list(self.DEFAULT_3V3_RIGHT_KICKOFF_RIGHT_PLAYERS),
                )
            return (
                list(self.DEFAULT_3V3_LEFT_PLAYERS),
                list(self.DEFAULT_3V3_RIGHT_PLAYERS),
            )

        if side == "right":
            left_players = list(self.DEFAULT_RIGHT_KICKOFF_LEFT_PLAYERS[:self.config.n1])
            right_players = list(self.DEFAULT_RIGHT_KICKOFF_RIGHT_PLAYERS[:self.config.n2])
        else:
            left_players = list(self.DEFAULT_LEFT_PLAYERS[:self.config.n1])
            right_players = list(self.DEFAULT_RIGHT_PLAYERS[:self.config.n2])
        return left_players, right_players

    def _get_default_body_angles_for_current_config(self, kickoff_side: str = "left"):
        """
        DEFAULT_BODY_ANGLES 的组织方式是：
        [left_1 ... left_11, right_1 ... right_11]

        当前 player_list 的组织方式是：
        team1 前 n1 人 + team2 前 n2 人

        所以这里只需要按当前 n1/n2 截取并拼接。
        """
        side = str(kickoff_side).strip().lower()
        if side not in ("left", "right"):
            raise ValueError(f"kickoff_side must be 'left' or 'right', got {kickoff_side!r}")

        if self.config.n1 == 3 and self.config.n2 == 3:
            source = (
                self.DEFAULT_3V3_RIGHT_KICKOFF_BODY_ANGLES
                if side == "right"
                else self.DEFAULT_3V3_BODY_ANGLES
            )
            out = np.asarray(source, dtype=np.float32)
            expected = self.config.n1 + self.config.n2
            if out.shape != (expected,):
                raise ValueError(
                    f"default 3v3 body angles shape mismatch: got {out.shape}, expected ({expected},)"
                )
            return out

        angles = np.asarray(
            self.DEFAULT_RIGHT_KICKOFF_BODY_ANGLES if side == "right" else self.DEFAULT_BODY_ANGLES,
            dtype=np.float32,
        )

        left_angles = angles[:11][:self.config.n1]
        right_angles = angles[11:22][:self.config.n2]

        out = np.concatenate([left_angles, right_angles], axis=0)

        expected = self.config.n1 + self.config.n2
        if out.shape != (expected,):
            raise ValueError(
                f"default body angles shape mismatch: got {out.shape}, expected ({expected},)"
            )

        return out

    def _get_custom_players_for_current_config(self):
        left_players = list(self.CUSTOM_LEFT_PLAYERS[:self.config.n1])
        right_players = list(self.CUSTOM_RIGHT_PLAYERS[:self.config.n2])
        return left_players, right_players

    def _get_custom_body_angles_for_current_config(self):
        angles = np.asarray(self.CUSTOM_BODY_ANGLES, dtype=np.float32)

        left_angles = angles[:11][:self.config.n1]
        right_angles = angles[11:22][:self.config.n2]

        out = np.concatenate([left_angles, right_angles], axis=0)

        expected = self.config.n1 + self.config.n2
        if out.shape != (expected,):
            raise ValueError(
                f"custom body angles shape mismatch: got {out.shape}, expected ({expected},)"
            )

        return out
