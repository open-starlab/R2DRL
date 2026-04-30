from __future__ import annotations

from typing import Dict, Tuple, List, Sequence
import numpy as np
from pathlib import Path
from .protocols import P
from . import ipc
import time
from collections import Counter, defaultdict

Flags = Tuple[int, int]

class Agents:

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
        self.coach_shm_id = coach_shm_id
        self.trainer_shm_id = trainer_shm_id
        self.player_shm_ids = dict(player_shm_ids)
        self.agent_mask = np.ones(self.config.n1, dtype=bool)
        self.current_mask_n = self.config.n1

        self.DEFAULT_BALL = (0.0, 0.0, 0.0, 0.0)
        self.DEFAULT_LEFT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = [
            (-49.7789, 0.0261, -0.03, 0.0, 0.0),
            (-9.1964, 4.7170, -27.154, 0.0, 0.0),
            (-9.1964, -4.7170, 27.154, 0.0, 0.0),
            (-12.2884, 5.0000, -22.141, 0.0, 0.0),
            (-12.2884, -5.0000, 22.141, 0.0, 0.0),
            (-11.1898, 15.5762, -54.307, 0.0, 0.0),
            (-11.1898, -15.5762, 54.307, 0.0, 0.0),
            (-7.9027, 8.0888, -45.667, 0.0, 0.0),
            (-7.9027, -8.0888, 45.667, 0.0, 0.0),
            (-2.1420, 11.7958, -79.708, 0.0, 0.0),
            (-2.1420, -11.7958, 79.708, 0.0, 0.0),
        ]
        self.DEFAULT_RIGHT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = [
            (49.7789, 0.0261, -179.970, 0.0, 0.0),
            (9.1964, 4.7170, -152.846, 0.0, 0.0),
            (9.1964, -4.7170, 152.846, 0.0, 0.0),
            (12.2884, 5.0000, -157.859, 0.0, 0.0),
            (12.2884, -5.0000, 157.859, 0.0, 0.0),
            (11.1898, 15.5762, -125.693, 0.0, 0.0),
            (11.1898, -15.5762, 125.693, 0.0, 0.0),
            (7.9027, 8.0888, -134.333, 0.0, 0.0),
            (7.9027, -8.0888, 134.333, 0.0, 0.0),
            (2.1420, 11.7958, -100.292, 0.0, 0.0),
            (2.1420, -11.7958, 100.292, 0.0, 0.0),
        ]
        self.DEFAULT_BODY_ANGLES = [
            -0.03,
            -27.154,
            27.154,
            -22.141,
            22.141,
            -54.307,
            54.307,
            -45.667,
            45.667,
            -79.708,
            79.708,
            -179.970,
            -152.846,
            152.846,
            -157.859,
            157.859,
            -125.693,
            125.693,
            -134.333,
            134.333,
            -100.292,
            100.292,
        ]
        self.DEFAULT_3V3_LEFT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = [
            (-49.7789, 0.0261, -0.03, 0.0, 0.0),
            (-12.2884, 0.0, 0.0, 0.0, 0.0),
            (-2.1420, 0.0, 0.0, 0.0, 0.0),
        ]
        self.DEFAULT_3V3_RIGHT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] = [
            (49.7789, 0.0261, -179.970, 0.0, 0.0),
            (12.2884, 0.0, -180.0, 0.0, 0.0),
            (2.1420, 0.0, -180.0, 0.0, 0.0),
        ]
        self.DEFAULT_3V3_BODY_ANGLES = [
            -0.03,
            0.0,
            0.0,
            -179.970,
            -180.0,
            -180.0,
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

        self._obs_buf = np.empty((len(self.player_list), P.player.STATE_NUM),dtype=np.float32,)
        self._mask_buf = np.empty((len(self.player_list), self.n_actions),dtype=np.int32,)
        
        self.kickable = False
        self.opponent_kickable = False
        self.ball_owner_team = 0
        self.kickable_threshold = 1.085

        self.epv_grid = self._load_epv_grid() if self.config.useMaxEpv else None
        self.current_epv = 0.0
        self.max_episode_epv = 0.0

    def get_player(self, team: int, unum: int) -> P.player.Player:
        return self.players[(team, unum)]

    def all_players(self) -> List[P.player.Player]:
        return self.player_list

    def request(self, target: str) -> None:
        """
        Send REQUEST signal to:
        - "trainer"
        - "player:team:unum"
        """

        if target == "trainer":
            self.trainer.write_request()

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
            player.write_request()

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
        epv_path = (
            Path(__file__).resolve().parent
            / "LaurieOnTracking"
            / "EPV_grid.csv"
        )
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
        field_length = float(self.config.half_length) * 2.0
        field_width = float(self.config.half_width) * 2.0

        if abs(x) > field_length / 2.0 or abs(y) > field_width / 2.0:
            return 0.0

        epv = self.epv_grid
        if attack_direction == -1:
            epv = np.fliplr(epv)

        ny, nx = epv.shape
        dx = field_length / float(nx)
        dy = field_width / float(ny)
        ix = int((x + field_length / 2.0 - 1e-4) / dx)
        iy = int((y + field_width / 2.0 - 1e-4) / dy)
        ix = min(max(ix, 0), nx - 1)
        iy = min(max(iy, 0), ny - 1)
        return float(epv[iy, ix])

    def reset_episode_epv(self) -> float:
        if not self.config.useMaxEpv:
            self.current_epv = 0.0
            self.max_episode_epv = 0.0
            return self.max_episode_epv
        self.current_epv = self.get_ball_position_epv()
        self.max_episode_epv = self.current_epv
        return self.max_episode_epv

    def update_episode_epv(self) -> float:
        if not self.config.useMaxEpv or not self.kickable:
            return self.max_episode_epv

        self.current_epv = self.get_ball_position_epv()
        self.max_episode_epv = max(self.max_episode_epv, self.current_epv)
        return self.max_episode_epv

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
        for i, p in enumerate(self.player_list):
            if self.config.team1 == "hybrid":
                self._mask_buf[i] = p.hybrid_mask()
            else:
                self._mask_buf[i] = p.base_mask()

        return self._mask_buf

    def write_base_actions(self, actions: np.ndarray):
        n1 = self.config.n1

        for idx, p in enumerate(self.player_list):
            if idx < n1:
                act = int(actions[idx])
                # act = int(p.default_base_action)
            else:
                act = int(p.default_base_action)
            
            p.write_base_action(act)
            p.write_request()

    def write_hybrid_actions(self, actions: np.ndarray):
        n1 = self.config.n1

        for idx, p in enumerate(self.player_list):
            if idx < n1:
                a  = int(actions[idx, 0])
                u0 = float(actions[idx, 1])
                u1 = float(actions[idx, 2])
            else:
                a, u0, u1 = p.default_hybrid_action
                a = int(a)
                u0 = float(u0)
                u1 = float(u1)

            p.write_hybrid_action(a, u0, u1)
            p.write_request()

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
        rescue_cooldown: float = 0.5,
    ):

        t_end = time.monotonic() + float(timeout)

        last_dist = None
        last_change_t = time.monotonic()
        last_rescue_t = 0.0

        total = len(self.player_list) + 1  # players + trainer

        while True:

            now = time.monotonic()

            pairs = []
            ready_entities = []

            # ---- check players ----
            for p in self.player_list:
                a, b = p.read_flags()
                ab = (int(a), int(b))
                pairs.append(ab)

                if ab == P.common.FLAG_READY:
                    ready_entities.append(("player", p))

            # ---- check trainer ----
            ta, tb = self.trainer.flags()
            tab = (int(ta), int(tb))
            pairs.append(tab)

            if tab == P.common.FLAG_READY:
                ready_entities.append(("trainer", self.trainer))

            # ---- all READY ----
            if len(ready_entities) == total:
                return True

            # ---- timeout ----
            if now >= t_end:
                if self.log:
                    self.log.info(
                        f"[wait_all_ready] timeout dist={dict(Counter(pairs))}"
                    )
                return False

            # ---- steady detection ----
            dist = Counter(pairs)
            if dist != last_dist:
                last_dist = dist
                last_change_t = now

            stuck = (now - last_change_t) >= float(stuck_window)

            # ---- rescue ----
            if stuck and ready_entities and (now - last_rescue_t) >= rescue_cooldown:

                pushed = 0

                for kind, obj in ready_entities:

                    if kind == "trainer":
                        obj.noop()
                    else:
                        obj.take_default_action(is_hybrid=(self.config.team1 == "hybrid"))
                        # if self.log:
                        #     self.log.info(
                        #     f"[rescue] dist={self.read_all_flags()}"
                            # )

                    pushed += 1

                last_rescue_t = now
                last_change_t = now

            time.sleep(float(poll))

    def write_all_body_targets(self, angles: np.ndarray):
        if len(angles) != len(self.player_list):
            raise ValueError(
                f"angles size mismatch: got {len(angles)}, expected {len(self.player_list)}"
            )
        for idx, p in enumerate(self.player_list):
            p.write_body_target_deg(float(angles[idx]))
            
    def reset_default(self) -> None:
        default_left_players, default_right_players = self._get_default_players_for_current_config()
        default_body_angles = self._get_default_body_angles_for_current_config()

        self.trainer.reset_players_and_ball(
            self.DEFAULT_BALL,
            default_left_players,
            default_right_players,
        )
        self.write_all_body_targets(default_body_angles)

        is_hybrid = (self.config.team1 == "hybrid")
        for p in self.player_list:
            p.take_empty_action(is_hybrid=is_hybrid)
        self.wait_all_ready()

    def reset_custom(self) -> None:
        custom_left_players = self.CUSTOM_LEFT_PLAYERS
        custom_right_players = self.CUSTOM_RIGHT_PLAYERS
        custom_body_angles = self.CUSTOM_BODY_ANGLES


        custom_left_players_zero = self.zero_player_velocities(custom_left_players)
        custom_right_players_zero = self.zero_player_velocities(custom_right_players)

        for i in range(5):
            self.trainer.reset_players_and_ball(
                self.CUSTOM_BALL,
                custom_left_players_zero,
                custom_right_players_zero,
            )
            self.write_all_body_targets(custom_body_angles)

            is_hybrid = (self.config.team1 == "hybrid")
            for p in self.player_list:
                p.take_empty_action(is_hybrid=is_hybrid)
            self.wait_all_ready()

        self.trainer.reset_players_and_ball(
            self.CUSTOM_BALL,
            custom_left_players,
            custom_right_players,
        )
        self.write_all_body_targets(custom_body_angles)

        is_hybrid = (self.config.team1 == "hybrid")
        for p in self.player_list:
            p.take_empty_action(is_hybrid=is_hybrid)
        self.wait_all_ready()

    def read_all_flags(self, include_cycles: bool = False) -> Dict[str, object]:
        who: Dict[str, object] = {}
        groups: Dict[Flags, List[str]] = defaultdict(list)
        pairs: List[Flags] = []

        # ---- players ----
        for i, p in enumerate(self.player_list):
            ab: Flags = tuple(map(int, p.read_flags()))
            name = f"player:{i}"
            pairs.append(ab)
            groups[ab].append(name)

            if include_cycles:
                try:
                    cyc = int(p.cycle())
                except Exception:
                    cyc = None
                who[name] = {"flags": ab, "cycle": cyc}
            else:
                who[name] = ab

        # ---- trainer ----
        tab: Flags = tuple(map(int, self.trainer.flags()))
        pairs.append(tab)
        groups[tab].append("trainer")
        who["trainer"] = {"flags": tab, "cycle": None} if include_cycles else tab

        return {
            "who": who,
            "groups": dict(groups),
            "dist": Counter(pairs),
            "trainer": tab,
        }

    def get_team1_obs(self, norm: bool = True, zero_inactive: bool = True):
        full_obs = self.obs(norm=norm)
        out = full_obs[:self.config.n1].copy()

        if zero_inactive:
            inactive_idx = np.flatnonzero(~self.agent_mask)
            for i in inactive_idx:
                out[i].fill(0.0)

        return out
    
    def get_team1_avail_actions(self):
        
        full_mask = self.avail_actions()
        out = full_mask[:self.config.n1].copy()
        inactive_idx = np.flatnonzero(~self.agent_mask)

        if inactive_idx.size > 0:
            out[inactive_idx] = 0
            for idx in inactive_idx:
                if self.config.team1 == "hybrid":
                    default_a = int(self.player_list[idx].default_hybrid_action[0])
                else:
                    default_a = int(self.player_list[idx].default_base_action)
                out[idx, default_a] = 1

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

        opp_x = team2_players[:, 0]
        opp_y = team2_players[:, 1]
        opp_dists = np.sqrt((opp_x - bx) ** 2 + (opp_y - by) ** 2)
        team2_min_dist = float(np.min(opp_dists)) if opp_dists.size > 0 else np.inf

        self.kickable = bool(team1_min_dist <= self.kickable_threshold)
        self.opponent_kickable = bool(team2_min_dist <= self.kickable_threshold)

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

        if left_players.shape != (self.config.n1, 5):
            raise ValueError(
                f"left_players must have shape ({self.config.n1}, 5), got {left_players.shape}"
            )

        if right_players.shape != (self.config.n2, 5):
            raise ValueError(
                f"right_players must have shape ({self.config.n2}, 5), got {right_players.shape}"
            )

        expected_angles = self.config.n1 + self.config.n2
        if body_angles.shape != (expected_angles,):
            raise ValueError(
                f"body_angles must have shape ({expected_angles},), got {body_angles.shape}"
            )

        self.CUSTOM_BALL = tuple(ball.tolist())
        self.CUSTOM_LEFT_PLAYERS = [tuple(row) for row in left_players.tolist()]
        self.CUSTOM_RIGHT_PLAYERS = [tuple(row) for row in right_players.tolist()]
        self.CUSTOM_BODY_ANGLES = body_angles.copy()

    def zero_player_velocities(self, players):
        return [(x, y, body, 0.0, 0.0) for x, y, body, vx, vy in players]
    
    def _get_default_players_for_current_config(self):
        """
        从固定 11v11 默认模板中，截取当前 n1/n2 需要的真实球员。
        """
        if self.config.n1 == 3 and self.config.n2 == 3:
            return (
                list(self.DEFAULT_3V3_LEFT_PLAYERS),
                list(self.DEFAULT_3V3_RIGHT_PLAYERS),
            )

        left_players = list(self.DEFAULT_LEFT_PLAYERS[:self.config.n1])
        right_players = list(self.DEFAULT_RIGHT_PLAYERS[:self.config.n2])
        return left_players, right_players

    def _get_default_body_angles_for_current_config(self):
        """
        DEFAULT_BODY_ANGLES 的组织方式是：
        [left_1 ... left_11, right_1 ... right_11]

        当前 player_list 的组织方式是：
        team1 前 n1 人 + team2 前 n2 人

        所以这里只需要按当前 n1/n2 截取并拼接。
        """
        if self.config.n1 == 3 and self.config.n2 == 3:
            out = np.asarray(self.DEFAULT_3V3_BODY_ANGLES, dtype=np.float32)
            expected = self.config.n1 + self.config.n2
            if out.shape != (expected,):
                raise ValueError(
                    f"default 3v3 body angles shape mismatch: got {out.shape}, expected ({expected},)"
                )
            return out

        angles = np.asarray(self.DEFAULT_BODY_ANGLES, dtype=np.float32)

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
