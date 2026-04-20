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

        self.DEFAULT_BALL =  (0.0, 0.0, 0.0, 0.0)
        self.DEFAULT_LEFT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] =  [(-49.4, 0.0, 91.653, 0.0, 0.0), (-15.1283, -4.5671, 16.782, 0.0065, 0.0017), (-16.162, 3.3026, -11.42, 0.0003, 0.0), (-10.5396, -14.8081, 45.641, 0.3438, -0.1142), (-11.6838, 13.1449, -48.203, 0.0615, 0.0175), (-9.6667, 0.05, 0.187, 1.0, 0.0032), (-3.4658, -4.1658, 71.133, 0.319, 0.9212), (-2.6538, 6.4602, 4.786, 0.9367, 0.0785), (-2.4045, -18.7443, -34.77, 0.8215, -0.5702), (-3.8442, 19.2776, 43.659, 0.7235, 0.6903), (-4.202, 0.0046, -1.803, 0.1202, -0.0022)]
        self.DEFAULT_RIGHT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] =  [(49.7789, 0.0261, -88.469, 0.0, 0.0), (12.2884, 5.0, -67.727, -0.0035, 0.0), (12.2555, -5.0, 67.876, -0.011, 0.0), (11.1898, 15.5762, -36.079, -0.0592, 0.024), (11.6398, -14.0389, -160.873, -0.0245, -0.0075), (12.0492, 0.4159, -87.745, -0.0255, 0.0007), (7.9027, 8.0888, -43.974, 0.0, 0.0), (8.0057, -8.2308, 43.315, 0.0, 0.0), (2.142, 11.7958, -9.925, -0.0103, 0.0015), (2.4851, -12.012, 11.958, -0.0082, -0.0013), (9.1964, 4.717, -60.697, -0.002, 0.0035)]
        self.DEFAULT_BODY_ANGLES =  [91.653, 16.782, -11.42, 45.641, -48.203, 0.187, 71.133, 4.786, -34.77, 43.659, -1.803, -88.469, -67.727, 67.876, -36.079, -160.873, -87.745, -43.974, 43.315, -9.925, 11.958, -60.697]
        
        self.CUSTOM_BALL =  (-13.1491, 19.7303, 0.4673, 0.561)
        self.CUSTOM_LEFT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] =  [(-49.8471, 5.9114, 90.356, 0.0, 0.0), (-23.1647, 3.9302, 1.776, 0.633, 0.0055), (-28.0549, 13.5735, 10.179, 0.9778, 0.1767), (-23.6264, -7.5945, -10.685, 0.9828, -0.1855), (-29.2598, 20.0184, 25.46, 0.1445, 0.0687), (-21.7763, 9.0218, 25.515, 0.87, 0.4077), (-17.4081, -4.1642, -10.551, 0.977, -0.1805), (-18.63, 18.1475, 2.413, 0.9928, 0.0432), (-7.6974, -18.9974, -18.357, 0.0057, -0.002), (-14.6238, 18.5795, 49.841, 0.557, 0.6262), (-1.8707, 8.3827, 12.983, 0.23, 0.1507)]
        self.CUSTOM_RIGHT_PLAYERS: Sequence[Tuple[float, float, float, float, float]] =  [(49.7789, 0.0, 90.607, 0.0, 0.0), (7.2389, 14.1914, 165.824, 0.0735, -0.0617), (5.6635, -0.2856, 136.878, 0.1797, 0.2612), (0.7673, 24.3103, 5.875, 0.5982, 0.0612), (-1.1262, -12.1448, 85.99, 0.2405, 0.019), (-5.0331, 9.6253, 48.578, 0.625, 0.4938), (-17.194, 18.044, 7.491, 0.985, 0.1307), (-17.451, -2.6793, 53.843, 0.5495, 0.6667), (-27.949, 26.1856, 3.103, 0.749, 0.0835), (-26.6065, -17.2931, -31.572, 0.852, -0.5235), (-24.2801, 13.6549, 12.76, 0.9752, 0.2208)]
        self.CUSTOM_BODY_ANGLES =  [90.356, 1.776, 10.179, -10.685, 25.46, 25.515, -10.551, 2.413, -18.357, 49.841, 12.983, 90.607, 165.824, 136.878, 5.875, 85.99, 48.578, 7.491, 53.843, 3.103, -31.572, 12.76]
        
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
        self.kickable_threshold = 1.085

        self.epv_grid = self._load_epv_grid()
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
        self.current_epv = self.get_ball_position_epv()
        self.max_episode_epv = self.current_epv
        return self.max_episode_epv

    def update_episode_epv(self) -> float:
        if not self.kickable:
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

        px = team1_players[:, 0]
        py = team1_players[:, 1]

        dists = np.sqrt((px - bx) ** 2 + (py - by) ** 2)
        sorted_idx = np.argsort(dists)
        nearest_idx = sorted_idx[:n]

        self.agent_mask[:] = False
        self.agent_mask[nearest_idx] = True

        if len(sorted_idx) > 0:
            nearest_dist = float(dists[sorted_idx[0]])
            self.kickable = (nearest_dist <= self.kickable_threshold)
        else:
            self.kickable = False

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
