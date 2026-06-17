from __future__ import annotations

import os
import random
from typing import Optional


import numpy as np


def _resolve_trajectory_path(base_dir: str, trajectory_path):
    if trajectory_path is None or str(trajectory_path).strip() == "":
        raise ValueError("trajectory_path must be provided in env_args")

    trajectory_path = str(trajectory_path)
    if os.path.isabs(trajectory_path):
        return trajectory_path

    if os.path.dirname(trajectory_path):
        return os.path.abspath(os.path.join(base_dir, trajectory_path))

    return os.path.join(base_dir, "trajectories", trajectory_path)


def _normalize_start_id(start_id):
    if start_id is None:
        return None
    start_id = str(start_id).strip()
    return None if start_id == "" else start_id


def _normalize_start_index(start_index):
    if start_index is None:
        return None
    start_index = int(start_index)
    if start_index < 0:
        raise ValueError(f"start_index must be >= 0, got {start_index}")
    return start_index


def _select_catalog_indices(data, num_traj: int, start_id, start_index) -> np.ndarray:
    if num_traj <= 0:
        return np.asarray([], dtype=np.int32)

    if start_index is not None:
        if start_index >= num_traj:
            raise IndexError(
                f"start_index={start_index} out of range for num_traj={num_traj}"
            )
        return np.asarray([start_index], dtype=np.int32)

    if start_id is not None:
        if "scenario_ids" not in data:
            raise ValueError("start_id was set, but trajectory npz has no scenario_ids")
        scenario_ids = np.asarray(data["scenario_ids"]).astype(str).reshape(-1)
        if scenario_ids.shape[0] != num_traj:
            raise ValueError(
                f"scenario_ids length must match num_traj={num_traj}, "
                f"got {scenario_ids.shape[0]}"
            )
        matches = np.flatnonzero(scenario_ids == start_id)
        if matches.size == 0:
            available = ", ".join(scenario_ids.tolist())
            raise KeyError(
                f"start_id={start_id!r} not found in scenario_ids. "
                f"Available: {available}"
            )
        if matches.size > 1:
            raise ValueError(f"start_id={start_id!r} appears more than once")
        return matches.astype(np.int32)

    return np.arange(num_traj, dtype=np.int32)


def get_catalog_launch_profile(
    *,
    trajectory_path: str,
    start_id: Optional[str] = None,
    start_index: Optional[int] = None,
):
    base_dir = os.path.dirname(os.path.abspath(__file__))
    traj_path = _resolve_trajectory_path(base_dir, trajectory_path)
    data = np.load(traj_path)
    traj_offsets = data["traj_offsets"]
    num_traj = len(traj_offsets) - 1
    start_id = _normalize_start_id(start_id)
    start_index = _normalize_start_index(start_index)
    selected = _select_catalog_indices(data, num_traj, start_id, start_index)

    if selected.size == 0:
        raise ValueError("no catalog scenario selected")

    if "n_control" in data:
        n_controls = np.asarray(data["n_control"], dtype=np.int32).reshape(-1)
    else:
        n_controls = None

    if "effective_attackers" in data:
        attackers = np.asarray(data["effective_attackers"], dtype=np.int32).reshape(-1)
    elif n_controls is not None:
        attackers = n_controls
    else:
        return None

    if "effective_defenders" in data:
        defenders = np.asarray(data["effective_defenders"], dtype=np.int32).reshape(-1)
    else:
        return None

    selected_attackers = attackers[selected]
    selected_defenders = defenders[selected]
    selected_controls = n_controls[selected] if n_controls is not None else selected_attackers

    n_control = int(np.max(selected_controls))
    effective_attackers = int(np.max(selected_attackers))
    effective_defenders = int(np.max(selected_defenders))
    launch_n1 = max(1, effective_attackers)
    # rcssserver/trainer needs a right-side team to complete reset handshakes.
    # 1v0 therefore keeps one frozen dummy right player, with no active right unums.
    launch_n2 = max(1, effective_defenders)
    launch_n = max(1, launch_n1, launch_n2)

    return {
        "trajectory_path": traj_path,
        "selected_indices": [int(i) for i in selected.tolist()],
        "n": launch_n,
        "n1": launch_n1,
        "n2": launch_n2,
        "init_n": n_control,
        "effective_attackers": effective_attackers,
        "effective_defenders": effective_defenders,
        "active_right_unums": list(range(1, effective_defenders + 1)),
    }


def apply_catalog_launch_profile(env_args: dict):
    if not env_args or env_args.get("_scenario_catalog_launch_profile_applied"):
        return None
    if not bool(env_args.get("start_catalog_enabled", False)):
        return None
    if not env_args.get("trajectory_path"):
        return None
    if env_args.get("scenario_launch_from_catalog", True) is False:
        return None

    profile = get_catalog_launch_profile(
        trajectory_path=env_args.get("trajectory_path"),
        start_id=env_args.get("start_id"),
        start_index=env_args.get("start_index"),
    )
    if profile is None:
        return None

    old = {
        "n": env_args.get("n"),
        "n1": env_args.get("n1"),
        "n2": env_args.get("n2"),
        "init_n": env_args.get("init_n"),
        "active_right_unums": env_args.get("active_right_unums"),
    }
    env_args["n"] = int(profile["n"])
    env_args["n1"] = int(profile["n1"])
    env_args["n2"] = int(profile["n2"])
    env_args["init_n"] = int(profile["init_n"])
    env_args["active_right_unums"] = list(profile["active_right_unums"])
    env_args["freeze_non_controlled"] = True
    env_args["_scenario_catalog_launch_profile_applied"] = True
    env_args["_scenario_catalog_launch_profile"] = profile

    print(
        "[ScenarioLaunch] catalog profile "
        f"start_id={env_args.get('start_id')!r} "
        f"n {old['n']}->{env_args['n']} "
        f"n1 {old['n1']}->{env_args['n1']} "
        f"n2 {old['n2']}->{env_args['n2']} "
        f"init_n {old['init_n']}->{env_args['init_n']} "
        f"active_right {old['active_right_unums']}->{env_args['active_right_unums']}"
    )
    return profile


class ScenarioStartSampler:
    PREFILTERED_TRAJECTORY_TAG = "_right_half_left_nearest_kickable"
    KICKABLE_THRESHOLD = 1.085

    def __init__(
        self,
        *,
        init_n: int,
        start_window_size: int = 1,
        progress_bucket_count: int = 100,
        current_target_window_size: int = 1,
        num_selected_trajectories: Optional[int] = 10,
        random_sample: bool = False,
        trajectory_path: str,
        scenario_difficulty: Optional[str] = None,
        scenario_difficulty_buckets: Optional[dict] = None,
        start_catalog_enabled: bool = False,
        start_id: Optional[str] = None,
        start_index: Optional[int] = None,
    ):
        self.current_n = int(init_n)
        self.start_window_size = int(start_window_size)
        self.progress_bucket_count = int(progress_bucket_count)
        self.current_target_window_size = int(current_target_window_size)
        self.kickable_threshold = float(self.KICKABLE_THRESHOLD)
        self.num_selected_trajectories = (
            None if num_selected_trajectories is None else int(num_selected_trajectories)
        )
        self.random_sample = bool(random_sample)
        self.start_catalog_enabled = bool(start_catalog_enabled)
        self.start_id = self._normalize_start_id(start_id)
        self.start_index = self._normalize_start_index(start_index)
        if self.start_id is not None and self.start_index is not None:
            raise ValueError("start_id and start_index cannot both be set")
        self.scenario_difficulty = self._normalize_scenario_difficulty(
            scenario_difficulty
        )
        self.scenario_difficulty_buckets = self._normalize_scenario_difficulty_buckets(
            scenario_difficulty_buckets
        )

        base_dir = os.path.dirname(os.path.abspath(__file__))
        traj_path = self._resolve_trajectory_path(base_dir, trajectory_path)
        self.trajectory_n_controls = []
        self.trajectories, self.traj_max_frame = self._load_trajectory_array(traj_path)
        self.max_n = self.n_players
        if self.current_n > self.max_n:
            self.current_n = self.max_n
        self.trajectory_n_controls = [
            min(int(n), self.max_n) for n in self.trajectory_n_controls
        ]

        self.bucket_idx = self._scenario_bucket_idx(traj_path)
        mode = "catalog" if self.start_catalog_enabled else "trajectory"
        print(
            "[ScenarioStart] "
            f"mode={mode}, "
            f"trajectory_path={traj_path}, "
            f"start_id={self.start_id!r}, "
            f"start_index={self.start_index!r}, "
            f"scenario_difficulty={self.scenario_difficulty}, "
            f"bucket={self.bucket_idx}, n_control={self.trajectory_n_controls}"
        )

    def sample_start_and_n(self):
        traj_id = random.randrange(len(self.trajectories))
        frame_start, frame_end = self._frame_range_for_bucket(traj_id, self.bucket_idx)
        frame_idx = random.randint(int(frame_start), int(frame_end))

        traj_len, frames = self.trajectories[traj_id]
        if frame_idx < 0 or frame_idx >= traj_len:
            raise IndexError(
                f"invalid frame_idx={frame_idx} for traj_id={traj_id}, traj_len={traj_len}"
            )

        start_idx = max(0, int(frame_idx) - self.start_window_size + 1)
        sampled_frame_idx = random.randint(start_idx, int(frame_idx))
        return frames[sampled_frame_idx], int(self.trajectory_n_controls[traj_id])

    def _resolve_trajectory_path(self, base_dir: str, trajectory_path):
        if trajectory_path is None or str(trajectory_path).strip() == "":
            raise ValueError("trajectory_path must be provided in env_args")

        trajectory_path = str(trajectory_path)
        if os.path.isabs(trajectory_path):
            return trajectory_path

        if os.path.dirname(trajectory_path):
            return os.path.abspath(os.path.join(base_dir, trajectory_path))

        return os.path.join(base_dir, "trajectories", trajectory_path)

    def _normalize_scenario_difficulty(self, difficulty):
        if difficulty is None:
            return None
        difficulty = str(difficulty).strip().lower()
        if difficulty == "":
            return None
        if difficulty == "middle":
            difficulty = "medium"
        if difficulty not in ("easy", "medium", "hard"):
            raise ValueError(
                "scenario_difficulty must be one of: easy, medium/middle, hard; "
                f"got {difficulty!r}"
            )
        return difficulty

    def _normalize_scenario_difficulty_buckets(self, buckets):
        if buckets is None:
            return None
        if not isinstance(buckets, dict):
            raise TypeError("scenario_difficulty_buckets must be a YAML mapping")

        normalized = {}
        for name, bucket_idx in buckets.items():
            name = self._normalize_scenario_difficulty(name)
            if name is not None:
                normalized[name] = int(bucket_idx)
        return normalized

    def _normalize_start_id(self, start_id):
        if start_id is None:
            return None
        start_id = str(start_id).strip()
        if start_id == "":
            return None
        return start_id

    def _normalize_start_index(self, start_index):
        if start_index is None:
            return None
        start_index = int(start_index)
        if start_index < 0:
            raise ValueError(f"start_index must be >= 0, got {start_index}")
        return start_index

    def _scenario_bucket_idx(self, trajectory_path: str):
        if self.scenario_difficulty is None:
            return 0
        if self.n_players != 3:
            raise ValueError(
                "scenario_difficulty windows are defined only for 3v3 "
                f"trajectories; got {self.n_players} players per side from {trajectory_path}"
            )
        if not self.scenario_difficulty_buckets:
            raise ValueError(
                "scenario_difficulty_buckets must be provided in the YAML when "
                "scenario_difficulty is set"
            )
        if self.scenario_difficulty not in self.scenario_difficulty_buckets:
            raise ValueError(
                f"scenario_difficulty={self.scenario_difficulty!r} has no bucket in "
                "scenario_difficulty_buckets"
            )
        bucket_idx = int(self.scenario_difficulty_buckets[self.scenario_difficulty])
        if not (0 <= bucket_idx < self.progress_bucket_count):
            raise ValueError(
                f"scenario_difficulty bucket must be in [0, {self.progress_bucket_count - 1}], "
                f"got {bucket_idx}"
            )
        return bucket_idx

    def _frame_range_for_bucket(self, traj_id: int, bucket_idx: int):
        max_frame = int(self.traj_max_frame[traj_id])
        if max_frame <= 0:
            return 0, 0

        left_ratio = float(bucket_idx) / float(self.progress_bucket_count)
        right_ratio = float(bucket_idx + self.current_target_window_size) / float(
            self.progress_bucket_count
        )

        frame_end = int(np.floor((1.0 - left_ratio) * max_frame))
        frame_start = int(np.ceil((1.0 - right_ratio) * max_frame))
        frame_start = int(np.clip(frame_start, 0, max_frame))
        frame_end = int(np.clip(frame_end, 0, max_frame))
        if frame_start > frame_end:
            frame_start = frame_end
        return frame_start, frame_end

    def _load_trajectory_array(self, trajectory_path: str):
        data = np.load(trajectory_path)
        states = data["states"]
        traj_offsets = data["traj_offsets"]

        if states.ndim != 2:
            raise ValueError(f"states must be 2D, got shape={states.shape}")

        self.n_players = self._infer_n_players_from_state_dim(int(states.shape[1]))
        num_traj = len(traj_offsets) - 1
        if self.start_catalog_enabled:
            selected_indices = self._select_catalog_indices(data, num_traj)
        else:
            selected_indices = self._select_trajectory_indices(num_traj)

        n_controls = None
        if "n_control" in data:
            n_controls = np.asarray(data["n_control"], dtype=np.int32).reshape(-1)
            if n_controls.shape[0] != num_traj:
                raise ValueError(
                    f"n_control length must match num_traj={num_traj}, "
                    f"got {n_controls.shape[0]}"
                )

        effective_attackers = None
        effective_defenders = None
        ball_owner_left_unums = None
        if self.start_catalog_enabled:
            if "effective_attackers" in data and "effective_defenders" in data:
                effective_attackers = np.asarray(
                    data["effective_attackers"], dtype=np.int32
                ).reshape(-1)
                effective_defenders = np.asarray(
                    data["effective_defenders"], dtype=np.int32
                ).reshape(-1)
                if effective_attackers.shape[0] != num_traj:
                    raise ValueError(
                        f"effective_attackers length must match num_traj={num_traj}, "
                        f"got {effective_attackers.shape[0]}"
                    )
                if effective_defenders.shape[0] != num_traj:
                    raise ValueError(
                        f"effective_defenders length must match num_traj={num_traj}, "
                        f"got {effective_defenders.shape[0]}"
                    )
                if "ball_owner_left_unum" in data:
                    ball_owner_left_unums = np.asarray(
                        data["ball_owner_left_unum"], dtype=np.int32
                    ).reshape(-1)
                    if ball_owner_left_unums.shape[0] != num_traj:
                        raise ValueError(
                            f"ball_owner_left_unum length must match num_traj={num_traj}, "
                            f"got {ball_owner_left_unums.shape[0]}"
                        )

        trajectories = []
        traj_max_frame = []
        for traj_id in selected_indices:
            start = int(traj_offsets[traj_id])
            end = int(traj_offsets[traj_id + 1])
            frames = []
            for global_idx in range(start, end):
                frame = self._decode_frame_vector(states[global_idx])
                if effective_attackers is not None and effective_defenders is not None:
                    owner_unum = None
                    if ball_owner_left_unums is not None:
                        owner_unum = int(ball_owner_left_unums[int(traj_id)])
                    frame = self._remap_catalog_frame(
                        frame,
                        effective_attackers=int(effective_attackers[int(traj_id)]),
                        effective_defenders=int(effective_defenders[int(traj_id)]),
                        ball_owner_left_unum=owner_unum,
                    )
                frames.append(frame)
            if (not self.start_catalog_enabled) and (not self._trajectory_is_prefiltered(trajectory_path)):
                frames = self._filter_frames_for_scenario_start(frames)
            if not frames:
                continue
            trajectories.append((len(frames), frames))
            traj_max_frame.append(max(0, len(frames) - 3))
            if self.start_catalog_enabled and n_controls is not None:
                n_control = int(n_controls[int(traj_id)])
            else:
                n_control = int(self.current_n)
            if n_control < 1:
                raise ValueError(f"n_control must be >= 1, got {n_control}")
            self.trajectory_n_controls.append(n_control)

        if not trajectories:
            raise ValueError("all trajectories became empty after start-frame filtering")

        return trajectories, np.asarray(traj_max_frame, dtype=np.int32)

    def _select_catalog_indices(self, data, num_traj: int) -> np.ndarray:
        if num_traj <= 0:
            return np.asarray([], dtype=np.int32)

        if self.start_index is not None:
            if self.start_index >= num_traj:
                raise IndexError(
                    f"start_index={self.start_index} out of range for num_traj={num_traj}"
                )
            return np.asarray([self.start_index], dtype=np.int32)

        if self.start_id is not None:
            if "scenario_ids" not in data:
                raise ValueError("start_id was set, but trajectory npz has no scenario_ids")
            scenario_ids = np.asarray(data["scenario_ids"]).astype(str).reshape(-1)
            if scenario_ids.shape[0] != num_traj:
                raise ValueError(
                    f"scenario_ids length must match num_traj={num_traj}, "
                    f"got {scenario_ids.shape[0]}"
                )
            matches = np.flatnonzero(scenario_ids == self.start_id)
            if matches.size == 0:
                available = ", ".join(scenario_ids.tolist())
                raise KeyError(
                    f"start_id={self.start_id!r} not found in scenario_ids. "
                    f"Available: {available}"
                )
            if matches.size > 1:
                raise ValueError(f"start_id={self.start_id!r} appears more than once")
            return matches.astype(np.int32)

        return np.arange(num_traj, dtype=np.int32)

    def _trajectory_is_prefiltered(self, trajectory_path: str) -> bool:
        filename = os.path.basename(str(trajectory_path))
        return self.PREFILTERED_TRAJECTORY_TAG in filename

    def _select_trajectory_indices(self, num_traj: int) -> np.ndarray:
        if num_traj <= 0:
            return np.asarray([], dtype=np.int32)
        requested = self.num_selected_trajectories
        if requested is None or requested <= 0 or requested >= num_traj:
            return np.arange(num_traj, dtype=np.int32)
        if self.random_sample:
            return np.sort(np.asarray(random.sample(range(num_traj), requested), dtype=np.int32))
        return np.arange(requested, dtype=np.int32)

    def _select_catalog_active_indices(
        self,
        players: np.ndarray,
        count: int,
        *,
        ball_xy: np.ndarray,
        force_unum: Optional[int] = None,
    ):
        count = max(0, int(count))
        if count <= 0:
            return []
        count = min(count, int(players.shape[0]))
        dists = np.linalg.norm(players[:, :2] - ball_xy.reshape(1, 2), axis=1)

        selected = []
        if force_unum is not None:
            force_idx = int(force_unum) - 1
            if 0 <= force_idx < int(players.shape[0]):
                selected.append(force_idx)

        for idx in np.argsort(dists):
            idx = int(idx)
            if idx not in selected:
                selected.append(idx)
            if len(selected) >= count:
                break

        return sorted(selected[:count])

    def _catalog_corner_player(self, side: str, pad_idx: int):
        y = 32.0 - 3.0 * (max(0, int(pad_idx)) % 5)
        if side == "left":
            return np.asarray([-51.0, y, 0.0, 0.0, 0.0], dtype=np.float32)
        return np.asarray([51.0, y, 180.0, 0.0, 0.0], dtype=np.float32)

    def _remap_catalog_frame(
        self,
        frame: dict,
        *,
        effective_attackers: int,
        effective_defenders: int,
        ball_owner_left_unum: Optional[int] = None,
    ):
        ball = np.asarray(frame["ball"], dtype=np.float32)
        left_players = np.asarray(frame["left_players"], dtype=np.float32)
        right_players = np.asarray(frame["right_players"], dtype=np.float32)
        max_active = max(1, int(effective_attackers), int(effective_defenders))
        ball_xy = ball[:2].astype(np.float32)

        left_active = self._select_catalog_active_indices(
            left_players,
            int(effective_attackers),
            ball_xy=ball_xy,
            force_unum=ball_owner_left_unum,
        )
        right_active = self._select_catalog_active_indices(
            right_players,
            int(effective_defenders),
            ball_xy=ball_xy,
        )

        def build_side(side: str, players: np.ndarray, active_indices):
            active_set = set(active_indices)
            rows = [players[idx].copy() for idx in active_indices]
            pad_idx = 0
            while len(rows) < max_active:
                rows.append(self._catalog_corner_player(side, pad_idx))
                pad_idx += 1
            for idx in range(int(players.shape[0])):
                if idx not in active_set:
                    rows.append(players[idx].copy())
            return np.asarray(rows[: int(players.shape[0])], dtype=np.float32)

        new_left = build_side("left", left_players, left_active)
        new_right = build_side("right", right_players, right_active)
        body_angles = np.concatenate([new_left[:, 2], new_right[:, 2]]).astype(
            np.float32
        )
        return {
            "ball": ball,
            "left_players": new_left,
            "right_players": new_right,
            "body_angles": body_angles,
        }

    def _filter_frames_for_scenario_start(self, frames):
        return [
            frame for frame in frames
            if self._is_right_half_frame(frame) and self._is_left_team_kickable_frame(frame)
        ]

    def _decode_frame_vector(self, vec: np.ndarray):
        vec = np.asarray(vec, dtype=np.float32).reshape(-1)
        expected_dim = 4 + 10 * int(self.n_players)
        if vec.shape[0] != expected_dim:
            raise ValueError(
                f"frame dim must be {expected_dim} for n_players={self.n_players}, "
                f"got {vec.shape[0]}"
            )

        idx = 0
        ball = vec[idx:idx + 4].astype(np.float32)
        idx += 4

        left_players = []
        for _ in range(self.n_players):
            x, y, body, vx, vy = vec[idx:idx + 5]
            left_players.append([x, y, body, vx, vy])
            idx += 5

        right_players = []
        for _ in range(self.n_players):
            x, y, body, vx, vy = vec[idx:idx + 5]
            right_players.append([x, y, body, vx, vy])
            idx += 5

        left_players = np.array(left_players, dtype=np.float32)
        right_players = np.array(right_players, dtype=np.float32)
        body_angles = np.concatenate([left_players[:, 2], right_players[:, 2]]).astype(
            np.float32
        )
        return {
            "ball": ball,
            "left_players": left_players,
            "right_players": right_players,
            "body_angles": body_angles,
        }

    def _is_right_half_frame(self, frame: dict) -> bool:
        return float(frame["ball"][0]) >= 0.0

    def _is_left_team_kickable_frame(self, frame: dict) -> bool:
        ball = np.asarray(frame["ball"], dtype=np.float32)
        left_players = np.asarray(frame["left_players"], dtype=np.float32)
        if left_players.size == 0:
            return False
        bx = float(ball[0])
        by = float(ball[1])
        dists = np.sqrt((left_players[:, 0] - bx) ** 2 + (left_players[:, 1] - by) ** 2)
        return bool(float(np.min(dists)) <= self.kickable_threshold)

    def _infer_n_players_from_state_dim(self, frame_dim: int) -> int:
        frame_dim = int(frame_dim)
        if frame_dim < 14:
            raise ValueError(
                f"frame dim too small: {frame_dim}, expected at least 14"
            )
        remain = frame_dim - 4
        if remain % 10 != 0:
            raise ValueError(
                f"cannot infer n_players from frame_dim={frame_dim}, "
                "expected frame_dim = 4 + 10*n"
            )
        return remain // 10
