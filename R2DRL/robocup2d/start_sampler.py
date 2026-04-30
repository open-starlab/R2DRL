from __future__ import annotations

import os
import random
from typing import Optional

import numpy as np


class ScenarioStartSampler:
    PREFILTERED_TRAJECTORY_TAG = "_right_half_left_nearest_kickable"

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
    ):
        self.current_n = int(init_n)
        self.start_window_size = int(start_window_size)
        self.progress_bucket_count = int(progress_bucket_count)
        self.current_target_window_size = int(current_target_window_size)
        self.num_selected_trajectories = (
            None if num_selected_trajectories is None else int(num_selected_trajectories)
        )
        self.random_sample = bool(random_sample)
        self.scenario_difficulty = self._normalize_scenario_difficulty(
            scenario_difficulty
        )
        self.scenario_difficulty_buckets = self._normalize_scenario_difficulty_buckets(
            scenario_difficulty_buckets
        )

        base_dir = os.path.dirname(os.path.abspath(__file__))
        traj_path = self._resolve_trajectory_path(base_dir, trajectory_path)
        self.trajectories, self.traj_max_frame = self._load_trajectory_array(traj_path)
        self.max_n = self.n_players
        if self.current_n > self.max_n:
            self.current_n = self.max_n

        self.bucket_idx = self._scenario_bucket_idx(traj_path)
        print(
            "[ScenarioStart] "
            f"trajectory_path={traj_path}, "
            f"scenario_difficulty={self.scenario_difficulty}, "
            f"bucket={self.bucket_idx}, n_control={self.current_n}"
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
        return frames[sampled_frame_idx], self.current_n

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
        if difficulty == "medium":
            difficulty = "middle"
        if difficulty not in ("easy", "middle", "hard"):
            raise ValueError(
                "scenario_difficulty must be one of: easy, middle/medium, hard; "
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
        selected_indices = self._select_trajectory_indices(num_traj)

        trajectories = []
        traj_max_frame = []
        for traj_id in selected_indices:
            start = int(traj_offsets[traj_id])
            end = int(traj_offsets[traj_id + 1])
            frames = [
                self._decode_frame_vector(states[global_idx])
                for global_idx in range(start, end)
            ]
            if not self._trajectory_is_prefiltered(trajectory_path):
                frames = self._filter_frames_for_scenario_start(frames)
            if not frames:
                continue
            trajectories.append((len(frames), frames))
            traj_max_frame.append(max(0, len(frames) - 3))

        if not trajectories:
            raise ValueError("all trajectories became empty after start-frame filtering")

        return trajectories, np.asarray(traj_max_frame, dtype=np.int32)

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
        return bool(float(np.min(dists)) <= self.KICKABLE_THRESHOLD)

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
