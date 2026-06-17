from __future__ import annotations
import random
import numpy as np
from collections import deque
import os

class CurriculumController:
    def __init__(
        self,
        init_n=1,
        start_window_size=1,
        return_window_size=5,
        progress_bucket_count=100,
        current_target_window_size=3,
        window_move_win_rate_threshold=1.5,
        new_key_probability=None,
        num_selected_trajectories=10,
        random_sample=False,
    ):

        self.current_n = init_n

        # 取 start state 时的窗口宽度
        self.start_window_size = int(start_window_size)
        print(f"CurriculumController.start_window_size={self.start_window_size}")

        # 统计最近 return 的滑动窗口长度（只针对当前 frontier）
        self.return_window_size = int(return_window_size)
        self.n_players = 0
        self.num_selected_trajectories = (
            None if num_selected_trajectories is None else int(num_selected_trajectories)
        )
        self.random_sample = (
            False if random_sample is None else bool(random_sample)
        )
        base_dir = os.path.dirname(os.path.abspath(__file__))
        traj_path = os.path.join(base_dir, "trajectories", "3v3trajectories.npz")
        self.trajectories, self.traj_progress = self.load_trajectory_array(traj_path)
        self.traj_max_frame = self.traj_progress.copy()
        self.traj_max_progress_sum = float(np.sum(self.traj_max_frame))
        self.traj_curr_progress_sum = self.traj_max_progress_sum

        self.max_n = self.n_players
        if self.current_n > self.max_n:
            print(f"[Curriculum] init_n={self.current_n} > max_n={self.max_n}, clamp to max_n")
            self.current_n = self.max_n

        print(f"[Curriculum] detected players per side = {self.n_players}")
        print(f"[Curriculum] max_n = {self.max_n}")
        self.step = 0

        self.num_traj = len(self.trajectories)

        self.progress_bucket_count = int(progress_bucket_count)
        self.current_target_window_size = int(current_target_window_size)
        self.current_target_window_start = 0
        self.window_move_win_rate_threshold = float(window_move_win_rate_threshold)
        if new_key_probability is None:
            raise ValueError("new_key_probability must be provided")
        self.new_key_probability = float(new_key_probability)

        self.window_recent_returns = deque()
        self.window_recent_win_count = 0
        self.window_recent_loss_count = 0
        self.window_move_count = 0

        self.bucket_ranges = [[] for _ in range(self.progress_bucket_count)]
        self.current_window_candidates = []
        self.old_window_candidates = []

        self.advance_check_interval = 5000
        self.advance_threshold = 5
        self.period_update_count = 0
        self.period_advanced_count = 0

        selected_msg = (
            str(self.num_selected_trajectories)
            if self.num_selected_trajectories is not None
            else "ALL"
        )
        print(f"[Curriculum] num_selected_trajectories={selected_msg}")
        print(f"[Curriculum] random_sample={int(self.random_sample)}")
        print(f"[Curriculum] new_key_probability={self.new_key_probability}")

        self._rebuild_progress_buckets()
        self._refresh_sampling_cache()

    def generate_new_key(self):
        """从当前百分比窗口内均匀采样一个 start key。"""
        if len(self.trajectories) == 0:
            raise ValueError("no trajectories loaded")

        return self._sample_key_from_candidates(self.current_window_candidates)

    def generate_old_key(self):
        """
        从当前窗口之前的更容易 bucket 里随机采样 old key。
        """
        if len(self.trajectories) == 0:
            raise ValueError("no trajectories loaded")

        if self.current_target_window_start <= 0:
            return None

        return self._sample_key_from_candidates(self.old_window_candidates)

    def generate_key(self, p_new=None):
        """
        按给定概率混合采样当前窗口和历史更容易窗口。
        """
        if p_new is None:
            p_new = self.new_key_probability
        use_new = (random.random() < p_new)

        if use_new:
            key = self.generate_new_key()
            if key is not None:
                return key
            return self.generate_old_key()
        else:
            key = self.generate_old_key()
            if key is not None:
                return key
            return self.generate_new_key()

    # ============================================================
    # progress bucket / window curriculum
    # ============================================================
    def _max_window_start(self):
        return max(0, self.progress_bucket_count - self.current_target_window_size)

    def _safe_win_loss_ratio(self, win_count: int, loss_count: int) -> float:
        if win_count <= 0:
            return 0.0
        if loss_count <= 0:
            return 0.0
        return float(win_count) / float(loss_count)

    def _get_current_window_bucket_ids(self):
        end = min(
            self.progress_bucket_count,
            self.current_target_window_start + self.current_target_window_size,
        )
        return list(range(self.current_target_window_start, end))

    def _frame_idx_for_bucket(self, traj_id: int, bucket_idx: int) -> int:
        max_frame = int(self.traj_max_frame[traj_id])
        if max_frame <= 0:
            return 0

        difficulty_ratio = float(bucket_idx) / float(self.progress_bucket_count - 1)
        frame_ratio = 1.0 - difficulty_ratio
        frame_idx = int(round(frame_ratio * max_frame))
        return int(np.clip(frame_idx, 0, max_frame))

    def _frame_range_for_bucket(self, traj_id: int, bucket_idx: int):
        max_frame = int(self.traj_max_frame[traj_id])
        if max_frame <= 0:
            return 0, 0

        left_ratio = float(bucket_idx) / float(self.progress_bucket_count)
        right_ratio = float(bucket_idx + 1) / float(self.progress_bucket_count)

        # bucket 越靠后，区间越靠近轨迹开头；每个 bucket 对应一段连续 frame
        frame_end = int(round((1.0 - left_ratio) * max_frame))
        frame_start = int(round((1.0 - right_ratio) * max_frame))

        frame_start = int(np.clip(frame_start, 0, max_frame))
        frame_end = int(np.clip(frame_end, 0, max_frame))
        if frame_start > frame_end:
            frame_start, frame_end = frame_end, frame_start

        return frame_start, frame_end

    def _bucket_idx_for_frame(self, traj_id: int, frame_idx: int) -> int:
        max_frame = int(self.traj_max_frame[traj_id])
        if max_frame <= 0:
            return 0

        frame_ratio = float(frame_idx) / float(max_frame)
        difficulty_ratio = 1.0 - frame_ratio
        bucket_idx = int(round(difficulty_ratio * (self.progress_bucket_count - 1)))
        return int(np.clip(bucket_idx, 0, self.progress_bucket_count - 1))

    def _rebuild_progress_buckets(self):
        self.bucket_ranges = [[] for _ in range(self.progress_bucket_count)]
        for bucket_idx in range(self.progress_bucket_count):
            bucket = self.bucket_ranges[bucket_idx]
            for traj_id in range(self.num_traj):
                frame_start, frame_end = self._frame_range_for_bucket(traj_id, bucket_idx)
                bucket.append((traj_id, frame_start, frame_end, self.current_n))

    def _refresh_sampling_cache(self):
        self.current_window_candidates = []
        for bucket_idx in self._get_current_window_bucket_ids():
            if 0 <= bucket_idx < self.progress_bucket_count:
                self.current_window_candidates.extend(self.bucket_ranges[bucket_idx])

        self.old_window_candidates = []
        for bucket_idx in range(self.current_target_window_start):
            if 0 <= bucket_idx < self.progress_bucket_count:
                self.old_window_candidates.extend(self.bucket_ranges[bucket_idx])

    def _sample_key_from_candidates(self, candidate_ranges):
        if not candidate_ranges:
            return None

        traj_id, frame_start, frame_end, _ = random.choice(candidate_ranges)
        frame_idx = random.randint(int(frame_start), int(frame_end))
        return (int(traj_id), int(frame_idx), self.current_n)

    def _clear_recent_window_stats(self):
        self.window_recent_returns.clear()
        self.window_recent_win_count = 0
        self.window_recent_loss_count = 0

    def _append_recent_return(self, episode_return: float):
        episode_return = float(episode_return)
        if episode_return not in (1.0, -1.0):
            return

        if len(self.window_recent_returns) >= self.return_window_size:
            old_return = self.window_recent_returns.popleft()
            if old_return == 1.0:
                self.window_recent_win_count -= 1
            elif old_return == -1.0:
                self.window_recent_loss_count -= 1

        self.window_recent_returns.append(episode_return)
        if episode_return == 1.0:
            self.window_recent_win_count += 1
        else:
            self.window_recent_loss_count += 1

    def _advance_target_window(self):
        if self.current_target_window_start >= self._max_window_start():
            return False

        old_start = self.current_target_window_start
        self.current_target_window_start += 1
        self.window_move_count += 1
        self._clear_recent_window_stats()
        self._refresh_sampling_cache()

        print(
            f"[Curriculum] target window advance: "
            f"{old_start}% -> {self.current_target_window_start}% "
            f"(width={self.current_target_window_size}, n={self.current_n})"
        )
        return True

    def update_key_stats(self, key, episode_return, high=None):
        """
        只跟踪当前百分比窗口内的最近 episode 结果。
        当最近窗口满了且 (return=1 的局数 / return=-1 的局数) 超过阈值时，
        课程窗口右移 1%。

        返回:
            mean_return, visits, level, advanced
        """
        self.step += 1
        traj_id, frame_idx, n_control = key

        bucket_idx = self._bucket_idx_for_frame(traj_id, frame_idx)
        in_current_window = bucket_idx in self._get_current_window_bucket_ids()

        if in_current_window:
            self._append_recent_return(float(episode_return))

        # 统计当前周期
        self.period_update_count += 1

        filled = len(self.window_recent_returns)
        key_visits = 0
        level = "ready"
        mean_return = float(episode_return)
        win_count = self.window_recent_win_count
        loss_count = self.window_recent_loss_count
        win_loss_ratio = self._safe_win_loss_ratio(win_count, loss_count)

        threshold = (
            self.window_move_win_rate_threshold if high is None else float(high)
        )
        advanced = False
        if filled >= self.return_window_size and win_count > 0 and loss_count == 0:
            advanced = self._advance_target_window()
        elif filled >= self.return_window_size and win_loss_ratio >= threshold:
            advanced = self._advance_target_window()

        if advanced:
            self.period_advanced_count += 1

        return (
            mean_return,
            key_visits,
            level,
            advanced,
        )

    def update_after_episode(self, key, episode_return):
        return self.update_key_stats(key=key, episode_return=episode_return)

    def advance_n(self):
        if self.current_n >= self.max_n:
            print(f"[Curriculum] current_n already at max_n={self.max_n}")
            return False

        old_n = self.current_n
        self.current_n += 1

        self.current_target_window_start = 0
        self.window_move_count = 0
        self._clear_recent_window_stats()
        self._rebuild_progress_buckets()
        self._refresh_sampling_cache()

        print(
            f"[Curriculum] n advanced: {old_n} -> {self.current_n}, "
            f"target window reset"
        )
        return True

    # ============================================================
    # trajectory loading
    # ============================================================
    def load_trajectory_array(self, trajectory_path: str):
        data = np.load(trajectory_path)

        starts = data["states"]
        traj_offsets = data["traj_offsets"]
        cycles = data["cycles"]

        if starts.ndim != 2:
            raise ValueError(f"states must be 2D, got shape={starts.shape}")

        frame_dim = int(starts.shape[1])
        self.n_players = self.infer_n_players_from_state_dim(frame_dim)

        print(f"[Trajectory] inferred frame_dim = {frame_dim}")
        print(f"[Trajectory] inferred players per side = {self.n_players}")

        num_traj = len(traj_offsets) - 1
        selected_indices = self._select_trajectory_indices(num_traj)

        print(f"[Trajectory] loaded from: {trajectory_path}")
        print(f"[Trajectory] starts.shape = {starts.shape}")
        print(f"[Trajectory] traj_offsets.shape = {traj_offsets.shape}")
        print(f"[Trajectory] cycles.shape = {cycles.shape}")

        print(f"[Trajectory] num_traj = {num_traj}")
        print(f"[Trajectory] selected_num_traj = {len(selected_indices)}")
        if len(selected_indices) != num_traj:
            print(f"[Trajectory] selected_indices = {selected_indices.tolist()}")

        trajectories = []
        filtered_lengths = []
        original_lengths = []
        traj_progress_list = []

        num_empty_after_filter = 0
        total_original_frames = 0
        total_filtered_frames = 0

        for new_traj_id, traj_id in enumerate(selected_indices):
            start = int(traj_offsets[traj_id])
            end = int(traj_offsets[traj_id + 1])

            original_len = end - start
            original_lengths.append(original_len)
            total_original_frames += original_len

            all_frames = []
            for global_idx in range(start, end):
                vec = starts[global_idx]
                frame = self.decode_frame_vector(vec)
                all_frames.append(frame)

            # 过滤：只保留 ball_x >= 0 的帧
            filtered_frames = [frame for frame in all_frames if float(frame["ball"][0]) >= 0.0]
            filtered_len = len(filtered_frames)

            filtered_lengths.append(filtered_len)
            total_filtered_frames += filtered_len

            if filtered_len == 0:
                num_empty_after_filter += 1
                # 为了不让后面 bucket / randint 出错，这里直接跳过空轨迹
                continue

            trajectories.append((filtered_len, filtered_frames))
            traj_progress_list.append(max(0, filtered_len - 3))

        if len(trajectories) == 0:
            raise ValueError("all trajectories became empty after left-half filtering")

        traj_progress = np.asarray(traj_progress_list, dtype=np.int32)

        filtered_lengths_np = np.asarray(filtered_lengths, dtype=np.int32)
        original_lengths_np = np.asarray(original_lengths, dtype=np.int32)

        print(f"[Trajectory] original traj_lengths = {original_lengths_np.tolist()}")
        print(f"[Trajectory] filtered traj_lengths = {filtered_lengths_np.tolist()}")
        print(f"[Trajectory] original min_len = {original_lengths_np.min()}")
        print(f"[Trajectory] original max_len = {original_lengths_np.max()}")
        print(f"[Trajectory] original mean_len = {original_lengths_np.mean():.2f}")
        print(f"[Trajectory] original total_frames = {total_original_frames}")

        non_empty_filtered = filtered_lengths_np[filtered_lengths_np > 0]
        if len(non_empty_filtered) > 0:
            print(f"[Trajectory] filtered min_len = {non_empty_filtered.min()}")
            print(f"[Trajectory] filtered max_len = {non_empty_filtered.max()}")
            print(f"[Trajectory] filtered mean_len = {non_empty_filtered.mean():.2f}")
        print(f"[Trajectory] filtered total_frames = {total_filtered_frames}")
        print(f"[Trajectory] num_empty_after_filter = {num_empty_after_filter}")
        print(f"[Trajectory] kept_ratio = {total_filtered_frames / max(1, total_original_frames):.4f}")

        print(f"[Curriculum] kept non-empty trajectories = {len(trajectories)}")
        print(f"[Curriculum] traj_progress.shape = {traj_progress.shape}")
        print(f"[Curriculum] first 10 traj_progress = {traj_progress[:10].tolist()}")

        return trajectories, traj_progress

    def _select_trajectory_indices(self, num_traj: int) -> np.ndarray:
        if num_traj <= 0:
            return np.asarray([], dtype=np.int32)

        requested = self.num_selected_trajectories
        if requested is None or requested <= 0 or requested >= num_traj:
            return np.arange(num_traj, dtype=np.int32)

        if self.random_sample:
            return np.sort(np.asarray(random.sample(range(num_traj), requested), dtype=np.int32))

        return np.arange(requested, dtype=np.int32)

    # ============================================================
    # decode frame
    # ============================================================
    def decode_frame_vector(self, vec: np.ndarray):
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

        body_angles = np.concatenate(
            [left_players[:, 2], right_players[:, 2]]
        ).astype(np.float32)

        return {
            "ball": ball,
            "left_players": left_players,
            "right_players": right_players,
            "body_angles": body_angles,
        }

    # ============================================================
    # mask control / reset application
    # ============================================================
    def set_player_mask_n(self, env, n):
        env.agents.set_mask_n(n)
        return env.agents.current_mask_n

    def apply_start_and_n_by_key(self, env, key):
        traj_id, frame_idx, n_control = key

        _, start, _, _ = self.get_starts_by_key(key)

        self.set_player_mask_n(env, n_control)

        env.agents.configure_reset_start(
            ball=start["ball"],
            left_players=start["left_players"],
            right_players=start["right_players"],
            body_angles=start["body_angles"],
        )

        return frame_idx, start

    def get_starts_by_key(self, key):
        """
        根据 key 取状态。
        - key 仍然是 (traj_id, frame_idx, n_control)
        - 当 start_window_size == 1 时，直接取 frame_idx
        - 当 start_window_size > 1 时，把 frame_idx 看作窗口右端点，
          从 [frame_idx - start_window_size + 1, frame_idx] 中随机采样实际 frame_id
        """
        traj_id, frame_idx, n_control = key

        if traj_id < 0 or traj_id >= len(self.trajectories):
            raise IndexError(f"invalid traj_id={traj_id}")

        traj_len, frames = self.trajectories[traj_id]

        if frame_idx < 0 or frame_idx >= traj_len:
            raise IndexError(
                f"invalid frame_idx={frame_idx} for traj_id={traj_id}, traj_len={traj_len}"
            )

        start_idx = max(0, int(frame_idx) - self.start_window_size + 1)
        end_idx = int(frame_idx)

        sampled_frame_idx = random.randint(start_idx, end_idx)
        start = frames[sampled_frame_idx]

        bucket_idx = self._bucket_idx_for_frame(traj_id, frame_idx)
        return sampled_frame_idx, start, bucket_idx, n_control

    def get_frontier_stats(self):
        """
        O(1) 返回当前课程窗口统计。
        """
        stats = {}

        stats["current_target_window_start"] = float(self.current_target_window_start)
        stats["current_target_window_size"] = float(self.current_target_window_size)
        if len(self.window_recent_returns) < self.return_window_size:
            recent_win_loss_ratio = 0.0
        else:
            recent_win_loss_ratio = self._safe_win_loss_ratio(
                self.window_recent_win_count, self.window_recent_loss_count
            )
        stats["recent_win_count"] = float(self.window_recent_win_count)
        stats["recent_loss_count"] = float(self.window_recent_loss_count)
        stats["recent_win_loss_ratio"] = float(recent_win_loss_ratio)

        stats["current_n"] = self.current_n

        return stats

    def infer_n_players_from_state_dim(self, frame_dim: int) -> int:
        """
        从单帧向量维度自动推断每边人数。

        约定：
            frame = [ball(4), left(n*5), right(n*5)]
        所以：
            frame_dim = 4 + 10 * n
        """
        frame_dim = int(frame_dim)

        if frame_dim < 14:
            raise ValueError(
                f"frame dim too small: {frame_dim}, expected at least 14"
            )

        remain = frame_dim - 4
        if remain % 10 != 0:
            raise ValueError(
                f"cannot infer n_players from frame_dim={frame_dim}, "
                f"expected frame_dim = 4 + 10*n"
            )

        n_players = remain // 10
        if n_players <= 0:
            raise ValueError(f"invalid n_players={n_players}")

        return int(n_players)
