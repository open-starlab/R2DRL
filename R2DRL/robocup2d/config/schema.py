from __future__ import annotations
from typing import Any, Dict, List, Optional


class EnvConfig:

    def __init__(self, args: Dict[str, Any]):

        required_keys = [
            "n", "team",
            "episode_limit", "half_time", "seed",
            "goal_x", "goal_y", "HALF_LENGTH", "HALF_WIDTH",
            "host",
            "coach_port_offset", "debug_port_offset", "trainer_port_offset",
            "auto_port_start", "auto_port_end", "auto_port_step",
            "player_dir", "player_exe",
            "coach_dir", "coach_exe",
            "trainer_dir", "trainer_exe",
            "server_path",
            "config_dir", "player_config",
            "logs_dir",
            "aggressive_kill", "reset_retries",
            "wait_ready_timeout", "playon_timeout",
            "trainer_ready_timeout_ms",
            "ports_wait_timeout", "server_wait_seconds",
            "use_custom_start",
            "useMaxEpv",
            "text_logging",
            "game_logging",

            # custom start parameters
            "init_n",

            # tensorboard
            "tb",
            "tb_log_dir",

            "lib_paths",
        ]

        for k in required_keys:
            if k not in args:
                raise KeyError(f"Missing required config field: '{k}'")

        # ---------- Team ----------
        self.n: int = self._require_int(args, "n")
        if self.n < 1:
            raise ValueError(f"n must be >= 1, got {self.n}")

        self.team: str = self._require_str(args, "team").lower()
        if self.team not in ("base", "hybrid"):
            raise ValueError(f"Unknown team type: {self.team}")

        # 兼容旧代码：仍然提供 n1/n2/team1/team2
        self.n1: int = self.n
        self.n2: int = self.n
        self.n_total: int = self.n1 + self.n2

        self.team1: str = self.team
        self.team2: str = self.team

        self.is_hybrid: bool = (self.team == "hybrid")
        self.opponent_level: float = float(args.get("opponent_level", 1.0))
        if not (0.0 <= self.opponent_level <= 1.0):
            raise ValueError(f"opponent_level must be in [0, 1], got {self.opponent_level}")

        # ---------- Logging ----------
        self.text_logging: bool = self._require_bool(args, "text_logging")
        self.game_logging: bool = self._require_bool(args, "game_logging")

        # ---------- Episode ----------
        self.episode_limit: int = self._require_int(args, "episode_limit")
        self.half_time: int = self._require_int(args, "half_time")
        self.seed: int = self._require_int(args, "seed")

        # ---------- Geometry ----------
        self.goal_x: float = self._require_float(args, "goal_x")
        self.goal_y: float = self._require_float(args, "goal_y")
        self.half_length: float = self._require_float(args, "HALF_LENGTH")
        self.half_width: float = self._require_float(args, "HALF_WIDTH")

        # ---------- Networking ----------
        self.host: str = self._require_str(args, "host")

        self.coach_port_offset: int = self._require_int(args, "coach_port_offset")
        self.debug_port_offset: int = self._require_int(args, "debug_port_offset")
        self.trainer_port_offset: int = self._require_int(args, "trainer_port_offset")

        self.auto_port_start: int = self._require_int(args, "auto_port_start")
        self.auto_port_end: int = self._require_int(args, "auto_port_end")
        self.auto_port_step: int = self._require_int(args, "auto_port_step")

        # ---------- Paths ----------
        self.player_dir: str = self._require_str(args, "player_dir")
        self.player_exe: str = self._require_str(args, "player_exe")

        self.coach_dir: str = self._require_str(args, "coach_dir")
        self.coach_exe: str = self._require_str(args, "coach_exe")

        self.trainer_dir: str = self._require_str(args, "trainer_dir")
        self.trainer_exe: str = self._require_str(args, "trainer_exe")

        self.server_path: str = self._require_str(args, "server_path")

        self.config_dir: str = self._require_str(args, "config_dir")
        self.player_config: str = self._require_str(args, "player_config")
        self.logs_dir: str = self._require_str(args, "logs_dir")

        # ---------- Runtime ----------
        self.aggressive_kill: bool = self._require_bool(args, "aggressive_kill")
        self.reset_retries: int = self._require_int(args, "reset_retries")

        # ---------- Timeouts ----------
        self.wait_ready_timeout: float = self._require_float(args, "wait_ready_timeout")
        self.playon_timeout: float = self._require_float(args, "playon_timeout")
        self.trainer_ready_timeout_ms: float = self._require_float(args, "trainer_ready_timeout_ms")
        self.ports_wait_timeout: float = self._require_float(args, "ports_wait_timeout")
        self.server_wait_seconds: float = self._require_float(args, "server_wait_seconds")

        # ---------- Custom Start ----------
        self.use_custom_start: bool = self._require_bool(args, "use_custom_start")
        self.useMaxEpv: bool = self._require_bool(args, "useMaxEpv")
        self.benchmark_mode: Optional[str] = self._normalize_benchmark_mode(
            self._optional_str(args, "benchmark_mode", None)
        )
        self.init_n: int = self._require_int(args, "init_n")
        self.trajectory_path: Optional[str] = self._optional_str(
            args, "trajectory_path", None
        )
        self.start_window_size: int = int(args.get("start_window_size", 1))
        self.progress_bucket_count: int = int(args.get("progress_bucket_count", 100))
        self.current_target_window_size: int = int(
            args.get("current_target_window_size", 3)
        )
        self.num_selected_trajectories = args.get("num_selected_trajectories", 10)
        self.random_sample: bool = bool(args.get("random_sample", False))
        self.terminate_on_goal: bool = self._optional_bool(
            args, "terminate_on_goal", True
        )
        self.terminate_on_possession_loss: bool = self._optional_bool(
            args, "terminate_on_possession_loss", False
        )
        self.reward_on_possession_loss: float = self._optional_float(
            args, "reward_on_possession_loss", 0.0
        )
        self.terminate_on_ball_out: bool = self._optional_bool(
            args, "terminate_on_ball_out", False
        )
        self.reward_on_ball_out: float = self._optional_float(
            args, "reward_on_ball_out", 0.0
        )
        self.scenario_start: Optional[str] = self._normalize_scenario_label(
            self._optional_str(args, "scenario_start", None),
            key="scenario_start",
        )
        self.scenario_difficulty: Optional[str] = self._normalize_scenario_label(
            self._optional_str(args, "scenario_difficulty", None),
            key="scenario_difficulty",
        )
        self.scenario_difficulty_buckets = args.get(
            "scenario_difficulty_buckets", None
        )
        if self.scenario_difficulty is None:
            self.scenario_difficulty = self.scenario_start
        elif (
            self.scenario_start is not None
            and self.scenario_difficulty != self.scenario_start
        ):
            raise ValueError(
                "scenario_start and scenario_difficulty must match when both are set; "
                f"got scenario_start={self.scenario_start!r}, "
                f"scenario_difficulty={self.scenario_difficulty!r}"
            )

        if not (1 <= self.init_n <= self.n):
            raise ValueError(
                f"init_n must be between 1 and {self.n}, got {self.init_n}"
            )

        # ---------- TensorBoard ----------
        self.tb: bool = self._require_bool(args, "tb")
        self.tb_log_dir: str = self._require_str(args, "tb_log_dir")

        # ---------- Multi Env ----------
        self.n_envs: int = int(args.get("n_envs", 1))
        if self.n_envs < 1:
            raise ValueError(f"n_envs must be >= 1, got {self.n_envs}")

        # ---------- Lib paths ----------
        lib_paths = args["lib_paths"]
        if not isinstance(lib_paths, list):
            raise TypeError("'lib_paths' must be a list of strings")

        self.lib_paths: List[str] = [str(p) for p in lib_paths]

    # =============================
    # Type Check
    # =============================

    def _require_int(self, args: Dict[str, Any], key: str) -> int:
        val = args[key]
        if not isinstance(val, int):
            raise TypeError(f"'{key}' must be int, got {type(val)}")
        return val

    def _require_float(self, args: Dict[str, Any], key: str) -> float:
        val = args[key]
        if not isinstance(val, (int, float)):
            raise TypeError(f"'{key}' must be float, got {type(val)}")
        return float(val)

    def _require_str(self, args: Dict[str, Any], key: str) -> str:
        val = args[key]
        if not isinstance(val, str):
            raise TypeError(f"'{key}' must be str, got {type(val)}")
        return val

    def _optional_str(
        self,
        args: Dict[str, Any],
        key: str,
        default: Optional[str] = None,
    ) -> Optional[str]:
        if key not in args or args[key] is None:
            return default
        val = args[key]
        if not isinstance(val, str):
            raise TypeError(f"'{key}' must be str, got {type(val)}")
        return val

    def _require_bool(self, args: Dict[str, Any], key: str) -> bool:
        val = args[key]
        if not isinstance(val, bool):
            raise TypeError(f"'{key}' must be bool, got {type(val)}")
        return val

    def _optional_bool(
        self, args: Dict[str, Any], key: str, default: bool
    ) -> bool:
        if key not in args:
            return bool(default)
        return self._require_bool(args, key)

    def _optional_float(
        self, args: Dict[str, Any], key: str, default: float
    ) -> float:
        if key not in args:
            return float(default)
        return self._require_float(args, key)

    def _normalize_benchmark_mode(self, benchmark_mode: Optional[str]) -> Optional[str]:
        if benchmark_mode is None:
            return None

        normalized = benchmark_mode.strip().lower().replace("-", "_")
        if normalized == "":
            return None
        if normalized == "fullmatch":
            normalized = "full_match"
        if normalized == "action_space":
            normalized = "actionspace"

        if normalized not in ("scenario", "full_match", "actionspace"):
            raise ValueError(
                "benchmark_mode must be one of: scenario, full_match, actionspace; "
                f"got {benchmark_mode!r}"
            )
        return normalized

    def _normalize_scenario_label(
        self,
        scenario_label: Optional[str],
        *,
        key: str,
    ) -> Optional[str]:
        if scenario_label is None:
            return None

        normalized = scenario_label.strip().lower()
        if normalized == "":
            return None
        if normalized == "middle":
            normalized = "medium"
        if normalized not in ("easy", "medium", "hard"):
            raise ValueError(
                f"{key} must be one of: easy, medium/middle, hard; "
                f"got {scenario_label!r}"
            )
        return normalized
