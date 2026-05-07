from __future__ import annotations

import re
import os
from typing import Dict, List, Optional, Set, Tuple

import numpy as np
from tqdm import tqdm


# =========================
# Shared regex / type aliases
# =========================
_NUM = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"

SHOW_HEAD_PAT        = re.compile(r"^\(show\s+(\d+)\s")
PLAYMODE_PLAYON_PAT  = re.compile(r"^\(playmode\s+\d+\s+play_on\)")
NONSHOW_SWAP_L_PAT   = re.compile(r"(\b\w+)_l\b")
NONSHOW_SWAP_R_PAT   = re.compile(r"(\b\w+)_r\b")
NONSHOW_SWAP_TMP_PAT = re.compile(r"(\b\w+)__TMP__\b")

BALL_PAT = re.compile(
    rf"\(\(b\)\s+({_NUM})\s+({_NUM})\s+({_NUM})\s+({_NUM})\)"
)
FP_PAT = re.compile(rf"\(fp\s+({_NUM})\s+({_NUM})\)")
PLAYER_PAT = re.compile(
    rf"\(\(\s*([lr])\s+(\d+)\s*\)\s+"
    rf"(\d+)\s+(0x[0-9a-fA-F]+)\s+"
    rf"({_NUM})\s+({_NUM})\s+"
    rf"({_NUM})\s+({_NUM})\s+"
    rf"({_NUM})\s+({_NUM})"
)

Ball4   = Tuple[float, float, float, float]           # (x, y, vx, vy)
Player5 = Tuple[float, float, float, float, float]    # (x, y, body, vx, vy)


# =========================
# Shared helpers
# =========================
def _round_val(x: float, ndigits: int) -> float:
    y = round(float(x), ndigits)
    return 0.0 if y == -0.0 else y


def flip_line_keep_left_as_self(line: str) -> str:
    """
    以 x=0 镜像翻转一条 rcg show 行，同时交换左右队。
    非 show 行只做 *_l <-> *_r 交换。
    """

    def norm_angle(a: float) -> float:
        a = (a + 180.0) % 360.0 - 180.0
        return 0.0 if abs(a) < 1e-12 else a

    def fmt(x: float) -> str:
        if abs(x) < 1e-12:
            x = 0.0
        return f"{x:.10g}"

    if not line.startswith("(show "):
        line = NONSHOW_SWAP_L_PAT.sub(r"\1__TMP__", line)
        line = NONSHOW_SWAP_R_PAT.sub(r"\1_l", line)
        line = NONSHOW_SWAP_TMP_PAT.sub(r"\1_r", line)
        return line

    def ball_repl(m: re.Match) -> str:
        x, y, vx, vy = map(float, m.groups())
        return f"((b) {fmt(-x)} {fmt(y)} {fmt(-vx)} {fmt(vy)})"

    def fp_repl(m: re.Match) -> str:
        fx, fy = map(float, m.groups())
        return f"(fp {fmt(-fx)} {fmt(fy)})"

    def player_repl(m: re.Match) -> str:
        side, unum, p0, hx, x, y, vx, vy, body, neck = m.groups()
        side2 = "r" if side == "l" else "l"
        body2 = norm_angle(180.0 - float(body))
        neck2 = -float(neck)
        return (
            f"(({side2} {unum}) {p0} {hx} "
            f"{fmt(-float(x))} {fmt(float(y))} "
            f"{fmt(-float(vx))} {fmt(float(vy))} "
            f"{fmt(body2)} {fmt(neck2)}"
        )

    line = BALL_PAT.sub(ball_repl, line)
    line = FP_PAT.sub(fp_repl, line)
    line = PLAYER_PAT.sub(player_repl, line)
    return line


# =========================
# Player-count detection
# =========================
def infer_n_from_show_line(show_line: str) -> Tuple[int, int]:
    """返回 (n_left_active, n_right_active)，基于 hex flag != '0'。"""
    n_left = n_right = 0
    for m in PLAYER_PAT.finditer(show_line):
        side    = m.group(1)
        unum    = int(m.group(2))
        hexflag = m.group(4)
        if hexflag != "0":
            if side == "l":
                n_left  = max(n_left,  unum)
            else:
                n_right = max(n_right, unum)
    return n_left, n_right


def detect_n_players(show_line: str) -> int:
    """两队人数必须相等，返回单边人数。"""
    left_ids: Set[int]  = set()
    right_ids: Set[int] = set()
    for m in PLAYER_PAT.finditer(show_line):
        side = m.group(1)
        unum = int(m.group(2))
        (left_ids if side == "l" else right_ids).add(unum)
    if not left_ids or not right_ids:
        raise ValueError("cannot detect players")
    if len(left_ids) != len(right_ids):
        raise ValueError(f"unequal team size: L={len(left_ids)}, R={len(right_ids)}")
    return max(left_ids)


# =========================
# Parse / encode / decode
# =========================
def parse_show_line(
    show_line: str,
    *,
    ndigits: int,
    n_players: int,
) -> Tuple[int, Ball4, List[Player5], List[Player5]]:
    m = SHOW_HEAD_PAT.match(show_line)
    if not m:
        raise ValueError("not a show line")
    cycle = int(m.group(1))

    mb = BALL_PAT.search(show_line)
    if not mb:
        raise ValueError(f"ball not found cycle={cycle}")
    ball: Ball4 = tuple(_round_val(float(mb.group(i)), ndigits) for i in range(1, 5))  # type: ignore

    left: Dict[int, Player5]  = {}
    right: Dict[int, Player5] = {}
    for pm in PLAYER_PAT.finditer(show_line):
        side = pm.group(1)
        unum = int(pm.group(2))
        x    = _round_val(float(pm.group(5)), ndigits)
        y    = _round_val(float(pm.group(6)), ndigits)
        vx   = _round_val(float(pm.group(7)), ndigits)
        vy   = _round_val(float(pm.group(8)), ndigits)
        body = _round_val(float(pm.group(9)), ndigits)
        p: Player5 = (x, y, body, vx, vy)
        (left if side == "l" else right)[unum] = p

    if len(left) != n_players or len(right) != n_players:
        raise ValueError(f"player count mismatch at cycle={cycle}")

    left_list  = [left[i]  for i in sorted(left.keys())]
    right_list = [right[i] for i in sorted(right.keys())]
    return cycle, ball, left_list, right_list


def show_to_players_n(
    show_line: str,
    *,
    n1: int,
    n2: int,
    ndigits: int = 4,
    prev: bool = False,
    ball_decay: float = 0.94,
    player_decay: float = 0.4,
    debug: bool = False,
) -> Tuple[Ball4, List[Player5], List[Player5]]:
    """解析一条 show 行，可选返回上一帧推算值。"""

    def _r(x: float) -> float:
        return _round_val(x, ndigits)

    def _back_ball(b: Ball4) -> Ball4:
        x, y, vx, vy = b
        vx_p = vx / ball_decay
        vy_p = vy / ball_decay
        return (_r(x - vx_p), _r(y - vy_p), _r(vx_p), _r(vy_p))

    def _back_player(p: Player5) -> Player5:
        x, y, body, vx, vy = p
        vx_p = vx / player_decay
        vy_p = vy / player_decay
        return (_r(x - vx_p), _r(y - vy_p), body, _r(vx_p), _r(vy_p))

    mb = BALL_PAT.search(show_line)
    if not mb:
        raise ValueError("Ball chunk not found in show line")
    ball: Ball4 = tuple(float(mb.group(i)) for i in range(1, 5))  # type: ignore

    left: Dict[int, Player5]  = {}
    right: Dict[int, Player5] = {}
    for pm in PLAYER_PAT.finditer(show_line):
        side = pm.group(1)
        unum = int(pm.group(2))
        x    = float(pm.group(5));  y    = float(pm.group(6))
        vx   = float(pm.group(7));  vy   = float(pm.group(8))
        body = float(pm.group(9));  neck = float(pm.group(10))
        p: Player5 = (x, y, body, vx, vy)
        if debug and unum <= 3:
            print(f"[PARSE] side={side} unum={unum} x={x} y={y} vx={vx} vy={vy} body={body} neck={neck}")
        if debug and (abs(vx) > 5 or abs(vy) > 5):
            print(f"[WARN-SPEED] side={side} unum={unum} vx={vx} vy={vy}")
        (left if side == "l" else right)[unum] = p

    missing_l = [i for i in range(1, n1 + 1) if i not in left]
    missing_r = [i for i in range(1, n2 + 1) if i not in right]
    if missing_l or missing_r:
        raise ValueError(f"Missing players: left={missing_l}, right={missing_r}")

    left_list  = [left[i]  for i in range(1, n1 + 1)]
    right_list = [right[i] for i in range(1, n2 + 1)]

    if prev:
        ball       = _back_ball(ball)
        left_list  = [_back_player(p) for p in left_list]
        right_list = [_back_player(p) for p in right_list]

    ball_out  = (_r(ball[0]), _r(ball[1]), _r(ball[2]), _r(ball[3]))
    left_out  = [(_r(x), _r(y), _r(body), _r(vx), _r(vy)) for x, y, body, vx, vy in left_list]
    right_out = [(_r(x), _r(y), _r(body), _r(vx), _r(vy)) for x, y, body, vx, vy in right_list]
    return ball_out, left_out, right_out


def prev_frame_inertia(
    show_line: str,
    *,
    ball_decay: float,
    player_decay: float,
    ndigits: int,
    n_players: int,
) -> Tuple[Ball4, List[Player5], List[Player5]]:
    """通过 parse_show_line 解析，然后推算上一帧。"""
    _, ball, left_players, right_players = parse_show_line(
        show_line, ndigits=ndigits, n_players=n_players
    )
    bx, by, bvx, bvy = ball
    bvx_p = _round_val(bvx / ball_decay, ndigits)
    bvy_p = _round_val(bvy / ball_decay, ndigits)
    ball_prev: Ball4 = (
        _round_val(bx - bvx_p, ndigits),
        _round_val(by - bvy_p, ndigits),
        bvx_p, bvy_p,
    )

    def _p(p: Player5) -> Player5:
        x, y, body, vx, vy = p
        vx_p = _round_val(vx / player_decay, ndigits)
        vy_p = _round_val(vy / player_decay, ndigits)
        return (_round_val(x - vx_p, ndigits), _round_val(y - vy_p, ndigits), body, vx_p, vy_p)

    return ball_prev, [_p(p) for p in left_players], [_p(p) for p in right_players]


def encode_frame_vector(ball, left, right) -> np.ndarray:
    """[ball(4), left(n*5), right(n*5)] → float32 ndarray。"""
    parts = [np.asarray(ball, dtype=np.float32).reshape(-1)]
    for p in left:
        parts.append(np.asarray(p, dtype=np.float32).reshape(-1))
    for p in right:
        parts.append(np.asarray(p, dtype=np.float32).reshape(-1))
    return np.concatenate(parts)


def pack_frame_vector(ball, left_players, right_players, dtype=np.float32) -> np.ndarray:
    """encode_frame_vector 的别名，带 dtype 参数。"""
    return encode_frame_vector(ball, left_players, right_players).astype(dtype, copy=False)


def decode_frame_vector(vec, n_players: int) -> dict:
    vec = np.asarray(vec, dtype=np.float32)
    idx  = 0
    ball = tuple(vec[idx:idx + 4]);  idx += 4
    left  = [tuple(vec[idx + i*5: idx + i*5 + 5]) for i in range(n_players)]; idx += 5 * n_players
    right = [tuple(vec[idx + i*5: idx + i*5 + 5]) for i in range(n_players)]
    return dict(ball=ball, left_players=left, right_players=right)


def normalize_players_for_npz(players) -> list:
    out = []
    for p in players:
        if len(p) != 5:
            raise ValueError(f"Player tuple must have len=5, got {p}")
        x, y, body, vx, vy = p
        out.append((float(x), float(y), float(body), float(vx), float(vy)))
    return out


# =========================
# rcg scanning
# =========================
def read_reset_cycles_from_out(out_path: str) -> List[int]:
    pat = re.compile(
        r"\[RESET\]\s*turn=(\d+),\s*score=\[(\d+),\s*(\d+)\],\s*cycle=(\d+)"
    )
    cycles: List[int] = []
    with open(out_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = pat.search(line)
            if m:
                cycles.append(int(m.group(4)))
    return cycles


def scan_rcg_once_for_build_subset(
    rcg_path: str,
    show_progress: bool = True,
) -> Tuple[List[str], List[Tuple[int, str]], Dict[int, int]]:
    """
    一次扫描 rcg，同时返回：
      lines            : 所有原始行
      gm_change_cycles : [(show_cycle, mode), ...] playmode 后首个 show
      cycle2line       : {cycle: first_show_line_no}
    """
    lines: List[str] = []
    gm_change_cycles: List[Tuple[int, str]] = []
    cycle2line: Dict[int, int] = {}
    pending_mode: Optional[str] = None

    total_bytes = os.path.getsize(rcg_path)
    pbar = tqdm(total=total_bytes, unit="B", unit_scale=True,
                desc="Scanning rcg", disable=not show_progress)

    with open(rcg_path, "r", encoding="utf-8", errors="ignore", buffering=1 << 20) as f:
        for line_no, line in enumerate(f, start=1):
            lines.append(line)
            pbar.update(len(line.encode("utf-8", errors="ignore")))

            if line.startswith("(playmode "):
                parts = line.split(" ", 2)
                if len(parts) == 3:
                    pending_mode = parts[2].rstrip(")\r\n")
                continue

            if not line.startswith("(show "):
                continue

            parts = line.split(" ", 2)
            if len(parts) < 2:
                continue
            try:
                cycle = int(parts[1])
            except Exception:
                continue

            if cycle not in cycle2line:
                cycle2line[cycle] = line_no
            if pending_mode is not None:
                gm_change_cycles.append((cycle, pending_mode))
                pending_mode = None

    pbar.close()
    return lines, gm_change_cycles, cycle2line


# =========================
# Timeline helpers
# =========================
def merge_reset_and_gm_cycles(
    reset_cycles: List[int],
    gm_change_cycles: List[Tuple[int, str]],
) -> List[Tuple[int, str]]:
    merged: Dict[int, str] = {c: "reset" for c in reset_cycles}
    for c, gm in gm_change_cycles:
        merged[c] = gm
    return sorted(merged.items(), key=lambda x: x[0])


def extract_reset_goal_intervals(
    timeline: List[Tuple[int, str]]
) -> List[Tuple[int, int, int]]:
    """严格相邻的 reset -> goal_l/goal_r，返回 [(reset_cycle, goal_cycle, side)]。"""
    intervals: List[Tuple[int, int, int]] = []
    for i in range(1, len(timeline)):
        prev_cycle, prev_tag = timeline[i - 1]
        cur_cycle,  cur_tag  = timeline[i]
        if prev_tag != "reset":
            continue
        if cur_tag == "goal_l":
            intervals.append((prev_cycle, cur_cycle, +1))
        elif cur_tag == "goal_r":
            intervals.append((prev_cycle, cur_cycle, -1))
    return intervals


def adjust_interval_lines_for_manual_rcg_fix(
    intervals_with_lines: List[Tuple[int, int, int, int, int]],
    reset_offset: int = +1,
    goal_offset: int  = -3,
) -> List[Tuple[int, int, int, int, int]]:
    return [
        (rc, rl + reset_offset, gc, gl + goal_offset, side)
        for rc, rl, gc, gl, side in intervals_with_lines
    ]


# =========================
# NPZ builders
# =========================
def build_npz_from_interval_lines(
    rcg_path: str,
    intervals_with_lines: List[Tuple[int, int, int, int, int]],
    dst_npz_path: str = "trajectories.npz",
    ndigits: int = 4,
    use_prev_frame: bool = False,
    verbose: bool = True,
    debug: bool = False,
):
    with open(rcg_path, "r", encoding="utf-8", errors="ignore") as f:
        all_lines = f.readlines()

    all_states: List[np.ndarray] = []
    all_cycles: List[int] = []
    traj_offsets: List[int] = [0]
    detected_n1 = detected_n2 = None
    num_kept_traj = 0

    for traj_id, (reset_cycle, reset_line, goal_cycle, goal_line, side) in enumerate(intervals_with_lines):
        traj_states: List[np.ndarray] = []
        traj_cycles: List[int] = []

        for raw_line in all_lines[reset_line - 1: goal_line]:
            if not raw_line.startswith("(show "):
                continue
            line = flip_line_keep_left_as_self(raw_line) if side == -1 else raw_line

            try:
                cycle = int(line.split(" ", 2)[1])
            except Exception:
                continue

            n1, n2 = infer_n_from_show_line(line)
            if detected_n1 is None:
                detected_n1, detected_n2 = n1, n2
                if verbose:
                    print(f"[NPZ] detected players: left={n1}, right={n2}")
            elif (n1, n2) != (detected_n1, detected_n2):
                raise ValueError(
                    f"Inconsistent player count traj={traj_id} cycle={cycle}: "
                    f"got ({n1},{n2}), expected ({detected_n1},{detected_n2})"
                )
            if n1 != n2:
                raise ValueError(f"n1 != n2 at traj={traj_id} cycle={cycle}")

            ball, left, right = show_to_players_n(
                line, n1=n1, n2=n2, ndigits=ndigits, prev=use_prev_frame, debug=debug
            )
            left  = normalize_players_for_npz(left)
            right = normalize_players_for_npz(right)
            vec   = pack_frame_vector(ball, left, right)

            if debug:
                print(f"[NPZ] traj={traj_id} cycle={cycle} ball={vec[:4]}")

            traj_states.append(vec)
            traj_cycles.append(cycle)

        if not traj_states:
            if verbose:
                print(f"[NPZ] skip empty traj {traj_id}: lines [{reset_line},{goal_line}]")
            continue

        all_states.extend(traj_states)
        all_cycles.extend(traj_cycles)
        traj_offsets.append(len(all_states))
        num_kept_traj += 1

        if verbose:
            print(f"[NPZ] traj={traj_id} frames={len(traj_states)} "
                  f"cycles=[{traj_cycles[0]}->{traj_cycles[-1]}]")

    if not all_states:
        raise ValueError("No valid show frames found; cannot build npz")

    states       = np.stack(all_states).astype(np.float32)
    cycles       = np.asarray(all_cycles, dtype=np.int32)
    traj_offsets = np.asarray(traj_offsets, dtype=np.int32)

    np.savez_compressed(dst_npz_path, states=states, traj_offsets=traj_offsets, cycles=cycles)

    if verbose:
        print(f"[NPZ] saved → {dst_npz_path}")
        print(f"[NPZ] states={states.shape}  cycles={cycles.shape}  "
              f"traj_offsets={traj_offsets.shape}  num_traj={num_kept_traj}")
        if detected_n1 is not None:
            print(f"[NPZ] players_per_side={detected_n1}")

    return states, traj_offsets, cycles


def convert_rcg_playon_to_npz(
    rcg_path: str,
    npz_path: str,
    *,
    ball_decay: float = 0.94,
    player_decay: float = 0.4,
    ndigits: int = 4,
):
    """
    直接从 generated_subset.rcg（只含 play_on 区间）转换为 npz。
    每遇到 (playmode ... play_on) 开启新 trajectory。
    """
    all_states: List[np.ndarray] = []
    all_cycles: List[int] = []
    traj_offsets: List[int] = [0]
    current_states: List[np.ndarray] = []
    current_cycles: List[int] = []
    in_traj   = False
    n_players = None

    with open(rcg_path, "r", encoding="utf-8", errors="ignore") as f:
        total_lines = sum(1 for _ in f)

    with open(rcg_path, "r", encoding="utf-8", errors="ignore") as f:
        for line in tqdm(f, total=total_lines):
            line = line.strip()
            if not line:
                continue

            if PLAYMODE_PLAYON_PAT.match(line):
                if in_traj and current_states:
                    all_states.extend(current_states)
                    all_cycles.extend(current_cycles)
                    traj_offsets.append(len(all_states))
                current_states = []
                current_cycles = []
                in_traj = True
                continue

            if in_traj and line.startswith("(show "):
                if n_players is None:
                    n_players = detect_n_players(line)
                    print(f"[INFO] detected n_players = {n_players}")

                cycle = int(SHOW_HEAD_PAT.match(line).group(1))
                ball_prev, left_prev, right_prev = prev_frame_inertia(
                    line,
                    ball_decay=ball_decay,
                    player_decay=player_decay,
                    ndigits=ndigits,
                    n_players=n_players,
                )
                current_states.append(encode_frame_vector(ball_prev, left_prev, right_prev))
                current_cycles.append(cycle)

    if current_states:
        all_states.extend(current_states)
        all_cycles.extend(current_cycles)
        traj_offsets.append(len(all_states))

    states       = np.stack(all_states)
    cycles       = np.asarray(all_cycles, dtype=np.int32)
    traj_offsets = np.asarray(traj_offsets, dtype=np.int32)

    np.savez_compressed(
        npz_path,
        states=states,
        cycles=cycles,
        traj_offsets=traj_offsets,
        n_players=n_players,
    )
    print("===== DONE =====")
    print("n_players:", n_players)
    print("state_dim:", states.shape[1])


def build_generated_subset_rcg(
    rcg_path: str,
    log_path: str,
    dst_rcg_path: str = "generated_subset.rcg",
    reset_offset: int = +1,
    goal_offset: int  = -3,
    verbose: bool = True,
) -> List[Tuple[int, int, int, int, int]]:
    if verbose: print("# 1) 从 .out 抽取 reset cycle", flush=True)
    reset_cycles = read_reset_cycles_from_out(log_path)

    if verbose: print("# 2) 扫描 .rcg", flush=True)
    lines, gm_change_cycles, cycle2line = scan_rcg_once_for_build_subset(rcg_path)

    if verbose: print("# 3) 合并时间轴", flush=True)
    timeline = merge_reset_and_gm_cycles(reset_cycles, gm_change_cycles)

    if verbose: print("# 4) 提取 reset -> goal 区间", flush=True)
    intervals = extract_reset_goal_intervals(timeline)
    if verbose:
        for s, e, sd in intervals:
            print(f"  {s} -> {e} (side={sd})")

    if verbose: print("# 5) 映射行号", flush=True)
    intervals_with_lines: List[Tuple[int, int, int, int, int]] = []
    for reset_c, goal_c, side in intervals:
        if reset_c not in cycle2line:
            raise RuntimeError(f"reset cycle not found: {reset_c}")
        if goal_c not in cycle2line:
            raise RuntimeError(f"goal cycle not found: {goal_c}")
        intervals_with_lines.append((
            reset_c, cycle2line[reset_c],
            goal_c,  cycle2line[goal_c],
            side,
        ))

    if verbose: print("# 6) 行号偏移修正", flush=True)
    fixed = adjust_interval_lines_for_manual_rcg_fix(
        intervals_with_lines, reset_offset=reset_offset, goal_offset=goal_offset
    )
    if verbose:
        for rc, rl, gc, gl, sd in fixed:
            print(f"  reset=({rc},{rl})  goal=({gc},{gl})  side={sd}")

    if verbose: print("# 7) 写入新 rcg", flush=True)
    max_line = len(lines)
    with open(dst_rcg_path, "w", encoding="utf-8") as f:
        f.writelines(lines[:4])
        for reset_cycle, reset_line, _, goal_line, side in fixed:
            f.write(f"(playmode {reset_cycle} play_on)\n")
            start = max(1, reset_line)
            end   = min(goal_line, max_line)
            if start > end:
                continue
            seg = lines[start - 1: end]
            if side == +1:
                f.writelines(seg)
            else:
                for ln in seg:
                    f.write(flip_line_keep_left_as_self(ln))

    if verbose: print(f"Generated: {dst_rcg_path}")
    return fixed


def build_generated_subset_npz(
    rcg_path: str,
    log_path: str,
    dst_npz_path: str = "trajectories.npz",
    reset_offset: int = +1,
    goal_offset: int  = -3,
    ndigits: int = 4,
    use_prev_frame: bool = False,
    verbose: bool = True,
):
    fixed_intervals = build_generated_subset_rcg(
        rcg_path=rcg_path,
        log_path=log_path,
        dst_rcg_path="generated_subset.rcg",
        reset_offset=reset_offset,
        goal_offset=goal_offset,
        verbose=verbose,
    )
    return build_npz_from_interval_lines(
        rcg_path=rcg_path,
        intervals_with_lines=fixed_intervals,
        dst_npz_path=dst_npz_path,
        ndigits=ndigits,
        use_prev_frame=use_prev_frame,
        verbose=verbose,
        debug=False,
    )


# =========================
# Entry point
# =========================
if __name__ == "__main__":
    rcg_path = "/fsws1/h_qin/robocup/robocup/pymarl/src/log/a1e6bdbf66ba474793389dafea4d839f/rcg/incomplete.rcg"
    log_path = "/fsws1/h_qin/robocup/robocup/pymarl/src/slurm-6789952.out"

    build_generated_subset_rcg(
        rcg_path=rcg_path,
        log_path=log_path,
        dst_rcg_path="generated_subset.rcg",
        reset_offset=+1,
        goal_offset=-3,
        verbose=True,
    )

    convert_rcg_playon_to_npz(
        rcg_path="generated_subset.rcg",
        npz_path="generated_subset.npz",
    )