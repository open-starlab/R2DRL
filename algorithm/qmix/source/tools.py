from __future__ import annotations
import re
import bisect
from typing import Dict, Tuple, Optional, Set, List
from collections import Counter
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FFMpegWriter
import shutil
import os
from tqdm import tqdm
# ========= fast regex for flip =========
_NUM = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"

NONSHOW_SWAP_L_PAT = re.compile(r"(\b\w+)_l\b")
NONSHOW_SWAP_R_PAT = re.compile(r"(\b\w+)_r\b")
NONSHOW_SWAP_TMP_PAT = re.compile(r"(\b\w+)__TMP__\b")

BALL_PAT = re.compile(rf"\(\(b\)\s+({_NUM})\s+({_NUM})\s+({_NUM})\s+({_NUM})\)")
FP_PAT   = re.compile(rf"\(fp\s+({_NUM})\s+({_NUM})\)")
PLAYER_PAT = re.compile(
    rf"\(\(\s*([lr])\s+(\d+)\s*\)\s+"
    rf"(\d+)\s+(0x[0-9a-fA-F]+)\s+"
    rf"({_NUM})\s+({_NUM})\s+"
    rf"({_NUM})\s+({_NUM})\s+"
    rf"({_NUM})\s+({_NUM})"
)


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


def merge_reset_and_gm_cycles(
    reset_cycles: List[int],
    gm_change_cycles: List[Tuple[int, str]],
) -> List[Tuple[int, str]]:
    """
    合并 reset_cycles 与 gm_change_cycles

    规则：
      - reset → (cycle, "reset")
      - 若同一 cycle 同时存在 reset 和 gmchange，则丢弃 reset
      - 返回按 cycle 升序排列的 [(cycle, label)]
    """
    merged: Dict[int, str] = {}

    for c in reset_cycles:
        merged[c] = "reset"

    for c, gm in gm_change_cycles:
        merged[c] = gm

    return sorted(merged.items(), key=lambda x: x[0])


def extract_reset_goal_intervals(
    timeline: List[Tuple[int, str]]
) -> List[Tuple[int, int, int]]:
    """
    只保留严格相邻的：
      reset -> goal_l / goal_r

    返回：
      [(reset_cycle, goal_cycle, side), ...]

    side:
      goal_l -> +1
      goal_r -> -1
    """
    intervals: List[Tuple[int, int, int]] = []

    for i in range(1, len(timeline)):
        prev_cycle, prev_tag = timeline[i - 1]
        cur_cycle, cur_tag = timeline[i]

        if prev_tag != "reset":
            continue

        if cur_tag == "goal_l":
            intervals.append((prev_cycle, cur_cycle, +1))
        elif cur_tag == "goal_r":
            intervals.append((prev_cycle, cur_cycle, -1))

    return intervals


def scan_rcg_once_for_build_subset(
    rcg_path: str,
    show_progress: bool = True,
) -> Tuple[List[str], List[Tuple[int, str]], Dict[int, int]]:
    """
    只扫描一次 rcg，同时返回：

    1) lines:
       整个 rcg 的所有行（1-based line_no 时，实际索引要减 1）

    2) gm_change_cycles:
       对每个 (playmode cycle mode)，记录其后第一个 (show X ...) 的 X
       返回 [(X, mode), ...]

    3) cycle2line:
       每个 show cycle 第一次出现时对应的行号
       返回 {cycle: first_show_line_no}
    """
    lines: List[str] = []
    gm_change_cycles: List[Tuple[int, str]] = []
    cycle2line: Dict[int, int] = {}

    pending_mode: Optional[str] = None

    total_bytes = os.path.getsize(rcg_path)
    pbar = tqdm(
        total=total_bytes,
        unit="B",
        unit_scale=True,
        desc="Scanning rcg",
        disable=not show_progress,
    )

    with open(rcg_path, "r", encoding="utf-8", errors="ignore", buffering=1024 * 1024) as f:
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


def flip_line_keep_left_as_self(line: str) -> str:
    """
    输入一行输出一行：
    - (show ...) 行：以 x=0 镜像：x->-x, vx->-vx；body->norm(180-body)；neck->-neck
      同时交换左右队 (l n)<->(r n)，用于保持"左边=自己"的语义一致性
    - 非 (show ...) 行：可选交换 *_l <-> *_r（如 playmode/goal/kick_off 等），没有则不影响
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
        x = float(x)
        y = float(y)
        vx = float(vx)
        vy = float(vy)
        body = float(body)
        neck = float(neck)

        side2 = "r" if side == "l" else "l"
        body2 = norm_angle(180.0 - body)
        neck2 = -neck

        return (
            f"(({side2} {unum}) {p0} {hx} "
            f"{fmt(-x)} {fmt(y)} {fmt(-vx)} {fmt(vy)} "
            f"{fmt(body2)} {fmt(neck2)}"
        )

    line = BALL_PAT.sub(ball_repl, line)
    line = FP_PAT.sub(fp_repl, line)
    line = PLAYER_PAT.sub(player_repl, line)
    return line


Ball4 = Tuple[float, float, float, float]              # (x, y, vx, vy)
Player5 = Tuple[float, float, float, float, float]     # (x, y, body, vx, vy)


def adjust_interval_lines_for_manual_rcg_fix(
    intervals_with_lines: List[Tuple[int, int, int, int, int]],
    reset_offset: int = +1,
    goal_offset: int = -3,
) -> List[Tuple[int, int, int, int, int]]:
    """
    对 intervals_with_lines 中的 rcg 行号做人工偏移修正

    输入：
      (reset_cycle, reset_line, goal_cycle, goal_line, side)

    输出：
      (reset_cycle, reset_line+reset_offset,
       goal_cycle,  goal_line+goal_offset, side)
    """
    fixed = []

    for rc, rl, gc, gl, side in intervals_with_lines:
        fixed.append((
            rc,
            rl + reset_offset,
            gc,
            gl + goal_offset,
            side,
        ))

    return fixed


def infer_n_from_show_line(show_line: str) -> Tuple[int, int]:
    re_player = re.compile(
        r"\(\((l|r)\s+(\d+)\)\s+"
        r"([-\d.eE+]+)\s+"
        r"(0x[0-9a-fA-F]+|0)\s+"
        r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+"
        r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+"
        r"([-\d.eE+]+)\s+([-\d.eE+]+)"
    )

    n_left = 0
    n_right = 0

    for m in re_player.finditer(show_line):
        side = m.group(1)
        unum = int(m.group(2))
        hexflag = m.group(4)

        is_active = (hexflag != "0")

        if is_active:
            if side == "l":
                n_left = max(n_left, unum)
            else:
                n_right = max(n_right, unum)

    return n_left, n_right


def normalize_players_for_npz(players):
    """
    强制统一为 npz / curriculum 内部格式:
        (x, y, body, vx, vy)
    """
    out = []
    for p in players:
        if len(p) != 5:
            raise ValueError(f"Player tuple must have len=5, got {p}")
        x, y, body, vx, vy = p
        out.append((float(x), float(y), float(body), float(vx), float(vy)))
    return out


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
):
    re_ball = re.compile(
        r"\(\(b\)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\s+([-\d.eE+]+)\)"
    )
    re_player = re.compile(
        r"\(\((l|r)\s+(\d+)\)\s+"
        r"[-\d.eE+]+\s+"
        r"(0x[0-9a-fA-F]+|0)\s+"
        r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+"   # x, y
        r"([-\d.eE+]+)\s+([-\d.eE+]+)\s+"   # vx, vy
        r"([-\d.eE+]+)\s+([-\d.eE+]+)"      # body, neck
    )

    def _r(x: float) -> float:
        y = round(float(x), ndigits)
        return 0.0 if y == -0.0 else y

    def _back_ball(b):
        x, y, vx, vy = b
        vx_prev = vx / ball_decay
        vy_prev = vy / ball_decay
        x_prev = x - vx_prev
        y_prev = y - vy_prev
        return (x_prev, y_prev, vx_prev, vy_prev)

    def _back_player(p):
        x, y, body, vx, vy = p
        vx_prev = vx / player_decay
        vy_prev = vy / player_decay
        x_prev = x - vx_prev
        y_prev = y - vy_prev
        return (x_prev, y_prev, body, vx_prev, vy_prev)

    m = re_ball.search(show_line)
    if not m:
        raise ValueError("Ball chunk not found in show line")
    ball = tuple(float(m.group(i)) for i in range(1, 5))

    left = {}
    right = {}

    for pm in re_player.finditer(show_line):
        side = pm.group(1)
        unum = int(pm.group(2))

        x = float(pm.group(4))
        y = float(pm.group(5))
        vx = float(pm.group(6))
        vy = float(pm.group(7))
        body = float(pm.group(8))
        neck = float(pm.group(9))

        # 内部统一顺序: (x, y, body, vx, vy)
        p = (x, y, body, vx, vy)

        if debug and unum <= 3:
            print(
                f"[PARSE] side={side} unum={unum} "
                f"rcg_raw=(x={x}, y={y}, vx={vx}, vy={vy}, body={body}, neck={neck}) "
                f"packed={p}"
            )

        if debug and (abs(vx) > 5 or abs(vy) > 5):
            print(
                f"[WARN-SPEED] side={side} unum={unum} "
                f"vx={vx}, vy={vy}, body={body}, neck={neck}"
            )

        if side == "l":
            left[unum] = p
        else:
            right[unum] = p

    missing_l = [i for i in range(1, n1 + 1) if i not in left]
    missing_r = [i for i in range(1, n2 + 1) if i not in right]
    if missing_l or missing_r:
        raise ValueError(f"Missing players: left={missing_l}, right={missing_r}")

    left_list = [left[i] for i in range(1, n1 + 1)]
    right_list = [right[i] for i in range(1, n2 + 1)]

    if prev:
        ball = _back_ball(ball)
        left_list = [_back_player(p) for p in left_list]
        right_list = [_back_player(p) for p in right_list]

    ball_out = (_r(ball[0]), _r(ball[1]), _r(ball[2]), _r(ball[3]))
    left_out = [(_r(x), _r(y), _r(body), _r(vx), _r(vy)) for (x, y, body, vx, vy) in left_list]
    right_out = [(_r(x), _r(y), _r(body), _r(vx), _r(vy)) for (x, y, body, vx, vy) in right_list]

    return ball_out, left_out, right_out


def pack_frame_vector(
    ball,
    left_players,
    right_players,
    dtype=np.float32,
) -> np.ndarray:
    """
    内部 npz 存储格式:
        [ball(4), left(n*5), right(n*5)]
    其中每个 player 是:
        (x, y, body, vx, vy)
    """
    parts = [np.asarray(ball, dtype=dtype).reshape(-1)]

    for p in left_players:
        parts.append(np.asarray(p, dtype=dtype).reshape(-1))

    for p in right_players:
        parts.append(np.asarray(p, dtype=dtype).reshape(-1))

    vec = np.concatenate(parts, axis=0)
    return vec.astype(dtype, copy=False)


def build_npz_from_interval_lines(
    rcg_path: str,
    intervals_with_lines: List[Tuple[int, int, int, int, int]],
    dst_npz_path: str = "trajectories.npz",
    ndigits: int = 4,
    use_prev_frame: bool = False,
    verbose: bool = True,
    debug: bool = False,
):
    """
    根据区间行号直接从 rcg 生成 curriculum 可读取的 .npz

    输入区间格式:
        (reset_cycle, reset_line, goal_cycle, goal_line, side)

    side:
        +1 -> 原样读取
        -1 -> 先用 flip_line_keep_left_as_self() 翻转，再解析

    输出 npz:
        - states
        - traj_offsets
        - cycles
    """
    with open(rcg_path, "r", encoding="utf-8", errors="ignore") as f:
        all_lines = f.readlines()

    all_states = []
    all_cycles = []
    traj_offsets = [0]

    detected_n1 = None
    detected_n2 = None
    num_kept_traj = 0

    for traj_id, (reset_cycle, reset_line, goal_cycle, goal_line, side) in enumerate(intervals_with_lines):
        traj_states = []
        traj_cycles = []

        seg = all_lines[reset_line - 1: goal_line]

        for raw_line in seg:
            if not raw_line.startswith("(show "):
                continue

            line = raw_line
            if side == -1:
                line = flip_line_keep_left_as_self(line)

            try:
                cycle = int(line.split(" ", 2)[1])
            except Exception:
                continue

            n1, n2 = infer_n_from_show_line(line)

            if detected_n1 is None:
                detected_n1 = n1
                detected_n2 = n2
                if verbose:
                    print(f"[NPZ] detected players: left={detected_n1}, right={detected_n2}")
            else:
                if n1 != detected_n1 or n2 != detected_n2:
                    raise ValueError(
                        f"Inconsistent player count detected in traj={traj_id}, cycle={cycle}: "
                        f"got ({n1}, {n2}), expected ({detected_n1}, {detected_n2})"
                    )

            if n1 != n2:
                raise ValueError(
                    f"Current curriculum format requires n1 == n2, but got n1={n1}, n2={n2} "
                    f"at traj={traj_id}, cycle={cycle}"
                )

            ball, left, right = show_to_players_n(
                line,
                n1=n1,
                n2=n2,
                ndigits=ndigits,
                prev=use_prev_frame,
                debug=debug,
            )

            left = normalize_players_for_npz(left)
            right = normalize_players_for_npz(right)

            if debug and len(left) > 0 and len(right) > 0:
                print(f"[NPZ-BEFORE-PACK] traj={traj_id} cycle={cycle}")
                print(f"  left[0]={left[0]}")
                print(f"  right[0]={right[0]}")

            vec = pack_frame_vector(ball, left, right)

            if debug:
                print(f"[NPZ-AFTER-PACK] traj={traj_id} cycle={cycle}")
                print(f"  ball={vec[:4]}")
                print(f"  left0={vec[4:9]}")
                print(f"  left1={vec[9:14]}")
                right0_start = 4 + 5 * n1
                print(f"  right0={vec[right0_start:right0_start + 5]}")

            traj_states.append(vec)
            traj_cycles.append(cycle)

        if len(traj_states) == 0:
            if verbose:
                print(f"[NPZ] skip empty traj {traj_id}: lines [{reset_line}, {goal_line}]")
            continue

        all_states.extend(traj_states)
        all_cycles.extend(traj_cycles)
        traj_offsets.append(len(all_states))
        num_kept_traj += 1

        if verbose:
            print(
                f"[NPZ] traj={traj_id} "
                f"frames={len(traj_states)} "
                f"cycles=[{traj_cycles[0]} -> {traj_cycles[-1]}]"
            )

    if len(all_states) == 0:
        raise ValueError("No valid show frames found; cannot build npz")

    states = np.stack(all_states, axis=0).astype(np.float32)
    cycles = np.asarray(all_cycles, dtype=np.int32)
    traj_offsets = np.asarray(traj_offsets, dtype=np.int32)

    np.savez_compressed(
        dst_npz_path,
        states=states,
        traj_offsets=traj_offsets,
        cycles=cycles,
    )

    if verbose:
        print(f"[NPZ] saved to: {dst_npz_path}")
        print(f"[NPZ] states.shape = {states.shape}")
        print(f"[NPZ] cycles.shape = {cycles.shape}")
        print(f"[NPZ] traj_offsets.shape = {traj_offsets.shape}")
        print(f"[NPZ] num_traj = {num_kept_traj}")
        if detected_n1 is not None:
            print(f"[NPZ] players_per_side = {detected_n1}")

    return states, traj_offsets, cycles


def build_generated_subset_rcg(
    rcg_path: str,
    log_path: str,
    dst_rcg_path: str = "generated_subset.rcg",
    reset_offset: int = +1,
    goal_offset: int = -3,
    verbose: bool = True,
) -> List[Tuple[int, int, int, int, int]]:
    """
    从 .out 和 .rcg 中提取严格相邻的 reset -> goal 区间，
    映射到 rcg 行号，做人工偏移修正后，生成新的 subset rcg 文件。
    """

    if verbose:
        print("# 1) 从.out抽取 reset 的 cycle", flush=True)
    reset_cycles = read_reset_cycles_from_out(log_path)

    if verbose:
        print("# 2) 只扫一次 .rcg，收集 lines / gm_change_cycles / cycle2line", flush=True)
    lines, gm_change_cycles, cycle2line = scan_rcg_once_for_build_subset(rcg_path)

    if verbose:
        print("# 3) 合并成统一时间轴", flush=True)
    timeline = merge_reset_and_gm_cycles(reset_cycles, gm_change_cycles)

    if verbose:
        print("# 4) 抽取严格相邻的 reset -> goal 区间", flush=True)
    intervals = extract_reset_goal_intervals(timeline)

    if verbose:
        print("Extracted intervals:")
        for start, end, side in intervals:
            print(f"  {start} -> {end} (Side: {side})")

    if verbose:
        print("# 5) 用内存中的 cycle2line 映射 rcg 行号区间", flush=True)

    intervals_with_lines: List[Tuple[int, int, int, int, int]] = []
    for reset_c, goal_c, side in intervals:
        if reset_c not in cycle2line:
            raise RuntimeError(f"reset cycle not found in rcg show lines: {reset_c}")
        if goal_c not in cycle2line:
            raise RuntimeError(f"goal cycle not found in rcg show lines: {goal_c}")

        intervals_with_lines.append((
            reset_c,
            cycle2line[reset_c],
            goal_c,
            cycle2line[goal_c],
            side,
        ))

    if verbose:
        print("# 6) 对行号做人工修正", flush=True)
    fixed_intervals = adjust_interval_lines_for_manual_rcg_fix(
        intervals_with_lines,
        reset_offset=reset_offset,
        goal_offset=goal_offset,
    )

    if verbose:
        print("\nFixed intervals with lines:")
        for rc, rl, gc, gl, side in fixed_intervals:
            print(
                f"reset_cycle={rc}, reset_line={rl}, "
                f"goal_cycle={gc}, goal_line={gl}, side={side}"
            )

    if verbose:
        print("# 7) 用内存中的 lines 生成新的 rcg 文件", flush=True)

    max_line = len(lines)
    with open(dst_rcg_path, "w", encoding="utf-8") as f:
        f.writelines(lines[:4])

        for reset_cycle, reset_line, _, goal_line, side in fixed_intervals:
            f.write(f"(playmode {reset_cycle} play_on)\n")

            start = max(1, reset_line)
            end = min(goal_line, max_line)
            if start > end:
                continue

            seg = lines[start - 1:end]

            if side == +1:
                f.writelines(seg)
            else:
                for line in seg:
                    f.write(flip_line_keep_left_as_self(line))

    if verbose:
        print(f"\nGenerated subset rcg: {dst_rcg_path}")

    return fixed_intervals


def build_generated_subset_npz(
    rcg_path: str,
    log_path: str,
    dst_npz_path: str = "trajectories.npz",
    reset_offset: int = +1,
    goal_offset: int = -3,
    ndigits: int = 4,
    use_prev_frame: bool = False,
    verbose: bool = True,
):
    """
    一步完成：
      1. 从 .out + .rcg 提取 reset -> goal 区间
      2. 映射到 rcg 行号
      3. 按左边统一视角解析每个区间
      4. 生成 curriculum 可直接读取的 trajectories.npz
    """
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


if __name__ == "__main__":
    print("tools.py")
    rcg_path = "./log/example/rcg/incomplete.rcg"
    log_path = "./slurm-example.out"

    fixed_intervals = build_generated_subset_rcg(
        rcg_path=rcg_path,
        log_path=log_path,
        dst_rcg_path="generated_subset.rcg",
        reset_offset=+1,
        goal_offset=-3,
        verbose=True,
    )