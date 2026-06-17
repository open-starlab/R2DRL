import re
import math
import matplotlib.pyplot as plt

RCG_FRAME_TEXT = r"""
(show 30 ((b) 44.7247 0.3719 0.1133 -0.7255) ((l 1) 0 0x9 -49.4234 0.2634 -0 -0 -93.509 90 (v h 180) (fp 0 0) (s 8000 1 1 130600) (f l 11) (c 0 1 142 0 1 144 3 3 0 0 30 0)) ((l 2) 1 0x1 0.6372 11.3897 0 0 150.95 4 (v h 60) (fp 0 0) (s 8000 0.85717 1 130600) (f l 11) (c 0 10 130 0 1 141 3 3 0 0 30 0)) ((l 3) 2 0x1 -0.8365 -3.8472 0 -0 -74.218 -90 (v h 60) (fp 0 0) (s 8000 0.987597 1 130600) (f l 11) (c 0 11 128 0 1 140 3 4 0 0 30 0)) ((l 4) 3 0x1 7.5619 24.3031 0.0005 -0.0002 -161.454 -41 (v h 60) (fp 0 0) (s 8000 0.846571 1 130600) (f l 11) (c 0 20 119 0 1 140 3 3 0 0 30 0)) ((l 5) 4 0x1 8.6894 -14.7653 0 0 -95.01 -90 (v h 60) (fp 0 0) (s 8000 0.810835 1 130600) (f l 11) (c 0 14 124 0 1 139 3 3 0 0 30 0)) ((l 6) 5 0x1 23.0512 4.6426 0.1166 -0.0029 -102.541 -90 (v h 60) (fp 0 0) (s 8000 0.828529 1 130600) (f l 11) (c 0 25 112 0 1 138 3 3 0 0 30 0)) ((l 7) 6 0x1 31.4383 17.4255 0.0436 -0.073 -62.962 -90 (v h 60) (fp 0 0) (s 7999.6 0.953574 1 130550) (f l 11) (c 0 20 117 0 1 138 3 3 0 0 30 0)) ((l 8) 7 0x1 34.7294 2.6678 0 0 20.784 84 (v h 60) (fp 0 0) (s 8000 0.941101 1 130600) (f l 11) (c 0 17 119 0 1 137 3 2 0 0 30 0)) ((l 9) 8 0x1 36.9566 25.0515 0.0009 -0.0005 10.673 -53 (v h 120) (fp 0 0) (s 8000 0.899266 1 130600) (f l 11) (c 0 23 113 0 1 137 6 2 0 0 30 0)) ((l 10) 9 0x1 44.1039 1.0265 0.0353 0.0284 -33.751 -41 (v h 120) (fp 0 0) (s 8000 0.813651 1 130600) (f l 11) (c 0 26 109 0 1 136 4 2 0 0 30 0)) ((l 11) 10 0x1 36.3948 15.8295 0.087 -0.2304 -64.104 -50 (v h 120) (fp 0 0) (s 7946.78 0.915587 1 130553) (c 1 19 115 0 1 136 4 3 0 0 30 0)) ((r 1) 0 0x9 49.8224 5.9949 0 -0 -104.387 -90 (v h 180) (fp 0 0) (s 8000 1 1 130600) (f r 11) (c 0 3 131 0 1 135 1 3 0 0 30 0)) ((r 2) 1 0x1 41.4962 6.0375 -0.0104 -0.0035 178.048 90 (v h 180) (fp 0 0) (s 8000 0.85717 1 130600) (f r 11) (c 0 23 111 0 1 135 4 3 0 0 30 0)) ((r 3) 2 0x1 41.4678 13.5901 -0.0721 -0.0211 -87.564 90 (v h 60) (fp 0 0) (s 7967.07 0.987597 1 130553) (f r 7) (c 0 22 111 0 1 134 3 4 0 0 30 0)) ((r 4) 3 0x1 37.6301 -4.0703 -0.0192 0.0093 179.966 -90 (v h 120) (fp 0 0) (s 8000 0.846571 1 130600) (f r 11) (c 0 26 106 0 1 133 4 3 0 0 30 0)) ((r 5) 4 0x1 42.3504 33.1793 -0.0043 -0.0009 86.107 -31 (v h 60) (fp 0 0) (s 8000 0.810835 1 130600) (f r 8) (c 0 22 110 0 1 133 3 3 0 0 30 0)) ((r 6) 5 0x1 32.3831 12.4705 -0.0107 0.0005 -128.11 -90 (v h 60) (fp 0 0) (s 8000 0.828529 1 130600) (f r 7) (c 0 24 107 0 1 132 3 4 0 0 30 0)) ((r 7) 6 0x1 21.9719 -1.1957 -0 0 84.286 52 (v h 60) (fp 0 0) (s 8000 0.953574 1 130600) (f r 8) (c 0 14 117 0 1 132 3 4 0 0 30 0)) ((r 8) 7 0x1 24.1695 17.9296 -0.0001 -0 91.171 -50 (v h 60) (fp 0 0) (s 8000 0.941101 1 130600) (f r 9) (c 0 14 116 0 1 131 3 3 0 0 30 0)) ((r 9) 8 0x1 10.5848 -15.2282 -0.0059 0.0009 92.451 -90 (v h 60) (fp 0 0) (s 8000 0.899266 1 130600) (f r 8) (c 0 25 104 0 1 130 3 3 0 0 30 0)) ((r 10) 9 0x1 11.7849 26.026 -0.0037 0.0001 -59.406 9 (v h 60) (fp 0 0) (s 8000 0.813651 1 130600) (f r 11) (c 0 24 105 0 1 130 3 3 0 0 30 0)) ((r 11) 10 0x1 10.4372 10.4988 -0.0076 0.0099 110.443 14 (v h 60) (fp 0 0) (s 8000 0.915587 1 130600) (f r 7) (c 0 14 114 0 1 129 3 3 0 0 30 0)))
""".strip()


def parse_rcg_show_frame(text: str):
    m = re.search(r"\(\(b\)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\)", text)
    if not m:
        raise ValueError("Ball not found.")
    bx, by, bvx, bvy = map(float, m.groups())

    players = []
    for tm, unum, x, y, vx, vy, body, neck in re.findall(
        r"\(\(([lr])\s+(\d+)\)\s+\d+\s+0x[0-9a-fA-F]+\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)",
        text
    ):
        players.append({
            "team": tm,
            "unum": int(unum),
            "x": float(x),
            "y": float(y),
            "vx": float(vx),
            "vy": float(vy),
            "body": float(body),
            "neck": float(neck),
        })

    if len(players) != 22:
        raise ValueError(f"Expected 22 players, got {len(players)}. Regex may need adjustment.")
    return (bx, by, bvx, bvy), players


def draw_and_save_frame(
    text: str,
    out_path: str = "rcg_cycle201.png",
    *,
    title: str = "RoboCup RCG frame",
    show_numbers: bool = True,
    dpi: int = 200,
):
    (bx, by, bvx, bvy), players = parse_rcg_show_frame(text)

    left = [p for p in players if p["team"] == "l"]
    right = [p for p in players if p["team"] == "r"]

    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_aspect("equal", adjustable="box")

    ax.set_xlim(-55, 55)
    ax.set_ylim(-38, 38)
    ax.axhline(0, linewidth=0.8)
    ax.axvline(0, linewidth=0.8)

    GOAL_X = 52.5
    GOAL_HALF_W = 7.01
    ax.plot([GOAL_X, GOAL_X], [-GOAL_HALF_W, GOAL_HALF_W], linewidth=4)
    ax.plot([-GOAL_X, -GOAL_X], [-GOAL_HALF_W, GOAL_HALF_W], linewidth=4)

    # --- vertical flip (上下翻转) ---
    ax.invert_yaxis()

    # Plot players
    ax.scatter([p["x"] for p in left], [p["y"] for p in left], marker="o", label="Left")
    ax.scatter([p["x"] for p in right], [p["y"] for p in right], marker="^", label="Right")

    # --- draw player orientation (body angle), also flipped ---
    ORI_LEN = 1.8

    def deg2vec_flipped(deg: float):
        # y 翻转相当于镜像：theta' = -theta
        rad = math.radians(-deg)
        ux = math.cos(rad) * ORI_LEN
        uy = -math.sin(rad) * ORI_LEN  # still need negate because invert_yaxis()
        return ux, uy

    lx = [p["x"] for p in left]
    ly = [p["y"] for p in left]
    lu, lv = [], []
    for p in left:
        ux, uy = deg2vec_flipped(p["body"])
        lu.append(ux)
        lv.append(uy)
    ax.quiver(lx, ly, lu, lv, angles="xy", scale_units="xy", scale=1, width=0.003)

    rx = [p["x"] for p in right]
    ry = [p["y"] for p in right]
    ru, rv = [], []
    for p in right:
        ux, uy = deg2vec_flipped(p["body"])
        ru.append(ux)
        rv.append(uy)
    ax.quiver(rx, ry, ru, rv, angles="xy", scale_units="xy", scale=1, width=0.003)

    # Plot ball + velocity arrow (keep as-is)
    ax.scatter([bx], [by], marker="x", s=80, label="Ball")
    ax.arrow(bx, by, bvx * 3, bvy * 3, head_width=0.5, length_includes_head=True)

    if show_numbers:
        for p in left:
            ax.text(p["x"], p["y"], str(p["unum"]), fontsize=9, ha="center", va="center")
        for p in right:
            ax.text(p["x"], p["y"], str(p["unum"]), fontsize=9, ha="center", va="center")

    vmag = math.sqrt(bvx * bvx + bvy * bvy)
    ax.set_title(f"{title}  |  ball v={vmag:.3f}  (cycle 201)")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(out_path, dpi=dpi)
    plt.close(fig)


if __name__ == "__main__":
    draw_and_save_frame(RCG_FRAME_TEXT, out_path="rcg_cycle201.png", title="RoboCup frame from (show 201)")
    print("Saved: rcg_cycle201.png")
