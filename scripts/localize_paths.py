#!/usr/bin/env python3
"""Rewrite RoboCup2D YAML paths so this checkout is self-contained.

The training sources were copied from the running experiment tree, whose YAML files
contained absolute paths to that machine. This script rewrites only the runtime
path fields and keeps all scenario/algorithm settings unchanged.
"""
from __future__ import annotations

import argparse
from pathlib import Path

CONFIG_DIRS = [
    Path("R2DRL/robocup2d/config"),
    Path("algorithm/qmix/source/envs/robocup2d/config"),
    Path("algorithm/paradqn/source/environments/robocup2d/config"),
]

FIELD_PREFIXES = (
    "player_dir:",
    "coach_dir:",
    "server_path:",
    "trainer_dir:",
    "config_dir:",
    "player_config:",
    "trajectory_path:",
    "tb_log_dir:",
)


def _github_root() -> Path:
    return Path(__file__).resolve().parents[1]


def _replacement_for(root: Path, stripped: str, yaml_path: Path) -> str | None:
    r2drl = root / "R2DRL"
    mapping = {
        "player_dir:": f"player_dir: {r2drl / 'helios-base/src/player'}",
        "coach_dir:": f"coach_dir: {r2drl / 'helios-base/src/coach'}",
        "server_path:": f"server_path: {r2drl / 'rcssserver/build/rcssserver'}",
        "trainer_dir:": f"trainer_dir: {r2drl / 'helios-base/src/trainer'}",
        "config_dir:": f"config_dir: {r2drl / 'helios-base/src/formations-dt'}",
        "player_config:": f"player_config: {r2drl / 'helios-base/src/player.conf'}",
        "tb_log_dir:": "tb_log_dir: ./runs/${CONFIG_STEM}",
    }
    for prefix, value in mapping.items():
        if stripped.startswith(prefix):
            if prefix == "tb_log_dir:":
                return value.replace("${CONFIG_STEM}", yaml_path.stem)
            return value
    if stripped.startswith("trajectory_path:"):
        current = stripped.split(":", 1)[1].strip()
        if "scenarioes.npz" in current or "front-goal" in yaml_path.name:
            return f"trajectory_path: {r2drl / 'robocup2d/trajectories/scenarioes.npz'}"
        if "3v3trajectories_right_half_left_nearest_kickable.npz" in current:
            # Keep legacy 3v3 configs runnable only if that file is provided next to this package.
            return "trajectory_path: trajectories/3v3trajectories_right_half_left_nearest_kickable.npz"
    return None


def _lib_paths(root: Path) -> list[str]:
    r2drl = root / "R2DRL"
    return [
        str(r2drl / "librcsc/rcsc/.libs"),
        str(r2drl / "runtime_libs"),
        str(r2drl / "rcssserver/build/rcss/clang"),
        str(r2drl / "rcssserver/build/rcss/conf"),
        str(r2drl / "rcssserver/build/rcss/net"),
        str(r2drl / "rcssserver/build/rcss/gzip"),
    ]


def localize_yaml(path: Path, root: Path) -> bool:
    original = path.read_text()
    lines = original.splitlines(keepends=True)
    out: list[str] = []
    i = 0
    changed = False
    while i < len(lines):
        line = lines[i]
        stripped = line.strip()
        indent = line[: len(line) - len(line.lstrip(" "))]

        if stripped.startswith("lib_paths:"):
            out.append(f"{indent}lib_paths:\n")
            for item in _lib_paths(root):
                out.append(f"{indent}- {item}\n")
            changed = True
            i += 1
            while i < len(lines):
                next_stripped = lines[i].strip()
                next_indent = lines[i][: len(lines[i]) - len(lines[i].lstrip(" "))]
                if not next_stripped:
                    out.append(lines[i])
                    i += 1
                    break
                if next_stripped.startswith("-") and len(next_indent) >= len(indent):
                    i += 1
                    continue
                break
            continue

        if any(stripped.startswith(prefix) for prefix in FIELD_PREFIXES):
            replacement = _replacement_for(root, stripped, path)
            if replacement is not None:
                out.append(f"{indent}{replacement}\n")
                changed = changed or (line != out[-1])
                i += 1
                continue

        out.append(line)
        i += 1

    updated = "".join(out)
    if updated != original:
        path.write_text(updated)
        return True
    return False


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="Only report files that would be inspected.")
    args = parser.parse_args()

    root = _github_root()
    yaml_files: list[Path] = []
    for rel in CONFIG_DIRS:
        directory = root / rel
        if directory.is_dir():
            yaml_files.extend(sorted(directory.glob("*.yaml")))

    changed = []
    for path in yaml_files:
        if not args.check and localize_yaml(path, root):
            changed.append(path.relative_to(root))

    if args.check:
        print(f"checked_roots={len(CONFIG_DIRS)} yaml_files={len(yaml_files)}")
    else:
        print(f"localized_yaml_files={len(changed)}")
        for rel in changed:
            print(rel)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
