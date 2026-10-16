"""Standard-library checks of the compiled synthetic fixture; no games or models."""
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def main():
    fixture = sys.argv[1]
    clean = {k: v for k, v in os.environ.items() if not k.startswith("R2DRL_WM_TRACE_")}

    def run(mode="trace", side="l", **config):
        result = subprocess.run([fixture, mode, side], env={**clean, **config},
                                text=True, capture_output=True, timeout=10)
        assert result.returncode == 0, (result.returncode, result.stdout, result.stderr)
        return result

    primitives = json.loads(run("primitives").stdout)
    assert primitives == ['quote"\n\\tab\t', None, None, None]
    row = json.loads(run("fixture").stdout)
    assert row["source_shm"] == 'quote"\n\\tab\t'
    assert row["self"]["unum"] == 6 and row["side"] == "l"
    assert row["commands"]["body"] == "(move -1.25 2.5)"
    assert row["commands"]["turn_neck"] == "(turn_neck 15)"
    assert row["ball"]["pos_valid"] is False
    assert row["commands"]["status"] == "queued_not_server_acknowledged"

    with tempfile.TemporaryDirectory(prefix="r2drl-wm-test-") as temporary:
        root = Path(temporary)
        # No output path: no file is created and no WM trace warning appears.
        disabled = run()
        assert "[WM_TRACE]" not in disabled.stderr
        assert not list(root.iterdir())

        def collect(name, side="l", **extra):
            destination = root / name
            destination.mkdir()
            result = run(side=side, R2DRL_WM_TRACE_DIR=str(destination), **extra)
            files = list(destination.glob("*.jsonl"))
            rows = [json.loads(line) for f in files for line in f.read_text().splitlines()]
            return result, files, rows

        _, files, rows = collect("enabled")
        assert len(files) == 1 and len(rows) == 6
        assert [r["phase"] for r in rows] == ["pre_decision", "post_decision"] * 3
        assert [r["decision_index"] for r in rows] == [1, 1, 2, 2, 3, 3]
        assert rows[0]["commands"] is None and rows[0]["base_action_id"] is None
        assert rows[1]["base_action_id"] == 17
        assert rows[4]["base_action_id"] is None and rows[4]["hybrid_action_id"] is None
        assert rows[5]["base_action_id"] is None and rows[5]["hybrid_action_id"] == 2
        assert rows[5]["hybrid_parameters"] == [0.25, 0.75]
        assert all(r["commands"]["body"] == "(move -1.25 2.5)" for r in rows[1::2])
        for before, after in zip(rows[::2], rows[1::2]):
            for key in ("self", "ball", "players", "intercept"):
                assert before[key] == after[key]

        _, files, rows = collect("right_only_excluded", R2DRL_WM_TRACE_SIDE="r")
        assert not files and not rows
        _, files, rows = collect("right_only_included", side="r", R2DRL_WM_TRACE_SIDE="r")
        assert len(files) == 1 and len(rows) == 6 and all(r["side"] == "r" for r in rows)
        _, _, rows = collect("sampled", R2DRL_WM_TRACE_EVERY="2")
        assert [r["decision_index"] for r in rows] == [1, 1, 3, 3]
        for value in ("0", "-1", "garbage", "1000001"):
            result, files, _ = collect("bad_interval_" + value, R2DRL_WM_TRACE_EVERY=value)
            assert not files and result.stderr.count("[WM_TRACE] disabled:") == 1
        result, files, _ = collect("bad_side", R2DRL_WM_TRACE_SIDE="red")
        assert not files and result.stderr.count("[WM_TRACE] disabled:") == 1
        missing = root / "does_not_exist"
        result = run(R2DRL_WM_TRACE_DIR=str(missing))
        assert not missing.exists() and result.stderr.count("[WM_TRACE] disabled:") == 1
    print("PASS: JSON, queued commands, both sides, opt-in, sampling, invalid configuration, I/O failure, and RNG/command nonmutation")


if __name__ == "__main__":
    main()
