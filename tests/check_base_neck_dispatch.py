"""Structural regression for the native HELIOS dispatch boundary (no match).

Runtime neck execution is checked separately with diagnostic WM/debug logs.
This guard prevents accidentally reintroducing a blanket post-dispatch neck
replacement while leaving the RL and Hybrid policies unchanged.
"""
from pathlib import Path
import unittest


SOURCE = (Path(__file__).resolve().parents[1] / "R2DRL/helios-base/src/player/sample_player.cpp").read_text()
TAKE_ACTION = SOURCE.split("void SamplePlayer::takeAction(int n) {", 1)[1].split("void SamplePlayer::setActionMask()", 1)[0]
TAIL = TAKE_ACTION[TAKE_ACTION.index("if (!this->effector().bodyCommand()") :]


class NativeHeliosNeckDispatch(unittest.TestCase):
    def test_native_helios_is_excluded_from_default_neck_replacement(self):
        self.assertRegex(TAIL, r"else\s+if\s*\(n != ACTION_HELIOS\)\s*\{(?:[^{}]|\n)*?setNeckAction\(new Neck_TurnToBallOrScan\(0\)\);")
        self.assertEqual(TAIL.count("setNeckAction("), 2)

    def test_wait_keeps_scan_and_body_fallback_is_unchanged(self):
        self.assertRegex(TAIL, r"if\s*\(n == ACTION_WAIT\)\s*\{\s*this->setNeckAction\(new Neck_ScanField\(\)\);")
        self.assertRegex(TAIL, r"if\s*\(!this->effector\(\).bodyCommand\(\) && !world\(\).self\(\).isFrozen\(\)\)\s*\{\s*this->doTurn\(0.0\);")

    def test_helios_level_branches_still_dispatch_without_neck_replacement(self):
        case = TAKE_ACTION.split("case 17:", 1)[1].split("case 18:", 1)[0]
        self.assertIn("runHeliosFrame_();", case)
        self.assertIn("runHeliosMaintenanceAction_();", case)
        self.assertNotIn("setNeckAction", case)

    def test_hybrid_native_and_wait_exceptions_remain(self):
        self.assertRegex(SOURCE, r"else if \(a != 4 && a != 5\)\s*\{\s*this->setNeckAction\(new Neck_TurnToBallOrScan\(0\)\);")


if __name__ == "__main__":
    unittest.main()
