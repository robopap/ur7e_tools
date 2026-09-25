import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
UI = ROOT / "ur7e_tools" / "human_reference_ui.py"


class HumanReferenceSingleRecordingUiContractTest(unittest.TestCase):
    def test_ui_uses_one_recording_protocol(self):
        source = UI.read_text(encoding="utf-8")
        self.assertIn("recordings_complete = valid_reps == [1]", source)
        self.assertIn('QLabel("1 RECORDING")', source)
        self.assertIn('self.status.setText("RECORDING READY")', source)
        self.assertNotIn("Rep 1 / 3", source)

    def test_ui_exposes_zero_world_x_build_option(self):
        source = UI.read_text(encoding="utf-8")
        self.assertIn("QCheckBox", source)
        self.assertIn('QCheckBox("ZERO WORLD X (Robot1 Z)")', source)
        self.assertIn('args.append("--zero-world-x")', source)


if __name__ == "__main__":
    unittest.main()
