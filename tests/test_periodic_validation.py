import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


class PeriodicValidationTest(unittest.TestCase):
    def run_validator(self, final_y, periodic_y=True):
        with tempfile.TemporaryDirectory(prefix="sfm-periodic-validation-") as directory:
            root = Path(directory)
            with (root / "trajectory.csv").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["time_s", "agent_id", "x_m", "y_m", "vx_m_per_s", "vy_m_per_s"])
                writer.writerow([0, 7, 19.9, 9.9, 1, 1])
                writer.writerow([1, 7, 0.1, final_y, 1, 1])
            script = Path(__file__).resolve().parents[1] / "scripts" / "check_periodic_run.py"
            command = [sys.executable, str(script), str(root), "--length", "20", "--agents", "1"]
            if periodic_y:
                command.extend(["--height", "10"])
            result = subprocess.run(command, text=True, capture_output=True)
            return result

    def test_corner_wrap_is_counted(self):
        result = self.run_validator(0.1)
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertTrue(report["all_y_positions_wrapped"])
        self.assertEqual(report["rightward_wraps_detected"], 1)
        self.assertEqual(report["upward_wraps_detected"], 1)

    def test_outside_y_is_rejected_only_when_periodic_y_is_requested(self):
        self.assertNotEqual(self.run_validator(10.1).returncode, 0)
        result = self.run_validator(10.1, periodic_y=False)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertNotIn("all_y_positions_wrapped", json.loads(result.stdout))


if __name__ == "__main__":
    unittest.main()
