import csv
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from analyse_lanes import analyse, expected_imbalance


class LaneAnalysisTest(unittest.TestCase):
    def test_exact_random_label_baseline(self):
        self.assertEqual(expected_imbalance(4, 0), 0)
        self.assertEqual(expected_imbalance(4, 1), 1)
        self.assertAlmostEqual(expected_imbalance(4, 2), 2 / 3)
        self.assertEqual(expected_imbalance(4, 4), 0)

    def test_perfect_lanes_and_speed(self):
        with tempfile.TemporaryDirectory(prefix="sfm-lane-test-") as directory:
            root = Path(directory)
            with (root / "trajectory.csv").open("w", newline="") as stream:
                writer = csv.writer(stream)
                writer.writerow(["time_s", "agent_id", "y_m", "vx_m_per_s"])
                for agent in range(4):
                    # All rightward agents in the lower strip, all leftward in upper.
                    writer.writerow([180, agent, 0.25 if agent % 2 == 0 else 0.75,
                                     1 if agent % 2 == 0 else -1])
            result = analyse(root, height=1, bins=2, late_window=60)
            self.assertEqual(result["late_mean_corrected_segregation"], 1)
            self.assertEqual(result["late_mean_forward_speed_m_per_s"], 1)
            self.assertEqual(result["samples_outside_y_walls"], 0)


if __name__ == "__main__":
    unittest.main()
