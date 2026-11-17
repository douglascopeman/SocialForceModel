"""End-to-end configuration, window scheduling and output contract checks."""
import csv
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


SIMULATOR = str(Path(sys.argv.pop(1)).resolve())


class FieldOutputTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)

    def run_case(self, name, coarse="", duration=0.13, dt=0.03, success=True):
        output = self.root / name
        config = self.root / (name + ".toml")
        config.write_text(f'''[run]
name = "{name}"
seed = 123
duration_s = {duration}
time_step_s = {dt}
[domain]
width_m = 4.0
height_m = 3.0
boundary_x = "periodic"
boundary_y = "periodic"
[scenario]
type = "unidirectional"
direction_mode = "fixed_vector"
[agents]
count = 4
initial_placement = "uniform"
desired_speed_stddev_m_per_s = 0.0
[output]
directory = "{output}"
trajectory_interval_s = 0.03
{coarse}
''')
        result = subprocess.run([SIMULATOR, str(config)], capture_output=True, text=True)
        if success:
            self.assertEqual(result.returncode, 0, result.stderr)
        else:
            self.assertNotEqual(result.returncode, 0)
        return output

    @staticmethod
    def rows(path):
        with path.open() as stream:
            return list(csv.DictReader(stream))

    def test_default_disabled_and_trajectory_unchanged(self):
        baseline = self.run_case("baseline")
        disabled = self.run_case("disabled", "[coarse_graining]\nenabled = false")
        enabled = self.run_case("enabled", "[coarse_graining]\nenabled = true\nwindow_s = 0.05")
        self.assertFalse((baseline / "fields").exists())
        self.assertFalse((disabled / "fields").exists())
        self.assertEqual((baseline / "trajectory.csv").read_bytes(), (enabled / "trajectory.csv").read_bytes())
        self.assertEqual((baseline / "trajectory.csv").read_bytes(), (disabled / "trajectory.csv").read_bytes())

    def test_windows_metadata_conservation_and_field_selection(self):
        output = self.run_case("fields", '''[coarse_graining]
enabled = true
fields = ["polarisation", "density"]
grid_spacing_m = 0.7
kernel = "gaussian"
kernel_width_m = 0.4
kernel_cutoff_sigma = 3.0
window_s = 0.05
boundary_policy = "renormalize"
''') / "fields"
        metadata = json.loads((output / "metadata.json").read_text())
        self.assertEqual(metadata["fields"], ["polarisation", "density"])
        self.assertEqual(metadata["boundary_y"], "periodic")
        windows = self.rows(output / "windows.csv")
        self.assertEqual(len(windows), 3)
        for window, start, end in zip(windows, [0, 0.05, 0.1], [0.05, 0.1, 0.13]):
            self.assertAlmostEqual(float(window["time_start_s"]), start)
            self.assertAlmostEqual(float(window["time_end_s"]), end)
            self.assertAlmostEqual(float(window["integrated_density"]), 4)
            self.assertAlmostEqual(float(window["mean_in_domain_agents"]), 4)
            self.assertEqual(float(window["mean_excluded_agents"]), 0)
            rows = self.rows(output / window["file"])
            self.assertEqual(len(rows), 6 * 5)
            self.assertNotIn("u_x_m_per_s", rows[0])
            total = sum(float(row["rho_per_m2"]) * float(row["cell_width_m"]) * float(row["cell_height_m"]) for row in rows)
            self.assertAlmostEqual(total, 4)
            for row in rows:
                polarisation = float(row["polarisation"])
                if int(row["sample_count"]):
                    self.assertTrue(0 <= polarisation <= 1)
                else:
                    self.assertTrue(math.isnan(polarisation))

    def test_exact_windows_and_velocity_only(self):
        output = self.run_case("exact", '''[coarse_graining]
enabled = true
fields = ["velocity"]
window_s = 0.1
''', duration=0.2, dt=0.01) / "fields"
        windows = self.rows(output / "windows.csv")
        self.assertEqual(len(windows), 2, "no spurious final window from floating point residual")
        rows = self.rows(output / windows[0]["file"])
        self.assertIn("u_x_m_per_s", rows[0])
        self.assertNotIn("rho_per_m2", rows[0])
        self.assertNotIn("polarisation", rows[0])

    def test_window_shorter_than_step(self):
        output = self.run_case("short", "[coarse_graining]\nenabled = true\nwindow_s = 0.01", duration=0.07, dt=0.03) / "fields"
        windows = self.rows(output / "windows.csv")
        self.assertEqual(len(windows), 7)
        self.assertAlmostEqual(float(windows[-1]["time_end_s"]), 0.07)

    def test_configuration_errors(self):
        invalid = [
            'fields = ["unknown"]', 'fields = ["density", "density"]', 'fields = []',
            'fields = "density"', 'fields = [density]', 'fields = ["density" "velocity"]',
            'kernel_width_m = 0', 'window_s = nan', 'grid_spacing_m = 0.5oops',
            'boundary_policy = "unknown"', 'kernel = "unknown"', 'enable = true',
        ]
        for i, setting in enumerate(invalid):
            with self.subTest(setting=setting):
                self.run_case(f"invalid_{i}", "[coarse_graining]\nenabled = true\n" + setting, success=False)


if __name__ == "__main__":
    unittest.main()
