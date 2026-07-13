#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    assert spec and spec.loader
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


roadmap = load("analyze_roadmap_data", ROOT / "scripts" / "analyze_roadmap_data.py")
imu = load("analyze_imu", ROOT / "scripts" / "analyze_imu.py")
hil = load("hil_smoke", ROOT / "scripts" / "hil_smoke.py")
hil_imu = load("hil_imu_calibration", ROOT / "scripts" / "hil_imu_calibration.py")
straight = load("analyze_straight_hil", ROOT / "scripts" / "analyze_straight_hil.py")


class AnalysisTests(unittest.TestCase):
    def test_straight_hil_excludes_caster_transition_and_emits_ram_only_commands(self):
        telemetry = [
            {"straight_transition_distance_m": 0.10,
             "actual_left_mps": 0.30, "actual_right_mps": 0.10},
            {"straight_transition_distance_m": 0.31,
             "actual_left_mps": 0.17, "actual_right_mps": 0.13},
            {"straight_transition_distance_m": 0.60,
             "actual_left_mps": 0.16, "actual_right_mps": 0.14},
        ]
        measurements = [
            {"run_id": "f015-1", "direction": "forward", "speed_mps": "0.15",
             "caster_transition": "1", "distance_m": "0.30", "lateral_error_m": "0.20",
             "yaw_error_deg": "8", "firmware_sha": "deadbeef", "battery_v": "12.0"},
            {"run_id": "f015-1", "direction": "forward", "speed_mps": "0.15",
             "caster_transition": "0", "distance_m": "2.00", "lateral_error_m": "0.04",
             "yaw_error_deg": "1.5", "firmware_sha": "deadbeef", "battery_v": "11.9"},
        ]
        report = straight.analyze(telemetry, measurements, {})
        group = report["matrix"]["forward_015"]
        self.assertEqual(group["caster_transition_runs"], 1)
        self.assertEqual(group["steady_runs"], 1)
        self.assertTrue(group["meets_geometry"])
        self.assertAlmostEqual(report["trim_updates"]["straight_trim_forward_015_mps"], 0.015)
        commands = "\n".join(report["ram_commands"])
        self.assertIn("set straight_trim_forward_015_mps 0.015000", commands)
        self.assertNotIn("save", commands.lower())
        self.assertFalse(report["acceptance"]["matrix_complete"])
        self.assertFalse(report["acceptance"]["passed"])

    def test_straight_hil_requires_one_firmware_identity(self):
        measurements = []
        for direction in ("forward", "reverse"):
            for speed in ("0.15", "0.30"):
                for index in range(5):
                    measurements.append({
                        "run_id": f"{direction}-{speed}-{index}", "direction": direction,
                        "speed_mps": speed, "caster_transition": "0", "distance_m": "2.0",
                        "lateral_error_m": "0.02", "yaw_error_deg": "1.0",
                        "firmware_sha": "sha-a" if index < 4 else "sha-b", "battery_v": "12.0",
                    })
        report = straight.analyze([], measurements, {})
        self.assertTrue(report["acceptance"]["matrix_complete"])
        self.assertFalse(report["acceptance"]["firmware_identity_consistent"])
        self.assertFalse(report["acceptance"]["passed"])

    def test_straight_hil_counts_unique_runs_and_rejects_wrong_speed(self):
        measurements = []
        for direction in ("forward", "reverse"):
            for speed in ("0.15", "0.30"):
                for _ in range(5):
                    measurements.append({
                        "run_id": f"same-{direction}-{speed}", "direction": direction,
                        "speed_mps": speed, "caster_transition": "0", "distance_m": "2.0",
                        "lateral_error_m": "0.02", "yaw_error_deg": "1.0",
                        "firmware_sha": "sha-a", "battery_v": "12.0",
                    })
        report = straight.analyze([], measurements, {})
        self.assertFalse(report["acceptance"]["enough_final_runs"])

        measurements[0]["speed_mps"] = "0.20"
        with self.assertRaises(ValueError):
            straight.analyze([], measurements, {})

    def test_straight_hil_rejects_any_missing_firmware_identity(self):
        measurements = [{
            "run_id": "f015-1", "direction": "forward", "speed_mps": "0.15",
            "caster_transition": "0", "distance_m": "2.0", "lateral_error_m": "0.02",
            "yaw_error_deg": "1.0", "firmware_sha": "", "battery_v": "12.0",
        }]
        report = straight.analyze([], measurements, {})
        self.assertFalse(report["acceptance"]["firmware_identity_consistent"])

    def test_line_separation_and_polarity(self):
        rows = []
        for surface, value in (("floor", "900"), ("floor", "880"),
                               ("line", "100"), ("line", "120")):
            row = {"surface": surface}
            row.update({f"ch{i}": value for i in range(8)})
            rows.append(row)
        result = roadmap.analyze_line(rows)
        self.assertTrue(result["accepted"])
        self.assertTrue(result["channels"]["ch0"]["active_low"])
        self.assertEqual(result["channels"]["ch0"]["threshold"], 500.0)

    def test_encoder_validity_is_statistical(self):
        rows = [{"m1_speed_mps": "0.1", "m1_valid": "1"},
                {"m1_speed_mps": "0.2", "m1_valid": "0"}]
        result = roadmap.analyze_encoder(rows)
        self.assertEqual(result["motors"]["m1"]["valid_ratio"], 0.5)

    def test_imu_drift_uses_sample_timestamps(self):
        rows = [
            {"t_ms": "0", "roll_deg": "0", "pitch_deg": "0", "yaw_deg": "1", "temperature_c": "20"},
            {"t_ms": "60000", "roll_deg": "0.3", "pitch_deg": "0.4", "yaw_deg": "3", "temperature_c": "21"},
        ]
        result = imu.analyze(rows)
        self.assertEqual(result["yaw_drift_deg_per_min"], 2.0)
        self.assertAlmostEqual(result["level_return_error_deg"], 0.5)

    def test_hil_report_has_per_command_assertions(self):
        responses = [
            {"command": "version", "response": "version fw=2.0.0 sha=deadbeef build=Debug protocol=2 param=3 diagnostic=1"},
            {"command": "status", "response": "POST: ok\nPOST done=1\nPARAM ok"},
            {"command": "i2cscan", "response": "I2C scan done"},
            {"command": "imutest", "response": "bmi270 chip=0x24"},
            {"command": "espflash status", "response": "ESPFLASH active=0"},
        ]
        report = hil.build_report("COM1", 115200, "POST: boot", responses,
                                  "2026-01-01T00:00:00+00:00")
        self.assertTrue(report["passed"])
        self.assertTrue(all(item["assertions"] for item in report["commands"]))
        responses[2]["response"] = ""
        self.assertFalse(hil.build_report("COM1", 115200, "POST: boot", responses,
                                         "2026-01-01T00:00:00+00:00")["passed"])

    def test_hil_report_accepts_connection_after_boot_banner(self):
        responses = [
            {"command": "version", "response": "version fw=2.0.0 sha=deadbeef build=Release protocol=2 param=3 diagnostic=1"},
            {"command": "status", "response": "POST done=1 errors=0\nPARAM vmax=500"},
            {"command": "i2cscan", "response": "I2C scan done"},
            {"command": "imutest", "response": "bmi270 chip=0x24"},
            {"command": "espflash status", "response": "ESPFLASH active=0"},
        ]
        report = hil.build_report("COM3", 115200, "", responses,
                                  "2026-01-01T00:00:00+00:00")
        self.assertTrue(report["passed"])

    def test_hil_imu_timeout_snapshot_requires_all_monitored_tasks(self):
        text = "\n".join(
            f"RTOS {task} stack_free=100 missed=0 timeout={index}"
            for index, task in enumerate(hil_imu.MONITORED_TASKS)
        )
        self.assertEqual(
            hil_imu.timeout_snapshot(text),
            {task: index for index, task in enumerate(hil_imu.MONITORED_TASKS)},
        )
        with self.assertRaisesRegex(AssertionError, "missing RTOS task lines: oled"):
            hil_imu.timeout_snapshot(text.rsplit("\n", 1)[0])


if __name__ == "__main__":
    unittest.main()
