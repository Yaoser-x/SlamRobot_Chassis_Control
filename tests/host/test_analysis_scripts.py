#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import sys
import tempfile
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
control_analysis = load("analyze_control", ROOT / "scripts" / "analyze_control.py")
architecture = load(
    "check_architecture_dependencies",
    ROOT / "scripts" / "check_architecture_dependencies.py",
)
naming = load(
    "check_naming_conventions",
    ROOT / "scripts" / "check_naming_conventions.py",
)
ownership = load(
    "check_service_ownership",
    ROOT / "scripts" / "check_service_ownership.py",
)


class AnalysisTests(unittest.TestCase):
    def test_control_analysis_reports_expected_metrics_and_rejects_bad_time(self):
        rows = []
        for index, speed in enumerate((0.0, 0.1, 0.5, 0.9, 1.1, 1.0, 1.0, 1.0, 1.0, 1.0)):
            rows.append({
                "time_s": str(index * 0.1), "target_mps": "1.0",
                "left_mps": str(speed - 0.01), "right_mps": str(speed + 0.01),
                "left_permille": "1000" if index == 2 else "500",
                "right_permille": "500", "heading_error_deg": "2.0",
            })
        report = control_analysis.analyze(rows)
        self.assertAlmostEqual(report["rise_time_s"], 0.2)
        self.assertAlmostEqual(report["overshoot_mps"], 0.1)
        self.assertAlmostEqual(report["saturation_ratio"], 0.1)
        self.assertAlmostEqual(report["mean_left_right_speed_difference_mps"], 0.02)
        rows[2]["time_s"] = rows[1]["time_s"]
        with self.assertRaises(ValueError):
            control_analysis.analyze(rows)

    def test_five_layer_architecture_dependencies(self):
        self.assertEqual(architecture.analyze(ROOT), [])
        self.assertEqual(naming.analyze(ROOT), [])
        self.assertEqual(ownership.analyze(ROOT), [])

    @staticmethod
    def write_fixture(root: Path, relative: str, source: str) -> None:
        path = root / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(source, encoding="utf-8")

    def test_architecture_rejects_private_service_internal_include(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(root, "Service/motion/internal/private.h", "#pragma once\n")
            self.write_fixture(root, "App/bad.c", '#include "private.h"\n')
            errors = architecture.analyze(root)
            self.assertTrue(any("private Service internal header" in error for error in errors), errors)

    def test_architecture_rejects_public_service_dependency_leak(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(root, "BSP/adc/power_adc_driver.h", "#pragma once\n")
            self.write_fixture(root, "Service/power/power_service.h", '#include "power_adc_driver.h"\n')
            errors = architecture.analyze(root)
            self.assertTrue(any("public Service header leaks BSP" in error for error in errors), errors)

    def test_architecture_rejects_public_service_hardware_type_leak(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(root, "Service/power/power_status.h", "typedef struct motor_driver_state_t power_status_t;\n")
            errors = architecture.analyze(root)
            self.assertTrue(any("public Service header leaks hardware type" in error for error in errors), errors)

    def test_architecture_rejects_public_service_callback_port(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(
                root,
                "Service/example/example_service.h",
                "typedef void (*example_provider_t)(unsigned value);\n",
            )
            errors = architecture.analyze(root)
            self.assertTrue(any("public Service callback port is forbidden" in error for error in errors), errors)

    def test_architecture_rejects_imported_service_callback_alias(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(
                root,
                "Service/example/example_service.h",
                '#include "external_callbacks.h"\n'
                "void Example_Configure(external_provider_t provider);\n",
            )
            errors = architecture.analyze(root)
            self.assertTrue(any("public Service callback alias is forbidden" in error for error in errors), errors)

    def test_hil_imu_success_requires_terminal_state(self):
        self.assertTrue(hil_imu.calibration_succeeded("IMU acal=4,1,1"))
        self.assertFalse(hil_imu.calibration_succeeded("IMU acal=3,500,1"))

    def test_architecture_rejects_forbidden_top_level_directory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(root, "Common/common.h", "#pragma once\n")
            errors = architecture.analyze(root)
            self.assertIn("Common/: forbidden top-level architecture directory", errors)

    def test_architecture_rejects_new_mixed_configuration_consumer(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(root, "Domain/config/control_config.h", "#pragma once\n")
            self.write_fixture(root, "Service/power/power_service.c", '#include "control_config.h"\n')
            errors = architecture.analyze(root)
            self.assertTrue(any("legacy mixed configuration" in error for error in errors), errors)

    def test_architecture_rejects_unlisted_app_bsp_include(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(root, "BSP/motor/motor_driver.h", "#pragma once\n")
            self.write_fixture(root, "App/tasks/task_motor.c", '#include "motor_driver.h"\n')
            errors = architecture.analyze(root)
            self.assertTrue(any("App BSP include" in error for error in errors), errors)

    def test_architecture_rejects_app_bsp_include_beside_adapter(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(root, "BSP/led/status_led_driver.h", "#pragma once\n")
            self.write_fixture(root, "App/tasks/status_led_adapter.c", '#include "status_led_driver.h"\n')
            self.write_fixture(root, "App/tasks/task_led.c", '#include "status_led_driver.h"\n')
            self.write_fixture(
                root,
                "CMakeLists.txt",
                "set(F407_APP_ADAPTER_SOURCES\n"
                "    App/tasks/status_led_adapter.c\n"
                ")\n",
            )
            errors = architecture.analyze(root)
            self.assertTrue(any(error.startswith("App/tasks/task_led.c: App BSP include") for error in errors), errors)
            self.assertFalse(any(error.startswith("App/tasks/status_led_adapter.c: App BSP include") for error in errors), errors)

    def test_cmake_rejects_public_algorithm_link_from_service(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(
                root,
                "CMakeLists.txt",
                "target_link_libraries(f407_service\n"
                "    PUBLIC f407_algorithm\n"
                ")\n"
                "target_link_libraries(f407_app_adapters\n"
                "    PRIVATE f407_service\n"
                ")\n",
            )
            errors = architecture.cmake_dependency_errors(root)
            self.assertIn("CMakeLists.txt: f407_service must link f407_algorithm privately", errors)

    def test_cmake_rejects_algorithm_link_from_app_adapters(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(
                root,
                "CMakeLists.txt",
                "target_link_libraries(f407_service\n"
                "    PRIVATE f407_algorithm\n"
                ")\n"
                "target_link_libraries(f407_app_adapters\n"
                "    PRIVATE f407_service f407_algorithm\n"
                ")\n",
            )
            errors = architecture.cmake_dependency_errors(root)
            self.assertIn("CMakeLists.txt: f407_app_adapters must not link f407_algorithm", errors)

    def test_final_architecture_rejects_service_cycle_and_forbidden_edge(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for capability in architecture.FINAL_SERVICE_DEPENDENCIES:
                self.write_fixture(root, f"Service/{capability}/{capability}_service.h", "#pragma once\n")
            self.write_fixture(root, "Algorithm/pid.c", "void pid(void) {}\n")
            self.write_fixture(
                root,
                "Service/command_management/command_management_service.c",
                '#include "safety_management_service.h"\n',
            )
            self.write_fixture(
                root,
                "Service/safety_management/safety_management_service.c",
                '#include "command_management_service.h"\n',
            )
            errors = architecture.analyze(root, final=True)
            self.assertIn(
                "Service dependency not allowed: command_management -> safety_management",
                errors,
            )
            self.assertIn(
                "Service dependency cycle: command_management -> safety_management -> command_management",
                errors,
            )

    def test_architecture_rejects_bsp_cmsis_rtos_header(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(root, "BSP/motor/motor_driver.c", '#include "cmsis_os2.h"\nvoid f(void) {}\n')
            errors = architecture.analyze(root)
            self.assertTrue(any("forbidden BSP header" in error for error in errors), errors)

    def test_architecture_rejects_bsp_freertos_api(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(root, "BSP/imu/bmi270_driver.c", "void f(void) { osKernelGetTickCount(); }\n")
            errors = architecture.analyze(root)
            self.assertTrue(any("forbidden BSP API" in error for error in errors), errors)

    def test_naming_rejects_non_snake_case_and_final_legacy_layer(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(root, "Algorithm/BadName.c", "void bad(void) {}\n")
            self.write_fixture(root, "Domain/legacy.c", "void legacy(void) {}\n")
            errors = naming.analyze(root, final=True)
            self.assertTrue(any("source name must be lower_snake_case" in error for error in errors), errors)
            self.assertIn("Domain/: legacy layer name is forbidden in Beta5 final", errors)

    def test_ownership_rejects_motor_output_outside_motion(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(
                root,
                "Service/safety_management/safety_management_service.c",
                "void stop(void) { MotorDriver_StopAll(MOTOR_STOP_LOW_SIDE_BRAKE); }\n",
            )
            errors = ownership.analyze(root)
            self.assertTrue(any("owned only by Motion Control" in error for error in errors), errors)

    def test_ownership_rejects_bmi270_lifecycle_outside_state_estimation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self.write_fixture(
                root,
                "Service/safety_management/internal/power_on_self_test.c",
                "void probe(void) { (void)Bmi270Driver_ProbeNow(); }\n",
            )
            errors = ownership.analyze(root)
            self.assertTrue(any("owned only by State Estimation" in error for error in errors), errors)

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
        self.assertAlmostEqual(result["temperature_yaw_regression"]["slope"], 2.0)
        self.assertEqual(result["allan_deviation_yaw_deg"][0]["tau_s"], 60.0)

    def test_imu_six_face_report_is_evidence_only(self):
        rows = [
            {"t_ms": str(i), "roll_deg": "0", "pitch_deg": "0", "yaw_deg": "0", "temperature_c": "25",
             "face": face, "accel_x_g": x, "accel_y_g": "0", "accel_z_g": "0"}
            for i, (face, x) in enumerate((("+x", "1"), ("-x", "-1")))
        ]
        result = imu.analyze(rows)
        self.assertEqual(result["six_face_report"]["+x"]["accel_x_g"], 1.0)
        self.assertNotIn("parameter_update", result)

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
