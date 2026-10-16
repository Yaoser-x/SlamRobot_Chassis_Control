"""The ESP policy gate must reject unsafe asynchronous bridge transitions."""
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
import sys
import unittest

sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "firmware/esp12f/F407_ESP12F/tools"))
import check_esp_safety_policy as policy

BRIDGE = (ROOT / "App/adapters/esp12f_flash_bridge.c").read_text(encoding="utf-8")


class EspSafetyPolicyTests(unittest.TestCase):
    def test_actual_sources_pass_the_complete_policy(self):
        with redirect_stdout(StringIO()):
            self.assertEqual(policy.main(), 0)

    def test_unsafe_bridge_mutations_are_rejected(self):
        changes = {
            "missing_maintenance_acquisition": (
                "AppMotionMaintenance_Begin() != APP_MOTION_MAINTENANCE_OK", "0U"),
            "restore_reentry_allowed": (
                "if (bridge_phase == ESP12F_BRIDGE_RESTORING)", "if (0U)"),
            "release_before_normal_boot": (
                "    Esp12fBootControl_EnterNormal();\n}",
                "    Esp12fFlashBridge_ReleaseMaintenance();\n    Esp12fBootControl_EnterNormal();\n}"),
            "rx_failure_without_restore": (
                "                Esp12fFlashBridge_BeginRestore();", "                return;"),
            "boot_readiness_ignored": (
                "Esp12fBootControl_Update(PlatformTime_NowMs()) != 0U", "1U"),
            "wireless_rx_not_restored": (
                "    WirelessCommunication_RestartRx();", ""),
            "maintenance_released_before_uart_restore": (
                "            Esp12fFlashBridge_RestoreServices();\n            bridge_phase = ESP12F_BRIDGE_OFF;\n            Esp12fFlashBridge_ReleaseMaintenance();",
                "            Esp12fFlashBridge_ReleaseMaintenance();\n            Esp12fFlashBridge_RestoreServices();\n            bridge_phase = ESP12F_BRIDGE_OFF;"),
            "idle_exit_removed": (
                "        Esp12fFlashBridge_Disable();", "        return;"),
        }
        for name, (before, after) in changes.items():
            with self.subTest(name=name):
                self.assertIn(before, BRIDGE)
                self.assertFalse(policy.bridge_holds_maintenance(BRIDGE.replace(before, after)))


if __name__ == "__main__":
    unittest.main()
