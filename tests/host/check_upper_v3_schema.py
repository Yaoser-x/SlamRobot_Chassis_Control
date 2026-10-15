import json
import re
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))

from upper_v3_codec import decode_payload, load_schema, parse_frame  # noqa: E402
from hil_upper_v3 import decode_report_payload  # noqa: E402


def main() -> int:
    schema = load_schema()
    fixture = json.loads((ROOT / "tests" / "fixtures" / "upper_v3_golden.json").read_text(encoding="utf-8"))
    header = (ROOT / "Service" / "communication" / "communication_protocol_types.h").read_text(encoding="utf-8")
    names_by_command = {message["command"]: name for name, message in schema["messages"].items()}
    macro_names = {
        "SET_VELOCITY":"SET_VELOCITY", "ESTOP":"ESTOP", "LINE_CTRL":"LINE_CTRL",
        "CLEAR_FAULT":"CLEAR_FAULT", "GET_INFO":"GET_INFO", "HELLO":"HELLO",
        "STATUS":"STATUS", "DIAGNOSTIC":"DIAGNOSTIC", "IMU_STATUS":"IMU_STATUS",
    }

    assert schema["protocol_version"] == fixture["protocol_version"] == 3
    for name, message in schema["messages"].items():
        occupied: set[int] = set()
        for field in message["fields"]:
            assert {"offset", "width", "type", "unit", "range", "invalid", "owner"} <= field.keys()
            field_bytes = set(range(field["offset"], field["offset"] + field["width"]))
            assert not occupied & field_bytes, f"overlapping field in {name}: {field['name']}"
            occupied |= field_bytes
        assert occupied == set(range(message["payload_length"])), f"layout gap in {name}"
        macro = rf"COMMUNICATION_{macro_names[name]}_PAYLOAD_LENGTH\s+{message['payload_length']}U"
        assert re.search(macro, header), f"C length differs for {name}"

    fixture_names = set()
    for entry in fixture["frames"]:
        command, payload = parse_frame(bytes.fromhex(entry["frame_hex"]))
        assert command == entry["command"]
        assert len(payload) == entry["payload_length"]
        fixture_names.add(entry["name"])
        name = names_by_command[command]
        decoded = decode_payload(name, payload, schema)
        # Reproduce the real HIL frames/report JSON path for every golden message,
        # including HELLO git_commit and DIAGNOSTIC reserved schema byte fields.
        report = {"frames": {name: [decode_report_payload(name, payload)]}}
        serialized = json.loads(json.dumps(report))["frames"][name][0]
        assert serialized.keys() == decoded.keys()
        for key, value in decoded.items():
            assert serialized[key] == (value.hex() if isinstance(value, bytes) else value)
    assert {"status_default_2wd", "status_4wd", "status_single_wheel_anomaly"} <= fixture_names
    assert decode_report_payload("0xff", b"\x00\xff") == {"payload_hex": "00ff"}

    by_name = {entry["name"]: entry for entry in fixture["frames"]}
    expected_masks = {
        "status_default_2wd": (0x06, 0x06, 0x00),
        "status_4wd": (0x0F, 0x0F, 0x00),
        "status_single_wheel_anomaly": (0x06, 0x02, 0x04),
    }
    for name, masks in expected_masks.items():
        _, payload = parse_frame(bytes.fromhex(by_name[name]["frame_hex"]))
        status = decode_payload("STATUS", payload, schema)
        assert (status["motor_enabled_mask"], status["motor_speed_valid_mask"], status["encoder_anomaly_mask"]) == masks
        assert status["motor_speed_valid_mask"] & ~status["motor_enabled_mask"] == 0
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
