"""Safe-by-default Host USART3 Upper v3 HIL runner."""

from __future__ import annotations

import argparse
import json
import secrets
import time
from pathlib import Path

from upper_v3_codec import StreamDecoder, build_frame, decode_payload, encode_set_velocity, encode_version_only


COMMAND_NAMES = {0x80: "HELLO", 0x81: "STATUS", 0x82: "DIAGNOSTIC", 0x83: "IMU_STATUS"}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--platform", default="unspecified", help="set to raspberry-pi for compatibility evidence")
    parser.add_argument("--arm-motion", action="store_true", help="allow nonzero motion scenarios")
    args = parser.parse_args()
    try:
        import serial
    except ImportError as exc:
        raise SystemExit("pyserial is required for UART HIL") from exc

    session_id = secrets.randbits(64) or 1
    transcript: list[dict[str, object]] = []
    assertions: dict[str, bool] = {}
    decoder = StreamDecoder()
    frames: dict[str, list[dict[str, object]]] = {}

    def send(port: object, name: str, frame: bytes) -> None:
        port.write(frame)
        transcript.append({"direction": "tx", "name": name, "monotonic_ms": int(time.monotonic() * 1000), "hex": frame.hex()})

    def collect(port: object, duration: float) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            chunk = port.read(max(1, port.in_waiting))
            if not chunk:
                continue
            transcript.append({"direction": "rx", "monotonic_ms": int(time.monotonic() * 1000), "hex": chunk.hex()})
            for command, payload in decoder.feed(chunk):
                name = COMMAND_NAMES.get(command, f"0x{command:02x}")
                decoded = decode_payload(name, payload) if name in COMMAND_NAMES.values() else {"payload_hex": payload.hex()}
                if isinstance(decoded.get("git_commit"), bytes):
                    decoded["git_commit"] = decoded["git_commit"].hex()
                decoded["hil_receive_ms"] = int(time.monotonic() * 1000)
                frames.setdefault(name, []).append(decoded)

    with serial.Serial(args.port, args.baud, timeout=0.02) as port:
        port.reset_input_buffer()
        send(port, "GET_INFO", encode_version_only(0x05))
        send(port, "REARM_DISABLE", encode_set_velocity(0.0, 0.0, False, session_id, 1))

        malformed = bytearray(encode_version_only(0x05))
        malformed[-1] ^= 0xFF
        send(port, "CRC_ERROR_INJECTION", bytes(malformed))
        half = encode_version_only(0x05)
        port.write(half[:3])
        transcript.append({"direction": "tx", "name": "HALF_FRAME_PREFIX", "hex": half[:3].hex()})
        time.sleep(0.12)
        port.write(b"\x00\xffrandom")
        transcript.append({"direction": "tx", "name": "RANDOM_BYTES", "hex": b"\x00\xffrandom".hex()})
        send(port, "GET_INFO_AFTER_RESYNC", half)

        collect(port, args.duration)

        assertions["hello_received"] = bool(frames.get("HELLO"))
        assertions["status_received"] = bool(frames.get("STATUS"))
        assertions["diagnostic_received"] = bool(frames.get("DIAGNOSTIC"))
        assertions["imu_received"] = bool(frames.get("IMU_STATUS"))
        assertions["release_applied"] = any(
            status["session_id"] == session_id and status["applied_sequence"] == 1 and status["ack_flags"] & 0x04
            for status in frames.get("STATUS", [])
        )

        if assertions["release_applied"]:
            zero_enable = encode_set_velocity(0.0, 0.0, True, session_id, 2)
            send(port, "ZERO_ENABLE", zero_enable)
            for _ in range(3):
                collect(port, 0.05)
                send(port, "ZERO_KEEPALIVE_DUPLICATE", zero_enable)
            collect(port, 0.08)
            active_statuses = list(frames.get("STATUS", []))
            assertions["zero_command_applied"] = any(
                status["applied_sequence"] == 2 and status["ack_flags"] & 0x04 for status in active_statuses
            )
            assertions["keepalive_preserved_host_lease"] = any(
                status["control_source"] == 1 and status["applied_sequence"] == 2 for status in active_statuses
            )
            collect(port, 0.25)
            assertions["host_timeout_released_source"] = bool(frames.get("STATUS")) and frames["STATUS"][-1]["control_source"] != 1
            send(port, "FINAL_DISABLE", encode_set_velocity(0.0, 0.0, False, session_id, 3))
            collect(port, 0.1)
        else:
            assertions["zero_command_applied"] = False
            assertions["keepalive_preserved_host_lease"] = False
            assertions["host_timeout_released_source"] = False

        if args.arm_motion:
            send(port, "LOW_FORWARD", encode_set_velocity(0.05, 0.0, True, session_id, 4))
            time.sleep(0.1)
            send(port, "STOP", encode_set_velocity(0.0, 0.0, False, session_id, 5))

    report = {
        "runner": "hil_upper_v3.py",
        "timestamp_unix_ms": int(time.time() * 1000),
        "port": args.port,
        "baud": args.baud,
        "platform": args.platform,
        "session_id": session_id,
        "motion_armed": args.arm_motion,
        "assertions": assertions,
        "frames": frames,
        "transcript": transcript,
        "manual_pending": ["serial disconnect", "MCU restart", "ESTOP release", "physical stop evidence"]
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0 if all(assertions.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
