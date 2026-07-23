"""ESP v3 HTTP/WebSocket HIL runner; motion remains disarmed by default."""

from __future__ import annotations

import argparse
import json
import time
import urllib.request
from pathlib import Path


def fetch_json(url: str) -> dict[str, object]:
    with urllib.request.urlopen(url, timeout=2.0) as response:
        return json.load(response)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", required=True, help="ESP IP or hostname")
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--arm-motion", action="store_true")
    args = parser.parse_args()
    base = f"http://{args.host}"
    before = fetch_json(base + "/status")
    assertions = {
        "protocol_v3": before.get("pv") == 3,
        "link_online": before.get("online") is True,
        "hello_valid": before.get("hello_valid") is True,
        "build_identity_capability": bool(int(before.get("capabilities", 0)) & 0x10),
    }
    websocket_events: list[object] = []
    if args.arm_motion:
        try:
            import websocket
        except ImportError as exc:
            raise SystemExit("websocket-client is required with --arm-motion") from exc
        ws = websocket.create_connection(f"ws://{args.host}:81", timeout=2)
        ws.send('{"cmd":"claim"}')
        websocket_events.append(ws.recv())
        ws.send('{"cmd":"stop"}')
        time.sleep(0.1)
        ws.close()
    after = fetch_json(base + "/status")
    report = {
        "runner": "hil_esp_v3.py",
        "timestamp_unix_ms": int(time.time() * 1000),
        "host": args.host,
        "motion_armed": args.arm_motion,
        "assertions": assertions,
        "status_before": before,
        "status_after": after,
        "websocket_events": websocket_events,
        "manual_pending": ["owner/observer contention", "disconnect stop", "fault then new disable ACK"]
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0 if all(assertions.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
