#!/usr/bin/env python3
"""Connect to both robots, continuously print their TCP positions every X seconds, and save to JSON."""

import json
import sys
import time
from datetime import datetime
from pathlib import Path

import uri_if

URI1_HOST = "192.168.56.101"
URI2_HOST = "192.168.57.101"

OUTPUT_FILE = Path(__file__).parent / "tcp_positions.json"


def fmt_pose(pose):
    return f"[{', '.join(f'{v:.4f}' for v in pose)}]"


def main():
    if len(sys.argv) < 2:
        print(f"Usage: python {sys.argv[0]} <interval_seconds>")
        sys.exit(1)

    interval = float(sys.argv[1])

    uri1 = uri_if.RMPLAB_Uri(URI1_HOST)
    uri2 = uri_if.RMPLAB_Uri(URI2_HOST)

    try:
        uri1.connect(False)
    except Exception as e:
        print(f"ERROR: Failed to connect URI1 ({URI1_HOST}): {e}")
        return
    try:
        uri2.connect(False)
    except Exception as e:
        print(f"ERROR: Failed to connect URI2 ({URI2_HOST}): {e}")
        uri1.disconnect()
        return

    print(f"Logging every {interval}s. Press Ctrl+C to stop.\n")

    try:
        while True:
            tcp1 = uri1.recieve.getActualTCPPose()
            tcp2 = uri2.recieve.getActualTCPPose()

            print(f"URI1 TCP: {fmt_pose(tcp1)}")
            print(f"URI2 TCP: {fmt_pose(tcp2)}")
            print()

            entry = {
                "timestamp": datetime.now().isoformat(),
                "uri1_tcp": list(tcp1),
                "uri2_tcp": list(tcp2),
            }

            # Append to existing data or start fresh
            data = []
            if OUTPUT_FILE.exists():
                try:
                    with open(OUTPUT_FILE, "r") as f:
                        data = json.load(f)
                except (json.JSONDecodeError, Exception):
                    data = []

            data.append(entry)

            with open(OUTPUT_FILE, "w") as f:
                json.dump(data, f, indent=2)

            time.sleep(interval)

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        uri1.disconnect()
        uri2.disconnect()
        print(f"Data saved to {OUTPUT_FILE}")


if __name__ == "__main__":
    main()
