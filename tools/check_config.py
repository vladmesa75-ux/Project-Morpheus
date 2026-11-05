#!/usr/bin/env python3
"""Static consistency checks for imported Morpheus Due and Nano Config.h files.

This is not a compiler, a radio test, or a safety certification.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
DUE = ROOT / "firmware/airplane-mode/DueFlightComputerBase_v2/Config.h"
NANO = ROOT / "firmware/nano-receiver/NanoLoraServoReceiver_6Byte/Config.h"

def setting(source, name):
    match = re.search(r"^\s*(?:const\s+)?[A-Za-z_][\w\s*]*?\b" + re.escape(name) +
                      r"\s*=\s*([^;]+);", source, re.MULTILINE)
    if match is None:
        raise ValueError("Missing setting: " + name)
    expr = match.group(1).strip().rstrip("LlUu")
    if not re.fullmatch(r"(?:\d+|\d+(?:\.\d+)?[Ee][+\-]?\d+)", expr):
        raise ValueError("Unexpected value for " + name + ": " + expr)
    return int(float(expr))

def main():
    due = DUE.read_text(encoding="utf-8")
    nano = NANO.read_text(encoding="utf-8")
    pairs = [
        ("LORA_FREQUENCY_HZ", "LORA_FREQUENCY_HZ"),
        ("LORA_SPREADING_FACTOR", "LORA_SPREADING_FACTOR"),
        ("LORA_BANDWIDTH_HZ", "LORA_BANDWIDTH_HZ"),
        ("LORA_CODING_RATE_DENOMINATOR", "LORA_CODING_RATE_DENOMINATOR"),
        ("LORA_PACKET_BYTES", "PACKET_SIZE"),
    ]
    errors = []
    for left, right in pairs:
        a, b = setting(due, left), setting(nano, right)
        print(f"{left}: Due={a}, Nano={b}")
        if a != b:
            errors.append(f"{left} differs: Due={a}, Nano={b}")
    if "LoRa.enableCrc();" not in (ROOT / "firmware/airplane-mode/DueFlightComputerBase_v2/LoraTelemetry.ino").read_text():
        errors.append("Due hardware LoRa CRC is not enabled")
    if setting(nano,"USE_LORA_CRC") != 1:
        errors.append("Nano USE_LORA_CRC is disabled")
    if setting(due, "LORA_PACKET_BYTES") != 6:
        errors.append("Packet layout changed: update both endpoints and protocol documentation")
    if errors:
        for err in errors:
            print("ERROR:", err, file=sys.stderr)
        return 1
    print("Static Due/Nano LoRa settings and packet size agree.")
    print("NOTE: Due can emit throttle 255; Nano currently scales using UNSIGNED_INPUT_MAX=254.")
    print("NOTE: Due pin 43 is attached but tail fold control is NOT implemented in imported code.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
