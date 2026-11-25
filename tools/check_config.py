#!/usr/bin/env python3
"""Static consistency checks for the airplane-mode Due and packet-timed Nano receiver.

This is not a compiler, a radio test, or a safety certification.
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
DUE = ROOT / "firmware/airplane-mode/DueFlightComputerBase_v2/Config.h"
NANO = ROOT / "firmware/nano-receiver/NanoFlaperonReceiver/Radio.h"

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
    # The receiver configures the radio directly in beginRadio().
    pairs = [
        ("LORA_FREQUENCY_HZ", r"LoRa\.begin\((\d+)[Ll]?\)"),
        ("LORA_SPREADING_FACTOR", r"LoRa\.setSpreadingFactor\((\d+)\)"),
        ("LORA_BANDWIDTH_HZ", r"LoRa\.setSignalBandwidth\((\d+)[Ll]?\)"),
        ("LORA_CODING_RATE_DENOMINATOR", r"LoRa\.setCodingRate4\((\d+)\)"),
        ("LORA_PACKET_BYTES", r"if\s*\(length != (\d+)\)"),
    ]
    errors = []
    for left, right in pairs:
        match = re.search(right, nano)
        if match is None:
            errors.append(f"Cannot read {left} from {NANO.name}")
            continue
        a, b = setting(due, left), int(match.group(1))
        print(f"{left}: Due={a}, Nano={b}")
        if a != b:
            errors.append(f"{left} differs: Due={a}, Nano={b}")
    if "LoRa.enableCrc();" not in (ROOT / "firmware/airplane-mode/DueFlightComputerBase_v2/LoraTelemetry.ino").read_text():
        errors.append("Due hardware LoRa CRC is not enabled")
    if "LoRa.enableCrc();" not in nano:
        errors.append("Nano hardware LoRa CRC is disabled")
    if setting(due, "LORA_PACKET_BYTES") != 6:
        errors.append("Packet layout changed: update both endpoints and protocol documentation")
    if errors:
        for err in errors:
            print("ERROR:", err, file=sys.stderr)
        return 1
    print("Static Due/Nano LoRa settings and packet size agree.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
