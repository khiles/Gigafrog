#!/usr/bin/env python3
"""Report which CAN addresses actually arrive, per bus.

    python frogpilot/tools/can_presence_check.py            # 20s, then a report
    python frogpilot/tools/can_presence_check.py --seconds 60
    python frogpilot/tools/can_presence_check.py --all      # also list every address seen

Why this exists: this Tesla port has no CAN fingerprint, and every CANParser is built
with an empty message list (carstate.py), so messages are registered lazily on first
access. An address that is NOT on the bus therefore decodes silently to zero rather than
raising — and for a signal like LgtSens_Night, zero means "DAY". Whether a message exists
cannot be answered from source; it has to be measured.

Read-only. Subscribes to `can` and counts addresses. Sends nothing.
"""
import argparse
import time

from collections import defaultdict

import cereal.messaging as messaging

# Candidates worth knowing about, and what each would unlock. Addresses are from
# opendbc/dbc/tesla_can.dbc (chassis) and tesla_raven_party.dbc (party).
CANDIDATES = {
  0x135: ("ESP_135h", "stability control, ABS event, brake lamp"),
  0x155: ("ESP_B", "wheel speeds (already used)"),
  0x283: ("BODY_R1", "light sensor, outside air temp, high beam state"),
  0x2BF: ("DAS_control", "openpilot longitudinal TX"),
  0x318: ("GTW_carState", "doors, blinkers (already used)"),
  0x348: ("GTW_status", "driver present, HVAC, power state"),
  0x368: ("DI_state", "cruise state, set speed (already used)"),
  0x398: ("GTW_carConfig", "RHD flag, park assist, radar hardware"),
  0x399: ("AutopilotStatus", "blind spot, dashboard speed limit, FCW, LDW"),
  0x3E9: ("DAS_bodyControls", "turn indicator, wipers, high beams (openpilot TX)"),
  0x45:  ("STW_ACTN_RQ", "cruise stalk, wiper stalk, gap button (already used)"),
  0x488: ("DAS_steeringControl", "openpilot steering TX"),
  0x211: ("RCM_status", "seatbelts, occupancy"),
}


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--seconds", type=float, default=20.0, help="how long to listen")
  parser.add_argument("--all", action="store_true", help="list every address seen, not just candidates")
  args = parser.parse_args()

  can_sock = messaging.sub_sock("can", timeout=100)

  seen: dict[int, set[int]] = defaultdict(set)   # bus -> addresses
  counts: dict[tuple[int, int], int] = defaultdict(int)  # (bus, addr) -> frames

  print(f"Listening for {args.seconds:.0f}s. Drive normally — some messages only appear when moving.\n")
  start = time.monotonic()
  total = 0

  while time.monotonic() - start < args.seconds:
    for msg in messaging.drain_sock(can_sock, wait_for_one=True):
      if msg.which() != "can":
        continue
      for frame in msg.can:
        # src >= 128 is openpilot's own echoed TX, not something the car sent
        seen[frame.src].add(frame.address)
        counts[(frame.src, frame.address)] += 1
        total += 1

  elapsed = time.monotonic() - start
  if total == 0:
    print("!! No CAN frames at all. Is the car on and the panda connected?")
    return

  print(f"{total} frames over {elapsed:.1f}s across buses {sorted(seen)}\n")
  print(f"{'ADDR':>6}  {'BUS':>3}  {'Hz':>6}  {'NAME':<22} WHAT IT WOULD UNLOCK")
  print("-" * 100)

  for addr in sorted(CANDIDATES):
    name, unlocks = CANDIDATES[addr]
    buses = [b for b in sorted(seen) if addr in seen[b]]
    if buses:
      for bus in buses:
        rate = counts[(bus, addr)] / elapsed
        marker = " (openpilot TX echo)" if bus >= 128 else ""
        print(f"0x{addr:03X}  {bus:>3}  {rate:>6.1f}  {name:<22} present{marker}")
    else:
      print(f"0x{addr:03X}    -       -  {name:<22} ABSENT — {unlocks} not available")

  if args.all:
    print("\nEvery address seen, by bus:")
    for bus in sorted(seen):
      addrs = " ".join(f"0x{a:03X}" for a in sorted(seen[bus]))
      print(f"\n  bus {bus} ({len(seen[bus])} addresses):\n    {addrs}")

  print("\nNotes:")
  print("  A candidate marked ABSENT cannot be used — reading it would silently return zero.")
  print("  0x399 absent would mean the blind spot fix has no effect.")
  print("  0x3E9 appearing only as an openpilot TX echo, with no car-side response, is")
  print("  consistent with the turn indicator request not reaching the body controller.")


if __name__ == "__main__":
  main()
