#!/usr/bin/env python3
"""Find what is leaking memory, from logs the device already recorded.

    python frogpilot/tools/memory_report.py                 # the most recent drive
    python frogpilot/tools/memory_report.py --list          # show available drives
    python frogpilot/tools/memory_report.py --route 2026-07-30--18-22-11
    python frogpilot/tools/memory_report.py --top 30
    python frogpilot/tools/memory_report.py --device        # run it on the car from here
    python frogpilot/tools/memory_report.py --device --longest   # pick the longest drive

Run it on the device, or from a machine on the same network with --device, which runs it over
SSH on the car and prints the report here. The logs never leave the device either way — only
the few lines of report come back, which matters because a route is gigabytes of video.

--longest picks the longest recorded drive rather than the most recent. For a leak that takes
about an hour to bite, the most recent drive is often a short one that never reached the
failure; the longest is the one that has the evidence in it.

Why this exists: openpilot's own proclogd already samples every process's RSS and the system
memory totals, and procLog is logged at one sample every 30s (cereal/services.py). So the drive
that ran out of memory already recorded exactly what was growing — there is no need to fit a
tool and drive again. This reads that back and ranks processes by growth rate.

A leak is a *rate*, not a level: the 250MB process that is flat is innocent, the 40MB one
climbing 60MB/hour is the culprit. The report sorts by MB/hour for that reason.
"""

from __future__ import annotations
# Annotations stay lazy so this also runs on older Pythons than the device ships

import argparse
import os
import subprocess
import sys

from pathlib import Path

# Same device defaults as make_taunt_speech.py
DEVICE_HOST = os.environ.get("FROGPILOT_DEVICE", "192.168.1.11")
DEVICE_USER = "comma"
DEVICE_REPO = "/data/openpilot"
# Running ON the device is the normal case for this tool; --device is only for driving it from
# another machine. Detecting that here means --device on the car degrades to "just run it" rather
# than trying to SSH to itself, which fails with a message about waking the car you are sitting in.
ON_DEVICE = os.path.isfile("/AGNOS")

sys.path.append(str(Path(__file__).resolve().parents[2]))


def route_root() -> Path:
  """Where loggerd writes, per system/hardware/hw.py."""
  for candidate in ("/data/media/0/realdata", "/data/media/0/realdata_HD",
                    "/data/media/0/realdata_konik"):
    if Path(candidate).is_dir():
      return Path(candidate)
  raise SystemExit("No route directory found. Pass --root if your logs live elsewhere.")


def segments_for(root: Path, route: str | None, longest: bool = False):
  """Segment directories for one route, in order. A route is <name>--<segment number>."""
  segs = sorted(p for p in root.iterdir() if p.is_dir() and "--" in p.name)
  if not segs:
    raise SystemExit(f"No segments under {root}")

  routes: dict[str, list[Path]] = {}
  for p in segs:
    routes.setdefault(p.name.rsplit("--", 1)[0], []).append(p)

  if route is None:
    # Segments are about a minute each, so segment count is a good proxy for drive length.
    route = max(routes, key=lambda r: len(routes[r])) if longest else sorted(routes)[-1]
  if route not in routes:
    raise SystemExit(f"Route {route!r} not found. Try --list.")
  return route, sorted(routes[route], key=lambda p: int(p.name.rsplit("--", 1)[1]))


def read_proclogs(segment_dirs):
  """Yield (monotime, {name: rss_bytes}, system_used_fraction) per procLog sample."""
  from openpilot.tools.lib.logreader import LogReader

  for seg in segment_dirs:
    log = next((seg / n for n in ("rlog.zst", "rlog.bz2", "rlog") if (seg / n).is_file()), None)
    if log is None:
      continue
    try:
      lr = LogReader(str(log))
    except Exception as exc:
      print(f"  (skipping {seg.name}: {exc})")
      continue

    for msg in lr:
      if msg.which() != "procLog":
        continue
      procs: dict[str, int] = {}
      for proc in msg.procLog.procs:
        # openpilot runs many processes as "python"; the script name is what identifies them
        name = proc.name
        cmdline = list(proc.cmdline)
        if "python" in name and len(cmdline) > 1:
          name = cmdline[-1].split("/")[-1] or name
        procs[name] = max(procs.get(name, 0), proc.memRss)
      mem = msg.procLog.mem
      used = 1.0 - (mem.available / mem.total) if mem.total else 0.0
      yield msg.logMonoTime / 1e9, procs, used


def run_on_device(host, passthrough):
  """Copy this script to the car, run it there, and stream its output back.

  It copies rather than invoking the device's own copy so the version on the car does not have
  to match this one. Otherwise a flag added here fails on a device that has not updated yet,
  with a remote argparse error that looks nothing like the real cause.

  The destination is inside the repo's own tools directory because the script locates openpilot
  relative to its own path; running it from /tmp would put the imports out of reach."""
  ssh_opts = ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
              "-o", "ConnectTimeout=8"]
  target = f"{DEVICE_REPO}/frogpilot/tools/_memory_report_remote.py"

  print(f"Running on {DEVICE_USER}@{host} ...\n")

  copy = subprocess.run(["scp"] + ssh_opts + [str(Path(__file__).resolve()),
                                              f"{DEVICE_USER}@{host}:{target}"],
                        capture_output=True, text=True)
  if copy.returncode != 0:
    err = (copy.stderr or "").strip().splitlines()
    print(f"Cannot reach {host}: {err[-1] if err else 'unknown error'}")
    print("A comma device is only on the network while the car is awake — wake the car and")
    print("retry. Override the address with --device-host.")
    return 1

  remote = " ".join(["cd", DEVICE_REPO, "&&", "python3", target] + [f"'{a}'" for a in passthrough])
  try:
    result = subprocess.run(["ssh"] + ssh_opts + [f"{DEVICE_USER}@{host}", remote])
    return result.returncode
  finally:
    # Leaving a stray script inside the repo would show up as a dirty working tree on the
    # device and can make the updater refuse to fast-forward.
    subprocess.run(["ssh"] + ssh_opts + [f"{DEVICE_USER}@{host}", f"rm -f {target}"],
                   capture_output=True)


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--root", default=None, help="route directory (default: the device's)")
  parser.add_argument("--route", default=None, help="route name (default: most recent)")
  parser.add_argument("--list", action="store_true", help="list available routes and exit")
  parser.add_argument("--top", type=int, default=15, help="how many processes to show")
  parser.add_argument("--longest", action="store_true",
                      help="analyse the longest recorded drive rather than the most recent")
  parser.add_argument("--device", action="store_true",
                      help=f"run this on the car at {DEVICE_HOST} over SSH and print the result here")
  parser.add_argument("--device-host", default=DEVICE_HOST,
                      help=f"address for --device (default {DEVICE_HOST})")
  args = parser.parse_args()

  if args.device and ON_DEVICE:
    print("Already running on the device — ignoring --device and reading the logs directly.\n")
    args.device = False

  if args.device:
    passthrough = [a for a in sys.argv[1:] if a not in ("--device",)
                   and not a.startswith("--device-host")]
    # strip the value of --device-host if it was given separately
    if "--device-host" in sys.argv:
      i = sys.argv.index("--device-host")
      drop = sys.argv[i + 1] if i + 1 < len(sys.argv) else None
      passthrough = [a for a in passthrough if a != drop]
    raise SystemExit(run_on_device(args.device_host, passthrough))

  root = Path(args.root) if args.root else route_root()

  if args.list:
    segs = sorted(p.name for p in root.iterdir() if p.is_dir() and "--" in p.name)
    routes: dict[str, int] = {}
    for name in segs:
      routes[name.rsplit("--", 1)[0]] = routes.get(name.rsplit("--", 1)[0], 0) + 1
    print(f"{len(routes)} route(s) under {root}:\n")
    for name in sorted(routes):
      print(f"  {name}  ({routes[name]} segments, ~{routes[name]} min)")
    return

  route, segment_dirs = segments_for(root, args.route, args.longest)
  print(f"Reading {route} ({len(segment_dirs)} segments) from {root}\n")

  first: dict[str, int] = {}
  last: dict[str, int] = {}
  first_t = last_t = None
  sys_first = sys_last = None
  samples = 0

  for t, procs, used in read_proclogs(segment_dirs):
    samples += 1
    if first_t is None:
      first_t, sys_first = t, used
    last_t, sys_last = t, used
    for name, rss in procs.items():
      first.setdefault(name, rss)
      last[name] = rss

  if not samples:
    raise SystemExit("No procLog messages found. Was logging disabled for this drive?")

  minutes = (last_t - first_t) / 60.0
  print(f"{samples} samples over {minutes:.0f} minutes")
  print(f"system memory used: {sys_first * 100:.0f}% -> {sys_last * 100:.0f}%\n")

  rows = []
  for name, end in last.items():
    start = first.get(name, end)
    delta_mb = (end - start) / 1e6
    rate = delta_mb / (minutes / 60.0) if minutes > 1 else 0.0
    rows.append((rate, delta_mb, start / 1e6, end / 1e6, name))
  rows.sort(reverse=True)

  print(f"{'process':34} {'start MB':>9} {'end MB':>9} {'growth':>9} {'MB/hour':>9}")
  for rate, delta, start, end, name in rows[:args.top]:
    print(f"{name[:34]:34} {start:9.1f} {end:9.1f} {delta:+9.1f} {rate:+9.1f}")

  worst = rows[0] if rows else None
  print()
  if worst and worst[0] > 5.0:
    print(f"Prime suspect: {worst[4]} growing {worst[0]:.0f} MB/hour.")
    print("A process with a steady positive rate is the leak; a large but flat one is innocent.")
  else:
    print("No process shows meaningful growth. If memory still ran out, the pressure is likely")
    print("outside these processes — check the system used figure above against free memory.")


if __name__ == "__main__":
  main()
