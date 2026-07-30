#!/usr/bin/env python3
"""Find what is leaking memory, from logs the device already recorded.

    python frogpilot/tools/memory_report.py                 # the most recent drive
    python frogpilot/tools/memory_report.py --list          # show available drives
    python frogpilot/tools/memory_report.py --route 2026-07-30--18-22-11
    python frogpilot/tools/memory_report.py --top 30

Run it on the device (or anywhere the route directory is reachable).

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
import sys

from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parents[2]))


def route_root() -> Path:
  """Where loggerd writes, per system/hardware/hw.py."""
  for candidate in ("/data/media/0/realdata", "/data/media/0/realdata_HD",
                    "/data/media/0/realdata_konik"):
    if Path(candidate).is_dir():
      return Path(candidate)
  raise SystemExit("No route directory found. Pass --root if your logs live elsewhere.")


def segments_for(root: Path, route: str | None):
  """Segment directories for one route, in order. A route is <name>--<segment number>."""
  segs = sorted(p for p in root.iterdir() if p.is_dir() and "--" in p.name)
  if not segs:
    raise SystemExit(f"No segments under {root}")

  routes: dict[str, list[Path]] = {}
  for p in segs:
    routes.setdefault(p.name.rsplit("--", 1)[0], []).append(p)

  if route is None:
    route = sorted(routes)[-1]
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


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--root", default=None, help="route directory (default: the device's)")
  parser.add_argument("--route", default=None, help="route name (default: most recent)")
  parser.add_argument("--list", action="store_true", help="list available routes and exit")
  parser.add_argument("--top", type=int, default=15, help="how many processes to show")
  args = parser.parse_args()

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

  route, segment_dirs = segments_for(root, args.route)
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
