#!/usr/bin/env python3
"""Log per-process memory over time, to find what is growing.

    python frogpilot/tools/memory_watch.py                    # 60s samples to /data/media/memory_watch.csv
    python frogpilot/tools/memory_watch.py --interval 30
    python frogpilot/tools/memory_watch.py --report           # summarise an existing log and exit

Why this exists: the device reports Low Memory and soft-disengages after an hour or more of
driving. That is a growth rate, not a level, so a single snapshot cannot find it — the process
that is 200 MB and stable is innocent and the one that is 40 MB and climbing is not. Static
review of the recent changes did not turn up an unbounded allocation, so this measures instead
of guessing.

Read-only: samples /proc and writes one CSV. Leave it running for a drive, then --report.
"""
import argparse
import csv
import os
import time

DEFAULT_LOG = "/data/media/memory_watch.csv"


def sample():
  """Per-process RSS in MB, plus system totals. psutil is available on the device."""
  import psutil

  rows = []
  for proc in psutil.process_iter(["pid", "name", "cmdline", "memory_info"]):
    try:
      info = proc.info
      mem = info["memory_info"]
      if mem is None or mem.rss < 5 * 1024 * 1024:  # ignore the long tail of tiny processes
        continue
      # openpilot runs many processes as "python", so the script name is what identifies them
      cmdline = info["cmdline"] or []
      name = info["name"] or "?"
      if "python" in name and len(cmdline) > 1:
        name = os.path.basename(cmdline[-1]) or name
      rows.append((info["pid"], name, mem.rss / 1e6))
    except (psutil.NoSuchProcess, psutil.AccessDenied):
      continue

  vm = psutil.virtual_memory()
  return rows, vm.percent, vm.available / 1e6


def report(path):
  """Rank processes by how much they grew between the first and last sample."""
  first: dict[str, float] = {}
  last: dict[str, float] = {}
  first_t = last_t = None
  sys_first = sys_last = None

  with open(path) as f:
    for row in csv.DictReader(f):
      t, name, rss = float(row["time"]), row["name"], float(row["rss_mb"])
      if first_t is None:
        first_t = t
      last_t = t
      if name not in first:
        first[name] = rss
      last[name] = rss
      if sys_first is None:
        sys_first = float(row["sys_percent"])
      sys_last = float(row["sys_percent"])

  if first_t is None:
    print("no samples in", path)
    return

  minutes = (last_t - first_t) / 60.0
  print(f"{path}: {minutes:.1f} minutes of samples")
  print(f"system memory used: {sys_first:.0f}% -> {sys_last:.0f}%\n")
  print(f"{'process':32} {'start MB':>9} {'end MB':>9} {'growth':>9} {'MB/hour':>9}")

  growth = sorted(((last[n] - first.get(n, last[n]), n) for n in last), reverse=True)
  for delta, name in growth[:20]:
    rate = delta / (minutes / 60.0) if minutes > 1 else float("nan")
    print(f"{name[:32]:32} {first.get(name, 0):9.1f} {last[name]:9.1f} {delta:+9.1f} {rate:+9.1f}")

  print("\nA process with a steady positive MB/hour is the leak. A large but flat one is not.")


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--interval", type=float, default=60.0, help="seconds between samples")
  parser.add_argument("--log", default=DEFAULT_LOG, help="where to write the CSV")
  parser.add_argument("--report", action="store_true", help="summarise an existing log and exit")
  args = parser.parse_args()

  if args.report:
    report(args.log)
    return

  new_file = not os.path.exists(args.log)
  with open(args.log, "a", newline="") as f:
    writer = csv.writer(f)
    if new_file:
      writer.writerow(["time", "pid", "name", "rss_mb", "sys_percent", "sys_available_mb"])

    print(f"Sampling every {args.interval:.0f}s to {args.log}. Ctrl-C to stop, then --report.")
    while True:
      rows, percent, available = sample()
      now = time.time()
      for pid, name, rss in rows:
        writer.writerow([f"{now:.0f}", pid, name, f"{rss:.1f}", f"{percent:.1f}", f"{available:.0f}"])
      f.flush()
      print(f"  {time.strftime('%H:%M:%S')}  system {percent:.0f}% used, {available:.0f} MB free, "
            f"{len(rows)} processes")
      time.sleep(args.interval)


if __name__ == "__main__":
  main()
