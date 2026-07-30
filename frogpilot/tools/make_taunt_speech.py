#!/usr/bin/env python3
"""Render the Sissy Mode taunt lines to speech so the car speaks them.

    python frogpilot/tools/make_taunt_speech.py --list          # show the lines and voices
    python frogpilot/tools/make_taunt_speech.py                 # render into the repo asset dir
    python frogpilot/tools/make_taunt_speech.py --voice Daniel  # pick a macOS voice
    python frogpilot/tools/make_taunt_speech.py --out /tmp/x    # somewhere else

By default it renders into frogpilot/assets/taunt_speech/ — commit those and they ship with the
fork, so the device has them after its next update with nothing to copy. Use --out to render
somewhere else, then scp that to /data/media/taunt_speech/ on the device, which takes priority.

Why this runs here and not on the car: the device has no speech synthesiser, and installing one
onto AGNOS is fragile across updates. Rendering on a machine that already has a good one is both
better sounding and less to break. The cost is that you re-run this after editing a line.

Naming: each file is named by a hash of LINE 1 ONLY, matching taunt_speech_key in
selfdrive/ui/soundd.py. Line 2 carries live values such as the running offence count, so hashing
it would change the key every firing and leave the line silent. Only line 1 is spoken, which also
keeps the clip short enough to finish before the 4s alert clears. A line with no file is silent.

Output format is forced to mono 16-bit 48kHz because soundd asserts exactly that and will refuse
anything else.
"""
import argparse
import hashlib
import os
import platform
import shutil
import subprocess
import sys
import wave

from pathlib import Path

SAMPLE_RATE = 48000

REPO_ROOT = Path(__file__).resolve().parents[2]
# Rendering straight into the repo is the point: committed wavs ship with the fork and
# reach the device on the next update, so nothing has to be copied by hand.
ASSET_DIR = REPO_ROOT / "frogpilot/assets/taunt_speech"

sys.path.append(str(REPO_ROOT))


def taunt_speech_key(line_1: str) -> str:
  """Must stay identical to taunt_speech_key in selfdrive/ui/soundd.py.

  Line 1 only — line 2 carries live values (the running offence count), so including it would
  change the key every firing and leave the line silent."""
  return hashlib.sha1(line_1.encode()).hexdigest()[:16]


def load_taunts():
  """Read the pool straight from events.py so this can never drift from what the car says."""
  import ast
  events_py = REPO_ROOT / "selfdrive/selfdrived/events.py"
  tree = ast.parse(events_py.read_text())
  for node in tree.body:
    if isinstance(node, ast.Assign) and any(getattr(t, "id", None) == "SISSY_TAUNTS" for t in node.targets):
      return ast.literal_eval(node.value)
  raise SystemExit(f"could not find SISSY_TAUNTS in {events_py}")


def have(binary):
  return shutil.which(binary) is not None


def render_macos(text, out_path, voice):
  """macOS: `say` writes AIFF, `afconvert` gets it into the format soundd demands. Both built in."""
  aiff = out_path.with_suffix(".aiff")
  cmd = ["say", "-o", str(aiff)]
  if voice:
    cmd += ["-v", voice]
  subprocess.run(cmd + [text], check=True)
  subprocess.run(["afconvert", str(aiff), str(out_path),
                  "-d", "LEI16@48000", "-c", "1", "-f", "WAVE"], check=True)
  aiff.unlink()


def render_espeak(text, out_path, voice):
  cmd = ["espeak-ng", "-w", str(out_path), "-s", "150"]
  if voice:
    cmd += ["-v", voice]
  subprocess.run(cmd + [text], check=True)
  normalise(out_path)


def normalise(path):
  """espeak writes 22kHz; resample crudely to what soundd asserts. Speech survives this fine."""
  import numpy as np
  with wave.open(str(path), "r") as w:
    rate, frames, channels = w.getframerate(), w.getnframes(), w.getnchannels()
    data = np.frombuffer(w.readframes(frames), dtype=np.int16)
  if channels > 1:
    data = data[::channels]
  if rate != SAMPLE_RATE:
    n = int(len(data) * SAMPLE_RATE / rate)
    data = np.interp(np.linspace(0, len(data) - 1, n), np.arange(len(data)), data).astype("<i2")
  with wave.open(str(path), "w") as w:
    w.setnchannels(1)
    w.setsampwidth(2)
    w.setframerate(SAMPLE_RATE)
    w.writeframes(data.tobytes())


def pick_renderer():
  if platform.system() == "Darwin" and have("say") and have("afconvert"):
    return render_macos, "macOS say"
  if have("espeak-ng"):
    return render_espeak, "espeak-ng"
  raise SystemExit(
    "No speech synthesiser found.\n"
    "  macOS: `say` and `afconvert` are built in — you should not see this.\n"
    "  Linux: install espeak-ng (apt install espeak-ng), or render elsewhere and copy the wavs in."
  )


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--out", default=str(ASSET_DIR),
                      help="output directory (defaults to the in-repo asset dir)")
  parser.add_argument("--voice", default=None, help="voice name (macOS: say -v '?' to list)")
  parser.add_argument("--list", action="store_true", help="print the lines and exit")
  parser.add_argument("--force", action="store_true", help="re-render lines that already exist")
  args = parser.parse_args()

  taunts = load_taunts()

  if args.list:
    for trigger, lines in taunts.items():
      print(f"\n{trigger}:")
      for line_1, line_2 in lines:
        print(f"  [{taunt_speech_key(line_1)}] {line_1} / {line_2}")
    if platform.system() == "Darwin":
      print("\nVoices: say -v '?'")
    return

  render, engine = pick_renderer()
  out_dir = Path(args.out)
  out_dir.mkdir(parents=True, exist_ok=True)
  print(f"Rendering with {engine}"
        f"{f' (voice: {args.voice})' if args.voice else ''} into {out_dir}/\n")

  rendered = skipped = failed = 0
  keys = set()
  for trigger, lines in taunts.items():
    for line_1, line_2 in lines:
      key = taunt_speech_key(line_1)
      keys.add(key)
      out_path = out_dir / f"{key}.wav"

      if out_path.exists() and not args.force:
        skipped += 1
        continue

      # Only line 1 is spoken: line 2 varies at runtime, and a short clip finishes
      # before the 4s alert clears instead of running on past it.
      spoken = line_1
      try:
        render(spoken, out_path, args.voice)
        rendered += 1
        print(f"  {key}  {line_1[:48]}")
      except subprocess.CalledProcessError as exc:
        failed += 1
        print(f"  FAILED {key}: {exc}")

  # Lines that were edited leave their old audio behind; say so rather than silently hoarding
  stale = [p for p in out_dir.glob("*.wav") if p.stem not in keys]
  print(f"\n{rendered} rendered, {skipped} already present, {failed} failed")
  if stale:
    print(f"{len(stale)} stale file(s) from edited or removed lines:")
    for p in stale:
      print(f"  {p.name}")
    print("Safe to delete — nothing references them.")

  if out_dir.resolve() == ASSET_DIR.resolve():
    print("\nRendered into the repo. Commit them and they ship to the device on the next update:")
    print(f"  git add {ASSET_DIR.relative_to(REPO_ROOT)} && git commit -m 'taunt speech' && git push")
  else:
    print(f"\nCopy to the device:\n  scp -r {out_dir}/ comma@<device>:/data/media/")


if __name__ == "__main__":
  main()
