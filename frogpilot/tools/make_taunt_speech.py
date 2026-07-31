#!/usr/bin/env python3
"""Render the Sissy Mode taunt lines to speech so the car speaks them.

    python frogpilot/tools/make_taunt_speech.py --list          # show the lines and voices
    python frogpilot/tools/make_taunt_speech.py                 # render into the repo asset dir
    python frogpilot/tools/make_taunt_speech.py --voice Daniel  # pick a macOS voice
    python frogpilot/tools/make_taunt_speech.py --out /tmp/x    # somewhere else
    python frogpilot/tools/make_taunt_speech.py --deploy         # render, then copy to the device

Where the audio lives. soundd looks in /data/media/taunt_speech/ first and the in-repo asset dir
second, so there are two ways to get it onto the car:

  --deploy   copies straight to /data/media/taunt_speech/ over SSH. Preferred. The wavs are large
             binaries, so committing every re-render grows the git history by ~75M a time and it
             never shrinks. The device keeps them across updates because /data/media is not
             touched by the updater.
  committed  the in-repo asset dir ships with the fork and needs no network, but pays that cost.

--deploy needs the device powered on and reachable, which for a comma device means the car is
awake. It verifies what actually landed rather than trusting the copy.

Why this runs here and not on the car: the device has no speech synthesiser, and installing one
onto AGNOS is fragile across updates. Rendering on a machine that already has a good one is both
better sounding and less to break. The cost is that you re-run this after editing a line.

Naming: each file is named by a hash of LINE 1 ONLY, matching taunt_speech_key in
selfdrive/ui/soundd.py. Line 2 carries live values such as the running offence count, so hashing
it would change the key every firing and leave the line silent. Both lines ARE spoken — the hash
is only the filename and does not have to match the audio. A line with no file is silent.

Because the key ignores line 2, editing line 2 alone leaves the filename unchanged. manifest.json
records what each file actually says, so that case is caught and re-rendered instead of quietly
keeping the old audio.

Output format is forced to mono 16-bit 48kHz because soundd asserts exactly that and will refuse
anything else.
"""
import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
import wave

from pathlib import Path

SAMPLE_RATE = 48000

# A different voice per tier, so the escalation is audible and not only textual. None means "use
# whatever --voice or the system default gives you". macOS: `say -v '?'` lists what you have.
TIER_VOICES = {
  "mild": None,
  "harsh": None,
  "brutal": None,
}

# The car. Override with --device or FROGPILOT_DEVICE.
DEVICE_HOST = os.environ.get("FROGPILOT_DEVICE", "192.168.1.11")
DEVICE_USER = "comma"
# /data/media survives updates and is what soundd checks first, so a deploy here outranks
# whatever shipped in the repo.
DEVICE_DIR = "/data/media/taunt_speech"

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


def ssh_base(host):
  return ["ssh", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
          "-o", "ConnectTimeout=8", f"{DEVICE_USER}@{host}"]


def deploy(out_dir, host):
  """Copy the rendered audio to the device and verify what actually landed.

  Returns True on success. Everything here is deliberately loud about failure: a deploy that
  silently half-worked leaves some lines silent in the car, which is indistinguishable from a
  line simply not having been rendered."""
  wavs = sorted(out_dir.glob("*.wav"))
  if not wavs:
    print(f"Nothing to deploy — no wavs in {out_dir}")
    return False

  total_mb = sum(p.stat().st_size for p in wavs) / 1e6
  print(f"\nDeploying {len(wavs)} file(s), {total_mb:.0f}MB to {DEVICE_USER}@{host}:{DEVICE_DIR}/")

  probe = subprocess.run(ssh_base(host) + ["echo ok"], capture_output=True, text=True)
  if probe.returncode != 0:
    err = (probe.stderr or "").strip().splitlines()
    print(f"  Cannot reach {host}: {err[-1] if err else 'unknown error'}")
    print("  A comma device is only on the network while the car is awake. Wake the car and retry.")
    return False

  subprocess.run(ssh_base(host) + [f"mkdir -p {DEVICE_DIR}"], check=True)

  # rsync only sends what changed, which matters because re-rendering one line should not push
  # 75MB. It is not guaranteed to be on AGNOS, so fall back to tar over ssh, which needs nothing
  # that is not already there.
  if have("rsync") and subprocess.run(ssh_base(host) + ["command -v rsync"],
                                      capture_output=True).returncode == 0:
    cmd = ["rsync", "-az", "--delete", "--info=stats1",
           "-e", "ssh -o BatchMode=yes -o StrictHostKeyChecking=accept-new -o ConnectTimeout=8",
           f"{out_dir}/", f"{DEVICE_USER}@{host}:{DEVICE_DIR}/"]
    result = subprocess.run(cmd)
  else:
    print("  (rsync unavailable on one end, falling back to tar over ssh)")
    tar = subprocess.Popen(["tar", "cz", "-C", str(out_dir)] + [p.name for p in wavs]
                           + (["manifest.json"] if (out_dir / "manifest.json").exists() else []),
                           stdout=subprocess.PIPE)
    result = subprocess.run(ssh_base(host) + [f"tar xz -C {DEVICE_DIR}"], stdin=tar.stdout)
    tar.stdout.close()
    tar.wait()

  if result.returncode != 0:
    print(f"  Copy failed (exit {result.returncode}).")
    return False

  # Verify by size rather than by name. A truncated or zero-length wav is the failure that would
  # otherwise reach the car looking fine and simply play nothing.
  listing = subprocess.run(ssh_base(host) + [f"cd {DEVICE_DIR} && wc -c *.wav 2>/dev/null"],
                           capture_output=True, text=True)
  remote = {}
  for line in listing.stdout.splitlines():
    parts = line.split(None, 1)
    if len(parts) == 2 and parts[1].strip() != "total":
      remote[parts[1].strip()] = int(parts[0])

  missing = [p.name for p in wavs if p.name not in remote]
  wrong = [p.name for p in wavs if p.name in remote and remote[p.name] != p.stat().st_size]
  if missing or wrong:
    print(f"  Verification FAILED: {len(missing)} missing, {len(wrong)} wrong size")
    for n in (missing + wrong)[:10]:
      print(f"    {n}")
    return False

  print(f"  Verified {len(wavs)} file(s) on the device, all matching size.")
  print("  soundd reads /data/media first, so these take effect on the next drive.")
  return True


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument("--out", default=str(ASSET_DIR),
                      help="output directory (defaults to the in-repo asset dir)")
  parser.add_argument("--voice", default=None,
                      help="voice for every tier (macOS: say -v '?' to list). Per-tier voices "
                           "set in TIER_VOICES override this.")
  parser.add_argument("--list", action="store_true", help="print the lines and exit")
  parser.add_argument("--force", action="store_true", help="re-render lines that already exist")
  parser.add_argument("--deploy", action="store_true",
                      help=f"copy the result to the device at {DEVICE_HOST} over SSH")
  parser.add_argument("--device", default=DEVICE_HOST,
                      help=f"device address for --deploy (default {DEVICE_HOST})")
  args = parser.parse_args()

  taunts = load_taunts()

  if args.list:
    for trigger, tiers in taunts.items():
      print(f"\n{trigger}:")
      for tier, lines in tiers.items():
        print(f"  {tier}:")
        for line_1, line_2 in lines:
          print(f"    [{taunt_speech_key(line_1)}] {line_1} / {line_2}")
    if platform.system() == "Darwin":
      print("\nVoices: say -v '?'")
    return

  render, engine = pick_renderer()
  out_dir = Path(args.out)
  out_dir.mkdir(parents=True, exist_ok=True)
  print(f"Rendering with {engine}"
        f"{f' (voice: {args.voice})' if args.voice else ''} into {out_dir}/\n")

  # Because the key covers line 1 only, editing line 2 leaves the filename unchanged and the old
  # audio would be silently reused. The manifest records what each file actually says so that is
  # detected and re-rendered rather than quietly wrong.
  manifest_path = out_dir / "manifest.json"
  try:
    manifest = json.loads(manifest_path.read_text())
  except (FileNotFoundError, ValueError):
    manifest = {}

  rendered = skipped = failed = stale_text = 0
  keys = set()
  for trigger, tiers in taunts.items():
    for tier, lines in tiers.items():
      for line_1, line_2 in lines:
        key = taunt_speech_key(line_1)
        keys.add(key)
        out_path = out_dir / f"{key}.wav"

        spoken = f"{line_1}. {line_2}"

        if out_path.exists() and not args.force:
          expected = f"[{TIER_VOICES.get(tier) or args.voice or 'default'}] {spoken}"
          if manifest.get(key) == expected:
            skipped += 1
            continue
          stale_text += 1   # text or voice changed under an unchanged line 1

        voice = TIER_VOICES.get(tier) or args.voice
        try:
          render(spoken, out_path, voice)
          # Record the voice too: changing a tier's voice must re-render even though the words
          # are identical, which a text-only manifest would miss.
          manifest[key] = f"[{voice or 'default'}] {spoken}"
          rendered += 1
          print(f"  {key}  {line_1[:48]}")
        except subprocess.CalledProcessError as exc:
          failed += 1
          print(f"  FAILED {key}: {exc}")

  # Lines that were edited leave their old audio behind; say so rather than silently hoarding
  stale = [p for p in out_dir.glob("*.wav") if p.stem not in keys]
  manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True))

  print(f"\n{rendered} rendered, {skipped} already present, {failed} failed")
  if stale_text:
    print(f"  ({stale_text} of those re-rendered because line 2 changed under an unchanged line 1)")
  if stale:
    print(f"{len(stale)} stale file(s) from edited or removed lines:")
    for p in stale:
      print(f"  {p.name}")
    print("Safe to delete — nothing references them. --deploy removes them from the device too.")

  if args.deploy:
    if not deploy(out_dir, args.device):
      raise SystemExit(1)
    return

  print(f"\nGet it onto the car with:\n  {sys.argv[0]} --deploy")
  if out_dir.resolve() == ASSET_DIR.resolve():
    print("Or commit it, which needs no network but grows the git history by the full size of "
          "the audio\nevery re-render:")
    print(f"  git add {ASSET_DIR.relative_to(REPO_ROOT)} && git commit -m 'taunt speech' && git push")


if __name__ == "__main__":
  main()
