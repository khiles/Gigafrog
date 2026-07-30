import hashlib
import math
import numpy as np
import time
import wave

from pathlib import Path

from cereal import car, custom, messaging
from openpilot.common.basedir import BASEDIR
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.common.utils import retry
from openpilot.common.swaglog import cloudlog

from openpilot.system import micd
from openpilot.system.hardware import HARDWARE

from openpilot.frogpilot.common.frogpilot_variables import ACTIVE_THEME_PATH, ERROR_LOGS_PATH, RANDOM_EVENTS_PATH, get_frogpilot_toggles

SAMPLE_RATE = 48000
SAMPLE_BUFFER = 4096 # (approx 100ms)
MAX_VOLUME = 1.0
MIN_VOLUME = 0.1
SELFDRIVE_STATE_TIMEOUT = 5 # 5 seconds
FILTER_DT = 1. / (micd.SAMPLE_RATE / micd.FFT_SAMPLES)

AMBIENT_DB = 30 # DB where MIN_VOLUME is applied
DB_SCALE = 30 # AMBIENT_DB + DB_SCALE is where MAX_VOLUME is applied

VOLUME_BASE = 20
if HARDWARE.get_device_type() in ("tici", "tizi"):
  VOLUME_BASE = 10

AudibleAlert = car.CarControl.HUDControl.AudibleAlert

# Spoken Sissy Mode taunts, one wav per line, named by a hash of the text so the pool stays
# editable. Rendered off-device by frogpilot/tools/make_taunt_speech.py.
#
# Checked in that order: the repo copy ships with the fork and arrives on the device with any
# update, so rendered lines are always present without copying anything by hand. /data/media
# wins when both exist, so a line can still be dropped on the device directly without a commit.
#
# Loaded on demand rather than with the rest: soundd keeps every sound resident as float32,
# which is ~190KB per second of audio, and a pile of spoken lines would be a real memory cost.
TAUNT_SPEECH_PATHS = (Path("/data/media/taunt_speech"),
                      Path(BASEDIR) / "frogpilot/assets/taunt_speech")
TAUNT_CACHE_SIZE = 4


def taunt_speech_key(line_1: str, line_2: str) -> str:
  """Hash of the exact displayed text. Must match make_taunt_speech.py or nothing plays."""
  return hashlib.sha1(f"{line_1}\n{line_2}".encode()).hexdigest()[:16]

# FrogPilot variables
FrogPilotAudibleAlert = custom.FrogPilotCarControl.HUDControl.AudibleAlert


sound_list: dict[int, tuple[str, int | None, float]] = {
  # AudibleAlert, file name, play count (none for infinite)
  AudibleAlert.engage: ("engage.wav", 1, MAX_VOLUME),
  AudibleAlert.disengage: ("disengage.wav", 1, MAX_VOLUME),
  AudibleAlert.refuse: ("refuse.wav", 1, MAX_VOLUME),

  AudibleAlert.prompt: ("prompt.wav", 1, MAX_VOLUME),
  AudibleAlert.promptRepeat: ("prompt.wav", None, MAX_VOLUME),
  AudibleAlert.promptDistracted: ("prompt_distracted.wav", None, MAX_VOLUME),

  AudibleAlert.warningSoft: ("warning_soft.wav", None, MAX_VOLUME),
  AudibleAlert.warningImmediate: ("warning_immediate.wav", None, MAX_VOLUME),

  # FrogPilot variables
  FrogPilotAudibleAlert.angry: ("angry.wav", 1, MAX_VOLUME),
  FrogPilotAudibleAlert.continued: ("continued.wav", 1, MAX_VOLUME),
  FrogPilotAudibleAlert.dejaVu: ("dejaVu.wav", 1, MAX_VOLUME),
  FrogPilotAudibleAlert.doc: ("doc.wav", 1, MAX_VOLUME),
  FrogPilotAudibleAlert.fart: ("fart.wav", 1, MAX_VOLUME),
  FrogPilotAudibleAlert.firefox: ("firefox.wav", 1, MAX_VOLUME),
  FrogPilotAudibleAlert.goat: ("goat.wav", None, MAX_VOLUME),
  FrogPilotAudibleAlert.hal9000: ("hal9000.wav", 1, MAX_VOLUME),
  FrogPilotAudibleAlert.mail: ("mail.wav", 1, MAX_VOLUME),
  FrogPilotAudibleAlert.nessie: ("nessie.wav", 1, MAX_VOLUME),
  FrogPilotAudibleAlert.noice: ("noice.wav", 1, MAX_VOLUME),
  FrogPilotAudibleAlert.startup: ("startup.wav", 1, MAX_VOLUME),
  FrogPilotAudibleAlert.thisIsFine: ("this_is_fine.wav", 1, MAX_VOLUME),
  FrogPilotAudibleAlert.uwu: ("uwu.wav", 1, MAX_VOLUME),
  # Resolved at play time from the alert text, so there is no fixed file here.
  FrogPilotAudibleAlert.sissyTaunt: (None, 1, MAX_VOLUME),
}
if HARDWARE.get_device_type() in ("tici", "tizi"):
  sound_list.update({
    AudibleAlert.engage: ("engage_tizi.wav", 1, MAX_VOLUME),
    AudibleAlert.disengage: ("disengage_tizi.wav", 1, MAX_VOLUME),
  })

def check_selfdrive_timeout_alert(sm):
  ss_missing = time.monotonic() - sm.recv_time['selfdriveState']

  if ss_missing > SELFDRIVE_STATE_TIMEOUT:
    if sm['selfdriveState'].enabled and (ss_missing - SELFDRIVE_STATE_TIMEOUT) < 10:
      return True

  return False


class Soundd:
  def __init__(self):
    self.current_alert = AudibleAlert.none
    self.current_volume = MIN_VOLUME
    self.current_sound_frame = 0

    self.selfdrive_timeout_alert = False

    self.spl_filter_weighted = FirstOrderFilter(0, 2.5, FILTER_DT, initialized=False)

    # FrogPilot variables
    self.params_memory = Params(memory=True)

    self.frogpilot_toggles = get_frogpilot_toggles()

    self.openpilot_crashed_played = False

    self.auto_volume = 0

    self.previous_sound_pack = None

    # key -> samples, bounded so a long taunt pool cannot grow soundd's memory
    self.taunt_sounds: dict[str, np.ndarray] = {}
    self.taunt_current: np.ndarray | None = None

    self.error_log = ERROR_LOGS_PATH / "error.txt"
    self.random_events_directory = RANDOM_EVENTS_PATH / "sounds"

    self.update_frogpilot_sounds()

  def load_sounds(self):
    self.loaded_sounds: dict[int, np.ndarray] = {}

    # Load all sounds
    for sound in sound_list:
      filename, play_count, volume = sound_list[sound]
      if filename is None:  # resolved at play time, see load_taunt_speech
        continue

      random_events_path = self.random_events_directory / filename
      sounds_path = self.sound_directory / filename

      if not sounds_path.exists() and "_tizi" in filename:
        standard_path = self.sound_directory / filename.replace("_tizi", "")
        if standard_path.exists():
          sounds_path = standard_path

      if random_events_path.exists():
        wavefile = wave.open(str(random_events_path), 'r')
      elif sounds_path.exists():
        wavefile = wave.open(str(sounds_path), 'r')
      else:
        if filename == "startup.wav":
          filename = "engage.wav"
        wavefile = wave.open(BASEDIR + "/selfdrive/assets/sounds/" + filename, 'r')

      assert wavefile.getnchannels() == 1
      assert wavefile.getsampwidth() == 2
      assert wavefile.getframerate() == SAMPLE_RATE

      length = wavefile.getnframes()
      self.loaded_sounds[sound] = np.frombuffer(wavefile.readframes(length), dtype=np.int16).astype(np.float32) / (2**16/2)

  def load_taunt_speech(self, line_1, line_2):
    """Resolve the spoken line for the taunt about to play, loading it only if needed.

    A missing file is normal, not a fault: the pool is meant to be edited and a line that has
    not been rendered yet simply has no audio. Never raise here — soundd dying takes every
    alert sound with it, including the safety-relevant ones.
    """
    key = taunt_speech_key(line_1, line_2)

    if key not in self.taunt_sounds:
      path = next((p / f"{key}.wav" for p in TAUNT_SPEECH_PATHS if (p / f"{key}.wav").is_file()), None)
      if path is None:
        return None
      try:
        with wave.open(str(path), "r") as wavefile:
          if (wavefile.getnchannels() != 1 or wavefile.getsampwidth() != 2 or
              wavefile.getframerate() != SAMPLE_RATE):
            cloudlog.warning(f"soundd: {path} is not mono/16-bit/{SAMPLE_RATE}Hz, skipping")
            return None
          frames = wavefile.readframes(wavefile.getnframes())
        samples = np.frombuffer(frames, dtype=np.int16).astype(np.float32) / (2**16/2)
      except FileNotFoundError:
        return None
      except Exception as exc:
        cloudlog.warning(f"soundd: could not load {path}: {exc}")
        return None

      if len(self.taunt_sounds) >= TAUNT_CACHE_SIZE:
        self.taunt_sounds.pop(next(iter(self.taunt_sounds)))
      self.taunt_sounds[key] = samples

    return self.taunt_sounds[key]

  def get_sound_data(self, frames): # get "frames" worth of data from the current alert sound, looping when required

    ret = np.zeros(frames, dtype=np.float32)

    if self.current_alert != AudibleAlert.none:
      num_loops = sound_list[self.current_alert][1]
      if self.current_alert == FrogPilotAudibleAlert.sissyTaunt:
        sound_data = self.taunt_current
        if sound_data is None:  # line not rendered yet — stay silent rather than crash
          return ret
      else:
        sound_data = self.loaded_sounds[self.current_alert]
      written_frames = 0

      current_sound_frame = self.current_sound_frame % len(sound_data)
      loops = self.current_sound_frame // len(sound_data)

      while written_frames < frames and (num_loops is None or loops < num_loops):
        available_frames = sound_data.shape[0] - current_sound_frame
        frames_to_write = min(available_frames, frames - written_frames)
        ret[written_frames:written_frames+frames_to_write] = sound_data[current_sound_frame:current_sound_frame+frames_to_write]
        written_frames += frames_to_write
        self.current_sound_frame += frames_to_write

    return ret * self.current_volume

  def callback(self, data_out: np.ndarray, frames: int, time, status) -> None:
    if status:
      cloudlog.warning(f"soundd stream over/underflow: {status}")
    data_out[:frames, 0] = self.get_sound_data(frames)

  def update_alert(self, new_alert):
    playing = self.taunt_current if self.current_alert == FrogPilotAudibleAlert.sissyTaunt \
              else self.loaded_sounds.get(self.current_alert)
    current_alert_played_once = self.current_alert == AudibleAlert.none or playing is None \
                                or self.current_sound_frame > len(playing)
    if self.current_alert != new_alert and (new_alert != AudibleAlert.none or current_alert_played_once):
      self.current_alert = new_alert
      self.current_sound_frame = 0

  def get_audible_alert(self, sm):
    if self.params_memory.get("TestAlert"):
      self.update_alert(getattr(AudibleAlert, self.params_memory.get("TestAlert")))
      self.params_memory.remove("TestAlert")
    elif not self.openpilot_crashed_played and self.error_log.is_file():
      self.update_alert(AudibleAlert.prompt)
      self.openpilot_crashed_played = True
    elif sm.updated['selfdriveState']:
      new_alert = sm['selfdriveState'].alertSound.raw

      # FrogPilot variables
      fpss = sm['frogpilotSelfdriveState']
      new_frogpilot_alert = fpss.alertSound.raw
      if new_alert == AudibleAlert.none and new_frogpilot_alert != FrogPilotAudibleAlert.none:
        new_alert = new_frogpilot_alert

      # Resolve the spoken line before switching to it. The text on this message is exactly what
      # is being displayed, so hashing it here needs no extra plumbing from the planner.
      if new_alert == FrogPilotAudibleAlert.sissyTaunt and new_alert != self.current_alert:
        self.taunt_current = self.load_taunt_speech(fpss.alertText1, fpss.alertText2)

      self.update_alert(new_alert)
    elif check_selfdrive_timeout_alert(sm):
      self.update_alert(AudibleAlert.warningImmediate)
      self.selfdrive_timeout_alert = True
    elif self.selfdrive_timeout_alert:
      self.update_alert(AudibleAlert.none)
      self.selfdrive_timeout_alert = False

  def calculate_volume(self, weighted_db):
    volume = ((weighted_db - AMBIENT_DB) / DB_SCALE) * (MAX_VOLUME - MIN_VOLUME) + MIN_VOLUME
    return math.pow(VOLUME_BASE, (np.clip(volume, MIN_VOLUME, MAX_VOLUME) - 1))

  @retry(attempts=10, delay=3)
  def get_stream(self, sd):
    # reload sounddevice to reinitialize portaudio
    sd._terminate()
    sd._initialize()
    return sd.OutputStream(channels=1, samplerate=SAMPLE_RATE, callback=self.callback, blocksize=SAMPLE_BUFFER)

  def soundd_thread(self):
    # sounddevice must be imported after forking processes
    import sounddevice as sd

    sm = messaging.SubMaster(['selfdriveState', 'soundPressure'])

    # FrogPilot variables
    sm = sm.extend(['frogpilotSelfdriveState', 'frogpilotPlan'])

    with self.get_stream(sd) as stream:
      rk = Ratekeeper(20)

      cloudlog.info(f"soundd stream started: {stream.samplerate=} {stream.channels=} {stream.dtype=} {stream.device=}, {stream.blocksize=}")
      while True:
        sm.update(0)

        if sm.updated['soundPressure'] and self.current_alert == AudibleAlert.none: # only update volume filter when not playing alert
          self.spl_filter_weighted.update(sm["soundPressure"].soundPressureWeightedDb)
          self.current_volume = self.calculate_volume(float(self.spl_filter_weighted.x))

          if self.frogpilot_toggles.alert_volume_controller:
            self.auto_volume = self.current_volume
            self.current_volume = 0.0

        elif self.current_alert in self.volume_map and self.frogpilot_toggles.alert_volume_controller:
          self.current_volume = self.volume_map[self.current_alert]
          if self.current_volume == 1.01:
            self.current_volume = self.auto_volume

        self.get_audible_alert(sm)

        rk.keep_time()

        assert stream.active

        # FrogPilot variables
        frogpilot_toggles = get_frogpilot_toggles(sm)
        if frogpilot_toggles != self.frogpilot_toggles:
          self.frogpilot_toggles = frogpilot_toggles

          stream = self.update_frogpilot_sounds(sd, stream)

  def update_frogpilot_sounds(self, sd=None, stream=None):
    self.volume_map = {
      AudibleAlert.engage: self.frogpilot_toggles.engage_volume / 100.0,
      AudibleAlert.disengage: self.frogpilot_toggles.disengage_volume / 100.0,
      AudibleAlert.refuse: self.frogpilot_toggles.refuse_volume / 100.0,

      AudibleAlert.prompt: self.frogpilot_toggles.prompt_volume / 100.0,
      AudibleAlert.promptRepeat: self.frogpilot_toggles.prompt_volume / 100.0,
      AudibleAlert.promptDistracted: self.frogpilot_toggles.promptDistracted_volume / 100.0,

      AudibleAlert.warningSoft: self.frogpilot_toggles.warningSoft_volume / 100.0,
      AudibleAlert.warningImmediate: self.frogpilot_toggles.warningImmediate_volume / 100.0,

      FrogPilotAudibleAlert.goat: self.frogpilot_toggles.prompt_volume / 100.0,
      FrogPilotAudibleAlert.startup: self.frogpilot_toggles.engage_volume / 100.0
    }

    for sound in sound_list:
      if sound not in self.volume_map:
        self.volume_map[sound] = 1.01

    if self.frogpilot_toggles.sound_pack != "stock":
      self.sound_directory = ACTIVE_THEME_PATH / "sounds"
    else:
      self.sound_directory = Path(BASEDIR) / "selfdrive" / "assets" / "sounds"

    if self.frogpilot_toggles.sound_pack != self.previous_sound_pack:
      self.load_sounds()

      self.previous_sound_pack = self.frogpilot_toggles.sound_pack

      if stream is not None:
        stream.close()
        stream = self.get_stream(sd)
        stream.start()

    return stream


def main():
  s = Soundd()
  s.soundd_thread()


if __name__ == "__main__":
  main()
