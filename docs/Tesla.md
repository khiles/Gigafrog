# Tesla Support in FrogPilot

This page covers everything specific to Tesla vehicles in this fork — which cars are supported, what hardware you need, and what features are working.

---

## Supported Vehicles

| Model | Years | Hardware | Harness | Longitudinal |
|-------|-------|----------|---------|--------------|
| Model 3 | 2019–23 | HW3 | Tesla A | Optional (alpha) |
| Model 3 | 2024–25 | HW4 | Tesla B | Optional (alpha) |
| Model Y | 2020–23 | HW3 | Tesla A | Optional (alpha) |
| Model Y | 2024–25 | HW4 | Tesla B | Optional (alpha) |
| Model S | 2020–23 | HW3 | Model S/X HW3 | Always on |
| Model S | 2024 | HW4 | Tesla B | Optional (alpha) |
| Model S | 2014–16 | HW1 | Model S HW1 | Always on |
| Model S | 2017–19 | HW2 | Model S/X HW2 | Always on |
| Model X | 2024 | HW4 | Tesla B | Optional (alpha) |
| Model X | 2014–16 | HW1 | Model S/X HW1 | Always on |
| Model X | 2016–19 | HW2 | Model S/X HW2 | Always on |

**To find your hardware version:** on your car's touchscreen go to *Software → Additional Vehicle Information* and look for **Autopilot computer**.

> Some 2023 Model 3/Y were shipped with HW4. See [this page](https://www.notateslaapp.com/news/2173/how-to-check-if-your-tesla-has-hardware-4-ai4-or-hardware-3) for how to tell.

---

## What Works

### Lateral Control (Steering)
openpilot takes over steering using angle-based control on all supported vehicles. The steering angle request is sent directly to the EPAS (electric power steering) module.

**Smooth hand-off when you steer:** If you put your hands on the wheel, openpilot doesn't abruptly cut out. Instead it gradually reduces its steering authority as you apply more force (none at light touch, 50% at medium pressure, fully off at heavy pressure), then smoothly resumes control when you let go. The car blends back to openpilot's desired angle rather than snapping back.

### Longitudinal Control (Speed)
openpilot sends acceleration and deceleration commands directly to Tesla's ACC system over CAN.

- **Model 3/Y/S HW4/Model X HW4:** Longitudinal is optional and marked alpha. Enable it in settings.
- **Model S/X HW1/HW2/HW3:** Longitudinal is always on and cannot be disabled.

### Turn Indicators During Lane Changes
When openpilot performs a lane change, it keeps the turn indicator on for the full duration of the maneuver. Tesla's stock system cancels the blinker after three flashes, which can turn it off mid-lane-change. This fix sends a turn indicator CAN message every 200ms to override that behaviour.

### Blindspot Monitoring

**Model 3/Y (HW3/HW4):** openpilot reads the blindspot status directly from Tesla's Autopilot module (`DAS_blindSpotRearLeft` / `DAS_blindSpotRearRight`). If a vehicle is detected in a blindspot, openpilot will not initiate a lane change.

**Model S HW3:** Same signals are available from the `AutopilotStatus` message on the chassis CAN bus and are now read correctly.

**All legacy vehicles (HW1/HW2/HW3):** A software-based blindspot monitor supplements the hardware signals. It tracks radar targets in the zone alongside and slightly behind your car, so lane-change blocking works even if the hardware signal is unavailable.

### Speed Limit Controller
openpilot reads the speed limit Tesla's own system detects and can automatically adjust your set speed.

- The speed limit comes from two sources on the CAN bus:
  - **Camera-detected (vision-only):** What the camera sees right now. Reacts immediately to temporary signs like construction zones or variable speed limits on motorways.
  - **Camera + map fused:** A combination of what the camera sees and what the map expects. More stable on long straight roads.
- openpilot **prefers the camera-detected value** when it is valid, and falls back to the fused value otherwise. This means a 40mph construction zone sign is picked up straight away, even if the map still says 70mph.
- The speed limit is shown on your device's dashboard display as usual.

### Gap Adjust (Follow Distance)
**Model S/X HW3 and older legacy vehicles:** The follow distance setting from the steering wheel scroll wheel is now read from the CAN bus (`STW_ACTN_RQ.DTR_Dist_Rq`). When you adjust the gap using the scroll wheel, FrogPilot's follow distance setting updates to match.

### Autopark Detection
**Model S HW3:** When Tesla's built-in autopark feature is offered or active (`DAS_autoparkReady`), openpilot automatically disables its cruise control to avoid fighting with the parking manoeuvre. It re-enables automatically once autopark is finished.

### Standstill Detection
**Model S/X HW3 and older legacy vehicles:** Vehicle standstill is now detected using `DI_vehicleHoldState` directly from the drive inverter module. Previously this relied on ACC being in a standstill state, which meant it only worked while cruise was engaged. The new signal works at all times.

---

## Known Limitations

- **Model X HW4 (2024):** Currently dashcam-only while we find the correct signal to confirm stock autosteer is off before engaging.
- **Model 3/Y with FSD 14:** Slightly different CAN encoding for the steering control type — handled automatically.
- **HW2 Model S/X:** Radar unavailable (the radar is on a separate bus that is not currently parsed).
- **HUD control:** openpilot does not yet write to Tesla's instrument cluster display. The set speed and engagement indicator are only shown on the comma device.

---

## Hardware Setup

Tesla vehicles require a special harness that taps into the Autopilot CAN bus. The harness you need depends on your model:

- **Tesla Harness A** — Model 3 / Model Y with HW3
- **Tesla Harness B** — Model 3 / Model Y / Model S / Model X with HW4
- **Model S HW1 Harness** — Model S with HW1 (2014–16)
- **Model S/X HW2 Harness** — Model S/X with HW2 (2016–19)
- **Model S/X HW3 Harness** — Model S/X with HW3 (2020–23)
- **Model X HW1 Harness** — Model X with HW1 (2014–16)

For full setup instructions see the [Tesla setup wiki page](https://github.com/commaai/openpilot/wiki/tesla).

---

## Technical Notes

### CAN Bus Layout (Model S HW3)
The HW3 Model S has an unusual multi-bus layout. openpilot connects to several buses simultaneously:

| Bus | Contents |
|-----|----------|
| 0 (party) | EPAS steering commands, APS monitor |
| 1 (chassis) | Vehicle speed, gear, doors, blindspot, speed limit, autopark |
| 2 (autopilot party) | AP steering control, AEB status |
| 4 (powertrain) | Acceleration commands, pedal position |
| 6 (autopilot powertrain) | DAS control messages |

### Speed Limit Signal Priority
```
DAS_visionOnlySpeedLimit (valid?) → use it
        ↓ no
DAS_fusedSpeedLimit (valid?) → use it
        ↓ no
No speed limit shown
```
Both signals are encoded the same way: `raw_value × 5 = speed in mph or kph`. Values 0 (unknown) and 31 (none) are treated as invalid.

### Steering Override Behaviour
openpilot maps `EPAS_handsOnLevel` (0–3) to steering authority:

| Hands-on level | openpilot authority |
|----------------|---------------------|
| 0–1 (no/light contact) | 100% |
| 2 (medium pressure) | 50% |
| 3+ (heavy pressure) | 0% (disabled) |

When you release the wheel, steering blends back smoothly to openpilot's desired angle over one second rather than snapping back instantly.
