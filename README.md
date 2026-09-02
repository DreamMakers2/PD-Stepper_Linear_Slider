# PD-Stepper Linear Slider Controller

Firmware for a belt-driven linear slider using the PD-Stepper V1.1, a 1.8° NEMA 17 motor, a GT2 20-tooth pulley, and the onboard AS5600 encoder. Motion and safety remain local to the ESP32-S3; the network exposes only high-level JSON commands and telemetry.

The controller intentionally has no webpage. It also does not implement compatibility aliases for the original firmware URLs.

The `sensorless-homing-12v-working-tested` milestone is hardware-tested on the assembled slider: both sensorless endpoint stops and 5 mm backoffs completed cleanly, encoder-calibrated travel was 471.93 mm, and the automatic midpoint move finished within 0.08 mm of center.

This revision has also completed a live 12 V full-cycle test with the fixed homing profile retained through the automatic center move. It measured 472.158 mm of encoder-calibrated travel, restored the persistent 32×/`FREEWHEELING` runtime profile only after centering, preserved calibration through a 32×→4×→32× change, and completed conservative moves at both resolutions. Physical button operation, center-button interruption, and a hand-moved freewheel recovery remain commissioning checks because they cannot be exercised remotely.

## Before building

1. Install the PlatformIO IDE extension in VS Code.
2. Edit [`include/wifi_config.h`](include/wifi_config.h) as described in [`WIFI_CONFIG.md`](WIFI_CONFIG.md). This file is ignored by Git.
3. Connect the PD-Stepper through its ESP32-S3 USB port.
4. Use **PlatformIO: Build**, then **PlatformIO: Upload and Monitor**.

The project pins the ESP32 Arduino toolchain and motion/driver/network dependencies in `platformio.ini`.

## Safety model

- Every boot drives TMC `ENN` high before the startup delay and starts disabled and unhomed. Position and velocity commands are rejected until homing succeeds.
- The complete homing cycle uses a fixed 12 V profile: 800 mA RMS, 4× microstepping, 1000 microsteps/s (50 mm/s), 1500 microsteps/s², StallGuard threshold 100, and TCOOLTHRS 210. Each seek is independently limited to 500 mm and 15 seconds; each backoff is limited to 2 seconds. The 5 mm backoffs use 50/75 mm units, and the automatic center move uses its validated 40 mm/s and 75 mm/s² profile. Runtime settings are restored only after centering succeeds.
- Before the first homing move, StealthChop is powered at the full 800 mA run-current scale for 150 ms so its standstill auto-tuning completes at 12 V.
- Homing then uses deterministic NORMAL standstill with 50% hold. This freezes the repository's previous/default behavior; the persisted standstill value present during the historical hardware test was not recorded. Runtime NORMAL mode instead uses 30% hold, subject to TMC register quantization.
- Detected physical endpoints are measured from the unwrapped AS5600 values. After both endpoint backoffs, the carriage moves to the calibrated midpoint. The normal commandable range keeps 5 mm away from each physical endpoint, with 0.2 mm of fault-check tolerance for encoder and microstep rounding.
- DIAG and power-good loss immediately disable EN and force-stop FastAccelStepper. The queued step position is then replaced with an encoder-derived position.
- PG is sampled directly before and after every EN activation. An enabled driver can never coexist with bad PG in the controller state; a missed edge is caught by the main-loop invariant and latches `POWER_LOSS`.
- Encoder hard-stop protection compares commanded and observed shaft movement over a rolling 125 ms window. It trips when their difference reaches 164 AS5600 counts; error is never accumulated across windows.
- Five consecutive AS5600 sample failures (about 25 ms) latch `ENCODER_FAULT` whenever the controller is homed or the driver is enabled. New move and velocity commands are rejected while the latest encoder sample is invalid.
- The TMC2209 UART and driver status are checked at startup, before configuration or enable transitions, and after a homing DIAG stop before accepting the endpoint. A CRC error or invalid all-ones status latches `TMC_COMM`; serious driver status flags latch `TMC_DRIVER`. No UART polling is performed during motion.
- Faults are latched. Fault reset leaves the motor disabled and unhomed; a new controlled home is also allowed for recoverable obstruction/homing faults.
- GPIO35 and GPIO37 provide debounced hold-to-jog toward MIN and MAX after homing. Holding GPIO36 for one second and releasing starts homing. Pressing GPIO36 during any motion immediately disables ENN, force-stops motion, cancels queued work, clears calibration, and requires a new home. It is a priority software stop, not a substitute for a hardware e-stop.
- Firmware drives GPIO10 and GPIO12 low before the startup delay and keeps both controllable LEDs off.

Do initial testing with the belt disconnected or the carriage able to move harmlessly. Never connect or disconnect the motor while powered. Confirm motor direction, encoder response, DIAG, and EN behavior before the first coupled homing run.

## API

All responses are JSON. Reads return HTTP 200; accepted actions/configuration return HTTP 202; invalid input returns 400; state conflicts return 409.

The compact technical manifest at `GET /api/help` lists every route, request shape, runtime setting, fixed homing value, limit, button action, and important operating constraint:

```sh
curl http://CONTROLLER_IP/api/help
```

### Read state

```sh
curl http://CONTROLLER_IP/api/state
```

The result includes mode, homing state, encoder and commanded position, calibrated physical and soft limits, active fault/reason, power-good, VBUS, AS5600 counts, DIAG, and the TMC status from the most recent readiness check. Existing detailed UART keys remain in the response but are not continuously tracked.

Modes are `DISABLED`, `IDLE`, `MOVING`, `HOMING_MIN`, `BACKOFF_MIN`, `HOMING_MAX`, `BACKOFF_MAX`, and `FAULTED`. Fault codes are `HOME_FAILED`, `ENCODER_HARD_STOP`, `ENCODER_FAULT`, `TMC_COMM`, `TMC_DRIVER`, `POWER_LOSS`, `TRAVEL_LIMIT`, and `UNEXPECTED_STALL`; `fault.reason` provides the specific cause.

### Home and move

```sh
curl -X POST http://CONTROLLER_IP/api/command \
  -H 'Content-Type: application/json' \
  -d '{"action":"home"}'

curl -X POST http://CONTROLLER_IP/api/command \
  -H 'Content-Type: application/json' \
  -d '{"action":"move","position_mm":120,"speed_mm_s":25,"acceleration_mm_s2":50}'

curl -X POST http://CONTROLLER_IP/api/command \
  -H 'Content-Type: application/json' \
  -d '{"action":"velocity","velocity_mm_s":-15}'
```

Position moves default to the persistent runtime profile, initially 50 mm/s and 75 mm/s². Per-command values may be supplied up to 150 mm/s and 300 mm/s². These are software ceilings, not claims that every mechanism is qualified to run at them. A velocity command is implemented as a finite position move toward the appropriate calibrated soft limit at the requested speed. It reports `MOVING` and returns normally to `IDLE` at the limit without raising `TRAVEL_LIMIT` or invalidating homing.

`home` is asynchronous. Successful completion means the controller has measured both unwrapped encoder endpoints, backed away 5 mm from each stop, set `homed: true`, moved to half the calibrated travel, and returned to `IDLE` without a fault.

### Stop, enable, disable, and reset

```sh
curl -X POST http://CONTROLLER_IP/api/command -H 'Content-Type: application/json' -d '{"action":"stop"}'
curl -X POST http://CONTROLLER_IP/api/command -H 'Content-Type: application/json' -d '{"action":"enable"}'
curl -X POST http://CONTROLLER_IP/api/command -H 'Content-Type: application/json' -d '{"action":"disable"}'
curl -X POST http://CONTROLLER_IP/api/command -H 'Content-Type: application/json' -d '{"action":"reset_fault"}'
```

`stop` preempts normal motion but retains calibration. Stopping during homing aborts and disables. `disable`, power loss, reset, configuration polarity changes, and reboot invalidate calibration. Idle microstep changes preserve calibration and rescale FastAccelStepper from a fresh encoder position.

### Driver configuration

```sh
curl http://CONTROLLER_IP/api/config

curl -X PUT http://CONTROLLER_IP/api/config \
  -H 'Content-Type: application/json' \
  -d '{
    "pd_voltage_v":12,
    "run_current_ma":800,
    "microsteps":32,
    "stallguard_threshold":20,
    "standstill_mode":"FREEWHEELING",
    "default_speed_mm_s":50,
    "default_acceleration_mm_s2":75,
    "invert_direction":false,
    "invert_encoder":false
  }'
```

Supported voltages are 5, 9, and 12 V. The default and homing voltage is 12 V; higher PD voltages are rejected. Current accepts 100–2000 mA, StallGuard accepts 0–255, and microsteps may be 1, 2, 4, 8, 16, 32, 64, 128, or 256. Public 1× selects the driver's full-step mode. Standstill modes are `NORMAL`, `FREEWHEELING`, `BRAKING`, and `STRONG_BRAKING`. Runtime configuration is persistent, but homing calibration is not. Attempts to write the read-only `homing` object return `INVALID_CONFIG`.

Fresh storage defaults to 32× microstepping and `FREEWHEELING`. Existing NVS values are deliberately preserved; configure an upgraded board once if it still contains the previous 4×/`NORMAL` defaults, then confirm the values survive a reboot.

While homed and idle, the AS5600 is the physical position authority. Manual freewheel movement updates the reported and FastAccelStepper coordinates. Before every powered move from FREEWHEELING, firmware captures/rebases, briefly uses NORMAL hold to re-engage the rotor, waits for bounded encoder stability, captures/rebases again, restores FREEWHEELING, and only then queues the command. Re-energizing can still cause a small physical alignment movement; zero movement is not guaranteed. Continuous controller power, a valid encoder, intact belt coupling, and a position inside the calibrated limits are required. The shaft encoder cannot detect belt or pulley slip.

`invert_direction` and `invert_encoder` remain the installed-orientation transforms used by homing and normal motion. Changing either invalidates the existing calibration and requires homing again.

The general StallGuard configuration remains available for the preserved driver controls, but homing always overrides it with the fixed validated threshold above.

The API accepts 100–2000 mA because that is the driver-level range used for validation; this is not a claim that every motor or an uncooled board can safely run at 2 A. Set no more than the motor rating and reduce current if the board or motor becomes hot.

### Optional session OTA maintenance

OTA is not part of the normal public control API and is absent unless the local, Git-ignored `include/wifi_config.h` defines `SLIDER_SESSION_OTA_TOKEN`. The currently tested board was built with this option active. Use a long random token, perform the first installation over USB, and keep the token out of tracked files.

An OTA upload is accepted only while the driver is disabled and the controller is `DISABLED` or `FAULTED`. It writes the normal inactive application slot and reboots after a successful upload; it does not erase flash or change the bootloader or partition layout.

```sh
curl -X POST http://CONTROLLER_IP/api/session/firmware \
  -H "X-Session-Token: $SLIDER_OTA_TOKEN" \
  -F 'firmware=@.pio/build/pd_stepper/firmware.bin;type=application/octet-stream'
```

Missing or incorrect authorization returns HTTP 403; an unsafe controller state returns HTTP 409. Remove the token definition and reinstall over USB or an already-authorized session to omit the route from future firmware.

## First commissioning

1. Power from a USB-PD supply supporting 12 V and check `/api/state`: `power.good`, encoder validity, TMC UART `ok`, and VBUS should be plausible.
2. With the mechanism uncoupled, issue a slow move only after a controlled homing test fixture or confirm direction during a guarded homing attempt. Set `invert_direction` if logical MIN is wrong.
3. Turn the motor slowly by hand while stationary/freewheeling and confirm unwrapped encoder counts and millimetres change continuously through wraparound. Use `invert_encoder` if pre-calibration movement comparison runs opposite to commanded steps.
4. Verify DIAG disables EN at the first stop and that state reports an endpoint transition or latched unexpected-stall fault.
5. Couple the belt with clear access to power removal. Home once while watching the carriage and serial/state output. Confirm measured travel is close to the expected 470 mm, soft limits are 5 mm inside both stops, and the final position is approximately half the measured travel.
6. At conservative speed, verify 4×↔32× changes preserve position and limits, then manually reposition in freewheel and issue a small move from the new position.
7. Test both jog buttons, simultaneous-button inhibition, the center priority stop, `stop`, PG power removal, TMC UART disconnection during an enable attempt, and encoder obstruction at low mechanical risk before unattended use.

## Development checks

Run the host-side conversion, encoder unwrap, synchronization, hard-stop, and soft-limit tests with:

```sh
pio test -e native
```

Build the firmware with:

```sh
pio run -e pd_stepper
```
