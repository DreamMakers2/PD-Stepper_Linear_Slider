# PD-Stepper Linear Slider Controller

Firmware for a belt-driven linear slider using the PD-Stepper V1.1, a 1.8° NEMA 17 motor, a GT2 20-tooth pulley, and the onboard AS5600 encoder. Motion and safety remain local to the ESP32-S3; the network exposes only high-level JSON commands and telemetry.

The controller intentionally has no webpage. It also does not implement compatibility aliases for the original firmware URLs.

## Before building

1. Install the PlatformIO IDE extension in VS Code.
2. Edit [`include/wifi_config.h`](include/wifi_config.h) as described in [`WIFI_CONFIG.md`](WIFI_CONFIG.md). This file is ignored by Git.
3. Connect the PD-Stepper through its ESP32-S3 USB port.
4. Use **PlatformIO: Build**, then **PlatformIO: Upload and Monitor**.

The project pins the ESP32 Arduino toolchain and motion/driver/network dependencies in `platformio.ini`.

## Safety model

- Every boot drives TMC `ENN` high before the startup delay and starts disabled and unhomed. Position and velocity commands are rejected until homing succeeds.
- Homing uses the validated fixed 12 V profile: 800 mA RMS, 4x microstepping, 1000 microsteps/s (50 mm/s), 1500 microsteps/s², StallGuard threshold 100, and TCOOLTHRS 210. Each seek is independently limited to 500 mm and 15 seconds.
- Before the first homing move, StealthChop is powered at the full 800 mA run-current scale for 150 ms so its standstill auto-tuning completes at 12 V.
- Detected physical endpoints are measured from the unwrapped AS5600 values. After both endpoint backoffs, the carriage moves to the calibrated midpoint. The normal commandable range keeps 5 mm away from each physical endpoint, with 0.2 mm of fault-check tolerance for encoder and microstep rounding.
- DIAG and power-good loss immediately disable EN and force-stop FastAccelStepper. The queued step position is then replaced with an encoder-derived position.
- PG is sampled directly before and after every EN activation. An enabled driver can never coexist with bad PG in the controller state; a missed edge is caught by the main-loop invariant and latches `POWER_LOSS`.
- Encoder hard-stop protection compares commanded and observed shaft movement over a rolling 125 ms window. It trips when their difference reaches 164 AS5600 counts; error is never accumulated across windows.
- Five consecutive AS5600 sample failures (about 25 ms) latch `ENCODER_FAULT` whenever the controller is homed or the driver is enabled. New move and velocity commands are rejected while the latest encoder sample is invalid.
- The TMC2209 UART and driver status are checked at startup, before configuration or enable transitions, and after a homing DIAG stop before accepting the endpoint. A CRC error or invalid all-ones status latches `TMC_COMM`; serious driver status flags latch `TMC_DRIVER`. No UART polling is performed during motion.
- Faults are latched. Fault reset leaves the motor disabled and unhomed; a new controlled home is also allowed for recoverable obstruction/homing faults.

Do initial testing with the belt disconnected or the carriage able to move harmlessly. Never connect or disconnect the motor while powered. Confirm motor direction, encoder response, DIAG, and EN behavior before the first coupled homing run.

## API

All responses are JSON. Reads return HTTP 200; accepted actions/configuration return HTTP 202; invalid input returns 400; state conflicts return 409.

### Read state

```sh
curl http://CONTROLLER_IP/api/state
```

The result includes mode, homing state, encoder and commanded position, calibrated physical and soft limits, active fault/reason, power-good, VBUS, AS5600 counts, DIAG, and the TMC status from the most recent readiness check. Existing detailed UART keys remain in the response but are not continuously tracked.

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

Position move speed and acceleration default to 40 mm/s and 75 mm/s² and may be reduced per command. A velocity command is implemented as a finite position move toward the appropriate calibrated soft limit at the requested speed. It reports `MOVING` and returns normally to `IDLE` at the limit without raising `TRAVEL_LIMIT` or invalidating homing.

### Stop, enable, disable, and reset

```sh
curl -X POST http://CONTROLLER_IP/api/command -H 'Content-Type: application/json' -d '{"action":"stop"}'
curl -X POST http://CONTROLLER_IP/api/command -H 'Content-Type: application/json' -d '{"action":"enable"}'
curl -X POST http://CONTROLLER_IP/api/command -H 'Content-Type: application/json' -d '{"action":"disable"}'
curl -X POST http://CONTROLLER_IP/api/command -H 'Content-Type: application/json' -d '{"action":"reset_fault"}'
```

`stop` preempts normal motion but retains calibration. Stopping during homing aborts and disables. `disable`, power loss, reset, configuration polarity/microstep changes, and reboot invalidate calibration.

### Driver configuration

```sh
curl http://CONTROLLER_IP/api/config

curl -X PUT http://CONTROLLER_IP/api/config \
  -H 'Content-Type: application/json' \
  -d '{
    "pd_voltage_v":12,
    "run_current_ma":800,
    "microsteps":4,
    "stallguard_threshold":20,
    "standstill_mode":"NORMAL",
    "invert_direction":false,
    "invert_encoder":false
  }'
```

Supported voltages are 5, 9, and 12 V. The default and homing voltage is 12 V; higher PD voltages are rejected. Microsteps are 1, 2, 4, 8, 16, 32, 64, 128, or 256. Standstill modes are `NORMAL`, `FREEWHEELING`, `BRAKING`, and `STRONG_BRAKING`. Configuration is persistent, but homing/calibration is not.

The general StallGuard configuration remains available for the preserved driver controls, but homing always overrides it with the fixed validated threshold above.

The API accepts 100–2000 mA because that is the driver-level range used for validation; this is not a claim that every motor or an uncooled board can safely run at 2 A. Set no more than the motor rating and reduce current if the board or motor becomes hot.

## First commissioning

1. Power from a USB-PD supply supporting 12 V and check `/api/state`: `power.good`, encoder validity, TMC UART `ok`, and VBUS should be plausible.
2. With the mechanism uncoupled, issue a slow move only after a controlled homing test fixture or confirm direction during a guarded homing attempt. Set `invert_direction` if logical MIN is wrong.
3. Turn the motor slowly by hand while disabled and confirm unwrapped encoder counts change continuously through wraparound. Use `invert_encoder` if pre-calibration movement comparison runs opposite to commanded steps.
4. Verify DIAG disables EN at the first stop and that state reports an endpoint transition or latched unexpected-stall fault.
5. Couple the belt with clear access to power removal. Home once while watching the carriage and serial/state output. Confirm measured travel is close to the expected 470 mm and soft limits are 5 mm inside both stops.
6. Test `stop`, PG power removal, TMC UART disconnection during an enable attempt, and encoder obstruction at low mechanical risk before unattended use.

## Development checks

Run the host-side conversion, encoder unwrap, synchronization, hard-stop, and soft-limit tests with:

```sh
pio test -e native
```

Build the firmware with:

```sh
pio run -e pd_stepper
```
