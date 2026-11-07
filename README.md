# Industrial Paint Mixing Controller

An ESP32 based tabletop process controller built for the Lam Research Challenge 2026 Stage 3 with T-Works. Our team built a teal paint mixing setup using separate blue, green, yellow, and flush lines. The project combines pumps, solenoid valves, flow sensors, two temperature probes, and a browser dashboard.

The repository contains both final controller sketches: U2 for the web dashboard and actuators, and U7 for flow and temperature telemetry. The two U2 uploads supplied for this repository were identical, so only one copy is included.

## What it does

- Drives four pumps with PWM and controls the corresponding solenoid valves from ESP32 #1 (U2).
- Hosts a local Wi-Fi dashboard on U2 to start and stop the process, monitor status, and test the flush valve.
- Reads three flow sensor counters and two DS18B20 temperatures on ESP32 #2 (U7); the final U2 firmware expects U7 to POST these readings to `/api/flow`.
- Runs a flush stage, checks the temperature difference between two tanks, then doses the three colours. During dosing, U2 converts flow pulses into estimated volumes and stops each pump and closes its valve at the configured target.

## Hardware architecture

| Component | Role |
| --- | --- |
| ESP32 #1 (U2) | Actuator control, sequence logic, and Wi-Fi dashboard |
| ESP32 #2 (U7) | Flow and temperature acquisition |
| Four pumps and motor drivers | Blue, green, yellow, and flush lines |
| Solenoid valves | Route and isolate the fluid lines |
| FS1, FS2, FS3 | Blue/flush shared line, yellow line, and green line |
| TS1, TS2 | Tank temperature measurements |

In the final U2 sketch, the pump PWM outputs are GPIO 26 (blue), 33 (green), 32 (yellow), and 25 (flush). Solenoid signals are GPIO 19 (blue/SV4), 21 (green/SV3), 23 (yellow/SV1), 22 (flush/SV2), and 18 (SV5). U7 reads FS1 on GPIO 33, FS2 on GPIO 25, and FS3 on GPIO 26. These GPIO numbers belong to **different ESP32 boards**.

## Control sequence

1. Flush water through the selected route into a separate jug. The final U2 sketch uses a calibrated **time interval** for this stage because its flush path has no dedicated flow sensor.
2. Monitor TS1 and TS2 until the configured temperature difference is reached.
3. Start the colour pumps and track each line separately with flow pulses. Stop each pump and close its valve at its target.
4. De-energize the pumps and close the valves when dosing completes.

The firmware has a manual temperature-pass control and a manual flush-valve override for testing. A forced pass does not represent a measured temperature threshold.

## Calibration and testing

We collected known fluid volumes and compared them with flow sensor pulses to tune each line's pulses-per-litre factor. During testing, the team also investigated false counts on inactive sensor lines and adjusted the dosing sequence and calibration. Flow-based volume is an **estimate**; the physical output should be checked with a measured container before relying on a new calibration.

## Firmware

- [U2 controller sketch](firmware/U2_Teal_Controller/U2_Teal_Controller.ino) — select an ESP32 Dev Module with ESP32 Arduino core 3.x.
- [U7 sensor sketch](firmware/U7_Sensor_Node/U7_Sensor_Node.ino) — requires the OneWire and DallasTemperature Arduino libraries.
- Before flashing, choose a new password in U2's `WiFi.softAP(...)` and use **exactly the same password** in U7's `WIFI_PASSWORD`. The public repository uses a placeholder instead of the local demo password.
- The controller dashboard is available at `http://192.168.4.1` when connected to its access point.
- U7 sends form fields `boot`, `b`, `g`, `y`, and, when valid, `ty` and `tg` to U2's `POST /api/flow` endpoint.

## Implementation notes

The latest U2 controller uses an HTTP `/api/flow` endpoint for U7 telemetry. An older U7 UART sketch that prints `$FLOW,...` packets is **not compatible** with this U2 build.

The available U2 sketch includes a timed flush, manual overrides, and hard-coded calibration factors. This README describes the implementation and does not claim that every challenge interlock (empty source tank, full mixing tank, or no-flow trip) was validated.

