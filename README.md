# ESP32-C3 Mini Brushed Quadcopter

> A compact DIY quadcopter project focused on embedded firmware, sensor fusion, motor PWM control, and flight stabilization.

**Status:** IMU, attitude, motor PWM, telemetry and task startup are implemented using ESP-IDF FreeRTOS. Hardware validation is pending; **not flight-ready**.

## 1. Project overview

Build a small X-layout quadcopter with an **ESP32-C3 Mini**, **MPU6050 6-axis IMU**, and **four 3.7 V Coreless 720 brushed motors**. The intended flight controller reads inertial data over I²C, estimates attitude, computes stabilization corrections, and drives four independent motor PWM channels. Current source covers sensor acquisition, roll/pitch estimation and motor PWM; PID control and motor mixing are not implemented.

The initial objective is a safe, repeatable bench prototype, followed by a restrained test rig and only then controlled flight testing.

### Goals

- Read accelerometer and gyroscope data reliably.
- Calibrate IMU offsets and estimate roll/pitch angles.
- Generate four independent PWM signals and verify motor directions.
- Implement a basic rate/attitude PID controller and X-quad motor mixer.
- Log telemetry and build a staged test procedure.
- Keep hardware abstraction separate from flight-control logic.

### Out of scope for v0.1

- Autonomous navigation, GPS, computer vision, and altitude hold.
- Unrestrained flight before validating thrust, current, battery sag, and failsafes.

## 2. Hardware

| Component                         |  Quantity | Notes                                                                                        |
| --------------------------------- | --------: | -------------------------------------------------------------------------------------------- |
| ESP32-C3 Mini development board   |         1 | 3.3 V GPIO; confirm board-specific pinout                                                    |
| MPU6050 module                    |         1 | I²C IMU; ensure 3.3 V-compatible supply and pull-ups                                        |
| Coreless 720 brushed motor, 3.7 V |         4 | Nominal 7 × 20 mm; verify actual motor current and thrust                                   |
| Matching propellers               |         4 | Two clockwise (CW) and two counterclockwise (CCW), compatible with shaft and motor direction |
| AO3400A logic-level N-MOSFET      |         4 | Preferred low-side motor switches; verify datasheet and thermal limits                       |
| Gate series resistor              |         4 | 100 Ω starting value                                                                        |
| Gate-to-GND pull-down resistor    |         4 | 100 kΩ                                                                                      |
| Flyback diode                     |         4 | Schottky diode, e.g. 1N5819 if electrically appropriate                                      |
| Bulk capacitor                    |         1 | 470–1000 µF, voltage rating ≥6.3 V, across motor VBAT/GND                                 |
| Local decoupling capacitors       | As needed | Place near power and logic modules                                                           |
| LiPo 1S battery                   |         1 | 3.0–4.2 V operating range; choose capacity/discharge rating from measured current           |
| 3.3 V regulator                   |         1 | 1S-compatible buck-boost or suitable LDO; check dropout, heat, and peak current              |
| Switch, connectors, frame, wiring | As needed | Secure battery and strain-relieve wires                                                      |

**Motor reference:** [Coreless 720 with propeller — Điện Tử Tuyết Nga](https://dientutuyetnga.com/products/dong-co-coreless-720-kem-canh-quat). Seller specifications are not a substitute for measuring current, stall current, thrust, or propeller handedness.

### Why AO3400A instead of 2N2222?

AO3400A is the preferred option for a brushed-motor power stage because its low on-resistance can reduce voltage drop and heat when correctly driven at 3.3 V. A **2N2222 alternative is experimental only**: its safe motor current and base-drive requirement must be checked against the exact transistor datasheet and measured motor starting/stall current. A 1 kΩ base resistor does **not** guarantee adequate saturation at high motor currents.

## 3. System architecture

```text
                   +-------------------+
 LiPo 1S ----------| Power distribution|---- VBAT ----+---- Motor 1
                   +-------------------+              +---- Motor 2
                             |                         +---- Motor 3
                             +--> 3.3 V regulator      +---- Motor 4
                                      |
                                 +----v-----+
                                 | ESP32-C3 |---- PWM 1..4 ---> MOSFET drivers
                                 +----+-----+
                                      |
                                    I2C
                                      |
                                 +----v-----+
                                 | MPU6050  |
                                 +----------+

 All grounds are common. Each motor has its own low-side switch and flyback diode.
```

### GPIO assignment (firmware defaults)

| Function                | ESP32-C3 GPIO | Connection               |
| ----------------------- | ------------: | ------------------------ |
| I²C SDA                |         GPIO6 | MPU6050 SDA              |
| I²C SCL                |         GPIO7 | MPU6050 SCL              |
| Motor M1 — front left  |         GPIO3 | Gate resistor → Q1 gate |
| Motor M2 — front right |         GPIO4 | Gate resistor → Q2 gate |
| Motor M3 — rear left   |         GPIO5 | Gate resistor → Q3 gate |
| Motor M4 — rear right  |        GPIO10 | Gate resistor → Q4 gate |
| 3.3 V                   |           3V3 | Regulated logic supply   |
| Ground                  |           GND | Shared reference         |

Verify that these GPIOs are actually exposed on your **specific ESP32-C3 Mini board** and that boot, USB, and programming behavior are unaffected. GPIO2, GPIO8, and GPIO9 are ESP32-C3 strapping pins and are avoided in this initial mapping.

### Power wiring

- LiPo **positive** → switch → **VBAT** rail for all four motors and input of the logic regulator.
- LiPo **negative** → common **GND** rail.
- Regulated **3.3 V** → ESP32-C3 **3V3** (only if the board permits external regulated 3.3 V input) and MPU6050 VCC (only if the module supports it).
- Never connect raw LiPo 1S directly to a 3.3 V-only pin. A conventional step-down-only buck cannot hold 3.3 V when the battery drops below its required input headroom.
- Add a bulk capacitor across VBAT and GND near the MOSFETs; keep high-current motor return paths away from the IMU ground path where practical.

### Motor driver — repeat four times

```text
                   VBAT (+)
                      |
              +-------+------------------+
              |                          |
          Motor (+)                 Diode cathode (K)
              |                          |
          Motor (-)-----------------Diode anode (A)
              |
           Drain (D)
          AO3400A
           Source (S) ---------------------- GND
           Gate (G)
              |
 GPIO PWM -- 100 Ω
              |
      [Gate node] -- 100 kΩ -- GND
```

**Interpretation:** The motor is connected between VBAT and MOSFET drain. The diode is **across the motor**, with **cathode to VBAT** and **anode to drain**. The 100 kΩ pull-down must connect from the **gate node** to GND (not in series with the gate). Confirm AO3400A SOT-23 pin numbering using the exact manufacturer's datasheet; do not infer pin order from a drawing.

> **Important:** This text schematic is conceptual; use the connection rules above when drawing a PCB schematic. Do not wire from AI-generated illustrative images without independent electrical verification.

### Optional 2N2222 experimental driver

```text
 VBAT (+) --- Motor (+)
                Motor (-) --- Collector (C)
                               2N2222
 GPIO PWM --- Rbase -------- Base (B)
                         Emitter (E) --- GND

 Flyback diode across motor:
   cathode -> VBAT (+)
   anode   -> Collector (C)
```

Choose **Rbase only after measuring motor current** and checking the transistor's forced beta, allowable collector current, base current, power dissipation, and ESP32-C3 GPIO drive limits. Do not assume the transistor can safely handle the 720 motor simply because a no-load current is low.

## 4. Quadcopter layout

```text
                      FRONT
                        ^
             M1 (FL)         M2 (FR)
                  \           /
                   \         /
                    [ ESP32 ]
                    [ MPU   ]
                   /         \
                  /           \
             M3 (RL)         M4 (RR)

                  X configuration
```

Opposing motors must use the appropriate CW/CCW propellers and rotation directions to cancel yaw torque. **Do not assume** motor wire color or seller propeller orientation; verify each motor's actual rotation and each propeller's intended thrust direction. Record the final rotation mapping in firmware configuration before any flight test.

## 5. Firmware structure and current implementation

The application uses **C++17**, **ESP-IDF 5.5.6** and the **ESP32-C3** target. Project drivers live in `driver/`; application workers and processing live in `firmware/`. ESP-IDF supplies the peripheral APIs and FreeRTOS kernel under `third_party/esp-idf/`.

### Directory layout

Generated build directories and SDK internals are omitted below. `app_control/` is currently empty; local `tests/` files are ignored by Git and are outside the firmware build.

```text
mini_quadcopter/
├── README.md
├── CMakeLists.txt                 # Build entry point from repository root
├── platformio.ini                # Basic PlatformIO board/framework configuration
├── .vscode/settings.json         # CMake source and IntelliSense settings
├── cmake/
│   └── esp_idf.cmake             # Select bundled SDK and ESP32-C3 minimal build
├── app_control/                  # Empty; control application not implemented
├── driver/
│   ├── gpio.{h,cpp}
│   ├── i2c.{h,cpp}
│   ├── imu_types.h
│   ├── imu_mpu6050.{h,cpp}
│   ├── motor_types.h
│   └── motor_pwm.{h,cpp}
├── firmware/
│   ├── CMakeLists.txt            # Build entry point used by VS Code
│   ├── sdkconfig.defaults       # FreeRTOS tick rate: 1000 Hz
│   ├── main/
│   │   ├── CMakeLists.txt        # Application sources, includes and SDK dependencies
│   │   └── app_main.cpp         # Calls quad::start_tasks()
│   ├── tasks/
│   │   ├── task_manager.{h,cpp} # Create queues/tasks using ESP-IDF FreeRTOS
│   │   ├── task_context.h       # Internal queue handles and worker declarations
│   │   ├── task_messages.h      # IMU/attitude messages and freshness checks
│   │   ├── task_mpu6050.cpp
│   │   ├── task_motor.{h,cpp}
│   │   ├── task_attitude.cpp
│   │   ├── attitude_processor.{h,cpp}
│   │   ├── task_telemetry.cpp
│   │   └── failsafe.{h,cpp}     # Empty placeholders
│   ├── attitude_estimator.{h,cpp}
│   └── pid_controller.{h,cpp}   # Empty placeholders
├── docs/
│   ├── imu-mpu6050.md
│   ├── motor-task.md
│   ├── esp32-c3_datasheet_en.pdf
│   └── esp32-c3_technical_reference_manual_en.pdf
├── scripts/
│   └── vendor_esp_idf.py         # Regenerate the bundled SDK subset
└── third_party/
    ├── README.md
    ├── esp-idf-manifest.json     # SDK revision and retained components/files
    └── esp-idf/                  # ESP32-C3 SDK source subset, v5.5.6
```

### FreeRTOS and task startup

The project uses the FreeRTOS kernel bundled at `third_party/esp-idf/components/freertos/`, including its native task, queue and notification APIs. No separate project-level `freertos/` directory is needed. The application component declares `freertos` as an ESP-IDF dependency.

`firmware/main/app_main.cpp` calls `quad::start_tasks()` from `firmware/tasks/task_manager.h`. Its implementation creates three queues of length one and four workers through `xQueueCreate()` and `xTaskCreate()`. Queue handles and worker entry declarations live in `firmware/tasks/task_context.h`.

Workers initially wait for a task notification. The manager releases them only after every queue and task has been created; allocation failure deletes blocked workers and queues and returns `ESP_ERR_NO_MEM`. Successful startup means workers were created; peripheral initialization happens inside each worker. Repeated startup returns `ESP_ERR_INVALID_STATE`.

The MPU6050 and motor tasks use priority 5, attitude priority 4 and telemetry priority 1, with a 4096-byte stack for each worker. These initial settings still require validation on hardware.

### Application tasks

Worker source files, entry functions and FreeRTOS task names follow `task_<module>`, for example `task_mpu6050.cpp`, `quad::task_mpu6050()` and `"task_mpu6050"`. New workers should follow the same convention.

| Source                                | Responsibility                                                                                       |
| ------------------------------------- | ---------------------------------------------------------------------------------------------------- |
| `firmware/tasks/task_mpu6050.cpp`   | Own the MPU6050 driver; initialize, wait 2 seconds, calibrate and publish fresh samples              |
| `firmware/tasks/task_motor.cpp`     | Own the PWM driver; initialize outputs at zero, consume commands and stop on stale/disarmed commands |
| `firmware/tasks/task_attitude.cpp`  | Consume IMU messages and publish roll/pitch estimates through `AttitudeProcessor`                  |
| `firmware/tasks/task_telemetry.cpp` | Log state, measured sample rate, errors, skipped samples and valid angles every second               |

```text
task_mpu6050 → driver/imu_mpu6050 → driver/i2c → ESP-IDF I²C → MPU6050
     └─→ [ImuFrame queue] → task_attitude → [AttitudeFrame queue] → task_telemetry

command producer → [MotorCommand queue] → task_motor → driver/motor_pwm
                                                        ├─→ driver/gpio
                                                        └─→ ESP-IDF LEDC → motors
```

Workers wait for a startup notification before using their context. Their `xQueueOverwrite()` calls require queues of length one, carrying copied messages so sampling can proceed independently of logging. Cumulative counters track sample loss and read errors. IMU data older than 100 ms is invalidated, and telemetry checks freshness again before logging angles.

Hardware access stays in `task_mpu6050` and `task_motor`. Drivers handle peripheral operations; workers handle scheduling, messages and command validity. `task_attitude` and `task_telemetry` consume copied data.

Motor commands use `MotorDuty` in M1–M4 order, with values from **0 to 1023**. Commands require `armed=true` and a timestamp less than **100 ms** old. The worker checks for commands every 10 ms and stops output when the latest command becomes inactive. Call `submit_motor_command()` after successful startup; the manager stamps and copies the latest command into the queue. Invalid duty values return `ESP_ERR_INVALID_ARG` and replace the pending command with a disarmed stop. There is currently no command producer in the boot application, so initialized outputs remain at zero. These checks do not replace a complete flight failsafe. See [motor task and PWM interface](docs/motor-task.md) for interface notes.

### Project drivers and ESP-IDF

`driver/` remains the project's hardware interface above ESP-IDF. It owns device configuration, resource lifetimes, sample conversion and peripheral error handling; the SDK supplies the underlying hardware APIs.

| Module                 | Responsibility                                                                                                       |
| ---------------------- | -------------------------------------------------------------------------------------------------------------------- |
| `driver/gpio`        | Validate GPIOs; configure inputs/pulls and initial output levels; read, write and reset pins through ESP-IDF         |
| `driver/i2c`         | Own I²C bus/device handles; probe 7-bit addresses; perform bounded write and repeated-start write/read transactions |
| `driver/imu_mpu6050` | Configure sensor registers, decode samples and calibrate through `I2cBus`/`I2cDevice`                            |
| `driver/motor_pwm`   | Configure motor pins through `Gpio`, then drive four LEDC PWM channels at 20 kHz with 10-bit duty                  |

`I2cBus` owns port 0 and validates SDA/SCL through `Gpio`. Its devices must be removed before the bus is destroyed. The MPU6050 driver currently owns its own bus and device. GPIO operations use native SDK declarations, including pull-up constants; fake peripheral implementations are outside the production drivers.

### IMU and attitude processing (`task_mpu6050`)

Implemented in source:

- Probe the configured MPU6050 address, verify WHO_AM_I, reset and verify register configuration.
- Configure **200 Hz**, **±2 g** accelerometer and **±250 °/s** gyroscope ranges.
- Read fresh accelerometer, gyro and temperature samples with bounded I²C waits. Units are g, degrees/s, °C and microsecond timestamps.
- Calibrate 400 samples by default, rejecting motion or an incorrect pose; retain successful biases in RAM.
- Estimate roll/pitch in sensor axes using a complementary filter and actual sample timing.
- Invalidate estimates on faults, stale data or unsuitable timing; report status and cumulative counters through telemetry.

Default wiring is **SDA GPIO6**, **SCL GPIO7**, address **0x68** (`0x69` with AD0 high). Keep the IMU level and motionless, **+Z upward**, during calibration. Hardware sample rate, axis orientation and stationary stability remain to be verified. See [MPU6050 setup and bench validation](docs/imu-mpu6050.md).

### Bundled SDK

`third_party/esp-idf/` is a reduced source snapshot stored as regular repository files, rather than a Git submodule. It retains 32 components required by the application and bootloader, including GPIO, I²C, LEDC, FreeRTOS and their dependencies. Examples, SDK documentation/tests, unused components and other chip support are omitted.

Both CMake entry points load this SDK through `cmake/esp_idf.cmake`, select ESP32-C3 and enable `MINIMAL_BUILD`. Compiler binaries and Python packages are installed separately. New peripherals, network features or SDK configuration changes may require updating the manifest and regenerating the subset. See [SDK selection and regeneration](third_party/README.md).

### Build and flash

From the repository root, install ESP32-C3 tools once:

```sh
./third_party/esp-idf/install.sh esp32c3
```

Then activate the bundled SDK in each shell and build from `firmware/`:

```sh
. ./third_party/esp-idf/export.sh
cd firmware
idf.py set-target esp32c3
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

Replace the serial port with the board's actual port. ESP-IDF installs tools and the Python environment under `~/.espressif`. Generated configuration and build outputs are `firmware/sdkconfig` and `firmware/build/`, both ignored by Git. The root `CMakeLists.txt` is an alternative entry point for the same application; the commands above match the VS Code CMake source directory.

`platformio.ini` currently contains basic board/framework settings and serial speeds. It does not configure the bundled SDK path; the native ESP-IDF commands above are the documented build workflow. VS Code reads `firmware/build/compile_commands.json`; adjust its compiler path if the toolchain is installed somewhere other than the configured PlatformIO directory.

### Remaining flight-control work

`pid_controller.{h,cpp}` and `tasks/failsafe.{h,cpp}` are empty placeholders. A motor mixer, pilot command transport, integrated arming policy and full flight-control loop are not implemented.

The intended pipeline after bringing up the existing workers is:

1. Receive pilot commands through a transport/protocol still to be selected.
2. Compute rate/attitude PID corrections from sensor estimates.
3. Mix throttle and roll/pitch/yaw corrections into M1–M4 commands.
4. Apply arming, limits and failsafe decisions, then submit refreshed motor commands.
5. Monitor timing, sensor health and motor output on a restrained rig.

The driver's current 20 kHz PWM setting still requires hardware checks for motor response, switch temperature, current and noise.

## 6. Development roadmap

| Phase                        | Deliverable                              | Acceptance criteria                                                     |
| ---------------------------- | ---------------------------------------- | ----------------------------------------------------------------------- |
| 0 — Hardware validation     | Confirm parts and electrical limits      | Motor starting/stall current measured; supply and switches sized safely |
| 1 — Board bring-up          | ESP32-C3 toolchain, UART logs, I²C scan | Reliable flashing, logs, MPU6050 detected                               |
| 2 — IMU                     | Calibration and filtered roll/pitch      | Stable stationary measurements and documented sample rate               |
| 3 — Motor drivers           | Four PWM channels, propellers removed    | Each channel responds correctly; no excessive MOSFET heat or brownouts  |
| 4 — Flight controller       | PID + X mixer + arming logic             | Mixer signs verified in software and restrained tests                   |
| 5 — Ground tests            | Battery and thrust measurements          | Sufficient thrust margin, current margin, and effective emergency stop  |
| 6 — Restrained flight tests | Controlled hover evaluation              | Only after all previous checks pass; logs reviewed                      |

## 7. Hardware validation checklist

- [ ] Verify motor voltage, no-load current, loaded current, and **stall current**.
- [ ] Check MOSFET gate/source/drain pinout and diode polarity with a multimeter.
- [ ] Verify 3.3 V regulator across the intended battery voltage range.
- [ ] Confirm shared ground, no shorts, and acceptable voltage sag.
- [ ] Flash ESP32-C3 and check I²C communication with MPU6050.
- [ ] Validate IMU axis orientation and calibration.
- [ ] Test each motor individually **without propellers**.
- [ ] Verify PWM-to-motor mapping M1–M4.
- [ ] Verify motor spin direction and propeller handedness.
- [ ] Implement disarmed-on-boot, signal-loss timeout, and emergency motor cutoff.
- [ ] Measure total takeoff mass and per-motor thrust using a safe test fixture.
- [ ] Verify motor and MOSFET temperatures after representative runs.
- [ ] Confirm controller response and mixer signs on a restrained rig.
- [ ] Perform first flight only in a safe, open, controlled environment.

## 8. Safety

**This is an experimental aircraft, not a flight-certified design.**

- Remove propellers for firmware flashing, PWM checks, and most bench tests.
- Use a current-limited bench supply for early power-stage testing where practical.
- A 1S LiPo is **4.2 V fully charged**; size components for this voltage, not only 3.7 V nominal.
- Do not over-discharge, puncture, short, or charge LiPo batteries unattended; use a suitable 1S charger and low-voltage protection strategy.
- Do not power motors from the ESP32-C3 3V3 pin.
- Implement a physical battery disconnect and firmware arming/failsafe logic before flight.
- Confirm the motor/propeller combination produces enough thrust for the actual all-up mass; physical size alone does not prove flyability.

## 9. Open decisions

- [ ] Confirm exact ESP32-C3 Mini board and its external 3.3 V supply requirements.
- [ ] Select battery capacity and discharge rating after motor current measurements.
- [ ] Confirm 720 motor propeller CW/CCW pairs and measured thrust.
- [ ] Choose control link (Wi-Fi/BLE/other), protocol, and failsafe timeout.
- [ ] Choose IMU mounting orientation and vibration isolation.
- [ ] Decide whether to add a battery voltage sensing divider/ADC.
- [ ] Finalize PCB/wiring schematic and validate it electrically.

## 10. References

- [ESP32-C3 documentation — Espressif](https://docs.espressif.com/projects/esp-idf/en/stable/esp32c3/)
- [MPU-6050 product specification — TDK InvenSense](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [Coreless 720 with propeller — Điện Tử Tuyết Nga](https://dientutuyetnga.com/products/dong-co-coreless-720-kem-canh-quat)

---

**License:** TBD
**Maintainer:** TBD
**Version:** v0.1 — bench firmware in development
