# mcu_ws – ESP32 firmware

Requirements → tests → code. Start with [docs/REQUIREMENTS.md](docs/REQUIREMENTS.md).

```
docs/REQUIREMENTS.md   what the firmware must do (IDs like MOT-3, SAF-2)
test/                  host unit tests (g++ only, no board needed)
bronco_mcu/            the Arduino sketch (open bronco_mcu.ino)
  pins.h               ALL GPIO numbers  <- edit when you have the schematic
  tuning.h             wheelbase, limits, calibration, inversion flags
  *.h                  hardware-independent logic (tested on the PC)
  arduino_hal.h        real ESP32 pin/PWM/ADC code
  bronco_mcu.ino       micro-ROS glue
```

## Run the tests

```bash
make -C mcu_ws/test          # runs all tests + checks every requirement has a test
```

Each test is tagged with the requirement IDs it covers, e.g.
`TEST(deadband_mapping, "MOT-3")`. To add a feature: add rows to
REQUIREMENTS.md, add tagged tests, watch `make` fail, then write the code.

## Upload (Arduino IDE)

1. Install the ESP32 board package (Espressif, version 3.x) and the
   `micro_ros_arduino` library (release matching your ROS 2 distro).
2. Open `bronco_mcu/bronco_mcu.ino`, pick your ESP32 board, upload.
3. On the Pi: run the micro-ROS agent on the ESP32's serial port, e.g.
   `ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0`.

The `.ino` and `arduino_hal.h` were not compiled here (no board/Arduino
toolchain on this machine). Expect to fix small build issues on the first
upload, especially around micro_ros_arduino's ESP32 support for your ROS distro.
