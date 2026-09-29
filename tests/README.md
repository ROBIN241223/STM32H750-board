# Flight controller host checks

Two suites cover the hardware-independent flight stack:

- `test_fc_math.c` covers the estimator, attitude and rate controllers, mixer, position
  loop, land detector, guard, scheduler, topic bus, param server, mission navigator, and
  the ICM-42688 driver on a fake SPI bus. The estimator cases include the sustained-turn
  regression and the online gyro-bias and quiescence behaviour described in
  PROJECT_LOG.md; `test_estimator_turn_holds_gyro` is built from a recorded flight and is
  the one to read first if attitude ever disagrees with ground truth again.
- `test_fc_nav.c` covers the navigation sensors: GPS quality gating, home-relative NED,
  hard/soft-iron compass calibration, the NMEA and UBX parsers, and the parser feeding
  the GPS module.

Neither accesses STM32 HAL, FreeRTOS, real SPI or motor PWM, so both build and run on the
host with a plain `gcc`.

## Flight stack

Build and run from the project root:

```bash
gcc -std=c11 -Wall -Wextra -ICore/Inc \
  tests/test_fc_math.c \
  Core/Src/fc_*.c \
  Core/Src/imu_icm42688.c \
  Core/Src/imu_port_spi.c \
  -lm -o /tmp/test_fc_math
/tmp/test_fc_math
```

`Core/Src/fc_*.c` covers every `fc_*` module at once, so the command does not need to be
edited when a new module is added. `Core/Src/imu_port_null.c` is deliberately excluded:
the IMU tests drive `imu_port_spi.c` through a fake bus instead.

## Navigation sensors

```bash
gcc -std=c11 -Wall -Wextra -ICore/Inc \
  tests/test_fc_nav.c \
  Core/Src/fc_gps.c \
  Core/Src/fc_compass.c \
  Core/Src/gps_parser.c \
  Core/Src/gps_port_null.c \
  -lm -o /tmp/test_fc_nav
/tmp/test_fc_nav
```

This suite lists its sources explicitly rather than using the `fc_*` glob. `gps_parser.c`
is not an `fc_*` module, and `gps_port_uart.c` is excluded because it pulls in the HAL;
the UART framing is covered through `gps_port_null.c` instead.

Each run ends with `ALL TESTS PASSED` / `ALL NAV TESTS PASSED`. Most checks use
`assert()`, so a healthy run prints only a few progress lines; the exit status and the
final line are the signal.

Current status: both build clean with no warnings under `-Wall -Wextra` and pass. The
firmware itself is a separate build (`make -j$(npcpu)`) and must be rerun after changing
any `Core/Src` file.
