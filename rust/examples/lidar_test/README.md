# RPLIDAR A1 Wiring (NUCLEO-F439ZI)

| Wire | Function | Nucleo |
| --- | --- | --- |
| Red | Motor 5V | 5V |
| Orange | Core 5V | 5V |
| Black | Motor GND | GND |
| White | Core GND | GND |
| Yellow | Lidar TX | PD6 |
| Green | Lidar RX | PD5 |
| Blue | Motor PWM | PD15 (D9) |

Run from `rust/`: `cargo run --release -p lidar_test`
