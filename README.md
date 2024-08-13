# Motor String Cat Toy

## Contributing

This project is built in a `no_std` environment utilizing the `esp-hal` crate.

### Requirements

### Hardware

- [ESP32-WROOM-32](https://www.espressif.com/sites/default/files/documentation/esp32-wroom-32_datasheet_en.pdf) (Xtensa architecture)
  ![ESP32-WROOM-32 Pinout](/datasheets/ESP32_WROOM_Development_board_pinout.png)
- [28BYJ48 Stepper motor](/datasheets/28BYJ-48.pdf)
- [ULN2003 Stepper motor driver](/datasheets/ULN2003_PCB_stepper_motor_driver.pdf)

### Development Environment

- Rust via [rustup](https://rustup.rs/)
- Install [ESP32 Rust tooling](https://docs.esp-rs.org/book/installation/index.html)

```shell
cargo install espup
espup install
```

- Source environment variables in each shell or in shell profile:

```shell
. $HOME/export-esp.sh
```
