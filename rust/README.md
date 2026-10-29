# Rust for RailRat 

RailRat's firmware utilizes Rust Embassy on the STM32F439ZI Nucleo-144 development board.

## Directory Structure

This directory contains:
- *RAILRAT_MAIN_PROJECT/* - This is the project that RailRat will execute upon deployment.
- *examples/* - Contains various example Rust projects to implement and debug microcontroller applications.

## Contributing
**Install dependencies**
1) Install the STM32's [ST-LINK driver](https://www.st.com/en/development-tools/stsw-link009.html?utm_source=chatgpt.com) for flashing code, communication to the PC through UART3, and other functionality.
2) Install [Rust and rustup](https://rust-lang.org/).
3) Install the compilation target for the STM32F439ZI's ARM Cortex M4:

`rustup target add thumbv7em-none-eabihf`

4) Install probe-rs:

`cargo install probe-rs-tools --locked`

**Configure build environment**
1) To create a new project:

`cargo new <project_name>`

2) Configure `.cargo/config.toml` to target the STM32 (create the directory if it does not yet exist):

```
[build]
target = "thumbv7em-none-eabihf"

[target.thumbv7em-none-eabihf]
runner = "probe-rs run --chip STM32F439ZI"

[env]
DEFMT_LOG = "info"
```

3) Configure dependencies in `Cargo.toml`

**Building & loading**
1) Build the project:

`cargo build --release`

2) To flash the STM32:

`cargo run --release`
