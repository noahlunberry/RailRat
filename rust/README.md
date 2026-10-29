# Rust for RailRat 

RailRat's firmware utilizes Rust Embassy on the STM32F439ZI Nucleo-144 development board.

## Directory Structure

This directory contains:
- *RAILRAT_MAIN_PROJECT/* - This is the project that RailRat will execute upon deployment.
- *examples/* - Contains various example Rust projects to implement and debug microcontroller applications.

## Prerequisites


## Building & Loading
1) The following command installs dependencies and builds the source code.

   `cargo build --release`
   
2) Flash the board.
   
   `cargo run --release`
