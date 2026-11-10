# Rust for RailRat 

RailRat's firmware utilizes Rust Embassy on the STM32F439ZI Nucleo-144 development board.

## Directory Structure

This directory contains:
- `RAILRAT_MAIN_PROJECT/` - The project that RailRat will execute upon deployment.
- `examples/` - Contains various example Rust projects to implement and debug microcontroller applications.

## Getting Started
### 1. Install dependencies
1) Set up USB connection for the ST-LINK:

    - **WINDOWS** - Install the STM32's [ST-LINK USB driver](https://www.st.com/en/development-tools/stsw-link009.html?utm_source=chatgpt.com) for flashing code, communication to the PC through UART3, and other functionality.

    - **LINUX** - The ST-LINK is already Linux-compatible. If root/admin privileges are not available, you will need to grant USB permissions through [udev](https://probe.rs/docs/getting-started/probe-setup/).

    - **MACOS** - The ST-LINK is already compatible. No further setup is needed to use it.

2) Install [Rust and rustup](https://rust-lang.org/).
3) Install and add the compilation target for the STM32F439ZI's ARM Cortex M4:

```bash
rustup default stable
rustup target add thumbv7em-none-eabihf # Install target core
cargo install probe-rs-tools --locked   # Install probe-rs
```

### 2. Building & loading
From the `rust/` directory:
```bash
cargo build --release # Compiles your firmware
cargo run --release   # Flashes the STM32
```

### OPTIONAL: Creating an example project
1) Create a new project:

```bash
cargo new examples/<PROJECT_NAME>
```

2) Copy the contents of `RAILRAT_MAIN_PROJECT/Cargo.toml` into the example project's `Cargo.toml`. Change the project name appropriately.

3) Copy any necessary source code from `RAILRAT_MAIN_PROJECT/src/` into the example project's `src/` directory.

4) To build and flash from the `rust/` directory:

```bash
cargo build -p <PROJECT_NAME> --release
cargo run -p <PROJECT_NAME> --release
```


## Contributing

The `main` branch contains the latest stable version of the project. **Do not commit or push directly to `main`.**

All development should take place on separate branches.

Use the following branch naming conventions:

| Prefix | Purpose | Example |
|---|---|---|
| `feature/` | New functionality | `feature/motor-control` |
| `fix/` | Bug fixes | `fix/uart-timeout` |
| `refactor/` | Code restructuring | `refactor/sensor-drivers` |
| `docs/` | Documentation changes | `docs/setup-guide` |
| `test/` | Testing and validation | `test/gps-driver` |


### 1. Create a new branch
Before starting work, ensure your local `main` branch is up to date.

```bash
git switch main
git pull origin main
```

Create and switch to a new branch:

```bash
git switch -c feature/your-feature-name
```

### 2. Making Changes

Make changes on your branch and commit them regularly.

```bash
git add .
git commit -m "Example message"
```

    - **WINDOWS** - Install the STM32's [ST-LINK USB driver](https://www.st.com/en/development-tools/stsw-link009.html?utm_source=chatgpt.com) for flashing code, communication to the PC through UART3, and other functionality.

    - **LINUX** - The ST-LINK is already Linux-compatible. If root/admin privileges are not available, you will need to grant USB permissions through [udev](https://probe.rs/docs/getting-started/probe-setup/).

    - **MACOS** - The ST-LINK is already compatible. No further setup is needed to use it.

2) Install [Rust and rustup](https://rust-lang.org/).
3) Install and add the compilation target for the STM32F439ZI's ARM Cortex M4:

```bash
rustup default stable
rustup target add thumbv7em-none-eabihf # Install target core
cargo install probe-rs-tools --locked   # Install probe-rs
```

### 2. Building & loading
```bash
cargo build --release # Compiles your firmware
cargo run --release   # Flashes the STM32
```

### OPTIONAL: Creating a new example project
1) Create a new project:
```bash
cargo new <project_name>
```

2) Configure `.cargo/config.toml` to target the STM32 (create the directory if it does not yet exist):

```toml
[build]
target = "thumbv7em-none-eabihf"

Push the new branch:

```bash
git branch # Your branch is the one with *
git push -u origin feature/your-feature-name
```

### 3. Keeping Your Branch Updated

While developing, periodically incorporate changes from `main` to reduce merge conflicts.

```bash
git fetch origin
git merge origin/main
```

Resolve any merge conflicts, if necessary, before continuing development.

[env]
DEFMT_LOG = "info"  # Can be changed
```

3) Configure dependencies in `Cargo.toml`. Examples of this file can be found in the `examples/` directory.


## Contributing

The `main` branch contains the latest stable version of the project. **Do not commit or push directly to `main`.**

All development should take place on separate branches.

Use the following branch naming conventions:

| Prefix | Purpose | Example |
|---|---|---|
| `feature/` | New functionality | `feature/motor-control` |
| `fix/` | Bug fixes | `fix/uart-timeout` |
| `refactor/` | Code restructuring | `refactor/sensor-drivers` |
| `docs/` | Documentation changes | `docs/setup-guide` |
| `test/` | Testing and validation | `test/gps-driver` |


### 1. Create a new branch
Before starting work, ensure your local `main` branch is up to date.

```bash
git switch main
git pull origin main
```

Create and switch to a new branch:

```bash
git switch -c feature/your-feature-name
```

### 2. Making Changes

Make changes on your branch and commit them regularly.

```bash
git add .
git commit -m "Example message"
```

Push the new branch:

```bash
git branch # Your branch is the one with *
git push -u origin feature/your-feature-name
```

### 3. Keeping Your Branch Updated

While developing, periodically incorporate changes from `main` to reduce merge conflicts.

```bash
git fetch origin
git merge origin/main
```

Resolve any merge conflicts, if necessary, before continuing development.

### 4. Pull Requests

Once your feature is complete:

1) Push your latest changes to GitHub.
2) Open a **Pull Request (PR)** targeting `main` on the GitHub website.
3) Request a review from another team member.
4) Address any feedback or merge conflicts.
5) Merge the PR once approved and tested.

### 5. After Merging

Once your pull request has been merged, update your local repository:

```bash
git switch main
git pull origin main
```

You can then delete your completed feature branch:

```bash
git branch -d feature/your-feature-name
```

Start a new branch for your next task.
