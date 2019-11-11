# Setting up the dev environment

This project has been developed on a Linux development environment. The scripts and steps discussed below are unlikely to work on Windows or OSX. [The Embedded Rust Book](https://rust-embedded.github.io/book/) introduction section has a detailed installation guide for all platforms. You will also need to adjust `firmware/.cargo/config` which discussed [in the hardware section of the book](https://rust-embedded.github.io/book/start/hardware.html). The [cortex-m-quickstart repo](https://github.com/rust-embedded/cortex-m-quickstart) also has lots of useful information. If you follow that then skip to the [Project specific section](#project-specific) you should be all set up.

## Tools
### openocd
* Install [openocd](http://openocd.org/getting-openocd/)
* `apt install openocd` will likely be sufficient if you are using Ubuntu or Debian
* If you have any issues (particularly around the device not being reset correctly) consider installing a more up to date version from source

### rustup
* Install [rustup](https://rustup.rs/)

### bootstrap `(firmware/bootsrap)`
This script will 
* add the required toolchains using `rustup`
* add `cargo-binutils`
* add `llvm-tools-preview`
* add `cargo-watch`
* kick off a full build & doc 

## Project specific
### run_openocd `(firmware/run_openocd)`
This script will 
* launch `openocd` with the included `openocd.cfg` configuration

You'll need to have this running for [release](#release-firmwarerelease), [debug](#debug-firmwaredebug), [device_reset_halt](#device_reset_halt-firmwaredevice_reset_halt) and [device_reset_run](#device_reset_run-firmwaredevice_reset_run) to work correctly.

### watch `(firmware/watch)`
This script will
* monitor the rust source and build whenever a file changes

### release `(firmware/release)`
This script will
* run `device_reset_halt` to ask openocd to reset and halt the target hardware
* run the release version of the `main` binary (`src/bin/main.rs`) on the target hardware using `gdb`

### debug `(firmware/debug)`
As [release](#release-firmwarerelease) but in compiled in release mode which makes stepping through code easier but can be extremely slow in some circumstances. Suggest adding `cortex_m::asm::bkpt()` just before whatever you want to step through then running `debug`.

### device_reset_halt `(firmware/device_reset_halt)`
This script will
* telnet to openocd (127.0.0.1:4444) and issue a `reset halt` command which will cause the MCU to reset and stop ready to be programmed

### device_reset_run `(firmware/device_reset_run)`
This script will
* telnet to openocd (127.0.0.1:4444) and issue a `reset run` command which will cause the MCU to reset and run whatever is currently on it's Flash/SRAM (depending on BOOT0/1 jumper config)

### tmux_workspace `(firmware/tmux_workspace)`
This script will
* start or resume a `tmux` session
* split the window into 5
* run `watch`
* run `../../itm_log` //TODO: upload this tool
* run `openocd`
