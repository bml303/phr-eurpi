# Rust framework for the EuroPI module

See here: [EuroPi](https://github.com/Allen-Synthesis/EuroPi/tree/main). It's currently a work in progress with the idea of providing a similar API but for algorithms  written in Rust. Since the EuroPi includes an I2C interface it might make sense to use it for periperals - not yet determined which ones.

## Prerequisites (assuming a Debian or Ubuntu Linux host environment)

```shell
apt install automake autoconf build-essential texinfo libtool libftdi-dev libusb-1.0-0-dev libudev-dev openocd

rustup target install thumbv6m-none-eabi

curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh
cargo install elf2uf2-rs
```

## Running from the host

A [Raspberry Pi Debug Probe] (https://www.raspberrypi.com/documentation/microcontrollers/debug-probe.html) is required. Once this is established and the Pico is connected all you need to do is

```shell
cargo run
```

The code at this point in time is completely untested and running it might result in mild or even severe disappointment. If things go according to plan (which they never but even so) this will change in the forseeable future.

## Installing for prioduction

Once the code is ready to be run in production compile the release version and install the u2f binary:

```shell
cargo build --release --target=thumbv6m-none-eabi
elf2uf2-rs ./target/thumbv6m-none-eabi/release/phr-rpcem
cp ./target/thumbv6m-none-eabi/release/phr-rpcem.uf2 /media/$USERNAME/RPI-RP2
```