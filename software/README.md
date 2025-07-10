# RP2040 Flight Controller Build Guide

Clone this repository:
> `git clone https://github.com/Chaves02/NAV_flight_controller.git` <br />

## Get the SDK to work with RP2040 microcontroller ##
The Raspberry Pi Pico development is fully supported with both C/C++ SDK and an official MicroPython port. <br />
As Pico is built around the RP2040 microcontroller designed by Raspberry Pi, we can use the same environment to build our projects around our own board. <br />

The SDK can be got in https://github.com/raspberrypi/pico-sdk: (inside NAV_flight_controller folder)
> `git clone -b master https://github.com/raspberrypi/pico-sdk.git` <br />
> `cd pico-sdk` <br />
> `git checkout 5258ee640bf08909cd8d3e9c3d1c75afb51d9352` <br />
> `git submodule update --init`

To build the applications is necessary to install some extra tools:
> `sudo apt update` <br />
> `sudo apt install cmake gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential`

## Add Submodule: `RP2040_AGROLIB` 

To use this repository, create 'extern' folder and add it as a submodule of the repository NAV_flight_controller, inside extern folder.

> `mkdir extern` <br />
> `cd extern` <br />
> `git submodule add https://gitlab.inesctec.pt/agrob/rp2040_agrolib.git`

Some submodules have to be initialized and updated. Use the following command inside its folders.

> `git submodule update --init`

### Add SDK Path as environment variable ###

Add SDK path to the file (define the absolute path in your environment):

> `export PICO_SDK_PATH = /.../pico-sdk/` (inside scripts folder)

### Build Submodule: `RP2040_AGROLIB`

> `cd extern/rp2040_agrolib/software/scripts` <br />
> `./build.sh`

## Build `NAV_flight_controller`

> `cd software` <br />
> `mkdir build` <br />
> `cd build` <br />
> `cmake ..` <br />
> `make`

## Upload to RP2040

> `cp /home/chaves/Documentos/NAV_flight_controller/software/build/velocity_mode.uf2 /media/chaves/RPI-RP2`
