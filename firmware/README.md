# Firmware

Notes and description of the firmware.

## Building Picotool
It's a nice statically linked binary that seems better supported than [elf2uf2-rs](https://github.com/JoNil/elf2uf2-rs),
going to try to use the 'upstream' supported tools to flash the device.

- The usual build tools; `apt install build-essential pkg-config libusb-1.0-0-dev cmake`, usually just `libusb-1.0-0-dev`.
- Clone `https://github.com/raspberrypi/picotool`, checkout release tag (`2.1.1`).
- Clone `https://github.com/raspberrypi/pico-sdk`, checkout release tag (`2.1.1`).
- `cmake ../picotool/ -DPICO_SDK_PATH=../pico-sdk`
- `make picotool`
- `./picotool`, or move it elsewhere.

Don't forget to install the udev rules from the picotool udev directory to `/etc/udev/rules.d/`.

## Probe-rs
Upstream version complains about glibc versions, so compile from source with;
```
cargo install probe-rs-tools --locked
```

## Reset interface

The USB endpoint to allow the picotool to reset works. It is located in [./src/usb_picotool_reset.rs](./src/usb_picotool_reset.rs).

Relevant links:

- https://github.com/embassy-rs/embassy/issues/3726
- https://github.com/raspberrypi/pico-sdk/blob/9a4113fbbae65ee82d8cd6537963bc3d3b14bcca/src/common/pico_usb_reset_interface_headers/include/pico/usb_reset_interface.h
- https://github.com/raspberrypi/pico-sdk/tree/9a4113fbbae65ee82d8cd6537963bc3d3b14bcca/src/rp2_common/pico_stdio_usb

- https://github.com/raspberrypi/pico-sdk/blob/9a4113fbbae65ee82d8cd6537963bc3d3b14bcca/src/rp2_common/pico_stdio_usb/reset_interface.c


# defmt-print

Currently uses a logger that pushes the defmt data over the serial port; [./src/defmt_serial.rs](./src/defmt_serial.rs).

From https://crates.io/crates/defmt-print, install with `cargo install defmt-print`

```
defmt-print  -e ./target/thumbv8m.main-none-eabihf/release/firmware serial --path /dev/ttyACM
```

# Wifi / Bluetooth firmware

> Figure out how these flash partitions work, can we do something similar to [this PR](https://github.com/raspberrypi/pico-sdk/pull/1969)?

We now have four partitions, as specified in [partitions.json](./partitions.json). This is deployed to the mcu with `make partition`.

- One for firmware at the start, which is 2044K in size.
- Three more, of appropriate sizes for the `43439A0_clm.bin`, `43439A0_btfw.bin` and `43439A0.bin` firmwares.

The firmware is now deployed into partition zero with `picotool load -p 0 -u -v -x -t elf`, note the `-p 0`.

Without setting a `start` attribute on the second partition (first data) partition it gets a very large size, which is wrong.

We can load the data by using the `XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE` pointer and offset from there, that's where the flash
itself is in the address space. I put a helper into the `rp2350_util` module to obtain byte slices to the utils.

Currently the offset & lengths need to be read from the partition table and inserted into the program to create the slice.

In the future, we could consider a single partition with static data, seems `picotool load` can write to arbitrary addresses. We can
write all metadata/files into a single data partition with some 'file' header and then just iterate over those after obtaining the
partition table to find the correct partition.
