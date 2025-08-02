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
Total flash size is `4096K`.

> Figure out how these flash partitions work, can we do something similar to [this PR](https://github.com/raspberrypi/pico-sdk/pull/1969)?

Changed it back to two partitions, one for the program, and a second one for the static files, now created by the [static_files](./src/static_files.rs) module. This allows packing multiple static files into a single partition and iterating over the files in it
by filename.

- `make partition` creates the partition table and flashes it to the device.
- `make partition-static-files-upload` creates the static files and flashes them into the static files partition.

Note that the ids in picotool's `-p` is a zero indexed partition counter, not the partition id.

The firmware is now deployed into partition zero with `picotool load -p 0 -u -v -x -t elf`, note the `-p 0`.

Without setting a `start` attribute on the second partition (first data) partition it gets a very large size, which is wrong.

Next up is consuming the data from the static files in the firmware load, and ideally finding the partition first in the XIP offset.

We can load the data by using the `XIP_NOCACHE_NOALLOC_NOTRANSLATE_BASE` pointer and offset from there, that's where the flash
itself is in the address space. I put a helper into the `rp2350_util` module to obtain byte slices to the utils.

Currently the offset & lengths need to be read from the partition table and inserted into the program to create the slice.

In the future, we could consider a single partition with static data, seems `picotool load` can write to arbitrary addresses. We can
write all metadata/files into a single data partition with some 'file' header and then just iterate over those after obtaining the
partition table to find the correct partition.

## Commisioning / AHK / Matter
Apple homekit either uses BLE or it uses WiFi, it doesn't facilitate providing wifi passwords easily.

Matter uses ble for setup, then switches to wifi.

There is a project [rs-matter](https://github.com/project-chip/rs-matter) that implements the spec in Rust, one of its devs has an [integration with embassy](https://github.com/ivmarkov/rs-matter-embassy) that appears to make this all pretty simple. The official sdk for matter is [here](https://github.com/project-chip/connectedhomeip) from the looks of it. The [rs-matter](https://github.com/project-chip/rs-matter) project hasn't had a release in a long time, on Linux its examples are flaky at the time of writing. Resulting in `Error::TLVTypeMismatch` for the on off example and iOS 18.5, pairing with the QR code in the terminal is slick though. And [rs-matter-embassy](https://github.com/sysgrok/rs-matter-embassy) seems to be more ready made, but its examples pull OpenThreads, and seem to also depend on libssl?

That does seem a LOT more daunting than just mocking an Apple Homekit Accessory though, if I just want to control a few switches and read some values.
