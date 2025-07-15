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
- https://github.com/embassy-rs/embassy/issues/3726
- https://github.com/raspberrypi/pico-sdk/blob/9a4113fbbae65ee82d8cd6537963bc3d3b14bcca/src/common/pico_usb_reset_interface_headers/include/pico/usb_reset_interface.h
- https://github.com/raspberrypi/pico-sdk/tree/9a4113fbbae65ee82d8cd6537963bc3d3b14bcca/src/rp2_common/pico_stdio_usb

- https://github.com/raspberrypi/pico-sdk/blob/9a4113fbbae65ee82d8cd6537963bc3d3b14bcca/src/rp2_common/pico_stdio_usb/reset_interface.c

```
pico_enable_stdio_usb(<yourTargetName> 1)
reduces to
set_target_properties(${TARGET} PROPERTIES PICO_TARGET_STDIO_USB ${ENABLED})

```

# defmt-print


From https://crates.io/crates/defmt-print, install with `cargo install defmt-print`

```
defmt-print  -e ./target/thumbv8m.main-none-eabihf/release/firmware serial --path /dev/ttyACM
```

# Wifi / Bluetooth firmware

Figure out how these flash partitions work, can we do something similar to [this PR](https://github.com/raspberrypi/pico-sdk/pull/1969)?
