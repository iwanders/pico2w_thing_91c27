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

> Figure out how these flash partitions work, can we do something similar to [this PR](https://github.com/raspberrypi/pico-sdk/pull/1969)?

We now have four partitions, as specified in [partitions.json](./partitions.json). This is deployed to the mcu with `make partition`.

- One for firmware at the start, which is 2044K in size.
- Three more, of appropriate sizes for the `43439A0_clm.bin`, `43439A0_btfw.bin` and `43439A0.bin` firmwares.

The firmware is now deployed into partition zero with `picotool load -p 0 -u -v -x -t elf`, note the `-p 0`.

Not sure yet how to put the firmwares into these partitions, the information table is also odd;
```
un-partitioned_space : S(rw) NSBOOT(rw) NS(rw), uf2 { absolute }
partitions:
  0(A)       00002000->00201000 S(rw) NSBOOT(rw) NS(rw), id=0000000000000000, "firmware", uf2 { rp2350-arm-s, rp2350-riscv }, arm_boot 1, riscv_boot 1
  1(A)       00201000->00601000 S(rw) NSBOOT(rw) NS(rw), id=0000000000000001, "43439A0_clm.bin", uf2 { data }, arm_boot 1, riscv_boot 1
  2(A)       00601000->00603000 S(rw) NSBOOT(rw) NS(rw), id=0000000000000002, "43439A0_btfw.bin", uf2 { data }, arm_boot 1, riscv_boot 1
  3(A)       00603000->0063d000 S(rw) NSBOOT(rw) NS(rw), id=0000000000000003, "43439A0.bin", uf2 { data }, arm_boot 1, riscv_boot 1
```

Okay, that's fixed by setting a start position on the first data paritition.

Loading fw from flash isn't quite working yet, it stalls when that happens.
