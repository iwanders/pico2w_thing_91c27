# Pico2w_thingy_91c27

A project for a PCB with a bunch of sensors on it that attaches to a Raspberry Pi Pico 2W to log things.

- ICM-42688 IMU
- LSM6DSV320X IMU
- ICS-43434 Microphone
- BME280 Humidity, Temperature, Pressure sensor.
- MX25L25645GM2I 256 megabit flash.
- MCP73833 LiPo Battery charger


See the [./board/](./board/) or the [./firmware/](./firmware/) readme's for more information.

This project led to [micro_hap](https://github.com/iwanders/micro_hap), a no_alloc, no_std implementation of Apple's
HomeKit accessory protocol. Currently the firmware makes a HomeKit temperature & humidity sensor.


## What's with the name?
Naming is hard... This is a bit of a vague project to attempt... also:
```
$ head -c 100 /dev/urandom  | md5sum | head -c 5
91c27
```

## micro_hap

A large portion of this repository's history is around the HomeKit Accessory Protocol implementation.
This is now split out as the [micro_hap](https://github.com/iwanders/micro_hap) repository.


# License
License is [`BSD-3-Clause`](./LICENSE) for the board & firmware.
