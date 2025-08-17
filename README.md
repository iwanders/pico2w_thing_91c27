# Pico2w_thingy_91c27

A work in progress repo for a PCB with a bunch of sensors on it that attaches to a Raspberry Pi Pico 2W to log things.

- ICM-42688 IMU
- LSM6DSV320X IMU
- ICS-43434 Microphone
- BME280 Humidity, Temperature, Pressure sensor.
- MX25L25645GM2I 256 megabit flash.
- MCP73833 LiPo Battery charger


See the [./board/](./board/) or the [./firmware/](./firmware/) readme's for more information.


## What's with the name?
Naming is hard... This is a bit of a vague project to attempt... also:
```
$ head -c 100 /dev/urandom  | md5sum | head -c 5
91c27
```


# License
License is [`BSD-3-Clause`](./LICENSE) for the board & firmware. Apache-2.0 for the `micro_hap` component, since that's
based on the code from [HomeKitADK](https://github.com/apple/HomeKitADK).
