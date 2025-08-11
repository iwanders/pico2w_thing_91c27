# Board

Notes for myself mostly.

## Connections & Information

Table of relevant connections & information.

| Device| Pin(#) | Rpi Pin | Rpi GPIO/Fun | Comment |
| --- |--- | --- | --- | --- |
| Led Indicator | 31 | GPIO26/ADC0 | | Blue, center of board |
| | | | |
| ICM-42688 |AP_SDO (1) | 6 | GPIO4/SPI0_RX  | SPI Serial data output|
| ICM-42688 |INT1 (4) | 9 | GPIO6  | Interrupt 1|
| ICM-42688 |INT2 (9) | 2 | GPIO1 | Interrupt 2, push/pull or open drain|
| ICM-42688 |AP_CS (12) | 7 | GPIO6/SPIO0_CSn | SPI Chip Select|
| ICM-42688 |AP_SCL (13) | 4 | GPIO2/SPI0_SCK | SPI Serial Clock|
| ICM-42688 |AP_SDI (14) | 5 | GPIO3/SPI0_TX | Spi Serial data Input|
| | | | |
| LSM6DSV320X | SDO (1) | 16 | GPIO12/SPI1_RX  | SPI Serial data output|
| LSM6DSV320X | INT1 (4) | 19 | GPIO14 | Programmable Interrupt |
| LSM6DSV320X | INT2 (9) | 20 | GPIO15 | Programmable Interrupt |
| LSM6DSV320X | CS (12) | 17 | GPIO13 | SPI Chip select |
| LSM6DSV320X | SCL (13) | 14 | GPIO10/SPI1_SCK | SPI Serial clock |
| LSM6DSV320X | SDA (14) | 15 | GPIO11/SPI1_TX | SPI Serial Data Input |
| | | | |
| ICS-43434 | WS (1) | 10 | GPIO7  | Serial Data-Word Select I2S |
| ICS-43434 | LR (2) |  | | Channel select solder jumper GND |
| ICS-43434 | SCK (4) | 11 | GPIO8 | Serial Data Clock I2S |
| ICS-43434 | SD (6) | 12 | GPIO9 | Serial Data Output I2S |
| | | | |
| BME280 | SDI(3) | 26  | GPIO20 /I2C0_SDA  | I2C SDA, 4.7k pullup |
| BME280 | SCK(4) | 27  | GPIO21/I2C0_SCL   | I2C SCL, 4.7k pullup |
| | | | |
| MX25L25645GM2I | CS (1) | 22  | GPIO17 / SPI0_CSn| Chip Select |
| MX25L25645GM2I | SO (2) | 21   | GPIO16 / SPI0_RX | Serial Data Output |
| MX25L25645GM2I | SCLK (6) | 24 | GPIO18/SPI0_SCK | Clock Input |
| MX25L25645GM2I | SI (5) |  25  | GPIO19/SPI0_TX  | Serial Data Input |
| | | | |
| SDCARD | CD (2) | 32  | GPIO27 / ADC1 | Chip Select |
| SDCARD | CMD (3) | 25  | GPIO19/SPI0_TX  | Serial Data Input |
| SDCARD | CLK (5) | 24 | GPIO18/SPI0_SCK | Clock Input |
| SDCARD | DAT0 (7) | 21   | GPIO16 / SPI0_RX | Serial Data Output |
| SDCARD | DET_A (10) | 29   | GPIO22  | Connects to GND with card (use internal pullup) |
| | | | |
| MCP73833| STAT1 (3) | | | Charging, Orange |
| MCP73833| STAT2 (4) | | | End of Charge, Red |
| MCP73833| PG (7) | | | Power Good, Green |
| MCP73833| PROG (6) | | | Charge current, 2.5k with two parallel 5k, 400mA default, 200mA if trace cut|
| | | | |



## Dimension notes

Pin 1 is at the origin, such that we can conveniently use a 2.54mm grid. Note to self; nothing ended up aligned on a 2.54 grid.

Vertical Center is thus at `17.78 / 2 = 8.89mm`.

Top of board is at `-(51-48.26)/2 = -1.37mm`.

Left edge of board is at `-(21-17.78) / 2 = -1.61mm`

Right edge of board is at `21 - 1.61 = 19.39mm`

Bottom edge of board is at `51 - 1.37 = 49.63mm`

Top left mounting hole is at x: `8.89 - 11.4 / 2 = 3.19`, y: `2 - 1.37=0.63mm`

Top right mounting hole is at x: `8.89 + 11.4 / 2 = 14.59`, y: `2 - 1.37=0.63mm`

Bottom mounting holes at y: `51 - 2 - 1.37`

Hole clearance: `(3.8 - 2.1) / 2 = 0.85mm`.

Debug headers are NOT at full 2.54mm offset.

Y value is 1.6mm off the botom edge; `49.63 - 1.6=48.03mm`

For the 2w; debug gnd at x: `19.39-7.3759=12.0141mm`, y at `49.63-19.8= 29.83mm`.

Swdio thus atx: `12.0141 + 2.54=14.5541mm`, swclk at `12.0141-2.54 = 9.4741mm`.

## Pico 2W
- 2648-SC1633CT-ND
- https://www.digikey.ca/en/products/detail/raspberry-pi/SC1633/25862726

## Battery Charger

Chip is `MCP73833`. Only the part ending in 33 has the power good pin, 34 is the timer flavour, which we don't want.

Is a DFN package... It has various flavours, digikey only carries VReg of 4.20v, going with no timer such that we can charge large batteries if necessary, with preconditioning, but no timer (in case we want to charge a large battery): `MCP73833-AMI/MF` in full.

- `MCP73833T-AMI/MFCT-ND`
- https://www.digikey.ca/en/products/detail/microchip-technology/MCP73833T-AMI-MF/1223181


Likely going to use a small LiPo battery, 180 mAh; like 30mm by 12mm, charge at 1C, gives about 200 mA charge current.

Equation 5-1, p17 of datasheet;

```math
I_\text{reg} = \frac{1000V}{R_\text{prog}}
```
With $I_\text{reg}$ in mA and $R_\text{prog}$ in kOhm.

To get 200mA current, we need $R_\text{prog}$ to be 5k Ohm.

To get 400mA current, we need $R_\text{prog}$ to be 2.5k Ohm.

Modified this to be two resistors of 5k, with one trace that's easily cut to drop from 400mA to 200mA.

> When the voltage at the VBAT pin reaches the regulation voltage, VREG, constant voltage regulation begins. The regulation voltage is factory set to 4.20V, 4.35V, 4.40V, or 4.50V with a tolerance of ± 0.75%.

> If temperature monitoring is not required, place a standard 10 kΩ resistor from THERM to VSS.


Capacitor C8, decoupling on vbat:
- Switched it to 0603, because there's plenty of space there (and more available)
- `1276-1044-1-ND`
- https://www.digikey.ca/en/products/detail/samsung-electro-mechanics/CL10A475KP8NNNC/3886702

Verified pin inputs.

### LEDs

Lets go with [OSRAM SMARTLED® 0603](https://ams-osram.com/products/product-families/smartled-0603) series, they seem made for diffuse indicator lights. Effectively `Kx EELP41.xx` with x changing depending on the color. Digikey [filter link](https://www.digikey.ca/en/products/filter/led-indication-discrete/105?s=N4IgjCBcoLQCxVAYygMwIYBsDOBTANCAPZQDaIAbAExwDMtIAugL7OFVkgCiXAMgApNmQA); search on series and `EELP`.


This [application note](https://look.ams-osram.com/m/7936f76d4c70ced0/original/Determination-of-resistances-for-brightness-compensation.pdf) `AN041` is helpful.

Key takehome is that it is not known which brightness we get when ordering, but a particular reel has a particular brightness group. Using this code on the reel, one can pick the resistor value.

Lets go for a nice traffic light, easy to remember, and with battery disconnected we see a green LED:

- Power Good: Green, PG pin on battery charger
- Charging: Orange, STAT1 pin on battery charger
- EOC: Red, STAT2 pin on battery charger


Positive voltage at the 'round' pin, negative voltage at the flat side.

#### Red
VBus is from USB, so 5V, the pin sinks.

- Color: `super red`:
- `KS EELP41.22-P1R2-58-A8J8-020-R18`
- https://www.digikey.ca/en/products/detail/ams-osram-usa-inc/KS-EELP41-22-P1R2-58-A8J8-020-R18/24765247

Brightness groups:
- P1 is brightness group 45-56 mcd at 20mA. (~50)
- 4 more groups here.
- R2 brightness group 140-180 mcd at 20mA. (~150)

Average of all groups is ~100 mcd at 20mA.

Forward voltage groups:
- 1.6V - 2.0V
- 2.0V - 2.4V
Nominal is 2V.

Relative luminous intensity is a straight line. Lets say aim is 10mcd?

- P1 brightness group needs 10/50 * 20mA = 4mA, so $R=\frac{V_s - V_f}{I_f}$, `(5-2)/4e-3` = 750 ohm.
- Avg needs 10/100 * 20mA = 2mA, so $R=\frac{V_s - V_f}{I_f}$, `(5-2)/2e-3` = 1500 ohm.
- R2 brightness group needs 10/150 * 20mA = 1.3mA, so $R=\frac{V_s - V_f}{I_f}$, `(5-2)/1.3e-3` = 2250 ohm.

Note; 3.3k ohm seems to be more than bright enough. 6.8k also works. 10k is dim, but still easily visisble, makes the 'dot' visible. Lets go with 4.3k

#### Orange
, VBus is from USB, so 5V, the pin sinks.

- Color: `orange`:
- `KO EELP41.22-Q1S2-25-A8J8-020-R18`
- https://www.digikey.ca/en/products/detail/ams-osram-usa-inc/KO-EELP41-22-Q1S2-25-A8J8-020-R18/24765237

Brightness groups:
- Q1 is brightness group 71-90 mcd at 20mA. (~80)
- 4 more groups here.
- S2 brightness group 224-280 mcd at 20mA. (~252)

Average of all groups is ~166 mcd at 20mA.

Forward voltage groups:
- 1.6V - 2.0V
- 2.0V - 2.4V
Nominal is 2V.

Relative luminous intensity is a straight line. Lets say aim is 10mcd?

- Q1 brightness group needs 10/80 * 20mA = 2.5mA, so $R=\frac{V_s - V_f}{I_f}$, `(5-2)/2.5e-3` = 1200 ohm.
- Avg needs 10/166 * 20mA = 1.2mA, so $R=\frac{V_s - V_f}{I_f}$, `(5-2)/1.2e-3` = 2500 ohm.
- S2 brightness group needs 10/252 * 20mA = 0.8mA, so $R=\frac{V_s - V_f}{I_f}$, `(5-2)/0.8e-3` = 3750 ohm.

10k is dim, but still easily visisble, makes the 'dot' visible. 5.1k is present, but not overly bright. 3.3k is also fine. Doesn't seem to be super bright like the green one is. 2k is definitely getting to 'no longer an indicator light'

#### Green
VBus is from USB, so 5V, the pin sinks.

- Color: `green`:
- `KT EELP41.12-S2U1-25-2X4Y-5-R18`
- https://www.digikey.ca/en/products/detail/ams-osram-usa-inc/KT-EELP41-12-S2U1-25-2X4Y-5-R18/24765233

Brightness groups:
- S2 brightness group 224-280 mcd at 20mA. (~252)
- 2 more groups here.
- U1 brightness group 450-560 mcd at 20mA. (~505)

Average of all groups is ~378 mcd at 20mA.

Forward voltage groups:
- 2X 2.3V - 2.4V
- 6 more groups
- 4Y 3.0V - 3.1V
Nominal is 2.7V.

Relative luminous intensity is not a straight line, graph is non uniform, but not by much... 7.5mcd goal?

- S2 brightness group needs 7.5/252 * 20mA = 0.6mA, so $R=\frac{V_s - V_f}{I_f}$, `(5-2.7)/0.6e-3` = 3833 ohm.
- Avg needs 10/378 * 20mA = 0.529mA, so $R=\frac{V_s - V_f}{I_f}$, `(5-2.7)/0.529e-3` = 4347 ohm.
- U2 brightness group needs 7.5/505 * 20mA = 0.3mA, so $R=\frac{V_s - V_f}{I_f}$, `(5-2.7)/0.3e-3` = 7666 ohm.


3.3k is still very very bright. 6.8 is still bright, 10k is nice for an always on light.
## Battery input

Following the guidance of the Pico 2W datasheet; `Diodes DMG2305UX`, just in SOT23 package.
- `DMG2305UX-7DICT-ND`
- https://www.digikey.ca/en/products/detail/diodes-incorporated/DMG2305UX-7/4340667

Enlarged landing pads on footprint, made a new footprint for this. Verified Pins

## Indicator LED
Connected to the uC, 3.3v driving voltage.

- Color: `blue`:
- `KB EELP41.12-P1R2-36-3X4X-5-R18`
- https://www.digikey.ca/en/products/detail/ams-osram-usa-inc/KB-EELP41-12-P1R2-36-3X4X-5-R18/24765234

Brightness groups:
- P1 is brightness group 45-56 mcd at 20mA. (~50)
- 4 more groups here.
- R2 brightness group 140-180 mcd at 20mA. (~150)

Forward voltage groups:
- 3X 2.6V - 2.7V
- 2 more groups
- 4X 2.9V - 3.0V
Nominal is 2.75V.

This comes from the uC, it can be PWM'd, but it does consume battery and the like. Lets go with 4mA, that's 10-30 mcd.

Nominal  $R=\frac{V_s - V_f}{I_f}$, `(3.3-2.75)/4e-3` = 137.5 ohm.

At full brightness;`(3.3-2.75)/20e-3` = 27 ohm, 30 ohm is approx 18mA.

Well, 3.3k is perfectly fine for an indicator led. 200 ohm is bright enough to be uncomfortable (47 even more so), 2k is also fine.


## LMS6DSV320X

Imu, large acceleration range;
- `497-LSM6DSV320XTRCT-ND`
- https://www.digikey.ca/en/products/detail/stmicroelectronics/LSM6DSV320XTR/26254547

Decoupling caps, 100nF, 0402:
- `1276-1022-1-ND`
- https://www.digikey.ca/en/products/detail/samsung-electro-mechanics/CL05A104KP5NNNC/3886680

Verified the pins.

## ICM-42688-P

Imu, high data rate;
- `1428-ICM-42688-PCT-ND`
- https://www.digikey.ca/en/products/detail/tdk-invensense/ICM-42688-P/10824934

Caps:
- One 100nF, see the other IMU.

Cap 10nF:
- `1276-1028-1-ND`
- https://www.digikey.ca/en/products/detail/samsung-electro-mechanics/CL05B103KB5NNNC/3886686

Cap 2.2uF:
- `1276-1461-1-ND`
- https://www.digikey.ca/en/products/detail/samsung-electro-mechanics/CL05A225KO5NQNC/3887119

Verified the pins.

## ICS-43434

Decoupling cap is 100nF, see above.

- `1428-1066-1-ND`
- https://www.digikey.ca/en/products/detail/tdk-invensense/ICS-43434/6140298

Verified pins.

## Memory

Decoupling cap is 100nF, see above.

- `1092-1234-ND`
- https://www.digikey.ca/en/products/detail/macronix/MX25L25645GM2I-08G/7914972


## SD Card

Decoupling cap 100nF, see above.

- `WM6357CT-ND`
- https://www.digikey.ca/en/products/detail/molex/1040310811/2370379

## BME280

Decoupling caps 100nF, see above.
- `828-1063-1-ND`
- https://www.digikey.ca/en/products/detail/bosch-sensortec/BME280/6136306

## Connector

JST PH 2mm pitch, through hole side connector.
- `455-1719-ND`
- https://www.digikey.ca/en/products/detail/jst-sales-america-inc/S2B-PH-K-S/926626

## Resistors

~Filter [link](https://www.digikey.ca/en/products/filter/chip-resistor-surface-mount/52?s=N4IgjCBcoMxaBjKAzAhgGwM4FMA0IB7KAbXDBggF18AHAFyhAGU6AnASwDsBzEAX3xgAbPBBJIaLHkIkQMAJxgArAA4Q1EPUYsOPfvgC0EaGJQYc%2BIpFJgATAHYRlASAO3R4tgFdpV0kvUXA3kPKG9fWSoggBZQiXMI6xBomCUnPgygA), `Samsung Electro-Mechanics`, `1%`, `RC` series, and in stock.~

But [this filter](https://www.digikey.ca/en/products/filter/chip-resistor-surface-mount/52?s=N4IgjCBcoMxaBjKAzAhgGwM4FMA0IB7KAbXDBgnwDYBOKkfAdgoZAA4AWN1mx%2BgXXwAHAC5QQAZREAnAJYA7AOYgAvvjD1oIJJDRY8hEiBg0wAVm6CQo8VLlLV%2BALQQtOvTnxFIpMAAYQfjUQJxp4bSgZAFcDb19A4KcAJnCdaNijMwSVFSA) `Panasonic Electronic Components`, `<1%`, `ERJ-2RK` series has more options.

- R1, R2: 4.7k  -> `P4.70KLCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF4701X/1746231
- R4, 4347 -> 4.3k `P4.32KLCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF4321X/192339
- R5, 1500 -> 1.5k `P1.50KLCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF1501X/192060
- R3, 2500 -> 2.49k `P2.49KLCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF2491X/192203
- R7,8,10 10k -> `P10.0KLCT-ND`,  https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF1002X/192073
- R9, 137 -> `P137LCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF1370X/192127
- R6, R11 -> 4.99k `10-ERA-2AEB4991XCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERA-2AEB4991X/2026116

Backups:
- R3: 1200 -> 1.21k, `P1.21KLCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF1211X/192051
- R3: 3750 -> 3.74k, `P3.74KLCT-ND` https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF3741X/192286
- R5 750 -> `P750LCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF7500X/192470
- R5 2250 -> 2.2k `P2.20KLCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF2201X/1746150
- R4 3833 -> 3.83k, `P3.83KLCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF3831X/192287
- R4 7666 -> 7.68k `P7.68KLCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF7681X/192458
- R9 470 -> 470 `P470LCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF4700X/1746229
- R9 30 -> `P30DDCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERA-2AKD300X/1706065


- 4.7uF in 0402; `1276-CL05A475KP5ZRNCCT-ND`, https://www.digikey.ca/en/products/detail/samsung-electro-mechanics/CL05A475KP5ZRNC/10478972


## Stencil

Actual info: [TDK AN-000393 IMU PCB Design and MEMS Assembly Guidelines](https://invensense.tdk.com/download-pdf/an-000393-imu-pcb-design-and-mems-assembly-guidelines/), for LGA they state the land should be different from the solder mask opening, solder mask being +0.1mm larger. Stencil thickness should be at least 100um. Not too much on stencil layout.

That is the same as STMs `TN0018: Handling, mounting, and soldering guidelines for MEMS devices` pdf, which also states solder mask opening should be +0.1mm.

So the footprint should be:
```
For LGA pad spacing greater than 200 μm:
A = PCB land length = LGA solder pad length + 0.1 mm
B = PCB land width = LGA solder pad width + 0.1 mm
```
Pad is 0.25mm by 0.475mm, so pad should be 0.35mm by 0.575mm. Doesn't state to round corners.

And then the solder mask:
```
C = Solder mask opening length (where applicable) = PCB land length + 0.1 mm
D = Solder mask opening width = PCB land width + 0.1 mm
```

Solder mask opening length is pad + 0.1mm, so grow of 0.05mm.  They do state to not put solder mask below the IMU's.

Seems for 0402 one should subtract 1mil (0.0254mm) from the pad? Or about -10%?

Lets go with:
> Board Setup -> Solder Mask/Paste -> Solder Paste Settings -> Relative clearance -10%

Note after assembling all three boards; wouldn't change a thing about the solder stencil.


## Prep

Trying jlcpcb instead of oshpark because they can produce a stencil. [Instructions](https://jlcpcb.com/help/article/how-to-generate-gerber-and-drill-files-in-kicad-8) for kicad export.

PCB is 21 by 51mm,  lets do 100x120mm to ensure we have space for the paste and tape.

## Notes on v1

- ICM-42688-p pin 8 should've exited straight (towards charge ic), there's space and currently the solder mask is over the trace which may pull the IC to the side.
- Microphone hole looks like it could've been larger.


## First board assembly

Approach; double sided poster tape to tape spare PCBs to the work area, build a rig that holds the boards, don't forget tape (with non-stick cover) underneath the board we're working on. Use other tape for the stencil hinge, ensure orientation in translation and rotation, push the board for final adjustment if you have to during the paste application.

Both JST plugs I have have opposite polarity from my board...

- Swapped ICM 2.2uF and 100nF (C1 & C2) during assembly.
- Indicator led works, reasonable brightness for an indicator.
- Battery charger: no battery; all leds on. Green LED could've been less bright. Can power the board with battery.
  - Battery 280mAh initially at 4.0v, red turns off when battery connected, orange stays on. Red turned off, with battery at 4.15v.
  - Battery does start the program on the board.
  - Big 1200mAh battery start at 3.7v, IC does heat up, but nothing excessive. Red at 4.17v.
- ICM42688 is responsive through exclusive SPI0, who am I register.
- LSM6dsv320x is responsive through exclusive SPI1, who am I register.
- BME280 responds the Chip ID register.
- Flash is responsive, reading JEDEC register works.
- SDCard is functional (though at low clock rate?), card detect also works.

## Second board assembly
Changing charging resistor, the orange LED to be brighter; 2.49k instead of 4.9k. Can't change green as I don't have 0402 higher than 10k.
Leds have 'flat' edge towards inside of the board.

- More solder paste would've been better, first squeeze didn't apply enough, had to wipe everything and retry, still sparse on the imu's, they don't seem to float above the board. Probably need to move slower and more deliberate push it into the holes. One of the imu's is _very_ close to the pcb, instead of 'laying' on solder.
- hw test passes, microphone, bme, flash, imu's are reachable, as is sd card. Charger and power switchover seems to work as well.

## Third board
Same as second, but without the LSM. Had to do two passes with the solder paste, but turned out fine.
