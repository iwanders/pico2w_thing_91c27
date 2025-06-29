# Board

Notes for myself mostly.

## Dimension notes

Pin 1 is at the origin, such that we can conveniently use a 2.54mm grid.

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

> When the voltage at the VBAT pin reaches the regulation voltage, VREG, constant voltage regulation begins. The regulation voltage is factory set to 4.20V, 4.35V, 4.40V, or 4.50V with a tolerance of ± 0.75%. 

> If temperature monitoring is not required, place a standard 10 kΩ resistor from THERM to VSS.


Capacitor C8, decoupling on vbat:
- Switched it to 0603, because there's plenty of space there.
- `1276-1044-1-ND`
- https://www.digikey.ca/en/products/detail/samsung-electro-mechanics/CL10A475KP8NNNC/3886702

Verified pin inputs.

## Battery input

Following the guidance of the Pico 2W datasheet; `Diodes DMG2305UX`, just in SOT23 package.
- `DMG2305UX-7DICT-ND`
- https://www.digikey.ca/en/products/detail/diodes-incorporated/DMG2305UX-7/4340667

Enlarged landing pads on footprint, made a new footprint for this. Verified Pins

## LEDs

Lets go with [OSRAM SMARTLED® 0603](https://ams-osram.com/products/product-families/smartled-0603) series, they seem made for diffuse indicator lights. Effectively `Kx EELP41.xx` with x changing depending on the color. Digikey [filter link](https://www.digikey.ca/en/products/filter/led-indication-discrete/105?s=N4IgjCBcoLQCxVAYygMwIYBsDOBTANCAPZQDaIAbAExwDMtIAugL7OFVkgCiXAMgApNmQA); search on series and `EELP`.


This [application note](https://look.ams-osram.com/m/7936f76d4c70ced0/original/Determination-of-resistances-for-brightness-compensation.pdf) `AN041` is helpful.

Key takehome is that it is not known which brightness we get when ordering, but a particular reel has a particular brightness group. Using this code on the reel, one can pick the resistor value.

### Power Good
PG pin on battery charger, VBus is from USB, so 5V, the pin sinks.

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
- Avg needs 10/100 * 20mA = 2mA, so $R=\frac{V_s - V_f}{I_f}$, `(5-2)/4e-3` = 1500 ohm. 
- R2 brightness group needs 10/150 * 20mA = 1.3mA, so $R=\frac{V_s - V_f}{I_f}$, `(5-2)/4e-3` = 2250 ohm.

### Charging
STAT1 pin on battery charger, VBus is from USB, so 5V, the pin sinks.

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

### End of Charge
STAT2 pin on battery charger, VBus is from USB, so 5V, the pin sinks.

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

### Indicator LED
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


## LMS6DSV320X

Imu, large acceleration range;
- `497-LSM6DSV320XTRCT-ND`
- https://www.digikey.ca/en/products/detail/stmicroelectronics/LSM6DSV320XTR/26254547?s=N4IgTCBcDaIDIGUCyA2AIggagZjABhAF0BfIA

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
- R3, R6 2500 -> 2.49k `P2.49KLCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF2491X/192203
- R7,8,10 10k -> `P10.0KLCT-ND`,  https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF1002X/192073
- R9, 137 -> `P137LCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF1370X/192127

Backups:
- R3: 1200 -> 1.21k, `P1.21KLCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF1211X/192051
- R3: 3750 -> 3.74k, `P3.74KLCT-ND` https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF3741X/192286
- R5 750 -> `P750LCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF7500X/192470
- R5 2250 -> 2.2k `P2.20KLCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF2201X/1746150
- R4 3833 -> 3.83k, `P3.83KLCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF3831X/192287
- R4 7666 -> 7.68k `P7.68KLCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF7681X/192458
- R9 470 -> 470 `P470LCT-ND`, https://www.digikey.ca/en/products/detail/panasonic-electronic-components/ERJ-2RKF4700X/1746229



## BOM

