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

## Battery Charger MCP73833


Only the part ending in 33 has the power good pin, 34 is the timer flavour, which we don't want.

Likely going to use a small LiPo battery, 180 mAh; like 30mm by 12mm, charge at 1C, gives about
200 mA charge current.


Equation 5-1, p17 of datasheet;

```math
I_\text{reg} = \frac{1000V}{R_\text{prog}}
```
With $I_\text{reg}$ in mA and $R_\text{prog}$ in kOhm.

To get 200mA current, we need $R_\text{prog}$ to be 5k Ohm.

> When the voltage at the VBAT pin reaches the regulation voltage, VREG, constant voltage regulation begins. The regulation voltage is factory set to 4.20V, 4.35V, 4.40V, or 4.50V with a tolerance of ± 0.75%. 

Need to pick the right 'flavour' of IC to ensure we get the correct termination voltage.

> If temperature monitoring is not required, place a standard 10 kΩ resistor from THERM to VSS.

### LEDs

Lets go with [OSRAM SMARTLED® 0603](https://ams-osram.com/products/product-families/smartled-0603) series, they seem made for diffuse indicator lights. Effectively `Kx EELP41.xx` with x changing depending on the color.


This [application note](https://look.ams-osram.com/m/7936f76d4c70ced0/original/Determination-of-resistances-for-brightness-compensation.pdf) `AN041` is helpful.

Key takehome is that it is not known which brightness we get when ordering, but a particular reel has a particular brightness group. Using this code on the reel, one can pick the resistor value.

#### Power Good
VBus is from USB, so 5V, the pins sink.

Power good, `super red`:
- `KS EELP41.22-P1R2-58-A8J8-020-R18`
- https://www.digikey.ca/en/products/detail/ams-osram-usa-inc/KS-EELP41-22-P1R2-58-A8J8-020-R18/24765247

Brightness groups:
- P1 is brightness group 45-56 mcd at 20mA. (~50)
- 4 more groups here.
- R2 brightness group 140-180 mcd at 20mA. (~150)
Middle is ~100 mcd at 20mA.

Forward voltage groups:
- 1.6V - 2.0V
- 2.0V - 2.4V
Nominal is 2V.

Relative luminous intensity is a straight line. Lets say aim is 10mcd?

- P1 brightness group needs 10/50 * 20mA = 4mA, so $R=\frac{V_s - V_f}{I_f}$ (`5-2/4e-3`) = 750 ohm.
- R2 brightness group needs 10/150 * 20mA = 1.3mA, so $R=\frac{V_s - V_f}{I_f}$ (`5-2/4e-3`) = 2250 ohm.


- Charging: Orange
- Done: Green


