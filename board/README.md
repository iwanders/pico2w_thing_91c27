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

Likely going to use a small LiPo battery, 180 mAh; like 30mm by 12mm, charge at 1C, gives about
200 mA charge current.

Equation 5-1, p17 of datasheet;

```math
I_\text{reg} = \frac{1000V}{R_\text{prog}}
```
With $I_\text{reg}$ in mA and $R_\text{prog}$ in kOhm.

To get 200mA current, we need $R_\text{prog}$ to be 5k Ohm.
