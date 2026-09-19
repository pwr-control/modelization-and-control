# Foundations of Cooling Systems

Thermal and hydraulic sizing of the cooling used by the power converters of this repository.

## Contents

### `cooling_plate`
- Thermal-hydraulic sizing of an aluminium plate mounted on a pin-fin cold plate
  (160 × 40 mm, 5 mm fins, 6 mm pitch) cooled by 30 % ethylene-glycol/water at about 50 °C.
- Two approaches are run in parallel and compared: the manufacturer figure
  (18 mK/W at 18 l/min) scaled to the operating point with a power law, and an analytical
  cross-check through hydraulic diameter, Reynolds number, Gnielinski correlation, convective
  coefficient and straight-fin efficiency.
- Reported: conduction resistance of the plate, total resistance at 18 and 24 l/min, area
  enhancement over the plain base, thermal capacitance and time constant.
- Figures: thermal resistance versus flow rate from 10 to 35 l/min, stacked breakdown
  (aluminium conduction versus cold plate) at the two design flows, and fin efficiency versus
  the convective coefficient.
- `thermal_cold_plate_ver_ii.m` is the same script using the analytical result as the reference
  instead of the manufacturer figure.

### `cooling_systems/liquid_cooling_system`
- Simscape model of a liquid cooling loop. The init script sets only the solver step and the
  hydraulic geometry (utility branches ⌀10 mm × 5 m, mains ⌀25 mm × 25 m); pump, heat exchanger,
  loads and fluid properties are inside the model.
- A zipped copy of the project is kept in the parent folder.
