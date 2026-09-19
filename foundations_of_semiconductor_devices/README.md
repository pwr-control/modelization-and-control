# Foundations of Semiconductor Devices

Switching-behaviour studies of power semiconductors in Simscape. The device models themselves
(`IGBT (switching dynamics)`, `Power MOSFET (switching dynamics)`, `RR Diode (Lauritzen-Ma)`,
`IGBT_SIMPLE_2TH`, `MOS_SIMPLE_2TH`) come from the companion **`library`** repository; the models
here are the test benches that exercise them.

Two levels of modelization are used:

- **Switching dynamics** — nonlinear capacitances, gate loop, stray inductances and
  reverse-recovery diode, with a very fine solver step. Used to look at the transient itself
  (turn-on and turn-off waveforms, overvoltage, dv/dt, di/dt).
- **Simplified device with thermal ports** — conduction losses plus Eon/Eoff/Err lookup tables,
  with junction, heatsink and ambient thermal networks. Used to look at losses and temperatures
  over a full operating cycle.

All folders share the same `init_model.m` template (690 V application, fPWM = 4 kHz,
Mitsubishi CM1200DW-34T as IGBT, Danfoss SKM1700MB20R4S2I4 as SiC MOSFET, thermal model on).

## Contents

### `dynamic_analysis_igbt/dpt`
- Double pulse test of a single IGBT half-leg: switching-dynamics IGBT, Lauritzen-Ma
  reverse-recovery diode, DC link, load inductor, snubber and stray-inductance blocks.
- Very fine Simscape step (`tc_factor = 16000`) over a 200 µs window to resolve the transient.
- Results are observed on scopes; there is no logging, so no scripted Eon/Eoff/Err extraction in
  this folder.

### `dynamic_analysis_igbt/buck`
- The same IGBT leg in a buck converter with an RLC load, PWM-driven, 12 ms simulation.
- Device voltages and currents, diode currents and snubber currents are logged to the workspace,
  so switching losses and dv/dt can be evaluated from the waveforms.

### `dynamic_analysis_igbt/afe`
- Three-phase grid-connected AFE using the simplified IGBT with two thermal ports, LCL filter,
  cable effects and the complete control chain (dq-PLL / FHT-PLL, dq and resonant current
  control, DC-link loop, space-vector PWM, IEC power meter), 1.25 s simulation.
- Logged quantities are loss- and temperature-oriented: IGBT and diode losses, junction-to-
  heatsink and heatsink-to-ambient temperatures, on a liquid-cooled plate model.

### `dynamic_analysis_mosfet/dpt`
- Double pulse test of a power MOSFET half-leg, same structure as the IGBT version, 400 µs
  window; thermal model disabled.

### `dynamic_analysis_mosfet/afe`
- The AFE bench built around the simplified MOSFET with thermal ports, with the same logged set
  as the IGBT version — the two allow a direct Si/SiC comparison on the same converter.
