# scripts — shared calculation scripts

Stand-alone MATLAB scripts, no Simulink model.

| Script | Content |
|---|---|
| `snubber_rlc_check.m` | RC snubber check for a high-power three-phase diode or thyristor bridge: commutation inductance and overlap, di/dt, reverse-recovery peak, snap-off RLC transient, snubber losses, R/C sweeps. Worked case: 690 V, 4000 A DC, 4.5 MVA transformer, uk = 6 %. |
| `some_calculus.m` | Line and transformer impedance sizing for a 480 V / 60 Hz application (50 kA short-circuit line inductance, 1250 kVA transformer at Ucc = 5 %). |

## How to use

Run the script from MATLAB; the inputs are at the top of each file.
