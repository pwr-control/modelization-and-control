# afe_uvw_three_phase — three-phase AFE with phase-frame (uvw) control

## Overview

| Item | Description |
|---|---|
| What it simulates | Three-phase two-level active front end controlled in the phase (uvw / rst) frame with the library blocks `afe_uvw_ctrl`; grid emulator with FRT tests; 690 V / 480 V / 400 V data sets selectable in the init. |
| Model | `afe_rst_ctrl.slx` |
| Init | `init_model.m` (struct `param`, functions in `private/`) |
| Post-processing | `plotting_results_afe.m`, `sim_results_post_process.m`, `voltage_spectrum_analysis.m` |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.m` from this folder (fills `param`, opens the model). |
| 3 | Run the simulation. |
| 4 | Run the post-processing scripts. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `param.sim.simlength` | 1.75 s | model StopTime |
| `param.sim.tc` | 1 µs | fixed step and logging base |
| `param.time.fpwm` | 4 kHz | double update |
| `ts` | 125 µs | control sample time |
| `param.time.t_measure` | = `simlength` | logging window |
| `param.sim.use_thermal_model` | 0 | ideal switches |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
