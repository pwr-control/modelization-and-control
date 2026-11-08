# afe_dq_three_phase — three-phase two-level AFE with dq control

## Overview

| Item | Description |
|---|---|
| What it simulates | Three-phase two-level active front end, 400 V, 250 kW, on a 1 MVA Dyn11 grid emulator; dq vector control with resonant PI, DC-link PI, FHT-based dq-PLL; IGBT thermal model. |
| Model | `afe_dqctrl_param.slx` |
| Init | `init_model.m` (struct `param`) |
| Post-processing | `plotting_results_afe.m`, `plotting_results_dclink.m`, `sim_results_post_process.m` (losses, temperatures), `voltage_spectrum_analysis.m`, `voltage_islandmode_spectrum_analysis.m` |
| Other | `migrate_afe_dqctrl.m` (migration of the model to `param.*`), `non_virtual_blocks_counting.m` |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.m` from this folder: it fills `param` and opens the model. |
| 3 | Run the simulation. |
| 4 | Run the post-processing scripts. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `param.sim.simlength` | 1 s | model StopTime |
| `param.sim.tc` | 0.625 µs | MaxStep of ode23tb and logging base |
| `param.time.fpwm` | 4 kHz | double update |
| `ts` | 125 µs | control sample time |
| `param.time.t_measure` | 0.4 s | logging window |
| `param.sim.use_thermal_model` | 1 | IGBT loss/thermal model on (5 nonlinear iterations, slower) |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
