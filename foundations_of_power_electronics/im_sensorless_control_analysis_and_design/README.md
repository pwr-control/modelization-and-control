# im_sensorless_control_analysis_and_design — sensorless IFOC of an induction machine

## Overview

| Item | Description |
|---|---|
| What it simulates | Sensorless indirect field-oriented control of a 250 kW / 400 V / 465 A Siemens induction motor: speed PI, dq current PIs, zero-sequence loop, Luenberger and double-integrator speed observers, 4-state Kalman filter. |
| Model | `im_inv_sensorless_ctrl_model.slx` |
| Init | `init_model.mlx` |
| Post-processing | `graphics.mlx` |
| Other | `im_param_calc_1/2.mlx`, `im_param_calc_3.m` (machine parameters from no-load and locked-rotor tests, results in `im_data_motor.txt`), `normalization.mlx`, `flux_obs/` (flux-observer study on a 440 kW SICME machine), `references/` (papers) |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.mlx` from this folder (opens the model). |
| 3 | Run the simulation. |
| 4 | Run `graphics.mlx`. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 6 s | model StopTime |
| `fPWM_INV` | 8 kHz | double update |
| `ts_inv` | 62.5 µs | control sample time |
| `tc` | 0.625 µs | `ts_inv/100`, solver step |
| `t_misura` | = `simlength` | logging window |
| `flux_obs/` | 4 kHz, `tc` = 2.5 µs | `init_flux_obs.mlx`; the StopTime of `flux_obs.slx` is fixed at 22 s in the model |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
