# modular_multiphase — modular multiphase inverter + induction machine

## Overview

| Item | Description |
|---|---|
| What it simulates | Multiphase / modular inverter feeding a 250 kW induction machine (GM 355L 6); rotor-flux vector control with explicit zero-sequence current loop, per-module current tracking. |
| Model | `im_inv_model_mmp.slx` |
| Init | `init_model.mlx` |
| Post-processing | `graphics.mlx` |
| Other | `im_param_calc_1/2.mlx` (machine parameters, `im_data_motor.txt`), `normalization.mlx` |

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
| `fPWM_INV` | 10 kHz | double update |
| `ts_inv` | 50 µs | control sample time |
| `tc` | 0.5 µs | `ts_inv/100`, solver step |
| `t_misura` | = `simlength` | logging window |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
