# psm_sensorless_ctrl — simplified PMSM models for sensorless control design

## Overview

| Item | Description |
|---|---|
| What it simulates | Simplified PMSM models (subsystems of a 1.5 MW direct-drive generator at 17.8 rpm) for fast control design, in four variants: space vector or one-step MPC, back-EMF observer or EKF. |
| Models | `psm_sv_bemf_ctrl.slx`, `psm_sv_ekf_bemf_ctrl.slx` (default), `psm_mpc_bemf_ctrl.slx`, `psm_mpc_ekf_bemf_ctrl.slx` |
| Init | `init_model.m` (the last `model = ...` line selects the model) |
| Post-processing | `plotting_results_inv.m`, `power_losses_calculus.m`, `spectrum.m` |
| Other | `kalman_psm.m` (6-state EKF), `grid_emulator.m` |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Select the model in `init_model.m` (lines 13–16). |
| 3 | Run `init_model.m` from this folder (opens the model). |
| 4 | Run the simulation. |
| 5 | Run the post-processing scripts. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 2 s | model StopTime |
| `fPWM_INV` | 5 kHz | `2 × 2.5 kHz`, single update |
| `ts_inv` | 200 µs | control sample time |
| `tc` | 2 µs | `ts/100`, solver step |
| `t_misura` | 5 electrical periods | logging window at `tc` |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
