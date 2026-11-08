# distributed_ctrl — AFE + inverter + PMSM with distributed control

## Overview

| Item | Description |
|---|---|
| What it simulates | Same plant as `afe_inv_psm` with the control split over two controllers and an explicit two-sample communication delay; full grid-support layer (FRT, VDE reactive current) and machine-side observers. |
| Model | `afe_rst_inv_psm_distributed_control_two_delays.slx` |
| Init | `init_model.mlx` |
| Post-processing | `plotting_results_afe.m`, `plotting_results_inv.m`, `voltage_spectrum_analysis.m` |
| Other | `grid_fault_generator.m`, `asymmetric_error.m`, `grid_emulator.m` |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.mlx` from this folder (opens the model). |
| 3 | Run the simulation. |
| 4 | Run the post-processing scripts. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 3.25 s | model StopTime |
| `fPWM_AFE` = `fPWM_INV` | 4 kHz | double update, no dead time |
| `ts_afe` = `ts_inv` | 125 µs | control sample time |
| `tc` | 1.25 µs | `ts_afe/100`, solver step |
| `t_misura` | = `simlength` | logging window |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
