# afe_inv_loopback — AFE + inverter in loopback (test bench)

## Overview

| Item | Description |
|---|---|
| What it simulates | AFE + inverter in loopback configuration: 690 V grid through a 1.6 MVA Dyn11 emulator, DC link 1070 V, 250 kW PMSM at 17.8 rpm in generator mode; LVRT/FRT test catalogue. |
| Model | `afe_inv_loopback.slx` |
| Init | `init_model.m` (flat workspace) |
| Post-processing | `plotting_results_afe.m`, `plotting_results_inv.m`, `plotting_results_dclink.m`, `voltage_spectrum_analysis.m` |
| Other | `grid_fault_generator.m`, `asymmetric_error.m` (FRT tests), `grid_emulator.m` |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.m` from this folder (opens the model). |
| 3 | Run the simulation. |
| 4 | Run the post-processing scripts. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 2 s | model StopTime |
| `fPWM_AFE` = `fPWM_INV` | 4 kHz | double update |
| `ts_afe` = `ts_inv` | 125 µs | control sample time |
| `tc` | 1.25 µs | `ts_afe/100`, solver step |
| `t_misura` | 0.25 s | logging window at `tc` (`Nc`) |
| `use_thermal_model` | 1 | IGBT thermal model on |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
