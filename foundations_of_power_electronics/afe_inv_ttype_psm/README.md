# afe_inv_ttype_psm — three-level T-type AFE + inverter + PMSM

## Overview

| Item | Description |
|---|---|
| What it simulates | AFE and inverter both built as three-level T-type bridges feeding a PMSM, 690 V, 250 kW. |
| Model | `afe_inv_ttype_psm.slx` |
| Init | `init_model.m` (flat workspace) |
| Post-processing | `plotting_results_afe.m`, `plotting_results_inv.m`, `plotting_results_dclink.m`, `plotting_results_igbt.m`, `voltage_spectrum_analysis.m` |

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
| `simlength` | 1.75 s | model StopTime |
| `fpwm` | 5 kHz | AFE and inverter, double update |
| `ts` | 100 µs | control sample time |
| `tc_factor` | 200 | `tc = ts/tc_factor` |
| `tc` | 0.5 µs | solver MaxStep (`glb_time.tc`) |
| `use_thermal_model` | 0 | ideal switches |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
