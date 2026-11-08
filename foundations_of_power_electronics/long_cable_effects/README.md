# long_cable_effects — AFE + inverter + PMSM with HF cable models

## Overview

| Item | Description |
|---|---|
| What it simulates | AFE + inverter + PMSM at 400 V with high-frequency models: lumped HF power cable (two sections), common-mode inductances; overvoltage at the motor terminals and common-mode effects, grid-connected and island mode. |
| Model | `afe_inv_psm_with_hf_models.slx` |
| Init | `init_model.m` (flat workspace) |
| Post-processing | `plotting_power_cables.m`, `plotting_results_afe.m`, `plotting_results_inv.m`, `plotting_results_dclink.m`, `sim_results_post_process.m`, `voltage_spectrum_analysis.m`, `voltage_islandmode_spectrum_analysis.m` |
| Data | `data/`: 25 measured oscilloscope traces (625 kHz) for comparison |

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
| `fpwm` | 4 kHz | double update |
| `ts` | 125 µs | control sample time |
| `tc_factor` | 1000 | `tc = ts/tc_factor` |
| `tc` | 0.125 µs | solver MaxStep: small, because of the HF models |
| `t_measure` | 0.3 s | logging window |
| `use_thermal_model` | 0 | ideal switches |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
