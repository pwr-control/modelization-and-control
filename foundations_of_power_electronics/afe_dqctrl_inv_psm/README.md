# afe_dqctrl_inv_psm — back-to-back AFE + inverter + PMSM (dq-control variant)

## Overview

| Item | Description |
|---|---|
| What it simulates | Variant of `afe_inv_psm`: three-phase AFE + DC link + inverter + PMSM wind generator, 690 V, 250 kW per stage, dq control, FHT dq-PLL, MTPA, sensorless observers, thermal model. |
| Model | `afe_dqctrl_inv_psm.slx` |
| Init | `init_model.m` (flat workspace) |
| Post-processing | `plotting_results_afe.m`, `plotting_results_inv.m`, `plotting_results_dclink.m`, `sim_results_post_process.m`, `voltage_spectrum_analysis.m`, `voltage_islandmode_spectrum_analysis.m` |
| Other | `calcolo_filtro_LRC_400V.mlx` (LRC grid filter sizing), `power_loss/` (loss data set) |

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
| `tc` | 0.25 µs | solver MaxStep (`glb_time.tc`) |
| `fpwm` | 4 kHz | AFE and inverter, double update |
| `ts` | 125 µs | control sample time |
| `t_measure` | = `simlength` | logging window |
| `use_thermal_model` | 1 | IGBT thermal model on |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
