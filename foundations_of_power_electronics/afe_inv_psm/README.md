# afe_inv_psm — back-to-back AFE + inverter + PMSM (reference system)

## Overview

| Item | Description |
|---|---|
| What it simulates | Reference back-to-back system: three-phase AFE + DC link + inverter + PMSM wind generator, 690 V, 250 kW per stage; resonant PI, DC-link PI, FHT dq-PLL, MTPA, sensorless observers (several blocks as C-Caller); thermal model (FF1200R17IP5). |
| Model | `afe_inv_psm_param.slx` |
| Init | `init_model.m` (struct `param`) |
| Post-processing | `plotting_results_afe.m`, `plotting_results_inv.m`, `plotting_results_dclink.m`, `sim_results_post_process.m`, `voltage_spectrum_analysis.m`, `voltage_islandmode_spectrum_analysis.m` |
| Other | `power_loss/` (loss data set of an FF650R17IE4), `non_virtual_blocks_counting.m` |

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
| `param.sim.tc` | 0.25 µs | fixed step and logging base |
| `param.time.fpwm` | 4 kHz | double update |
| `ts` | 125 µs | control sample time |
| `param.time.t_measure` | = `simlength` | logging window |
| `param.sim.use_thermal_model` | 1 | IGBT thermal model on (5 nonlinear iterations) |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
