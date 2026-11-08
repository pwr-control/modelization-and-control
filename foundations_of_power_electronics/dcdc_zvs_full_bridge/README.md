# dcdc_zvs_full_bridge — isolated battery-to-battery ZVS full bridge

## Overview

| Item | Description |
|---|---|
| What it simulates | Isolated ZVS full bridge, 175 kW, 750 V → 540 V; power flow by phase shift between the primary legs, diode rectifier at the secondary; thermal model. |
| Model | `dcdc_zvs_full_bridge.slx` |
| Init | `init_model.m` (also derives the ZVS current constraint) |
| Post-processing | `power_loss_calculus.m`, `spectrum.m`; results in `figures_case_scenario_1..6/` |
| Other | `run_sim_output_current_ctrl.m` (sweep of six scenarios), `magnetics_design_transformer.m` (HF transformer design), `grid_emulator.m` |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.m` from this folder (opens the model). |
| 3 | Run the simulation, or `run_sim_output_current_ctrl.m` for the six scenarios. |
| 4 | Run the post-processing scripts. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 1.5 s | model StopTime |
| `fPWM` | 5 kHz | single update (`TRGO_double_update = 0`) |
| `ts_dab` | 200 µs | control sample time |
| `tc` | 0.5 µs | `ts_dab/400`, solver step |
| `t_misura` | 0.6 s | logging window |
| `use_thermal_model` | 1 | thermal model on |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
