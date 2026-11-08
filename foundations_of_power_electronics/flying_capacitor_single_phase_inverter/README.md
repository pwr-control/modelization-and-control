# flying_capacitor_single_phase_inverter — single-phase 5-level flying-capacitor inverter

## Overview

| Item | Description |
|---|---|
| What it simulates | Single-phase 5-level flying-capacitor inverter from 3000 V DC, phase-shifted carriers; fundamental model and model with parasitics and thermal network. |
| Models | `single_phase_multilevel.slx` (with parasitics and thermal network), `single_phase_multilevel_sm.slx` (simplified) |
| Init | `init_model.mlx` |
| Post-processing | `plotting_results.m`, `plotting_results_hf.m` (high-frequency), `plotting_modulation.m` |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Choose the study in `init_model.mlx` (`enable_dclink_precharge`). |
| 3 | Run `init_model.mlx` from this folder. |
| 4 | Open the model and run it. |
| 5 | Run the post-processing scripts. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `enable_dclink_precharge` | 0 | 0 = high-frequency analysis, 1 = DC-link precharge sequence |
| `simlength` | 1.25 s / 3.75 s | HF analysis / precharge |
| `fPWM` | 5 kHz | `2 × 2.5 kHz` |
| `ts` | 200 µs | control sample time |
| `tc` | 80 ns / 0.8 µs | `ts/2500` (HF analysis) / `ts/250` (precharge) |
| `t_misura` | 0.2 s / = `simlength` | logging window |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
