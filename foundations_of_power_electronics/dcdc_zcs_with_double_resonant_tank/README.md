# dcdc_zcs_with_double_resonant_tank — half-bridge ZCS DC-DC with CLL double resonant tank

## Overview

| Item | Description |
|---|---|
| What it simulates | Unidirectional isolated DC-DC: half bridge with CLL double resonant tank, zero-current switching at variable frequency (up to 48 kHz), 10 V / 100 A output; cascaded voltage/current PI plus even-harmonic (2nd–14th) compensator bank. |
| Model | `cll_zcs_ver_v.slx` |
| Init | `init_zcs_model_ctrl.m` (also Bode-plots the resonant tank) |
| Post-processing | `plot_results.m` (with/without ripple compensation), `spectrum_analysis.m` |
| Other | `grid_emulator.m` |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_zcs_model_ctrl.m` from this folder (opens the model). |
| 3 | Run the simulation. |
| 4 | Run `plot_results.m`. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 10 s | model StopTime: long run |
| `fPWM_ZCS_MAX` | 48 kHz | maximum switching frequency |
| `ts` | 100 µs | control sample time |
| `tc` | 0.1 µs | simulation step |
| `t_misura_tc` | 0.1 s | logging window at `tc` |
| `t_misura_ts` | = `simlength` | logging window at `ts` |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
