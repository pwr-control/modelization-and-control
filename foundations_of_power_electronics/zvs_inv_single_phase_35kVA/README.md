# zvs_inv_single_phase_35kVA — 35 kVA single-phase UPS, two parallel modules

## Overview

| Item | Description |
|---|---|
| What it simulates | Two parallel modules: ZVS phase-shifted DC-DC at 20 kHz feeding a single-phase H-bridge at 10 kHz, 500 V / 55 A output, 820 V DC link; Kalman observer, load estimator, PLL. |
| Models | `two_parallel_full_model_simscape_ver_ii.slx` (opened by the init), `two_parallel_full_model_simscape.slx`, `two_parallel_half_model_simscape(_ver_ii).slx`, `check_observer.slx` (observer/PLL bench, 0.2 s) |
| Init | `init_model.m` |
| Post-processing | `spectrum.m` |
| Other | `scaling.m` (ADC scale and offset for the firmware) |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.m` from this folder (opens the full model ver. II). |
| 3 | Run the simulation. |
| 4 | Run `spectrum.m`. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 3.2 s | model StopTime |
| `fPWM_INV` | 10 kHz | inverter |
| `fPWM_ZVS` | 20 kHz | ZVS DC-DC |
| `ts_inv` = `ts_zvs` | 100 µs | control sample times |
| `tc` | 0.5 µs | `ts_inv/200`, solver step |
| `t_misura` | = `simlength` | logging window |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
