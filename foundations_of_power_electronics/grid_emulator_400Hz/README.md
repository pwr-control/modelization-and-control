# grid_emulator_400Hz — 400 Hz three-phase grid emulator

## Overview

| Item | Description |
|---|---|
| What it simulates | 400 Hz three-phase power supply / grid emulator: 90 kW, 200 V line, battery-fed 320 V DC link, LCL filter into a 90 kVA 400 Hz output transformer; dq PI current and voltage loops, oscillator Kalman filter. |
| Model | `ps_model.slx` |
| Init | `init_model.mlx` |
| Post-processing | `plotting_results/`, `spectrum_analysis/` (FFT of voltage and currents) |
| Other | `grid_emulator.m` |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.mlx` from this folder (opens the model). |
| 3 | Run the simulation. |
| 4 | Run the scripts in `plotting_results/` and `spectrum_analysis/`. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 1 s | model StopTime |
| `fPWM_INV` | 8 kHz | single update |
| `ts_inv` | 125 µs | control sample time |
| `tc` | 0.625 µs | `ts_inv/200`, solver step |
| `t_measure` | = `simlength` | logging window |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
