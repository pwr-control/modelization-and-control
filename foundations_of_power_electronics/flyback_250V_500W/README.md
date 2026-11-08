# flyback_250V_500W — 500 W flyback from a 1070 V DC link

## Overview

| Item | Description |
|---|---|
| What it simulates | Flyback converter, 500 W at 250 V from a 1070 V DC link, in Simscape with the magnetic (flux) domain; coupled-inductor design in the init, DC-link parasitics and RC snubber. |
| Model | `flyback.slx` |
| Init | `init_model.mlx` |
| Post-processing | `plotting_results_inv.m` |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.mlx` from this folder. |
| 3 | Open `flyback.slx` and run it. |
| 4 | Run `plotting_results_inv.m`. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 0.2 s | model StopTime |
| `fPWM` | 100 kHz |  |
| `ts` | 5 µs | control sample time |
| `tc` | 25 ns | `ts/200`, solver step |
| `t_misura` | = `simlength` | logging window |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
