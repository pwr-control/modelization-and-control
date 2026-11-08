# ac_ac_buck_converter — direct AC/AC half-bridge buck chopper

## Overview

| Item | Description |
|---|---|
| What it simulates | Direct AC/AC buck chopper (no DC link) on a 230 V / 50 Hz single-phase grid, built with bidirectional switches (two anti-series IGBTs); overlapping-deadtime modulation. |
| Models | `acac_half_bridge_buck.slx` (standard arrangement), `acac_half_bridge_buck_common_drain.slx` (common-drain arrangement) |
| Init | `init_model.mlx` |
| Post-processing | `plotting_results.mlx` (input/output voltage and current) |
| Docs | `images/modulator_strategy.pdf` (modulation strategy) |
| Control | open loop |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.mlx` from this folder. |
| 3 | Open one of the two models and run it. |
| 4 | Run `plotting_results.mlx`. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 1 s | model StopTime |
| `fPWM` | 6 kHz |  |
| `ts` | 83.3 µs | `1/fPWM/2` |
| `tc` | 0.208 µs | `ts/400`, model FixedStep |
| `t_misura` | 0.975 s | logging window (`simlength − 25 ms`) |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
