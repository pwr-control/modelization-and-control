# zvs_inv_single_phase_6kVA — 6 kVA single-phase UPS with ZVS DC-DC

## Overview

| Item | Description |
|---|---|
| What it simulates | 6 kVA UPS: lithium-ion battery → phase-shifted full bridge (ZVS) → single-phase inverter, 230 V / 30 A output, 400 V DC link; 8 kVA / 25 kHz HF transformer, SiC devices; dq or resonant PI inverter control, PLL, Kalman observers, battery SOC estimator. |
| Model | `battery_phfb_inv_single_phase.slx` |
| Init | `init_model.m` |
| Post-processing | `sim_results_mosfet/`, `sim_results_thermal/`, `sim_results_grid_back/` (three simulated cases), `spectrum.m` |
| Docs | `ups_single_phase_6kVA.pdf` |
| Other | `scaling.m` (ADC scaling constants) |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.m` from this folder. |
| 3 | Open `battery_phfb_inv_single_phase.slx` and run it. |
| 4 | Run the scripts of the case in `sim_results_*/`. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 3 s | model StopTime |
| `fPWM_INV` = `fPWM_ZVS` | 20 kHz |  |
| `ts_inv` | 50 µs | inverter control sample time |
| `ts_zvs` | 100 µs | DC-DC control sample time |
| `tc` | 0.25 µs | `ts_inv/200`, solver step |
| `t_misura` | 0.6 s | logging window (`simlength/5`) |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
