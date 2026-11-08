# dab_inv_single_phase_6kVA — 6 kVA UPS: battery → DAB → single-phase inverter

## Overview

| Item | Description |
|---|---|
| What it simulates | UPS for emergency lighting: lithium-ion battery → DAB → single-phase inverter, 230 V / 30 A output, 8 kVA / 25 kHz HF transformer, cascaded PI on the DAB, PI + resonant on the inverter, PLL, Kalman filters (50 Hz harmonic, battery SOC). |
| Models | `battery_dab_inv_single_phase.slx` (full system), `inv_single_phase.slx` (inverter only), `modulator.slx` (modulator bench) |
| Init | `init_model.m` |
| Post-processing | `sim_results_mosfet/`, `spectrum.m` |
| Other | `scaling.m` (ADC scaling constants) |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.m` from this folder. |
| 3 | Open `battery_dab_inv_single_phase.slx` and run it. |
| 4 | Run the scripts in `sim_results_mosfet/` and `spectrum.m`. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 3 s | model StopTime |
| `fPWM_INV` = `fPWM_ZVS` | 35 kHz |  |
| `ts_inv` | 28.6 µs | inverter control sample time |
| `ts_zvs` | 57.1 µs | DAB control sample time (`2·tPWM`) |
| `tc` | 0.143 µs | `ts_inv/200`, solver step |
| `t_misura` | 0.6 s | logging window (`simlength/5`) |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
