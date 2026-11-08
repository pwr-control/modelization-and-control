# sic_inverter_for_drone — 20 kW SiC inverter for a propeller drive

## Overview

| Item | Description |
|---|---|
| What it simulates | 20 kW class SiC inverter (Wolfspeed CAB016M12FM3, ALPHA LT13070 heatsink) on a 400 V DC link feeding a 62-pole PMSM with propeller load; sensorless FOC with MTPA, field weakening, I/F start-up, selectable observers. |
| Model | `powertrain_model.slx` |
| Init | `init_model.m` |
| Post-processing | `plotting_results.m`, `plotting_results_motor_load.m`, `Spectrum.m`, `analysis_cc.m` |
| Data | `pmsm_data/`, `module_data/`, `heatsink_data/`, `load_data/`, `propeller_data/`, `bemf_measure/` (measured back-EMF) |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.m` from this folder (opens the model). |
| 3 | Run the simulation. |
| 4 | Run the post-processing scripts. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 2.75 s | model StopTime |
| `fPWM_INV` | 25 kHz | single update, dead time 0.42 µs |
| `ts_inv` | 40 µs | control sample time |
| `tc` | 0.2 µs | `ts_inv/200` (the init comment says it is needed for solver stability) |
| `t_measure` | 0.1375 s | logging window (`simlength/20`) |
| `use_cc_ccaller`, `use_observer_ccaller` | 1 | current control and observer as C-Caller |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
