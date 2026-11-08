# inv_im — inverter + induction machine

## Overview

| Item | Description |
|---|---|
| What it simulates | Two-level inverter + induction machine (ABB M3BP 355MLB 6, 261 kW) with dq PI current control, MTPA, torque-curve mode, EKF of the machine (sensorless by default: starts on the encoder and switches to the EKF). |
| Model | `inv_im_param.slx` |
| Init | `init_model.m` (struct `param`) |
| Post-processing | `plotting_results_inv.m`, `plotting_results_dclink.m`, `power_losses_calculus.m`, `spectrum.m` |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.m` from this folder (fills `param`, opens the model). |
| 3 | Run the simulation. |
| 4 | Run the post-processing scripts. |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `param.sim.simlength` | 10 s | model StopTime: long run |
| `param.sim.tc` | 0.625 µs | MaxStep of ode23tb and logging base |
| `param.time.fpwm` | 4 kHz | double update |
| `ts` | 125 µs | control sample time |
| `param.time.tc_decimation` | 10 | logging decimation at `tc` |
| `param.sim.use_thermal_model` | 0 | ideal switches; IGBT/MOSFET thermal bridges selectable |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
