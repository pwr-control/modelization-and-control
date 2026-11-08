# diode_based_models — six-pulse diode rectifier

## Overview

| Item | Description |
|---|---|
| What it simulates | Six-pulse diode (D-stack) rectifier, 400 V / 50 kVA transformer to 200 V secondary, 340 V / 90 A DC output, LC filter with 100/150/250/350 Hz traps; harmonic observer, RMS calculator and dq-PLL in C-Caller. |
| Model | `six_pulse_dstack_based_rectifier.slx` |
| Init | `init_model.m` |

## How to use

| Step | Action |
|---|---|
| 1 | Add the `library` repository to the MATLAB path (the init scripts look for `C:\github\library`). |
| 2 | Run `init_model.m` from this folder. |
| 3 | Open `six_pulse_dstack_based_rectifier.slx` and run it (see the note below). |

## Main simulation parameters

| Parameter | Value | Note |
|---|---|---|
| `simlength` | 1.2 s | model StopTime |
| `fPWM_RECT` | 5 kHz | control sampling base |
| `Tcontrol_rect` (`ts`) | 200 µs | control sample time |
| `Tc` = `Ts` | 50 ns | `Tcontrol_rect/4000`, solver step |
| `t_misura` | 20 ms | logging window |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.

## Notes

- `init_model.m` opens `six_pulse_diode_based_rectifier`, while the model file is `six_pulse_dstack_based_rectifier.slx`: the names must be aligned before the init runs to the end.
