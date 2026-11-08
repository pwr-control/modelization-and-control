# thyristor_based_models — line-commutated thyristor rectifiers

Line-commutated rectifiers and their firing mechanism, also in C code (C-Caller). `readme.mlx` explains the file naming.

## Sub-projects

| Folder | Content |
|---|---|
| `six_pulse_thyristor_rectifier/` | `sptr.slx`: six-pulse bridge, 400 V mains, 50 kVA transformer, 340 V / 90 A DC, LC filter with tuned traps; cascaded voltage/current PI, dq-PLL, harmonic observer (`hobs_test.slx` tests it alone) |
| [`six_pulses_thyristor_based_rectifier/`](six_pulses_thyristor_based_rectifier/README.md) | six-pulse bridge of `thyristor_rr` thyristors (charge model), 400 V, 100 kVA Dyn11; struct `param` (see its README) |
| `twelve_pulse_thyristor_rectifier/` | twelve-pulse bridge on a three-winding transformer for a 775 V traction bus from 20 kV / 1250 kVA: `pm/` detailed, `pm_reduced_model/` reduced, `sm/` simplified + Simscape firing circuit |
| `dual_converter_thyristor_based/` | `dual_thyristor_bridge_rectifier.slx`: two six-pulse bridges for four-quadrant DC output |

## Main simulation parameters

| Sub-project | Init | simlength | tc (solver step) | Control / PWM | Note |
|---|---|---|---|---|---|
| `six_pulse_thyristor_rectifier` | `init_model.m` | 5 s | 10 µs | control 5 kHz (200 µs) |  |
| `six_pulses_thyristor_based_rectifier` | `init_model.m` | 1.2 s | 1 µs | 5 kHz (200 µs) | `t_measure` 20 ms |
| `twelve_pulse/pm` | `init_model.mlx` | 8 s | 1 µs | control 8 kHz (125 µs) | `tc = ts_ctrl/125` |
| `twelve_pulse/pm_reduced_model` | `init_model.m` | 5 s | 10 µs | 5 kHz |  |
| `twelve_pulse/sm` | `init_model.m` | 5 s | 10 µs | 5 kHz |  |
| `dual_converter_thyristor_based` | `init_model.mlx` | 8 s | 1 µs | control 8 kHz (125 µs) |  |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.

## Notes

- How to use: add the `library` repository to the MATLAB path, run the init of the sub-folder (it opens the model), then run the simulation.
