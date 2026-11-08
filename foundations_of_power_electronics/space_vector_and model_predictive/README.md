# space_vector_and model_predictive — space vector modulation theory

Theoretical investigation of space vector modulation (Geyer, Rodriguez). Despite the folder name, no model predictive control code is here: MPC is in `mpc_psm`, `mpc_ttype_psm` and `psm_sensorless_ctrl`.

## Sub-projects

| Folder | Content |
|---|---|
| root | `init_model.mlx` + `modulator.slx`: modulator bench |
| `two_level_sv/` | `sec.m`: two-level sector index from the signs of the phase voltages |
| `three_level_sv/analysis/` | `init.mlx` + `three_level_converters.slx`, `three_level_sv.slx`, `three_level_npc_pwm.slx`, `three_level_ttype_pwm.slx`: NPC and T-type legs driven by the Simulink 3-level PWM or by the hand-built SV modulator; `tests_sv/` MATLAB reference of the three-level algorithm |
| `three_level_sv/model/` | `init_model.mlx` + `afe_abc_inv_psm.slx`: grid-tied AFE + inverter + PMSM at 690 V using the modulator, with FRT/LVRT logic |
| `pwm_modulators/`, `three_level_sv/mapping/`, `derivation/` | derivations and space-vector maps (`maps.nb`, Mathematica) |

## Main simulation parameters

| Sub-project | Init | simlength | tc (solver step) | Control / PWM | Note |
|---|---|---|---|---|---|
| root (`modulator`) | `init_model.mlx` | 0.02 s | 0.417 µs | 12 kHz double update, ts = 41.7 µs |  |
| `three_level_sv/analysis` | `init.mlx` | 0.25 s | 1 µs | 5 kHz double update, ts = 100 µs | `t_misura` = `simlength/5` |
| `three_level_sv/model` | `init_model.mlx` | 2 s | 1.25 µs | AFE 4 kHz (ts = 125 µs), inverter 2.5 kHz |  |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.

## Notes

- How to use: add the `library` repository to the MATLAB path, run the init of the sub-folder (it opens the model), then run the simulation.
