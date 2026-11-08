# single_phase_inverter — single-phase inverters

Single-phase inverters with dq vector PI or resonant PI control, virtual dq from SOGI / phase-shift filter / FHT, open-loop U/f mode, system identification, output transformer and harmonic filters. Each sub-folder has `init_model.m` (struct `param`), `init_model_legacy.m` (old flat init), the `*_param.slx` model migrated to `param` and the legacy model.

## Sub-projects

| Folder | Content |
|---|---|
| `two_levels/` | Single-phase H-bridge, 400 V. Model `single_phase_inverter_param.slx`. Post-processing: `plotting_results_sim_results_{50,80,400}Hz_10kHz_1.m`, `power_losses_calculation.m`, `spectrum.m`; `plant_estimator.m`, `virtual_phase_shift_filter_analysis.m`. |
| `three_levels/` | Single-phase T-type bridge, 690 V, MOSFET thermal model. Model `single_phase_ttype_inverter_param.slx`. Post-processing: `plotting_results_sim_results_{50,400}Hz_10kHz_1.m`, `power_losses_calculation.m`, `spectrum.m`. |

## Main simulation parameters

| Sub-project | Init | simlength | tc (solver step) | Control / PWM | Note |
|---|---|---|---|---|---|
| `two_levels` | `init_model.m` | 1.25 s | 0.5 µs | 10 kHz double update, ts = 50 µs | ideal switches |
| `three_levels` | `init_model.m` | 1.25 s | 0.625 µs | 4 kHz double update, ts = 125 µs | MOSFET thermal model on |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.

## Notes

- How to use: add the `library` repository to the MATLAB path, run `init_model.m` from the sub-folder (it fills `param` and opens the model), run the simulation, then the post-processing scripts.
