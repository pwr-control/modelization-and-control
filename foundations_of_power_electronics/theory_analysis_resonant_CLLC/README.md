# theory_analysis_resonant_CLLC — resonant CLLC converter analysis

Two batteries connected through a CLLC resonant converter, 250 kW, for modulation and efficiency analysis; three bridge implementations (ideal switch, MOSFET with thermal model, MOSFET ZVS variant).

## Sub-projects

| Folder | Content |
|---|---|
| `resonant_cllc/` | Model `single_phase_cllc_param.slx` (legacy `single_phase_cllc.slx`), init `init_model.m` (struct `param`; `init_model_legacy.m` kept). Post-processing: `plotting_sim_results*.m`, `power_loss_calculus.m`, `spectrum.m`. Design: `cllc_analysis.m` (resonant tank transfer function), `transformer_sizing.m` (380 kVA, 150 kHz HF transformer). |
| `cllc_modulator_analysis/` | `CLLC_modulator_analysis.slx`: modulator studied alone |
| `doc/` | `transformer_losses_sweep.m` (core and winding losses 1–200 kHz), note on DAB/CLLC inductance, internship report |

## Main simulation parameters

| Sub-project | Init | simlength | tc (solver step) | Control / PWM | Note |
|---|---|---|---|---|---|
| `resonant_cllc` | `init_model.m` | 0.25 s | 0.625 µs | fpwm_cllc 12 kHz (3 × 4 kHz) | MOSFET thermal model and ZVS snubber on |
| `cllc_modulator_analysis` | none | 0.5 ms (fixed in the model) | auto |  |  |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.

## Notes

- How to use: add the `library` repository to the MATLAB path, run `init_model.m` from `resonant_cllc/` (it fills `param` and opens the model), then run the simulation and the post-processing scripts.
