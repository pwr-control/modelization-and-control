# theory_analysis_dab — dual active bridge analysis

Two batteries connected through a DAB, for the analysis of modulation strategies and efficiency with detailed MOSFET models. Each sub-project has its own README.

## Sub-projects

| Folder | Content |
|---|---|
| [`single_phase_dab/`](single_phase_dab/README.md) | 250 kW single-phase DAB, phase-shift modulator, cascaded output voltage and current PI, ZVS post-processing |
| [`three_phase_dab/`](three_phase_dab/README.md) | 750 kW three-phase DAB (De Doncker) with three-phase HF transformer and MOSFET thermal model |
| `literature/` | references (Akagi, De Doncker, Kheraluwala, Tao, Krismer) |
| `components/` | device datasheets |

## Main simulation parameters

| Sub-project | Init | simlength | tc (solver step) | Control / PWM | Note |
|---|---|---|---|---|---|
| `single_phase_dab` | `init_model.m` | 2 s | 0.625 µs | fpwm_dab 12 kHz, double update | logging over the whole run |
| `three_phase_dab` | `init_model.m` | 0.3 s | 0.1 µs | see its README | `t_measure` = last 20 ms; `bench_case` selects the operating case |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.
