# grid_forming — grid-forming converter

Grid-forming studies in the dq frame.

## Sub-projects

| Folder | Content |
|---|---|
| `single_phase_load/` | `grid_forming_dq.slx`: three-phase bridge with LCL filter, battery on the DC link (250 kW), Dyg transformer (1 MVA, 400 V) and RL load on the formed grid; resonant PI control; IGBT thermal model. Init `init_model.m` (struct `param`, functions in `private/`). Docs: `grid_forming_ctrl_ver_ii.pdf`, `trafo.pdf`; `claude_analysis/rpi_analysis.py` (resonant PI check). |

## Main simulation parameters

| Sub-project | Init | simlength | tc (solver step) | Control / PWM | Note |
|---|---|---|---|---|---|
| `single_phase_load` | `init_model.m` | 1 s | 0.625 µs | 4 kHz double update, ts = 125 µs | `t_measure` 0.4 s, thermal model on |

> **Run time.** `tc` is the step of the power-stage solver (FixedStep / MaxStep) and the logging base: halving `tc` roughly doubles the run time and the logged data (`Nc = t_measure / tc` points). `simlength` scales the run time linearly.

## Notes

- How to use: add the `library` repository to the MATLAB path, run `init_model.m` from `single_phase_load/` (it opens the model), then run the simulation.
