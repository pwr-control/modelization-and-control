# Modelization and Control

A personal collection of multi-domain physical models and control applications built with
MATLAB, Simulink and Simscape. The repository covers power electronics, power systems,
semiconductor devices, analog electronics, magnetics, mechanics, cooling systems and a few
foundational topics (mathematics, quantum mechanics, control theory).

Most projects are complete simulation benches rather than isolated examples: a converter or a
machine modelled in Simscape, the full control chain (modulator, current and voltage loops,
PLL, observers), and post-processing scripts that extract waveforms, spectra, losses and
temperatures.

## Prerequisites

- MATLAB with Simulink, Simscape and Simscape Electrical.
- The companion **`library`** repository, added to the MATLAB path **with subfolders**. It
  provides:
  - `library/foundation` — the custom Simscape (`.ssc`) components used by the models
    (IGBT and MOSFET loss and switching-dynamics models, reverse-recovery diodes, thyristors,
    machines, transformers, batteries, heatsinks), compiled with `ssc_build`;
  - `library/user_defined_functions/ccaller` — the C functions called from the models through
    C-Caller blocks (current controllers, observers, PLLs, space-vector modulators, state
    machines). These let the same control code run in simulation and on the target;
  - the setup functions used by the initialization scripts (`init_environment`, `timing_setup`,
    `*_hwdata`, `psm_calculus`, `im_calculus`, `ekf_*_setup`, `grid_three_phase_emulator`,
    `lithium_ion_battery_setup`, `device_igbt_setup`, ...).

Documentation of the shared components lives in the **`library`** repository.

## How to use a project

Each project folder follows the same pattern:

1. `init_model.m` (or `init_model.mlx`) — sets every parameter in the base workspace and opens
   the model. Run it first; nothing else works without it.
2. `<model_name>.slx` — the Simulink/Simscape model. Variants of the same power stage
   (ideal switch / IGBT with thermal model / MOSFET with thermal model) are usually selected by
   flags set in the init script.
3. `plotting_results*.m`, `spectrum*.m`, `power_loss*.m` — post-processing of the logged
   results (`sim_results*.mat`): waveforms, FFT, device losses, junction and heatsink
   temperatures. Figures are generally exported as EPS with LaTeX labels.

## Repository layout

| Folder | Content |
|---|---|
| [`foundations_of_power_electronics`](foundations_of_power_electronics) | Converters and drives: AFE, inverters, DAB, CLLC, ZVS/ZCS DC-DC, flying capacitor, thyristor rectifiers, Vienna rectifier, PMSM and induction machine drives, MPC, PLLs |
| [`foundations_of_power_systems`](foundations_of_power_systems) | Grid-side studies: IEC power measurement, weak grid, symmetrical components, graph theory of networks |
| [`foundations_of_semiconductor_devices`](foundations_of_semiconductor_devices) | Switching-dynamics models of IGBT and MOSFET: double pulse test, buck leg, AFE leg, losses and thermal behaviour |
| [`foundations_of_electronics`](foundations_of_electronics) | Analog circuits of a converter control board: gate drivers, measurement chains, comparators, level shifters, deadtime generators, auxiliary supplies, PLL |
| [`foundations_of_magnetics`](foundations_of_magnetics) | Magnetics for power electronics: FEMM inductor, HF transformer design literature, Paschen curve, moving-coil actuator |
| [`foundations_of_mechanics`](foundations_of_mechanics) | Beams, strings, flexible shafts, PDE/ODE coupled problems and their control (FEM, finite differences, LQR) |
| [`foundations_of_cooling_systems`](foundations_of_cooling_systems) | Cold plate sizing and a liquid cooling loop |
| [`foundations_of_control_systems`](foundations_of_control_systems) | Control theory examples |
| [`foundations_of_mathematics`](foundations_of_mathematics) | Numerical experiments (Lorenz system, Lyapunov exponent) |
| [`foundations_of_qm`](foundations_of_qm) | 1D Schrödinger equation solved by finite differences |

Each folder has its own README with a project-by-project description.

## Conventions

- Application voltages recur across projects: 400 V, 480 V, 690 V for the AC side; 3000 V DC
  for the rail/catenary applications.
- Control is sampled at the PWM rate, usually with double update (`ts = 1/(2*fPWM)`); the
  Simscape solver step is a fraction of `ts` set by `tc_factor`.
- Where a control block exists both as a Simulink subsystem and as C-Caller code, a flag in
  `init_model.m` (`use_*_ccaller`) selects which one is active.
- Loss and thermal models are optional and switched by `use_thermal_model`.
