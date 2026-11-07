# single_phase_dqPLLs_analysis — three single-phase PLLs on one test voltage

The model `single_phase_pll_analysis.slx` is a test bench for single-phase grid synchronization. There is
no power stage: the block `test_signal` gives one voltage in pu of the nominal peak, with sags, phase jumps,
frequency steps, DC offset and harmonics, together with the angle `theta_g` and the pulsation `omega_g` of
its fundamental, so the phase error of every PLL is known exactly. Three phase locked loops read the same
voltage, all discrete at the control period ts = 125 µs:

- **Park PLL** (block `single_phase_pll`, from the single-phase inverter controls): quadrature signal
  generator made of a direct and an inverse Park transform with two low-pass filters (a SOGI with gain
  0.5 in disguise), PI on `eta` with the nominal frequency as feed-forward, double-integrator observer;
- **FHT-PLL** (from the AFE control): first harmonic tracker as quadrature generator, two more trackers and
  the positive-sequence calculator (stage A), then the loop of the library (stage B: PI, phase integrator,
  observer);
- **DDSRF-PLL**: the same tracker, then the decoupled double synchronous reference frame of the
  three-phase AFE control (the block of `frt_dqpll`).

The init was rewritten in November 2026 as a `param` struct with the functions of `private/`, the model
was re-pointed to the new names, the sources of the 2021 test were replaced by the block `test_signal`
with six test cases, the logging was added and the dead custom C code settings were removed (see
[Changes to the model](#changes-to-the-model-november-2026)).

**Design note:** [`repos_doc/single_phase_lock_loop`](https://github.com/pwr-control/repositories-documentation/tree/main/repos_doc/single_phase_lock_loop)
of pwr-control/repositories-documentation, rev. 01 (November 2026): the quadrature generators compared
through their complex gain, the three PLLs as the blocks compute them, their linear models checked against
the simulation, the results of the six test cases, the criticalities and the changes, tested. Its Python
scripts (`py/make_figures.py`, `py/sp_pll_models.py`) read the file saved by `run_test_cases.m`.

| File | Content |
|---|---|
| `init_model.m` | initialization (plain-text live script): every number in the struct `param`, the design check of the three loops, then it opens the model, closes the scopes and checks the `param.*` references and the From/Goto tags. Run it **from this folder**. |
| `single_phase_pll_analysis_private.m` | handles to the functions of `private/` (`prv.<name>`): the Live Editor runs a live script from a copy outside this folder and does not see `private/` |
| `private/` | the functions of this model (table below) |
| `single_phase_pll_analysis.slx` | the model: `test_signal`, the three PLLs, the comparison scopes, `data_logging` |
| `run_test_cases.m` | simulates the test cases (`run_test_cases(1:6)`), prints one table of errors per case and returns the logs with the metrics; `run_test_cases(1:6, 'sim_results_cases.mat')` also saves them for `plotting_results_plls.m` and for the scripts of the note |
| `plotting_results_plls.m` | input, phase errors and frequencies of the three PLLs: after a run (`plotting_results_plls`) or from a saved file (`plotting_results_plls('sim_results_cases.mat', [1 3])`) |
| `init_model_legacy.m` | the old init (the whole template of the AFE and inverter models), kept for reference: it is not needed any more |
| `plant_estimator.m`, `power_losses_calculation.m`, `spectrum.m`, `virtual_phase_shift_filter_analysis.m`, `test_phase_shift_filter.slx` | files of the 2021 template, not used by this model (the scripts read signals of other models); unchanged |

## The model

- `Digital Clock` and `test_signal` (MATLAB Function, mask parameter `p` = `param.test`), sample time
  `param.glb_time.ts_inv`:
  `u = A(t) ( cos(phi + theta0) + sum_k harm_amp(k) sin(harm_order(k) phi + harm_phase(k)) ) + dc`,
  with `phi = omega_nom t`, the frequency step and the phase jump; `theta_g` = `phi + theta0` wrapped to
  (-pi, pi] (cosine convention: the angle the three loops lock on), `omega_g` in pu.
- `single_phase_pll`, `FHT-PLL`, `DDSRF-PLL`: the three PLLs, outputs angle [rad] and pulsation [pu].
  The Park PLL gives the angle of its observer, the other two the angle of the loop; the three give the
  frequency of the observer.
- `SOGI_single_phase_dqPLL`, `DDSRF_single_phase_dqPLL`, `single_phase_PLL_comparison`,
  `single_phase_PLL_comparison1`: scopes; `theta_g` and `omega_g` were added to their buses.
- `data_logging`: four To Workspace blocks (Array, one row per control period, `MaxDataPoints`
  `param.glb_time.Ns_afe`): `sig_log` [t u theta_g omega_g], `park_pll_log`, `fht_pll_log`,
  `ddsrf_pll_log` [phase omega].
- Solver: discrete fixed step `param.glb_time.ts_inv` (ode1), stop time `param.sim.simlength` = 1 s.

## Parameters

`init_model.m` fills the struct `param`; the model reads only `param.*` names (23 distinct references,
checked by the init). The PLL blocks keep the names of the AFE control of `frt_dqpll`, so a block copied
between the two models needs no change.

| Group | Content |
|---|---|
| `param.sim` | `simlength` 1 s, `test_case` (1 to 6) and its name, `check_model`, `design_figures` |
| `param.time` → `param.glb_time` | `fpwm` 4 kHz with double update: `prv.timing_param` gives `ts_afe` = `ts_inv` = 125 µs and `Ns_afe` (logging window of `t_measure` = 1 s) |
| `param.grid` | `fgrid` 50 Hz and `param.grid.emu.omega_grid_nom` (the name of the grid emulator of `frt_dqpll`; this model has no emulator) |
| `param.test` | test signal of the case, `prv.test_param` (table below) |
| `param.afe.ctrl` | class `ctrl_afe_setup` of the library: trackers `fht_grid` (δ 0.05, poles −ω0 and −4ω0, forward Euler), observer `di_obs` (poles −ω0 and −4ω0), FHT-PLL gains `pll_p_fht` 1 and `pll_i1_fht` 80, `tau_ddsrf` = 1/ω0; the init adds the DDSRF-PLL gains `pll_p_ddsrt` 0.32 and `pll_i1_ddsrt` 8 (the values of `frt_dqpll`) |
| `param.afe.pll.fht_undamped` | option, default false: trackers on the undamped oscillator with the exact discretization `expm(A ts)` (no angle error at 50 Hz, see the design check) |
| `param.afe.pll.park` | Park PLL: `tau` = 2/ω0 of the two filters, `kp` 1, `ki` 0, output limits `sat_pos` 2 / `sat_neg` −2 pu around the nominal frequency |
| `param.afe.design` | result of the design check `prv.pll_design` |

| `private/` function | What it does |
|---|---|
| `timing_param` | control period of the three PLLs and the logging window |
| `test_param` | the six test signals (struct `param.test` of the block `test_signal`) |
| `pll_design` | design check: Park generator as a SOGI and its averaged detector at 1 pu and at 0.2 pu, frequency response of the tracker (angle error, image, harmonics) and of stage A, loops of the FHT-PLL and of the DDSRF-PLL, observer; figures when `param.sim.design_figures` is true |
| `pll_metrics` | lock time, error before the event, peak, settling time, final error and ripple of one PLL on one test (used by `run_test_cases` and `plotting_results_plls`) |
| `library_setup` | puts `pwr-control/library` on the path (next to this repository, or the folder of `PWR_CONTROL_LIBRARY`), cloned when missing |
| `check_model_param_refs`, `check_from_goto` | checks after `open_system`: every `param.*` read by the blocks exists; From tags without Goto |

| Test case | Signal |
|---|---|
| 1 | the 2021 test: 4th and 5th harmonic 10 %, sag to 0.2 pu from 0.25 s to 0.5 s, start 180° away from the PLLs |
| 2 | phase jump of +30° at 0.4 s |
| 3 | frequency step from 50 Hz to 49 Hz at 0.4 s |
| 4 | DC offset of 0.02 pu from 0.4 s |
| 5 | odd harmonics from 0 s: 3rd 5 %, 5th 6 %, 7th 5 % (the limits of EN 50160) |
| 6 | sag to 0.2 pu from 0.25 s to 0.5 s, no harmonics |

Old workspace names and their place in `param`:

| Legacy name | Now |
|---|---|
| `single_phase_inverter_ctrl.fht_load.Ld_fht`, `.Ad_fht`, `.Cfht` (four trackers) | `param.afe.ctrl.fht_grid.Ld_fht`, `.Ad_fht`, `.Cfht` (same design in the two classes: δ 0.05, poles −ω0 and −4ω0, 125 µs) |
| `∓1/single_phase_inverter_ctrl.omega_base` | `∓1/param.afe.ctrl.fht_grid.omega_fht` |
| `afe_ctrl.di_obs.Ldso`, `.Adso`, `.Cso` (two observers) | `param.afe.ctrl.di_obs.Ldso`, `.Adso`, `.Cso` |
| `afe_ctrl.pll_i1_fht*glb_time.ts_afe`, `afe_ctrl.pll_p_fht` | `param.afe.ctrl.pll_i1_fht*param.glb_time.ts_afe`, `param.afe.ctrl.pll_p_fht` |
| `grid_emu.omega_grid_nom` | `param.grid.emu.omega_grid_nom` |
| `glb_time.ts_afe`, `glb_time.ts_inv` | `param.glb_time.ts_afe`, `param.glb_time.ts_inv` |
| `omega_set*glb_time.ts_inv` (angle integrator of the Park PLL) | `param.grid.emu.omega_grid_nom*param.glb_time.ts_inv` |
| `2/omega_set` (`tau` of `uxi_pll_flt`, `ueta_pll_flt`) | `param.afe.pll.park.tau` |
| `pi-ctrl` of the Park PLL: kp 1, ki 0, limits ±2 (numbers in the block) | `param.afe.pll.park.kp`, `.ki`, `.sat_pos`, `.sat_neg` |
| `simlength` (1.25 s) | `param.sim.simlength` (1 s: the events of the cases fall between 0.25 s and 0.5 s) |

## Changes to the model (November 2026)

- Parameters re-pointed to `param.*` (table above, 38 block parameters).
- The DDSRF-PLL copied from `frt_dqpll` had a scoped Goto `Vgrid_unorm_ddsrf` without its Goto Tag
  Visibility block, and the model did not compile: the block was added in `DDSRF-PLL`.
- The sources of the 2021 test (three Sine Wave, two Step, Sum, Product, Gain blocks) were replaced by
  `Digital Clock` and `test_signal`; case 1 is the same signal.
- `theta_g` and `omega_g` added to the comparison scopes; subsystem `data_logging` added.
- Stop time `param.sim.simlength`, fixed step `param.glb_time.ts_inv`.
- **Custom code removed**: the Simulation Target settings held a custom header (709 characters), the include
  folders `include src` and a list of C sources (502 characters) of files that do not exist in this folder,
  while the model has no C Caller block. Every run gave the warnings "Error resolving Custom Code" and
  "include/src specified in custom include directory paths string does not exist"; the settings were
  cleared (header, sources, include folders, libraries, initializer, terminator) and the warnings are gone.

The model was run in MATLAB R2026b on the six cases with the new init: the results are below. Python
replicas of the three blocks (in the note) reproduce the logged phases within 4e-13 rad.

## Design check (November 2026)

The numbers come from `prv.pll_design` (printed by the init) and from the note, where the linear models are
checked against the simulation. ω0 = 2π 50 rad/s.

1. **Park generator.** With the filters `tau` = 2/ω0 the quadrature generator is a SOGI with gain
   k = 1/(ω0 tau) = 0.5: exact at its tuning frequency, and it follows the frequency of the PLL. But the
   phase detector is the direct Park transform of the **raw** input with `beta`: harmonics and DC pass with
   half their amplitude. Averaged model of the detector: `eta/eps = V (s + wf)/(2 s + wf)`, wf = 1/tau.
2. **Park PLL loop** (kp 1, ki 0): crossover 189 rad/s, phase margin 73° at 1 pu; 55 rad/s at 0.2 pu
   (`eta` is not normalized). Type 1: a frequency step leaves 1.15° per Hz at 1 pu (5.7° per Hz at 0.2 pu).
3. **Trackers** (δ 0.05, forward Euler, poles −ω0 and −4ω0): the damping of the oscillator model puts the
   positive sequence 0.74° ahead of the input at 50 Hz with an image of 1.4 %; stage A of the FHT-PLL (three
   trackers) leads by 1.49°. Undamped with the exact discretization (`fht_undamped`): 0° at 50 Hz.
   The trackers are wide (a DC offset passes with gain 1.47, a 3rd harmonic with 0.73 and 0.36 on its two
   components) and fixed at 50 Hz.
4. **FHT-PLL loop** (kp 1, ki 80): the trackers are outside the loop; crossover 324 rad/s, margin 76°.
5. **DDSRF-PLL loop** (kp 0.32, ki 8): the decoupling network is inside the loop; its exact transfer
   function (not the low-pass alone) gives crossover 103 rad/s and margin 57°. The frequency integrator
   starts at 0 and the Saturation block holds it at 0.5 pu from the first step: lock in 94 to 180 ms.
6. **Observer** `di_obs`: poles −ω0 and −4ω0, about 4 ms behind the loop.

Results of the six cases (as built, logs of the model; angles in degrees):

| Case | Park PLL | FHT-PLL | DDSRF-PLL |
|---|---|---|---|
| 1, 2021 test | lock 44 ms, ripple 3.39 pp, sag peak 17.2 | lock 36 ms, offset 1.49, ripple 2.59, peak 21.6 | lock 94 ms, offset 0.85, ripple 0.39, peak 18.9 |
| 2, jump +30° | settles in 26 ms | 33 ms | 89 ms |
| 3, 50 → 49 Hz | final error 1.15 | 3.22 | 1.61 |
| 4, DC 0.02 pu | ripple 3.37 pp | 2.09 | 1.46 |
| 5, EN 50160 harmonics | ripple 1.78 pp | 1.93 | 0.31 |
| 6, sag to 0.2 pu | peak 15.2 | 20.5 | not settled before the sag (start) |

## Recommendations (tested in the note, not applied)

The blocks stay as in the controls they come from; the init text names the three changes.

1. **Park PLL: `param.afe.pll.park.ki` = 20.** No error after a frequency step; margin 67° at 1 pu and 55°
   at 0.2 pu; the jump settles in 37 ms instead of 26 ms.
2. **Trackers: `param.afe.pll.fht_undamped` = true.** No offset at 50 Hz (FHT-PLL 0° instead of 1.49°,
   DDSRF-PLL 0° instead of 0.74°), half the error at 49 Hz. The right place for the change is the class
   `ctrl_afe_setup` of the library (`first_harmonic_tracker`), shared with the AFE models.
3. **DDSRF-PLL: frequency integrator started at 1 pu** (initial condition of the integrator of stage B,
   inside the clamp): lock in 0 to 135 ms instead of 94 to 180 ms. The block is the one of `frt_dqpll`:
   the same change applies there.

Options with a price (numbers in the note): the PI of the Park PLL on the filtered `eta` (the detector of
the SOGI-PLL, kp 0.3 and ki 3: ripple 0.14° in case 1 instead of 3.39°, but the jump settles in 141 ms); DC
estimators; narrower trackers (less harmonic ripple, more error at 49 Hz).

## Open points

- The trackers are tuned at 50 Hz: an adaptive tuning (frequency of the loop fed to the trackers) needs its
  own design; a first attempt with the unfiltered frequency lost the lock.
- The errors of 15° to 20° at the edges of a sag come from the generators; holding the loop for a few ms
  after an amplitude step was not tested.
- The model feeds the PLLs with an exact sample of the voltage: no acquisition delay (one control period
  is 2.25° at 50 Hz) and no quantization.
