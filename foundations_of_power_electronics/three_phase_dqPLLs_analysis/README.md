# three_phase_dqPLLs_analysis — grid PLLs under an asymmetric grid fault

A 690 V, 1 MW grid emulator (Dyn11 transformer, `configurable_trafo`) feeds the LCL filter of a 250 kW
AFE whose converter side is **open** (the `AFE` subsystem holds three Open Circuit blocks and the modulator
inputs are zero: `sv_pwm` only gives the 4 kHz trigger of the control). The AFE control runs on the grid
voltages at the filter capacitor and compares four phase locked loops on the same signal: the plain dq-PLL
(Simulink), the same dq-PLL as a C caller (`dqpll_grid`), the DDSRF-PLL and the FHT-PLL (first harmonic
tracker of each phase); a Variant Source picks one of them for the grid support blocks of the library
(`afe_uvw_ctrl`: FRT detection VDE-AR-N 4110, voltage and current sequences, islanding detector). The grid
emulator applies test 25.4 of VDE-AR-N 4110 in its asymmetric variant 1 (phase U to 0.25 pu, phases V and W
to 0.875 pu rotated by 21.8 degrees) from 0.75 s for 915 ms; the simulation runs for 2.25 s, through the fault and its recovery.

The init was rewritten in November 2026 as a `param` struct with the functions of `private/`, the model was
re-pointed to the new names and the Goto/From tags of the control blocks copied from the library were fixed
(see [Wiring fixes](#wiring-fixes-of-the-control-tags-november-2026)).

**Design note:** [`repos_doc/three_phase_plls_comparison`](https://github.com/pwr-control/repositories-documentation/tree/main/repos_doc/three_phase_plls_comparison)
of pwr-control/repositories-documentation (November 2026): the three PLL blocks (dq-PLL, DDSRF-PLL, FHT-PLL)
from the control point of view, their continuous-time models, the performance on the logged fault and on
test signals, the criticalities and what to change. Its MATLAB script is `plotting_results_plls.m` (below).

| File | Content |
|---|---|
| `init_model.m` | initialization (plain-text live script): every number in the struct `param`, the design check of the PLL loops, then it opens the model, imports the two C types of the C callers, closes the scopes, creates the bus of the power meter and checks the `param.*` references and the From/Goto tags. Run it **from this folder**. |
| `frt_dqpll_private.m` | handles to the functions of `private/` (`prv.<name>`): the Live Editor runs a live script from a copy outside this folder and does not see `private/` |
| `private/` | the parameter functions of this model (table below) |
| `frt_dqpll.slx` | the model (grid emulator, LCL filter, AFE control with the four PLLs, logging) |
| `plotting_results_afe.m`, `plotting_results_fhtsr_dqpll.m` | post-processing of the logged grid and PLL quantities |
| `plotting_results_plls.m` | the PLL variants of the design note re-run offline on the logged capacitor voltage (dq-PLL with constant and with scheduled gain, FHT-PLL, DDSRF-PLL as built, with the positive feed and corrected): checks the FHT-PLL branch against the log, prints the metrics table and draws four figures. Works on the base workspace after a run, or on a saved file: `plotting_results_plls('sim_results_01.mat')` |
| `video_plot/video_plot_FHT_dqPLL.m` | animation of the FHT dq-PLL response |
| `voltage_spectrum_analysis.m` | FFT of the grid phase voltage `vg_abc_sim` (bases now from `param`, they were the 400 V ones of another model) |
| `plotting_results_inv.m`, `plotting_results_dclink.m` | post-processing of other models (PMSM inverter, DC link): they plot signals this model does not log |
| `non_virtual_blocks_counting.m` | counts the non-virtual blocks of the model |

## The model

**Power stage** (Simscape, solver step `tc` = 625 ns, 3 nonlinear iterations).
- `grid/grid_emulator`: library MATLAB Function grid emulator (`power_supply_emulators`, `_mfcn`) with the
  Dyn11 transformer `configurable_trafo` (mask from `param.grid.emu.trafo.pu.*`, zero-sequence magnetizing
  inductance `lm0` = 0.5 pu); the fault comes from `param.frt.data.*` (amplitudes U V W, phase shift, start,
  length, 20 ms ramp). The IEC power meter (`power meter iec mfunc`) gives the sequence powers
  `P1p_global` ... `Q1n_global` through the bus `PowerMeterIecBus`.
- `LCL filter`: grid-side measurements, filter capacitors `CFu` = 200 µF with 50 mΩ, converter-side
  inductors 500 µH + common mode 500 µH; the converter side is open. The only series impedance between the
  emulator voltage and the capacitor is the transformer leakage (5 %, 76 µH): the equivalent grid impedance
  of the emulator object (`eq_grid_inductance`, `eq_grid_resistance`) is **not read** by any block.
- `global timing`, `sv_pwm`: time bases and the trigger `TRGO_afe_module_1` of the control, 4 kHz PWM with
  double update, so `ts_afe` = 125 µs. Logging at `tc` and at `ts_afe` in `afe_measurement_sim`
  and `scopes`.

**Control** (`AFE_control1/afe_control`, triggered at 125 µs, quantities in pu of the AFE):
- acquisition of `ug_abc`, `ig_abc`, `is_abc` with 12 bit quantization and one step delay;
- `grid_voltage_phase_lock_loops`: the four PLLs, each as stage A (dq transformation of the normalized
  voltage, `Vgrid_unorm_*`) and stage B (loop PI on `u_pos_eta`, forward Euler, phase integrator, double
  integrator observer that gives `omega_grid_pll_pu` and `phase_grid_pll` one step late). The dq-PLL
  schedules its proportional gain with `frt_active` (`pll_p` = 1 normal, `pll_p_frt` = 0.2 in the fault).
  The Variant Source reads `param.afe.pll.use_dq_pll_mode1..4` (1 dq-PLL, 2 C caller, 3 DDSRF, 4 FHT,
  default 4; exactly one must be 1);
- library blocks (copies of `afe_uvw_ctrl`, mask parameter `p` = `param.afe.uvw`): `FRT-VDE4110`
  (`frt_active`, `i_grid_seq_ref`, `pwr_lim_enable`), `grid_voltage_sequences` (`v_grid_pos/neg_xi/eta_avg`,
  `ug_alpha_pu`, `ug_beta_pu`), `grid_current_sequences_and_reactive_pwr_ctrl` (`i_grid_*_avg`,
  `i_stack_*_ref`), `islanding_mode_detector`;
- `data_to_worspace_module_1` and `interesting scopes` log and show the sequences and the PLL outputs.

## Parameters

`init_model.m` fills the struct `param`; the model reads only `param.*` names.

| Group | Content |
|---|---|
| `param.sim` | `simlength` 2.25 s, nonlinear iterations, `check_model`, `design_figures` |
| `param.time` → `param.glb_time` | `fpwm` 4 kHz, double update, dead time 3 µs, `tc_factor` 100 (tc 1.25 µs): `prv.timing_param` gives `ts_afe`, `tc`, `Nc`, `Ns_afe`, `fPWM_AFE`, ... (AFE domain only) |
| `param.grid` → `param.grid.emu` | 1 MW, 690 V, Dyn11, core data: `prv.grid_param` adds the dataset of the voltage class (50 Hz, η 95 %, `ucc` 5 %, iron losses 1.8 kW), `n1`, `Lm1`, the no-load current and the object `grid_three_phase_emulator` with `trafo.pu.*` |
| `param.frt` → `param.frt.data` | test 25.4, asymmetric type 1, start 0.75 s: `prv.frt_param` merges `frt_settings` and `grid_fault_generator` of the library |
| `param.afe.hw`, `hw_in` | 250 kW hardware of the 690 V class (`prv.afe_hw_param`: `udc_nom` 1070 V, LFu, CFu, CFi, brake) through `hw_afe_three_phase_setup` |
| `param.afe.ctrl` | `ctrl_afe_setup` of the library: PLL gains `pll_i1` 80, `pll_p` 1, `pll_p_frt` 0.2, double integrator observer, first harmonic trackers, resonant PI, rms windows |
| `param.afe.pll`, `ref`, `behav` | PLL variant switches, current references and reactive steps (at 2.0 s and 2.5 s, after the end of the simulation), clock gain and PWM phase of the module |
| `param.afe.uvw` | parameters of the four library control blocks (`prv.afe_uvw_param`, same values as `afe_uvw_three_phase`) |
| `param.afe.design` | result of the design check `prv.pll_design` (loops, observer, FHT, LCL) |
| `param.inv.behav`, `param.meas`, `param.ctrl` | clock gain of a second module, ADC quantization and ranges, moving average and the 50 Hz second order filter of the emulator |

| `private/` function | What it does |
|---|---|
| `timing_param` | sample times of the AFE domain and the logging base `tc` |
| `grid_param` | transformer dataset, magnetizing inductance, no-load current, `grid_three_phase_emulator` |
| `frt_param` | fault settings of the library, prints the fault summary |
| `afe_hw_param` | LCL/DC link hardware of the voltage class, `hw_afe_three_phase_setup` |
| `afe_uvw_param` | parameters of the library control blocks (sequences, FRT, current control, islanding) |
| `pll_design` | closed loop of the dq-PLL (normal, fault and an alternative gain set), observer and FHT poles, LCL seen from the grid; three figures when `param.sim.design_figures` is true |
| `power_meter_bus` | the bus object `PowerMeterIecBus` of the power meter (base workspace) |
| `library_setup` | puts the library on the path (`pwr-control/library` next to this repository) |
| `check_model_param_refs`, `check_from_goto` | checks after `open_system`: every `param.*` read by the blocks exists; From tags without Goto |

Old workspace names and their place in `param`:

| Legacy name | Now |
|---|---|
| `hwdata.afe` | `param.afe.hw` |
| `afe_ctrl` | `param.afe.ctrl` |
| `glb_time` | `param.glb_time` |
| `grid_emu` | `param.grid.emu` |
| `adc_quantization` | `param.meas.adc_quantization` |
| `afe_pwm_phase_shift_mod1` | `param.afe.behav.pwm_phase_shift_mod1` |
| `simlength` | `param.sim.simlength` (also the stop time of the configuration set) |
| `time_gain_afe_module_1`, `time_gain_inv_module_2` | `param.afe.behav.time_gain_module_1`, `param.inv.behav.time_gain_module_2` |
| `use_dq_pll_mode1` ... `use_dq_pll_mode4` | `param.afe.pll.use_dq_pll_mode1` ... `4` |
| `tc` (fixed step of the solver, undefined in the old init) | `param.glb_time.tc` |

The new init gives the same values as the legacy one for every expression the old init defined (checked
with Octave stubs on the 87 parameter expressions of the model).

## Wiring fixes of the control tags (November 2026)

The four control blocks copied from the library write scoped Goto tags without the `_mod1` suffix
(`frt_active`, `i_grid_seq_ref`, `v_grid_*_avg`, `i_grid_*_avg`, `ug_alpha_pu`, `ug_beta_pu`) and read
`ugd_abc_pu`, `omega_grid_pll_pu`, `phase_grid_pll`, while the rest of `afe_control` still used the
`_mod1` names: 20 From blocks had no Goto, so the sequence blocks ran on a zero grid voltage, PLL phase and frequency
and the logged sequences were zero. Fixed in `frt_dqpll.slx` (46 blocks, same edits as in
`afe_rst_ctrl`):
- Goto Tag Visibility blocks of `afe_control` renamed without `_mod1` (`i_grid_*_avg`, `v_grid_*_avg`,
  `ug_alpha_pu`, `ug_beta_pu`, `omega_grid_pll_pu`, `phase_grid_pll`, `ugd_abc_pu`); the unused
  `pwr_act_grid_mod1` and `pwr_react_grid_mod1` became `frt_active` and `i_grid_seq_ref`;
- Goto `omega_grid_pll_pu_mod1` → `omega_grid_pll_pu`, `phase_grid_pll_mod1` → `phase_grid_pll` (PLL
  output), `ugd_abc_pu_mod1` → `ugd_abc_pu` (acquired grid voltage) with their From blocks;
- From blocks of `data_to_worspace_module_1`, `interesting scopes`, the scheduled gain of the dq-PLL
  (`frt_active`) and the islanding detector inputs (`ug_alpha_pu`, `ug_beta_pu`) renamed; the top From
  `gamma_grid_emulator` → `phase_grid_emulator` (the tag of the emulator), so the scope of the PLL
  performance compares with the true grid phase again;
- the power meter scope of `afe_control` and its two From blocks (`pwr_act_grid_mod1`,
  `pwr_react_grid_mod1`: nothing writes them in this model) are commented out.

Still without Goto, as in the other models: `local_time_corr_afe1` and `local_time_corr_inv1` in
`global timing` (read 0). The scoped Gotos `i_stack_*_ref` and `pwr_lim_enable` have no reader here.

## Design check (November 2026)

The numbers come from `prv.pll_design` (printed by the init, figures with `param.sim.design_figures`).
Loop of every PLL: e = `u_pos_eta` · Vnorm, ω = kp e + ki ∫e, θ = ∫ω_n ω, so
T(s) = ω_n (kp s + ki) / (s² + ω_n kp s + ω_n ki) with ω_n = 2π 50.

1. **dq-PLL, normal gains** kp 1, ki 80: natural frequency 25 Hz, damping 0.99; the discrete loop
   (forward Euler at 125 µs, output one step late) has poles |z| = 0.983, 0.977, 0.04: nothing lost.
2. **Fault gains**: kp goes to 0.2 with `frt_active` but ki stays 80, so the damping drops to **0.20**:
   the phase rings at 25 Hz for about 150 ms after every phase jump of the fault (the asymmetric test has
   two of them). If the aim is a slower PLL in the fault, ki must scale with kp²: ki = 3.2 keeps damping
   0.99 at 5 Hz, with a 100 Hz ripple gain 0.100 rad/pu instead of 0.126.
3. **100 Hz ripple**: test 25.4 type 1 has a negative sequence of 0.375 pu (positive 0.625 pu). On the
   plain dq-PLL it enters `u_pos_eta` at 100 Hz with |T(j 2ω_n)| = 0.475 rad/pu (normal gains): a phase
   ripple of 0.18 rad (10 degrees) peak; 0.047 rad (2.7 degrees) with the fault gains. The DDSRF and FHT
   PLLs remove it by construction: this is what the comparison shows.
4. **Start**: the frequency integrator starts at 0, so every PLL starts from zero frequency and locks in
   the first 50 ms; an initial condition of 1 pu would remove this transient.
5. **DDSRF-PLL, stage B**: its loop is fed with outputs 3 and 4 of the DDSRF block (the negative-sequence
   voltages) through a gain −1; with mode 3 selected the loop should see the positive-sequence q voltage.
   Not changed (the default is the FHT-PLL): please check.
6. **Integrator limits**: the dq-PLL limits its frequency integrator to ±2 pu, the DDSRF and FHT loops
   have no limit: a lost lock winds up.
7. **FHT**: the C code discretizes the tracker with forward Euler; with the exact (Taylor 2) matrix the
   observer poles move from [−314 −1257] to [−320 −1264] rad/s: negligible at 125 µs.
8. **Double integrator observer** on the PLL phase: poles [−314 −1257] rad/s, about 4 ms of delay on
   `omega_grid_pll_pu` and `phase_grid_pll`, which the sequence blocks use.
9. **LCL seen from the grid** (converter open): 76.8 µH (leakage + 1 µH) with 200 µF and 50 mΩ resonate at
   **1284 Hz** with damping 0.06 (gain 8 at the resonance on the capacitor voltage the control measures);
   every step of the fault excites it. No grid inductance in the model, see above.
10. **Simulation length**: 2.25 s (it was 1.25 s, inside the fault, until the run of November 2026) covers
    the fault (0.75 → 1.665 s), the recovery and the first reactive current step of `param.afe.ref` (2.0 s);
    the second step (2.5 s) needs `param.sim.simlength` of 2.7 s or more.
11. `plotting_results_inv.m` and `plotting_results_dclink.m` plot signals of other models (PMSM currents,
    DC link): they stop on missing variables.

## Open points

- The fault gain set (point 2) and the DDSRF stage B input (point 5) are design choices of the author,
  reported and not changed.
- The Goto Tag Visibility blocks of the full AFE control (`isa_mod1`, `udcd_pu_afe_mod1`, `u_7th_*`, ...)
  stay in `afe_control` without a Goto: harmless.
