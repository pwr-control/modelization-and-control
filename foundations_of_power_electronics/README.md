# Foundations of Power Electronics

Converter and drive projects built with Simulink and Simscape. Each subfolder is a
self-contained study: power stage, control, and post-processing scripts.

All projects need the companion **`library`** repository on the MATLAB path (custom Simscape
components and the C-Caller control code). Run `init_model.m` / `init_model.mlx` first — it sets
the parameters and opens the model.

Recurring building blocks across the projects:

- **Current control** — dq vector PI, resonant PI (stationary frame), or model predictive
  control, available both as Simulink subsystems and as C-Caller code.
- **Grid synchronization** — dq-PLL, DDSRF-PLL, SOGI, phase-shift filter, and a first-harmonic
  tracker (FHT) based dq-PLL developed here.
- **Observers** — back-EMF observer, Luenberger speed observer with load-torque estimator,
  extended Kalman filter for PMSM and induction machine, SOC estimator for the battery models.
- **Grid support** — FRT/LVRT test catalogue (`grid_fault_generator.m`, `asymmetric_error.m`),
  VDE 4110 logic, positive/negative sequence reactive current references.
- **Device models** — ideal switch, IGBT and MOSFET with loss and thermal models
  (Mitsubishi CM1200DW-34T, Infineon FF1200R17IP5, Danfoss SKM1700MB20R4S2I4, Wolfspeed
  CAB016M12FM3, ...), on air- or liquid-cooled heatsink models.

---

## Three-phase AFE and drives

### `afe_dq_three_phase`
- Three-phase two-level active front end, 400 V, 250 kW, on a 1 MVA Dyn11 grid emulator;
  fPWM = 4 kHz with double update, 3 µs deadtime.
- dq vector control with resonant PI, DC-link voltage PI, FHT-based dq-PLL.
- Thermal model enabled (IGBT and SiC MOSFET device sets); post-processing reports device
  losses and junction-to-heatsink / heatsink-to-water temperatures.
- Includes an island-mode voltage spectrum analysis script.

### `afe_dq_ttype_three_phase`
- Same AFE study on a three-level T-type bridge, 690 V, 250 kW, fPWM = 4 kHz.
- Same control stack (resonant PI, FHT dq-PLL, SOGI, EKF setups) and device/thermal sets.

### `afe_inv_psm`
- Reference back-to-back system of the repository: three-phase AFE + DC link + inverter + PMSM,
  690 V, 250 kW per stage, fPWM = 4 kHz, Simscape step 0.25 µs.
- dq vector control with resonant PI, DC-link PI, FHT dq-PLL, MTPA, sensorless operation via
  back-EMF/EKF observers; several blocks run as C-Caller code.
- Thermal model on (FF1200R17IP5); `power_loss/` holds a loss dataset for an FF650R17IE4 module.
- `calcolo_filtro_LRC_400V.mlx` sizes the LRC grid filter.

### `afe_dqctrl_inv_psm`
- Variant of `afe_inv_psm` with the same ratings and control set, kept as a separate
  configuration of the AFE dq control.

### `afe_inv_loopback`
- AFE + inverter in loopback (test-bench) configuration, 690 V grid through a 1.6 MVA Dyn11
  emulator, DC link 1070 V, 250 kW machine at 17.8 rpm in generator mode.
- Resonant PI current control with separate positive/negative sequence reactive current loops,
  C-Caller dq-PLL, double-integrator observer, first-harmonic tracker for grid cleaning.
- Machine side: Luenberger position/speed observer, load-torque estimator, field weakening, MTPA.
- `grid_fault_generator.m` implements the LVRT/FRT test catalogue (symmetric and asymmetric
  dips, variants C and D).

### `afe_inv_ttype_psm`
- AFE and inverter both built as three-level T-type bridges feeding a PMSM, 690 V, 250 kW,
  fPWM = 5 kHz.
- Also carries the rail/catenary parameter set (3000 V nominal, 4000 V max, ISOP chain of
  4 × 250 kW) inherited from the shared hardware library.

### `afe_inv_psm_cascaded`
- Two (up to four) cascaded AFE + inverter + PMSM modules with independent time domains: each
  module generates its own sampling/PWM trigger, so sliding PWM and clock deviation between
  modules can be analysed.
- Hard-parallelization features: common-mode voltage control, per-module reactive current
  references, double moving-average filtering.
- The most advanced model of the repository in terms of time-distribution modelling.

### `distributed_ctrl`
- Same plant as `afe_inv_psm` but with the control distributed over two controllers and an
  explicit two-sample communication delay — a study of delayed feedback.
- Full grid-support layer (FRT detection, VDE-style reactive current injection, online sequence
  extraction) and machine-side observers.

### `inv_psm`
- Inverter + PMSM only (no AFE), 690 V, 550 V / 370 A inverter, fPWM = 4 kHz. Used mainly for
  control design and loss evaluation.
- Thermal model on; `doc/iposim/` contains Infineon IPOSIM cross-checks at 370 A / 4 kHz for
  FF1200R17IP5 and FF650R17IE4, including generating-mode cases.

### `inv_im`
- Inverter + induction machine (ABB M3BP 355MLB 6, 261 kW) with encoder, fPWM = 4 kHz.
- dq PI current control, MTPA, torque-curve mode; bridge selectable between ideal switch,
  IGBT and MOSFET with thermal model.

### `im_analysis_and_design`
- Induction machine modelling and indirect field-oriented control; three model variants
  (`im_inv_model`, `im_gyro_inv_model`, `inv_im_ekf`).
- `graphics.mlx` plots stator/rotor currents and fluxes in the stationary and rotor frames —
  the notebook is for machine-model verification rather than drive tuning.

### `im_sensorless_control_analysis_and_design`
- Sensorless IFOC of a 250 kW / 400 V / 465 A Siemens induction motor, fPWM = 8 kHz.
- Machine parameters identified from no-load and locked-rotor tests
  (`im_param_calc_*.mlx`, results in `im_data_motor.txt`).
- Speed PI, dq current PIs, zero-sequence current loop, Luenberger and double-integrator speed
  observers, and a 4-state Kalman filter (stator currents + rotor fluxes).
- `flux_obs/` repeats the framework on a 440 kW SICME machine for flux-observer study;
  `references/` collects the papers used (Schauder, Hori, Utkin, Montanari).

### `modular_multiphase`
- Multiphase / modular inverter feeding an induction machine (250 kW GM 355L 6), fPWM = 10 kHz.
- Rotor-flux vector control with an explicit zero-sequence current loop — the loop that
  distinguishes the multiphase case — plus per-module current tracking plots.

### `psm_sensorless_ctrl`
- Simplified PMSM models for fast control design (six subsystems of a 1.5 MW direct-drive
  generator at 17.8 rpm), in four variants:
  - `psm_sv_bemf_ctrl` — space vector modulation + back-EMF observer;
  - `psm_sv_ekf_bemf_ctrl` — space vector + extended Kalman filter;
  - `psm_mpc_bemf_ctrl` — one-step model predictive control + back-EMF observer;
  - `psm_mpc_ekf_bemf_ctrl` — one-step MPC + EKF.
- `kalman_psm.m` builds the 6-state EKF (αβ currents, back-EMF, angle, speed).

### `mpc_psm`
- Model predictive current control of a PMSM drive, 690 V, 250 kW. The inverter runs at
  6 × fPWM (15 kHz) with single update — the oversampling typical of FCS-MPC.
- Formulation based on the work of Geyer and Rodriguez; cost function and candidate vectors
  live inside the model.

### `mpc_ttype_psm`
- Predictive control of a three-level T-type inverter feeding a PMSM, fPWM = 4 kHz with double
  update. Useful for MPC analysis and convex-optimization investigations on a three-level leg.

### `sic_inverter_for_drone`
- Hardware/software design of a 20 kW class SiC inverter for a propeller drive:
  fPWM = 25 kHz, 400 V DC link, Wolfspeed CAB016M12FM3, ALPHA LT13070 heatsink.
- 62-pole PMSM, 64 Nm at 2291 rpm, modelled in per-unit with spatial harmonics and a flexible
  shaft; propeller load from measured thrust/torque tables.
- Sensorless FOC with MTPA, field weakening, I/F startup; selectable back-EMF, Luenberger,
  fourth-order load and oscillator-Kalman observers.
- `bemf_measure/` holds oscilloscope captures of the back-EMF measured by hand rotation;
  `pmsm_data/`, `module_data/`, `heatsink_data/`, `load_data/`, `propeller_data/` hold the
  parameter sets.

---

## Single-phase converters

### `single_phase_afe`
- Single-phase full-bridge active front end, fPWM = 5 kHz, 690 V, 225 kVA class.
- DC-link voltage PI over dq vector PI or resonant PI current control; three orthogonal-signal
  generators compared (SOGI, phase-shift filter, first-harmonic tracker), each in Simulink and
  C-Caller form.
- Dedicated DC-link power decoupling / buffer branch (`dcdc_buffer.slx`) to handle the pulsating
  single-phase power, with a resonant-PI ripple controller.
- `plant_estimator.m` identifies R and L of the load from dq-projected signals.

### `single_phase_afe_ttype`
- Same study on a three-level T-type single-phase bridge, fPWM = 4 kHz, 690 V.
- Includes an impedance-measurement / system-identification subsystem and the FRT machinery
  (disabled in the committed configuration).

### `single_phase_catenary`
- Single-phase AFE applied to a DC catenary auxiliary supply: catenary 3000 V nominal
  (4000 V max), 2 × 5.4 MW per 12 km, ISOP stack of 4 × 250 kW full bridges.
- Shares the power stage, control and post-processing of `single_phase_afe`.

### `single_phase_inverter`
- `two_levels/` — single-phase H-bridge, fPWM = 10 kHz, 400 V, with result scripts for 50 Hz,
  80 Hz and 400 Hz output; full device loss and junction temperature breakdown.
- `three_levels/` — single-phase T-type bridge, fPWM = 4 kHz, 690 V, MOSFET thermal model.
- Both: dq vector PI or resonant PI, virtual dq from SOGI / phase-shift filter / FHT, open-loop
  U/f mode, system identification, output transformer and H5/H7 harmonic shunt filters.

### `battery_single_phase_inverter`
- Single-phase full bridge fed by a lithium-ion battery model, 400 V, fPWM = 10 kHz with
  sub-microsecond deadtimes (wide-bandgap devices).
- Resonant PI in the stationary frame with a dq PI alternative; SOGI from C-Caller code.
- `plant_estimator.m` (offline R/L identification) and `virtual_phase_shift_filter_analysis.m`
  (αβ construction) support the control design; `power_losses_calculation.m` extracts MOSFET
  losses, ΔT junction-heatsink, RMS currents and DC-link capacitor current.

### `flying_capacitor_single_phase_inverter`
- Single-phase 5-level flying-capacitor inverter from 3000 V DC, phase-shifted carriers,
  fPWM = 5 kHz per cell.
- Two orders of modelization: a fundamental model and one with parasitics (capacitor ESR/ESL,
  stray inductance, snubbers, HF branches) plus a full junction-to-ambient thermal network.
- Two study modes: DC-link precharge sequence, or high-frequency analysis with a ten-times
  finer solver step.

### `zvs_inv_single_phase_6kVA`
- 6 kVA single-phase UPS: lithium-ion battery → phase-shifted full bridge (ZVS) → single-phase
  inverter, fPWM = 20 kHz, 230 V / 30 A output, 400 V DC link.
- Centre-tapped 8 kVA / 25 kHz HF transformer; SiC FF23MR12W1M1P_B11 on an ALPHA LT13070
  heatsink.
- Two interchangeable inverter controllers (dq-based and resonant-PI based), single-phase PLL,
  double-integrator observer and a 50 Hz oscillator Kalman observer; battery SOC estimator.
- `sim_results_mosfet/`, `sim_results_thermal/`, `sim_results_grid_back/` hold the
  post-processing scripts of the three simulated cases.

### `zvs_inv_single_phase_35kVA`
- Same architecture at 35 kVA with two parallel modules: ZVS DC-DC at 20 kHz feeding a
  single-phase H-bridge at 10 kHz, 500 V / 55 A output, 820 V DC link.
- Four model variants (half/full model, two versions); `check_observer.slx` is a standalone
  bench for the Kalman observer, load estimator and PLL.
- `scaling.m` computes the ADC scale and offset constants for the firmware.

### `dab_inv_single_phase_6kVA`
- UPS for emergency lighting: lithium-ion battery → DAB → single-phase inverter,
  fPWM = 35 kHz, 230 V / 30 A output.
- 8 kVA / 25 kHz HF transformer with magnetizing and iron-loss branches; FPGA-style modulator
  clocked at 25 MHz.
- Cascaded voltage + current PI on the DAB, PI with resonant term on the inverter, single-phase
  PLL, Kalman filter for the 50 Hz harmonic state and a 3-state Kalman for battery SOC.
- Battery: 30-cell pack with analytic OCV(SOC) model.

---

## Isolated DC-DC converters

### `theory_analysis_dab`
- Two batteries connected through a DAB, intended for deep analysis of modulation strategies
  and efficiency. Detailed MOSFET model.
- `single_phase_dab/` — 250 kW, fPWM = 12 kHz, phase-shift modulator, cascaded output voltage
  and current PI; `plotting_results_zvs.m` post-processes the soft-switching behaviour.
- `three_phase_dab/` — 750 kW version (De Doncker arrangement) with a three-phase HF
  transformer and MOSFET thermal model.
- `literature/` (Akagi, De Doncker, Kheraluwala, Tao, Krismer) and `components/` (device
  datasheets) collect the references used.

### `theory_analysis_resonant_CLLC`
- Two batteries connected through a CLLC resonant converter, 250 kW, fPWM = 12 kHz, for
  modulation and efficiency analysis; three bridge implementations (ideal switch, MOSFET with
  thermal model, MOSFET ZVS variant). The converter model is in `resonant_cllc/`, the modulator
  studied alone in `cllc_modulator_analysis/`.
- `cllc_analysis.m` builds the resonant tank transfer function symbolically and evaluates gain
  and phase at the switching frequency.
- `transformer_sizing.m` designs the HF transformer (380 kVA, 150 kHz, nanocrystalline cut
  cores); `doc/transformer_losses_sweep.m` sweeps 1 kHz–200 kHz comparing N87, 3C95 and
  nanocrystalline cores with Steinmetz core loss and Dowell winding factor.
- `doc/` also holds a note on DAB/CLLC inductance and an internship report.

### `dcdc_zvs_full_bridge`
- Isolated battery-to-battery ZVS full bridge, 175 kW, 750 V → 540 V, fPWM = 5 kHz; power flow
  by phase shift between the two primary legs, diode rectifier at the secondary.
- `magnetics_design_transformer.m` is a complete HF transformer design (5 kHz, 0.2 T,
  3 A/mm², nanocrystalline cut cores, core and copper losses).
- An explicit ZVS current constraint is derived in the init script; `run_sim_output_current_ctrl.m`
  sweeps six operating scenarios (input/output battery voltage × output current) whose results
  feed the six `figures_case_scenario_N/` folders.

### `dcdc_zcs_with_double_resonant_tank`
- Unidirectional isolated DC-DC based on a half bridge with a CLL double resonant tank,
  zero-current switching with variable switching frequency (up to 48 kHz), 10 V / 100 A output.
- Cascaded output voltage and current PI plus an even-harmonic compensator bank
  (2nd to 14th) for DC-link ripple compensation; the effect is shown with and without
  compensation in `plot_results.m`.
- The init script Bode-plots the resonant tank transfer function with the reflected load.

### `flyback_250V_500W`
- Flyback converter, 500 W at 250 V from a 1070 V DC link, fPWM = 100 kHz, studied in Simscape
  with the flux domain.
- Coupled-inductor design computed in the script (turns, gap, ΔB, magnetizing and leakage
  inductance); detailed DC-link parasitic model and RC snubber.

### `ps_25V_12kA`
- High-current process power supply: three-phase AFE → phase-shifted full bridge → diode
  rectifier → 25 V / 12 kA output (a 10 V / 12 kA case is also stored).
- 480 V / 60 Hz front end, 325 kW per stage, fPWM = 4 kHz.
- Device modelling is the point of the two stored runs: ideal-switch AFE, IGBT with thermal
  model on the full bridge, charge (reverse-recovery) model on the rectifier diodes.
- `docs/ps_25V_12kA_r.pdf` is the design document.

### `ps_isop_rail`
- Railway auxiliary supply as an input-series output-parallel chain on a DC catenary:
  3000 V nominal (4000 V max) split over 4 series systems of 250 kW, fPWM = 4 kHz.
- Dedicated ISOP controller plus DAB/PSFBC/CLLC loops, FHT dq-PLL, FRT enabled.
- `trafo/four_winding_transformer.tex` derives the four-winding transformer analytically
  (flux-linkage matrix, T-equivalent, MMF balance, port relations).

### `ps_isop_rail_bidirectional`
- Bidirectional version of the ISOP rail supply: the battery is moved to 1.5 × catenary voltage
  and sized on the catenary power (10.8 MW) instead of the converter power.

---

## Non-isolated DC-DC converters

### `interleaved_buck_converter`
- Three-leg interleaved buck, 660 V battery to a 200 V bus, 750 A (150 kW, 250 A per leg); Infineon
  FF450R17ME4 half bridges with the thermal model used as three buck legs, fPWM = 3 kHz per leg with
  double update (control at 6 kHz, 9 kHz on the output), 0.8 mH per leg, 7.5 mF output capacitor,
  7.2 mF DC link, resistive load.
- Output voltage PI → total current reference → one PI per leg (reference `i_ref/3`, inherent current
  sharing), each leg current sampled at the extremes of its own carrier, duties of legs 2 and 3 loaded
  at their own extremes; gains from crossover targets (`private/buck_ctrl_param.m`, 225 Hz and 60 Hz),
  linear check in `claude_analysis/`.
- Two models from one init: `interleaved_buck.slx` (discrete blocks) and `interleaved_buck_compact.slx`
  (the compact block `interleaved_buck_3legs_mod1` of the library, built by
  `build_interleaved_buck_compact.m`). See the README of the folder.

### `interleaved_boost_converter`
- The hardware of the interleaved buck with the power flow reversed: 200 V battery on the inductor
  side, 660 V DC link with the load, 750 A of input current (150 kW, 250 A per leg, duty of the lower
  switch 0.697); same FF450R17ME4 half bridges, 3 kHz per leg with double update, 0.8 mH per leg,
  7.5 mF across the battery, 7.2 mF DC link.
- DC-link voltage PI → total input current reference → one PI per leg on the leg current (sampled at
  the extremes of its own carrier) → duty of the lower switch, `1 - d` to the unchanged modulator of
  the buck; gains from crossover targets (`private/boost_ctrl_param.m`: 225 Hz for the current loops,
  the same integrator plant as the buck; 40 Hz for the voltage loop, a quarter of the right-half-plane
  zero of the boost at nominal load, 157 Hz), reference ramp from the precharged link; linear and
  nonlinear check in `claude_analysis/`.
- Two models from one init: `interleaved_boost.slx` (discrete blocks) and `interleaved_boost_compact.slx`
  (the compact block `interleaved_boost_3legs_mod1` of the library, built by
  `build_interleaved_boost_compact.m`). See the README of the folder.

## Rectifiers

### `thyristor_based_models`
- Line-commutated rectifiers and their firing mechanism, implemented and analysed also in C code.
- `six_pulse_thyristor_rectifier/` (`sptr`) — six-pulse bridge, 400 V mains through a 50 kVA
  transformer, 340 V / 90 A DC output, LC filter with 2nd/3rd/5th/7th tuned traps; cascaded
  voltage and current PI, dq-PLL and a harmonic observer (`hobs_test.slx` tests the observer
  alone).
- `six_pulses_thyristor_based_rectifier/` — a second six-pulse variant with the same data.
- `twelve_pulse_thyristor_rectifier/` — twelve-pulse bridge on a three-winding Dy/Dd
  transformer for a 775 V traction DC bus from a 20 kV / 1250 kVA supply, with 5th/7th/11th/13th
  tuned shunt filters. `pm/` is the detailed model, `pm_reduced_model/` its reduced-order
  version, `sm/` a simplified one together with a Simscape circuit-level firing circuit.
- `dual_converter_thyristor_based/` — two six-pulse bridges (positive and negative) for
  four-quadrant DC output.
- Synchronization uses a double-integrator observer, an optional DDSRF-PLL and a first-harmonic
  tracker to clean the grid voltage, all through C-Caller blocks.
- `readme.mlx` explains the file naming (`sptr`, `tp_thyr_bridge`, `dual_converter_*`).

### `diode_based_models`
- Six-pulse diode (D-stack) rectifier, 400 V / 50 kVA transformer to 200 V secondary,
  340 V / 90 A DC output, LC filter with 100/150/250/350 Hz traps.
- Harmonic observer, RMS calculator and dq-PLL from C-Caller code.
- Note: `init_model.m` opens `six_pulse_diode_based_rectifier`, while the model in the folder is
  named `six_pulse_dstack_based_rectifier.slx` — the name has to be aligned before running.

### `vienna_rectifier`
- Three-phase three-level Vienna rectifier, 690 V, 250 kVA, fPWM = 4 kHz, LCL filter,
  thermal model enabled.
- DC-link voltage PI over dq vector PI or resonant PI, plus the zero-current and zero-sequence
  current loops characteristic of the topology.
- FHT-based dq-PLL, sequence extraction, moving-average P and Q, instantaneous power meter;
  CMOS-style deadtime generators for the T-type legs.
- Grid frequency selectable between 50 Hz and 300 Hz.

### `ac_ac_buck_converter`
- Direct AC/AC half-bridge buck chopper (no DC link) fed from a 230 V / 50 Hz single-phase grid,
  fPWM = 6 kHz, built with bidirectional switches (two anti-series IGBTs).
- Two switch arrangements are compared, the second using a common-drain realization; the
  modulation uses overlapping deadtime, documented in `images/modulator_strategy.pdf`.
- Open-loop study: the post-processing only plots input and output voltage and current.

---

## Grid emulation, PLLs and grid-side analysis

### `grid_emulator_400Hz`
- 400 Hz three-phase power supply / grid emulator: 90 kW, 200 V line, fPWM = 8 kHz, battery-fed
  320 V DC link, LCL filter into a 90 kVA 400 Hz output transformer.
- dq PI current and voltage loops, torque/I-F startup modes, oscillator Kalman filter and LPF/HPF
  bank; IGBT FF650R17IE4 data with RC snubber.
- `spectrum_analysis/` holds FFT scripts for output voltage and current, stack current and
  filter capacitor current.

### `three_phase_dqPLLs_analysis`
- Three-phase AFE bench dedicated to PLL behaviour under grid faults (FRT enabled, 690 V,
  1 MVA emulator).
- Three implementations compared side by side: plain dq-PLL, DDSRF-PLL, and the FHT-based
  dq-PLL (per-phase first-harmonic trackers), all through a double-integrator observer; PLL gain
  is reduced during faults.
- Grid-support logic: FRT detection, VDE 4110 reactive current references, islanding detector,
  positive/negative sequence decomposition.
- `video_plot/` animates the FHT dq-PLL response.

### `single_phase_dqPLLs_analysis`
- Control-analysis project with no power stage: comparison of the orthogonal-signal generators
  used to build a virtual dq frame in single phase — first-harmonic tracker, SOGI and
  phase-shift filter — each in Simulink and C-Caller form.
- `test_phase_shift_filter.slx` isolates the C-Caller phase-shift filter.

### `long_cable_effects`
- AFE + inverter + PMSM at 400 V with high-frequency models added: a lumped HF power-cable model
  (two cascaded sections with distributed capacitance and damping) plus common-mode inductances
  on both sides.
- Targets overvoltage/reflection at the motor terminals and common-mode phenomena (zero-sequence
  current, DC-link rail-to-ground voltage), in grid-connected and island mode.
- `data/` holds 25 measured oscilloscope traces sampled at 625 kHz with their plotting scripts,
  compared against the simulations.

---

## Modulation theory

### `space_vector_and model_predictive`
- Theoretical investigation of space vector modulation following Geyer and Rodriguez.
- `two_level_sv/sec.m` derives the two-level sector index from the signs of the phase voltages.
- `three_level_sv/analysis/tests_sv/` is a MATLAB reference implementation of the three-level
  algorithm: all 27 switching states, their αβ positions, the two adjacent vectors and the dwell
  times t1/t2/t0, plus the state-to-gate-pattern table.
- `three_level_sv/analysis/` compares NPC and T-type legs driven either by the Simulink
  `PWM Generator (3-Level)` or by the hand-built space-vector modulator, with DC-link balancing.
- `three_level_sv/model/` contains a full grid-tied AFE + inverter + PMSM system at 690 V used as
  the application of the modulator, including the FRT/LVRT logic.
- `pwm_modulators/single_phase_pwm/`, `mapping/` and `derivation/maps.nb` (Mathematica) hold the
  derivations and the space-vector maps.
- Note: despite the folder name, no model predictive control code is present here — MPC lives in
  `mpc_psm`, `mpc_ttype_psm` and `psm_sensorless_ctrl`.

---

## Shared scripts

### `scripts`
- `snubber_rlc_check.m` — RC snubber verification for a high-power three-phase diode or
  thyristor bridge: commutation inductance and overlap angle, di/dt at turn-off, reverse-recovery
  peak from Qrr and softness factor, snap-off RLC transient and repetitive peak voltage, Mohan
  normalization, snubber losses, and R/C sweeps. Worked case: 690 V, 4000 A DC, 4.5 MVA
  transformer at uk = 6 %. References McMurray (1972) and Mohan.
- `some_calculus.m` — quick line and transformer impedance sizing for a 480 V / 60 Hz
  application (50 kA short-circuit line inductance, 1250 kVA transformer at Ucc = 5 %).
