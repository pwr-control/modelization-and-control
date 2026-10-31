# afe_dq_ttype — three-phase three-level T-type active front end with dq control

Three-phase three-level T-type active front end (IGBT Mitsubishi CM1200DW-34T, loss and thermal model)
between a 690 V grid and a split DC link (2 × 14.4 mF, centre tap = neutral point of the bridge) loaded by
a 275 kW resistor. 250 kW data set (bases with a 10 % margin), 4 kHz PWM with double update (control at
8 kHz), LCL filter made of the converter inductor, a star capacitor bank and the leakage of the grid
transformer. The DC-link voltage loop sets the active current, the dq vector PI controls the converter
current, the space vector modulation drives a three-level PWM.

| File | Content |
|---|---|
| `init_model.m` | initialization: every number in the struct `param`, design figures, then it opens the model and checks the `param.*` references. Run it **from this folder** (the functions of `private/` are visible only to the files of this folder). |
| `private/` | the parameter functions of this model (table below) |
| `afe_dq_ttype.slx` | the model: grid emulator and Dyn11 transformer, LCL filter, T-type bridge (IGBT with loss and thermal model; ideal-switch bridge commented out), split DC link, load, the whole control |
| `init_model_legacy.m` | the old flat init (shared template with seven clock domains and the DAB/PSFBC/CLLC/machine sections), kept for reference only: it no longer runs against the current library (see below) |

## The model

**Power stage.** The grid emulator (library block `three_phase_voltage_source_with_frt_emulator`: ideal
sources, amplitude ramp of 20 ms from t = 0 through a 50 Hz second-order filter, fault generator off)
feeds the primary of a Dyn11 1 MVA 690/690 V transformer made of three single-phase transformer blocks
(`Ld1` 113.7 µH, `Rd1` 11.9 mΩ, `Ld2` 37.9 µH, `Rd2` 11.9 mΩ, `Lm1` 144.8 mH, `Rfe1` 265 Ω, 24/14
turns: the secondary is at 697 V, +1.0 %), star point of the secondary grounded. Filter (`LCL_filter`):
three-phase inductor 500 µH (`RLFu` 3.9 mΩ), 3 × 200 µF in star with 50 mΩ in series (floating star
point), 3 × 1 µH, then the transformer: the leakage referred to the secondary is 77 µH and the filter
resonates at about 1.38 kHz. The bridge is the library block
`igbt_based_three_level_three_phase_inverter_ttype` with the CM1200DW-34T data set embedded in
`private/device_igbt_param.m` for the twelve switches (four per leg: two outer, two in the bidirectional
path to the neutral point; each commutation at `udc/2`). The DC link is the library block
`dclink_for_three_levels`: 2 × 14.4 mF, 107 kΩ measuring resistors (`Rudc_measure`), initial voltages
`udc_nom/2`; the load is a 4.16 Ω resistor across the whole link (275 kW at 1070 V).

**Control** (all at `ts = 125 µs`, triggered at the bottom and at the top of the carrier; every
acquisition is quantized and delayed by one sample):

- PLL: four PLLs run in parallel (Simulink dq PLL, C-Caller dq PLL, Simulink FHT-PLL, C-Caller FHT-PLL)
  and `Variant Source1/2` pick one; the active one is the C-Caller FHT-PLL (`param.afe.variant.dq_pll_mode4`);
- DC-link voltage PI 4/45 (`udc_ctrl`): reference step at 1 ms to 1 pu through a 20 ms filter, output =
  active current reference limited to ±1.65 pu, anti-windup by back-calculation;
- converter current: αβ→dq at the PLL angle, dq vector PI 0.25/35 on the converter-side current (the
  PMSM inverter gains of the legacy init), vector clip |m| ≤ 1 with integrator freeze; no grid-voltage
  feed-forward; reactive reference 0;
- zero-sequence current PI 0.5/18: its output reaches the modulator through `u_cm_comp_mod1` = 0, so it
  has no effect (the zero-sequence current has no low-impedance path anyway: the capacitor star is
  floating and the DC link reaches ground only through the high-value resistors of the DC-link block);
- dq→αβ, space vector modulation (C-Caller `sv_pwm_cm`, inputs scaled by √3/2: 1 pu of the controller
  output is `udc/√3` of phase voltage, the linear limit; centred zero vectors), the library three-level
  modulator (`three_phase_three_levels_PWM_modulator`, input bus `mu(k)`, `mv(k)`, `mw(k)`), the
  dead-time generator of the model (3 µs) and the gates of the bridge;
- the neutral-point voltage is not controlled.

## How to run

1. Add the `library` repository to the MATLAB path with subfolders (`init_model.m` does it for
   `C:\github\library`): the library provides the Simulink/Simscape blocks, the C-Caller sources and
   the device data sets other than the embedded one, no parameter.
2. From this folder run `init_model.m`: it fills `param`, prints the local functions of `private/`, the
   bases, the design figures and a loss estimate, opens `afe_dq_ttype.slx`, imports the C-Caller bus
   types (`first_harmonic_tracker_output_t`, `dqpll_grid_output_t`, `sv_pwm_cm_output_t`), closes the
   scopes and checks that every `param.*` name read by the model exists (`check_model_param_refs`).
3. Run the simulation (`param.sim.simlength = 1.25` s, `ode23tb` with `MaxStep` and the Simscape local
   solver step at `param.sim.tc` = 0.625 µs).

## Parameter structure

| Group | Content |
|---|---|
| `param.sim` | `simlength`, `tc` (0.625 µs: `FixedStep`, `MaxStep`, Simscape local solver step and logging base), `use_thermal_model` (only sets `nonlinear_iterations` 3 → 5 as in the legacy init), `nonlinear_iterations` (Solver Configuration of the grid emulator), `check_param_refs` |
| `param.time`, `param.glb_time` | `fpwm` 4 kHz, `trgo` 1 (double update), `dead_time` 3 µs, `tc_factor` 200; `glb_time = timing_param(param.time)` gives `fPWM_AFE`, `TRGO_AFE_double_update`, `dead_time_AFE`, `ts_afe` 125 µs, `tc`, `Nc`, `Ns_afe` (the names the blocks read). The legacy `glb_time.ts_inv` read by the current PI had the same value and is mapped to `ts_afe`. |
| `param.grid` | 690 V, 50 Hz, transformer data (`pwr_nom` 1 MVA, `ucc` 5 %, `eta` 95 %, `p_iron` 1.8 kW, turns, core geometry, `mur`); `frt = grid_fault_param('two_phase_D', 0.75, 0.915, 0.75, 0, 0)` (legacy test 25.4, both enables at 0); `emu = grid_emulator_param(param.grid, param.grid.frt)` (bases, references, `sof_50Hz` state space of the amplitude ramps, `trafo` = the fields of the three transformer blocks) |
| `param.afe.hw` | `afe_hw_param(690, 250e3, fpwm)`: bases (`ubez` 563.4 V, `ibez` 328.7 A, `udc_nom` 1070 V, `norm.*`), split DC link (`CFi1`, `CFi2` 14.4 mF, `Rudc_measure` 107 kΩ, `udc1_init`, `udc2_init`), filter (`LFu`, `LFu_cm`, `RLFu`, `RLFu_cm`, `CFu`, `RCFu`) |
| `param.afe.pwm` | `fpwm`, `double_update`, `dead_time`, `tc` (informative copy: the modulator reads `param.glb_time.*`) |
| `param.afe.behav` | `time_gain_module_1` (clock gain against the master time), `pwm_phase_shift_mod1`, `u_cm_comp_mod1` (gain of the zero-sequence controller output into the SVM, 0) |
| `param.afe.variant` | `dq_pll_mode1..4` (exactly one active; mode4 = C-Caller FHT-PLL) |
| `param.afe.ctrl` | `afe_ctrl_param(ts, omega_nom, gains)`: current PI `kp_i`/`ki_i`/`clip_release`, udc PI `kp_udc`/`ki_udc`/`udc_pu_ref`/`i_ref_lim`, zero-sequence PI `kp_is_z`/`ki_is_z`, `pll.*`, `align.*` (local time alignment), `fht.*` (first harmonic tracker model and observer gains), `di_obs.*` (phase observer), `rms.n1`/`n10` |
| `param.load` | `pwr_nom` 275 kW, `R` = `udc_nom^2/pwr_nom` (was the literal `1070^2/275e3` in the block) |
| `param.afe.design` | informative figures printed by the init: operating point and voltage headroom, LCL resonance against fs/6, crossover estimates, neutral-point ripple, switching ripple |
| `param.afe.igbt`, `param.dev` | IGBT data set (`device_igbt_param`, CM1200DW-34T embedded, other names through `device_data_load` of the library) |
| `param.meas` | ADC quantization (`adc_quantization` 1/2^11, read by every quantizer of the acquisitions) and full scales |
| `param.thermal` | heatsink object read by the bridge mask (`heatsink.no_weight`, `cp`, `Tambient`, `DThs_init`, `Rth_switch_HA`, `Rth_diode_HA`), `loss_estimate` (averaged losses and junction temperatures at the design point) |

### Functions of `private/`

| Function | Role |
|---|---|
| `timing_param(t)` | the converter clock domain (one domain: bridge and control) |
| `afe_hw_param(V, P, fpwm)` | hardware data and per-unit bases of the bridge, split DC link, filter |
| `afe_ctrl_param(ts, omega, gains)` | every control gain; FHT and phase-observer models with closed-form pole placement (no toolbox) |
| `grid_emulator_param(g, fault)`, `grid_fault_param(...)` | grid emulator bases and references, Dyn11 transformer data (same formulas as the library class `three_phase_transformer_setup` and the legacy core geometry), 50 Hz second-order filter, fault definition |
| `device_igbt_param(name, fpwm, udc)` | IGBT data set and derived thermal/snubber values |
| `check_model_param_refs(model, param)` | lists the `param.*` names read by the model and reports the missing ones |

## What changed in the model (October 2026)

- 342 block, mask and configuration parameters re-pointed from the flat names (`glb_time.*`,
  `hwdata.afe.*`, `grid_emu.*`, `frt_data.*`, `filters.afe.SOF_50Hz.*`, `afe_ctrl.*`, `psm_ctrl.*`,
  `igbt.afe.*`, `heatsink.*`, `CTRPIFF_CLIP_RELEASE`, `kp_align`, `adc_quantization`, ...) to `param.*`;
  the variant conditions read `param.afe.variant.*`. 114 distinct `param.*` names, all defined by the init.
- The current PI read `psm_ctrl.kp_i`/`ki_i` (the gains of the PMSM inverter of the shared template) and
  `glb_time.ts_inv`: now `param.afe.ctrl.kp_i`/`ki_i` and `param.glb_time.ts_afe` (same values).
- Configuration set: `StopTime = param.sim.simlength`, `FixedStep = param.sim.tc` (it read `tc`, a
  name the legacy init never defined), `MaxStep = param.glb_time.tc`; Solver Configuration
  `MaxNonlinIter = param.sim.nonlinear_iterations`, `LocalSolverSampleTime = param.glb_time.tc`.
- Two literals typed in the blocks became parameters: the measuring divider of the DC link
  (`107e3` → `param.afe.hw.Rudc_measure`) and the load (`1070^2/275e3` → `param.load.R`).
- Three-level modulator: the three lines into its input bus are now named `mu(k)`, `mv(k)`, `mw(k)`,
  the names the bus selector of the current library block reads (they carried the names `du(t)`,
  `dv(t)`, `dw(t)` of the `td2tc` filters).
- New `ModelUUID`.

The legacy init `init_model_legacy.m` calls `grid_fault_generator` as a script (it is now a function with
arguments) and builds the whole shared template: it does not run against the current library and is kept
only to read the old numbers.

**Checks done.** The parameter part of the new init was run in Octave without the library on the path and
compared with a reduced legacy init run with the library functions and classes of `dev`: the 126 distinct
expressions of the model evaluate to the same values (the 50 Hz filter of the emulator has a different
state-space realization with the same transfer function, and the integrators of the block start from zero,
so the behaviour is the same). Every `param.*` name of the model resolves. The model was not simulated
here (no MATLAB): open it, run the init and save it in Simulink.

## Open points found during the migration

The control was not changed; these are the points to look at. Numbers from the init and from an averaged
check of the loops (discrete current loop with the LCL and the transformer, the one-sample measurement
delay and the dq PI written in the stationary frame; udc loop with the resistive load; averaged start-up
with ideal PLL, no switching).

1. **LCL resonance and converter-current feedback.** The resonance (about 1.38 kHz) is just above fs/6 =
   1333 Hz: with converter-side current feedback and 1.5 samples of delay that is the side where the
   undamped loop is unstable. The loop is stable (crossover 135 Hz, phase margin 69°, gain margin 16 dB at
   1.5 kHz) because of the 50 mΩ in series with each filter capacitor: the LCL mode has a damping ratio of
   0.05; with 10 mΩ (closer to a film capacitor) it drops to 0.018, with 0 mΩ to 0.009, with no resistance
   at all it is slightly unstable. A higher `kp_i` reduces the damping (0.04 at kp 1, 0.01 at kp 1.5); a
   weaker grid (extra inductance) moves the resonance below fs/6 and keeps it damped.
2. **Current controller.** The gains 0.25/35 are those of the PMSM inverter of the shared template:
   crossover about 130 Hz, slow for a 4 kHz bridge but well damped. There is no grid-voltage
   feed-forward: the integrator carries the whole grid voltage (0.92 pu of modulation), so every change of
   the grid voltage becomes a current transient (a ramp of r pu/s leaves a current error of r/ki pu: 1.3 pu
   during the 20 ms start-up ramp of the emulator); faults (the FRT generator is off in this model) would
   be handled the same way.
3. **Start-up.** The DC link starts at `udc_nom` while the udc reference starts from 0 (step at 1 ms, 20 ms
   filter): the udc PI starts at −1.65 pu and the back-calculation leaves its integrator at +2.35 pu, the
   converter works as an inverter for about 7 ms, `udc` drops to about 720 V while the 275 kW resistor
   discharges the link, then the current controller, lagging the grid ramp, takes about 2.7 pu (870 A) at
   18 ms and `udc` overshoots to about 1130 V and settles in about 0.25 s (averaged estimate, ideal PLL: the
   PLL also has to lock on a grid voltage that starts from zero). A reference that starts from the measured
   `udc` (or a pre-charge sequence) and a grid-voltage feed-forward would remove most of it.
4. **Voltage headroom.** At 275 kW the converter needs 570 V peak of phase voltage against the SVM limit
   `udc/√3` = 618 V (|m| = 0.92, 8 % headroom); at no load the SVM saturates for a grid 8.5 % above
   nominal. The modulation is not divided by the measured `udc`, so the loop gain follows `udc`.
5. **Neutral point.** No balancing loop: with centred SVM the neutral-point current is the third harmonic
   Σ(1 − |m|)·i (54 A peak at 275 kW, 3.3 V peak-to-peak on 2 × 14.4 mF), but any asymmetry (dead time,
   unequal capacitances, offsets) makes the centre tap drift and only the 107 kΩ measuring resistors of
   the DC-link block act on it (τ of the order of 1500 s). The zero-sequence controller output (`ugamma`, `k_offset` of `sv_pwm_cm`) is the natural
   actuator for a balancing loop on `udc1 − udc2`; `u_cm_comp_mod1` is 0 and the zero-sequence current PI
   has nothing to control in a three-wire system.
6. **Bases and ratings.** `ibez` = 328.7 A comes from 250 kW with a 10 % margin: the 275 kW load takes
   322 A peak = 0.98 pu. The power-meter subsystem (`instantaneus_power`) multiplies the currents in pu of
   `ibez` by `grid_emu.igrid_bez` = 381.8 A (a fixed value of the library emulator class), so its active
   and reactive power read 16 % high; the control does not use it.
7. **Transformer ratio.** The turns 24/14 (`n1 = floor(14·√3)`) give 697 V on the secondary (+1.0 %).
8. **Semiconductors.** The same 1700 V / 1200 A module is used for the outer and for the middle switches
   (the middle path sees only `udc/2`); at 322 A peak the averaged estimate gives about 3 kW for the
   bridge (outer diode 271 W, middle IGBT 174 W, middle diode 48 W per leg), junction temperatures
   below 65 °C with 40 °C water, efficiency 98.7 % with the filter. Dead time 3 µs (td_off + tf of the
   module 1.4 µs): about 6.4 V of average voltage error at the current zero crossings (1.4 % of the phase
   voltage).
9. **Simscape solver.** `nonlinear_iterations` = 5 as in the legacy init. The new T-type bridges of
   `library/masked_models/semiconductors_bridges` (not used here) need at least 10 iterations, 20 by
   default, because two diodes switch in the same step; if this model is moved to them, raise
   `param.sim.nonlinear_iterations` and expect the error "Temperature must be greater than absolute zero"
   with 5.
10. **Model details.** The From `pwr_lim_enable` in `udc_ctrl` has no Goto (the switch keeps the PI
    output); the zero-sequence and power-meter subsystems are computed but not used by the control.
