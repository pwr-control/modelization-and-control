# afe_inv_psm_cascaded — two AFE + inverter modules in parallel on one PMSM wind generator

Two identical back-to-back modules of 250 kW are connected in parallel on both sides ("hard parallel"):
on the grid side at the secondary of one 1 MVA Dyn11 transformer, on the machine side at the terminals of
one permanent magnet wind generator (two of the six 267 kW systems of the 1.6 MW, 17.8 rpm, 104-pole
WindGen generator, driven at fixed speed). Each module has an LCL filter on the grid side, a two-level AFE,
a 7.2 mF DC link with a brake chopper, a two-level inverter and a 230 µH output inductor. The bridges are
ideal switches, 4 kHz PWM with double update (control at 8 kHz), 3 µs dead time. Each converter has its own
local clock with an alignment loop to the master time.

The folder name says "cascaded", but the two modules are in parallel. The model file is
`afe_inv_psm_n.slx`.

| File | Content |
|---|---|
| `init_model.m` | initialization: every number in the struct `param`, design figures, then it opens the model, sets the mechanical setting, warns about the current controller of module 2 (open point 1) and checks the `param.*` references. Run it **from this folder** (the functions of `private/` are visible only to the files of this folder). |
| `private/` | the parameter functions of this model (table below) |
| `afe_inv_psm_n.slx` | the model (grid, two modules, machine, control, logging) |
| `non_virtual_blocks_counting.m` | counts the non-virtual blocks of the model (`find_system` with `LookUnderMasks` `all`, `FollowLinks` `on`, `Virtual` `off`) and lists the subsystems with most blocks |
| `init_model_legacy.m` | the old flat init, kept until the merge; it no longer runs with the library (see below) |

## The model

**Grid** (`grid`).
- `grid_emulator/three_phase_voltage_source_with_frt_emulator`: library emulator (ideal voltage sources,
  690 V, 50 Hz) with its fault generator (open point 6).
- `grid_emulator/3ph_Dyg`: Dyn11 transformer, 1 MVA, 690/690 V, made of three single-phase transformer
  blocks, star point of the secondary grounded. Turns 24/14 (`n1 = floor(n2·√3)`), so the secondary is at
  697 V (+1.0 %). Leakage from ucc 5 %: 75.8 µH, that is 76.6 µH and 16.0 mΩ referred to the secondary.
  Magnetizing inductance 145 mH (8.8 A), iron losses 1.8 kW.
- `power meter iec`, `threephase_vi_measures`.

**Modules** (`afe_abc_inv_psm_mod1`, `afe_abc_inv_psm_mod2`, the same power stage).
- `LCL_filter`: three-phase inductor `LFu` 500 µH (zero sequence `LFu_cm` 500 µH), then three capacitors
  `CFu` 200 µF with `RCFu` 50 mΩ in star, star point floating. The capacitors of both modules are on the
  same node, the transformer secondary: the grid-side inductor of the LCL is the transformer leakage, and
  the two modules share it.
- `afe`: two-level bridge `three_phase_inverter_ideal_switch_based_model` (data set
  `silicon_high_power_ideal_switch`: 1.5 mΩ, diode 2 V + 0.15 mΩ, snubber 2200 Ω + 10 nF). The IGBT and
  MOSFET bridges with thermal model are commented out (the legacy init did not switch them either). DC link
  `dclink1` 7.2 mF. `sv_pwm`: `Three phase two level PWM generator with global TRGO` (the space vector
  modulation is in the control).
- `brake`: chopper with `Rbrake` 4 Ω, on at 1.175·`udc_nom` = 1257 V, off at 1.15·`udc_nom` = 1230 V
  (literals in a MATLAB Function), 395 kW. It is enabled until `param.sim.time_aux_power_supply_fault`
  (1e3 s: always).
- `inverter/inverter`: the same bridge types (ideal switch active) and the same PWM generator.
- `L_filter`: three-phase inductor `LFi` 230 µH (`LFi_cm` 230 µH).
- `AFE_control/afe_control` and `INV_control/inverter_control`: the controls, triggered at both peaks of
  the carrier (below).

**Machine** (`pmsm/synchronous_motor`): library block `permanent_magnet_synchronous_machine` with the data
of one WindGen system (Rs 30.1 mΩ, Ld 6.24 mH, Lq 7.56 mH, magnet flux 3.774 Vs, 104 poles); the block
divides La, Lb and Rs by `param.psm.number_of_modules` = 2 and multiplies Jm and the friction by it, so it is
two systems in parallel. `fixed_speed_setting`: ideal speed source, 17.8 rpm (energy production, default).
`motor_load_setting` (speed-control variant) is commented out.

**Clocks** (`global timing`): `time_afe3` is the master clock (gain 1). Each converter has a local clock
`time = k·t` corrected by its alignment loop (`phase_compensation_1`): AFE gains 1.0002 / 1.0015, inverter
gains 1.0005 / 1.001. The corrections are used here (in `inv_psm` they were not).

**AFE control** (each module, 125 µs, pu of the AFE data set):
- acquisitions of the converter current, grid current and grid voltage: quantized and one sample late;
- PLL: four PLLs are in the model, one feeds the control: module 1 the C-Caller FHT-PLL (`dq_pll_mode4`),
  module 2 the Simulink FHT-PLL (`dq_pll_mode3_modn`);
- DC-link voltage PI → active current reference; PI of the grid current sequences; FRT (VDE 4110) →
  reactive current references;
- proportional-resonant controller of the converter current, one per phase (kp 0.6, ki 35, resonator at
  50 Hz, delta 0.015), output one sample late, no grid voltage feed-forward;
- zero-sequence current PI (kp 0.5, ki 18): only module 2 sends it to the modulator (`u_cm_comp_mod2` = −1,
  `u_cm_comp_mod1` = 0);
- space vector modulation: C-Caller `sv_pwm_cm` (`ctrl/space_vector`, with a common-mode input);
- moving averages of the sequences: C-Caller in module 1, Simulink in module 2.

**Inverter control** (each module, 125 µs, pu of one machine system):
- torque curve → q current reference (`torque_overload_factor` 1); MTPA block → d current reference (the
  same block as `inv_psm`, it sits on its limit −0.35 pu, open point 7);
- dq current PI with vector clip (kp 0.25, ki 35): module 1 the C-Caller `dqvector_pi`; module 2 see
  open point 1;
- angle and speed: EKF in both modules, encoder selected (`use_psm_encoder` = 1). This model has the newer
  `encoder-sensorless-switch` (the swapped speed inputs of `inv_psm` are not here);
- space vector modulation: C-Caller `sv_pwm` (`space_vector`), output one sample late.

Logging: `afe_measurement_sim`, `inverter_measurement_sim` (To Workspace), `scopes`.

## How to run

1. Add the `library` repository to the MATLAB path with subfolders (`init_model.m` does it for
   `C:\github\library`). The library gives the Simulink/Simscape blocks, the C-Caller sources and the
   device data sets other than the embedded ones, no parameter.
2. From this folder run `init_model.m`. It fills `param`, prints the local functions of `private/`, the
   bases and the design figures, opens `afe_inv_psm_n.slx`, imports the 16 C-Caller bus types, closes the
   scopes, sets the mechanical setting (`fixed_speed_setting` or `motor_load_setting`, by commenting
   blocks in and out as the legacy init did), warns when module 2 has no current controller (open point 1)
   and checks that every `param.*` name read by the model exists (`check_model_param_refs`).
3. Run the simulation (`param.sim.simlength` = 1.25 s, `ode23tb`, `MaxStep` and Simscape local solver step
   `param.glb_time.tc` = 0.625 µs = ts/200).

## Parameter structure

| Group | Content |
|---|---|
| `param.sim` | `simlength` 1.25 s, `tc` 0.625 µs, `use_thermal_model` (it only sets the Simscape nonlinear iterations, 3 or 5), `nonlinear_iterations`, `load_step_time` (speed control only), `time_aux_power_supply_fault`, `check_param_refs` |
| `param.time`, `param.glb_time` | `fpwm_afe`, `fpwm_inv` 4 kHz, `trgo_*` 1 (double update), `dead_time_*` 3 µs, `t_measure`, `tc_factor` 200; `glb_time = timing_param(param.time)` gives the names the blocks read: `fPWM_AFE`, `fPWM_INV`, `TRGO_AFE_double_update`, `TRGO_INV_double_update`, `ts_afe`, `ts_inv`, `dead_time_AFE`, `dead_time_INV`, `tc`, `decimation_tc`, `Nc`, `Ns_afe`, `Ns_inv` |
| `param.grid` | `application_voltage` 690 V, `f_nom`, `pwr_nom` 1 MVA, `ucc` 5 %, `eta`, `p_iron`, `n1`/`n2` 24/14, core data; `fault_in` (inputs of the fault test); `frt = grid_fault_param(fault_in)` (names read by the emulator: `Amplitude_U/V/W`, `Phase_shift`, `enable_frt_1/2`, `deepPOSxi` ... `deepNEGeta`, `start_time_LVRT`, `end_time_LVRT`, `t_ramp`); `emu = grid_emulator_param(grid, frt)`: `fgrid_nom`, `us_primary_nom`, voltage references, `ugrid_bez`, `igrid_bez`, `omega_grid_nom`, `sof_50Hz.ss_c`, `trafo.*` (the three transformer blocks) |
| `param.afe.hw` | `afe_hw_param(690, 250e3, fpwm)`: data set `afe690V` (`udc_nom` 1070 V, `ubez` 563.4 V, `ibez` 328.7 A = 250 kW/0.9), `LFu`/`LFu_cm` 500 µH, `RLFu`, `CFu` 200 µF, `RCFu` 50 mΩ, `CFi` 7.2 mF, `Rbrake` 4 Ω, `Rudc_measure` 1 MΩ |
| `param.afe.behav`, `param.inv.behav` | clock gains `time_gain_module_1/2`, PWM phase shifts `pwm_phase_shift_mod1/2`, `en_parallel_mode` and `u_cm_comp_mod1/2` |
| `param.afe.variant`, `param.afe.impl` | which PLL feeds the control in each module (`dq_pll_mode1..4`, `dq_pll_mode1_modn..4_modn`, exactly one per module), C-Caller or Simulink moving averages |
| `param.afe.ctrl` | `afe_ctrl_param(ts, omega_base, gains)`: udc PI, proportional-resonant controller `res_pi.*` (`Aresd`, `Bresd`, `Cresd`, `kp_rpi`, `ki_rpi`), sequence PIs, zero-sequence PI, PLL gains, limits of the current references, first harmonic tracker `fht_grid.*`, observers `di_obs.*`, `vr_obs.*`, rms windows |
| `param.afe.frt`, `param.afe.ref` | `k_frt_ref` 2, `settle_time` 0.175 s; active current references (their constant blocks are commented out: the udc PI gives the reference), reactive current steps |
| `param.inv.hw` | `inv_hw_param(690, 250e3, fpwm)`: data set `inv690V_250kW`, `LFi`/`LFi_cm` 230 µH, `RLFi`, `RLFi_cm` |
| `param.afe.ideal_switch`, `param.inv.ideal_switch`, `param.afe.igbt`, `param.inv.igbt`, `param.dev` | device data: ideal switch (bridges and brake chopper), Mitsubishi CM1200DW-34T embedded in `device_igbt_param` (bridges commented out), names of the data sets |
| `param.thermal` | `weight`, `no_weight`, `cp_al`, `Rth_*_HA` 15 mK/W, `Tambient` 40 °C (water), `DThs_init`: read by the brake resistor and by the commented-out IGBT bridges |
| `param.psm.machine`, `param.psm` | `psm_machine_param(machine_in)`: WindGen data, per-system values and bases (`ubez` 365.8 V, `ibez` 486.0 A, `tau_bez` 143.1 kNm); `number_of_modules` 2; `u_psm_scale`, `u_psm_scale_ekf`; `mode.*` (`use_torque_curve`, `use_speed_control`, `use_mtpa`, `use_psm_encoder`, `time_start_motor_control`); `impl.*` (which observer and which current controller is active in each module) |
| `param.psm.ctrl` | `psm_ctrl_param(...)`: speed PI, current PI `kp_i`/`ki_i`, `iq_lim`, `id_lim`, observer gains, `di_obs.*`, `ekf.*` |
| `param.load` | torque curve `rpm_base`/`torque_base` (per system), `torque_overload_factor` 1, `rpm_sim` 17.8, `omega_m_sim`, `external_load_inertia` |
| `param.ctrl` | `CTRPIFF_CLIP_RELEASE`, alignment PI `kp_align`/`ki_align`/`lim_up_align`/`lim_down_align`, `dmavg_filter_enable_time`, `mavarage_filter_frequency_base_order`, filters `filters.afe.*` (161 Hz) and `filters.inv.*` (1 Hz, 10 Hz, 50 Hz second order) |
| `param.meas` | ADC quantization (`adc_quantization` 1/2^11) and full scales |
| `param.design` | informative design figures (below) |

### Functions of `private/`

| Function | Role |
|---|---|
| `timing_param(t)` | the two clock domains (AFE and inverter) and the names the blocks read (formulas of `timing_setup`) |
| `grid_emulator_param(g, fault)` | emulator data, voltage references, bases, the Dyn11 transformer (formulas of `three_phase_transformer_setup`) and the 50 Hz second-order filter |
| `grid_fault_param(f)` | the fault test (replaces `frt_settings` + `grid_fault_generator` + `asymmetric_error`) |
| `afe_hw_param(V, P, fpwm)` | AFE data set, LCL filter, DC link, brake resistor, bases |
| `afe_ctrl_param(ts, omega_base, g)` | every AFE control datum (formulas of `ctrl_afe_setup`): resonant controller, first harmonic tracker, observers (closed-form pole placement) |
| `inv_hw_param(V, P, fpwm)` | inverter data set, output inductor, bases |
| `psm_machine_param(m)` | machine data, per-system values and bases (formulas of `pmsm_setup`) |
| `psm_ctrl_param(...)` | machine control gains, double integrator observer, EKF matrices (formulas of `ctrl_pmsm_setup` and `ekf_pmsm_setup`) |
| `device_ideal_switch_param`, `device_igbt_param` | device data sets |
| `check_model_param_refs(model, param)` | lists the `param.*` names read by the model and reports the missing ones |

## Old names and new names

| Old (flat workspace) | New |
|---|---|
| `glb_time.*` | `param.glb_time.*` (same names) |
| `psm.*` | `param.psm.machine.*` |
| `psm_ctrl.*` | `param.psm.ctrl.*` |
| `afe_ctrl.*` | `param.afe.ctrl.*` |
| `hwdata.afe.*`, `hwdata.inv.*` | `param.afe.hw.*`, `param.inv.hw.*` |
| `igbt.afe.*`, `igbt.inv.*` | `param.afe.igbt.*`, `param.inv.igbt.*` |
| `ideal_switch.*` | `param.afe.ideal_switch.*` (AFE bridge and brake chopper), `param.inv.ideal_switch.*` (inverter bridge) |
| `grid_emu.*` | `param.grid.emu.*` |
| `frt_data.*` | `param.grid.frt.*`; `frt_data.k_frt_ref` and `frt_data.settle_time` → `param.afe.frt.*` |
| `filters.afe.SOF_50Hz.ss_c` | `param.grid.emu.sof_50Hz.ss_c` |
| `filters.afe.*` (161 Hz), `filters.inv.*` | `param.ctrl.filters.afe.*`, `param.ctrl.filters.inv.*` |
| `CTRPIFF_CLIP_RELEASE`, `kp_align`, `ki_align`, `lim_up_align`, `lim_down_align`, `dmavg_filter_enable_time`, `mavarage_filter_frequency_base_order` | `param.ctrl.*` |
| `simlength`, `nonlinear_iterations`, `load_step_time`, `time_aux_power_supply_fault` | `param.sim.*` |
| `tc` | `param.glb_time.tc` (blocks), `param.sim.tc` (`FixedStep`) |
| `adc_quantization` | `param.meas.adc_quantization` |
| `Tambient`, `DThs_init`, `Rth_switch_HA`, `cp_al`, `weigth` | `param.thermal.*` (`weigth` → `weight`) |
| `afe_pwm_phase_shift_mod1/2`, `time_gain_afe_module_1/2`, `u_cm_comp_mod1/2` | `param.afe.behav.pwm_phase_shift_mod1/2`, `.time_gain_module_1/2`, `.u_cm_comp_mod1/2` |
| `inv_pwm_phase_shift_mod1/2`, `time_gain_inv_module_1/2` | `param.inv.behav.pwm_phase_shift_mod1/2`, `.time_gain_module_1/2` |
| `ixi_ref_mod1/2`, `i_react_pos_ref_1..3`, `time_i_react_pos_ref_1/2` | `param.afe.ref.*` |
| `use_dq_pll_mode1..4`, `use_dq_pll_mode1_modn..4_modn` | `param.afe.variant.dq_pll_mode1..4`, `.dq_pll_mode1_modn..4_modn` |
| `use_moving_average_from_ccaller_mod1..3` | `param.afe.impl.*` |
| `number_of_modules` | `param.psm.number_of_modules` |
| `omega_m_bez` | `param.psm.machine.omega_m_bez` |
| `rpm_base`, `torque_base`, `torque_overload_factor`, `rpm_sim`, `omega_m_sim`, `external_load_inertia` | `param.load.*` |
| `u_psm_scale_ekf` | `param.psm.u_psm_scale_ekf` |
| `use_torque_curve`, `use_speed_control`, `use_mtpa`, `use_psm_encoder`, `time_start_motor_control` | `param.psm.mode.*` |
| `use_ekf_bemf_module_N`, `use_observer_from_*_module_N`, `use_current_controller_from_*_module_N` | `param.psm.impl.*` |

## What changed in the model (2 November 2026)

- 1433 block, mask, variant and configuration parameters re-pointed from the flat names to `param.*`
  (table above): 895 block parameters, 431 values of library masks, 84 variant conditions, 20 mask
  parameters, 3 configuration entries; 1134 of them in active blocks, 299 in commented-out blocks. The
  model reads 273 distinct `param.*` names, all defined by the init. Same `ModelUUID` (the model itself is
  re-pointed, there is no copy).
- Configuration set: `StopTime = param.sim.simlength`, `FixedStep = param.sim.tc`,
  `MaxStep = param.glb_time.tc`.
- Two literals became parameters, in both modules: `Rudc_measure` `1e6` of the AFE DC link →
  `param.afe.hw.Rudc_measure`, and `double_sampling` `1` of the AFE PWM generator →
  `param.glb_time.TRGO_AFE_double_update` (the inverter PWM generator already read the TRGO name).
- Names that the legacy init never defined, all in commented-out blocks: `weigth` (heatsink mass of the
  IGBT bridges) → `param.thermal.weight`; `tc` (IGBT bridges, scope `inverter_voltage_ee`) →
  `param.glb_time.tc`; `omega_m_bez` (`speed_reference_generator/Step1`) → `param.psm.machine.omega_m_bez`.
  The configuration also had `FixedStep = tc` (not used by the variable-step solver): now `param.sim.tc`.
- `inverter_measurement_sim/module_1/From6` and `To Workspace12` are commented out: they read the tag
  `ileg_dc_inv_mod1`, which has no Goto in the whole model (a From without its Goto stops the update of the
  diagram).
- The legacy init also defined many names that this model does not read (timing and hardware of the
  single-phase inverter, DAB, PSFB, ISOP rail and CLLC, modules 3 and 4, induction machine, battery,
  UPQC transformer, ...). The new init keeps only what the model reads.

The legacy init `init_model_legacy.m` does not run with the `dev` library: `grid_fault_generator` is now a
function with arguments and `lithium_ion_battery_setup` takes the charge capacity.

**Checks done.** The parameter part of the new init was run in Octave without the library on the path and
compared with the legacy init run with the library functions of `dev` (with the old script version of
`grid_fault_generator`, and the battery call skipped: the model does not read the battery): the
305 distinct expressions of the model give the same values (295 identical, 6 within 1e-12, worst relative
difference 3e-14); four more could not be evaluated in the legacy (`weigth`, `tc` twice, `omega_m_bez`:
names the legacy never defined, see above). Every `param.*` name of the model resolves. The slx keeps the
order, compression and dates of its zip entries; all changed XML parts parse. The model was not simulated
here (no MATLAB).

## Design figures

From the init (`param.design`) and from the design check (averaged models of the loops, exact discrete
time with the delays of the model; script and figures are not part of the repository):

| Quantity | Value |
|---|---|
| AFE bases | `ubez` 563.4 V peak, `ibez` 328.7 A peak, 1 pu power 277.8 kW (250 kW/0.9), `udc_nom` 1070 V; control voltage 1 pu = `udc/√3` = 618 V |
| machine bases (one system) | `ubez` 365.8 V peak, `ibez` 486.0 A peak, `tau_bez` 143.1 kNm, 15.43 Hz |
| operating point at 17.8 rpm | torque curve 125 kNm per system → 233 kW per module; grid current 273 A peak per module (0.83 pu), converter voltage 0.92 pu |
| LCL resonance | 1381 Hz with one module alone on the transformer; 1039 Hz with the two modules (each one sees twice the transformer leakage); 977 Hz with one AFE stopped |
| AFE current loop | crossover about 275 Hz; loop delay 2.5 samples (1 measurement + 1 output + 0.5 PWM hold): it damps a resonance only below fs/10 = 800 Hz |
| DC-link voltage loop | crossover 11.1 Hz, phase margin 74° |
| zero-sequence current loop | crossover 8.6–19 Hz, phase margin 68–77° (it changes with the zero-vector time) |
| inverter current loops | machine (sum of the two modules): 13–14 Hz, phase margin 32–35°; circulating current between the two inverters (230 µH only): 223 Hz, phase margin 60°, gain margin 11 dB |
| brake chopper | on at 1257 V, off at 1230 V, 4 Ω → 395 kW (1.7 times the module power) |
| resonator of the PR controller | pole radius 1.000182 with forward Euler (open point 4) |

## Open points found during the migration

The control was not changed; these are the points to look at. Numbers from the init and from the design
check (`design_check_afe_inv_psm_n.py` and its output in the project files, folder
`afe_inv_psm_cascaded/`).

1. **Inverter of module 2 without a current controller.** `use_current_controller_from_simulink_module_2`
   = 0 (legacy value) selects the C-Caller `dq_vector_pi_ccaller` in module 2, and that block is commented
   out: the voltage reference of the inverter of module 2 is zero. The machine EMF then drives a current
   through the machine and the 230 µH: about 1029 A peak in module 2 (2.12 pu of one system), the machine
   gives 27 kNm instead of 265 kNm (10 %) and takes 50 kW from the shaft instead of 494 kW; module 2 sends
   nothing to its DC link. The Simulink controller was probably meant: the C-Caller `dqvector_pi` keeps one
   global state (`dqpi_ctrl` in `dqvector_pi_simulink.c`), so only one instance can run in a model, and module
   2 uses the Simulink versions everywhere else (FHT-PLL, moving averages). The init keeps the legacy value
   and prints a warning. Fix: set the flag to 1.
2. **AFE current loop: LCL resonance with little damping.** With one module alone the resonance (1381 Hz)
   would be above 800 Hz too, but with 5.4 % of passive damping the loop keeps 3.5 %. With the two modules
   the resonance moves to 1039 Hz and the least damped mode of the loop is at 1078 Hz with 0.9 % damping
   (passive 4.5 %); the `kp_rpi` limit is 0.71 against 0.6 now (gain margin 1.4 dB). The loop gain grows
   with `udc` (1 pu = `udc/√3`): at the brake chopper level (1257 V) the damping is 0 (limit 0.60). The
   legacy init raised `kp_rpi` from 0.2 (library) to 0.6 "for LVRT"; at 0.2 the damping is 3.6 %. Options:
   `kp_rpi` 0.4 (2.4 %); 1.5 samples of delay instead of 2.5 (6.3 %, limit 1.72); `RCFu` 200 mΩ (9.6 %,
   384 W per module at 50 Hz plus the ripple); active damping (capacitor current or grid current
   feedback). The circulating current between the two AFEs (500 µH only) is well damped (46 %). One AFE
   stopped: 2.4 %.
3. **Inverter current loop: slow, and limited by the parallel connection.** `kp_i` 0.25, `ki_i` 35 give
   13–14 Hz with 32–35° of phase margin for the machine current (as in `inv_psm`). Here `kp_i` cannot go
   much higher: the current that circulates between the two inverters sees only the 230 µH, and with the
   same gains its loop becomes unstable above `kp_i` 0.89. The gains suggested for `inv_psm` (5.3, 500) are
   unstable here. With `kp_i` 0.4 (`ki_i` 56): machine 17–19 Hz, 38–41°; circulating current 356 Hz, 47°,
   6.9 dB. A faster machine loop needs a separate control of the sum and of the difference of the two
   module currents, or more inductance in the difference path.
4. **Resonator of the PR controller.** Forward Euler (as the library): |z| = 1.000182 > 1 because delta
   0.015 < ω·ts/2 = 0.0196, so the resonator alone grows (time constant 0.69 s). In closed loop the 50 Hz
   pair is stable, but the resonator has no anti-windup: when the voltage is clipped (deep fault) its state
   grows. The exact discretization `expm(Ares·ts)` gives |z| = 0.999411; with forward Euler delta ≥ 0.029
   would also do. The block and its setup are in the library.
5. **No grid voltage feed-forward.** The `From v_grid_ff_abc` goes only to a scope, and the Add of
   `antiwind_up` adds a Constant 0. The controller builds the whole converter voltage: the inner loop
   alone tracks the 50 Hz current with 29 A peak of error on 275 A (10 %) with the forward-Euler
   resonator, and 70 A (25 %) with the exact one; the outer loops (udc PI, sequence PIs) remove the slow
   error. If point 4 is fixed, add the feed-forward at the same time.
6. **Fault test.** The abc fault (test 25.4, 75 % dip on two phases, type D) is off (`enable_frt_1` = 0);
   the sequence fault is on (`enable_frt_2` = 1). The emulator subtracts the `deep*` values from the
   pre-fault sequences, so from 0.75 s the positive sequence is 0.4 pu at +90° (a 90° phase jump) and the
   negative sequence 0.57 pu: at the transformer secondary the phase voltages are 0.97 / 0.40 / 0.61 pu.
   Please check that this is the test you want. The type D dip of test 25.4 in sequence form is
   `deepPOSxi` 0.375, `deepNEGxi` 0.375, `deepPOSeta` = `deepNEGeta` = 0 (0.25 / 0.875 / 0.875 pu at the
   emulator, as the abc fault). The fault ends at 1.665 s, after the end of the simulation (1.25 s): the
   recovery is not simulated, and the reactive current steps at 2.0 s and 2.5 s are never reached. With
   `k_frt_ref` 2 the FRT references ask 1.20 pu of positive-sequence reactive current (limit 1.0) and
   1.13 pu of negative sequence (limit 0.5). Figure `grid_fault.png`.
7. **MTPA block.** The same block as `inv_psm` (open point 1 there): gain `1/(Ld_norm − Lq_norm)/2` on the
   speed, so `id_ref` sits on −0.35 pu at any load (+6 % torque at rated load).
8. **Rating.** At 17.8 rpm the torque curve asks 233 kW per module; with the extra torque of the MTPA block
   each module takes about 247 kW from the shaft, for 250 kW converters. The AFE current is 0.83 pu at
   233 kW. The transformer is 1 MVA for 2 × 250 kW. The turns 24/14 put the secondary at 697 V (+1 %).
9. **Zero-sequence current (checked, fine).** The PI of module 2 has the right sign (negative feedback
   through `u_cm_comp_mod2` = −1 and the modulator), one controller for the one circulating path, 8.6–19 Hz
   and 68–77° of phase margin. It removes the DC and slow circulating current; differences at the switching
   frequency are limited only by the 1460 µH of the loop (zero-sequence inductors of both LCL filters and
   both output inductors).
10. **Name.** The folder is `afe_inv_psm_cascaded`, the modules are in parallel. The model has two modules;
    the legacy settings of modules 3 and 4 are not in the new init.
11. **Non-virtual blocks.** About 2100 in the model file by an estimate (library blocks counted as one,
    choices of the variants included; please check with `non_virtual_blocks_counting.m`). The AFE controls
    are the largest part (603 in module 1, 457 in module 2; the four PLLs alone 235 in module 1), then the
    inverter controls (370 and 306; the three back-EMF observers 184 in module 1).
12. **Not checked.** The PLLs, the FRT detection and timing (only the size of the references above), the
    sequence PIs, the islanding detector, the speed-control variant, the IGBT and MOSFET bridges with
    thermal model (commented out).
