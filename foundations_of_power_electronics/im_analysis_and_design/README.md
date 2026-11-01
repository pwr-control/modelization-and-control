# im_analysis_and_design — induction machine drive with field-oriented control

261 kW, 460 V, 60 Hz, six-pole squirrel-cage induction machine (ABB M3BP 355MLB 6) fed by a three-phase
two-level inverter (4 kHz PWM with double update, control at 8 kHz, space vector modulation) through a
230 µH three-phase inductor, DC link 2 × 14.4 mF across a 1200 V lithium-ion battery. Indirect
field-oriented control with encoder: speed PI, rotor-flux PI, rotor-flux observer (current model in
rotor coordinates), x/y current PI with a voltage-vector limit. An open-loop u/f variant can replace the
vector control.

| File | Content |
|---|---|
| `init_model.m` | initialization: every number in the struct `param`, design figures, then it opens the model and checks the `param.*` references. Run it **from this folder** (the functions of `private/` are visible only to the files of this folder). The model to open is set at the top (`im_gyro_inv_model` by default). |
| `private/` | the parameter functions of this model (table below) |
| `im_gyro_inv_model.slx` | the model with the averaged inverter (`gyro_inverter`: controlled voltage sources on the AC side, controlled current sources on the DC side); the bridge of ideal switches is commented out |
| `im_inv_model.slx` | the same model with the bridge of six ideal switches (`inverter/ctrl/inverter/inverter`) active; same parameters |
| `inv_im_ekf.slx` | older copy of `im_inv_model`, **not migrated** (open point 12) |
| `graphics.mlx` | post-processing live script: plots of the logged x/y currents and voltages, estimated and actual fluxes, speed and torque (run it after a simulation). It read `Ns_inv*ts_inv` and `Tbez`, names no init defined: now `param.glb_time.Ns_inv*param.glb_time.ts_inv` (the logging window) and `param.im.machine.torque_nom` |
| `non_virtual_blocks_counting.m` | counts the non-virtual blocks of both models and of their referenced models (`find_system` with `LookUnderMasks` `all`, `FollowLinks` `on`, `Virtual` `off`) |

## The model

**Power stage.** Battery: library block `lithium_ion_battery`, 1200 V, 250 kW, 208 Ah (1C, assumption
of the init), initial SOC 0.85; R0 and R1 halved and C1 divided by 50 after the setup, as in the legacy
init. DC link: library block `dclink_with_center_tap` (instance `inverter/ctrl/dc_source/dclink`, two
14.4 mF banks in series, 107 kΩ measuring resistors `Rudc_measure`). Inverter:
`gyro_inverter` (averaged switching functions) in `im_gyro_inv_model`, six ideal switches with RC snubbers
(data set `silicon_high_power_ideal_switch`) in `im_inv_model`; no dead time. Output filter: library
three-phase inductor, `LFi` 230 µH, `RLFi` 1.8 mΩ, common-mode inductance `LFi_cm`. Machine: library
block `poliphase_induction_machine` (Simscape `im_uvw_2`, dq amplitude-invariant model) with the
equivalent circuit of `im_machine_param`: `Ls` 3.914 mH, `Lm` 3.763 mH, `Lr` 3.914 mH, σLs 0.295 mH,
`Rs` 2 mΩ, `Rr` 2.74 mΩ (rotor time constant 1.43 s), `J` 5 kg m²; the only load is the viscous friction
of the block, 0.75 `torque_nom` at synchronous speed (13.05 Nm s/rad, 1640 Nm). The load generator
`induction motor/load generator` is commented out.

**Control** (all at `ts` = 125 µs, per unit of the machine: currents in pu of `ibez` = 581.2 A, current
PI output in pu of `udc/√3`):

- speed reference: ramp from `time_start_motor_control` (0.25 s) to synchronous speed in 1 s, then the
  10 Hz second-order filter `SOF_10Hz`;
- speed: encoder angle, double-integrator observer `di_obs` (poles at 60 and 240 Hz); speed PI 2.5/18,
  output = y current reference limited to ±1.4 pu (`iq_lim`), anti-windup by back-calculation;
- rotor flux: reference `psi_r_ref`/max(ω, 1)² (field weakening above synchronous speed), flux PI 2/20
  with back-calculation, output `Lm_norm·isx` limited to [0.05, 0.6]·`Lm_norm`; the observer is the
  current model in rotor coordinates (`alpha_norm`, `Lm_norm`), its angle plus the encoder angle give
  the field angle;
- currents: αβ → x/y at the field angle, x and y PI 0.25/18, voltage vector limited to |u| ≤ 1, the
  integrators are frozen and decay at `CTRPIFF_CLIP_RELEASE` while |u| ≥ √3/2; no decoupling;
- x/y → αβ → SVM (C-Caller `sv_pwm`, input scaled by √3/2, udc input = constant 1) → library PWM
  generators;
- the flux loop and the modulator run from t = 0.

Variant `param.im.mode.use_uf_control = 1` (three `Variant Source` blocks in
`inverter/ctrl/im_ctrl/im_control`): fixed stator frequency `omega_ref·Omega_s_bez`, voltage step to
`uq_ref/5` at 25 ms and to `uq_ref` at 12 s, rate-limited (open point 11).

## How to run

1. Add the `library` repository to the MATLAB path with subfolders (`init_model.m` does it for
   `C:\github\library`): the library provides the Simulink/Simscape blocks, the C-Caller sources and the
   ideal-switch data sets other than the embedded one, no parameter.
2. Choose the model at the top of `init_model.m` (`im_gyro_inv_model` or `im_inv_model`) and run it from
   this folder: it fills `param`, prints the local functions of `private/`, the timing, the hardware and
   machine data and the design figures, opens the model, imports the C-Caller bus type
   `sv_pwm_output_t`, closes the scopes and checks that every `param.*` name read by the model exists
   (`check_model_param_refs`).
3. Run the simulation (`param.sim.simlength` = 3.25 s, variable step, `MaxStep` = `param.glb_time.tc` =
   1.25 µs, Simscape nonlinear iterations `param.sim.nonlinear_iterations` = 3).

## Parameter structure

| Group | Content |
|---|---|
| `param.sim` | `simlength` (`StopTime`), `tc` 1.25 µs (`FixedStep`), `use_thermal_model` (0; only sets `nonlinear_iterations` 3 → 5), `nonlinear_iterations` (Solver Configuration of `inverter/ctrl/dc_source`), `check_param_refs` |
| `param.time`, `param.glb_time` | `fpwm` 4 kHz, `trgo` 1 (double update), `dead_time` 0, `tc_factor` 100, `fpwm_dab`/`trgo_dab` (clock of the battery block); `glb_time = timing_param(param.time)` gives `fPWM_INV`, `tPWM_INV`, `ts_inv` 125 µs, `dead_time_INV`, `ts_dab`, `tc`, `Nc`, `Ns_inv`, `decimation_tc` (the names the blocks read) |
| `param.inv.hw` | `inv_hw_param(690, 250e3, fpwm)`: data set inv690V_250kW (`ubez`, `ibez`, `udc_nom` 1070 V, `f_nom`), filter (`LFi`, `LFi_cm`, `RLFi`, `RLFi_cm`), DC link (`CFi1`, `CFi2` 14.4 mF, `RCFi1`, `RCFi2`, `Rudc_measure` 107 kΩ); `udc_nom` is the `Vdc_scale` of the DC-link block |
| `param.inv.ideal_switch`, `param.dev` | ideal-switch data set (`device_ideal_switch_param`): `Vth`, `Rds_on`, `Vdon_diode`, `Rdon_diode`, `Csnubber`, `Rsnubber` (read by the six switches of `im_inv_model`) |
| `param.battery` | `nominal_battery_voltage_1` (also the mask default of the block), power, capacity, SOC; `pack_1 = battery_param(...)` (the fields of `lithium_ion_battery_setup`, read by the block) |
| `param.im.machine_in`, `param.im.machine` | nameplate, no-load and locked-rotor tests; `im_machine_param` (same formulas as `im_setup` of the library): equivalent circuit, rotor resistance fitted on the rated torque, bases (`ubez` 375.6 V, `ibez` 581.2 A, `omega_bez`, `psi_bez` 0.996 Vs), normalized quantities (`Lm_norm`, `alpha_norm`, `Jm_norm`, ...), `load_friction` |
| `param.im.load` | `Tload`, `timeRamp`, `start_load_ramp` of the commented-out load generator (placeholders, the legacy init never defined them) |
| `param.im.mode` | `use_vector_control` / `use_uf_control` (variant, exactly one at 1), `time_start_motor_control` |
| `param.im.ctrl` | `im_ctrl_param(ts, omega_bez, Jm_norm, gains)`: speed PI `kp_w`/`ki_w`/`iq_lim`, current PI `kp_i`/`ki_i` (and the copies `kp_inv_d`/`ki_inv_d`/`kp_inv_q`/`ki_inv_q` the blocks read), flux PI `kp_psi`/`ki_psi`/`isx_lim_min`/`isx_lim_max`/`psi_r_ref`, `di_obs` (encoder observer), `SOF_10Hz` (speed reference filter) |
| `param.ctrl` | `CTRPIFF_CLIP_RELEASE` |
| `param.im.uf` | `omega_ref`, `Omega_s_bez`, `uq_ref` of the u/f variant |
| `param.im.design` | informative figures printed by the init: per-unit chain, crossover and phase margin of the current, flux and speed loops, flux at the start of the ramp, steady state at synchronous speed |
| `param.meas` | ADC quantization (`adc12_quantization`, read by the current acquisition) and full scales |

### Functions of `private/`

| Function | Role |
|---|---|
| `timing_param(t)` | inverter clock domain and the clock of the battery block |
| `inv_hw_param(V, P, fpwm)` | inverter data set, bases, output filter, DC link (same data sets and formulas as `three_phase_inverter_hwdata` for the inverter and `three_phase_afe_hwdata` for the DC link, which the legacy model read) |
| `im_machine_param(m)` | equivalent circuit, rotor resistance, bases and normalized quantities of the machine (same formulas as `im_setup`) |
| `im_ctrl_param(ts, omega, Jm_norm, g)` | every control gain; encoder observer with closed-form pole placement and the 10 Hz filter (no toolbox) |
| `battery_param(...)` | lithium-ion pack (same formulas as `lithium_ion_battery_setup`) |
| `device_ideal_switch_param(name, fpwm, udc)` | ideal-switch data set (`silicon_high_power_ideal_switch` embedded, others through `device_data_load` of the library) |
| `check_model_param_refs(model, param)` | lists the `param.*` names read by the model and reports the missing ones |

## What changed in the models (November 2026)

- Block, mask and configuration parameters re-pointed from the flat names to `param.*`: 325 changes in
  `im_gyro_inv_model`, 309 in `im_inv_model`. Each model reads 84 distinct `param.*` names (the commented
  blocks included), plus `param.battery.nominal_battery_voltage_1` from the mask default of the battery
  block; all are defined by the init.

  | Legacy name | New name |
  |---|---|
  | `glb_time.*` (`ts_afe`, `Ns_afe` → `ts_inv`, `Ns_inv`) | `param.glb_time.*` |
  | `hwdata.inv.*`, `hwdata.afe.*` | `param.inv.hw.*` |
  | `im.*` | `param.im.machine.*` |
  | `im_ctrl.*`, `filters.inv.SOF_10Hz.*` | `param.im.ctrl.*`, `param.im.ctrl.SOF_10Hz.*` |
  | `lithium_ion_battery_1.*` | `param.battery.pack_1.*` |
  | `ideal_switch.*` | `param.inv.ideal_switch.*` |
  | `use_vector_control`, `use_uf_control`, `time_start_motor_control` | `param.im.mode.*` |
  | `uf_uq_ref`, `uf_omega_ref`, `Omega_s_bez` | `param.im.uf.uq_ref`, `omega_ref`, `Omega_s_bez` |
  | `Tload`, `timeRamp`, `start_load_ramp` | `param.im.load.*` |
  | `ts_inv`, `Tcontrol_INV`, `fPWM_INV` | `param.glb_time.ts_inv`, `param.glb_time.fPWM_INV` |
  | `CTRPIFF_CLIP_RELEASE`, `adc12_quantization`, `simlength`, `nonlinear_iterations` | `param.ctrl.*`, `param.meas.*`, `param.sim.*` |

- Literals typed in the blocks became parameters: the flux PI of `magnetization current reference/pi`
  (`2` → `param.im.ctrl.kp_psi`, `20*glb_time.ts_inv` → `param.im.ctrl.ki_psi*param.glb_time.ts_inv`,
  `0.6*im.Lm_norm` / `0.05*im.Lm_norm` → `param.im.ctrl.isx_lim_max/min*param.im.machine.Lm_norm`), the
  flux reference (`psi_r_ref` Constant, default 1 → `param.im.ctrl.psi_r_ref`) and the measuring divider
  of the DC link (`107e3` → `param.inv.hw.Rudc_measure`).
- `graphics.mlx`: `Ns_inv*ts_inv` → `param.glb_time.Ns_inv*param.glb_time.ts_inv`, `Tbez` →
  `param.im.machine.torque_nom` (the code cells only; the stored outputs are unchanged).
- Configuration set: `StopTime = param.sim.simlength`, `FixedStep = param.sim.tc` (it read `tc`, a name
  the legacy init never defined; the solver is variable-step), `MaxStep = param.glb_time.tc`.

The legacy init `init_model_legacy.m` (removed after the migration, still in the history of the repository)
built the shared template (seven clock domains, grid emulator, DAB/PSFBC/CLLC/ISOP rail, PMSM, UPQC): it no
longer ran against the library (`lithium_ion_battery_setup`
takes the charge capacity, `grid_fault_generator` is a function, the battery block reads
`param.battery.nominal_battery_voltage_1`).

**Checks done.** The parameter part of the new init was run in Octave without the library and compared with
the values of the legacy init computed with the library functions of `dev`: in each model the 98 distinct
expressions give 85 equal values and 0 different ones; the other 13 read names that the legacy init never
defined (`ts_inv`, `fPWM_INV`, `Tcontrol_INV` of the old `three phase pwm` blocks, the load generator, the
u/f variant, `FixedStep`). Every `param.*` name of the models resolves. The models were edited as XML and
not simulated here (no MATLAB).

## Open points found during the migration

The control was not changed. Numbers from the init and from an averaged check (machine dq model, the
discrete control of the model at 8 kHz with its observer, delays, anti-windup and limits, ideal averaged
inverter at 1200 V, ideal encoder).

1. **Start-up with an unmagnetized machine.** The flux loop starts at t = 0 with isx ≤ 0.6 pu and τr =
   1.43 s, the speed ramp at 0.25 s: the rotor flux is 0.21 pu at the start of the ramp and 0.81 pu at
   its end. Since the speed PI output is the y current (not a torque divided by the flux), the speed error
   reaches 0.14 pu, the y reference 1.17 pu and the current 529 A rms (1.29 pu of the machine, 370 A in the
   inverter data set); the speed overshoots to 1.07 pu. With `isx_lim_max` = 0.8 and
   `time_start_motor_control` = 1.0 s (`simlength` 4 s), parameters only: flux 0.88 pu at the start of the
   ramp, speed error 0.074 pu, peak current 369 A rms, flux overshoot to 1.095 pu.
2. **Flux PI.** kp 2 / ki 20: crossover 0.61 Hz, phase margin 31°. The back-calculation x = sat − kp·e
   leaves the integrator at the saturated output when the loop leaves saturation, hence the overshoot of
   point 1. With ki = kp/τr (zero on the rotor pole) the same anti-windup resets the integrator to a large
   negative value and the flux stalls. A feed-forward of `psi_r_ref` with a clamped integrator (kp 6, ki
   = kp/τr) gives a flux overshoot of 1.022 pu (block change).
3. **Torque reference.** isy_ref is the speed PI output: the speed-loop gain follows the rotor flux
   (start-up, field weakening). isy_ref = T_ref/max(ψ̂r, ψmin) removes the dependence.
4. **Field weakening.** `psi_r_ref`/max(ω, 1)² starts at synchronous speed, where the inverter output is
   0.61 pu (clip flag 0.87, limit 1), and the speed overshoot triggers it during the transient (flux
   reference down to 0.88 pu). A voltage-based weakening (or a knee at 1.2 pu or more with 1/ω) fits
   better.
5. **Voltage per unit.** 1 pu of the current PI output is `udc/√3` = 692.8 V (1.84 `ubez`): the udc input
   of the SVM is the constant 1, so the current-loop gain follows the battery voltage (−17 % at 1000 V).
   The DC-link measurement in pu (`vdc_pu_inv_mod1`, scaled by `udc_nom` = 1070 V) is read by no block;
   `u_im_scale` of the legacy init was read by no block either.
6. **Decoupling and delays.** No feed-forward of the cross terms and of the back-EMF (0.52 pu at
   synchronous speed, tracking error about 0.03 pu during the ramp); the applied voltage lags the field
   angle by 1–2 control periods (2.7° per period at 60 Hz) without ω·ts compensation.
7. **Clip flag.** The current PI integrators are frozen from |u| ≥ √3/2 while the vector limit is 1.
8. **Over-flux.** `psi_r_ref` = 1 pu = 0.996 Vs against Lm·i0·√2 = 0.958 Vs (0.962 pu) of the no-load
   test: 483 V rms line-to-line at the machine terminals at synchronous speed (460 V rated); the machine
   model is linear.
9. **Hardware data.** Inverter data set 690 V class (550 V, 370 A, `udc_nom` 1070 V) with a 460 V / 411 A
   machine and a 1200 V battery: the machine current exceeds the inverter rating and the modulation index
   is low (0.61 pu at rated speed). Dead time 0. In the library the 480 V data set has `u_nom` 400 V.
10. **Load.** Viscous friction only; the load generator is commented out and its names are placeholders
    in `param.im.load`; the slope of its ramp depends on `simlength`.
11. **u/f variant.** Never active in the legacy (names undefined); it is a fixed-frequency start at reduced
    voltage, and its second voltage step (12 s) is after the end of the run.
12. **inv_im_ekf.slx.** Older copy of `im_inv_model` with flat names that no init defines (`ts_inv`,
    `kp_w`, `iq_lim`, `kp_inv_d`, `Ldrso`, `dead_time`, `m_up`, ...); despite the name it has no Kalman
    filter of the machine (only the SOC estimator of the battery block). Not migrated; candidate for
    removal. The EKF of the induction machine is in `../inv_im/inv_im_param.slx` (`ekf_im_setup` of the
    library).
13. **Library definitions.** `im_setup` computes `torque_nom` = `pwr_nom/eta/omega_m_bez` = 2186 Nm (the
    shaft torque at the rated point is 2086 Nm) and `Jm_norm` = ½·`Jm·omega_m_bez/torque_nom` (half of
    the mechanical time constant); `Jm_norm` is read only by the EKF setup, not by these models.
