# inv_psm — wind generator: inverter-fed permanent magnet synchronous machine

One of the six three-phase systems of a 1.6 MW, 17.8 rpm, 104-pole permanent magnet wind generator
(about 267 kW per system) feeds a two-level IGBT inverter (Infineon FF1200R17IP5, loss and thermal model,
4 kHz PWM with double update, 3 µs dead time) through a 230 µH inductor. The DC link (2 × 14.4 mF) sits
across a 1070 V lithium-ion battery. The shaft turns at a fixed speed; the control takes the torque from
the torque curve of the turbine and runs a dq current control with a sensorless angle from an extended
Kalman filter (EKF).

| File | Content |
|---|---|
| `init_model.m` | initialization: every number in the struct `param`, design figures, then it opens the model, picks the active inverter and checks the `param.*` references. Run it **from this folder** (the functions of `private/` are visible only to the files of this folder). |
| `private/` | the parameter functions of this model (table below) |
| `inv_psm.slx` | the model (power stage, machine, control, logging) |
| `non_virtual_blocks_counting.m` | counts the non-virtual blocks of the model (`find_system` with `LookUnderMasks` `all`, `FollowLinks` `on`, `Virtual` `off`) and lists the subsystems with most blocks |
| `plotting_results_inv.m`, `power_losses_calculus.m`, `spectrum.m`, `plotting_results_dclink.m` | post-processing (see open point 18) |
| `init_model_legacy.m` | the old flat init, kept until the merge; it no longer runs with the library (see below) |
| `doc/`, `figures/` | documents and figures of the original work |

## The model

**Power stage.**
- `battery_1/litium_ion_battery`: library lithium-ion battery (296 cells, open-circuit voltage model,
  state-of-charge estimator at `ts_dab` = 41.7 µs) across the whole DC link. `battery_2` is commented out.
- `inv_psm_mod1/dc_source/dclink`: 2 × 14.4 mF in series with the 107 kΩ measuring resistors.
- `inv_psm_mod1/inverter/inverter`: three inverter blocks, one active (the init comments the other two):
  `three_phase_inverter_igbt_adv_thermal_model` (default: six `igbt_simple_2th` cells with loss and thermal
  model, FF1200R17IP5 data), `three_phase_inverter_ideal_switch_based_model` (`param.sim.use_thermal_model = 0`),
  `three_phase_inverter_mosfet_based_with_thermal_model` (`param.sim.use_mosfet_thermal_model = 1`, values
  typed in the block). PWM: `Three phase two level PWM generator with global TRGO`, 4 kHz, 3 µs dead time.
- `inv_psm_mod1/output_filter`: three-phase inductor 230 µH (`RLFi` 1.8 mΩ).
- `pmsm/synchronous_motor`: library block `permanent_magnet_synchronous_machine` (Simscape component
  `pmsm_uvw_with_harmonics`), per system Rs 30.1 mΩ, Ld 6.24 mH, Lq 7.56 mH, magnet flux 3.774 Vs,
  104 poles, no 5th/7th harmonics.
- `fixed_speed_setting`: ideal speed source, step to 17.8 rpm at t = 0 through a 0.1 s lag (energy
  production, default). `motor_load_setting`: inertia and load torque for the speed-control variant.

**Control** (all at `ts` = 125 µs, triggered at both peaks of the carrier, quantities in pu of the
machine of one system):
- current acquisition: quantized (12 bit) and delayed by one sample;
- torque reference: `torque_reference` = torque curve (`param.load.rpm_base`, `torque_base`) at the
  filtered speed (10 Hz) × −1.15 / `tau_bez`; it is used directly as the q current reference. Variant:
  speed PI (`Speed control`, `param.psm.mode.use_speed_control`);
- d current reference: block `MTPA _mod1` (see open point 1);
- current control: dq vector PI with clip (C-Caller `dqvector_pi`, gains `kp_i` 0.25, `ki_i` 35, limit
  |u| ≤ 1 pu = `udc/√3`), no decoupling, no back-EMF or udc feed-forward; the output is delayed by one
  sample (`output voltage transformation`) and goes to the C-Caller space vector modulator `sv_pwm`;
- angle and speed: `bemf_observer` has three variants (EKF by default, Simulink and C-Caller back-EMF
  observers); `encoder-sensorless-switch` picks between the encoder (double integrator observer on the
  encoder angle) and the observer (see open point 2);
- `phase_compensation_1`: local time alignment loop (its output is not used, open point 14);
- logging: `inverter_measurement_sim` (To Workspace), `scopes`.

## How to run

1. Add the `library` repository to the MATLAB path with subfolders (`init_model.m` does it for
   `C:\github\library`). The library gives the Simulink/Simscape blocks, the C-Caller sources and the
   device data sets other than the embedded ones, no parameter.
2. From this folder run `init_model.m`. It fills `param`, prints the local functions of `private/`, the
   bases and the design figures, opens `inv_psm.slx`, imports the C-Caller bus types
   (`bemf_obsv_output_t`, `dqvector_pi_output_t`, `sv_pwm_output_t`), closes the scopes, sets the active
   inverter and mechanical setting (by commenting blocks in and out, as the legacy init did) and checks that
   every `param.*` name read by the model exists (`check_model_param_refs`).
3. Run the simulation (`param.sim.simlength` = 2.25 s, `ode23tb`, `MaxStep` and Simscape local solver step
   `param.glb_time.tc` = 0.5 µs).

## Parameter structure

| Group | Content |
|---|---|
| `param.sim` | `simlength`, `tc` (0.5 µs), inverter choice `use_mosfet_thermal_model` / `use_thermal_model`, `nonlinear_iterations` (5 with a thermal model, 3 without), `load_step_time` (speed control only), `check_param_refs` |
| `param.time`, `param.glb_time` | `fpwm` 4 kHz, `trgo` 1 (double update), `dead_time` 3 µs, `t_measure` (20 electrical periods at 17.8 rpm, 1.30 s), `tc_factor` (ts/tc = 250), `fpwm_dab` 12 kHz (sample time of the battery estimator); `glb_time = timing_param(param.time)` gives the names the blocks read: `fPWM_INV`, `tPWM_INV`, `TRGO_INV_double_update`, `ts_inv`, `dead_time_INV`, `minimum_pulse_time_INV`, `delayINV_modB`, `fPWM_DAB`, `tPWM_DAB`, `TRGO_DAB_double_update`, `ts_dab`, `tc`, `decimation_tc`, `t_measure`, `Nc`, `Ns_inv` |
| `param.inv.hw` | `inv_hw_param(690, 250e3, fpwm)`: data set `inv690V_250kW` (550 V, 370 A rms, `udc_nom` 1070 V, `ubez`, `ibez`), `LFi` 230 µH, `RLFi`, DC link `CFi1`/`CFi2` 14.4 mF, `Rudc_measure` 107 kΩ |
| `param.inv.behav`, `param.afe.behav` | clock gains of the local timing blocks (`time_gain_module_2` = 1.001 for the inverter PWM, `time_gain_module_1` of the AFE clock, read by an unused block), `pwm_phase_shift_mod1` |
| `param.inv.igbt`, `param.inv.ideal_switch`, `param.dev` | device data: FF1200R17IP5 embedded in `device_igbt_param` (other names through `device_data_load` of the library), ideal switch data set, names of the data sets |
| `param.thermal` | heat sink (`weight`, `no_weight`, `cp_al`, `Rth_switch_HA`/`Rth_diode_HA` 15 mK/W, `Tambient` 40 °C = water, `DThs_init`) and the object `heatsink.*` read by the bridge masks |
| `param.battery` | `nominal_battery_voltage_1` 1070 V (also read by the battery mask as its default), `nominal_battery_power` 750 kW, `initial_battery_soc` 0.85, `charge_capacity_1` (assumption, open point 12), `pack_1 = battery_param(...)`, `pack_2` (block commented out) |
| `param.psm.machine` | `psm_machine_param(param.psm.machine_in)`: WindGen data, per-system values (`*_m`), bases (`ubez` 365.8 V, `ibez` 486.0 A, `tau_bez` 143.1 kNm, `omega_bez` 96.93 rad/s), pu values (`Rs_norm`, `Ld_norm`, `Lq_norm`, `Ls_norm`, ...), friction `load_friction_m` |
| `param.psm` | `u_psm_scale` (non-EKF observers), `u_psm_scale_ekf` (EKF: `udc_nom/√3/ubez`), `mode.*` (`use_torque_curve`, `use_speed_control`, `use_mtpa`, `use_psm_encoder`, `time_start_motor_control`), `impl.*` (which observer and which current controller implementation is active), `design` (informative figures) |
| `param.psm.ctrl` | `psm_ctrl_param(ts, omega_bez, Jm_norm, u_psm_scale, machine, gains)`: speed PI `kp_w`/`ki_w`/`iq_lim`, `id_lim`, current PI `kp_i`/`ki_i`, observer gains `emf_fb_p`/`emf_p`, `omega_th`, `omega_base`, `di_obs.*` (double integrator observer, closed-form pole placement), `ekf.*` (`A1_tilde_ekf` ... `A6_tilde_ekf`, `Bd_ekf`, `C_ekf`, `Qkalman`, `Rkalman`) |
| `param.load` | torque curve `rpm_base`/`torque_base` (per system), `torque_overload_factor` 1.15, shaft speed `rpm_sim` 17.8, `omega_m_sim` and `external_load_inertia` (speed control) |
| `param.ctrl` | `CTRPIFF_CLIP_RELEASE`, alignment PI `kp_align`/`ki_align`/`lim_up_align`/`lim_down_align`, filters `filters.inv.*` (1 Hz and 10 Hz first-order, 50 Hz second-order of the scope data) |
| `param.meas` | ADC quantization (`adc_quantization` 1/2^11 read by the quantizers) and full scales |

### Functions of `private/`

| Function | Role |
|---|---|
| `timing_param(t)` | the clock domain of the inverter and its control (and the sample time of the battery estimator) |
| `inv_hw_param(V, P, fpwm)` | hardware data and bases of the inverter, output inductor, DC link |
| `psm_machine_param(m)` | machine data, per-system values and bases (formulas of `pmsm_setup` of the library) |
| `psm_ctrl_param(...)` | every control gain, the double integrator observer, the EKF matrices (formulas of `ctrl_pmsm_setup` and `ekf_pmsm_setup`) |
| `battery_param(...)` | lithium-ion pack (formulas of `lithium_ion_battery_setup`) |
| `device_igbt_param`, `device_ideal_switch_param` | device data sets and derived thermal/snubber values |
| `check_model_param_refs(model, param)` | lists the `param.*` names read by the model and reports the missing ones |

## Old names and new names

| Old (flat workspace) | New |
|---|---|
| `glb_time.*` (`glb_time.Ns_afe` → `Ns_inv`) | `param.glb_time.*` |
| `psm.*` | `param.psm.machine.*` |
| `psm_ctrl.*` | `param.psm.ctrl.*` |
| `hwdata.inv.*`, `hwdata.afe.*` | `param.inv.hw.*` |
| `igbt.inv.*`, `igbt.afe.*` | `param.inv.igbt.*` |
| `ideal_switch.*` | `param.inv.ideal_switch.*` |
| `heatsink.*`, `Tambient` | `param.thermal.heatsink.*`, `param.thermal.Tambient` |
| `lithium_ion_battery_1.*`, `lithium_ion_battery_2.*` | `param.battery.pack_1.*`, `param.battery.pack_2.*` |
| `filters.inv.*` | `param.ctrl.filters.inv.*` |
| `CTRPIFF_CLIP_RELEASE`, `kp_align`, `ki_align`, `lim_up_align`, `lim_down_align` | `param.ctrl.*` |
| `simlength`, `nonlinear_iterations`, `load_step_time` | `param.sim.*` |
| `adc_quantization` | `param.meas.adc_quantization` |
| `inv_pwm_phase_shift_mod1`, `time_gain_inv_module_2` | `param.inv.behav.pwm_phase_shift_mod1`, `param.inv.behav.time_gain_module_2` |
| `time_gain_afe_module_1` | `param.afe.behav.time_gain_module_1` |
| `rpm_base`, `torque_base`, `torque_overload_factor`, `rpm_sim`, `omega_m_sim`, `external_load_inertia` | `param.load.*` |
| `u_psm_scale_ekf` | `param.psm.u_psm_scale_ekf` |
| `use_torque_curve`, `use_speed_control`, `use_mtpa`, `use_psm_encoder`, `time_start_motor_control` | `param.psm.mode.*` |
| `use_ekf_bemf_module_1`, `use_observer_from_*_module_1`, `use_current_controller_from_*_module_1` | `param.psm.impl.*` |

## What changed in the model (1 November 2026)

- 443 block, mask, variant and configuration parameters re-pointed from the flat names to `param.*`
  (table above). The model reads 165 distinct `param.*` names (plus `param.battery.nominal_battery_voltage_1`
  in the default of the battery mask), all defined by the init. Same `ModelUUID` (the model itself is
  re-pointed, there is no copy).
- Configuration set: `StopTime = param.sim.simlength`, `FixedStep = param.sim.tc`,
  `MaxStep = param.glb_time.tc`.
- The literal `107e3` of the DC link block became `param.inv.hw.Rudc_measure`.
- `speed_reference_generator/Step1` (commented out) read `omega_m_bez`, a name the legacy init never
  defined: now `param.psm.machine.omega_m_bez`.
- 13 blocks read Goto tags that do not exist in the default configuration; a From without its Goto stops
  the update of the diagram, so they are now commented out:
  `inverter_measurement_sim/module_1/From6` and `To Workspace12` (tag `ileg_dc_inv_mod1`, no Goto
  anywhere), `global timing/From9` (tag `local_time_corr_afe1`, no Goto anywhere), and ten blocks that read
  the voltages `inverter_voltage_ee_phase_u/v/w` written only inside the ideal-switch inverter
  (`inverter_measurement_sim/From2`, `From3`, `From4`, `To Workspace5`, `To Workspace6`, `To Workspace8`,
  `scopes/inverter_data/From56`, `From57`, `From58`, scope `inverter_voltage_ee`): the init turns these ten
  on when the ideal-switch inverter is active.
- Post-processing scripts: the old names `glb_time.*`, `Tambient`, `igbt.inv.name`, `Nc`, `tc`, `fPWM_AFE`
  became `param.glb_time.*`, `param.thermal.Tambient`, `param.dev.igbt_name`, `param.glb_time.Nc`,
  `param.glb_time.tc`, `param.glb_time.fPWM_INV`.

The legacy init `init_model_legacy.m` does not run with the `dev` library: `lithium_ion_battery_setup` now
takes the charge capacity as an argument, and the battery mask reads `param.battery.nominal_battery_voltage_1`.

**Checks done.** The parameter part of the new init was run in Octave without the library on the path and
compared with the legacy init run with the library functions of `dev` (the battery call adapted): the 179
distinct expressions of the model give the same values; four more could not be evaluated in the legacy
(the three configuration entries, checked by hand, and the bare `omega_m_bez` above). Every `param.*` name of
the model resolves. The slx keeps the order, compression and dates of its zip entries; all changed XML parts
parse. The model was not simulated here (no MATLAB).

## Design figures

From the init (`param.psm.design`) and from the design check (an averaged simulation of the whole chain,
with the EKF rebuilt block by block; not part of the repository):

| Quantity | Value |
|---|---|
| bases per system | `ubez` 365.8 V peak, `ibez` 486.0 A peak, `tau_bez` 143.1 kNm, 15.43 Hz, 1 pu power 266.7 kW |
| pu of the machine | Rs 0.040, Ld 0.804, Lq 0.974 (Lq/Ld = 1.21), Ls 0.889 |
| control voltage | 1 pu = `udc/√3` = 618 V at 1070 V; the battery is at 1114 V open circuit (soc 0.85) and at about 1133 V while it takes 282 kW |
| operating point at 17.8 rpm | torque reference −1.005 pu; MTPA block id −0.35 pu; torque −1.056 pu (EKF angle) to −1.067 pu (encoder angle), 282–285 kW, 366 A rms, |u| 0.63 pu |
| current loop (present gains) | crossover 14 Hz (d) / 13 Hz (q), phase margin 35° / 32°, loop delay 2.5 samples |
| EKF at the operating point | angle error about −2.9°, speed estimate 1.6 % low |
| IGBT bridge losses | about 6.2 kW (2.2 %), junction about 81 °C (IGBT) and 73 °C (diode) with 40 °C water |

## Open points found during the migration

The control was not changed; these are the points to look at. Numbers from the init and from the
averaged check (start-up and 2.25 s at 17.8 rpm as in the model, dead time and battery included).

1. **MTPA block.** `MTPA _mod1` computes `id = A − sqrt(A² + iq²)` with `A = speed/(2·(Ld_norm − Lq_norm))`.
   The sign is wrong (it should be `Lq_norm − Ld_norm`) and the input is the speed instead of the magnet
   flux (`psi_m_norm` = 1). With a positive speed the output sits on the limit `−id_lim` = −0.35 pu at any
   load: the textbook MTPA gives −0.167 pu at rated torque and 0 at no load. Effect: +6 % torque at any
   load, and at light load a d current that is not needed (at 10 % load the current is 3.6 times the
   necessary one).
2. **Encoder-sensorless switch.** The MATLAB Function `switch_enc2obs(phase_psm, omega_obs, phase_obs,
   omega_psm, omega_th)` gets the encoder speed on `omega_obs` and the observer speed on `omega_psm`: the
   two speed inputs are swapped. With `omega_th` = 0 the control always uses the EKF angle with the
   **encoder** speed (torque curve and MTPA), so the "sensorless" mode still uses the encoder.
3. **EKF state prediction.** The block has two copies of `A_ekf`. The copy of the covariance builds
   `I + ts·omega_bez·(A1 + A2·iα + A3·iβ + A4·ψα + A5·ψβ + A6·ω)`; the copy of the state prediction has no
   `omega_bez` on the A6 path. With `F·x` this still gives the right rotation and back-EMF terms, plus an
   extra 1/`omega_bez` (1 %): the speed estimate is 1.6 % low at rated speed (0.6 % with the textbook
   prediction `x + ts·omega_bez·f(x, u)`). Once point 2 is fixed this speed drives the torque curve, which
   is steep at 17.8 rpm (+25 % from 17 to 17.8 rpm): about −7 % torque (−3 % with the textbook
   prediction). The double integrator observer that follows the EKF angle (its outputs are terminated)
   gives a speed without this bias.
4. **EKF model errors.** At rated load the EKF angle is about 2.9° behind. Four errors partly cancel:
   saliency (the EKF uses `Ls = (Ld + Lq)/2`, the machine has Lq/Ld = 1.21) about −5°; voltage scale (the
   EKF scales its voltage with the nominal 1070 V, the battery is at about 1133 V, 5.6 % more) about +3°;
   dead time (not compensated, 13 V per leg) about +1°; the angle is two samples old about −1.4°. With
   `Lq + LFi` in the EKF model (active flux) and the voltage scaled with the measured `udc` (the input
   `udc_pu` of `EKF_bemf_observer` is there and terminated) the error drops to +0.5°. The torque is
   about 1 % lower than with the encoder angle now, but the error moves with the battery voltage and the load.
5. **Current loop.** `kp_i` 0.25, `ki_i` 35 give a crossover of 13–14 Hz with 32–35° of phase margin, at
   an electrical frequency of 15.4 Hz, without decoupling: a q step of 0.5 pu takes 23 ms to 90 % and
   pushes id by 0.36 pu. `kp_i` 5.3, `ki_i` 500 give 139–167 Hz, 66–68° of phase margin and at least
   13.5 dB of gain margin (with the 2.5 samples of delay of the model): 2.6 ms and 0.05 pu. Decoupling
   (`ω·L·i`) and back-EMF feed-forward are possible additions.
6. **q reference.** The q current reference is the torque reference itself: the reluctance torque is not
   taken into account, so the machine gives more torque than asked (+6 % with the MTPA block, +3 % with the
   textbook MTPA).
7. **DC voltage.** No udc feed-forward (the SVM C-Caller has no udc input: 1 pu is `udc/√3` of the actual
   battery voltage, so the current loop gain follows the battery, +6 % at 1133 V; harmless). The EKF
   voltage scale uses 1070 V (point 4).
8. **Delays.** Current measurement one sample, voltage output one sample, PWM hold half a sample: 2.5
   samples (312 µs). The same angle is used for the current and for the voltage, without compensation: the
   voltage is applied about 1° (encoder) to 2.4° (EKF) late at rated speed. Small at 15 Hz.
9. **Torque curve below 6 rpm.** The 1-D lookup table extrapolates (linear by default): at 0 rpm it gives
   +0.13 pu (motoring) until about 4 rpm during the start ramp. Clip the table or start the curve at the
   cut-in speed.
10. **Rating.** At 17.8 rpm the machine gives 282–285 kW and 366 A rms per system, for an inverter data set
    of 250 kW / 370 A rms (113 % power, 99 % current); the machine itself is 267 kW per system. The
    overload factor 1.15 is applied to the torque curve.
11. **Losses.** Analytical estimate at the operating point (FF1200R17IP5 data of the model, 4 kHz,
    1133 V): per IGBT 648 W (553 W switching), per diode 389 W, bridge 6.2 kW (2.2 %), junction about
    81 °C (IGBT) and 73 °C (diode) with `Rth_HA` 15 mK/W per device and 40 °C water. To compare with the
    simulation (`power_losses_calculus.m`).
12. **Battery capacity.** The old `lithium_ion_battery_setup` had 50 Ah fixed inside; the `dev` version
    takes it as an argument. The init uses `P/V` = 701 Ah (1C) as an assumption. It does not matter in
    2.25 s (the state of charge moves by 0.02 %); please give the real value.
13. **From blocks without Goto.** See "What changed": 13 blocks are commented out, ten of them follow the
    ideal-switch inverter. `ileg_dc_inv_mod1` and `local_time_corr_afe1` have no Goto in the whole model.
14. **Clocks.** The PWM runs on the local clock `time_inv1` with gain 1.001 (`time_gain_module_2`), so the
    carrier is at 4004 Hz while the control samples at exactly 8 kHz; the alignment PI
    (`phase_compensation_1/alignment_ctrl`) computes a correction that nothing uses. The sampling instant
    slides through the PWM period (4 Hz beat with the carrier), so the samples are no longer taken where the
    switching ripple crosses its mean value and they pick up part of the ripple (a few A here; more noise in
    the PI output with higher `kp_i`). If this emulates the clock drift on purpose, fine; otherwise set the
    gain to 1 or close the alignment loop. The AFE clock `time_afe1` is not used by this model.
15. **Back-EMF observers (not active).** They get `u_psm_scale = 2/3·udc_nom/ubez`, 15 % more than the
    scale of the EKF (`udc_nom/√3/ubez`, the right one for 1 pu = `udc/√3`).
16. **IGBT cells.** The mask field "Sample&hold time constant (<< on-time) [us]" gets
    `param.glb_time.tc` = 0.5e-6, read in µs: 0.5 ps. Probably meant `param.glb_time.tc*1e6` (0.5 µs) or
    the library default 1 µs. The commented library block next to it reads `igbt.Rth_diode_CH` (22.9 mK/W)
    for the diode heatsink-to-water resistance instead of `heatsink.Rth_diode_HA` (15 mK/W).
17. **PMSM mask (library).** The parameter `pp` has the label "pole pairs" (default 52), but the component
    uses it as the number of poles (`pp = number_poles/2` inside). The model passes 104, which is right.
18. **Post-processing.** `plotting_results_inv.m` and `power_losses_calculus.m` start with `clear` and
    `load sim_results_*.mat` (files not in the repository): save `param` with the results.
    `plotting_results_dclink.m` reads AFE signals that this model does not log (`t_ts_afe_sim`,
    `udc_pos_sim`, `udc_neg_sim`), `spectrum.m` reads `time_tc_sim` (not logged).
19. **Non-virtual blocks.** About 854 by an estimate with the rule of `non_virtual_blocks_counting.m`
    (please check in MATLAB): six IGBT cells 180, EKF 93, inactive Simulink back-EMF observer 54, inactive
    C-Caller back-EMF observer 42, inactive Simulink dq PI 43, PWM generator 53, battery 58. Possible cuts:
    the inactive implementations (about 140), the EKF as a MATLAB Function like the one of `inv_im` (about
    90), the AFE clock and the alignment loop (about 15).
20. **Not checked.** The speed-control variant (`use_speed_control`); there the machine friction
    (`load_friction_m`, 1 pu torque at rated speed) also loads the drive.
