# mpc_ttype_psm — wind generator: three-level T-type inverter with predictive current control

One of the six three-phase systems of a 1.6 MW, 17.8 rpm, 104-pole permanent magnet wind generator
(about 267 kW per system) feeds a three-level T-type inverter with ideal switches through a 230 µH
inductor. The split DC link (2 × 14.4 mF, neutral point at the center tap) sits across a 1200 V
lithium-ion battery. The shaft turns at a fixed speed; the control takes the torque from the torque curve
of the turbine and runs a finite-set model predictive control (FCS-MPC) that tests the 27 states of the
bridge every 125 µs and writes the gate commands directly (no modulator). Angle and magnet flux come from an
extended Kalman filter (EKF), sensorless; the speed comes from a double integrator observer on the EKF angle.
The control was fixed on 1 November 2026: see [Control fixes](#control-fixes-1-november-2026).

| File | Content |
|---|---|
| `init_model.m` | initialization: every number in the struct `param`, design figures, then it opens the model, sets the mechanical setting and checks the `param.*` references. Run it **from this folder** (the functions of `private/` are visible only to the files of this folder). |
| `private/` | the parameter functions of this model (table below) |
| `mpc_ttype_psm.slx` | the model (power stage, machine, control, logging) |
| `spectrum.m` | FFT of a phase current of the machine (`iuvw_psm_sim`) |
| `plotting_results_inv.m`, `power_losses_calculus.m` | post-processing of other models, not for this one (open point 8) |
| `init_model_legacy.m` | the old flat init, kept until the merge; it no longer runs with the library (see below) |

## The model

**Power stage.**
- `battery/lithium_ion_battery`: library lithium-ion battery (332 cells, 1249 V open circuit at state of
  charge 0.85, state-of-charge estimator at `ts_dab` = 41.7 µs) across the whole DC link (P to N). The
  Solver Configuration starts the simulation from steady state, so the DC link starts at the battery
  voltage.
- `inv_psm_mod1/dclink/dclink`: library block `dclink_for_three_levels`, 2 × 14.4 mF with a 1 MΩ measuring
  resistor across each capacitor; it gives `vdc1`, `vdc2` to the control in pu of `udc_nom` = 1070 V.
- `inv_psm_mod1/inverter`: the old library block `three_phase_three_level_ttype_inverter_ideal_switch_based_model`
  (twelve ideal switches, threshold 5.5 V, 1.5 mΩ, with diodes and RC snubbers). Per leg Q1 (P), Q2 and Q3
  (bidirectional switch to the neutral point), Q4 (N): P = Q1+Q2, O = Q2+Q3, N = Q3+Q4. Before the bridge,
  `deadtime_generator_for_ttype` filters the four gates (time constant `tc`), puts a CMOS interlock with
  3 µs dead time between Q1 and Q4, and delays the turn-on of all four gates by 3 µs (fix 9). The `timer`
  (4 kHz, double update) only gives the trigger of the control.
- `inv_psm_mod1/inverter_output_filter`: three-phase inductor 230 µH (`RLFi` 1.8 mΩ).
- `psm`: library block `permanent_magnet_synchronous_machine`, per system Rs 30.1 mΩ, Ld 6.24 mH,
  Lq 7.56 mH, magnet flux 3.774 Vs, 104 poles.
- `fixed_speed_setting`: ideal speed source, step to 17.8 rpm at 0.025 s through a 0.1 s lag (energy
  production, default). `motor_load_setting`: inertia and load torque for the speed-control variant.

**Control** (triggered every `ts` = 125 µs, quantities in pu of the machine of one system):
- acquisition: phase currents, DC voltages, encoder angle and speed quantized (12 bit) and delayed by one
  step;
- torque reference: `torque_reference` = torque curve (`param.load.rpm_base`, `torque_base`) at the speed
  filtered at 1 Hz, × −1 / `tau_bez`; it is the q current reference, id = 0
  (`current_reference_generator`, at the control angle). Variant: speed PI (`Speed control`,
  `param.psm.mode.use_speed_control`);
- predictive control: `inner_ctrl/mp control/mpc_pu_d`, MATLAB Function `mpc_ttype_gate`. The current and
  the DC voltages arrive one step late, so it first computes the current and `vdc1 − vdc2` of now with the
  vector of the last step (delay compensation). Then, for each state [−1 0 1]³, it computes the alpha/beta
  voltage from `vdc1` (P) and `vdc2` (N) times `u_scale = udc_nom/ubez` (pu of `ubez`), predicts the
  current of the next step with `L_norm = (Lq + LFi)/Lbez`, `R_norm = (Rs + RLFi)/Xbez`,
  `ts_pu = ts·omega_bez` and the back-EMF of the EKF, predicts `vdc1 − vdc2` with
  `np_gain = ibez/(CFi1·udc_nom·omega_bez)` and the neutral-point current of the state, and applies the
  state with the lowest
  `J = k_kalman·|i_ref − i_pred|² + k_dc·(vdc1 − vdc2)²_pred + k_sw·(number of level changes of the legs)`,
  weights 1, 1, 1e-5. It gives the observers the voltage (pu of `ubez`) of the vector applied two steps
  before: the vector that made the current measured one step before;
- angle and speed: `bemf_observer1` has three variants (EKF by default, Simulink and C-Caller back-EMF
  observers), all fed with that voltage and the same R and L as the predictive control; the predictive
  control always takes speed and flux from the EKF. The speed output of the EKF block comes from the double
  integrator observer on the EKF angle. `encoder-sensorless-switch` picks the angle and speed used by the
  current reference and the torque curve (encoder through a double integrator observer, or EKF);
- `phase_compensation_1`: local time alignment loop (gains 0, open point 2);
- logging: `inverter_measurement_sim` (To Workspace), `scopes`.

## How to run

1. Add the `library` repository to the MATLAB path with subfolders (`init_model.m` does it for
   `C:\github\library`). The library gives the Simulink/Simscape blocks and the C-Caller source of the
   back-EMF observer, no parameter.
2. From this folder run `init_model.m`. It fills `param`, prints the local functions of `private/`, the
   bases and the design figures, opens `mpc_ttype_psm.slx`, imports the C-Caller bus type
   `bemf_obsv_output_t`, closes the scopes, sets the mechanical setting (`fixed_speed_setting` or
   `motor_load_setting`, by commenting the blocks in and out as the legacy init did) and checks that every
   `param.*` name read by the model exists (`check_model_param_refs`).
3. Run the simulation (`param.sim.simlength` = 2.25 s, `ode23tb`, `MaxStep` and Simscape local solver step
   `param.glb_time.tc` = 0.625 µs, 20 nonlinear iterations).

## Parameter structure

| Group | Content |
|---|---|
| `param.sim` | `simlength` 2.25 s, `nonlinear_iterations` 20 (Solver Configuration), `load_step_time` (speed control only), `check_param_refs` |
| `param.time`, `param.glb_time` | `fpwm` 4 kHz, `trgo` 1 (double update: control at 8 kHz), `dead_time` 3 µs, `t_measure` (the whole simulation), `tc_factor` 200 (tc = 0.625 µs), `fpwm_dab` 12 kHz (battery estimator); `glb_time = timing_param(param.time)` gives the names the blocks read: `fPWM_INV`, `TRGO_INV_double_update`, `ts_inv`, `dead_time_INV`, `tc`, `Nc`, `Ns_inv`, `ts_dab` |
| `param.inv.hw` | `inv_hw_param(690, 250e3, fpwm)`: data set `inv690V_250kW` (550 V, 370 A rms, `udc_nom` 1070 V, `ubez`, `ibez`), `LFi` 230 µH, `RLFi`, split DC link `CFi1`/`CFi2` 14.4 mF, `Rudc_measure` 1 MΩ |
| `param.inv.behav`, `param.afe.behav` | clock gains of the local timing blocks: `time_gain_module_2` = 1.001 (control clock), `time_gain_module_1` (AFE clock, output not used) |
| `param.battery` | `nominal_battery_voltage_1` 1200 V (also read by the battery mask as its default), `nominal_battery_power` 250 kW, `initial_battery_soc` 0.85, `charge_capacity_1` (assumption, open point 6), `pack_1 = battery_param(...)` with R0, R1 halved and C1 / 50 as in the legacy init |
| `param.psm.machine` | `psm_machine_param(param.psm.machine_in)`: WindGen data, per-system values (`*_m`), bases (`ubez` 365.8 V, `ibez` 486.0 A, `tau_bez` 143.1 kNm, `omega_bez` 96.93 rad/s), pu values (`Rs_norm`, `Ld_norm`, `Lq_norm`, `Ls_norm`, `Lalpha_norm`, `Lbeta_norm`, ...) |
| `param.psm` | `u_psm_scale` = 1 (non-EKF observers), `u_psm_scale_ekf` = 1 (EKF): the voltage of the predictive control is already in pu of `ubez`; `mode.*` (`use_torque_curve`, `use_speed_control`, `use_psm_encoder`, `time_start_motor_control`), `impl.*` (which back-EMF observer is active), `design` (informative figures) |
| `param.psm.ctrl` | `psm_ctrl_param(ts, omega_bez, Jm_norm, u_psm_scale, machine, inv.hw, gains)`: predictive control `mpc.L_norm`, `mpc.R_norm`, `mpc.u_scale`, `mpc.np_gain`, `mpc.ts_pu` and weights `mpc.k_kalman`/`mpc.k_dc`/`mpc.k_sw`, speed PI `kp_w`/`ki_w`/`iq_lim`, observer gains `emf_fb_p`/`emf_p`, `omega_th`, `omega_base`, `di_obs.*` (double integrator observer, closed-form pole placement), `ekf.*` (`A1_tilde_ekf` ... `A6_tilde_ekf`, `A6_pred_ekf`, `Bd_ekf`, `C_ekf`, `Qkalman`, `Rkalman`, with `R_norm` and `L_norm`) |
| `param.load` | torque curve `rpm_base`/`torque_base` (per system), `torque_overload_factor` 1, shaft speed `rpm_sim` 17.8, `omega_m_sim` and `external_load_inertia` (speed control) |
| `param.ctrl` | alignment PI `kp_align`/`ki_align`/`lim_up_align`/`lim_down_align`, filters `filters.inv.*` (1 Hz first-order of the torque curve, 50 Hz second-order of the scope data) |
| `param.meas` | ADC quantization (`adc_quantization` 1/2^11 read by the quantizers) and full scales |

### Functions of `private/`

| Function | Role |
|---|---|
| `timing_param(t)` | the clock domain of the inverter and its control (and the sample time of the battery estimator) |
| `inv_hw_param(V, P, fpwm)` | hardware data and bases of the inverter, output inductor, split DC link |
| `psm_machine_param(m)` | machine data, per-system values and bases (formulas of `pmsm_setup` of the library, same as `inv_psm`) |
| `psm_ctrl_param(...)` | model data and weights of the predictive control, speed PI, observer gains, double integrator observer, EKF matrices (formulas of `ctrl_pmsm_setup` and `ekf_pmsm_setup`, with the fixes below) |
| `battery_param(...)` | lithium-ion pack (formulas of `lithium_ion_battery_setup`) |
| `check_model_param_refs(model, param)` | lists the `param.*` names read by the model and reports the missing ones |

## Control fixes (1 November 2026)

The design check of the migration found the control far from its design (torque 11.5 % below the torque
curve, current error 0.024 pu rms, the O level used 8.5 % of the time, gate overlaps). These are the fixes,
with the number of the old open point (migration README) in brackets.

| # | Problem | Fix |
|---|---|---|
| 1 | (1) The prediction used the voltage of a state in pu of `udc_nom` in a machine model in pu of `ubez`: the true voltage was 2.925 times the predicted one. | State voltage times `u_scale = udc_nom/ubez`, from `vdc1` (P) and `vdc2` (N) separately. |
| 2 | (4) No delay compensation: the prediction started from the current measured one step before, but the chosen vector acts one step later. With fix 1 missing too, the loop seen as a deadbeat loop had the poles `z² − z + g = 0`, g = 2.6 (q) and 3.1 (d): unstable. | Current and `vdc1 − vdc2` of now computed with the vector of the last step, then the choice. Poles 0 and 1 − g: g = 1.00 on q (deadbeat), 1.20 on d (pole −0.20). |
| 3 | (3) Neutral point: `k_dc = 0` (the first state of a pair of redundant small vectors always won, the neutral point drifted by hundreds of volts) and a fictitious capacitance `1e-3` in the term, 3070 times too strong. | `np_gain = ibez/(CFi1·udc_nom·omega_bez)` = 0.325, neutral-point current from the phase currents of the legs at O, `k_dc = 1`. |
| 4 | (9) Switching: states with almost the same current error (zero vectors, small vectors) were picked by their order in the table: thousands of direct P↔N jumps of a leg per second at partial load. | `k_sw = 1e-5` times the number of level changes of the three legs (O↔P, O↔N count 1, P↔N counts 2): it only breaks the ties. |
| 5 | (2, 7) EKF voltage: 3 (Gain2, Gain3) × output × `u_psm_scale_ekf` was √3 times the true voltage, and it was the vector chosen in this step, not the one that made the measured current. | Output of `mpc_ttype_gate` = voltage of the vector applied two steps before, in pu of `ubez`; Gain2, Gain3 = 1; `u_psm_scale_ekf` = 1. |
| 6 | (7) EKF inductance: `Ls` = 0.889 pu and `Rs` without the output inductor (the bridge sees 0.833 pu on d, 1.003 pu on q): −5.7° of saliency error at rated torque. | EKF with `L_norm = (Lq + LFi)/Lbez` and `R_norm`: its flux is then the "active flux" ψm + (Ld − Lq)·id, on the d axis for any id. |
| 7 | (5) EKF state prediction: the copy of `A_ekf` of the state prediction had the A6 path without `omega_bez`: ω·ψ 1 % too large in `F·x`, speed estimate 0.982. | `A6_pred_ekf = zeros(6)` on that path: `F_pred·x = x + ts·omega_bez·f(x, u)` (the covariance copy keeps `A6_tilde_ekf`). |
| 8 | (5) Speed estimate: the speed state of the EKF is still 0.5 % low (forward Euler makes the flux grow by (ω·ts)²/2 per step and the filter shares the error between flux and speed). On the steep torque curve at 17.8 rpm this is −2.3 % of torque. | Speed output of the EKF block from the double integrator observer on the EKF angle (block already in the EKF subsystem): 0.999, i.e. the 1.001 clock (open point 1). |
| 9 | (6) Gates: the library interlock is between Q1 and Q4 only. At each O→P (O→N) change Q1 (Q4) turned on 0.625 µs after the command while Q3 (Q2) was still above the 5.5 V threshold (5.52 V): one step of the local solver with a DC-link capacitor shorted. | Turn-on delay of 3 µs (`dead_time`) on the four gates of each leg, after the filter and the interlock (Transport Delay + MinMax: min(gate, delayed gate)). The gate that turns off is below 5.5 V after 2 steps (1.25 µs), the other one turns on after 3.75 µs. |
| 10 | (8) `udcd_pu` was `2·udc1d_pu` (From2 and From3 both read `udc1d_pu`). | From3 reads `udc2d_pu`. |
| 11 | (15) The two non-EKF observers got about 2.3 times the true voltage (3 × output × `udcd_pu` × `2/3·udc_nom/ubez`) and used `Ls`, `Rs`. | Their `udc` input is the constant 1 (`inner_ctrl/udc_one_observers`), `u_psm_scale` = 1, R and L of the predictive control. Not simulated in the check. |

**What changed in the slx** (14 XML parts, 25 new blocks with SIDs 42318–42342; everything else is
byte-identical):
- `inner_ctrl/mp control/mpc_pu_d` (chart 69): new script of `mpc_ttype_gate` (comments in English);
  five inputs renamed in place, same port order: `Lalpha_norm` → `L_norm`, `Rs_norm` → `R_norm`,
  `Lbeta_norm` → `u_scale`, `omega_bez` → `np_gain`, `ts_inv` → `ts_pu`. The five Constants of `mp control`
  read `param.psm.ctrl.mpc.*`; Gain2, Gain3: 3 → 1.
- `EKF_bemf_observer/extended_kalman_observer/A_ekf` (copy of the state prediction): Gain1 reads
  `param.psm.ctrl.ekf.A6_pred_ekf`. `extended_kalman_observer`: the output `omega_elec_ekf_hat` comes from
  the double integrator observer (output 2), the speed state goes to `Terminator1`.
- `inverter_control/From3`: `udc1d_pu` → `udc2d_pu`.
- `inverter/deadtime_generator_for_ttype/dead_time_generator_cmos_simulink_phu`, `_phv`, `_phw`: per gate a
  Transport Delay `delay_Qk` (`dead_time` of the mask) and a MinMax `turn_on_delay_Qk` between the From
  and the Bus Creator; Bus Creator and Outport moved 250 px to the right.
- `inner_ctrl`: the From `udcd_pu` of the observer block is replaced by the Constant `udc_one_observers`
  (value 1). `bemf_observer/psi_alpha_hat`, `psi_beta_hat`, `bemf_observer_ccaller`: `Rs_norm`, `Ls_norm`
  → `param.psm.ctrl.mpc.R_norm`, `L_norm`.

**Results of the check** (switched simulation of the whole chain, see below; values at 1.2–1.5 s, 17.8 rpm):

| Case | Control angle error | Speed estimate | Torque vs torque curve | Current error rms | max \|vdc1 − vdc2\| | Leg at O | P↔N changes/s | Switching per device |
|---|---|---|---|---|---|---|---|---|
| before the fix | −21.6° | 0.982 | −11.5 % | 0.024 pu | 80 V | 8.5 % | 5199 | 1467 Hz |
| fixed, sensorless | −1.2° | 0.999 | −0.4 % | 0.008 pu | 8 V | 47 % | 108 | 1532 Hz |
| fixed, encoder | −0.7° | 0.999 | 0.0 % | 0.008 pu | 9 V | 46 % | 138 | 1546 Hz |
| fixed, no dead time | −1.2° | 0.999 | −0.4 % | 0.008 pu | 9 V | 46 % | 122 | 1569 Hz |
| fixed, machine L +20 % | −10.6° | 0.999 | 0.0 % | 0.008 pu | 8 V | 46 % | 124 | 1559 Hz |
| fixed, machine L −15 % | +6.2° | 0.999 | −2.6 % | 0.009 pu | 9 V | 46 % | 186 | 1549 Hz |
| fixed, DC-link C −50 % | −1.2° | 0.999 | −0.4 % | 0.008 pu | 17 V | 45 % | 198 | 1598 Hz |
| fixed, 12 rpm | −0.7° | 0.999 | −0.3 % | 0.004 pu | 11 V | 51 % | 24 | 1429 Hz |
| fixed, machine L −20 % | lost after about 1 s (+13° ± 18°) | 0.999 | −6.4 % | 0.039 pu | 32 V | 37 % | 684 | 1424 Hz |
| fixed, machine L −20 %, encoder | −0.7° | 0.999 | 0.0 % | 0.009 pu | 9 V | 48 % | 246 | 1651 Hz |
| fixed, without the switching term (`k_sw` = 0) | −1.3° | 0.999 | −0.4 % | 0.008 pu | 6 V | 45 % | 812 | 2712 Hz |
| fixed, 12 rpm, `k_sw` = 0 | −0.8° | 0.999 | −0.3 % | 0.004 pu | 1 V | 33 % | 4655 | 2878 Hz |
| fixed, speed from the EKF state (without fix 8) | −1.2° | 0.994 | −2.7 % | 0.008 pu | 9 V | 47 % | 144 | 1524 Hz |

The switching term halves the switching (2712 → 1532 Hz per device at 17.8 rpm, 2878 → 1429 Hz at 12 rpm)
and removes almost all direct P↔N changes (812 → 108 and 4655 → 24 per second) for 2.5 % more current
error. "P↔N changes/s" counts the three legs; "switching per device" is the number of level changes of a leg
per second divided by 2 (two of the four switches of a T-type leg commutate at a time).

## Migration (1 November 2026)

- 326 block, mask, variant and configuration parameters re-pointed from the flat names to `param.*`
  (table below). Same `ModelUUID` (the model itself is re-pointed, there is no copy). Configuration set:
  `StopTime = param.sim.simlength`, `FixedStep = param.glb_time.tc` (not used by `ode23tb`),
  `MaxStep = param.glb_time.tc`. After the control fixes the model reads 106 distinct `param.*` names
  (plus `param.battery.nominal_battery_voltage_1` in the default of the battery mask), all defined by the init.
- `battery/Solver Configuration`: `MaxNonlinIter` 5 → `param.sim.nonlinear_iterations` = 20. In a T-type leg
  two diodes can switch in the same step; the tests of the library bridges need at least 10 (with 5 they
  stop with "Temperature must be greater than absolute zero").
- The literal `1e6` of the DC link block became `param.inv.hw.Rudc_measure`.
- Some blocks that were already commented out (`inverter_measurement_sim/To Workspace6`, `8`, `12`, `16`,
  `17`, `scopes/scopes_for_mosfet_based_inverter`, `speed_reference_generator/Step1`) and `FixedStep` read
  `tc`, `Nc` and `omega_m_bez`, names the legacy init never defined: now `param.*` too.
- 28 blocks are now commented out because they read Goto tags that do not exist anywhere in the model (a
  From without its Goto stops the update of the diagram): `global timing/From9` (`local_time_corr_afe1`,
  clock correction of an AFE that this model does not have), and the IGBT logging of a bridge that this
  model does not have (tags `igbt_*_invQ1/Q2`, `diode_*_invQ1`): `inverter_measurement_sim/From4`,
  `From12` … `From21`, `From25`, `From26`, `To Workspace18` … `To Workspace30`, and the subsystem
  `scopes/scopes_for_igbt_based_inverter`.
- `spectrum.m`: `Nc`, `tc` → `param.glb_time.Nc`, `param.glb_time.tc`.

The legacy init `init_model_legacy.m` does not run with the `dev` library: `lithium_ion_battery_setup` now
takes the charge capacity as an argument, and the battery mask reads `param.battery.nominal_battery_voltage_1`.
It also has the old control values (it does not know the new names of the fixes).

### Old names and new names

| Old (flat workspace) | New |
|---|---|
| `glb_time.*` (`glb_time.Ns_afe` → `Ns_inv`) | `param.glb_time.*` |
| `psm.*` | `param.psm.machine.*` |
| `psm_ctrl.*` | `param.psm.ctrl.*` |
| `psm_ctrl.ekf.k_kalman`, `psm_ctrl.ekf.k_dc`, `psm_ctrl.ekf.k_sw` | `param.psm.ctrl.mpc.k_kalman`, `k_dc`, `k_sw` |
| `hwdata.inv.*`, `hwdata.afe.CFi1`, `CFi2`, `udc_nom` | `param.inv.hw.*` |
| `lithium_ion_battery_1.*` | `param.battery.pack_1.*` |
| `filters.inv.*` | `param.ctrl.filters.inv.*` |
| `kp_align`, `ki_align`, `lim_up_align`, `lim_down_align` | `param.ctrl.*` |
| `simlength`, `load_step_time`, literal `5` of `MaxNonlinIter` | `param.sim.simlength`, `param.sim.load_step_time`, `param.sim.nonlinear_iterations` |
| `adc_quantization` | `param.meas.adc_quantization` |
| `time_gain_inv_module_2`, `time_gain_afe_module_1` | `param.inv.behav.time_gain_module_2`, `param.afe.behav.time_gain_module_1` |
| `rpm_base`, `torque_base`, `torque_overload_factor`, `rpm_sim`, `omega_m_sim`, `external_load_inertia` | `param.load.*` |
| `u_psm_scale_ekf` | `param.psm.u_psm_scale_ekf` |
| `use_torque_curve`, `use_speed_control`, `use_psm_encoder`, `time_start_motor_control` | `param.psm.mode.*` |
| `use_ekf_bemf_module_1`, `use_observer_from_simulink_module_1`, `use_observer_from_ccaller_module_1` | `param.psm.impl.*` |
| `tc`, `Nc`, `omega_m_bez` (bare, see above) | `param.glb_time.tc`, `param.glb_time.Nc`, `param.psm.machine.omega_m_bez` |
| literal `1e6` (DC link `Rudc_measure`) | `param.inv.hw.Rudc_measure` |

## Checks done

- **Migration.** The parameter part of the new init was run in Octave without the library on the path and
  compared with the legacy init run with the library functions of `dev` (the battery call adapted with the
  same capacity): of the 326 re-pointed expressions, 325 gave the same value, the only difference was
  `MaxNonlinIter` (5 → 20).
- **Control fixes.** The new init runs in Octave and every one of the 106 `param.*` names of the fixed model
  resolves (`L_norm` 1.0033, `R_norm` 0.0424, `u_scale` 2.9252, `np_gain` 0.3254, `ts_pu` 0.01212). Every
  line of the changed XML parts has valid end points and every From of an active block has its Goto; the
  inputs of the MATLAB Function match its signature and the Inport names. The new `mpc_ttype_gate` was run
  in Octave and gives the same states and outputs as its Python copy used by the check.
- **Design check.** A switched simulation of the whole chain at vector level (Python, not part of the
  repository): shaft speed of `fixed_speed_setting`, torque curve, q current reference, the predictive
  control (line by line copy of `mpc_ttype_gate`), currents and DC voltages sampled, quantized and one step
  late, the EKF rebuilt block by block with the double integrator observer, the encoder-sensorless switch;
  plant: PMSM (Ld, Lq of one system) + 230 µH, T-type bridge with ideal switches and the 3 µs dead time
  (a leg keeps its old level for 3 µs when it moves with the current), split DC link, battery = open-circuit
  voltage + R0 + R1//C1; 1.5 s. It does not model the gate filter and the Simscape solver (the gate timing
  is checked apart on the tc grid), the diodes of the snubbers or the losses.
- **The model was not simulated in MATLAB** (no MATLAB here). Please run it and look at: the current of Q3
  of one leg around an O→P change (no short-circuit peak), `vdc1 − vdc2` (within about ±10 V at rated
  power), the torque against the torque curve, and the `Observer Comparison mod1` scope (the three observers
  should agree).

## Design figures

From the init (`param.psm.design`) and from the design check:

| Quantity | Value |
|---|---|
| bases per system | `ubez` 365.8 V peak, `ibez` 486.0 A peak, `tau_bez` 143.1 kNm, 15.43 Hz, 1 pu power 266.7 kW |
| pu of the machine | Rs 0.040, Ld 0.804, Lq 0.974, Ls 0.889; with the 230 µH inductor the bridge sees 0.833 (d) and 1.003 (q) |
| DC link | battery 1249 V open circuit at soc 0.85 (1.168 × `udc_nom`), about 1275 V while it takes 223 kW |
| operating point at 17.8 rpm | torque curve 125 kNm per system = −0.874 pu (id = 0), 233 kW, 300 A rms, bridge voltage 476 V peak = 1.30 pu of `ubez` = 0.66 of the linear limit `udc/√3` |
| predictive control | `u_scale` 2.925, `L_norm` 1.003, `R_norm` 0.042, `ts_pu` 0.0121; one-step gain 1.00 (q), 1.20 (d) |
| neutral point | 1 pu of neutral-point current for one step moves `vdc1 − vdc2` by 0.0039 pu (4.2 V) |
| gate timing | a gate is below the 5.5 V threshold 0.63 µs after its turn-off command (1.25 µs on the tc grid) and above it 3.29 µs after its turn-on command |

## Open points

1. **Clock.** The control clock `time_inv1` has gain 1.001 (8008 Hz) while the constants use 125 µs: the
   prediction step is 0.1 % short and the speed estimate is 0.999. Harmless here (no carrier), but it is
   the last error of the speed estimate.
2. **Alignment PI.** `phase_compensation_1/alignment_ctrl` feeds this clock but has `kp = 0` and `ki = 0`
   typed in its mask. The AFE clock `time_afe1` is not used by this model.
3. **Sensorless range.** The EKF and the prediction use `Lq + LFi` of the data set. With the machine
   inductances 15 % lower the angle error is +6° and the torque −2.6 %; at −20 % the sensorless control
   loses the angle after about 1 s (with the encoder it still works). +20 % is fine (−10.6°, torque on the
   curve). If the real inductances are not known within about ±15 %, an inductance or angle adaptation is
   needed.
4. **Remaining angle error.** The control angle is one step old with the encoder (−0.7° at 17.8 rpm) and
   about two steps old with the EKF (−1.2°), and the current reference is the target of the next step: the
   current lags the rotor frame by 1.4° (encoder) to 1.9° (EKF), id ≈ −0.02 to −0.03 pu. The torque is on
   the curve within 0.4 %; an angle advance of `(n + 1)·omega·ts` in the current reference would remove it.
5. **DC voltage.** The battery is at 1249–1275 V, 1.17–1.19 times `udc_nom` of the 690 V data set: please
   check the voltage rating of the switches. The control uses the measured `vdc1` and `vdc2`, so it follows
   the battery.
6. **Battery capacity.** The old `lithium_ion_battery_setup` had the capacity inside; the `dev` version
   takes it as an argument. The init uses `P/V` = 208 Ah (1C) as an assumption. It does not matter in
   2.25 s; please give the real value.
7. **Library blocks.** The bridge and the DC link are the old blocks of `semiconductors_bridges` and
   `dclinks`. The move to the new `three_phase_bridge_three_level_ttype_*` blocks is left to the planned
   migration of all the models; the 20 nonlinear iterations of the T-type rule are already set.
8. **Post-processing.** `plotting_results_inv.m` and `power_losses_calculus.m` come from other models: they
   read signals that this model does not log (grid currents, MOSFET/IGBT losses, `load sim_results_*.mat`)
   and flat names (`Nc`, `tc`, `decimation_tc`, `Tambient`, `igbt.inv.data`). Not changed.
9. **Start of the speed estimate.** The double integrator observer that now gives the speed of the EKF block
   starts at π (initial condition of its angle Unit Delay, as before) while the EKF angle starts at 45°: with
   the machine at standstill the speed output swings down to about −1.5 pu (at 5 ms), and it follows the
   true speed from about 0.2 s. The flux of the EKF is about zero at standstill, so the back-EMF of the
   prediction is zero and the current is not disturbed (check), but the scopes show it.
10. **Not checked.** The speed-control variant (`use_speed_control`) and the two non-EKF observers (fixed
    by construction, fix 11, not simulated).
