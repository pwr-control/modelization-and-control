# mpc_ttype_psm — wind generator: three-level T-type inverter with predictive current control

One of the six three-phase systems of a 1.6 MW, 17.8 rpm, 104-pole permanent magnet wind generator
(about 267 kW per system) feeds a three-level T-type inverter with ideal switches through a 230 µH
inductor. The split DC link (2 × 14.4 mF, neutral point at the center tap) sits across a 1200 V
lithium-ion battery. The shaft turns at a fixed speed; the control takes the torque from the torque curve
of the turbine and runs a finite-set model predictive control (FCS-MPC) that tests the 27 states of the
bridge every 125 µs and writes the gate commands directly (no modulator). Angle, speed and magnet flux come
from an extended Kalman filter (EKF), sensorless.

| File | Content |
|---|---|
| `init_model.m` | initialization: every number in the struct `param`, design figures, then it opens the model, sets the mechanical setting and checks the `param.*` references. Run it **from this folder** (the functions of `private/` are visible only to the files of this folder). |
| `private/` | the parameter functions of this model (table below) |
| `mpc_ttype_psm.slx` | the model (power stage, machine, control, logging) |
| `spectrum.m` | FFT of a phase current of the machine (`iuvw_psm_sim`) |
| `plotting_results_inv.m`, `power_losses_calculus.m` | post-processing of other models, not for this one (open point 14) |
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
  `deadtime_generator_for_ttype` filters the four gates (time constant `tc`) and puts a CMOS interlock with
  3 µs dead time between Q1 and Q4 (open point 6). The `timer` (4 kHz, double update) only gives the trigger
  of the control.
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
- predictive control: `inner_ctrl/mp control/mpc_pu_d`, MATLAB Function `mpc_ttype_gate`. For each state
  [−1 0 1]³ it computes the alpha/beta voltage `(vdc1 + vdc2)/3·(...)`, predicts the current one step ahead
  with the machine model in pu (`ts·omega_bez`, `Rs_norm`, `Lalpha_norm = Lbeta_norm = Ls_norm`, back-EMF
  from the EKF speed and flux) and takes the state with the lowest
  `J = k_kalman·|i_ref − i_pred|² + k_sw·|u − u_prev|² + k_dc·(vdc1 − vdc2 + inp·ts/1e-3)²`
  (`inp`: neutral-point current of the state). Weights 1, 0, 0. Its voltage output (× 3) goes to the EKF;
- angle and speed: `bemf_observer1` has three variants (EKF by default, Simulink and C-Caller back-EMF
  observers); the predictive control always takes speed and flux from the EKF. `encoder-sensorless-switch`
  picks the angle and speed used by the current reference and the torque curve (encoder through a double
  integrator observer, or EKF);
- `phase_compensation_1`: local time alignment loop (gains 0, open point 11);
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
| `param.battery` | `nominal_battery_voltage_1` 1200 V (also read by the battery mask as its default), `nominal_battery_power` 250 kW, `initial_battery_soc` 0.85, `charge_capacity_1` (assumption, open point 12), `pack_1 = battery_param(...)` with R0, R1 halved and C1 / 50 as in the legacy init |
| `param.psm.machine` | `psm_machine_param(param.psm.machine_in)`: WindGen data, per-system values (`*_m`), bases (`ubez` 365.8 V, `ibez` 486.0 A, `tau_bez` 143.1 kNm, `omega_bez` 96.93 rad/s), pu values (`Rs_norm`, `Ld_norm`, `Lq_norm`, `Ls_norm`, `Lalpha_norm`, `Lbeta_norm`, ...) |
| `param.psm` | `u_psm_scale` (non-EKF observers), `u_psm_scale_ekf` (EKF), `mode.*` (`use_torque_curve`, `use_speed_control`, `use_psm_encoder`, `time_start_motor_control`), `impl.*` (which back-EMF observer is active), `design` (informative figures) |
| `param.psm.ctrl` | `psm_ctrl_param(ts, omega_bez, Jm_norm, u_psm_scale, machine, gains)`: weights of the predictive control `mpc.k_kalman`/`mpc.k_dc`/`mpc.k_sw`, speed PI `kp_w`/`ki_w`/`iq_lim`, observer gains `emf_fb_p`/`emf_p`, `omega_th`, `omega_base`, `di_obs.*` (double integrator observer, closed-form pole placement), `ekf.*` (`A1_tilde_ekf` ... `A6_tilde_ekf`, `Bd_ekf`, `C_ekf`, `Qkalman`, `Rkalman`) |
| `param.load` | torque curve `rpm_base`/`torque_base` (per system), `torque_overload_factor` 1, shaft speed `rpm_sim` 17.8, `omega_m_sim` and `external_load_inertia` (speed control) |
| `param.ctrl` | alignment PI `kp_align`/`ki_align`/`lim_up_align`/`lim_down_align`, filters `filters.inv.*` (1 Hz first-order of the torque curve, 50 Hz second-order of the scope data) |
| `param.meas` | ADC quantization (`adc_quantization` 1/2^11 read by the quantizers) and full scales |

### Functions of `private/`

| Function | Role |
|---|---|
| `timing_param(t)` | the clock domain of the inverter and its control (and the sample time of the battery estimator) |
| `inv_hw_param(V, P, fpwm)` | hardware data and bases of the inverter, output inductor, split DC link |
| `psm_machine_param(m)` | machine data, per-system values and bases (formulas of `pmsm_setup` of the library, same as `inv_psm`) |
| `psm_ctrl_param(...)` | weights of the predictive control, speed PI, observer gains, double integrator observer, EKF matrices (formulas of `ctrl_pmsm_setup` and `ekf_pmsm_setup`) |
| `battery_param(...)` | lithium-ion pack (formulas of `lithium_ion_battery_setup`) |
| `check_model_param_refs(model, param)` | lists the `param.*` names read by the model and reports the missing ones |

## Old names and new names

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
| `tc`, `Nc`, `omega_m_bez` (bare, see below) | `param.glb_time.tc`, `param.glb_time.Nc`, `param.psm.machine.omega_m_bez` |
| literal `1e6` (DC link `Rudc_measure`) | `param.inv.hw.Rudc_measure` |

## What changed in the model (1 November 2026)

- 326 block, mask, variant and configuration parameters re-pointed from the flat names to `param.*`
  (table above). The model reads 104 distinct `param.*` names (plus `param.battery.nominal_battery_voltage_1`
  in the default of the battery mask), all defined by the init. Same `ModelUUID` (the model itself is
  re-pointed, there is no copy). Configuration set: `StopTime = param.sim.simlength`,
  `FixedStep = param.glb_time.tc` (not used by `ode23tb`), `MaxStep = param.glb_time.tc`.
- `battery/Solver Configuration`: `MaxNonlinIter` 5 → `param.sim.nonlinear_iterations` = 20. In a T-type leg
  two diodes can switch in the same step; the tests of the library bridges need at least 10 (with 5 they
  stop with "Temperature must be greater than absolute zero"). This is the only change of value.
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

**Checks done.** The parameter part of the new init was run in Octave without the library on the path and
compared with the legacy init run with the library functions of `dev` (the battery call adapted with the
same capacity): of the 326 re-pointed expressions, 325 give the same value and the only difference is
`MaxNonlinIter` (5 → 20). Every `param.*` name of the model resolves, and every From of an active block has
its Goto. The slx keeps the order, compression and dates of its zip entries; all changed XML parts parse.
The model was not simulated here (no MATLAB).

## Design figures

From the init (`param.psm.design`) and from the design check (a switched simulation of the whole chain at
vector level, with the predictive control copied from `mpc_ttype_gate` and the EKF rebuilt block by block;
not part of the repository):

| Quantity | Value |
|---|---|
| bases per system | `ubez` 365.8 V peak, `ibez` 486.0 A peak, `tau_bez` 143.1 kNm, 15.43 Hz, 1 pu power 266.7 kW |
| pu of the machine | Rs 0.040, Ld 0.804, Lq 0.974, Ls 0.889; with the 230 µH inductor the bridge sees 0.833 (d) and 1.003 (q) |
| DC link | battery 1249 V open circuit at soc 0.85 (1.168 × `udc_nom`), about 1273 V while it takes 200–220 kW |
| operating point at 17.8 rpm | torque curve 125 kNm per system = −0.874 pu (id = 0), 233 kW, 300 A rms, bridge voltage 476 V peak = 1.30 pu of `ubez` = 0.66 of the linear limit `udc/√3` |
| predictive control | prediction step `ts·omega_bez` = 0.0121; the true voltage of a state is `udc_nom/ubez` = 2.925 times the predicted one (open point 1) |
| neutral point | 1 pu of neutral-point current for one step moves `vdc1 − vdc2` by 0.0039 pu; the cost function assumes 12.1 pu (open point 3) |
| gate timing | after a command: Q1/Q4 on and off 0.433 µs, Q2/Q3 on 0.285 µs, off 0.627 µs (open point 6) |
| model as it is (check) | angle error −21.6°, EKF speed 0.982, torque −0.773 pu (−11.6 %), current error 0.024 pu rms, a leg at O 8.5 % of the time |
| with the fixes of points 1–5 (check) | angle error −6.6°, EKF speed 0.994, torque −0.855 pu (−2.2 %), current error 0.008 pu rms, a leg at O 47 % of the time |

## Open points found during the migration

The control was not changed; these are the points to look at. Numbers from the init and from the switched
check (start-up and 1.5 s at 17.8 rpm, values at 1.2–1.5 s; ideal switches, no dead time, no diode
clamping of the neutral point).

1. **Voltage base of the predictive control.** `mpc_ttype_gate` computes the voltage of each state from
   `vdc1 + vdc2` in pu of `udc_nom` (1070 V) and uses it in a machine model in pu of `ubez` (365.8 V): the
   true voltage is 2.925 times the predicted one. With the delay of point 4 the loop seen as a deadbeat
   loop has the poles `z² − z + g = 0` with g = 2.6 (q) and 3.1 (d), unstable; the finite set of vectors
   keeps the current bounded, but the control picks too large vectors: current error 0.024 pu rms, a leg
   uses the O level only 8.5 % of the time (the bridge works almost as a two-level bridge), and about 5200
   direct P↔N changes per second. Fix: multiply the state voltage by `udc_nom/ubez` in the prediction
   (for example a new input `param.psm.ctrl.mpc.u_scale`).
2. **EKF voltage.** The outputs `ualpha`, `ubeta` of `mp control` are multiplied by 3 (Gain2, Gain3) and the
   EKF scales them with `u_psm_scale_ekf = udc_nom/(√3·ubez)`: the EKF gets √3 times the true voltage.
   Result: angle error −21.6°, true id −0.29 pu, torque −0.773 pu for −0.874 pu asked by the torque curve
   (with point 5). The EKF needs `u·udc_nom/ubez` (u = output of `mpc_ttype_gate`):
   `u_psm_scale_ekf = udc_nom/(3·ubez)` = 0.975 instead of 1.689, or gains 1 and `udc_nom/ubez`.
3. **Neutral point.** `k_dc = 0`: the cost function does not see the neutral point. Of two states with the
   same voltage the first in the table wins (strict `<`), and that is always the N-type small vector, so the
   neutral point drifts: with the encoder angle `vdc1 − vdc2` reaches −588 V in 1.5 s; with points 1, 2, 4
   fixed and `k_dc = 0` it is lost after 0.47 s. The term itself uses a fictitious capacitance `1e-3` in the
   pu time of the prediction: 3070 times too strong. With `k_dc = 1` and this value the control stops using
   the O level (0.0 % of the time). With `ibez/(CFi1·udc_nom·omega_bez)` = 0.325 in place of `1/1e-3` and
   `k_dc = 1`, `vdc1 − vdc2` stays within a few volts and the O level is used 45 % of the time (with points
   1, 2, 4). Fix it together with point 1.
4. **Measurement delay.** The prediction starts from the current measured one step before, but the chosen
   vector acts one step later. Predicting i(k) with the vector of the last step and then choosing the
   vector for i(k+1) halves the current error (0.019 → 0.009 pu rms) and cuts the P↔N changes from 2300 to
   700 per second.
5. **EKF speed estimate 1.8 % low** (the same EKF as `inv_psm`). The copy of `A_ekf` of the state prediction
   has no `omega_bez` on the A6 path: `F·x` then has the rotation and back-EMF terms 1 % too large. The
   torque curve is steep at 17.8 rpm (+25 % from 17 to 17.8 rpm), so the torque reference is 8 % low.
   Adding `omega_bez` on the A6 path is not a fix: `F·x` then counts the products ω·ψ twice and the speed
   estimate halves. The nonlinear prediction `x + ts·omega_bez·f(x, u)` gives 0.994.
6. **Gate generator of the T-type legs (to check in Simulink).** The interlock with dead time is between Q1
   and Q4, which are never complementary in a T-type leg; the complementary pairs Q1/Q3 and Q2/Q4 (as in
   the three-level modulators of the library) get only the gate filter. From the values in the blocks, at
   an O→P change Q1 is on after 0.433 µs while Q3 is off only after 0.627 µs: P and the neutral point are
   shorted through Q1 and the Q2/Q3 branch (O→N: Q4 and Q2). The Simscape step is 0.625 µs and the commands
   change on the tc grid, so the first step after the change sees the gate of Q3 at 5.52 V against the
   5.5 V threshold: an 18 mV margin, the result also depends on the solver accuracy. If the model sees it,
   each O→P and O→N change is a one-step short of a 14.4 mF capacitor, about 120 kA and 46 J: about 84 kW
   at the 1800 changes per second of the model as it is, about 220 kW with points 1–4 fixed (estimate).
   To check: the current of Q3 of one leg around an O→P change, or the battery power (`current_battery_sim`,
   `voltage_battery_sim`) against the machine power. Fix: interlock Q1/Q3 and Q2/Q4.
7. **EKF angle with the voltage fixed.** About −5° to −6.6° remain. In `inv_psm` the main part was the
   saliency (the EKF uses Ls = 0.889 pu, the bridge sees 0.833 and 1.003 pu); not tested here. The EKF
   also uses the vector chosen in this step with the current measured one step before (the vector that made
   that current is the one of two steps before); fixing only this moves the angle from −5.1° to −6.4°
   (other errors cancel in part).
8. **`udcd_pu`.** In `inverter_control`, From2 and From3 both read `udc1d_pu`: `udcd_pu` is
   `2·udc1d_pu` instead of `udc1d_pu + udc2d_pu`. Only the two non-EKF observers read it.
9. **Switching weight.** `k_sw = 0`: nothing limits the switching but the 8 kHz sampling (1.3–1.8 kHz per
   leg in the check) and nothing avoids the direct P↔N changes (full DC voltage step on the phase).
10. **DC voltage.** The battery is at 1249–1273 V, 1.17–1.19 times `udc_nom` of the 690 V data set: please
    check the voltage rating of the switches. The control uses the measured `vdc1 + vdc2`, so it follows
    the battery.
11. **Clocks.** The control clock `time_inv1` has gain 1.001 (8008 Hz) while the constants use 125 µs
    (0.1 %, harmless here: there is no carrier). The alignment PI (`phase_compensation_1/alignment_ctrl`)
    feeds this clock but has `kp = 0` and `ki = 0` typed in its mask. The AFE clock `time_afe1` is not
    used by this model.
12. **Battery capacity.** The old `lithium_ion_battery_setup` had the capacity inside; the `dev` version
    takes it as an argument. The init uses `P/V` = 208 Ah (1C) as an assumption. It does not matter in
    2.25 s; please give the real value.
13. **Library blocks.** The bridge and the DC link are the old blocks of `semiconductors_bridges` and
    `dclinks`. The move to the new `three_phase_bridge_three_level_ttype_*` blocks is left to the planned
    migration of all the models; the 20 nonlinear iterations of the T-type rule are already set.
14. **Post-processing.** `plotting_results_inv.m` and `power_losses_calculus.m` come from other models: they
    read signals that this model does not log (grid currents, MOSFET/IGBT losses, `load sim_results_*.mat`)
    and flat names (`Nc`, `tc`, `decimation_tc`, `Tambient`, `igbt.inv.data`). Not changed.
15. **Not checked.** The speed-control variant (`use_speed_control`), and the voltage scale of the two
    non-EKF back-EMF observers (they get `ualpha_pu` = 3 × the output of the control times
    `u_psm_scale = 2/3·udc_nom/ubez`, and `udcd_pu` of point 8).
