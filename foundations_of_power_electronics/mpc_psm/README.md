# mpc_psm — wind generator: two-level inverter with predictive current control

One of the six three-phase systems of a 1.6 MW, 17.8 rpm, 104-pole permanent magnet wind generator
(about 267 kW per system) feeds a two-level inverter with ideal switches through a 230 µH inductor. The
DC link (2 × 14.4 mF in series, center tap used only as a measuring point) sits across a 1200 V
lithium-ion battery. The shaft turns at a fixed speed; the control takes the torque from the torque curve
of the turbine and runs a finite-set model predictive control (FCS-MPC) that tests the 8 states of the
bridge every 66.7 µs (15 kHz) and writes the six gate commands directly (no modulator). Angle and magnet
flux come from an extended Kalman filter (EKF), sensorless; the speed comes from a double integrator
observer on the EKF angle. The control was fixed in November 2026: see
[Control fixes](#control-fixes-november-2026).

**Design note:** [`repos_doc/psm_mpc_ekf`](https://github.com/pwr-control/repositories-documentation/tree/main/repos_doc/psm_mpc_ekf)
of `pwr-control/repositories-documentation` (PDF `psm_mpc_ekf.pdf`) explains the machine model in per unit
and the active flux, the predictive control and its current loop, the EKF and the speed of the double
integrator observer, the parameters and the design check of this model.

| File | Content |
|---|---|
| `init_model.m` | initialization (plain-text live script): every number in the struct `param`, design figures, then it opens the model, sets the mechanical setting and checks the `param.*` references and the From/Goto tags. Run it **from this folder**. |
| `mpc_psm_private.m` | handles to the functions of `private/` (`prv.<name>`): the Live Editor runs a live script from a copy outside this folder and does not see `private/` |
| `private/` | the parameter functions of this model (table below) |
| `mpc_psm.slx` | the model (power stage, machine, control, logging) |
| `spectrum.m` | FFT of a phase current of the machine (`iuvw_psm_sim`) and of a phase voltage (`us_uvw_sim_mod1`) |
| `plotting_results_inv.m`, `power_losses_calculus.m` | post-processing of other models, not for this one (open point 8) |
| `init_model_legacy.m` | the old flat init, kept until the merge; it no longer runs with the library (see below) |

## The model

**Power stage.**
- `battery/lithium_ion_battery`: library lithium-ion battery (332 cells, 1249 V open circuit at state of
  charge 0.85, state-of-charge estimator at `ts_dab` = 66.7 µs) across the whole DC link. The Solver
  Configuration starts the simulation from steady state, so the DC link starts at the battery voltage.
- `inv_psm_mod1/dclink/dclink`: library block `dclink_with_center_tap`, 2 × 14.4 mF with a 107 kΩ
  measuring resistor across each capacitor (value typed in the block); it gives the DC voltage to the
  control in pu of `udc_nom` = 1070 V.
- `inv_psm_mod1/inverter/inverter`: library block `Three-Phase Bridge (ideal)`
  (`ideal_bridges.three_phase_bridge`): six ideal switches with antiparallel diodes, gate threshold 2.5 V,
  Ron 1 mΩ, gate order [a+; a−; b+; b−; c+; c−]. Its built-in DC capacitor and AC inductors are **off**
  (fix 10). Before it, `dead_time_generator_cmos_simulink` filters the six gates (time constant `tc`) and
  puts the dead time between the two switches of each leg: the switch that turns on waits until an RC copy
  (time constant √2·3 µs) of the other gate is below 7.5 V, so ln 2·√2·3 µs = 2.94 µs. The six gates
  travel as a bus (`gate_cmd_Q1` … `Q6` = a+, a−, b+, b−, c+, c−) that the Simulink-PS Converter takes as a
  vector. The `timer` (15 kHz, single update) only gives the trigger of the control.
- `inv_psm_mod1/inverter_output_filter`: three-phase inductor 230 µH (`RLFi` 1.8 mΩ).
- `psm`: library block `permanent_magnet_synchronous_machine`, per system Rs 30.1 mΩ, Ld 6.24 mH,
  Lq 7.56 mH, magnet flux 3.774 Vs, 104 poles.
- `fixed_speed_setting`: ideal speed source, step to 17.8 rpm at 0.025 s through a 0.1 s lag (energy
  production, default). `motor_load_setting`: inertia and load torque for the speed-control variant.

**Control** (triggered every `ts` = 66.7 µs, quantities in pu of the machine of one system):
- acquisition: phase currents, DC voltage, encoder angle and speed quantized (12 bit) and delayed by one
  step;
- torque reference: `torque_reference` = torque curve (`param.load.rpm_base`, `torque_base`) at the speed
  filtered at 1 Hz, × −1 / `tau_bez`; it is the q current reference, id = 0
  (`current_reference_generator`, at the control angle). Variant: speed PI (`Speed control`,
  `param.psm.mode.use_speed_control`);
- predictive control: `inner_ctrl/mp control/mpc_pu_d`, MATLAB Function `mpc`. The current and the DC
  voltage arrive one step late, so it first computes the current of now with the vector of the last step
  (delay compensation). Then, for each of the 8 states, it computes the alpha/beta voltage from the measured
  DC voltage times `u_scale = udc_nom/ubez` (pu of `ubez`), predicts the current of the next step with
  `L_norm = (Lq + LFi)/Lbez`, `R_norm = (Rs + RLFi)/Xbez`, `ts·omega_bez` and the back-EMF of the EKF, and
  applies the state with the lowest
  `J = k_kalman·|i_ref − i_pred|² + k_sw·(number of legs that change state)`, weights 1 and 1e-5. It gives
  the observers the voltage (pu of `ubez`) of the vector applied two steps before: the vector that made the
  current measured one step before;
- angle and speed: `bemf_observer1` has three variants (EKF by default, Simulink and C-Caller back-EMF
  observers), all fed with that voltage and the same R and L as the predictive control; the predictive
  control always takes speed and flux from the EKF. The speed output of the EKF block comes from the double
  integrator observer on the EKF angle. `encoder-sensorless-switch` picks the angle and speed used by the
  current reference and the torque curve (encoder through a double integrator observer, or EKF);
- `phase_compensation_1`: local time alignment loop (gains 0, open point 2);
- logging: `inverter_measurement_sim` (To Workspace), `scopes`.

## How to run

1. Clone the `library` repository next to this one (or set the environment variable
   `PWR_CONTROL_LIBRARY`): `library_setup` puts it on the MATLAB path, and clones it when it is missing. The
   library gives the Simulink/Simscape blocks and the C-Caller source of the back-EMF observer, no
   parameter.
2. From this folder run `init_model.m` (Run, or from the Command Window). It fills `param`, prints the
   functions of `private/`, the bases and the design figures, opens `mpc_psm.slx`, imports the C-Caller
   bus type `bemf_obsv_output_t`, closes the scopes, sets the mechanical setting (`fixed_speed_setting` or
   `motor_load_setting`, by commenting the blocks in and out as the legacy init did) and checks that every
   `param.*` name read by the model exists and that every active From has its Goto.
3. Run the simulation (`param.sim.simlength` = 1.25 s, `ode23tb`, `MaxStep` and Simscape local solver step
   `param.glb_time.tc` = 0.333 µs, 5 nonlinear iterations).

## Parameter structure

| Group | Content |
|---|---|
| `param.sim` | `simlength` 1.25 s, `nonlinear_iterations` 5 (Solver Configuration), `load_step_time` (speed control only), `check_model` |
| `param.time`, `param.glb_time` | `fpwm` 15 kHz (control timer, no PWM), `trgo` 0 (single update), `dead_time` 3 µs, `t_measure` (the whole simulation), `tc_factor` 200 (tc = 0.333 µs), `fpwm_dab` 7.5 kHz with double update (battery estimator); `glb_time = timing_param(param.time)` gives the names the blocks read: `fPWM_INV`, `TRGO_INV_double_update`, `ts_inv`, `dead_time_INV`, `tc`, `Nc`, `Ns_inv`, `ts_dab` |
| `param.inv.hw` | `inv_hw_param(690, 250e3, fpwm)`: data set `inv690V_250kW` (550 V, 370 A rms, `udc_nom` 1070 V), `LFi` 230 µH, `RLFi`, `LFi_cm`, `RLFi_cm`, DC link `CFi1`/`CFi2` 14.4 mF |
| `param.inv.behav`, `param.afe.behav` | clock gains of the local timing blocks: `time_gain_module_2` = 1.001 (control clock), `time_gain_module_1` (AFE clock, output not used) |
| `param.battery` | `nominal_battery_voltage_1` 1200 V (also read by the battery mask as its default), `nominal_battery_power` 250 kW, `initial_battery_soc` 0.85, `charge_capacity_1` (assumption, open point 6), `pack_1 = battery_param(...)` with R0, R1 halved and C1 / 50 as in the legacy init |
| `param.psm.machine` | `psm_machine_param(param.psm.machine_in)`: WindGen data, per-system values (`*_m`), bases (`ubez` 365.8 V, `ibez` 486.0 A, `tau_bez` 143.1 kNm, `omega_bez` 96.93 rad/s), pu values (`Rs_norm`, `Ld_norm`, `Lq_norm`, `Ls_norm`, ...) |
| `param.psm` | `u_psm_scale` = 1 (non-EKF observers), `u_psm_scale_ekf` = 1 (EKF): the voltage of the predictive control is already in pu of `ubez`; `mode.*` (`use_torque_curve`, `use_speed_control`, `use_psm_encoder`, `time_start_motor_control`), `impl.*` (which back-EMF observer is active), `design` (informative figures) |
| `param.psm.ctrl` | `psm_ctrl_param(ts, omega_bez, Jm_norm, u_psm_scale, machine, inv.hw, gains)`: predictive control `mpc.L_norm`, `mpc.R_norm`, `mpc.u_scale` and weights `mpc.k_kalman`/`mpc.k_sw`, speed PI `kp_w`/`ki_w`/`iq_lim`, observer gains `emf_fb_p`/`emf_p`, `omega_th`, `omega_base`, `di_obs.*` (double integrator observer, closed-form pole placement), `ekf.*` (`A1_tilde_ekf` ... `A6_tilde_ekf`, `A6_pred_ekf`, `Bd_ekf`, `C_ekf`, `Qkalman`, `Rkalman`, with `R_norm` and `L_norm`) |
| `param.load` | torque curve `rpm_base`/`torque_base` (per system), `torque_overload_factor` 1, shaft speed `rpm_sim` 17.8, `omega_m_sim` and `external_load_inertia` (speed control) |
| `param.ctrl` | alignment PI `kp_align`/`ki_align`/`lim_up_align`/`lim_down_align`, filters `filters.inv.*` (1 Hz first-order of the torque curve, 50 Hz second-order of the scope data) |
| `param.meas` | ADC quantization (`adc_quantization` 1/2^11 read by the quantizers) and full scales |

### Functions of `private/`

| Function | Role |
|---|---|
| `library_setup()` | puts `pwr-control/library` on the path, clones it when missing (same file as in `ps_25V_12kA`) |
| `timing_param(t)` | the clock domain of the inverter and its control (and the sample time of the battery estimator) |
| `inv_hw_param(V, P, fpwm)` | hardware data of the inverter, output inductor, DC link (same file as in `mpc_ttype_psm`) |
| `psm_machine_param(m)` | machine data, per-system values and bases (formulas of `pmsm_setup` of the library, same as `inv_psm` and `mpc_ttype_psm`) |
| `psm_ctrl_param(...)` | model data and weights of the predictive control, speed PI, observer gains, double integrator observer, EKF matrices (formulas of `ctrl_pmsm_setup` and `ekf_pmsm_setup`, with the fixes below) |
| `battery_param(...)` | lithium-ion pack (formulas of `lithium_ion_battery_setup`) |
| `check_model_param_refs(model, param)` | lists the `param.*` names read by the model and reports the missing ones |
| `check_from_goto(model)` | lists the tags read by active From blocks that have no active Goto |

## Control fixes (November 2026)

The design check of the migration found the control far from its design. In the `dev` model (with the
new bridge block and its built-in passives on) the EKF speed was 1.30 times the true speed, the torque 16 %
above the torque curve and the angle error +9° on average with a standard deviation of 13°; without the passives the EKF lost the
angle completely. In the check the old control follows its reference at low speed, but an oscillation of
the angle and of id grows with the speed, and at about 0.9 s (near rated speed, iq −0.83 pu) the EKF speed
jumps to 1.3 pu and the torque reference goes to the top of the curve. These are the fixes:

| # | Problem | Fix |
|---|---|---|
| 1 | Voltage base of the prediction: the states were computed from `vdc` = 1 (a Constant) without a scale, 2/3 for [1 0 0] in pu of `udc_nom`, while the machine model of the prediction is in pu of `ubez`. At the battery voltage the true vector is 2/3·1249/365.8 = 2.277 pu: the prediction saw the voltage 3.42 times too small. | State voltage = measured DC voltage (From `udcd_pu_inv_mod1`, pu of `udc_nom`, one step late like the currents) × `u_scale = udc_nom/ubez` = 2.925. |
| 2 | No delay compensation: the prediction started from the current measured one step before, but the chosen vector acts one step later. With fix 1 missing too, the loop seen as a deadbeat loop had the poles `z² − z + g = 0`, g = 3.0 (q), 3.6 (d), 2.7 with the 1 mH of the bridge: unstable, oscillation near 3 kHz. Even g = 1 gives an undamped oscillation at 2.5 kHz without the compensation. | Current of now computed with the vector of the last step, then the choice: the error obeys `e(k+1) = (1 − g)·e(k−1)`, g = 1.00 on q (deadbeat), 1.20 on d (the error is multiplied by −0.20 every two steps). |
| 3 | States: 7 vectors with one zero vector ([1 1 1]); after an active vector only the same vector, its two neighbours and the zero vector were allowed, so a large current error could not be corrected in one step. The switching term weighted the squared voltage of the vector, with `k_sw` = 0. | The 8 states at every step (both zero vectors). `k_sw = 1e-5` times the number of legs that change state: it only breaks the ties (above all between [0 0 0] and [1 1 1]): 3.5 → 2.8 kHz per device at rated power with the same current error (2.8 A rms). |
| 4 | Inductance of the prediction: `Lalpha_norm` = `Lbeta_norm` = Ls = (Ld + Lq)/2 = 0.889 pu and `Rs_norm`, without the 230 µH inductor; the bridge sees 0.833 pu on d and 1.003 pu on q. | `L_norm = (Lq + LFi)/Lbez` = 1.0033 pu and `R_norm = (Rs + RLFi)/Xbez` = 0.0424 pu, for the prediction, the EKF and the two non-EKF observers. |
| 5 | EKF voltage: 3/2 (Gain7, Gain1) × the vector in pu of `udc_nom` × `u_psm_scale_ekf` (√3/2·2/3·`udc_nom`/`ubez` = 1.689) was 0.742 times the true voltage, and it was the vector chosen in this step, not the one that made the measured current. | Output of `mpc` = voltage of the vector of the last step in pu of `ubez`; with the Unit Delay of `mp control` the EKF gets the vector of two steps before. Gain7, Gain1 = 1; `u_psm_scale_ekf` = 1. |
| 6 | EKF inductance: with Ls and Rs (no output inductor) the flux of the EKF is not on the d axis: −5.7° of saliency error at rated torque. | `L_norm` and `R_norm` (fix 4): the flux of the EKF is then the "active flux" ψm + (Ld − Lq)·id, on the d axis for any id. |
| 7 | EKF state prediction: the copy of `A_ekf` of the state prediction had the A6 path without `omega_bez`: ω·ψ 1 % too large in `F·x`, speed estimate about 1 % low. | `A6_pred_ekf = zeros(6)` on that path: `F_pred·x = x + ts·omega_bez·f(x, u)`. The covariance copy keeps `A6_tilde_ekf` (the Jacobian). |
| 8 | Speed estimate: the speed state of the EKF is still 0.4 % low (forward Euler). On the steep torque curve at 17.8 rpm that is −1.2 % of torque. | Speed output of the EKF block from the double integrator observer on the EKF angle (block already in the EKF subsystem): 0.9992 × the true speed, i.e. the 1.001 clock (open point 1). |
| 9 | EKF noise: with `q_ekf` = 1/10 of the legacy init the EKF follows the model error too fast at 15 kHz: the angle jumps (std 14°) when the machine inductance is 10 % lower than the value of the control and is lost at −15 %. | `q_ekf` = 1/100 (only the ratio to `r_ekf` = 1 matters): it works from −25 % (+11.8°) to +20 % (−9.5°), and the angle noise at rated power is 0.35° std instead of 0.79°. |
| 10 | Bridge block: the new `Three-Phase Bridge (ideal)` had its built-in passives on: `cap_dc` (2 mF with the initial equation vdc = `Vdc0` = 0 V, against the 1249 V of the battery and of the DC link at the start) and `ind_ac` (1 mH + 10 mΩ per phase: 13 % more inductance than the control knows, −6.3° of angle error with the fixed control). | `cap_dc` and `ind_ac` off: the model has its own DC link (`dclink_with_center_tap`) and output inductor (230 µH). |
| 11 | The two non-EKF observers got 3/2 × the vector (pu of `udc_nom`) × `u_psm_scale` (2/3·`udc_nom`/`ubez`) = 0.86 times the true voltage, one step too new, and used Ls, Rs. | They get the same voltage as the EKF (their `udc` input is the constant 1 that was already there), `u_psm_scale` = 1, R and L of the predictive control. Not simulated in the check. |

**What changed in the slx** (10 XML parts, one new block with SID 40528; everything else is
byte-identical):
- `inner_ctrl/mp control/mpc_pu_d` (chart 58): new script of `mpc` (comments in English); three inputs
  renamed in place, same port order: `Lbeta_norm` → `u_scale`, `Lalpha_norm` → `L_norm`, `Rs_norm` →
  `R_norm`. In `mp control`: Constant1, Constant2, Constant4 read `param.psm.ctrl.mpc.u_scale`, `L_norm`,
  `R_norm`; Gain7, Gain1: 3/2 → 1; the Constant 1 on the `vdc` input is replaced by the From
  `udcd_pu_inv_mod1` (new block).
- `EKF_bemf_observer/extended_kalman_observer/A_ekf` (copy of the state prediction): Gain1 reads
  `param.psm.ctrl.ekf.A6_pred_ekf`. `extended_kalman_observer`: the output `omega_elec_ekf_hat` comes from
  the double integrator observer (output 2), the speed state goes to `Terminator1`.
- `bemf_observer/psi_alpha_hat`, `psi_beta_hat`, `bemf_observer_ccaller`: `Rs_norm`, `Ls_norm` →
  `param.psm.ctrl.mpc.R_norm`, `L_norm`.
- `inverter/inverter/Three-Phase Bridge (ideal)`: `cap_dc` and `ind_ac` off.

**Results of the check** (switched simulation of the whole chain, see below; values averaged over
1.0–1.25 s, 17.8 rpm unless said otherwise):

| Case | Control angle error (std) | Speed estimate | Torque vs torque curve | Current error rms | Switching per device |
|---|---|---|---|---|---|
| before the fix (`dev` model, bridge passives on) | +9.4° (12.6°) | 1.304 | +16.0 % | 50.4 A | 959 Hz |
| before the fix, bridge passives off | lost: +37° (29°) | 4.89 | — | 200 A | 297 Hz |
| fixed, sensorless | −0.1° (0.35°) | 0.9992 | −1.7 % | 2.8 A | 2823 Hz |
| fixed, encoder | −0.4° (0.00°) | 0.9990 | −1.1 % | 2.8 A | 2863 Hz |
| fixed, no dead time | −0.7° (0.02°) | 0.9990 | −1.6 % | 2.8 A | 2802 Hz |
| fixed, 12 rpm | −0.3° (0.40°) | 0.9989 | −0.3 % | 2.1 A | 2158 Hz |
| fixed, bridge passives still on | −6.3° (0.30°) | 0.9988 | −1.0 % | 2.6 A | 2907 Hz |
| fixed, machine L +20 % | −9.5° (0.29°) | 0.9987 | −1.1 % | 2.5 A | 2919 Hz |
| fixed, machine L −10 % | +4.6° (0.50°) | 0.9991 | −3.0 % | 3.0 A | 2765 Hz |
| fixed, machine L −20 % | +9.4° (1.29°) | 0.9993 | −5.0 % | 3.4 A | 2641 Hz |
| fixed, machine L −25 % | +11.8° (2.07°) | 0.9989 | −6.3 % | 3.7 A | 2553 Hz |
| fixed, machine L −25 %, encoder | −0.4° (0.00°) | 0.9995 | −1.1 % | 3.7 A | 2519 Hz |
| fixed, legacy EKF noise `q_ekf` = 1/10 | −0.2° (0.79°) | 0.9987 | −1.7 % | 2.8 A | 2806 Hz |
| fixed, `q_ekf` = 1/10, machine L −10 % | +7.4° (13.9°) | 1.0005 | −4.7 % | 15.5 A | 2020 Hz |
| fixed, `q_ekf` = 1/10, machine L −15 % | lost: +33° (27°) | 0.998 | — | 142 A | 600 Hz |
| fixed, without the switching term (`k_sw` = 0) | −1.2° (3.00°) | 0.9999 | −1.7 % | 2.8 A | 3503 Hz |
| fixed, `k_sw` = 1e-4 | −0.5° (0.25°) | 0.9991 | −1.5 % | 3.5 A | 1688 Hz |
| fixed, `k_sw` = 1e-3 | −0.7° (0.29°) | 0.9991 | −1.3 % | 11.6 A | 367 Hz |
| fixed, speed from the EKF state (without fix 8) | −0.1° (0.29°) | 0.9960 | −2.9 % | 2.8 A | 2836 Hz |

About −1.3 % of the torque error of every fixed case is the start of the run, not the control: the speed
of the torque curve goes through the 1 Hz filter after the 0.1 s lag of the speed step, and at 1.0–1.25 s
the filter is still 0.05 rpm behind, on a part of the curve with 31 kNm/rpm (the curve point at 17.8 rpm is
−0.874 pu). The rest is the speed estimate (0.08 % low = −0.35 % of torque). "Switching per device" is the
number of state changes of a leg per second divided by 2. The machine inductances change both Ld and Lq;
with them the angle error grows by about 0.47° per % of inductance error (open point 3).

The switching term: `k_sw` = 1e-5 lowers the switching by 20 % for the same current error and also makes
the EKF angle quieter (std 0.35° instead of 3.0°); 1e-4 gives 40 % less switching for 25 % more current
error; 1e-3 is too much (11.6 A).

## Migration (November 2026)

- 301 block and mask parameters re-pointed from the flat names to `param.*` (table below), plus the
  configuration set (`StopTime = param.sim.simlength`, `FixedStep = param.glb_time.tc` (not used by
  `ode23tb`), `MaxStep = param.glb_time.tc`) and the literal 5 of `MaxNonlinIter` of the Solver
  Configuration (`param.sim.nonlinear_iterations`, still 5: the library rule for two-level bridges). Same
  `ModelUUID` (the model itself is re-pointed, there is no copy). After the control fixes the model reads 95
  distinct `param.*` names, all defined by the init.
- 28 blocks are now commented out because they read Goto tags that do not exist anywhere in the model (a
  From without its Goto stops the update of the diagram): `global timing/From9` (`local_time_corr_afe1`,
  clock correction of an AFE that this model does not have), and the IGBT/diode logging of a bridge that
  this model does not have (tags `igbt_*_invQ1/Q2`, `diode_*_invQ1`): `inverter_measurement_sim/From4`,
  `From12` … `From21`, `From25`, `From26`, `To Workspace18` … `To Workspace30`, and the subsystem
  `scopes/scopes_for_igbt_based_inverter`.
- The legacy init had seven clock domains; the model uses only the inverter one (plus `ts_dab` for the
  battery estimator). The one logging block that read `glb_time.Ns_afe` now reads `Ns_inv` (same value).
- `spectrum.m`: `Nc`, `tc` → `param.glb_time.Nc`, `param.glb_time.tc`.

The legacy init `init_model_legacy.m` does not run with the `dev` library: `grid_fault_generator` is now a
function, `lithium_ion_battery_setup` takes the charge capacity as an argument, and it switches three
inverter blocks that the model no longer has. It also has the old control values.

### Old names and new names

| Old (flat workspace) | New |
|---|---|
| `glb_time.*` (`glb_time.Ns_afe` → `Ns_inv`) | `param.glb_time.*` |
| `psm.*` | `param.psm.machine.*` |
| `psm_ctrl.*` | `param.psm.ctrl.*` |
| `psm_ctrl.ekf.k_kalman`, `psm_ctrl.ekf.k_sw` | `param.psm.ctrl.mpc.k_kalman`, `k_sw` |
| `psm.Lalpha_norm`, `psm.Lbeta_norm`, `psm.Rs_norm` (inputs of the predictive control) | `param.psm.ctrl.mpc.L_norm`, `u_scale`, `R_norm` (fixes 1, 4) |
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
| `tc`, `omega_m_bez` (bare, in `FixedStep` and in the commented block `speed_reference_generator/Step1`) | `param.glb_time.tc`, `param.psm.machine.omega_m_bez` |

## Checks done

- **Migration.** The parameter part of the new init was run in Octave without the library on the path and
  compared with the legacy init run with the library functions of `dev` (the battery call adapted with the
  same capacity): the 112 distinct re-pointed expressions give the same values. The only exceptions are the
  two bare names that the legacy init never defined (`tc` of `FixedStep`, `omega_m_bez` of a commented
  block) and the state-space filter of the scopes (not a number, same definition).
- **Control fixes.** The new init runs in Octave against a copy of the block data of the fixed model: every
  one of the 95 `param.*` names resolves (`L_norm` 1.0033, `R_norm` 0.0424, `u_scale` 2.9252) and every
  From of an active block has its Goto. The changed XML parts are well formed, every line has valid end
  points, the inputs of the MATLAB Function match its signature and the Inport names. The new `mpc` was run
  in Octave for 2000 steps and gives the same states and outputs as its Python copy used by the check.
- **Design check.** A switched simulation of the whole chain at vector level (Python, not part of the
  repository; script and figures in the project files): shaft speed of `fixed_speed_setting`, torque
  curve with its 1 Hz filter, q current reference, the predictive control (line by line copy of `mpc`, and
  of the old function for the "before" cases), currents and DC voltage sampled, quantized and one step
  late, the EKF rebuilt block by block with the double integrator observer, the encoder-sensorless switch,
  the 1.001 clock; plant: PMSM (Ld, Lq of one system) + 230 µH, two-level bridge with ideal switches and the
  2.94 µs dead time (a leg keeps its old level when the switch that turns on must take the current), DC
  link with ESR, battery = open-circuit voltage + R0 + R1//C1; 1.25 s. It does not model the gate filter
  and the Simscape solver.
- **Not yet run in MATLAB/Simulink.** What the check predicts for the scopes: the current on its reference
  within about 3 A rms at rated power, the torque on the torque curve (within the settling of the 1 Hz
  speed filter), the EKF angle within about 0.5° of the encoder angle, and the DC link at about 1274 V.

## Design figures

From the init (`param.psm.design`) and from the design check:

| Quantity | Value |
|---|---|
| bases per system | `ubez` 365.8 V peak, `ibez` 486.0 A peak, `tau_bez` 143.1 kNm, 15.43 Hz, 1 pu power 266.7 kW |
| pu of the machine | Rs 0.040, Ld 0.804, Lq 0.974, Ls 0.889; with the 230 µH inductor the bridge sees 0.833 (d) and 1.003 (q) |
| DC link | battery 1249 V open circuit at soc 0.85 (1.168 × `udc_nom`), about 1274 V while it takes 219 kW; linear limit of the bridge 1.97 pu of `ubez` |
| operating point at 17.8 rpm | torque curve 125 kNm per system = −0.874 pu (id = 0), 233 kW, 300 A rms, bridge voltage 476 V peak = 1.30 pu of `ubez` = 0.66 of the linear limit `udc/√3`, cos φ −0.74 |
| predictive control | `u_scale` 2.925, `L_norm` 1.003, `R_norm` 0.042, `ts·omega_bez` 0.0065; one-step gain g = 1.00 (q), 1.20 (d): `e(k+1) = (1 − g)·e(k−1)` |
| angle delay of the chain | 3 steps = 1.11° at rated speed (id ≈ −0.017 pu for the rated iq), not compensated |
| dead time | ln 2·√2·3 µs = 2.94 µs seen by the bridge |
| double integrator observer | poles 15 and 62 Hz (−1 and −4 × `omega_bez`) |

## Open points

1. **Clock.** The control clock `time_inv1` has gain 1.001 (15015 Hz) while the constants use 66.7 µs:
   the prediction step is 0.1 % short and the speed estimate is 0.999. Harmless here (no carrier), but it is
   the last error of the speed estimate.
2. **Alignment PI.** `phase_compensation_1/alignment_ctrl` feeds this clock but has `kp = 0` and `ki = 0`
   typed in its mask. The AFE clock `time_afe1` is not used by this model.
3. **Sensorless range.** The EKF and the prediction use `Lq + LFi` of the data set. The angle error grows by
   about 0.47° per % of error of the machine inductances: +4.6° at −10 %, +11.8° at −25 % (torque −6.3 %),
   −9.5° at +20 %; the angle is not lost in this range with `q_ekf` = 1/100 (with the encoder all cases are
   fine). If the real inductances are not known within about ±10 %, an inductance or angle adaptation is
   needed, or the encoder.
4. **Remaining angle delay.** The control angle is one step old with the encoder and about two steps old
   with the EKF, and the vector acts until the next step: 3 steps = 1.1° at rated speed, id ≈ −0.01 pu.
   The torque is on the curve within the settling of the speed filter; an angle advance of
   `(n + 1)·omega·ts` in the current reference would remove it.
5. **DC voltage.** The battery is at 1249–1274 V, 1.17–1.19 times `udc_nom` of the 690 V data set: please
   check the voltage rating of the switches. The control now uses the measured DC voltage, so it follows the
   battery.
6. **Battery capacity.** The old `lithium_ion_battery_setup` had the capacity inside; the `dev` version
   takes it as an argument. The init uses `P/V` = 208 Ah (1C) as an assumption. It does not matter in
   1.25 s; please give the real value.
7. **Simulation length.** 1.25 s (legacy) ends while the 1 Hz speed filter of the torque curve is still
   settling (−1.3 % of torque at 1.0–1.25 s); 2 s would show the steady state.
8. **Post-processing.** `plotting_results_inv.m` and `power_losses_calculus.m` come from other models: they
   read signals that this model does not log (grid currents, MOSFET/IGBT losses, `load sim_results_*.mat`)
   and flat names (`Nc`, `tc`, `decimation_tc`, `frequency_set`, `f_grid`). Not changed.
9. **Start of the speed estimate.** The double integrator observer that now gives the speed of the EKF
   block starts at π (initial condition of its angle Unit Delay, as before) while the EKF angle starts
   elsewhere: with the machine at standstill the speed output swings down to about −1.5 pu (at 5 ms) and it
   follows the true speed from about 0.15 s. The flux of the EKF is about zero at standstill and the torque
   reference is zero below 4 rpm (until about 0.18 s), so the current is not disturbed (check), but the
   scopes show it.
10. **Not checked.** The speed-control variant (`use_speed_control`) and the two non-EKF observers (fixed
    by construction, fix 11, not simulated).
