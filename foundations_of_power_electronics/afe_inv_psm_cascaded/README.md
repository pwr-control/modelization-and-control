# afe_inv_psm_cascaded — two AFE + inverter modules in parallel on one PMSM wind generator

Two identical back-to-back modules of 250 kW are connected in parallel on both sides ("hard parallel"):
the two AFEs at the secondary of one 1 MVA Dyn11 transformer, the two inverters at the terminals of one
permanent magnet wind generator (two of the six 267 kW systems of the 1.6 MW, 17.8 rpm, 104-pole WindGen
generator, driven at fixed speed). The folder name says "cascaded", but the two modules are in parallel.
The model file is `afe_inv_psm_n.slx`.

Since commit dfca8c0 ("building cascade architecture") the modules are made of four blocks of
pwr-control/library: two **`grid_following_dqctrl_mod1`** (AFE, folder `masked_models/grid_following`)
and two **`pmsm_inverter_dqctrl_mod1`** (inverter, folder `masked_models/pmsm_inverter`). Each block is a
Simscape power stage (`afe_ssc.afe_two_level_lcl`: two-level bridge with ideal switches, DC-link
capacitor, LCL filter) plus the whole control in one MATLAB Function, and it takes one struct as its mask
parameter `p` (`p.hw`, `p.pwm`, `p.ctrl`). The old modules `afe_abc_inv_psm_mod1/2` are still in the
model, commented out.

| File | Content |
|---|---|
| `init_model.m` | the initialization, written as a **live script** (plain-text live script: a `.m` file that MATLAB opens in the Live Editor). It fills `param` (the four block structs and the rest of the model), checks the structs against the library examples, prints the design figures of the loops, opens the model and checks it. |
| `afe_inv_psm_cascaded_private.m` | handles to the functions of `private/` (`prv.timing_param`, `prv.loop_design_check`, ...): the Live Editor runs a live script from a copy outside this folder and does not see `private/` |
| `private/` | the functions of this model (table below) |
| `afe_inv_psm_n.slx` | the model (not changed by this revision) |
| `non_virtual_blocks_counting.m` | counts the non-virtual blocks of the model |
| `init_model_legacy.m` | the old flat init of the old architecture (it does not run with the `dev` library), kept for reference |

## The model

- **Grid** (`grid`): library emulator `three_phase_voltage_source_with_frt_emulator` (ideal sources, 690 V,
  50 Hz, with its fault generator) and a Dyn11 transformer of three single-phase blocks (1 MVA, turns
  24/14, so 697 V at the secondary; leakage 76.6 µH and 15.9 mΩ referred to the secondary, magnetizing
  inductance 145 mH, iron losses 1.8 kW), star point of the secondary grounded.
- **AFE** (`afe_dqctrl_mod1`, `afe_dqctrl_mod2`): `grid_following_dqctrl_mod1`. LCL filter 500 µH,
  200 µF + 50 mΩ in a floating star, grid-side inductor of the block only a 1 µH connection (the
  transformer leakage is the grid-side inductor), DC-link capacitor 3.6 mF. Both blocks are on the same
  node (transformer secondary). Control: local clock, carrier and TRGO, first harmonic tracker and PLL,
  DC-link voltage PI → active current reference, dq vector PI on the converter current, zero-sequence
  current PI, SVPWM with a common-mode input. The block has no enable input, no pre-charge, no FRT control
  and no brake chopper.
- **Inverter** (`inv_dqctrl_psm_mod1`, `inv_dqctrl_psm_mod2`): `pmsm_inverter_dqctrl_mod1`. The same power
  stage with an L filter of 230 µH (the capacitor branch is switched off by `Rcf` = 1e5 Ω) and a 3.6 mF
  DC-link capacitor (with the AFE block: 7.2 mF per module, as the old modules). Field-oriented control:
  encoder / back-EMF observer / EKF, torque curve, MTPA, dq current PI, SVPWM with dead-time compensation.
  Inputs `t_master`, `ref` = [enable; reference], `enc` (electrical angle).
- **Machine** (`pmsm`): library block `permanent_magnet_synchronous_machine` with the data of one WindGen
  system (Rs 30.1 mΩ, Ld 6.24 mH, Lq 7.56 mH, magnet flux 3.774 Vs, 104 poles), scaled by
  `param.psm.number_of_modules` = 2 (two systems in parallel). `fixed_speed_setting`: ideal speed source,
  17.8 rpm (step at 25 ms, first-order lag 0.1 s). `motor_load_setting` is commented out.
- **Clocks** (`global timing`): `time_afe3` is the master clock (Goto `time_master`). The four local clocks
  `time_afe1/2`, `time_inv1/2` still read the corrections `local_time_corr_*` of the old modules. The new
  blocks make their own local clock, `t_loc = t_master·p.ctrl.k_time·(1 + corr)`, and align it to
  `t_master` with their own PI.
- **Logging** (`afe_measurement_sim`, `inverter_measurement_sim`, `scopes`): still the signals of the old
  modules (see fix 5).

## How to run

1. `prv.library_setup()` (first section of the init) puts pwr-control/library on the path: the folder
   named by the environment variable `PWR_CONTROL_LIBRARY`, or `library` next to this repository; it clones
   the library there when it is missing.
2. Open `init_model.m` in MATLAB (R2025a or later; plain-text live scripts are the default format for new
   live scripts in R2026b). It opens in the Live Editor, with the output on the right. Run the whole script
   (Run). A section run on its own needs `prv` and `param` in the workspace. To have an `.mlx`, use Save As;
   the `.m` stays readable in git diffs.
3. The first time, make the five fixes of the next section in the model, save it and run the init again:
   the last section must print `connections of the four library blocks as expected`.
4. Run the simulation (`param.sim.simlength` = 1.25 s, `ode23tb`, Simscape local solver step
   `param.glb_time.tc` = 0.625 µs = ts/200).

## Model check: five fixes to make in the slx

This revision does not change the model. The last section of the init checks it
(`prv.check_model_param_refs`: every `param.*` read by an active block exists; `prv.check_model_wiring`:
the connections of the four library blocks and the From blocks without Goto) and prints what is open. With
the model of commit dfca8c0 it prints these points:

1. **Module 2 reads the struct of module 1.** The mask parameter `p` of
   `afe_dqctrl_mod2/grid_following_dqctrl_mod2` is `param.afe_follower(1)` and the one of
   `inv_dqctrl_psm_mod2/pmsm_inverter_dqctrl_mod2` is `param.inv_mod(1)`: set them to
   `param.afe_follower(2)` and `param.inv_mod(2)`. With index 1 both AFEs have `u_cm` = 0, so nothing
   controls the zero-sequence current that can circulate through the two modules, and the two modules have
   the same clock.
2. **`t_master` must be the master time.** The four blocks read the local times `time_afe_module_1/2` and
   `time_inv_module_1/2`. Each block makes its own local clock from `t_master` and aligns it to `t_master`,
   so with a local time at the input the clock ratio is applied twice and the alignment pulls the block to
   the wrong time. Set the GotoTag of `afe_dqctrl_mod1/From`, `afe_dqctrl_mod2/From`,
   `inv_dqctrl_psm_mod1/From8` and `inv_dqctrl_psm_mod2/From8` to `time_master`. In `global timing`, the
   local clocks `time_afe1/2` and `time_inv1/2` with their Unit Delays, their Gotos, the 8 From
   `local_time_corr_*` (no Goto any more) and the scope `pwm_alignments` are then not needed: delete them
   (the clock correction of each block is `ctrl_log(4)`). The init sets the gains of the old local clocks
   (`param.afe.behav.time_gain_module_1/2`, `param.inv.behav.time_gain_module_1/2`) to 1.
3. **Inverter input `ref`.** Both inverter blocks get a Constant `0`. The port takes [enable; reference]
   (2×1), and enable 0 keeps the gates off. Set the Constant value to `param.inv.ref` (= [1; 1]: enable,
   scale of the torque curve `torque_overload_factor`).
4. **Inverter input `enc`.** It comes from a Bus Creator (`theta_elec_psm`, `omega_elec_psm`). The block
   reads one angle, and a bus does not go into its MATLAB Function. Connect `From13` (`theta_elec_psm`)
   directly to `enc` and delete the Bus Creator and `From4`.
5. **Logging and scopes.** 38 From blocks with 25 tags have no active Goto (their Gotos are in the commented
   old modules): 8 in `global timing` (fix 2), 5 in `afe_measurement_sim`, 7 in
   `inverter_measurement_sim`, 18 in `scopes`. A From without its Goto stops the model update. The outputs
   `m` and `ctrl_log` of the four blocks are not connected. Take the signals from these outputs (for
   example one Goto per output and Selectors), or comment the old logging out for now:

   | Old tag | New signal |
   |---|---|
   | `is_abc_pu_mod1/2` (AFE converter current) | `m(1:3)` of the AFE block (pu of `ibez`) |
   | `ig_abc_pu_mod1` (grid current) | `m(7:9)` of the AFE block |
   | `ug_abc_pu_mod1` (grid voltage) | `m(10:12)` of the AFE block (pu of `ubez`); `m(4:6)` is the filter capacitor voltage |
   | `udc_mod1/2`, `vdc_pu_afe_mod1` | `m(13)` of the AFE or inverter block (pu of `udc_nom` = 1070 V) |
   | `ic_dclink_mod1/2` | `m(14)` (DC-link capacitor current of that block, pu of `ibez`) |
   | `iz_afe_pu_mod1/2` (zero-sequence current) | `ctrl_log(36)` of the AFE block |
   | `is_uvw_pu_mod1/2` (machine current) | `m(7:9)` of the inverter block (pu of the machine `ibez`) |
   | `us_uvw_pu_mod1/2` (machine voltage) | `m(10:12)` of the inverter block |
   | angle, speed, torque of the inverter control | `ctrl_log` of the inverter block, indices in `pmsm_inverter_log_index.m` of the library |

   The leg voltages of the old bridges (`inverter_voltage_ee_phase_*`, `all_voltages_about_dclink`) are not
   outputs of the new blocks.

With the five fixes applied to a copy of the model data, the init prints `all 58 param.* references of
the active blocks resolve` and `connections of the four library blocks as expected` (Octave, see
"Checks done").

## Parameters

Everything is in the struct `param`. The four blocks read their own element of a struct array:

| Group | Content |
|---|---|
| `param.sim` | `simlength` 1.25 s, `tc` 0.625 µs (FixedStep of the model), `nonlinear_iterations` 3, `check_model`, `design_figures`, `design_plots`, `load_step_time` |
| `param.time`, `param.glb_time` | 4 kHz with double update (control at 8 kHz), 3 µs dead time, `tc_factor` 200; `glb_time = prv.timing_param(param.time)` (`tc`, `ts_afe`, `ts_inv`, `decimation_tc`, `Nc`, ... read by the logging and the Simscape local solver) |
| `param.grid` | 690 V, 50 Hz, transformer data; `fault_in` (fault test, OFF), `frt = prv.grid_fault_param(fault_in)`, `emu = prv.grid_emulator_param(grid, frt)` (emulator and transformer blocks) |
| `param.afe_follower(1:2)` | mask struct `p` of the two AFE blocks, fields of `grid_following_param_example.m` of the library; the modules differ in `ctrl.k_time` (1.0002 / 1.0015) and `ctrl.u_cm` (0 / −1) |
| `param.afe.hw`, `param.afe.behav` | read outside the blocks: bases of the scopes, gains of the old local clocks (1) |
| `param.psm.machine_in`, `param.psm.machine`, `param.psm.number_of_modules` | WindGen data, per-system values and bases (`prv.psm_machine_param`), two systems in the machine block |
| `param.load` | torque curve `rpm_base` / `torque_base` (per system), `torque_overload_factor` 1, `rpm_sim` 17.8, `external_load_inertia` |
| `param.inv_mod(1:2)` | mask struct `p` of the two inverter blocks, fields of `pmsm_inverter_param_example.m`; the modules differ in `ctrl.k_time` (1.0005 / 1.001) |
| `param.inv.ref`, `param.inv.behav` | input `ref` of the inverter blocks [enable; scale of the torque curve]; gains of the old local clocks (1) |
| `param.ctrl.filters.inv.SOF_50Hz` | 50 Hz filter of `scopes/psm` |
| `param.design` | design figures (`prv.loop_design_check`) |

Main choices (the reasons are also in the text of the live script):

- AFE current PI 0.75 / 35 (library example), DC-link PI `kp_udc` 2 / 35 (old modules; the example 0.85
  gives 44° of phase margin with 7.2 mF), zero-sequence PI 0.5 / 18 only in the AFE of module 2 with
  `u_cm` = −1 (as `u_cm_comp_mod2` of the old model; +1 would be positive feedback with the duty
  convention of the block).
- Inverter current PI **`kp_i` 1.0, `ki_i` 150**, not the example values (16.3 / 3195): see the design
  figures and the two-inverter simulation. Angle from the encoder (`angle_src` 0, as the old model), torque
  curve (`mode` 2) with a slew of 2 pu/s, MTPA, gates on at `t_enable` = 0.25 s.
- Clock alignment of the four blocks `kp_align` 0.25, **`ki_align` 400** (old model and library examples:
  18), so that the carriers of the two AFEs and of the two inverters are aligned before the inverters start
  (design figures, last row).
- Fault test OFF: the new AFE block has no FRT control. The legacy test (25.4, 75 % dip on two phases,
  type D, from 0.75 s) is kept in `param.grid.fault_in`; set `enable_frt_2` = 1 to run it.

### Functions of `private/`

| Function | Role |
|---|---|
| `library_setup(url)` | pwr-control/library on the path, cloned when missing (copy of the one of `theory_analysis_dab/three_phase_dab`) |
| `timing_param(t)` | the clock domains and the names the model reads (formulas of `timing_setup`) |
| `grid_emulator_param(g, fault)` | emulator data, voltage references, bases, the Dyn11 transformer blocks, the 50 Hz filter |
| `grid_fault_param(f)` | the fault test |
| `psm_machine_param(m)` | machine data, per-system values and bases (formulas of `pmsm_setup`) |
| `fht_gains(ts, w, delta, poles)`, `observer_gains(A, z)` | gains of the first harmonic tracker and of the double integrator observer of the AFE (the local functions of `grid_following_param_example.m`) |
| `check_block_struct(p, p_ref, name)` | compares a block struct with the library example: missing fields (they stop the compilation), extra fields (not read), sizes |
| `loop_design_check(param, show_plots)` | design figures of the loops (next section), exact discrete time with the timing of the blocks; for the machine current also the two-axis loop with the dq cross-coupling |
| `check_model_param_refs(model, param)` | the `param.*` names read by the active blocks (commented blocks skipped, struct-array indices checked) and the missing ones |
| `check_model_wiring(model)` | the connections of the four library blocks and the From blocks without an active Goto (fixes above) |

## Design figures

From `prv.loop_design_check` (printed by the init, section "Design figures") and the same check in Python
(script and figures in the project files, folder `afe_inv_psm_cascaded/new_architecture/`). Small signal,
exact discrete time: the sample of the previous trigger is used, the output is held for one period.

| Quantity | Value |
|---|---|
| AFE bases | `ubez` 563.4 V, `ibez` 328.7 A (250 kW/0.9), `udc_nom` 1070 V; 1 pu of the modulator = udc/√3 = 617.8 V |
| machine bases (one system) | `ubez` 365.8 V, `ibez` 486.0 A, `omega_b` 96.93 rad/s (15.43 Hz), `Lbez` 7.765 mH; pu: rs 0.040, ld 0.804, lq 0.974, output filter 0.0298 (231 µH) |
| inverter current loop: the two modes | the machine current (sum of the two modules) sees machine + filter, about 1 pu; the current between the two inverters (difference) sees only the two output inductors, 0.0298 pu per module. The difference mode sets the limit `kp_i` < 2.45. With `kp_i` near 1 the dq cross-coupling of the machine (ω·L ≈ 1 pu at rated speed) is as large as the gain, and the feed-forward of the block uses the references, so it does not act on a current error: the machine current is a two-axis loop with a slow oscillating pole (the one-axis margins say little) |
| inverter current loop, `kp_i` 1.0 / `ki_i` 150 (init) | difference mode: 528 Hz, phase margin 52°, gain margin 7.6 dB. Machine current with the cross-coupling at 1 pu speed: slowest pole −33.5 ± 77.2j rad/s (time constant 30 ms); a step of the q current settles (2 %) in 89 ms, overshoot 27 %. With the torque slew of 2 pu/s the current error stays below 0.01 pu |
| inverter current loop, first choice 0.8 / 20 | difference mode: 417 Hz, 62°, 9.7 dB. Machine current: slowest pole −9.2 ± 12.6j rad/s (109 ms), step settling 199 ms |
| inverter current loop, example gains 16.27 / 3194.8 | machine current fast (5 ms); difference mode **unstable** (pole radius 2.6) |
| inverter current loop, old model 0.25 / 35 | difference mode 133 Hz, 72°; machine current: slowest pole −5.6 ± 28.4j rad/s (178 ms), step settling 419 ms |
| LCL resonance | 1037 Hz with the two modules on the transformer (each sees twice the leakage), 1373 Hz with one module alone |
| AFE current loop, 0.75 / 35 | two modules: 333 Hz, phase margin 68°, gain margin 7.1 dB, LCL mode 1122 Hz damped 6.9 %; one module alone: 386 Hz, 64°, 6.7 dB, 1418 Hz damped 4.7 %; difference of the two AFEs: 451 Hz, 59°, 9.0 dB. `kp_i` limit 1.71 (two modules), 1.63 (one), 2.14 (difference) |
| DC-link voltage loop (7.2 mF) | `kp_udc` 2 / 35: 11.2 Hz, phase margin 75° (example 0.85: 6.5 Hz, 44°) |
| zero-sequence current loop | path through both AFEs, both DC links and both inverters, 1460 µH; PI 0.5 / 18 in the AFE of module 2: 10–36 Hz, phase margin 60–79° (it changes with the zero-vector time 0.1–0.4) |
| operating point at 17.8 rpm | torque curve 125 kNm per system = 0.874 pu, 233 kW per module; AFE current 273 A peak (0.83 pu), modulation 0.92; inverter (MTPA: id −0.12, iq −0.86 pu) about 445 V peak of 618 V available |
| clock alignment | the block corrects a ratio, `t_loc = t_master·k_time·(1 + corr)`, so the gain of its loop grows with the time (`kp_align`·t). With `ki_align` 18 (old model) the carriers of the two AFEs pass through opposition at 0.11 s and are aligned within 2 µs only from 0.88 s; the two inverter carriers are still 68 µs (99°) apart when the inverters start at 0.25 s, aligned from 0.80 s. With 400 (init): aligned from 0.15 s (AFE) and 0.13 s (inverters). The linearized loop is stable only for t < min(1/kp, 2/(2·kp + ki·ts)): 3.6 s of simulated time with 400, 4.0 s with 18 (open point 5) |

## Two-inverter simulation (Octave twin)

To check the inverter gains with the real control code, the two inverter blocks were simulated in Octave:
the MATLAB Function of the library (`pmsm_dq_ctrl_pwm.m`, two copies with their own state, called every
0.625 µs as in Simulink) with the structs `param.inv_mod(1:2)` of the init; ideal floating DC sources of
1070 V (so no zero sequence), ideal switches with 3 µs dead time, 231 µH per inverter, machine of two
systems with the per-unit model of the control, fixed 1 pu speed, encoder angle; gates on at 0.25 s,
torque curve 0.874 pu with a slew of 20 pu/s (to see it settle within 0.4 s; the init has 2 pu/s). In the
second round the output inductor of module 2 is 5 % larger, which excites the current between the
inverters. Script, output and figures in the project files, folder
`afe_inv_psm_cascaded/new_architecture/twin/`.

| Case | Result |
|---|---|
| example gains 16.27 / 3194.8 | the current between the inverters is unstable: above 0.1 pu 1.4 ms after the start, then 207 A rms and 525 A peak, limited only by the voltage limiter; the torque is 24 % too high |
| first choice 0.8 / 20 | current between the inverters 3–4 A peak; the machine current is slow: 50 to 100 ms after the end of the ramp the torque is still 3–4 % above the reference and moves slowly (time constant about 110 ms) |
| init 1.0 / 150 | current between the inverters 3–4 A peak; torque within 1 % of the reference from 70 ms after the end of the ramp (0.2 % on average over 0.35–0.40 s), 12 % overshoot at the end of the fast 20 pu/s ramp |
| init 1.0 / 150 with the torque slew of the init (2 pu/s), run to 0.8 s | the torque is on its reference within 0.001 pu during the whole ramp and after it (0.874 pu from 0.69 s); current error below 0.012 pu; current between the inverters 3 A peak |
| clock alignment `ki_align` 18 (old; first round, 0.8 / 20) | the inverter carriers are 99° apart at the start: 28 A rms and 78 A peak of switching-frequency current between the inverters, phase current peak +7 % (with `ki_align` 400 or the same clock in both: none) |

## Open points

1. **Start-up of the AFEs.** The AFE block has no enable input and no pre-charge: both AFEs switch from
   t = 0, and the DC-link reference steps at 1 ms (`udc_t_step`, filtered with 20 ms). The Solver
   Configuration has "Start simulation from steady state" (`DoDC`) on, which sets the initial capacitor
   voltages and inductor currents. Please look at the first 20 ms (DC-link voltages, AFE currents). The
   inverters start at 0.25 s.
2. **No brake chopper** in the new architecture (the old modules had 4 Ω, on at 1257 V). In normal
   operation the AFE holds the DC link; in a grid fault the energy of the generator goes into the DC link.
3. **No FRT control** in the AFE block (no negative-sequence current control, no reactive current on a
   dip): the fault test is off.
4. **Machine current loop.** `kp_i` cannot go much above 1, because the current between the two inverters
   sees only the two output inductors; then the dq cross-coupling of the machine slows the machine current
   (30 ms with 1.0 / 150). A faster machine loop needs a change of the inverter block: the cross-coupling
   feed-forward from the measured current instead of the references, or a control of the sum and of the
   difference of the two module currents (the two blocks exchange their currents), or more inductance in
   the difference path.
5. **Clock alignment of the library blocks.** The gain of the alignment loop grows with the simulated time
   (design figures, last row): it is slow at the start and unstable after about 3.6 s with `ki_align` 400
   (4 s with 18); then the correction runs into its limit (±5 %) and the carriers drift. The model runs
   1.25 s. For longer runs the blocks need a small change in the library (for example the error divided by
   `t_master`, which gives a constant loop gain, or an additive correction `t_loc = k_time·t_master +
   offset`); the same loop is in `grid_following_dqctrl`, `grid_forming_dqctrl` and `pmsm_inverter_dqctrl`.
6. **Zero-sequence current.** Checked only in the design figures (one PI, in the AFE of module 2,
   `u_cm` = −1, 10–36 Hz). It acts only with fix 1 (module 2 reads `param.afe_follower(2)`).
7. **Rating and speed.** At 17.8 rpm the torque curve asks 0.874 pu (233 kW per module, 250 kW
   converters). `fixed_speed_setting` reaches 90 % of the speed at 0.25 s and 99 % at 0.5 s; with the slew of
   2 pu/s the torque follows the curve from about 0.7 s.
8. **Not checked here:** the AFE side in the time domain (PLL, start-up, DC-link control with the two
   modules), the speed-control variant (`motor_load_setting`), the back-EMF observer and the EKF in the
   parallel connection (`angle_src` 1 or 2: each inverter estimates the angle from its own current).

## Changes in this revision

- `init_model.m` rewritten for the four library blocks, as a live script. The structs of the blocks have
  the fields of the library examples (`grid_following_param_example.m`, `pmsm_inverter_param_example.m`);
  the init checks it at every run.
- New: `afe_inv_psm_cascaded_private.m`, `private/library_setup.m`, `fht_gains.m`, `observer_gains.m`,
  `check_block_struct.m`, `check_model_wiring.m`, `loop_design_check.m`. `check_model_param_refs.m` now
  skips the commented-out blocks and checks the struct-array indices.
- Removed (old architecture only, in the git history): `private/afe_ctrl_param.m`, `afe_hw_param.m`,
  `inv_hw_param.m`, `psm_ctrl_param.m`, `device_ideal_switch_param.m`, `device_igbt_param.m`.
- Kept: `timing_param.m`, `grid_emulator_param.m`, `grid_fault_param.m`, `psm_machine_param.m`.
- The model is not changed.

### Checks done

No MATLAB here. The init was run end to end in Octave 8.4 with small stand-ins for `find_system`,
`get_param` and `open_system` that read a dump of the model file:

- the structs of the four blocks have the fields of the library examples, the timing is consistent;
- the design figures are the same as in the Python check;
- the 55 `param.*` names read by the active blocks of the model as it is all resolve, and the wiring check
  prints the five fixes above (12 lines);
- with the five fixes applied to the dump: 58 names resolve, `connections of the four library blocks as
  expected`;
- the two inverter blocks in the Octave twin above (library control code, not Simscape); the AFE side was
  not simulated.

The live script format was checked line by line (text lines, headings, section breaks, appendix); please
tell me if the Live Editor shows something wrong.
