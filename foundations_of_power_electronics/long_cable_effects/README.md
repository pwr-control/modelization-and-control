# long_cable_effects — AFE + inverter + PMSM with HF cable models

A 400 V grid (emulator and a Dyn11 10 MVA transformer, star point of the secondary grounded) feeds a 250 kW
two-level AFE. The AFE and a two-level inverter share a 7.2 mF DC link (3.6 mF in each block). The inverter
drives one 250 kW system of a permanent magnet synchronous machine (MotorDrive of the library: four systems,
1 MW, 24.6 rpm). A power cable of two lumped high-frequency sections per phase (30 µH, 30 mΩ and 18 nF to
ground for the whole cable) is connected at the inverter terminals, with its far end open. The model shows the
voltage at the far end of the cable and the common-mode current through the cable capacitances. Both
converters switch at 4 kHz with double update; the Simscape step is 0.125 µs.

In November 2026 Davide rebuilt the model on the library blocks `grid_following_dqctrl_mod1` (AFE) and
`pmsm_inverter_dqctrl_mod1` (inverter). The model then did not run: the blocks read `param.afe_follower(1)`
and `param.inv_mod(1)`, which no init made, the rest of the model still read the flat names of the legacy
init, and 20 From blocks had lost their Goto in the rebuild. The init was rewritten as one struct `param` with
the functions of `private/`, the model was re-pointed to `param.*` and the 20 tags were fixed
(see [Model changes](#model-changes-november-2026)).

| File | Content |
|---|---|
| `init_model.m` | initialization (plain-text live script): every number in the struct `param`, the check of the two block structs against the library examples, the design check, then it opens the model, closes the scopes, sets the mechanical side, creates the bus of the power meter and checks the `param.*` references and the From/Goto tags. Run it **from this folder**, then run the model. |
| `long_cable_effects_private.m` | handles to the functions of `private/` (`prv.<name>`): the Live Editor runs a live script from a copy outside this folder and does not see `private/` |
| `private/` | the functions of this model (table below) |
| `afe_inv_psm_with_hf_models.slx` | the model |
| `plotting_*.m`, `sim_results_post_process.m`, `voltage_*spectrum_analysis.m` | post-processing of the old model (see [Open points](#open-points)) |
| `non_virtual_blocks_counting.m` | counts the non-virtual blocks of the model |
| `data/` | 25 measured oscilloscope traces (625 kHz) for comparison |

## How to use

| Step | Action |
|---|---|
| 1 | Clone `pwr-control/library` next to this repository (or set the environment variable `PWR_CONTROL_LIBRARY`). The init puts it on the path (`prv.library_setup`), and clones it when it is missing. |
| 2 | Run `init_model.m` from this folder (Run, the whole script). It prints the data, the design check and the model checks, and opens the model. |
| 3 | Run the simulation (1.75 s at 0.125 µs: about 14 million steps). |

## The model

**Power stage** (Simscape; solver ode23tb with MaxStep `tc`; one Solver Configuration in `grid/grid_emulator`
with the local solver Backward Euler at `tc` and `param.sim.nonlinear_iterations` = 3).
- `grid`: MATLAB Function copy of the library grid emulator (`three_phase_voltage_source_with_frt_emulator`,
  the fault generators are off) and `3ph_Dyg`: three single-phase transformer blocks, delta primary with 24
  turns, grounded star secondary with 14 turns, 10 MVA, `ucc` 5 %. Its leakage (2.6 µH at the secondary) is
  the grid side of the LCL filter of the AFE. The IEC power meter needs the bus `PowerMeterIecBus`.
- `afe_abc_inv_psm_mod1/afe_dqctrl_mod1`: library block `grid_following_dqctrl_mod1`, mask parameter
  `param.afe_follower(1)`: two-level bridge, DC-link capacitor 3.6 mF, converter-side inductor 330 µH
  (zero sequence 10 µH) with 5 µH of leakage, 360 µF in floating star with 50 mΩ, grid-side 1 µH, and the
  dq control (PLL, DC-link voltage PI, vector current PI, SVPWM). Selectors on its output `m` give the
  Gotos of the logging and of the scopes.
- `afe_abc_inv_psm_mod1/inv_dqctrl_psm_mod1`: library block `pmsm_inverter_dqctrl_mod1`, mask parameter
  `param.inv_mod(1)`: the same power stage with an L filter of 80 µH (zero sequence 5 µH) plus 5 µH and
  1 µH, the filter capacitor switched off by `Rcf` = 100 kΩ, the other 3.6 mF of the DC link, and the
  field-oriented control (torque curve, MTPA, angle from the EKF). Its `ref` input is [enable; scale of the
  torque curve], with the enable Step at `param.inv.t_ref_enable` = 0.125 s.
- `afe_abc_inv_psm_mod1`: also the voltages of Udc+ and Udc− to ground (`udc_pgnd`, `udc_ngnd`).
- `power_cables_and_psm_high_frequency_models`: per phase `power_cable_model_k` with the two sections
  `parasitic_lump_1/2` (15 µH and 15 mΩ in series, 9 nF to ground with 0.5 mΩ), the measurement at the
  inverter side (`driver_side`), the measurement at the far end (`machine_side`), then Open Circuit blocks.
  The machine is connected at the inverter terminals, in parallel with the cable: at high frequency the
  machine winding is a high impedance, so the open end is the machine terminal for the ringing.
- `pmsm/synchronous_motor`: one system of MotorDrive (Rs 15.1 mΩ, Ld 2.0 mH, Lq 3.2 mH, ψm 2.433 Vs,
  80 poles). The shaft turns at `param.load.rpm_sim` = 24.6 rpm (`fixed_speed_setting`; the init comments
  out `motor_load_setting`, which is for the speed control `mode` 1).

**Timing.** `global timing` gives the master time `time_master` (a Clock). Each library block reads it on
its first input and makes its own carrier and trigger from its local clock: `t_loc = t_master·k_time·(1 + corr)`,
with `k_time` 1.0002 (AFE) and 1.001 (inverter), as the old time gains, and an alignment PI. `param.glb_time`
is only for the Simscape step, the logging and the power meter.

**Logging and scopes.** `afe_measurement_sim` and `inverter_measurement_sim` (To Workspace at `tc` or at
`ts_afe`, the last `t_measure` = 0.3 s), `scopes/cable_effects` (cable voltages and currents, rms over one
grid period at `tc`), `scopes/grid_afe_side`, `scopes/machine_side`. The thermal scopes and To Workspace blocks
were already commented out (ideal switches).

## Parameters

`init_model.m` fills the struct `param`; the model reads only `param.*` names (60 different references in the
active blocks).

| Group | Content |
|---|---|
| `param.sim` | `simlength` 1.75 s, nonlinear iterations of the Solver Configuration, `check_model`, `design_check`, `load_step_time` (speed control only) |
| `param.time` → `param.glb_time` | 4 kHz, double update, dead time 3 µs for the AFE and the inverter, `t_measure` 0.3 s, `tc_factor` 1000: `prv.timing_param` gives `ts_afe` = `ts_inv` = 125 µs, `tc` 0.125 µs, `Nc`, `Ns_afe`, ... |
| `param.grid` → `param.grid.frt`, `param.grid.emu` | 400 V, 50 Hz, 10 MVA Dyn11 transformer, core data; fault test 25.4 of the legacy init (off); `prv.grid_fault_param`, `prv.grid_emulator_param` |
| `param.cable_in` → `param.cable` | the whole cable (30 µH, 30 mΩ, 18 nF, 1 mΩ) and the number of sections: `prv.cable_param` gives the data of one section (`Ls1`, `Rs1`, `Cp1`, `RCs1`, `Cp2`, `RCs2`, `Rp1`) and `f1`, `Z0`, `Q1` |
| `param.afe_follower` | mask struct of the AFE block: `hw` (power stage), `pwm`, `ctrl`; the field names of `grid_following_param_example.m` of the library |
| `param.afe.hw`, `param.afe.behav` | copy of `param.afe_follower.hw` for the bases read outside the block (`ibez`, `ubez`, `udc_nom`); time gain of the commented `time_afe1` |
| `param.psm.machine_in` → `param.psm.machine` | MotorDrive: `prv.psm_machine_param` gives the data of one system and its bases |
| `param.load` | torque curve of the legacy script, now in pu (see design check, point 5), `torque_overload_factor` 1, `rpm_sim` 24.6 rpm, load inertia of the speed control |
| `param.inv_mod` | mask struct of the inverter block: `hw`, `pwm`, `ctrl`; the field names of `pmsm_inverter_param_example.m` |
| `param.inv.hw`, `param.inv.t_ref_enable`, `param.inv.behav` | bases read outside the block (`ibez` for `is_uvw_cm`), enable of the `ref` input, time gain of the commented `time_inv1` |
| `param.ctrl.filters`, `param.ctrl.rms_tc` | 50 Hz second-order filter of two scopes, rms over one grid period at `tc` |
| `param.design` | result of the design check `prv.design_check` |

| `private/` function | What it does |
|---|---|
| `timing_param` | sample times of the AFE and inverter domains and the logging base `tc` |
| `grid_fault_param`, `grid_emulator_param` | fault test and data of the grid emulator and of the transformer blocks |
| `cable_param` | data of one cable section from the data of the whole cable |
| `psm_machine_param` | machine data of one system, bases, normalized parameters |
| `fht_gains`, `observer_gains` | gains of the first harmonic tracker and of the PLL observer of the AFE block |
| `check_block_struct` | compares the two block structs with the examples of the library (a missing field stops the compilation with a message from inside the block) |
| `design_check` | design check (below); returns the numbers in `param.design` |
| `power_meter_bus` | the bus object `PowerMeterIecBus` of the power meter (base workspace) |
| `library_setup` | puts the library on the path, clones it when missing |
| `check_model_param_refs`, `check_from_goto` | checks after `open_system`: every `param.*` read by the active blocks exists; every active From reaches an active Goto |

Old workspace names and their place in `param`:

| Legacy name | Now |
|---|---|
| `simlength`, `tc`, `glb_time.tc` (configuration set) | `param.sim.simlength`, `param.glb_time.tc` |
| `glb_time.*` | `param.glb_time.*` |
| `hf_power_cable_model.*` | `param.cable.*` |
| `psm.*` | `param.psm.machine.*` |
| `hwdata.afe.*` | `param.afe.hw.*` |
| `filters.*`, `rms_tc.*` | `param.ctrl.filters.*`, `param.ctrl.rms_tc.*` |
| `rpm_sim`, `external_load_inertia` | `param.load.rpm_sim`, `param.load.external_load_inertia` |
| `load_step_time` | `param.sim.load_step_time` |
| `time_gain_afe_module_1`, `time_gain_inv_module_2` | `param.afe.behav.time_gain_module_1`, `param.inv.behav.time_gain_module_1` (blocks commented out); the clock ratios are now `k_time` of the two block structs |
| typed in the blocks: enable Step of the inverter 0.125 s, Constant of the curve scale (1) | `param.inv.t_ref_enable`, `param.load.torque_overload_factor` |

## Model changes (November 2026)

Edited in the XML of `afe_inv_psm_with_hf_models.slx` (no MATLAB; script `repoint_lce.py` of the project):
205 block parameters and the three entries of the configuration set re-pointed to `param.*`, two typed values
moved to `param`, and these changes for the 20 From tags without Goto:
- **Master time**: the Froms at the time input of the two blocks read `time_master` (they read
  `time_afe_module_1` and `time_inv_module_1` of the old timing chain, which needed the alignment
  corrections `local_time_corr_afe1/inv1`, now inside the blocks). The old chain in `global timing`
  (`time_afe1`, `time_inv1`, two Unit Delays, two Froms, two Gotos) is commented out.
- **Measurements from the output `m` (17 × 1) of the AFE block** (`afe_dqctrl_mod1`): Selectors and Gotos
  `is_abc_pu_mod1` (m 1:3, converter current), `vs_abc_pu_mod1` (m 4:6, filter capacitor voltage),
  `ig_abc_pu_mod1` (m 7:9, current towards the grid), `ug_abc_pu_mod1` (m 10:12, terminal voltage),
  `vdc_pu_afe_mod1` (m 13), `ic_dclink_mod1` (m 14 × `ibez`, A), `iz_afe_pu_mod1` (mean of m 1:3),
  `udc_mod1` (m 13 × `udc_nom`, V).
- **Measurements from the output `m` of the inverter block** (`inv_dqctrl_psm_mod1`): `is_uvw_pu` (m 7:9),
  `us_uvw_pu` (m 10:12), `is_uvw_cm` (sum of the three output currents, A).
- **DC link to ground** (`afe_abc_inv_psm_mod1`): two voltage sensors from Udc+ and Udc− to ground, Gotos
  `udc_pgnd` and `udc_ngnd`, and `all_voltages_about_dclink` = [udc; udc_pgnd; (udc_pgnd + udc_ngnd)/2;
  udc_ngnd] for the scope `udc voltages`.
- **Commented out** (the new blocks have no such signal): `ileg_dc_inv_mod1` (DC current of the inverter
  legs) and `inverter_voltage_ee_phase_u/v/w`, with their To Workspace blocks.

Checked with Octave stubs of Simulink (`find_system`, `get_param` on the saved XML): the whole init runs, all
60 `param.*` references of the active blocks resolve, every active From block reaches its Goto, and the two
block structs have the fields of the library examples. The model has not been run in MATLAB yet.

## Design check (November 2026)

Numbers from `prv.design_check` (printed by the init, kept in `param.design`). Linear and small-signal:
limits, dead time and the PWM sampling are left out, except in point 7.

1. **Cable ringing and the solver.** Seen from the bridge, the cable rings in two modes per path: normal mode
   (86 µH of the inverter filter in series) 116 kHz and 624 kHz, common mode (29.6 µH: both filters in zero
   sequence and the transformer) 170 kHz and 640 kHz. The cable data give a damping ratio of about 0.0001
   (Q of a section 2634). The Backward Euler solver at `tc` adds 0.046 at 116 kHz and 0.23 at 624 kHz: **265 to
   1842 times the damping of the cable**. So the decay of the ringing in the simulation is set by `tc`, not by
   the cable: to 5 % in 91 µs with the solver, in 29 ms with the cable losses alone. For the switching of one
   leg (step of 660 V) the far end reaches 1364 V line to line without the numerical damping (2.07 times the
   step) and 1252 V with it (1.90 times); common-mode voltage 471 / 412 V; current to ground 12.9 / 11.3 A peak.
2. **AFE current loop.** LCL resonance 4461 Hz, above fs/6 = 1333 Hz and above the Nyquist frequency of the
   control (4 kHz): the converter-current loop does not damp it, `Rcf` does (Q 2.0). Crossover 238 Hz, phase
   margin 73°, gain margin 14.5 dB; closed-loop LCL mode 4310 Hz with 25.7 % damping.
3. **Inverter current loop** (kp 13.7, ki 2691 1/s: crossover 250 Hz on the q axis by design). q axis: 255 Hz,
   phase margin 66°, gain margin 13.9 dB; d axis (smaller inductance): 401 Hz, 59°, 10.0 dB, unstable from
   kp 44. With the dq cross-coupling at 1 pu speed a step of the q current settles to 2 % in 9.6 ms with 10 %
   overshoot.
4. **DC-link voltage loop** (7.2 mF, kp 2, ki 35): crossover 28.6 Hz, phase margin 81°. There is no
   zero-sequence control (`u_cm` = 0): the common-mode current through the cable capacitances is not controlled.
5. **Operating point.** The torque curve of the legacy script belongs to the 1.6 MW, 17.8 rpm generator of the
   690 V models. The old model read it in rpm and in Nm per system: at its 17.8 rpm it asked 187.5 kNm per
   system, 1.93 times the torque base of MotorDrive (about 350 kW in a 250 kW system). The init now gives the
   curve in pu (speed in pu of 17.8 rpm, torque in pu of 858.4 kNm), as in the 690 V models, and the shaft turns
   at 24.6 rpm: 84.8 kNm (0.87 pu of one system), 218 kW. AFE 441 A peak (0.78 pu), modulation 0.875; inverter
   id −0.21 pu, iq −0.82 pu, 281 V peak of the 381 V available (udc/√3).
6. **Start-up.** The grid voltage rises with 1/(3.2 ms·s + 1)² from t = 0, and the diodes of the AFE charge the
   DC link to 572 V with up to 476 A (0.84 pu): no pre-charge circuit. The DC-link reference (step at 1 ms,
   low-pass 20 ms, from 0) stays below the diode voltage until 41 ms: before that the PI asks for current into
   the grid to pull udc down. The inverter enables its `ref` input at 0.125 s and its gates at 0.25 s; with the
   torque ramp of 2 pu/s it reaches the operating point at 0.69 s.
7. **Clock alignment** (kp 0.25, ki 18 1/s, the legacy gains): the AFE carrier (`k_time` 1.0002) is within
   5° of the master from 0.65 s, the inverter carrier (`k_time` 1.001, up to 180° of error) from 0.81 s. Until
   then the carriers of the AFE and of the inverter slide against each other, so the common-mode voltage at
   the cable changes pattern over time. The loop becomes unstable after about 4 s of simulated time (its gain
   grows with the time): fine for 1.75 s, not for longer runs.

## Open points

- **Damping of the ringing** (point 1): the cable data have only the DC resistance (no skin and proximity
  effect), and the solver damps far more than the cable. To compare with the measured traces in `data/`:
  either add high-frequency losses to the sections (for example a resistor in parallel with `Ls1`, sized from
  the decay of a measured ring) so that they are larger than the numerical damping, or reduce `tc` (the
  numerical damping is about ω·tc/2: 0.01 at 116 kHz needs `tc_factor` ≈ 4500, 4.5 times the run time).
- **Meaning of `vs_abc_pu_mod1`**: it is now the filter capacitor voltage of the AFE block (m 4:6); in the old
  model it was the voltage of the bridge terminals to ground.
- **Post-processing scripts**: except `plotting_power_cables.m`, they read names that the model does not log
  any more (`vg_abc_sim`, `is_abc_sim`, `idc_sim`, `udc_pos_sim`, `id_pu_sim`, ...) and the flat names `Nc`,
  `tc`, `fPWM_AFE` (now `param.glb_time.*`). Not changed.
- **Start-up** (point 6): the DC-link reference from 0 makes the AFE push power into the grid for the first
  41 ms; a reference that starts at the diode voltage (or a pre-charge) avoids it. Not changed.
- **Clock alignment** (point 7): unstable after about 4 s; the fix belongs to the library blocks.
- **Not in the new architecture**: brake chopper, pre-charge, fault ride-through control (the fault test of
  the grid emulator is off).
- **Leftovers**: the commented timing chain in `global timing` and the commented `parasitic_lump_1/2` from the
  DC bus to ground can be deleted when no longer needed. The old init (`init_model_legacy.m`) was deleted in
  November 2026: it is in the git history.
