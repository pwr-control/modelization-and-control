# vienna_rectifier — 250 kW three-level Vienna rectifier on the 690 V grid

A grid emulator of 1 MVA at 690 V (three controlled voltage sources and a Dyn11 transformer made of three
single-phase units) feeds an LCL filter (500 µH on the converter side, 200 µF with 50 mΩ, the transformer
leakage on the grid side) and the three-level Vienna rectifier of the library with ideal switches and diodes.
The split DC link (2 × 14.4 mF) feeds a 5 Ω resistor: 229 kW at 1070 V. The control is the dq control of the
AFE models (DDSRF-PLL, DC-voltage PI, vector PI of the converter current, space vector modulation with common
mode, three-level PWM at 4 kHz with double update) and runs every 125 µs. The simulation lasts 1.25 s: the DC
link starts precharged at 1070 V and the grid voltage rises from zero in 20 ms.

The model as saved did not run: no init gave the `param.*` names that the grid emulator already read (the
legacy init, the shared template of the AFE and inverter models, stopped at `grid_fault_generator` with
"Not enough input arguments"), the current PI read the gains of the PMSM inverter of the template, the bus
object of the power meter was missing, the Bus Creator of the modulator had unnamed inputs, 23 From blocks
had no Goto, and the DDSRF-PLL was fed with the negative sequence, so it could not lock (switched model of
the design check: 1383 A peaks and 1916 V on the DC link). In November 2026 the init was rewritten as a
`param` struct with the functions of `private/`, the model was re-pointed to `param.*` and the broken parts
were fixed (see [Model changes](#model-changes-november-2026)).

**Design note:** [`repos_doc/vienna_rectifier`](https://github.com/pwr-control/repositories-documentation/tree/main/repos_doc/vienna_rectifier)
of pwr-control/repositories-documentation (November 2026): the model, the fixes, the linear checks of the
loops, the switched simulation of the start-up and of the steady state, and the open points. Its Python
scripts (`vienna_sim.py`, `vienna_design_check.py`) are in the `py` folder of the note.

| File | Content |
|---|---|
| `init_model.m` | initialization (plain-text live script): every number in the struct `param`, the design check, then it opens the model, imports the two C types of the C callers, closes the scopes, creates the bus of the power meter and checks the `param.*` references and the From/Goto tags. Run it **from this folder**, then run the model. |
| `vienna_rectifier_private.m` | handles to the functions of `private/` (`prv.<name>`): the Live Editor runs a live script from a copy outside this folder and does not see `private/` |
| `private/` | the parameter functions of this model (table below) |
| `vienna_rectifier.slx` | the model (grid emulator, LCL filter, Vienna bridge, DC link and load, AFE control, logging) |
| `non_virtual_blocks_counting.m` | counts the non-virtual blocks of the model |
| `init_model_legacy.m` | the old init (the whole template of the AFE and inverter models), kept for reference: it stops at `grid_fault_generator` and is not needed any more |

## The model

**Power stage** (Simscape; solver ode23tb with MaxStep `tc` = 0.625 µs; local solver of the grid emulator at
`tc` with `param.sim.nonlinear_iterations` = 5).
- `grid/grid_emulator`: library block `three_phase_voltage_source_with_frt_emulator` (the amplitude rises
  with a 20 ms ramp and a 50 Hz second order filter; both fault generators are off) and `3ph_Dyg`: three
  `single_phase_transformer` blocks, delta primary with 24 turns, grounded star secondary with 14 turns
  (697 V at no load), data from `param.grid.emu.trafo`. The IEC power meter (`power meter iec mfunc`) needs
  the bus `PowerMeterIecBus`.
- `afe_dq_mod1/LCL_filter`: converter side `LFu` 500 µH (with `LFu_cm` 500 µH of common mode) plus 1 µH,
  filter capacitors `CFu` 200 µF in star (star point floating) with `RCFu` 50 mΩ, measurement of the
  converter current (`stack`) and of the grid current and capacitor voltage (`stack1`). The grid side is the
  transformer leakage only: 76.6 µH at the secondary (the equivalent grid impedance of the emulator object is
  not read by any block).
- `afe_dq_mod1/rectifier`: library `vienna_rectifier_ideal_switch_based_model` (per leg: diode D1 to udc+,
  diode D4 from udc−, two switches Q2 and Q3 in common emitter to the neutral point), the dead-time
  generators of the T-type (3 µs turn-on delay), the split DC link `dclink_for_three_levels` (2 × 14.4 mF,
  measuring dividers of 107 kΩ per half), neutral point to ground through 100 kΩ.
- `load`: 5 Ω across the DC link. `global timing`: time base of the AFE (clock gain 1.0002) and of the master.

A Vienna leg can put its terminal only at the rail that has the sign of its current or at the neutral point:
positive current (into the bridge) goes to the neutral point when Q3 is on, otherwise through D1 to udc+;
negative current goes to the neutral point when Q2 is on, otherwise through D4 to udc−. So the bridge only
takes power from the grid, and a few degrees around each current zero crossing it cannot follow a voltage
reference of the opposite sign.

**Control** (`afe_dq_mod1/AFE_control/afe_control`, triggered by the modulator every 125 µs, quantities in pu
of `ubez` = 563.4 V and `ibez` = 328.7 A):
- `acquisitions`: 12 bit quantization and one step of delay; clock alignment PI (`phase_compensation_1`);
- `grid_voltage_phase_lock_loops/DDSRF-PLL` on the capacitor voltage: first harmonic tracker per phase
  (reset for the first 12.5 ms), DDSRF with low-pass filters τ = 1/ω, loop PI on the positive-sequence eta
  voltage, frequency integrator clamped to 0.5 ... 1.5 pu and started at 1 pu, double integrator observer;
- `udc_ctrl`: reference = Step at 1 ms to `udc_norm_ref` (1 pu) through a 20 ms low-pass; PI kp 4, ki 45 in
  velocity form with back-calculation, output ±1.65 pu = active current reference; Switch on
  `pwr_lim_enable` (0: the PI output passes);
- `current_ctrl`: vector PI kp 0.25, ki 35 on the xi and eta currents (|u| ≤ 1, integrator decay while the
  voltage is clipped), reactive current reference 0; zero-sequence PI whose output is multiplied by
  `u_cm_comp_mod1` = 0;
- `space_vector`: inverse Park and the C caller `sv_pwm_cm` (space vector modulation, centred common mode);
- `rectifier/sv_pwm`: the library three-level PWM (carriers in phase disposition, 4 kHz, double update)
  gives the gate commands Q1 ... Q4 of each leg of the T-type pattern; the Vienna leg uses Q2 and Q3.

Signs: the measured converter current goes through a gain −1 (rectifier convention in the control) and the
modulator reads the duty of `sv_pwm_cm` as the high fraction, so the converter voltage is the opposite of the
control voltage. The two inversions close the current loop with the right sign (checked in the switched model).

## Parameters

`init_model.m` fills the struct `param`; the model reads only `param.*` names.

| Group | Content |
|---|---|
| `param.sim` | `simlength` 1.25 s, nonlinear iterations of the grid emulator, `check_model`, `design_figures` |
| `param.time` → `param.glb_time` | `fpwm` 4 kHz, double update, dead time 3 µs, `tc_factor` 200: `prv.timing_param` gives `ts_afe` 125 µs, `tc` 0.625 µs, `Nc`, `Ns_afe`, `fPWM_AFE`, ... (AFE domain only) |
| `param.grid` → `param.grid.emu` | 1 MVA, 690 V, Dyn11, core data: `prv.grid_param` adds the data set of the voltage class (50 Hz, η 95 %, `ucc` 5 %, iron losses 1.8 kW), `n1`, `Lm1`, the line no-load current and the object `grid_three_phase_emulator` |
| `param.frt` → `param.frt.data` | test 25.4 of VDE-AR-N 4110, both generators off (not applied): `prv.frt_param` merges `frt_settings` and `grid_fault_generator` of the library; the emulator reads `t_ramp` |
| `param.afe.hw`, `hw_in` | 250 kW hardware of the 690 V class (`prv.afe_hw_param`: `udc_nom` 1070 V, LCL filter, DC link) through `hw_afe_three_phase_setup` |
| `param.afe.dev` | switches and diodes of the bridge: data set `silicon_high_power_ideal_switch` (the numbers that were typed in the mask: switch 1.5 mΩ, diode 2 V and 0.15 mΩ, RC snubber 10 nF and 2.2 kΩ) |
| `param.afe.dclink`, `param.load` | initial DC voltage 1070 V, measuring dividers 107 kΩ, neutral point to ground 100 kΩ; load 5 Ω |
| `param.afe.ctrl` | `ctrl_afe_setup` of the library with the gains of this model: udc PI 4 / 45, current PI 0.25 / 35 on both axes, DDSRF-PLL 0.32 / 8 |
| `param.afe.pll`, `param.afe.behav` | initial value of the PLL frequency integrator (1 pu); clock gain, PWM phase, zero-sequence output gain, `pwr_lim_enable` |
| `param.ctrl`, `param.meas` | decay of the current PI integrators while clipped, clock alignment PI, 50 Hz filter of the emulator; ADC quantization and ranges |
| `param.afe.design` | result of the design check `prv.vienna_design` |

| `private/` function | What it does |
|---|---|
| `timing_param` | sample times of the AFE domain and the logging base `tc` |
| `grid_param` | transformer data set, magnetizing inductance, no-load current, `grid_three_phase_emulator` |
| `frt_param` | fault settings of the library, prints the fault summary |
| `afe_hw_param` | LCL and DC-link hardware of the voltage class, `hw_afe_three_phase_setup` |
| `vienna_design` | design check (below); one figure when `param.sim.design_figures` is true |
| `power_meter_bus` | the bus object `PowerMeterIecBus` of the power meter (base workspace) |
| `library_setup` | puts the library on the path (`pwr-control/library` next to this repository, cloned when missing) |
| `check_model_param_refs`, `check_from_goto` | checks after `open_system`: every `param.*` read by the active blocks exists; From tags without Goto |

Old workspace names and their place in `param`:

| Legacy name | Now |
|---|---|
| `hwdata.afe` | `param.afe.hw` |
| `afe_ctrl` | `param.afe.ctrl` |
| `glb_time` | `param.glb_time` |
| `glb_time.ts_inv` (current PI) | `param.glb_time.ts_afe` (same value, 125 µs) |
| `psm_ctrl.kp_i`, `psm_ctrl.ki_i` (current PI, PMSM control of the template) | `param.afe.ctrl.kp_is_xi`, `ki_is_xi` (pi_d) and `kp_is_eta`, `ki_is_eta` (pi_q), same values 0.25 and 35 |
| `CTRPIFF_CLIP_RELEASE` | `param.ctrl.clip_release` |
| `grid_emu` | `param.grid.emu` |
| `adc_quantization` | `param.meas.adc_quantization` |
| `time_gain_afe_module_1` | `param.afe.behav.time_gain_module_1` |
| `afe_pwm_phase_shift_mod1`, `u_cm_comp_mod1` | `param.afe.behav.pwm_phase_shift_mod1`, `param.afe.behav.u_cm_comp_mod1` |
| `kp_align`, `ki_align`, `lim_up_align`, `lim_down_align` | `param.ctrl.align.kp`, `ki`, `lim_up`, `lim_down` |
| `simlength`, `tc`, `glb_time.tc` (configuration set) | `param.sim.simlength`, `param.glb_time.tc` |
| typed in the blocks: load 5, np resistor 1e5, `Rudc_measure` 107e3, Vienna mask (5.5, 1.5e-3, 2, 0.15e-3, 10e-9, 2200) | `param.load.R`, `param.afe.dclink.R_np_gnd`, `param.afe.dclink.Rudc_measure`, `param.afe.dev.*` |

## Model changes (November 2026)

Edited in the XML of `vienna_rectifier.slx` (no MATLAB): 158 block parameters and the three entries of the
configuration set re-pointed to `param.*`, 9 typed values moved to `param` (the initial voltage of the two
DC-link halves now reads `param.afe.dclink.udc_init`), and these fixes:
- **DDSRF-PLL** (as in `frt_dqpll` after its fix): `pll_stageA/sequences` reads outputs 1 and 2 of `ddsrf`
  (positive sequence) instead of 3 and 4 (negative sequence); the gain −1 in `pll_stageB` is now 1; a new
  Saturation 0.5 ... 1.5 pu clamps the frequency integrator; its Unit Delay starts at
  `param.afe.pll.integrator_init` = 1 pu. The gains are 0.32 and 8 as in `frt_dqpll`: the defaults of
  `ctrl_afe_setup` (1 and 80) leave 34 degrees of phase margin with the decoupling filter in the loop.
- **Modulator**: the three lines into the Bus Creator of `rectifier/sv_pwm/modulator1` are named `du(t)`,
  `dv(t)`, `dw(t)`, the names that the gate-command Bus Selectors of the library PWM block select (as in
  `afe_dq_ttype_three_phase`).
- **Tags**: the grid phase of `AFE_control` reads `phase_grid_emulator` (was `gamma_grid_emulator`); the DC
  voltage of `afe_measurement_sim` reads `udc` (was `udc_mod1`); the PLL log of `data_to_worspace_module_1`
  reads `phase_grid_pll` and `omega_grid_pll_pu` of the DDSRF-PLL; `pwr_lim_enable` (no source) is now the
  Constant `param.afe.behav.pwr_lim_enable` = 0. Commented out, with the To Workspace blocks they alone feed:
  the thermal data of a template bridge (the ideal Vienna block has none), the DC-link current
  `ic_dclink_mod1`, the other PLLs of the template and the power meter of the template control.
- **Current PI**: reads the AFE gains and `ts_afe` instead of the PMSM gains and `ts_inv` (same numbers).
- **Tag visibility** (Davide, in MATLAB): a Goto Tag Visibility `Vgrid_unorm_ddsrf` in
  `AFE_control/afe_control`. The Goto of the DDSRF-PLL with this tag is scoped and its From in `pll_stageB`
  did not see it (so already before the changes; the first version of `check_from_goto` compared the tags
  only and missed it).

Checked with Octave stubs of Simulink (`find_system`, `get_param` on the saved XML): all 93 `param.*`
references of the active blocks resolve and every active From block reaches its Goto: `check_from_goto` now
also applies the visibility rules (global, local in the same subsystem, scoped under a Goto Tag Visibility
block that contains both), and flags the model before the visibility fix above. The switched results of the
design note are still to be compared with a run of the model.

## Design check (November 2026)

Numbers from `prv.vienna_design` (printed by the init) and from the switched model of the design note
(`vienna_sim.py`: the same plant, modulator and control, step `tc`).

1. **Operating point**: 229 kW, converter current 271 A peak (0.83 pu), grid power factor 0.99; converter
   voltage 0.913 pu of udc/√3, so 8.7 % of margin to the linear limit of the modulation; the converter voltage
   lags the current by δ = 4.3 degrees.
2. **LCL resonance** 1381 Hz, just above fs/6 = 1333 Hz; passive damping 5.9 % from the 50 mΩ, 5.8 % with
   the current loop (a converter-current loop with 1.5 periods of delay damps only resonances below fs/6).
3. **Current loop** (kp 0.25, ki 35): crossover 135 Hz (positive sequence) and 131 Hz (negative sequence),
   phase margin 70 and 78 degrees, gain margin 16 dB; unstable from kp = 1.67.
4. **DC-voltage loop** (kp 4, ki 45, C = 7.2 mF, load pole 56 rad/s): crossover 21 Hz, phase margin 98
   degrees. Below about 980 V the converter voltage of the full load (564 V phase peak) is beyond the linear
   range of the modulation: after a dip (the start, or a step from no load to full load: about 910 V) the
   current PI clips, the bridge rectifies like a diode bridge and udc recovers in 0.25 ... 0.4 s.
5. **DDSRF-PLL** (0.32, 8): 8 Hz, damping 1.0 without the filter; with the decoupling filter in the loop
   crossover 16 Hz, phase margin 58 degrees, slowest pole 37 rad/s. Locked (error below 5 degrees) at 44 ms.
6. **Start-up**: largest converter current 752 A (2.3 pu) at 20 ms; udc falls to 758 V (the load drains the
   DC link while the grid voltage rises and the PLL waits for the 12.5 ms reset of its trackers), then rises
   to 1171 V (+9.4 %; the bridge cannot give energy back) and is within 2 % after 288 ms. With the PLL
   integrator from 0 (as in `frt_dqpll`) the peak is 1228 A and udc 1513 V, lock at 112 ms: hence the
   initial value 1 pu. The peak has two causes. First, the udc PI: its reference starts at 0 with udc at 1 pu,
   so the error is −1 pu and the output sits at the lower limit −1.65 pu; the PI is in velocity form with
   back-calculation and stores x = out − kp·e = +2.35 pu also while clamped. As the reference rises and udc
   falls, the output rises with the error: positive at 8 ms, +1.16 pu at 15 ms, while udc is still 220 V
   above its reference (point 7 of `prv.vienna_design` prints the threshold, an error of −0.59 pu). Second,
   the current PI has no feed-forward of the grid voltage: its integrators build the converter voltage while
   the grid voltage rises, and the current overshoots its reference (2.26 pu against 1.14 pu at 20 ms).
7. **Steady state**: THD of the grid current 3.4 % (h5 2.0 %, h7 1.6 %, h11 1.1 %, h13 0.9 %), converter
   current 2.8 %; the dead time adds 0.4 %. Most of the low-order distortion comes from the zero crossings:
   with the min-max zero sequence the pole reference and the current have opposite signs over 2δ = 8.6
   degrees per period (11 degrees in the switched model), where the leg stays at the neutral point. A zero
   sequence that follows the sign of the current (an option tried in the switched model, not in the model)
   gives 2.5 % (h5 1.2 %, h7 0.9 %).
8. **Neutral point**: third-harmonic current 33 A, ripple of vC2 − vC1 5.4 V peak to peak (linear estimate
   and switched model agree), mean +0.3 V, no drift.

## Open points

- **Start-up peak** (point 6): not changed in the model. Recommended, one of two changes (switched model:
  about 455 A instead of 752 A, no udc overshoot, same steady state):
  - in `udc_ctrl`, the lower limit of the Saturation at 0 (the bridge cannot give a negative active current)
    and conditional integration: the Unit Delay keeps its state while the output is clamped, instead of
    taking out − kp·e. One subsystem of this model; 456 A, within 2 % after 317 ms. Each half alone is not
    enough: the lower limit 0 with the back-calculation gives 1068 A, conditional integration with the
    −1.65 pu limit gives 587 A and a dip to 647 V;
  - a feed-forward of the measured capacitor voltage on the output of the current PI (sample advanced by
    1.5 ts, scaled by √3·ubez/udc) and a udc reference that starts from the measured udc (the low-pass block
    of the library has no initial state): 455 A, within 2 % after 235 ms. The reference from the measured
    udc alone gives 782 A, the feed-forward alone 511 A and 1185 V.

  Both together give the same as the second; the feed-forward with the reference from 0 should not be
  combined with the changed PI (677 A, dip to 611 V). The dip to about 770 V is the load draining the DC link
  while the grid voltage rises from zero. Enabling the PWM only after the PLL lock is a further option.
- **Minimum load**: connecting the load after the start does not help. Without the feed-forward the start-up
  current alone charges the unloaded DC link to 1341 V; with it the start draws less than 30 A, but the DC
  link still creeps up to 1125 V in 0.2 s: near zero load the current pulses that remain are rectified and
  the bridge cannot give the energy back. A Vienna rectifier needs a minimum load.
- **Voltage margin** (point 4): 8.7 % at 1070 V. After a dip below about 980 V udc recovers in
  0.25 ... 0.4 s; a faster recovery needs a higher udc reference, if the devices allow it, or smaller load
  steps.
- **Zero-crossing distortion** (point 7): a Vienna-aware zero sequence needs a change of `sv_pwm_cm` (C
  code of the library) or a block after it.
- **LCL** (point 2): the resonance depends on the grid inductance, here the transformer leakage only; a
  stiffer grid moves it further above fs/6, where the converter-current loop cannot damp it.
- **Neutral point**: no balancing loop (the zero-sequence PI is for parallel modules and its output gain is
  0); fine in this symmetric case, to check with unbalanced grids or devices.
- The FRT data of the grid emulator are computed but not applied; the Vienna control has no FRT logic.
- Ten scoped Gotos have no reader (`Cdc1_current`, `np_current`, `iz_*`, ...): harmless.
