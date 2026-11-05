# ps_25V_12kA — 25 V / 12 kA power supply: AFE, phase-shift full bridge, centre-tapped diode rectifier

A 300 kW DC power supply for a high-current process: 25 V at 12 kA (a 10 V / 12 kA case is also stored).
The 480 V / 60 Hz grid feeds a two-level active front end (AFE, 325 kW, DC link 750 V). The DC link feeds a
phase-shift full bridge (PSFB) at 4 kHz with a centre-tapped transformer 34:2:2, a two-diode rectifier, an
output inductor of 0.4 µH and an output capacitor of 45 mF. The full bridge controls the output current.
Design document: `docs/ps_25V_12kA_r.pdf`.

| File | Content |
|---|---|
| `init_model.m` | initialization as a live script (plain-text format: open it in the Live Editor). Every number goes in the struct `param`; then it prints the design figures, opens the model, selects the full-bridge variant and checks the model |
| `ps_25V_12kA_private.m` | handles to the functions of `private/` (`prv.<name>`): the Live Editor runs a live script from a copy and does not see `private/` |
| `private/` | the parameter functions of this model and the checks (table below) |
| `ps_25V_12kA_r.slx` | the model (grid emulator, AFE, full bridge, rectifier, load, logging) |
| `docs/ps_25V_12kA_r.pdf` | design document |
| `scripts/plotting_sim_results_7.m`, `_8.m` | plots of the stored runs (`sim_results/readme.txt`); they read a workspace saved after either init |
| `init_model_legacy.m` | the old flat init, kept until the merge; it does not run with the library any more (next section) |

## Why the old init did not run

1. `grid_fault_generator;` (line 309) ran the library file as a script. It is now a function,
   `frt_data = grid_fault_generator(input, start_time_LVRT)`, so MATLAB stops with "Not enough input arguments".
2. `lithium_ion_battery_setup(V, P, soc, ts)` (line 557): the library function now takes the charge capacity as
   third argument, `(V, P, capacity, soc, ts)`, so the call with four arguments fails too.
3. With these two lines fixed, the model still did not compile. In the saved model 37 `From` blocks (27 tags)
   have no `Goto`. Most of them log the IGBT and MOSFET full bridges, which are commented out: the init switched
   the bridges but not the blocks that read their data. The others read signals that no block writes (list in
   "What changed in the model").
4. The init built the data of every converter of the template it came from (ISOP rail, DAB, CLLC, inverters,
   induction and permanent-magnet machines, UPQC transformer, filters) with more than 30 library functions, so a
   change in any of them could stop this model. The new init builds only what the model reads.

## The model

**Power stage.**
- Grid: library block `three_phase_voltage_source_with_frt_emulator` (480 V, 60 Hz, fault generator), equivalent
  grid impedance 17.5 µH + 2 mΩ per phase, Dyn11 transformer 480/480 V, 1750 kVA, ucc 5 %, made of three
  single-phase transformer blocks (delta primary coils of 24 turns, star secondary coils of 14 turns, star point
  grounded).
- AFE `afe_abc_mod1`: LC filter (`LFu` 330 µH, `CFu` 360 µF in star with 50 mΩ), two-level bridge with ideal
  switches (the IGBT and MOSFET variants are commented out), DC link `CFi` 2.4 mF, brake chopper 2.5 Ω.
- Full bridge `full_bridge_converter`: input capacitor 1.2 mF on the same DC link; three bridge variants (ideal
  switches, IGBT CM1200DW-34T with thermal model, SiC MOSFET with thermal model and ZVS snubbers), switched by the
  init; transformer `three_windings_single_phase_transformer` 34:2:2 (`Lm` 1 mH, leakage 20.0 µH referred to the
  primary: `Ld1` 10.0 µH, `Ld2` 34.7 nH on each secondary); rectifier diodes ABB 5SDF 0131Z0401 (Simscape diode
  with charge model) `diode_rectifier_1` and `_2` (3 to 5 and `trafo_2` are commented out); output inductor
  0.4 µH; output capacitor 45 mF + 0.1 mΩ; root block `Resistor` 25 V / 12 kA = 2.08 mΩ.
- `battery_1` (lithium-ion pack on the DC link): commented out; the init still gives it data.

**Control.**
- AFE, 8 kHz (4 kHz with double update): dq PI of the converter current (kp 0.25, ki 45), DC-link voltage PI
  (kp 2, ki 35, reference 750 V through a 200 ms filter) that gives the active current reference (limit 1.65 pu),
  PI of the reactive grid current. Four PLLs run in parallel and two Variant Source blocks choose one (mode 4:
  FHT-PLL in C-Caller).
- Full bridge, 8 kHz: modulator `full_bridge_zvs_pwm_modulator` (4 kHz carriers, 3 µs dead time); output current
  PI on the load current (kp 1, ki 18, output 0 ... 1 = phase displacement of leg B, 0 ... π); reference stepped
  at 25 ms to 12 kA with a slope of 120 kA/s; "DC transformer current compensation": a PI (kp 0.1, ki 2,
  limit ±0.1) on the DC input current of the bridge (low-pass filter 20 ms), added to the duty 0.5 of both legs
  (open point 1).

## How to run

1. Clone `pwr-control/library` next to this repository, or set the environment variable `PWR_CONTROL_LIBRARY`.
   `prv.library_setup()` at the top of `init_model.m` adds it to the path, and clones it when it is missing.
2. Open `init_model.m` in the Live Editor and press Run (the whole script: a section alone needs `prv` and
   `param` in the workspace). It prints the data of each part and the design figures, opens the model, closes the
   scopes, selects the full-bridge variant and checks the model.
3. Choose the full bridge with `param.sim.use_thermal_model` (IGBT) or `param.sim.use_mosfet_thermal_model`
   (MOSFET, has priority). Both 0 gives the ideal switches: this is the default, as in the legacy init. The stored
   runs 7 and 8 used the IGBT bridge.
4. Run the simulation: 1.75 s, `ode23tb`, `MaxStep` and Simscape local solver step `param.glb_time.tc` = 0.625 µs.

## Parameter structure

| Group | Content |
|---|---|
| `param.sim` | `simlength` 1.75 s, `use_thermal_model`, `use_mosfet_thermal_model` (bridge variant), `nonlinear_iterations` 3 (5 with a thermal bridge), `check_model`, `design_figures` |
| `param.time`, `param.glb_time` | AFE and full bridge 4 kHz with double update, dead time 3 µs, `fpwm_dab` 12 kHz (only for `ts_dab`), `tc_factor` 200; `glb_time = timing_param(param.time)` gives the names the blocks read: `fPWM_AFE`, `dead_time_AFE`, `ts_afe` (125 µs), `fPWM_PSFBC`, `TRGO_PSFBC_double_update`, `dead_time_PSFBC`, `ts_psfbc` (125 µs), `ts_dab` (41.7 µs), `tc` (0.625 µs), `decimation_tc`, `Nc`, `Ns_afe`, `Ns_dab` |
| `param.grid` | 480 V, 60 Hz, 1750 kVA, ucc 5 %, eta 95 %, iron losses 1400 W, turns 24/14, core, `eq_grid_resistance`; `frt = grid_fault_param(...)` (test 25.4 of the legacy init, both enables 0); `emu = grid_emulator_param(...)`: bases, equivalent grid inductance, the 50 Hz filter of the emulator (`sof_50Hz`), transformer `emu.trafo` (coil data of the blocks: `Ld1`, `Rd1`, `Ld2`, `Rd2`, `Lm1`, `Rfe1`, `n1`, `n2`) |
| `param.afe` | `pwr_nom` 325 kW; `hw = afe_hw_param(...)` (bases `ubez` 391.9 V, `ibez` 614.2 A, `udc_nom` 750 V, `CFi`, `LFu`, `CFu`, `RCFu`, `Rbrake`); `behav` (clock gain, carrier phase, zero sequence, brake step); `variant.dq_pll_mode1..4` (variant, exactly one at 1); `ctrl = afe_ctrl_param(...)` (gains, first harmonic trackers, observer, rms windows, time alignment PI); `ideal_switch`, `igbt` (device data) |
| `param.psfbc` | `pwr_nom` 325 kW; `hw_in` (numbers of the converter) and `hw = psfbc_hw_param(...)` (transformer, capacitors, inductors, bases `idc1_bez` 433 A, `idc2_bez` = `idc2_nom` 12 kA, `udc2_bez` 45 V); `ctrl` (`kp_idc` 1, `ki_idc` 18, `kp_idc_trafo` 0.1, `ki_idc_trafo` 2, `lim_idc_trafo` 0.1, `tau_idc_trafo` 20 ms; `kp_udc`, `ki_udc` not used); `ideal_switch`, `igbt`, `mosfet` (device data) |
| `param.diode.rectifier` | data of the rectifier diodes (forward voltage, resistance, reverse recovery, thermal resistances, snubber) |
| `param.thermal` | heatsink (liquid-cooled plate, 15 mK/W to the water at 40 °C), `heatsink` struct read by the bridges with thermal model, `Tambient`, `DThs_init` |
| `param.battery.pack_1` | `battery_param(...)`: 750 V, 250 kW, 333 Ah (1C), soc 0.85, with the legacy changes (R0 and R1 halved, C1 divided by 50) |
| `param.load` | `u_out` 25 V, `i_out` 12 kA, `R` = 2.08 mΩ (was typed in the block) |
| `param.meas` | `adc_quantization`, `adc12_quantization` 1/2^11, `adc16_quantization` 1/2^15 |
| `param.design` | numbers of the design figures (when `param.sim.design_figures` is true) |

### Functions of `private/`

| Function | Role |
|---|---|
| `timing_param(t)` | the two clock domains (AFE, full bridge) and `ts_dab` (formulas of `timing_setup`) |
| `grid_emulator_param(g, fault)` | grid emulator, equivalent grid impedance, Dyn11 transformer (formulas of `grid_three_phase_emulator` and `three_phase_transformer_setup` after the fix of the delta primary of November 2026) |
| `grid_fault_param(...)` | fault test of the emulator (`frt_settings` + `grid_fault_generator`, reduced to the kinds of this model) |
| `afe_hw_param(...)`, `afe_ctrl_param(...)` | AFE hardware and control (`three_phase_afe_hwdata`, `hw_afe_three_phase_setup`, `ctrl_afe_setup`) |
| `psfbc_hw_param(...)` | full-bridge hardware (`single_phase_psfbc_hwdata` + `hw_psfbc_single_phase_setup`, numbers of the data set PSFBC_800V from the init) |
| `device_ideal_switch_param`, `device_igbt_param`, `device_mosfet_param` | device data sets (the three used ones are embedded; other names are read from the library data sets) |
| `battery_param(...)` | lithium-ion pack (`lithium_ion_battery_setup`) |
| `design_figures(param)` | the design figures below (plain MATLAB, no toolbox) |
| `check_model_param_refs(model, param)` | lists the `param.*` names read by the active blocks and reports the missing ones |
| `check_from_goto(model)` | lists the `From` blocks of the active part without an active `Goto` |
| `library_setup()` | puts `pwr-control/library` on the path (clones it when missing) |

## Old names and new names

| Old (flat workspace) | New |
|---|---|
| `glb_time.*` | `param.glb_time.*` |
| `glb_time.ts_inv` (AFE current PI) | `param.glb_time.ts_afe` (the inverter clock was the AFE clock) |
| `hwdata.afe.*` | `param.afe.hw.*` |
| `hwdata.psfbc.*`, `mu0` | `param.psfbc.hw.*`, `param.psfbc.hw.mu0` |
| `hwdata.dab.idc1_bez` | `param.psfbc.hw.idc1_bez` (value changed, open point 5) |
| `afe_ctrl.*`, `CTRPIFF_CLIP_RELEASE` | `param.afe.ctrl.*`, `param.afe.ctrl.clip_release` |
| `kp_align`, `ki_align`, `lim_up_align`, `lim_down_align` | `param.afe.ctrl.align.kp`, `.ki`, `.lim_up`, `.lim_down` |
| `time_gain_afe_module_1`, `afe_pwm_phase_shift_mod1`, `u_cm_comp_mod1`, `time_aux_power_supply_fault` | `param.afe.behav.time_gain_module_1`, `.pwm_phase_shift_mod1`, `.u_cm_comp_mod1`, `.time_aux_power_supply_fault` |
| `use_dq_pll_mode1` ... `use_dq_pll_mode4` | `param.afe.variant.dq_pll_mode1` ... `dq_pll_mode4` |
| `psfbc_ctrl.*` | `param.psfbc.ctrl.*` |
| `grid_emu.*`, `frt_data.*` | `param.grid.emu.*`, `param.grid.emu.frt.*` |
| `filters.afe.SOF_50Hz.ss_c.A`, `.B`, `.C` | `param.grid.emu.sof_50Hz.A`, `.B`, `.C` |
| `ideal_switch.*` (AFE, brake), `ideal_switch.dab.*` (full bridge) | `param.afe.ideal_switch.*`, `param.psfbc.ideal_switch.*` |
| `igbt.afe.*`, `igbt.psfbc.*`, `mosfet.psfbc.*` | `param.afe.igbt.*`, `param.psfbc.igbt.*`, `param.psfbc.mosfet.*` |
| `heatsink.*`, `Tambient`, `DThs_init` | `param.thermal.heatsink.*`, `param.thermal.Tambient`, `param.thermal.DThs_init` |
| `diode.rectifier.*` | `param.diode.rectifier.*` |
| `lithium_ion_battery_1.*` | `param.battery.pack_1.*` |
| `adc_quantization`, `adc12_quantization` | `param.meas.*` |
| `nonlinear_iterations` | `param.sim.nonlinear_iterations` |
| `simlength` (`StopTime`), `tc` (`FixedStep`), `glb_time.tc` (`MaxStep`) | `param.sim.simlength`, `param.glb_time.tc` |

## What changed in the model (5 November 2026)

The model was edited as XML (no MATLAB in the cloud): same file, same `ModelUUID`.
- 616 names re-pointed in the block parameters, mask values and variant controls, plus 3 entries of the
  configuration. The active blocks read 159 distinct `param.*` names with the ideal-switch bridge (173 with the IGBT
  bridge, 166 with the MOSFET bridge, 228 in the whole model with every block active); all exist.
- Values that were typed in blocks now read `param`: root `Resistor` 25/12e3 → `param.load.R`;
  `pi_ctrl_idc_trafo_1` kp 0.1, ki 2, limits ±0.1 and its low-pass filter tau 20 ms →
  `param.psfbc.ctrl.kp_idc_trafo`, `ki_idc_trafo`, `lim_idc_trafo`, `tau_idc_trafo`.
- Configuration: `StopTime` → `param.sim.simlength`, `MaxStep` → `param.glb_time.tc`; `FixedStep` was `tc`, a name
  the legacy init did not define (not used by the variable-step solver) → `param.glb_time.tc`.
- 56 blocks commented out. A `From` without its `Goto` stops the diagram update; nothing the model computes is lost.
  - 30 blocks follow the full-bridge variant: the `From`, `To Workspace` and scope blocks of `afe_measurement_sim`
    and `scope` that read the data of the IGBT bridge (tags `Q1/Q2_full_bridge_device_current`, `_voltage`,
    `_power_losses`, `D1/D2_full_bridge_device_power_losses`, `full_bridge_igbt_data`) and of the MOSFET bridge
    (`full_bridge_mosfet_data`). They are saved commented, like their bridges, and the init switches them together
    with the bridges (also the four readers of the ideal-switch bridge, tag `full_bridge_ideal_switch_data`).
  - 26 blocks stay commented: `frt_active` in the PLL gain scheduling (no block raises it, the Switch keeps `pll_p`);
    the negative-sequence averages and `i_grid_seq_ref` in `interesting scopes`; `P1p_global`, `P1n_global`,
    `Q1n_global` and their `To Workspace` blocks; `afe_igbt_data` (the IGBT AFE bridge is commented out);
    `idr3..5`, `udr3..5` (rectifiers 3 to 5 are commented out); `uout_ref_pu_psfbc` (the loop controls the current).
- `scripts/plotting_sim_results_7.m` and `_8.m`: when the saved workspace has `param`, they take `glb_time` and
  `hwdata.afe` from it, so they read results saved after either init.

**Checks done (in Octave, no MATLAB here).**
- The new init runs to the end for the three full-bridge variants, with stubs of `find_system`, `get_param` and
  `set_param` that read the block data of the `.slx`. Every `param.*` name of the active blocks exists, every block
  that the init comments in or out exists, and every active `From` has its `Goto`.
- The legacy init ran with the current library (the two calls above fixed, the library classes adapted to Octave),
  and every one of the 245 distinct expressions re-pointed in the model was evaluated with both inits: 240 give
  the same value. The other 5:
  - `1/hwdata.dab.idc1_bez` (1/350 A) → `1/param.psfbc.hw.idc1_bez` (1/433 A), open point 5;
  - `sof_50Hz.A`, `.B`, `.C`: another state-space realization of the same filter 1/(τs+1)², τ = 1/(2π·50 Hz). The
    filter starts from zero state, so its output does not change (the same realization as in the other models with
    `grid_emulator_param`);
  - `FixedStep`: `tc` was not defined by the legacy init.
- The model itself was not run in Simulink. Please run `init_model.m` and one simulation in MATLAB.

## Design figures (printed by `init_model.m`)

Averaged models without switching (`private/design_figures.m` explains them): estimates to compare with the
simulation.

| | Value |
|---|---|
| Secondary voltage at full duty, n·udc | 44.1 V (750 V, 34:2) |
| Duty lost in the commutation of the rectifier | 0.226 (0.83 mΩ equivalent) |
| Duty needed at 25 V, 12 kA | 0.826 |
| Largest duty with the compensation at its limit | 0.800 |
| Output at the largest duty | about 11.6 kA, 24.2 V |
| Power from the DC link | about 298 kW (conduction 16.7 kW, iron 0.5 kW) |
| Output filter (0.4 µH, 45 mF) | 1186 Hz, characteristic impedance 2.98 mΩ, load 2.08 mΩ |
| Full-bridge current loop, kp 1, ki 18 | phase margin 34°, gain margin 1.6 dB, closed-loop pair at 1349 Hz damped 0.057, slow pole −10 rad/s, stable for kp < 1.21 |
| Same loop, kp 0.3, ki 100 | crossover 21 Hz, phase margin 110°, gain margin 11.8 dB, pair at 1267 Hz damped 0.39, slow pole −92 rad/s |
| AFE LCL resonance | 1493 Hz (fs/6 = 1333 Hz), grid side 34.9 µH |
| AFE current loop, kp 0.25, ki 45 | crossover 82 Hz, phase margin 69°, gain margin 23.6 dB |
| AFE DC-link loop, kp 2, ki 35, 3.6 mF | crossover 60 Hz, phase margin 45.5°, gain margin 19.7 dB |
| With the current loop at kp 0.75 | current loop 231 Hz, 73°, 14.1 dB; DC-link loop 57 Hz, 73°, 22.4 dB |

## Open points

1. **The DC transformer current compensation caps the output.** It reads the DC input current of the bridge,
   which is positive whenever the converter delivers power, so its PI sits at its limit (−0.1) and both legs get
   the duty 0.4. With equal duty 0.4 on both legs the bridge applies voltage for at most 0.8 of each half period,
   while 25 V at 12 kA needs about 0.83: the current loop saturates near 11.6 kA, 24.2 V (estimate; the design
   document reports 12 kA). Also, the same duty on both legs gives the same positive and negative volt-seconds, so
   this correction cannot remove a DC flux in the transformer. Please check the output current and `d1` in the
   simulation. `param.psfbc.ctrl.lim_idc_trafo = 0` switches it off (largest duty 1). A working compensation would
   measure the DC part of the transformer current and change the duty of the two legs in opposite directions
   (leg A 0.5 + δ, leg B 0.5 − δ), which makes the positive and the negative pulses differ.
2. **The output current loop is near its stability limit** with kp 1 (gain margin 1.6 dB, output filter mode at
   1.35 kHz damped 6 %): expect ringing of the load current. kp 0.3, ki 100 gives 11.8 dB and a faster slow pole.
   The gains are left as in the legacy model.
3. **AFE: the DC-link loop (60 Hz) is close to the current loop (82 Hz).** The phase margin is still 45°. kp_is 0.75
   separates them (231 Hz against 57 Hz). The LCL resonance (1493 Hz) is above fs/6 with converter-current
   feedback; the margins above include the 50 mΩ in series with `CFu`, which damps it.
4. **`ts_dab` in the compensation.** Its low-pass filter and PI are set with `ts_dab` = 41.7 µs, but they run
   every 125 µs (the full-bridge trigger): their time constants are three times longer than the numbers in the
   init. The battery block (commented out) also uses `ts_dab`.
5. **Input current base.** The legacy model normalized the bridge input current with `hwdata.dab.idc1_bez` =
   350 A, the base of the DAB data set of the template. It now uses the base of this full bridge,
   `param.psfbc.hw.idc1_bez` = 325 kW / 750 V = 433 A, so the compensation sees 19 % less. With the compensation at
   its limit (point 1) this changes nothing at rated load.
6. **Design document and init differ.** The document gives 10 kHz and Ls = 8 µH for the full bridge, the
   CM1200DW-24T module, Lgrid ≈ 50 µH and Cdc2 = 50 mF. The init (as the legacy one) has 4 kHz, Ls = 20 µH, the
   CM1200DW-34T data set, 17.5 µH plus the transformer leakage, and 45 mF.
7. **Grid transformer.** Its coil data follow the library class after the fix of the delta primary (4 November
   2026): the factor 3 of a delta coil is on `Ld1`, `Rd1` and `Rfe1`; `Lm1` stays 144.8 mH (from the core) and
   `i1m` = 15.2 A is the line no-load current. The values are the same as those of the library now. Before the fix
   the legacy init gave other transformer losses.
8. **Rectifier diodes with charge model.** `sim_results/readme.txt` notes that with the charge model the thermal
   behaviour of the diodes has no meaning.
