# ps_isop_rail — rail auxiliary power supply: four input-series output-parallel phase-shift full bridges

A 1 MW class auxiliary power supply for a 3 kV DC railway catenary. Four identical 250 kW isolated DC/DC
modules are connected **in series at the input** and **in parallel at the output** (ISOP), so each module
sees only one quarter of the catenary voltage and gives a common 750 V output. Each module is a
phase-shift full bridge (PSFB, ideal switches, 4 kHz) with a 1:1 transformer. The special feature of this
model: the transformers work in **pairs**. The secondaries of modules 1 and 4 are connected in parallel on
the AC side and feed one diode rectifier, the secondaries of modules 2 and 3 feed the second rectifier. One
voltage PI gives the same phase shift to the four bridges; there is no input-voltage sharing loop.

| File | Content |
|---|---|
| `init_model.m` | initialization: every number in the struct `param`, design figures, then it opens the model, closes the scopes and checks the `param.*` references. Run it **from this folder** (the functions of `private/` are visible only to the files of this folder). |
| `private/` | the parameter functions of this model (table below) |
| `ps_isop_rail.slx` | the model (catenary emulator, four modules, two rectifiers, control, logging) |
| `claude_analysis/` | design check: switched simulation of the same circuit in Python, results and figures (see "Simulation results") |
| `trafo/four_winding_transformer.tex/.pdf` | derivation of a four-winding transformer (two primaries, two secondaries on one core); not used by the model, see open point 3 |
| `init_model_legacy.m` | the old flat init, kept until the merge; it no longer runs with the library (open point 9) |

A short note on the architecture and on the input-voltage sharing is in pwr-control/repositories-documentation,
`research/power_electronics/isop_rail_auxiliary_supply` (English, with its own copy of the simulator).

## The architecture

**Why input series, output parallel.** The catenary voltage (3000 V nominal, 2000 ... 3600 V permanent range
in EN 50163) is too high for one bridge with 1.7 kV devices. Four modules in series at the input share it:
750 V per module at 3 kV, 900 V at 3.6 kV. Each module needs its own isolation (the transformer) because
the inputs are at different potentials and the outputs are in parallel.

**Power stage of the model.**

```
battery_1 (catenary emulator, 4.5 kV) ── 1 mH ── Ldc_dc1 1 mH ── CFi_dc1 of modules 1, 2, 3, 4 in series
                                                                (1.2 mF each, one full bridge on each)

 module 1 ── T1 ──┐ AC                          module 2 ── T2 ──┐ AC
                  ├── rectifier_1 ── 250 µH ──┐                  ├── rectifier_2 ── 250 µH ──┐
 module 4 ── T4 ──┘                           │ module 3 ── T3 ──┘                           │
                                              └──────── Cdc_dc2 1.2 mF ── load 0.5 Ω ────────┘
```

**Averaged model of one module.** With the phase shift D (pu of π: the fraction of the period with voltage
on the transformer) the rectified output of module k is `n·D·v_k − Req·i_k`, where `v_k` is the input voltage
of the module and `Req = 4·n²·Ls·f = 0.417 Ω` is a lossless resistance that models the duty-cycle loss (the
time the leakage `Ls` needs to reverse the current). The input current of the module is `v_o·i_k / v_k`
(power balance).

**Why the input voltages balance without a sharing loop (common phase shift).** If the input voltage of one
module rises, its output voltage rises, it gives more output current, it takes more current from its input
capacitor and its voltage falls again. Linearized: the restoring conductance is
`g = (v_o/v_k)·(n·D/Req − i_k/v_k) ≈ 1 S`, so a deviation decays with `τ = Cin/g ≈ 1.2 ms`. The coupling goes
through `Req`, which behaves like a resistance: it moves real power from the module with the higher voltage.
In this model this holds **between the two rectifiers** (pair 1+4 against pair 2+3).

**Why it is weak inside a pair.** The two bridges of a pair are tied together on the AC side, through their
leakage inductances. A difference of their input voltages gives a difference of the *amplitudes* of their AC
voltages, and between two AC sources tied through inductors an amplitude difference drives a reactive
(circulating) current, not real power. In the fundamental-harmonic model the power of module k is
`P_k = V_k·V_r·sin θ / X_k` (`V_k` ∝ `v_k` its AC voltage, `V_r` the voltage at the rectifier, `X_k` its
leakage reactance), so its input current `P_k / v_k` does **not** depend on `v_k`: there is no restoring
force, only the losses give a little (τ ≈ 64 ms in the simulation instead of ≈ 2.7 ms between the pairs).
And a leakage difference changes `X_k`, so it changes the input current directly: the module with the larger
leakage draws less current and its capacitor charges up. This is open point 3.

## The model

**Power stage.**
- `battery_1/lithium_ion_battery`: library lithium-ion pack used as catenary emulator, 4500 V nominal
  (1.5 × the catenary), 4696 V open circuit at soc 0.85, 1249 cells, state-of-charge estimator at `ts_dab`;
  1 mF terminal capacitor and the 1 mH line inductor `battery_1/Inductor`. As in the legacy init `R0` and
  `R1` are halved (23.4 mΩ each) and `C1` is divided by 200 (2.5 mF).
- `isop/Ldc_dc1`: 1 mH + 3.14 mΩ in series with the string.
- `isop/fb_iso_mod_1..4`: input capacitor `CFi_dc1` 1.2 mF + 1 mΩ, library block `full-bridge ideal switch
  based` (1.5 mΩ, diode 2 V + 0.15 mΩ, snubber 2200 Ω + 10 nF), library `single_phase_transformer` 1:1,
  `Lm` 1 mH, `Rfe` 1 kΩ, `Rs1` = `Rs2` = 1 mΩ, `Ld1` = `Ld2` = 13.0 µH (`Ls` = 26.0 µH referred to the primary,
  from the DAB formula `Ls = V²/(8·f·P·2.7)` of the library data set, see open point 1).
- `isop/rectifier_1`, `isop/rectifier_2`: library `full_bridge_diode_rectifier` (piecewise-linear diodes,
  0.977 V + 22 µΩ, data of ABB 5SDF 0131Z0401), `Ldc_dc2` / `Ldc_dc3` 250 µH + 0.39 mΩ each,
  `Cdc_dc2` 1.2 mF + 0.1 mΩ, root `Resistor` 0.5 Ω.

**Control** (`isop/phase_shift_full_bridge_ctrl`, 8 kHz: the modulator gives a trigger at both peaks of the
carrier, ts = 125 µs):
- modulator `full_bridge_zvs_pwm_modulator`: triangle carriers at 4 kHz, leg B delayed by `phase_shift·T/2`,
  duty `d1` = 0.5 on both legs, 3 µs dead time; the same carrier and phase shift for the four bridges;
- `ui_ctrl/isop_rail_ctrl/voltage control/pi_ctrl`: output voltage PI, `kp_udc` 0.5, `ki_udc` 18, output
  limits 0 ... 1 (phase shift in pu of π), reference `udc_ref` = 1 pu = 750 V applied as a step at t = 0;
  feedback quantized (1/2^11) and sampled at the trigger;
- `current control/pi_ctrl`: output current PI, computed, output not connected (open point 7);
- `dc_trafo_current_measurements_and_compensation`: "DC transformer current" PI, computed, output not
  connected (open point 5).

## How to run

1. Add the `library` repository to the MATLAB path with subfolders (`init_model.m` does it for
   `C:\github\library`). The library gives the Simulink/Simscape blocks and the C sources of the
   configuration, no parameter.
2. From this folder run `init_model.m`. It fills `param`, prints the local functions of `private/`, the
   module data and the design figures, opens `ps_isop_rail.slx`, closes the scopes and checks that every
   `param.*` name read by the model exists (`check_model_param_refs`).
3. Run the simulation (`param.sim.simlength` = 1.75 s, `ode23tb`, `MaxStep` and Simscape local solver step
   `param.glb_time.tc` = 0.625 µs).

## Parameter structure

| Group | Content |
|---|---|
| `param.sim` | `simlength` 1.75 s, `nonlinear_iterations` 3, `check_param_refs` |
| `param.time`, `param.glb_time` | `fpwm` 4 kHz, `trgo` 1 (double update), `dead_time` 3 µs, `t_measure` (= `simlength`), `tc_factor` 200 (tc = 0.625 µs), `fpwm_dab` 12 kHz; `glb_time = timing_param(param.time)` gives the names the blocks read: `fPWM_ISOP_RAIL`, `tPWM_ISOP_RAIL`, `TRGO_ISOP_RAIL_double_update`, `ts_isop_rail` (125 µs), `dead_time_ISOP_RAIL`, `minimum_pulse_time_ISOP_RAIL`, `fPWM_DAB`, `tPWM_DAB`, `TRGO_DAB_double_update`, `ts_dab` (41.7 µs), `tc`, `decimation_tc`, `t_measure`, `Nc`, `Ns_isop_rail`, `Ns_dab` |
| `param.isop_rail` | catenary data (`catenary_nominal_voltage` 3000 V, `catenary_maximum_voltage`, `catenary_nominal_power` 10.8 MW, `catenary_nominal_current`), `number_of_systems` 4, `pwr_nom` 250 kW per module, `application_voltage` 750 V per module, `hw_in` (the numbers of one module), `design` (informative figures) |
| `param.isop_rail.hw` | `isop_rail_hw_param(750, 250e3, 4e3, hw_in)`: transformer `n1`, `n2`, `Lm`, `Rfe`, `Rs1`, `Rs2`, `Ld1`, `Ld2`, `core_length`, `core_mur`, `mu0`; `Cdc_dc1`, `RCdc_dc1`, `Cdc_dc2`, `RCdc_dc2`, `Ldc_dc1`, `RLdc_dc1`, `Ldc_dc2`, `RLdc_dc2`; bases `idc1_bez`, `idc2_bez`, `udc1_bez`, `udc2_bez`, `idc2_nom` |
| `param.isop_rail.ideal_switch`, `param.dev` | `device_ideal_switch_param('silicon_high_power_ideal_switch', ...)`: `Rds_on`, `Vth`, `Vdon_diode`, `Rdon_diode`, `Csnubber`, `Rsnubber` |
| `param.diode.rectifier` | rectifier diode data of the legacy init (`Vf`, `Rdon`, `Csnubber`, `Rsnubber` are read) |
| `param.isop_rail.ctrl` | `kp_udc` 0.5, `ki_udc` 18, `udc_ref` 1; `kp_idc`, `ki_idc` (current PI, not connected); `kp_idc_trafo` 0.1, `ki_idc_trafo` 2, `lim_idc_trafo` 0.1, `tau_idc_trafo` 20 ms (DC transformer current PI, not connected; these four were typed in the blocks) |
| `param.battery` | `nominal_battery_voltage_1` 4500 V (also read by the battery mask as its default), `nominal_battery_power` 10.8 MW, `initial_battery_soc` 0.85, `charge_capacity_1` 2400 Ah (assumption, open point 8), `pack_1 = battery_param(...)` with the legacy changes, `L_line` 1 mH (was typed in the block) |
| `param.load` | `rload` 0.5 Ω (was typed in the block) |
| `param.meas` | `adc_quantization` 1/2^11, `adc12_quantization` (read by the model), `adc16_quantization`, full scales |

### Functions of `private/`

| Function | Role |
|---|---|
| `timing_param(t)` | the clock domain of the four bridges and of the control (and `ts_dab` of the battery estimator) |
| `isop_rail_hw_param(udc1, P, fpwm, hw_in)` | data of one module: transformer, capacitors, inductors, bases (formulas of `isop_rail_aux_application` + `hw_isop_rail_setup` of the library, numbers from the init) |
| `device_ideal_switch_param(...)` | ideal-switch data set of the bridges (embedded, other data sets through the library) |
| `battery_param(...)` | lithium-ion pack (formulas of `lithium_ion_battery_setup`) |
| `check_model_param_refs(model, param)` | lists the `param.*` names read by the model and reports the missing ones |

## Old names and new names

| Old (flat workspace) | New |
|---|---|
| `glb_time.*` | `param.glb_time.*` |
| `glb_time.fPWM_PSFBC`, `TRGO_PSFBC_double_update`, `dead_time_PSFBC` | `param.glb_time.fPWM_ISOP_RAIL`, `TRGO_ISOP_RAIL_double_update`, `dead_time_ISOP_RAIL` |
| `glb_time.ts_psfbc`, `glb_time.ts_afe`, `glb_time.ts_isop_rail` | `param.glb_time.ts_isop_rail` (one clock: same frequency, double update and dead time) |
| `glb_time.Ns_afe` | `param.glb_time.Ns_isop_rail` |
| `hwdata.isop_rail.*`, `mu0` | `param.isop_rail.hw.*`, `param.isop_rail.hw.mu0` |
| `ideal_switch.isop_rail.*` | `param.isop_rail.ideal_switch.*` |
| `isop_rail_ctrl.*` | `param.isop_rail.ctrl.*` |
| `lithium_ion_battery_1.*` | `param.battery.pack_1.*` |
| `diode.rectifier.*` | `param.diode.rectifier.*` |
| `simlength`, `nonlinear_iterations` | `param.sim.*` |
| `tc` (configuration, `FixedStep`) | `param.glb_time.tc` |
| `adc12_quantization` | `param.meas.adc12_quantization` |

## What changed in the model (1 November 2026)

The model was edited as XML (no MATLAB in the cloud): same file, same `ModelUUID`.
- 247 parameter changes in the block dialogs and in the configuration; the model reads 75 distinct
  `param.*` names, all defined by the init (checked in Octave, see below).
- Values that were typed in blocks now read `param`: voltage reference `isop_rail_ctrl/Constant` (it had
  no value, so the block default 1 was used) → `param.isop_rail.ctrl.udc_ref`; `pi_ctrl_idc_trafo_1` kp 0.1,
  ki 2, limits ±0.1 and its low-pass filter tau 20 ms → `param.isop_rail.ctrl.*_idc_trafo`; root `Resistor`
  0.5 Ω → `param.load.rload`; `battery_1/Inductor` 1 mH → `param.battery.L_line`.
- Configuration: `StopTime` → `param.sim.simlength`, `MaxStep` → `param.glb_time.tc`, `FixedStep` was `tc`
  (a name the legacy init did not define; not used by the variable-step solver) → `param.glb_time.tc`.
- 86 blocks commented: 52 `From` blocks whose `Goto` does not exist in this model (leftovers of the AFE,
  inverter and PSFB template the model was built from) and the 34 scopes / `To Workspace` blocks fed only
  by them. Tags: `ig_abc`, `ug_abc`, `power_meter_iec`, `icu/icv/icw_CFu_mod1`, `i_abc_line`, `u_abc_line`,
  `udc_mod1`, `psfbc_current_output`, `psfbc_voltage_output`, `LFu_dc2_current`, `CFu_dc2_current`,
  `u1/i1/u2/i2_psfbc_transformer_1`, `Q1/Q2/D1/D2_full_bridge_device_*`, `full_bridge_1_ideal_switch_data`,
  `full_bridge_mosfet_data`, `full_bridge_igbt_data`, `afe_ideal_switch_data`, `diode_rectifier_D1/D2_ploss`,
  `diode_rectifier_JH/HA_temp_D1`, `uout_ref_pu_isop_rail`. A `From` without its `Goto` stops the diagram
  update, so they had to go; nothing that the model computes is lost.

**Checks done.** In Octave the legacy init (with the library functions it called) and the new init were
run side by side and every expression re-pointed in the model was evaluated with both: 80 same, 1 different
(the battery charge `Q`: 50 Ah fixed in the old library function, 2400 Ah now, open point 8), 1 not defined
in the legacy init (`FixedStep`). The 75 `param.*` names of the model all exist. The model itself was not
run in Simulink here; please run it once in MATLAB.

## Design figures (printed by `init_model.m`)

- Operating point: 750 V, 1500 A, 1.125 MW (281 kW and 375 A per module); battery 4696 V open circuit,
  1171 V per module.
- Leakage 26.0 µH → `Req` = 0.417 Ω, duty-cycle loss 156 V at 375 A; phase shift needed 0.777.
- At the nominal catenary voltage (750 V per module) the largest output at this load is 591 V: the 750 V
  reference cannot be reached (open point 1).
- Output filter (two 250 µH branches on 1.2 mF) 411 Hz; voltage loop crossover 4.8 Hz, phase margin 129°.
- Input voltage sharing between the rectifiers: τ = 1.21 ms (first order, output inductors left out).

## Simulation results (`claude_analysis/`)

`isop_sim.py` is a switched simulation of the same circuit with the values of `init_model.m` (Python +
numba): backward Euler at tc = 0.625 µs like the Simscape local solver, ideal switches and piecewise-linear
diodes with their snubbers, the transformer T model, the battery cell model, the modulator and the voltage PI
of the model with the 1/2^11 quantization and one sample of delay. It is not the Simscape model (I cannot run
MATLAB here) but it has the same structure and numbers. Run `python3 isop_sim.py <case> ...` (cases at the
end of the file), then `plots.py` and `analyze.py` / `analyze_win.py` / `classic_isop_estimate.py`; all
numbers are in `design_check_ps_isop_rail_output.txt`, the figures in `figures/` (PDF; `plots.py` also writes PNG
files, which git ignores).

- **Start and steady state (the model as it is).** From the DC steady state of the network (`DoDC` on) the
  output reaches 95 % in 178 ms and 99 % in 295 ms, no overshoot. Steady state: 751.5 V mean (the PI holds
  the *sampled* value at 750.1 V; the 3.3 V peak-to-peak 8 kHz ripple is sampled always at the same point),
  phase shift 0.782 (the init predicts 0.777), 1170.9 V on each module, string current 246 A, primary current
  380 A rms / 460 A peak, magnetizing current 52 A peak, 751.5 A from each rectifier. The input filter
  (2 mH, four 1.2 mF in series) rings at 205 Hz at the start (+37 V on each module) and is well damped by the
  slow voltage loop. Figures: [startup.pdf](claude_analysis/figures/startup.pdf),
  [waveforms.pdf](claude_analysis/figures/waveforms.pdf).

- **Input voltage sharing.** Identical modules share perfectly. A deviation between the two pairs decays in
  a few milliseconds (oscillating, envelope τ ≈ 2.7 ms); a deviation inside a pair decays with τ ≈ 64 ms.
  With one module different, steady state:

  | Case | The model: pairs on one rectifier (switched simulation) | One rectifier per module (averaged estimate) |
  |---|---|---|
  | equal modules | 1171 V on each module | 1171 V on each module |
  | module 1 turns +2 % | module 1: 1153 V, the others 1177 V | 1150 V / 1178 V |
  | module 2 leakage +5 % | module 2: 1351 V, module 3: 1005 V, modules 1 and 4: 1164 V | 1180 V / 1168 V |
  | module 2 leakage +20 % | module 2: 1450 V (+24 %), module 3: 959 V, modules 1 and 4: 1137 V; module 2 gives about 357 kW, 606 A rms, 900 A peak | 1208 V / 1158 V |

  A turns-ratio difference is handled well (the modules settle at equal `n·v`), a leakage difference is not.
  Figure: [voltage_sharing.pdf](claude_analysis/figures/voltage_sharing.pdf).

- **Start from 0 V and nominal catenary.** If the string capacitors really start at 0 V (their 0 V target
  has High priority in the model, but `DoDC` is on: which one Simscape takes I could not check), the input
  filter rings up to 2290 V per module (twice the nominal) with 1.8 kA in the string. With the emulator at
  3 kV the PI saturates (phase shift 1) and the output stays at 604 V. Figure: [zero_start_and_3kV.pdf](claude_analysis/figures/zero_start_and_3kV.pdf).

- **DC magnetizing current.** The start (phase shift changing from one half period to the next) leaves a DC
  part in the magnetizing current of −21 A (40 % of the 52 A AC peak; highest instantaneous value 68 A), which
  decays only through the winding and switch resistances, with a time constant of about 0.2 s. Nothing in the
  control corrects it (open point 5). Figure: [magnetizing_dc.pdf](claude_analysis/figures/magnetizing_dc.pdf).

## Open points found during the migration

1. **Turns ratio and leakage.** With n = 1 and the DAB-style leakage (26 µH, duty-cycle loss 156 V at full
   load) the modules need 1171 V each to give 750 V. That works only because the emulator is 1.5 × the
   catenary (4.7 kV, above the 3.6 kV highest permanent voltage of EN 50163). At the nominal 3 kV the output
   stops at 591 V (init) / 604 V (simulation). In a PSFB the leakage is not a power-transfer element: choose
   it for soft switching with a limited duty-cycle loss (for example 10 %), and choose the turns ratio from the
   lowest catenary voltage at which 750 V is needed (about n = 1.25 for 3 kV, about 1.85 for 2 kV, with a
   largest phase shift of 0.9). Then the emulator can be set to the catenary voltage.
2. **Load.** 0.5 Ω at 750 V is 1.125 MW = 4 × 281 kW, 112 % of the module rating. If 1 MW is meant,
   `rload` = 0.5625 Ω.
3. **Input voltage sharing inside a pair (the main point).** The pairs save two rectifiers and two
   inductors, but they make the two modules of a pair share the input voltage only through second-order
   effects (see "The architecture"): 64 ms instead of 2.7 ms, and a 5 % leakage difference puts 1351 V and
   1005 V on the two modules (+15 %), 20 % gives +24 % and 143 % of the rated power in one module. Options:
   (a) one rectifier (and one 250 µH inductor) per module, outputs in parallel on the DC side: the classic
   ISOP, the estimate gives +3 % for a 20 % leakage difference; (b) keep the pairs and add a sharing loop per
   pair that acts on the *relative phase* of the two bridges (a small time shift between them moves real
   power through the leakages, like a DAB); a duty correction does not work there, because it changes only
   the amplitude. The four-winding transformer of `trafo/` (both bridges of a pair on one core) ties the two
   bridges through leakage reactances in the same way, so I expect the same weak sharing (not simulated): it
   can save a core but it does not fix this point.
4. **Start-up.** The reference is a step from t = 0 and the start state depends on `DoDC` against the 0 V
   targets of the string capacitors. A real system needs a precharge of the string (resistor and bypass
   contactor) and a soft start of the reference; with a precharged string the model start is fine.
5. **"DC transformer current" compensation.** Its output is not connected (input 2 of the Sum in
   `modulator/d1_m` is open), so `d1` = 0.5 always and the block has no effect. Connected, it would not work:
   it measures the input current of the string (a DC current by nature, not the DC part of a transformer
   current) and it changes `d1` of both legs together, which gives no DC voltage on the transformer. A
   flux-balance loop would measure the DC part of each primary (or secondary) current (the `iac_2_1..4` Gotos
   exist) and act on the two legs in opposite directions; the usual alternatives are a DC-blocking capacitor
   in series with each primary or peak-current-mode control. With the 1 mH magnetizing inductance the start
   leaves 21 A DC (40 % of the AC peak), a saturation risk for a real core.
6. **Voltage loop.** The gains are the generic DAB values (crossover 4.8 Hz, closed-loop pole −14 rad/s,
   95 % in 178 ms). The proportional gain is limited by the output-filter resonance (411 Hz): gain margin
   9 dB already with kp = 0.5. A faster loop needs more integral gain, for example kp 0.6, ki 100: crossover
   33 Hz, phase margin 135°, gain margin 7 dB, about five times faster (init loop model). If the loop is made
   much faster, check the input filter (205 Hz, characteristic impedance 2.6 Ω) against the −19 Ω
   constant-power input of the converter (Middlebrook).
7. **Template leftovers.** The current PI is computed but not used (its reference is `idc2_nom` = 333 A, the
   current of one module, for a 1500 A output). The DC-current filter and PI use `ts_dab` (41.7 µs) but run
   at 125 µs: their real time constant is 3 times longer than the typed values. The `To Workspace` blocks of
   that subsystem allocate `Ns_dab` points. The scope of the PSFB tracking had the trace `uout_ref` from a
   missing Goto (commented). The four bridges use the same carrier: shifting pair 2+3 by a quarter period
   would interleave the two rectifier currents at 8 kHz and reduce the output and input ripple (optional).
8. **Battery charge capacity.** The old library function used a fixed 50 Ah; the new one takes the capacity
   as an input. I used the 1C assumption of the other migrated models (10.8 MW / 4500 V = 2400 Ah). In 1.75 s
   the state of charge moves by 0.005 % instead of 0.24 %: no visible effect.
9. **Legacy init.** `init_model_legacy.m` no longer runs with the library (`lithium_ion_battery_setup` now
   takes the charge capacity and the battery block reads `param.battery.nominal_battery_voltage_1` from its
   mask). It can be deleted after the merge.
