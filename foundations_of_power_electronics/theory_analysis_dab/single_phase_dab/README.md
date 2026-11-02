# single_phase_dab — 250 kW single-phase dual active bridge between two batteries

A bidirectional isolated DC/DC converter: a single-phase dual active bridge (DAB) between two lithium-ion
batteries, made for the study of the modulation and of the control. Side 1: `battery_1` → DC choke `LFi_dc1`
→ DC link `CFi_dc1` → full bridge 1. Side 2: full bridge 2 → `CFi_dc2` → `LFi_dc2` → `battery_2`. The two
bridges (2 kV SiC MOSFET modules) drive a 1:1 transformer with a series (DC-blocking) capacitor on each side.
Phase-shift modulation at 12 kHz with 2 µs dead time; one PI regulates the DC current of side 1 (the current
of `LFi_dc1`), with a feed-forward of the reference. The three-phase version of the same bench is
[`../three_phase_dab`](../three_phase_dab) (its sizing note `dab3_sizing.md` explains several effects that
appear here too).

| File | Content |
|---|---|
| `init_model.m` | initialization: every number in the struct `param`, design figures, then it opens the model, closes the scopes and checks the `param.*` references. Run it **from this folder**. |
| `single_phase_dab_private.m` | handles to the functions of `private/` (`prv.<name>`), so that the init also runs as a live script (the Live Editor does not see `./private`), as in `../three_phase_dab` |
| `private/` | the parameter functions of this model (table below) |
| `single_phase_dab.slx` | the model (batteries, DAB, control, logging) |
| `claude_analysis/` | design check: switched simulation of the same circuit in Python, results and figures (see "Simulation results") |
| `plotting_results.m`, `plotting_results_zvs.m`, `power_loss_calculus.m`, `spectrum.m` | old post-processing scripts, not changed (open point 10) |
| `init_model_legacy.m` | the old flat init, kept until the merge; it no longer runs with the library (open point 11) |

## The model

**Power stage** (`dab_modA/dcdc_with_galvanic_isolation`):
- `battery_1`, `battery_2`: library `lithium_ion_battery`, 1070 V nominal, 296 cells, 1112.9 V open circuit at
  soc 0.85; as in the legacy init `R0` and `R1` are halved (57.2 mΩ each) and `C1` is divided by 50 (10 mF).
  Terminal capacitor 1 mF.
- DC chokes `LFi_dc1`, `LFi_dc2` 250 µH + 1.96 mΩ; DC links `CFi_dc1`, `CFi_dc2` 3.6 mF + 1 mΩ.
- `full-bridge 1`, `full-bridge 2`: library `full-bridge_mosfet_based_with_thermal_model_zvs` with the data of
  `danfoss_SKM1700MB20R4S2I4` (1.04 mΩ, body diode 4 V; ideal switches with a thermal network and a
  switching-loss output) and an RC across each switch (`Csnubber_zvs` 1.88 nF + `Rsnubber_zvs` 8.86 kΩ). The
  ideal-switch bridges are in the model but commented.
- `single_phase_dab_transformer`: `Cs1` = `Cs2` = 230 µF in series with the windings (115 µF in total,
  resonance with the series inductance at fPWM/5 = 2.4 kHz), library `single_phase_transformer` with
  n = 1 (6:6), `Ld1` = `Ld2` = 19.1 µH (Ls = 38.2 µH), `Lm` = 1 mH, `Rfe` = 1 kΩ, 1 mΩ per winding.

**Control** (`dab_modA/dab_ctrl`):
- `dab_single_phase_pwm_modulator`: triangle carrier at 12 kHz, both legs at 50 % duty, 2 µs dead time,
  bridge 2 delayed by `u · phi_max` (`phi_max` = 90°); a control trigger at the peak and at the valley of the
  carrier (ts = 41.7 µs).
- `ui_ctrl/dab_ctrl/current control`: library `pi_ctrl` (kp 0.5, ki 18, limits ±1) on `iL(LFi_dc1)/Idc_FS`
  (Idc_FS = 250 A, quantized 1/2^11, one sample of delay) plus the feed-forward `k_ff · i_ref`; output u.
- Bench: the reference is 0.1 pu from t = 0 and 1 pu from 0.15 s (`Step6`), through a 20 ms discrete
  low-pass filter. The output voltage loop (`Vout Ctrl`, kp 0.5, ki 35) is in the model but not selected by
  the Variant Source.

## How to run

1. Clone `pwr-control/library` next to this repository (or set `PWR_CONTROL_LIBRARY`): `init_model.m` calls
   `prv.library_setup()`, which puts the library on the path and clones it when it is missing. The library
   gives the Simulink/Simscape blocks and the C sources of the configuration, no parameter.
2. From this folder run `init_model.m` (Command Window or Live Editor). It fills `param`, prints the timing,
   the hardware, the devices, the batteries and the design figures, opens `single_phase_dab.slx`, closes
   the scopes and checks that every `param.*` name read by the model exists.
3. Run the simulation (`param.sim.simlength` = 2 s, Simscape local solver Backward Euler at
   `param.glb_time.tc` = 0.625 µs, also `MaxStep` of the solver).

## Parameter structure

| Group | Content |
|---|---|
| `param.sim` | `simlength` 2 s, `tc` 625 ns, `nonlinear_iterations` 3 (Solver Configuration), `check_param_refs` |
| `param.time`, `param.glb_time` | `fpwm_dab` 12 kHz, `trgo_dab` 1 (double update), `dead_time_dab` 2 µs, `tc`, `t_measure` (= `simlength`), `tc_decimation`; `glb_time = timing_param(param.time)` gives the names the blocks read: `fPWM_DAB`, `tPWM_DAB`, `TRGO_DAB_double_update`, `ts_dab` (41.7 µs), `dead_time_DAB`, `tc`, `decimation_tc`, `t_measure`, `Nc`, `Ns_dab` |
| `param.dab` | `pwr_nom` 250 kW, `application_voltage` 690 (legacy selector of the data set), `fres` = fPWM/5, `hw_in` (the numbers of the data set `DAB_1200V`, were inside the library) |
| `param.dab.hw` | `dab_hw_param('DAB_1200V', ...)`: `Ls` = V²/(4π·f·P) = 38.2 µH, `Cs`, `Ld1`, `Ld2`, `Cs1`, `Cs2`, `Lm`, `Rfe`, `Rs1`, `Rs2`, `n1`, `n2`, core data; `Cdc_dc1/2`, `RCdc_dc1/2`, `Ldc_dc1/2`, `RLdc_dc1/2`; bases `udc1_nom` 1200 V, `idc1_nom` 250 A, `Idc_FS`, `Vdc_FS` |
| `param.dab.mod` | `phi_max` π/2 (was typed), `k_ff` 1 (was the block default) |
| `param.dab.ctrl` | `kp_idc` 0.5, `ki_idc` 18 (current PI); `kp_udc` 0.5, `ki_udc` 35 (voltage PI, not selected) |
| `param.dab.bench` | `i_ref0` 0.1, `t_step` 0.15 s, `i_ref1` 1, `tau_ref` 20 ms (were typed in `Step6` and in the filter) |
| `param.dab.ic` | `vc_dc1`, `vc_dc2`: initial voltage of the DC links = open-circuit voltage of the batteries (new) |
| `param.dab.mosfet`, `param.dab.ideal_switch`, `param.dev` | device data sets (`device_mosfet_param`, `device_ideal_switch_param`) |
| `param.dab.design` | design figures (informative, not read by the model) |
| `param.battery` | `nominal_battery_voltage_1/2` 1070 V, `nominal_battery_power` 250 kW, `initial_battery_soc` 0.85, `charge_capacity_1/2` 233.6 Ah (assumption, open point 9), `pack_1`, `pack_2` (`battery_param`, with the legacy changes), `ocv_1`, `ocv_2` (initial terminal voltage of the battery blocks) |
| `param.meas` | `adc_quantization` 1/2^11, `adc12_quantization` (read by the model), `adc16_quantization` |
| `param.thermal` | liquid-cooled plate data and `heatsink` (fields of `liquid_cooled_plate_2kw_setup`) |

### Functions of `private/`

| Function | Role |
|---|---|
| `timing_param(t)` | the clock domain of the bridges and of the control (DAB part of `timing_setup`) |
| `dab_hw_param(name, P, fpwm, fres, hw_in)` | transformer, series capacitors, DC links, chokes, bases (formulas of `hw_dab_single_phase_setup`, numbers from the init) |
| `device_mosfet_param(device, fpwm, udc)` | MOSFET data set (the Danfoss module is embedded, other data sets through the library) with the formulas of the ZVS RC |
| `device_ideal_switch_param(device, fpwm, udc)` | ideal-switch data set (bridges commented in the model) |
| `battery_param(...)` | lithium-ion pack (formulas of `lithium_ion_battery_setup`) |
| `dab_ideal_op(V1, V2r, fs, L, P)` | lossless DAB with phase shift: phase, largest power, rms and peak current, switching currents |
| `dab_harmonic_op(p, phi, nh, npts)` | steady state of the real tank (series capacitors, resistances, `Lm`, `Rfe`) by harmonic superposition |
| `dab_harmonic_phi(p, P, nh, npts)` | phase shift for a given power in the harmonic model |
| `check_model_param_refs(model, param)` | lists the `param.*` names read by the model and reports the missing ones |
| `library_setup(url)` | puts the library on the path, clones it when missing (copy of `../three_phase_dab/private`) |

## Old names and new names

| Old | New |
|---|---|
| `glb_time.*` | `param.glb_time.*` (same field names) |
| `hwdata.dab.*` | `param.dab.hw.*` |
| `dab_ctrl.*` | `param.dab.ctrl.*` |
| `mosfet.dab.*`, `ideal_switch.dab.*` | `param.dab.mosfet.*`, `param.dab.ideal_switch.*` |
| `lithium_ion_battery_1.*`, `lithium_ion_battery_2.*` | `param.battery.pack_1.*`, `param.battery.pack_2.*` |
| `heatsink.*` | `param.thermal.heatsink.*` |
| `adc12_quantization` | `param.meas.adc12_quantization` |
| `simlength` (`StopTime`) | `param.sim.simlength` |
| `tc` (`FixedStep`), `glb_time.tc` (`MaxStep`) | `param.glb_time.tc` |
| typed 3 (`MaxNonlinIter` of the Solver Configuration) | `param.sim.nonlinear_iterations` |
| typed `pi/2` (`max_phase_displacement` of the modulator) | `param.dab.mod.phi_max` |
| block default 1 (`Gain` of the feed-forward) | `param.dab.mod.k_ff` |
| typed 0.15, 0.1 and default 1 (`Step6`), typed 0.02 (filter) | `param.dab.bench.t_step`, `i_ref0`, `i_ref1`, `tau_ref` |
| not set (`battery_voltage_init_m`, mask default) | `param.battery.ocv_1`, `param.battery.ocv_2` |
| 0 V, not specified (initial voltage of `CFi_dc1`, `CFi_dc2`) | `param.dab.ic.vc_dc1`, `param.dab.ic.vc_dc2` |

## What changed in the model (2 November 2026)

The model was edited as XML (no MATLAB in the cloud): same file, same `ModelUUID`.
- 247 parameter changes in the block dialogs and in the configuration; the model reads 105 distinct
  `param.*` names, all defined by the init.
- Values that were typed in blocks (or block defaults) now read `param`: the table above.
- **New behaviour (one change on purpose):** the DC links start at the open-circuit voltage of their battery
  (`vc_specify` on, priority High), and so do the battery blocks (`battery_voltage_init_m`), as in
  `../three_phase_dab`. The legacy model started the DC links at 0 V: see result 6. Set `param.dab.ic.*` to 0
  for the legacy start.
- `global timing/From9` (tag `local_time_corr_llc_mod1`, no `Goto` in this model, a leftover of the CLLC
  template) is commented: a `From` without its `Goto` stops the diagram update. Its `Unit Delay` reads 0 as
  before.
- Configuration: `StopTime` → `param.sim.simlength` (both configuration sets), `FixedStep` (`tc`, a name the
  legacy init did not define) and `MaxStep` → `param.glb_time.tc`.

**Checks done.** In Octave the lines of the legacy init that build the names read by the model (with the
library functions they call) and the new init were run side by side, and every expression re-pointed in the
model was evaluated with both: 99 same; different on purpose: the battery charge `Q` (50 Ah fixed in the old
library function, 233.6 Ah now, open point 9) and the initial DC-link voltages (0 → 1112.9 V); new or not
defined before: `ocv_1`, `ocv_2` (mask default before), `k_ff` and `i_ref1` (block defaults 1),
`FixedStep`. All 105 `param.*` names of the model exist. `init_model.m` runs to the end in Octave (with stubs
for the Simulink calls). The model itself was not run in Simulink here: please run it once in MATLAB.

## Design figures (printed by `init_model.m`)

- Design point 1200 V / 1200 V, 250 kW, lossless: φ = 35.7°, largest power 393 kW at 90°, 242 A rms, 260 A
  peak, switching currents 260 A on both bridges.
- Bench at 1 pu (250 A out of battery_1): the DC links are at 1084 V / 1141 V (d = 1.053, the battery
  resistances), 271 kW; the harmonic model of the real tank gives φ = 47.5°, so u = 0.53, while the
  feed-forward gives `k_ff · 1` = 1.00 (open point 1); 303 A rms, 344 A peak, switching currents 311 / 360 A,
  magnetizing current 17 A peak.

## Simulation results (`claude_analysis/`)

`dab_sim.py` is a switched simulation of the same circuit with the values of `init_model.m` (Python +
numba): backward Euler at tc = 0.625 µs like the Simscape local solver, ideal switches and piecewise-linear
diodes with the RC of the bridges, the transformer T model with the series capacitors, the battery cell
model, the modulator (carrier computed at tc, dead time, double update) and the current PI with the
feed-forward, the 1/2^11 quantization and the sample of delay. It is not the Simscape model (I cannot run
MATLAB here) but it has the same structure and numbers. Not simulated: the thermal networks and the
switching-loss outputs of the MOSFET blocks (no effect on the circuit). Run `python3 dab_sim.py <case> ...`
(cases at the end of the file), `dab_sweep.py` (static characteristic), `dab_loop.py` (small-signal loop
analysis), `dab_plots.py` (figures, PDF in `figures/`) and `dab_report.py` (all numbers in
`design_check_single_phase_dab_output.txt`).

1. **Steady state at 1 pu (the model as it is).** u = 0.517 (46.5°; the init predicts 47.5°), DC links
   1084 V / 1139 V, primary current 294 A rms / 361 A peak, switching currents 270-327 A on bridge 1 and
   319-376 A on bridge 2, all with the current in the ZVS direction; magnetizing current 22.5 A peak (17 A
   of AC peak plus a slow oscillation of its DC part, open point 8), 67 V peak-to-peak on `Cs1`. The init and the simulation agree
   within a few percent. Figure: [waveforms.pdf](claude_analysis/figures/waveforms.pdf) (right).

2. **Bench step and feed-forward.** With `k_ff` = 1 the feed-forward asks about twice the phase shift that
   is needed (u = 1 against 0.52), the current overshoots and the slow integral action needs more than
   200 ms to remove the error. `k_ff` = 0.5 (= φ at 1 pu / `phi_max`, the same choice as in
   `../three_phase_dab`) gives no overshoot; the current then follows the filtered reference (the reference
   itself is in the 2 % band from about 226 ms). Figure: [bench.pdf](claude_analysis/figures/bench.pdf).

   | Case | Peak of iL1 after the step | 2 % band from | 1 % band from |
   |---|---|---|---|
   | `k_ff` = 1 (model now) | 1.14 pu (285 A) at 205 ms | 388 ms | 412 ms |
   | `k_ff` = 0.5 | 1.003 pu | 221 ms | 240 ms |
   | `k_ff` = 0 | no overshoot | 360 ms | 426 ms |
   | `k_ff` = 0.5, kp 0.05, ki 40 | 1.010 pu | 236 ms | 599 ms (257 ms with tc = 0.208 µs, see 4) |

3. **Dead zone at light load.** With the 2 µs dead time, bridge 1 loses its commutation at light load:
   the current at its edge is small (8-52 A at 0.1 pu), it falls to zero in the diodes before the dead time
   ends, the bridge floats and the incoming switches turn on hard at the end of the dead time (all 12 edges
   of bridge 1 in the 0.1 pu window, none of bridge 2, none at 1 pu). So the real edge comes up to 2 µs
   (8.6° at 12 kHz) late, and the phase shift is smaller than commanded. With a fixed u the DC current is
   stuck at about 9.5 A from u = 0.03 to 0.08, jumps to 21 A at u = 0.10 and 73 A at u = 0.15 (slope
   4.1 pu/pu against 2.24 pu/pu of the harmonic model), and reaches the harmonic curve only at u ≈ 0.2. A
   smaller solver step does not change it (0.208 µs: same curve); a 0.5 µs dead time removes it from
   u = 0.04 (25.2 A against 25.0 A of the harmonic model). The bridge with the lower DC voltage (bridge 1,
   d = 1.05 at full current) is the one that loses the current first, as described in `dab3_sizing.md`.
   Figures: [static_characteristic.pdf](claude_analysis/figures/static_characteristic.pdf),
   [waveforms.pdf](claude_analysis/figures/waveforms.pdf) (left: the spikes of `v_ab` of bridge 1).

   | u (phase) | Harmonic model | Model: tc 0.625 µs, dead time 2 µs | tc 0.208 µs, 2 µs | tc 69 ns, 0.5 µs |
   |---|---|---|---|---|
   | 0.04 (3.6°) | 25.0 A | 9.5 A | 9.4 A | 25.2 A |
   | 0.10 (9.0°) | 59.8 A | 21.0 A | 20.9 A | 60.3 A |
   | 0.15 (13.5°) | 87.6 A | 73.0 A | 72.4 A | 88.1 A |
   | 0.50 (45°) | 240.8 A | 245.8 A | 242.8 A | 241.7 A |

4. **Current loop.** The DC choke and the DC link (250 µH, 3.6 mF) resonate at 168 Hz; with the battery
   impedance the DC network has a pole pair at 182 Hz with ζ = 0.18, which is in the loop. The PI of the model
   (kp 0.5) puts the crossover at this resonance. Discrete loop gain (PI, sample of delay, ZOH, modulator
   delay, battery networks; `dab_loop.py`), broken at the controller output:

   | Operating point | Plant d(iL1)/du | PI of the model (kp 0.5, ki 18): crossover, PM, GM | kp 0.05, ki 40: crossover, PM, GM |
   |---|---|---|---|
   | 1 pu (u 0.518) | 1.25 pu/pu | 217 Hz, 28°, 11 dB | 8 Hz, 92°, 18.5 dB |
   | 0.1 pu, harmonic gain | 2.24 pu/pu | 251 Hz, 15°, 7.9 dB | 15 Hz, 94°, 14.8 dB |
   | 0.1 pu, gain of the switched model (dead zone) | 4.1 pu/pu | 302 Hz, 4°, 2.6 dB | 27 Hz, 97°, 9.6 dB |

   (At 1 pu the PI of the model crosses 0 dB also at 4.7 Hz; the resonance lifts the gain above 0 dB again
   up to 217 Hz.) The linear model matches the switched simulation for small reference steps (full load
   with `k_ff` = 1: first peak 2.13 against 2.01 of the linear model; light load: 3.49 against 3.78;
   light load with `k_ff` = 0: 1.38 against 1.29), figure
   [steps_linear_vs_switched.pdf](claude_analysis/figures/steps_linear_vs_switched.pdf). At light load the
   PI of the model rings at 300-420 Hz for tens of milliseconds and keeps a small oscillation, and its
   integral action is slow (time constant 40-70 ms).

   kp = 0.05, ki = 40 puts the crossover below the resonance (the method of `../three_phase_dab`, which uses
   kp 0.01, ki 60 with its 450 Hz resonance). In this model, at tc = 0.625 µs, it shows a larger ripple at
   light load (8 A peak-to-peak at 0.15 pu, 280 Hz, against 4 A with the PI of the model): the bridges switch
   only on the tc grid of the local solver, so φ moves in steps of 2.7° (0.9° on average over three periods,
   because T/tc = 133.3 is not an integer), the slow loop toggles between two steps and the DC network rings
   at each toggle. It is the resolution of the solver, not of the controller: at tc = 69 ns (0.3° steps) the ripple
   falls to 1.7 A peak-to-peak, below the 2.3 A of the PI of the model at the same step. A digital PWM with a
   few ns of resolution gives 0.01-0.04°. Figure:
   [light_load_ripple.pdf](claude_analysis/figures/light_load_ripple.pdf), loop gains:
   [loop_gain.pdf](claude_analysis/figures/loop_gain.pdf).

5. **Numerical loss of the solver.** Backward Euler dissipates about tc/(2L) · v_L² in every inductance
   (explained in `dab3_sizing.md`); here the leakage inductances see 2.2 kV steps. P_dc1 − P_dc2 (bridges +
   transformer) with a fixed u:

   | u | Harmonic model (windings, Rds_on, Rfe) | tc 0.625 µs | tc 0.208 µs | tc 69 ns (0.5 µs dead time) |
   |---|---|---|---|---|
   | 0.5 (about 1 pu) | 1.4 kW | 12.9 kW | 5.6 kW | 2.95 kW |
   | 1.0 | 2.2 kW | 25.2 kW | 10.4 kW | 5.1 kW |

   About 11 kW of the 12.9 kW at 0.625 µs are numerical (the rest: body diodes in the dead time, the RC of
   the bridges, the off-state conductances). The waveforms are right (result 1), but an efficiency computed
   from the battery or DC-link powers of the model is about 4 % too low. The loss outputs of the MOSFET
   blocks do not depend on it.

6. **Start with empty DC links (legacy init).** The batteries (1113 V) see the 0 V DC links through the
   250 µH chokes: 3.3 kA in `LFi_dc1` after 1.3 ms, 1.6 kV on DC link 1 and 1.79 kV on DC link 2 (2 kV
   devices), 1.1 kA in the transformer, ringing for about 20 ms. The new init starts the DC links at the
   battery voltage (a real system needs a pre-charge). Figure:
   [legacy_start.pdf](claude_analysis/figures/legacy_start.pdf).

## Open points found during the migration

1. **Feed-forward gain.** `k_ff` = 1 doubles the phase shift that is needed (14 % overshoot, 260 ms to settle
   at 1 %). Suggested: `param.dab.mod.k_ff = 0.5` (φ_nom / φ_max, as in `../three_phase_dab`): no overshoot,
   1 % band 90 ms after the step. Kept at 1 in the init (the legacy value) until you decide.
2. **Current-loop gains.** The PI of the model has its crossover at the 182 Hz resonance of the DC network,
   with small margins (28° / 11 dB at full load, 4-15° / 2.6-7.9 dB at light load). A crossover below the
   resonance (for example kp 0.05, ki 40: margins above 90° / 9.6 dB everywhere, faster integral action) is
   the safer choice for the converter. In this model it needs a finer phase resolution to look good
   (result 4), so it goes together with point 4. Alternatives that keep a fast loop: damp the DC network
   (series R-C across `CFi_dc1`), or use the current of the bridge instead of the choke current.
3. **Dead time and light load.** 2 µs is long for SiC at 12 kHz (8.6° of phase). At light load bridge 1
   loses its commutation (hard turn-on) and the phase shift is smaller than commanded (result 3: the current
   is below the harmonic value up to u ≈ 0.2, about 0.45 pu); this dead zone also makes the plant gain at
   light load about twice the harmonic one. Suggested: 0.5 µs, as in
   `../three_phase_dab` (check it against the real transition time of the module, 2·Coss·V/i: about 0.3 µs
   at 25 A with Coss 3.3 nF and 1.1 kV), or a dead-time compensation of the phase.
4. **Solver step.** tc = 0.625 µs (the legacy 4 kHz template value / 200) gives 133.3 steps per period: the
   edges move on a three-period pattern, φ moves in steps of 2.7°, and the numerical loss is about 11 kW at
   1 pu (result 5). It is fine for quick structural checks. For control and loss studies use an integer
   fraction of the period, for example T/800 = 104 ns (0.45°; `../three_phase_dab` uses 100 ns) or T/1200 =
   69 ns (0.3°); the run time grows as 1/tc. In the Python simulation the switch and diode states settle
   within 2 iterations per step (after the first steps), so `nonlinear_iterations` = 3 should be enough.
5. **Bases and battery voltage.** The DAB data set is for 1200 V / 250 A (Idc_FS = 250 A, 300 kW at 1200 V),
   the batteries are 1070 V nominal (the AFE value of the template): 1 pu of the bench is 271 kW at the
   1084 V DC link, 8 % above the 250 kW of `pwr_nom`, with d = 1.05. `../three_phase_dab` uses 1200 V
   batteries and 208 A (250 kW / 1200 V). Suggested: the same here, or a bench reference of 0.92 pu.
6. **The "ZVS" RC of the bridges.** `Csnubber_zvs` (1.88 nF) is in series with `Rsnubber_zvs` (8.86 kΩ):
   time constant 16.7 µs against a 2 µs dead time, so it does not shape the transitions; it is an RC damper
   that takes about 0.22 kW (8 · C · V² · f). The model has no Coss, the transitions are instantaneous.
7. **Switching-loss bookkeeping of the MOSFET blocks** (not simulated here, to check in MATLAB). In
   `../three_phase_dab` the bridge blocks add Eon and Eoff of the data set at every edge, as in hard
   switching (`dab3_sizing.md`). The bridges here are the `_zvs` variant (77 / 108 mJ at 1300 V, 1000 A,
   scaled with the voltage and current of the previous cycle); if they also count Eon at the ZVS turn-ons,
   at 1 pu they report about 2 kW of turn-on loss that does not exist (plus about 2.8 kW of turn-off loss,
   an upper bound for ZVS).
8. **Small items of the power stage.** The diode resistance of the MOSFET blocks (`Rdon_diode_m`) is wired to
   `Rds_on` (1.04 mΩ) instead of `Rdon_diode` (1.85 mΩ), as in the legacy model. `Rfe` = 1 kΩ takes 1.2 kW at
   any load (0.5 % of 250 kW; a placeholder). The series capacitors block the DC part of the magnetizing
   current, but `Cs1 + Cs2` (460 µF) with `Lm` form a 235 Hz resonance with very little damping (only the
   mΩ resistances and `Rfe`): the bench step leaves an oscillation of about ±3.5 A of the "DC" part of the
   magnetizing current, still there at the end of the run (about ±2 A with `k_ff` = 0.5).
9. **Battery charge capacity.** The old library function used a fixed 50 Ah; the new one takes the capacity
   as an input. I used the 1C assumption of the other migrated models (250 kW / 1070 V = 233.6 Ah). No
   visible effect in 2 s.
10. **Post-processing scripts.** `plotting_results.m`, `plotting_results_zvs.m`, `power_loss_calculus.m` and
    `spectrum.m` are template leftovers: they read the flat names (`Nc`, `tc`, `Ns_dab`, `ts_dab`: now
    `param.glb_time.*`), AFE and inverter signals that this model does not log (`ac_grid_*`,
    `inverter_device_data_modA_sim`), and `inverter_1_dab_devices_data_modA_sim` where the model logs
    `inverter_1_dab_devices_modA_sim`. Not changed; they need the new names before use.
11. **Legacy init.** `init_model_legacy.m` no longer runs with the library (`lithium_ion_battery_setup` now
    takes the charge capacity and the battery block reads `battery_voltage_init_m` from its mask). It can be
    deleted after the merge.
12. **Unused parts of the control.** The output voltage loop (`Vout Ctrl`) is not selected, and some steps,
    low-pass filters and a sum of the bench are not connected. Left as they are.
