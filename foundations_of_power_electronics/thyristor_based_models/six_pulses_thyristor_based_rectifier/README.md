# six_pulse_thyristor_based_rectifier — six-pulse thyristor rectifier with load current control

Six-pulse thyristor bridge (B6C) fed by a 400 V 50 Hz grid through a Dyn11 100 kVA transformer and three
65 µH inductors, with an LC filter on the DC side and a 3.4 Ω + 250 µH load. The control runs at 5 kHz: a
PLL written in C gives the phase of the voltage at the bridge, a C firing generator makes the six gate
pulses, and a PI on the load current sets the firing angle. Test: load current 20 A, step to 100 A at
0.5 s.

| File | Content |
|---|---|
| `init_model.m` | initialization: every number in the struct `param`, design figures, then it opens the model and checks the `param.*` references. Run it **from this folder** (the functions of `private/` are visible only to the files of this folder). |
| `private/` | the parameter functions of this model (table below) |
| `six_pulse_thyristor_based_rectifier.slx` | the model: grid emulator, transformer, bridge with its RC snubbers, DC filter, load and the whole control |
| `init_model_legacy.m` | the draft init, kept until the merge (see "What changed") |

## The model

**Power stage.**
- Grid emulator of the library (`three_phase_voltage_source_with_frt_emulator`, in `grid_line/Grid`): three
  ideal voltage sources at 400 V 50 Hz with the star point grounded; the amplitude rises from 0 with the
  ramp `1 - (1 + t/τ)·exp(-t/τ)`, τ = 1/(2π·50 Hz) (99 % at 20 ms). Fault data of test 25.4 of the library
  (75 % dip on two phases), disabled.
- Transformer `configurable_trafo` (library, one primary): Dyn11, 100 kVA, 400/400 V, r 0.005 + 0.005 pu,
  l 0.03 + 0.03 pu (short-circuit voltage 6 %), star point of the secondary grounded.
- Three 65 µH inductors between the transformer and the bridge (`param.rect.hw.Lac`).
- Bridge `BR2/ssc_thyristor_bridge`: six thyristors `thyristor_rr` of the library (Lauritzen-Ma charge model,
  data of the ABB 5STP 26N6500), each with an RC snubber of 10 Ω / 1 µF.
- DC filter: 1.2 mH split in two halves on DC+ and DC-, two 8.62 mF capacitors, one with a 0.25 Ω damping
  resistor (resonance 35 Hz with the filter inductor, 27.5 Hz with two phases of the commutation inductance
  in series).
- Load 3.4 Ω + 250 µH (340 V at 100 A).

**Control** (step `param.glb_time.ts_rect` = 200 µs).
- The three phase voltages at the bridge terminals (after the 65 µH inductors), divided by
  `param.meas.Vnom_ac_adc` (326.6 V), quantized at 2^-11, sampled and delayed by one step.
- One harmonic observer per phase (library `first_harmonic_tracker/harmonic_observer`, damping 0.05,
  observer poles at -ω and -4ω), one more unit delay.
- C-Caller PLL `dqpll_thyr` (library C code `dqpll_thyr.c`): gains kp 0.25, ki1 80, ki2 314.159 fixed in the C
  code; it is held in reset until `t_pll_reset` and restarts from ω = 0.
- C-Caller firing generator `sprcProcessSimulink` (C code `six_pulse_rect`): six ramps from the PLL phase,
  a pulse starts when the ramp passes the firing angle alpha, pulse width 1.9 rad (109°), all pulses blocked
  until `t_pulse_release`. The 0/12 V command goes straight to the gates (the firing circuit with the pulse
  transformer is commented out, the strobe generator is commented through).
- Load current PI `PI_i`: error in pu of `I_FS` (103.5 A), output cos(alpha) clamped to
  [`u_min`, `u_max`], integrator that follows the clamp, then `acos`, clamp of alpha to [`alpha_min`,
  `alpha_max`] and a unit delay. In the block the gain named "1/Ti" is kp and the gain named "kp" is ki·ts.
- The voltage PI `PI_v` (reference 200 V) is in the model but the Manual Switch selects `PI_i`.
- Measurements: a Simulink PLL (`pll_stageB`, to a scope only), IEC power meter at the control step, THD of
  the grid current, rms of the snubber currents.

## How to run

1. Add the `library` repository to the MATLAB path with subfolders (`init_model.m` does it for
   `C:\github\library`): the library provides the Simulink/Simscape blocks, the C-Caller sources and the
   Simscape components, no parameter.
2. From this folder run `init_model.m`: it fills `param`, prints the local functions of `private/`, the
   transformer, thyristor and control data and the design figures, opens the model, imports the C-Caller
   types (`spr_p_t`, `dqpll_thyr_output_t`), closes the scopes, checks that every `param.*` name read by the
   model exists (`check_model_param_refs`) and defines the bus object `PowerMeterIecBus` in the base
   workspace.
3. Run the simulation (`param.sim.simlength` = 1.2 s, `ode23tb` with `MaxStep` and the Simscape local solver
   step at `param.sim.tc` = 1 µs).

## Parameter structure

| Group | Content |
|---|---|
| `param.sim` | `simlength` 1.2 s, `tc` 1 µs (`FixedStep`, `MaxStep`, step of the Simscape local solver), `nonlinear_iterations` 3 (Solver Configuration of the grid emulator), `check_param_refs` |
| `param.time`, `param.glb_time` | `fs_rect` 5 kHz, `tc_factor` 200, `t_measure` 20 ms; `glb_time = timing_param(param.time, simlength)` gives `ts_rect` 200 µs, `tc`, `t_measure`, `Nc`, `Ns_rect`, `firing_step_deg_50Hz` (3.6°), `z_rect` |
| `param.grid` | `application_voltage` 400 V, `f_nom` 50 Hz, `omega_nom`; `frt = grid_fault_param('two_phase_D', 0.75, 0.915, 0.75, 0, 0)` (test 25.4, both enables at 0); `emu = grid_emulator_param(param.grid, param.grid.frt)`; `trafo = transformer_param(...)` (the 16 numbers of the `configurable_trafo` dialog, the check of the clock number, the vector group and derived values: leakage referred to the secondary `R_sc2`, `L_sc2`, `ucc`, nominal current, magnetizing current, iron losses) |
| `param.rect.hw` | `Lac` 65 µH, `LFu_dc` 1.2 mH, `RLFu_dc`, `f0_dc` 35 Hz, `CFu1_dc` = `CFu2_dc` 8.62 mF, `RCFu1_dc` 0.25 Ω, `RCFu2_dc` 2.5 mΩ |
| `param.rect.thy` | `thyristor_param('abb_5STP26N6500_A')`: `Vf`, `Ron`, `Goff`, `Vth`, `Vgt`, `Il`, `Ih`, `tau`, `Tm`, `Qth`, `tq` (the mask of `BR2/ssc_thyristor_bridge` reads them; they were typed there before) |
| `param.rect.snubber`, `param.rect.gate` | RC snubber 10 Ω / 1 µF; gate command level `u_on` 12 V |
| `param.rect.ctrl` | `rect_ctrl_param(ts, omega_nom, ctrl_in)`: `pi_i.kp`/`ki` (2, 35 1/s), `pi_v.kp`/`ki`, `u_min`/`u_max` (-1, 1), `alpha_min`/`alpha_max` (0, π), `iref_1`/`iref_2`/`t_iref_step` (20 A, 100 A, 0.5 s), `vref`, `t_pulse_release` and `t_pll_reset` (10 ms), `pll.*` (Simulink PLL), `fht.*` (harmonic observers), `c_code.*` (copies of the constants of the C code, informative) |
| `param.meas` | `margin_factor` 1.15, `Vout_dc_nom` 340 V, `Iout_dc_nom` 90 A, full scales `I_FS` 103.5 A and `V_FS` 391 V, `Vnom_ac_adc` 326.6 V, `rms.n1` |
| `param.load` | `R` 3.4 Ω (`0.9·340 V/90 A`), `L` 250 µH |
| `param.rect.design` | informative figures printed by the init: `Vdi0`, commutation inductance `Lc`, firing angle, overlap and extinction angle at 20 A and 100 A, largest safe firing angle, DC filter resonances, static gain of the current loop |

### Functions of `private/`

| Function | Role |
|---|---|
| `timing_param(t, simlength)` | the control clock domain (5 kHz) and the simulation step |
| `grid_emulator_param(g, fault)` | bases, references and the 50 Hz amplitude-ramp filter of the grid emulator |
| `grid_fault_param(kind, dip, length, start, en1, en2)` | fault data read by the emulator mask (local copy of `frt_settings` + `grid_fault_generator` + `asymmetric_error` of the library, reduced to the fault kinds used here) |
| `transformer_param(t)` | data of `configurable_trafo` and the checks of the vector group (clock number odd for a delta primary and a star secondary) |
| `thyristor_param(name)` | data sets of the thyristor (sets A and B of the ABB 5STP 26N6500) |
| `rect_ctrl_param(ts, omega_nom, c)` | every control number of the model |
| `check_model_param_refs(model, param)` | lists the `param.*` names read by the model and reports the missing ones |

## What changed (November 2026)

The draft init (`init_model_legacy.m`) is a copy of the shared template of the AFE and inverter models with the
B6C data added at the end. The model did not run with it: its grid emulator data were for 690 V
(`param.grid.emu` from `grid_three_phase_emulator` at 690 V, while the transformer and the rest of the model are
for 400 V), and the fault data were in `param.frt.data`, while the emulator mask reads `param.grid.frt.*`.

- New `init_model.m` with all the numbers and the local functions of `private/`; the library is used only for
  blocks, C sources and Simscape components.
- The model was re-pointed to `param.*` by editing its XML (no MATLAB here): 164 block parameters in the
  model and in its two configuration sets, 92 distinct names. The numbers typed in the blocks (PI gains and
  clamps, transformer dialog, thyristor and snubber mask, observer data, DC filter) now come from `param`.
- Transformer clock number of the secondary 0 → 11 (Dyn11, the vector group of the draft): the component
  rejects 0 with a delta primary and a star secondary. The PLL reads the voltages at the bridge, so the 30°
  shift does not change the firing.
- Grid emulator at 400 V (draft: 690 V).
- Power meter at the control step 200 µs with 6000 samples (draft: 125 µs and 14000 samples, from the AFE
  template).
- THD block at `tc`, the step of the model (draft: `param.glb_time.tc` = 0.25 µs from the template, while the
  model step was 0.5 µs).
- Fault data of test 25.4 kept with both enables at 0 (draft: `enable_frt_2 = 1`, which puts the dip during
  the 100 A test). Set the last argument of `grid_fault_param` to 1 to apply it.
- In his commit of 1 November 2026 (model saved in Simulink) Davide set `param.sim.tc` 0.5 → 1 µs (the step of the model, of the
  Simscape local solver and of the THD block) and the nonlinear iterations of the Solver Configuration 5 → 3.
- `init_model_legacy.m` stays in the folder until the merge; delete it after the check in MATLAB.

**Checks done.** The parameter part of the new init was run in Octave without the library on the path and
compared with the draft init: of the 191 distinct expressions of the model, 149 give the same value, 25
changed on purpose (20 of the power meter, 4 of the THD block, the clock number) and 17 were not defined by
the draft (the fault data of the emulator mask); this comparison was made with `tc` = 0.5 µs, before the change
to 1 µs. Every `param.*` name of the model resolves. The slx keeps the
order, compression and dates of its zip entries; all changed XML parts parse. The model was not simulated in
Simulink here (no MATLAB).

## Design figures

From the init (`param.rect.design`):

| Quantity | Value |
|---|---|
| bridge | `Vdi0` 540.2 V, `Lc` 370.6 µH per phase (transformer 305.6 µH + 65 µH), commutation drop 111.2 mΩ/A, tq 800 µs = 14.4° at 50 Hz |
| 20 A (68 V) | alpha 82.2°, overlap 0.5°, extinction angle 97.3°; the DC current is **discontinuous** (300 Hz ripple about ±50 A) |
| 100 A (340 V) | alpha 48.7°, overlap 3.1°, extinction angle 128.2°; continuous (ripple about ±38 A) |
| largest safe alpha at 100 A | 180° - overlap - tq = 162.5°; the clamp allows 180° |
| DC filter | resonance 35.0 Hz (filter only), 27.5 Hz (with 2·Lc), characteristic impedance 0.336 Ω |

## Open points found during the migration

The control was not changed; these are the points to look at. The numbers come from a design check outside
the repository (small-signal models of the PLL and of the current loop, and a switched simulation of the whole
model with a copy of the C code of the PLL and of the firing generator).

1. **Start-up.** The pulses are released at 10 ms, when the DC capacitors (17.2 mF) are empty and the PLL,
   which restarts from ω = 0 after its reset, is still far from lock (±2° only from 139 ms). The first
   firing angle is about 63°. Result: grid current peak about 780 A, bridge DC current about 900 A, vout up
   to 610 V and iout up to 179 A with a reference of 20 A, alpha at 180° for about 10 ms. Suggested: PLL reset
   until 20 ms, pulses released at 250 ms, integrator of `PI_i` held at zero until the release (small model
   change, for example error × (1 - block signal)), and the PI numbers of point 3: grid current peak 105 A,
   iout peak 24 A, vout peak 82 V. The late release alone is worse (the integrator reaches alpha = 0 before
   the release).
2. **Firing angle up to 180°.** The firing chain adds +3.6° to +7.4° to the commanded angle (point 5), and the
   safe limit at 100 A is 162.5°. Suggested: `u_min = -0.866`, `alpha_max = 150°` (the load is passive, the
   inverter range is not needed).
3. **Current PI at the stability limit.** Averaged model at 100 A: with kp 2, ki 35 the loop crosses at 56 Hz
   with about 0° phase margin and 0 dB gain margin. The switched simulation shows a 40-65 Hz ripple and a
   slow tail after the step (95 A after 147 ms). Suggested: kp 0.2, ki 25 1/s (crossover 6.4 Hz, phase margin
   96°, gain margin 13.3 dB): 95 A after 87 ms, at most 102.5 A.
4. **PLL.** kp 0.25, ki1 80 (constants of the C code) give a 27 Hz crossover with 28° phase margin, near the
   DC filter resonance (25-27.5 Hz); the PLL reads the notched bridge voltage and the observer reduces the
   5th and 7th harmonic by only 2 and 4 dB. With the slower PI of point 3 this gives 15 V at 25 Hz in vout.
   Suggested: kp 0.15, ki1 5 and restart from ω = 1 pu (3.4 V), or the grid-side voltage as PLL input
   (2.8 V, plus 30° for Dyn11). The C code is shared with `sptr`, the twelve-pulse models and the dual
   converter: better to pass the gains as inputs than to change the constants.
5. **Firing delay.** The real firing angle is the commanded one +5.6° on average (+3.6° to +7.4°): two unit
   delays and the observer before the PLL (3.5° in steady state) and the 200 µs grid of the pulse start
   (3.6°). The PI removes the mean, the limits must include it.
6. **Discontinuous conduction at 20 A.** Lower and non-linear plant gain; the loop must be checked at both
   operating points.
7. **Thyristor model `thyristor_rr` (library).** In the copy of the component used by the check, (a) a
   thyristor whose current falls below Ih in the same step as a mode change (its own turn-on at a very small
   current, or the turn-on of another thyristor) never sees the turn-off edge and stays on with a negative
   current: a line-to-line short of kA (with the model as it is the 100 A test fails); (b) after a turn-off
   the forward voltage passes Vth = 50 V within a few µs and the thyristor fires again without a gate pulse.
   Both depend on how Simscape evaluates edge events after a mode change: please check in Simulink (thyristor
   currents negative while on, kA spikes in the grid current).
8. **Smaller points.** Thyristor data of a 6.5 kV / 2880 A device (tq 800 µs) for a 400 V / 100 A bridge;
   snubbers 1 µF (about 70 W at 100 A); THD block at the model step with a 200000-sample window (heavy); the
   trapezoidal local solver can ring at the switching events (Backward Euler is more robust); misleading
   gain names in `PI_i` and `PI_v`.
