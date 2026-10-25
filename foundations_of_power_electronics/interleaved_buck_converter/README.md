# Three-leg interleaved buck converter

750 V battery to a 400 V DC bus, 250 kW (625 A, 208 A per leg), three buck legs with interleaved
carriers at 4 kHz (double update, control at 8 kHz; the output sees 12 kHz), 1 mH per leg, 3.6 mF of
output capacitance, resistive load. Two models of the same converter, driven by the same `init_model.m`
(`param.buck`, `param.battery`, `param.load`, `param.glb_time`):

- `interleaved_buck.slx` — discrete blocks: library modulator (three carriers, TRGO of each one),
  IGBT bridge with the loss and thermal model used as three buck legs, DC-link block, three inductors,
  three output capacitors, battery; control in Simulink blocks (`dcdc_ctrl`).
- `interleaved_buck_compact.slx` — the compact block `interleaved_buck_3legs_mod1` of the library
  (power stage as the Simscape language component `buck_ssc.interleaved_buck_3legs`, control and
  interleaved PWM as one MATLAB Function) with the same battery and load. Not tracked: built by
  `build_interleaved_buck_compact.m` after `init_model`, once the library block exists:

  ```matlab
  cd library/foundation/pwr_electronics
  ssc_build buck_ssc
  cd ../../masked_models/interleaved_buck_3legs
  update_interleaved_buck_3legs
  cd <this folder>
  init_model; build_interleaved_buck_compact; sim('interleaved_buck_compact')
  ```

## Control

Cascade of an output voltage PI (reference ramped at start, output the total current reference limited
at `i_lim`) and one PI per leg on the leg current with the reference `i_ref/3`: the legs share the
current by construction (the per-leg factors `kL`, `kRL` of the inductors give mismatched legs for the
sharing studies). Timing of the samples and of the duties:

- the control runs at the extremes of carrier 1 (double update);
- the current of each leg is sampled and held at the extremes of **its own** carrier, where the leg
  current is at its average value (the ON pulse of the upper switch is centred on the carrier bottom);
  the control reads the held values at its step, one step behind (acquisition delay);
- the duty of leg 1 is loaded at the control step, the duties of legs 2 and 3 at the next extreme of
  their own carriers (shadow registers): the sample age (up to 2/3 ts) and the shadow delay (the
  complement to one ts) add up to one control period, so legs 2 and 3 see 2.5 ts of loop delay against
  1.5 ts of leg 1.

Gains from the crossover targets in `private/buck_ctrl_param.m`: 300 Hz for the current loops, 60 Hz
for the voltage loop at nominal load (the plant of the voltage loop is the output capacitor with the
load, corner 69 Hz, behind the closed common-mode current loop, whose gain is only about 0.7 there
because the capacitor reacts to a common duty; about 100 Hz without load). Linear check in
`claude_analysis/interleaved_buck_loop_analysis.py` (python3, numpy/scipy/matplotlib; discrete model
at ts with the delays above, margins of both loops at nominal and light load, reference ramp and load
step): current loops 313 Hz with 59 deg of phase margin on leg 1, 42-44 deg and 7.5 dB of gain margin
on legs 2-3; voltage loop 61 Hz / 121 deg at nominal load, 114 Hz / 94 deg at 5 % load; 5 % dip on a
+0.2 pu load step recovered within 1 % in 40 ms. Figures `interleaved_buck_loops_bode.png`,
`interleaved_buck_step_response.png`.

## Notes on the first version (2026-10-25)

The model committed as "added interleaved buck converter" was a copy of the grid-forming AFE with the
buck legs wired in; the review found and fixed:

- carrier phases π/3, 2π/3 instead of 2π/3, 4π/3 (no cancellation of the ripple at the fundamental);
- the outer loop closed on the **load current** with a constant reference 1 instead of the output
  voltage (the acquired `u_out` was not connected);
- one PI on the sum of the three currents with the same duty for all legs (no sharing, and the sum
  sampled at the extremes of carrier 1, which is exact for the sum but not for the single legs: the
  samples of legs 2 and 3 would be off by ±0.36 of the ripple at D = 0.53);
- a three-limb coupled reactor as leg inductor: the three leg currents are DC and in phase, the core
  sees only the zero-sequence inductance (the model worked because the leakage was set equal to the
  main inductance);
- init and parameters of the AFE (LCL filter, transformer, resonant PI, 400 V grid data), PI gains not
  designed for the buck plant, an unused inverter clock domain, a scalar duty filter on a vector.
