# Three-leg interleaved buck converter

660 V battery to a 200 V DC bus, 750 A (150 kW, 250 A per leg, duty 0.303), three buck legs with
interleaved carriers at 3 kHz (double update, control at 6 kHz; the output sees 9 kHz), Infineon
FF450R17ME4 half bridges (1700 V / 450 A), 0.8 mH per leg, 7.5 mF of output capacitance, resistive
load. Two models of the same converter, driven by the same `init_model.m` (`param.buck`,
`param.battery`, `param.load`, `param.glb_time`):

- `interleaved_buck.slx` — discrete blocks: library modulator (three carriers, TRGO of each one),
  IGBT bridge with the loss and thermal model used as three buck legs, DC-link block, three inductors,
  three output capacitors, battery; control in Simulink blocks (`dcdc_ctrl`).
- `interleaved_buck_cmp.slx` — the compact block `interleaved_buck_3legs_mod1` of the library
  (power stage as the Simscape language component `buck_ssc.interleaved_buck_3legs`, control and
  interleaved PWM as one MATLAB Function) with the same battery, global timing and load.
  `build_interleaved_buck_compact.m` generates the minimal `interleaved_buck_compact.slx` (not
  tracked) around the same block, after `init_model`, once the library block exists:

  ```matlab
  cd library/foundation/pwr_electronics
  ssc_build buck_ssc
  cd ../../masked_models/interleaved_buck_3legs
  update_interleaved_buck_3legs
  cd <this folder>
  init_model; build_interleaved_buck_compact; sim('interleaved_buck_compact')
  ```

The previous version (PR #11, 2026-10-26 morning) was 750 V to 400 V, 250 kW, 4 kHz, 1 mH, 3.6 mF,
FF1800R12IE5; the operating point, the module and the sizing below are of 2026-10-26 afternoon.

## Sizing (2026-10-26)

Specifications: Udc = 660 V, Uout = 200 V, Iout = 750 A, output voltage ripple 0.2 % peak-peak,
fsw = 3 kHz per leg (the output sees 9 kHz), IGBT module FF450R17ME4. The design inputs are
`spec.*` in `init_model.m`, which prints the minima and the figures of the chosen values.

- **Leg inductor.** Triangular ripple of one leg `dI_leg = Udc D (1 - D)/(L fsw)`; with the
  criterion `dI_leg <= 25 %` of the leg current (250 A) at the nominal duty, `L_min = 0.743 mH`,
  chosen `L = 0.8 mH` (58 A peak-peak, 23 %). One separate inductor per leg.
- **Output capacitor, ripple.** The three ripples add with the 120° shift to a triangular current at
  `n fsw`: `dI_tot(D) = n Udc (D - k/n)((k+1)/n - D)/(L fsw)`, `k = floor(nD)`, zero at D = 1/3
  (7.6 A at the nominal duty), worst case over the duty `dI_tot_max = Udc/(4 n L fsw) = 22.9 A`
  (nD - k = 1/2, e.g. with the battery at a different voltage). The capacitor takes it:
  `dU = dI_tot/(8 n fsw C)`, so `C_ripple = dI_tot_max/(8 n fsw dU_max) = 0.80 mF` for 0.4 V
  (0.26 mF at the nominal duty only).
- **Output capacitor, control.** The LC resonance of the common mode, `f0 = 1/(2π sqrt(L/n C))`,
  must stay inside the current loops, `f0 <= f_c_i/2`, so that the loops damp it at any load:
  `C_ctrl = n (2/(2π f_c_i))²/L = 7.5 mF` with `f_c_i = 225 Hz`. This criterion dominates:
  with 0.8 mF the resonance is at 345 Hz, above the loops, and the common-mode current loop crosses
  0 dB a second time on the undamped resonance at light load (14° of phase margin at no load on
  legs 2-3); the voltage plant gain would also change 12× between no load and nominal load (load
  corner 746 Hz against the 60 Hz loop), and the cascade without a load-current feed-forward is
  either unstable at no load or 1 Hz slow at nominal load. Chosen `Cout = 7.5 mF` (three parts of
  2.5 mF, ESR 3 mΩ each): f0 = 112 Hz, zeta 0.35 at full load, load corner 80 Hz, ripple 14 mV at
  the nominal duty and 42 mV (0.02 %) worst case. A capacitor at the ripple minimum needs a
  different control (load-current feed-forward, current loops above the resonance), not available
  at fs = 6 kHz.
- **DC link.** `Cdc = 7.2 mF` kept: the input current is a pulse train at 9 kHz (the ON pulses of
  the legs do not overlap at D < 1/3), 72 A rms in the capacitor and
  `dU_dc = D'(1 - D') i_leg/(n fsw Cdc) = 0.3 V`, `D' = nD - k`.
- **Module.** FF450R17ME4 at 250 A per leg (300 A at `i_lim` = 1.2 pu), 660 V, 3 kHz, data set
  `infineon_FF450R17ME4` of the library (`setup_files/semiconductor_devices_data`, values at
  125 °C, read by `private/device_igbt_param` through `device_data_load`). Loss estimate of the
  init with the same linear loss model as the bridge: IGBT 133 W conduction + 354 W switching,
  diode 257 W conduction + 134 W recovery, 878 W per leg, Tj 93 °C (IGBT) and 105 °C (diode)
  with water at 40 °C, efficiency 97.2 % with the inductors (6 mΩ per leg). The linear scaling of
  the energies underestimates Erec at 250 A (the curve gives 85 mJ at 900 V against 61 mJ scaled).
- **Control.** `f_c_i = 225 Hz` (fs/26.7 at fs = 6 kHz, the ratio of the 4 kHz version: same
  delay margins), `f_c_u = 60 Hz`; `private/buck_ctrl_param.m` now warns when `f0 > f_c_i/2` or
  when the load corner is more than twice `f_c_u`.

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

Gains from the crossover targets in `private/buck_ctrl_param.m`: 225 Hz for the current loops, 60 Hz
for the voltage loop at nominal load (the plant of the voltage loop is the output capacitor with the
load, corner 80 Hz, behind the closed common-mode current loop, whose gain is only about 0.75 there
because the capacitor reacts to a common duty; about 100 Hz without load). Linear check in
`claude_analysis/interleaved_buck_loop_analysis.py` (python3, numpy/scipy/matplotlib; discrete model
at ts with the delays above, margins of both loops at nominal, light and no load, reference ramp and
load step): current loops 234 Hz on the inductor alone (276 Hz in common mode) with 59 deg of phase
margin on leg 1, 43 deg and 7.4 dB of gain margin on legs 2-3 (40 deg at no load); voltage loop
62 Hz / 123 deg at nominal load, 143 Hz / 73 deg at no load; 6.9 % dip on a +0.2 pu load step
recovered within 1 % in 44 ms. Figures `interleaved_buck_loops_bode.png`,
`interleaved_buck_step_response.png` (not tracked, written by the script).

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

## Signal dimensions in the discrete model (2026-10-26)

The first run of the fixed model stopped with "'Input Port 1' of .../current_ctrl/pi/PId/Sum5 is a
one dimensional vector with 1 elements": the three leg currents left the measurement stack as a
virtual bus (Bus Creator) and, once that bus had to cross the sample-and-hold Switch/Unit Delay loop
of `dcdc_ctrl`, Simulink could no longer propagate the width forward, so the per-leg PI was resolved
as a scalar while the modulator (Demux of the three duties) asked for three. Rules now applied in the
model: the per-leg quantities (`is_abc_dcdc_pu_mod1`, the held samples, the error of the three legs,
the three duties) are plain 3-element vectors built with Mux blocks, never buses; the Inports of
`buck_control`, `acquisitions`, `pi`, `PId`, `voltage_ctrl`, `pi_u` and of the modulator carry an
explicit port dimension (3 for the per-leg vectors, 1 for the scalars), so that no width depends on the
order of the propagation through the Unit Delay loops; the old `current_transformation` subsystem
(a Mux between a Demux and a scoped Goto, a leftover of the abc→dq transformation of the AFE) is gone,
the acquired vector feeds the PI directly.
