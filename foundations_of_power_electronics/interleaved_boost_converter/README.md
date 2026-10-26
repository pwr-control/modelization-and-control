# Three-leg interleaved boost converter

200 V battery to a 660 V DC link, 750 A of input current (150 kW, 250 A per leg, duty of the lower switch
0.697), three boost legs with interleaved carriers at 3 kHz (double update, control at 6 kHz; the capacitors
see 9 kHz), Infineon FF450R17ME4 half bridges (1700 V / 450 A), 0.8 mH per leg, 7.5 mF across the battery,
7.2 mF on the DC link, resistive load on the DC link. The hardware is, on purpose, the one of
[`../interleaved_buck_converter`](../interleaved_buck_converter) (same module, inductors, capacitors, DC
link, modulator): the same converter with the power flow reversed. The control is the boost one. Two
models of the same converter, driven by the same `init_model.m` (`param.boost`, `param.battery`,
`param.load`, `param.glb_time`):

- `interleaved_boost.slx` — discrete blocks: library modulator (three carriers, TRGO of each one),
  IGBT bridge with the loss and thermal model used as three legs, DC-link block, three inductors,
  three capacitors across the battery, battery; control in Simulink blocks (`dcdc_ctrl`).
- `interleaved_boost_cmp.slx` — the compact block `interleaved_boost_3legs_mod1` of the library
  (power stage as the Simscape language component `boost_ssc.interleaved_boost_3legs`, control and
  interleaved PWM as one MATLAB Function) with the same battery, global timing and load.
  `build_interleaved_boost_compact.m` generates the minimal `interleaved_boost_compact.slx` (not
  tracked) around the same block, after `init_model`, once the library block exists:

  ```matlab
  cd library/foundation/pwr_electronics
  ssc_build boost_ssc
  cd ../../masked_models/interleaved_boost_3legs
  update_interleaved_boost_3legs
  cd <this folder>
  init_model; build_interleaved_boost_compact; sim('interleaved_boost_compact')
  ```

Both `.slx` of this folder were derived (2026-10-26) from the buck models without MATLAB, by editing the
model XML: they must be opened, updated (Ctrl+D) and saved once in Simulink; `init_model` checks that every
`param.*` read by the blocks exists.

## From the buck to the boost (what changed in the discrete model)

Same half bridge, inductor on the midpoint: with the battery on the inductor side and the load on the
DC link the buck is a boost, so the power stage of `interleaved_buck.slx` is kept as it is and only the
connections and the control change:

- battery on the low-voltage terminals (the `u_out` side of the buck, now `u_in`), load on `Udc`;
- the three leg current sensors and the terminal current sensor are reversed: the currents are positive
  from the battery into the legs (boost direction), so the per-leg PI keeps its positive gain;
- the voltage loop regulates the acquired DC-link voltage `udc` (reference `param.boost.ctrl.udc_ref`,
  1 pu of `udc_nom`) instead of the low-voltage side; the reference ramp starts at
  `param.boost.ctrl.udc_ref_init` = u_in/udc_nom = 0.303 pu, because the DC link cannot be regulated below
  the battery voltage, and reaches 1 pu with `u_ref_slew` = 10 pu/s (70 ms; the DC link stores 1.6 kJ at
  660 V, a faster ramp with the nominal load connected saturates `i_lim`);
- the per-leg PI outputs the duty `d` of the **lower** switch (the boost switch), limited at
  `m_low..m_up`; the stage `1 - d` (`current_ctrl/one_minus_d`) gives the duty `m` of the upper switch to
  the unchanged modulator (polarity -1: Q1 on around the carrier bottom, Q2 around the top). Since the leg
  current rises while Q2 is on, it is at its average value at both extremes of its own carrier, as in the
  buck: the sampling scheme is unchanged;
- the capacitors across the battery start at `Vin0` = 200 V (no inrush from the battery at t = 0). The
  `dclink_for_two_levels` block has no initial voltage: in the discrete model the DC link starts at 0 V and
  the current loops charge it at `i_lim` (900 A) in the first 2 ms, the voltage loop takes over at 200 V
  (the compact block starts instead at `Vdc0` = 200 V, the link precharged through the upper diodes);
- the subsystems keep the structure of the buck (`dcdc_ctrl/boost_control`, `interleaved_boost`,
  `tri-interleaved_boost_pwm`); a negative current reference is the buck direction (DC link to battery),
  the model is bidirectional.

## Sizing (2026-10-26)

Specifications: Uin = 200 V, Udc = 660 V, Iin = 750 A, fsw = 3 kHz per leg, IGBT module FF450R17ME4,
hardware of the buck. The design inputs are `spec.*` in `init_model.m`, which prints the minima and the
figures of the chosen values (the boost figures of the buck hardware):

- **Leg inductor.** The inductor sees Uin for the fraction D = 1 - Uin/Udc = 0.697 of the period:
  `dI_leg = Uin D/(L fsw) = Udc M (1 - M)/(L fsw)`, M = 1 - D = 0.303, the same ripple as the buck at the
  same voltages; `L_min = 0.743 mH` for 25 % of 250 A, `L = 0.8 mH` (58 A peak-peak, 23 %).
- **Capacitors across the battery.** The three ripples add with the 120° shift to 8 A peak-peak at 9 kHz
  at the nominal duty (M close to 1/3), 22.9 A worst case over the duty: the 0.2 % ripple specification
  needs 0.8 mF, the 7.5 mF of the buck give 15 mV (42 mV worst case). In the boost this capacitor has no
  control role: the battery (R0 of a few mΩ) sits across it, the inductors see the battery voltage at the
  control frequencies and the L/3 – C resonance (112 Hz) is damped by the battery.
- **DC-link capacitor.** The regulated one. The upper diodes deliver a pulse train at 9 kHz (the
  conduction intervals of the three legs do not overlap at M < 1/3) with duty D' = 3M = 0.909: 75 A rms in
  the capacitor and `dU_dc = D'(1 - D') i_leg/(3 fsw Cdc) = 0.34 V` with the 7.2 mF of the buck (1.7 mF
  would meet 0.2 %). The load corner of the voltage plant is `2/(2π R_load Cdc) = 15 Hz` (the factor 2: at
  constant input current the DC-link current falls as 1/udc); a +20 % load step gives a 2.4 % dip.
- **Right-half-plane zero.** With the current loops closed the DC-link current of a leg is (1 - d) i_leg
  and a current increase is obtained by first increasing d: the plant from the input current to udc is
  `k_u (1 - s/w_z)/(s + 2/(R Cdc))`, `w_z = (Uin - 2 RL i_leg)/(L i_leg)` = 157 Hz at nominal load (it moves
  as 1/i_leg at lighter load, there is no zero without load). The voltage loop must cross below f_z/4 and
  its gain does not roll off above the zero: 40 Hz chosen (8 dB of gain margin on the zero, 83° of phase
  margin); at 60 Hz the gain margin would be 5.8 dB.
- **Module.** FF450R17ME4 at 250 A per leg (300 A at `i_lim` = 1.2 pu), 660 V, 3 kHz, same data set as the
  buck. Boost roles: the lower IGBT conducts for D = 0.697 of the period, the upper diode for 0.303 (the
  upper IGBT is driven but the reverse current goes through its diode). Loss estimate of the init with the
  linear loss model of the bridge: lower IGBT 300 W conduction + 354 W switching, upper diode 109 W
  conduction + 134 W recovery, 898 W per leg, Tj 111 °C (IGBT) and 81 °C (diode) with water at 40 °C,
  efficiency 97.5 % with the inductors (6 mΩ per leg). The linear scaling of the energies underestimates
  Erec at 250 A.
- **Control.** `f_c_i = 225 Hz` (the plant duty → leg current is the integrator udc_nom/(L ibez), the
  same as the buck: same gains), `f_c_u = 40 Hz`; `private/boost_ctrl_param.m` computes the gains, sweeps
  the closed-form voltage loop for its margins and warns when `f_c_u > f_z/4`, when the common-mode
  resonance of the current plant `M sqrt(3/(L Cdc))/2π` = 35 Hz is above `f_c_i/2`, or when the
  current loop crossover is above fs/20.

## Control

Cascade of a DC-link voltage PI (reference ramped from 0.303 pu, output the total input current reference
limited at `i_lim`) and one PI per leg on the leg current with the reference `i_ref/3`: the legs share the
current by construction (the per-leg factors `kL`, `kRL` of the inductors give mismatched legs for the
sharing studies). Timing of the samples and of the duties, unchanged from the buck:

- the control runs at the extremes of carrier 1 (double update);
- the current of each leg is sampled and held at the extremes of **its own** carrier, where the leg
  current is at its average value (the ON pulse of the lower switch is centred on the carrier top, the one
  of the upper switch on the bottom); the control reads the held values at its step, one step behind;
- the duty of leg 1 is loaded at the control step, the duties of legs 2 and 3 at the next extreme of
  their own carriers (shadow registers): legs 2 and 3 see 2.5 ts of loop delay against 1.5 ts of leg 1.

Gains from the crossover targets in `private/boost_ctrl_param.m`: 225 Hz for the current loops
(kp_i 1.285, ki_i 363), 40 Hz for the voltage loop at nominal load (kp_u 5.32, ki_u 267; the plant seen by
the voltage PI is the DC-link capacitor with the load and the RHP zero behind the closed common-mode
current loop, whose gain is 1.02 there; about 40 Hz without load, where the plant is the integrator
k_u/s). Linear check in `claude_analysis/interleaved_boost_loop_analysis.py` (python3,
numpy/scipy/matplotlib; linearized discrete model at ts with the delays above for the margins, nonlinear
averaged model for the time response, nominal, 5 % and no load, reference ramp from the precharged DC link
and load step): current loops 234 Hz on the inductor alone (240 Hz in common mode) with 59 deg of phase
margin on leg 1, 45 deg and 7.7 dB of gain margin on legs 2-3 (the same at no load: the common-mode
resonance at 35 Hz is inside the loops); voltage loop 40 Hz / 83 deg with 8.1 dB of gain margin at 174 Hz
at nominal load, 43 Hz / 78 deg at no load; ramp 0.303 → 1 pu in 70 ms without overshoot and without
reaching `i_lim` (277 A per leg); 2.4 % dip on a +20 % load step recovered within 1 % in 31 ms. Figures
`interleaved_boost_loops_bode.png`, `interleaved_boost_step_response.png` (not tracked, written by the
script).

## Signal dimensions in the discrete model

The rules of the buck apply unchanged (per-leg quantities as plain 3-element vectors built with Mux, never
buses; explicit port dimensions on the Inports of `boost_control`, `acquisitions`, `pi`, `PId`,
`voltage_ctrl`, `pi_u` and of the modulator); the `1 - d` stage is a Sum of a scalar Constant and the
3-element duty vector, so it keeps the width.
