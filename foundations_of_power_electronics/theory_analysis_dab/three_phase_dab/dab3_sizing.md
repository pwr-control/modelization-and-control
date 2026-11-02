# DAB3 1200 V ↔ 1200 V, 250 kW: sizing note

Rev. 02, 2026-10-29 (run 01 in Sec. 10). Isolated bidirectional DC/DC stage of 250 kW with the three-phase dual active
bridge (DAB3): two six-step bridges, three single-phase transformers in star-star with floating star
points, phase-shift control. The numbers are in `private/dab3_param.m`, the model is
`three_phase_dab.slx` (see `README.md`). The steady-state figures come from `dab3_analysis.m` (harmonic
superposition, `private/dab3_harmonic_op.m`), which `init_model.m` and this note share.

**Working assumptions, to be confirmed**: DC voltages 1200 V on both sides (the old "690 V application"
case of the library data), transformer ratio 1:1, transformer frequency 10 kHz. With other voltages or
another frequency the procedure is the same; the numbers of Sec. 3-8 change.

## 1. Operating point

| Quantity | Value | Note |
|---|---|---|
| Side 1, side 2 voltage | 1200 V / 1200 V | assumption; batteries of the bench about 1248 V at soc 0.85 |
| Turns ratio | 1 (14 : 14) | three single-phase transformers, Y-Y, floating star points |
| Power | 250 kW | both directions |
| DC current | 208 A | normalization of the current loop |
| Transformer frequency fs | 10 kHz | = switching frequency of every device |
| fPWM_DAB of the model | 60 kHz = 6 fs | the library modulator makes the six-step pattern at fPWM/6 |
| Series inductance per phase | 28 µH (14 + 14 µH) | transformer leakage + external or integrated inductor |
| Phase shift at 250 kW | 30° | De Doncker, lossless |
| Devices | 2 kV SiC MOSFET half-bridge, `danfoss_SKM1700MB20R4S2I4` | 1.04 mΩ, Coss 3.28 nF |

## 2. Operating principle

Each bridge makes a six-step voltage: the phase-to-star-point voltage of the transformer takes the
levels ±V/3, ±2V/3; the floating star points remove the triplen harmonics, the spectrum is
h = 6k ± 1. Bridge 2 lags bridge 1 by φ, and the power flows from 1 to 2 for φ > 0 (De Doncker,
lossless, magnetizing inductance neglected, L = Ld1 + Ld2' per phase, V2' = V2 n1/n2):

    |φ| ≤ π/3:          P = V1 V2' / (ω L) · φ (2/3 − |φ| / (2π))
    π/3 ≤ |φ| ≤ 2π/3:   P = V1 V2' / (ω L) · (|φ| − φ²/π − π/18)

The power grows up to |φ| = π/2, where Pmax = 7π/36 · V1 V2' / (ω L), and falls beyond it. The
current control drives the modulator with u ∈ [−1, 1], φ = u · φ_max: the old φ_max = 2π/3 put the
maximum of the power inside the control range, where the gain of the plant changes sign. The new
φ_max is π/3 (1.7 Pnom at nominal voltages, 1.45 Pnom with V2 = 0.85 V1).

The harmonic model of `dab3_harmonic_op.m` (per phase T equivalent: Rs1 + Rds_on, Ld1, Lm ∥ Rfe,
Ld2', Rs2' + Rds_on) agrees with the formula within 0.05 % in the lossless case and gives the
currents, the switching currents, the magnetizing current and the DC-side ripple.

## 3. Series inductance and phase shift

The phase shift at nominal power sets the series inductance. At 10 kHz, 1200 V / 1200 V, 250 kW:

| φ_nom | L per phase | I phase (rms / peak) | Switching current | Pmax (d = 1 / d = 0.85) |
|---|---|---|---|---|
| 20° | 19.6 µH | 156 A / 229 A | 119 A | 716 / 609 kW |
| 25° | 23.9 µH | 159 A / 234 A | 122 A | 586 / 498 kW |
| **30°** | **28.0 µH** | **161 A / 240 A** | **124 A** | **500 / 425 kW** |
| 35° | 31.9 µH | 164 A / 245 A | 127 A | 439 / 373 kW |
| 40° | 35.6 µH | 167 A / 251 A | 130 A | 394 / 335 kW |

The rms current changes little with φ_nom; a small φ_nom gives a large power gain dP/dφ (control
resolution and sensitivity to the timing of the gates), a large one gives less margin for the voltage
range. 30° is the middle choice: 28 µH, Pmax = 2 Pnom, 1.7 Pnom at φ_max. The library class
`hw_dab_three_phase_setup` derives its own Ls = udc1² / (4 fPWM P) = 24 µH from fPWM_DAB:
`init_model.m` writes the 14 + 14 µH of this sizing over it.

## 4. Transformer

Three single-phase transformers, one per phase, on the core set of the 2025 SST note
(repositories-documentation, `solid_state_transformers`): 3 × AM-NC-320C nanocrystalline cut cores
per transformer, Ac = 47 cm², about 26 kg.

| Quantity | Value | Note |
|---|---|---|
| Peak flux linkage | 13.3 mVs | V / (9 fs) of the six-step phase voltage; 11.6 mVs at full load (drop on Ld1) |
| Turns | 14 : 14 | B = 0.20 T peak (no load), 0.18 T at full load |
| Magnetizing inductance | 1 mH, gapped | Im = 12-13 A peak; µr,eff = 622 with le = 0.72 m (TBC) |
| Core losses | about 107 W per transformer | Rfe = 3 kΩ; 16 W/kg at 4 kHz, 0.8 T scaled with f^1.5 B² |
| Winding + terminations + external inductor | Rs1 = Rs2 = 1.3 mΩ (TBC) | 68 W per transformer at 161 A rms |
| Leakage | about 12 µH in total at 10 mm between the windings | the rest of the 28 µH as external or integrated inductor |

The gap of the magnetizing inductance is a choice: 1 mH costs about 12 A of extra peak current and
gives the ZVS at light load (Sec. 5); without the gap (µr about 5000 of the cut cores) Lm is about 8 mH: the ZVS
limit at d = 1 moves from 9 % to about 15 % of the load, and a few amperes of DC current saturate the core.

First comparison of the frequency (same core set, turns for B ≤ 0.35 T, same devices, turn-off losses
as an upper bound, see Sec. 8): 2 kHz 0.74 kW, 5 kHz 1.4 kW, 10 kHz 2.7 kW, 20 kHz 5.2 kW of total
losses; the transformer at 2 kHz needs 41 turns, five times the volt-seconds of 10 kHz. 10 kHz keeps the
losses near 1 % and the magnetics small; 5 kHz is the conservative alternative. The old model ran at
2 kHz without knowing it: fPWM_DAB = 12 kHz and the modulator divides by 6.

## 5. Semiconductors and ZVS

Both bridges use the 2 kV SiC half-bridge module of the original model (1.04 mΩ, Coss 3.28 nF,
Eon/Eoff 77/108 mJ at 1300 V, 1000 A). It is oversized for 161 A rms (conduction 162 W for the 12
switches); a module in the 300-600 A class would do.

ZVS: at each commutation the current of the phase charges and discharges the two Coss of the leg,
transition time 2 Coss V / i_sw. At the nominal point i_sw = 124 A on both bridges, 63 ns, well inside
a dead time of 500 ns. With d = V2'/V1 ≠ 1 the bridge of the lower voltage loses the current at light
load (`dab3_analysis.m`, dead time 500 ns):

| d | ZVS of both bridges from |
|---|---|
| 1.00 | 9 % of the load (magnetizing current: i_sw = 17 A at 10 %) |
| 0.95 / 1.05 | 29 % / 31 % |
| 0.90 / 1.10 | 45 % / 52 % |
| 0.85 / 1.15 | 59 % / 72 % |

The light-load range with voltage mismatch is the known limit of the DAB3 with six-step and phase shift
only; if the batteries move by more than ±5 % the options are a DC link that follows the battery
(front-end stage), a lower Lm, or a modulation with a third degree of freedom (duty cycle / asymmetric
pulses).

**The model does not simulate the transitions**: the library modulator (`three_phase_dab_modulator`)
does not apply its `dead_time` parameter and the bridges have no Coss, so the legs switch at once. The
model gives the switching currents (sign and value at the edges), not the transitions.

## 6. DC links and chokes

The bridge DC current has the six-pulse ripple at 6 fs = 60 kHz (115 A to 240 A at full load): about
40 A rms in the DC link capacitors on both sides, 0.6 V peak-to-peak with 500 µF of film. Between the
batteries and the DC links there are 250 µH chokes (2 mΩ): the LC resonates at 450 Hz, damped by the
resistance of the battery (R0 = 72 mΩ with the bench settings). The DC links start at the open circuit
voltage of the batteries (`param.dab3.ic`), the battery blocks too (`battery_voltage_init_m`).

## 7. Control

Current loop on the DC input current of side 1 (`dab_current_input_modA`: the current of the DC choke
where it enters the DC link; the capacitor carries the six-pulse ripple, this current has none),
normalized by 208 A, PI of the library (`pi_ctrl`, sample time ts_dab = 1/fPWM_DAB = 16.7 µs, the
period of the trigger), feed-forward u_ff = k_ff · i_ref with k_ff = φ_nom / φ_max = 0.5 (30° at 1 pu,
exact at the nominal point, about 1° too much at half load).

Plant at the nominal point: d(i/208 A)/du = 1.9. The LC of the DC side puts a lightly damped pole pair
at 450 Hz in the loop and limits the bandwidth. Discrete loop gain (PI, one-sample delay of the
feedback, ZOH, battery impedance):

| kp, ki | Crossover | Phase margin | Gain margin (at 450-500 Hz) |
|---|---|---|---|
| 0.01, 60 (chosen) | 18 Hz | 90° | 14 dB |
| 0.02, 100 | 30 Hz | 91° | 10 dB |
| 0.05, 150 | 45 Hz | 94° | 6.5 dB |
| 0.10, 300 | 95 Hz | 98° | 0.4 dB |

With the feed-forward, an averaged model of the bench (batteries, chokes, DC links, DAB3 power
formula, quantization) tracks the reference ramps of 20 pu/s within 0.024 pu and has no static error
10 ms after the end of a ramp. With the old gains (kp = 0.02, ki = 1, k_ff = 0.25 with φ_max = 2π/3)
the error of the feed-forward (5 % at 1 pu on this bench) lasts about a second. The voltage loop
(`Vout Ctrl`) is kept but not active (the Variant Source selects the current reference); its gains are
those of the old model.

Bench reference (`param.dab3.bench`, case `forward`): 0 until 10 ms, −1 pu (power from side 2 to
side 1) with a ramp of 20 pu/s, reversal at 0.1 s to +1 pu (100 ms), forward power at 1 pu from 0.2 s
to the end of the run (0.3 s); the last 20 ms are logged. The cases `reverse`, `light` and `mismatch`
end at −1 pu, +0.1 pu and +0.25 pu (the last with battery_2 at soc 0.3), see Sec. 10.

In run 01 the loop holds 208.4 A (reference 208.3 A), with a limit cycle of ±3 A at 500 Hz: the
modulator rounds φ to whole steps of tc (0.36° at 100 ns, about 1.2 % of the power at 28°), φ toggles
between two steps and the toggling excites the LC of the DC side. It is the resolution of the model; a
digital PWM with 1-10 ns of phase resolution would give 0.004-0.04°.

## 8. Loss budget at 250 kW (estimate)

| Item | Loss | Note |
|---|---|---|
| Conduction, 12 switches | 162 W | 1.04 mΩ, 161 A rms per leg |
| Turn-off, 12 switches | ≤ 1490 W | Eoff of the dataset scaled with V·I at 124 A: upper bound, with ZVS the Coss acts as a snubber |
| Turn-on | 0 | ZVS |
| Body diodes in the dead time | 26 W | 500 ns − 63 ns at 124 A, 4 V |
| Windings, terminations, external inductors | 203 W | 1.3 mΩ per side and phase |
| Cores | 320 W | Rfe = 3 kΩ |
| DC chokes and capacitors of the bench | about 175 W | 2 mΩ per side at 208 A, 1 mΩ at 40 A rms |
| **Total** | **≈ 2.4 kW** | **η ≈ 99.0 %** (99.6 % if the turn-off losses are small) |

The turn-off term needs the measured soft turn-off energy of the module at 1200 V, 100-150 A.

## 9. Model and numerical settings

- `init_model.m` builds `param` (numbers in `private/dab3_param.m`), the model reads only `param.*`.
  Library classes and blocks are used as they are; the DC links and the choke of side 2, the
  reference of the voltage loop, the battery data and the PI sample time no longer come from the
  single-phase DAB data (`hwdata.dab`) of the library.
- Simscape local solver Backward Euler at tc = 100 ns (1000 steps per period, phase resolution
  0.36°, 1 % of the power at 30°), ode23tb with MaxStep tc. 500 ns for quick structural checks.
- Backward Euler dissipates L/2 · (Δi)² per step in every inductance, that is tc/(2L) · v_L² on average.
  The leakage inductances carry currents with slopes up to 30 A/µs: in run 01 the numerical damping is
  0.87 kW in Ld1, Ld2 and 0.05 kW in Lm, more than the physical losses of the model (windings 0.20 kW,
  cores 0.30 kW, conduction 0.16 kW). It scales with tc (0.45 kW at 50 ns). `dab3_run_check.m`
  computes it from the logged currents and gives the efficiency without it.
- The bridges are `three_phase_inverter_mosfet_based_with_thermal_model`: ideal switches with a
  thermal network. They conduct 13 mA in the off state at 1.2 kV (off-state conductance of the model,
  about 0.1 kW for the 12 switches). Their `ploss` outputs add Eon and Eoff of the dataset at every
  edge, as in hard switching (2.7 kW in run 01); the electrical circuit has no switching loss.
- The transformer flux starts at zero when the bridges start: the magnetizing current has a DC
  offset of up to about 13 A, which decays with Lm / (R1 ∥ R2'), about 0.9 s (run 01: 8 A at 0.3 s). It
  shifts the switching currents of the rising and falling edges of a leg in opposite directions: average
  the two edges to read i_sw (`dab3_run_check.m` does).

## 10. Checks with the model

`init_model.m` selects the bench with `param.sim.bench_case`; after the run, `dab3_run_check.m` prints
the measured operating point next to the harmonic model, the loss breakdown and the DC side.

| Case | End of the run | Purpose | Status |
|---|---|---|---|
| `forward` | +1 pu | nominal point, Sec. 1-8 | run 01 |
| `reverse` | −1 pu | same quantities, power from side 2 to side 1 | run 02 |
| `light` | +0.1 pu | switching currents set by the magnetizing current | run 03 |
| `mismatch` | +0.25 pu, battery_2 at soc 0.3 (d ≈ 0.96) | ZVS limit of bridge 2 (Sec. 5) | run 04 |

The reversal (0.1-0.2 s) is outside the logged window: watch it on the scopes, or log it with a longer
`param.time.t_measure` and a decimation (the loss breakdown of `dab3_run_check.m` needs decimation 1).

Results (tc = 100 ns, logged window 0.28-0.30 s; in brackets the harmonic model at the measured
V1, V2 and φ):

| Quantity | 01 `forward` | 02 `reverse` | 03 `light` | 04 `mismatch` |
|---|---|---|---|---|
| V1 / V2 | 1210 / 1269 V (d = 1.05) | 1271 / 1208 V (d = 0.95) | 1237 / 1243 V (d = 1.005) | 1233 / 1186 V (d = 0.96) |
| Power side 1, side 2 | 252.1 kW in, 250.4 kW out | 264.7 kW out, 266.5 kW in | 25.7 kW in, 25.1 kW out | 64.3 kW in, 63.6 kW out |
| DC input current, side 1 | 208.4 A (reference 208.3 A) | −208.3 A | 20.8 A | 52.2 A |
| φ (mean over all the edges) | 28.25° | −30.14° | 2.56° | 6.83° |
| Phase current | 157.7 (157.6) A rms | 168.5 (168.4) A rms | 16.4 (15.1) A rms | 42.1 (41.6) A rms |
| Switching current, bridge 1 | 99.1 (100.5) A | 151.8 (151.8) A | 13.4 (14.8) A | 50.2 (51.6) A |
| Switching current, bridge 2 | 143.1 (143.0) A | 105.7 (107.1) A | 19.6 (19.6) A | 16.0 (15.9) A |
| Transition, bridge 1 / 2 (dead time 500 ns) | 80 / 58 ns | 55 / 75 ns | 607 / 415 ns | 161 / 487 ns |
| Losses between the DC links | 1.66 kW | 1.76 kW | 0.57 kW | 0.68 kW |
| of which numerical damping, off-state conduction | 0.91, 0.10 kW | 0.97, 0.10 kW | 0.13, 0.10 kW | 0.25, 0.10 kW |
| windings, cores, conduction | 0.20, 0.30, 0.16 kW | 0.22, 0.29, 0.18 kW | 0.00, 0.33, 0.00 kW | 0.01, 0.31, 0.01 kW |
| η without numerical damping and off-state conduction | 99.74 % | 99.74 % | 98.69 % | 99.48 % |
| Limit cycle of the DC input current (500 Hz) | ±3 A | ±0.6 A | ±4 A | ±3.4 A |
| Magnetizing current offset (start) | 2.1 / −8.3 / 6.2 A | 1.6 / −8.5 / 6.8 A | 1.7 / −8.2 / 6.5 A | 1.6 / −8.1 / 6.5 A |

The run loses about 1 kW more than the harmonic model at full load: the numerical damping and the
off-state conduction of the switch model. The peak currents and the magnetizing current carry the DC
offset of the start (Sec. 9). With the turn-off estimate of Sec. 8 at the point of run 01 (1.5 kW,
upper bound) the total is about 2.2 kW, η ≈ 99.1 %, as `dab3_analysis.m` gives for d = 1.05 at full
load.

- Run 02, reverse power: the loop regulates the current of side 1, so −1 pu with battery_1 charging
  at 1271 V is 265 kW on side 1 and 266.5 kW (220.6 A) from side 2, 6 % above 250 kW. A power
  reference, or the current reference scaled by 1200 V / V1, would keep 250 kW in both directions.
- Run 03, light load: bridge 1 (the lower voltage, d = 1.005) has 13.4 A at the commutations and
  needs 16.2 A to complete the transition in 500 ns: the leg voltage reaches 80-90 % of its swing
  before the turn-on (partial ZVS, about 10 W of residual turn-on loss, Sec. 5 puts the limit at 9 %
  of the load with d = 1). The cores dominate the losses (334 W, 1.3 % of 25 kW; Rfe is still a
  placeholder). One step of φ is 14 % of the power at 2.5°, so the limit cycle is ±20 % of the
  current; the flux offset of the start (8 A) brings some edges of bridge 1 to a negative switching
  current (minimum −1.6 A). In the converter a soft start with the first pulses at half width avoids
  the offset.
- Run 04, voltage mismatch: battery_2 at soc 0.3, +0.25 pu, d = 0.96. Bridge 2 (the lower voltage)
  commutates 16.0 A (harmonic model 15.9 A) against 15.6 A needed in 500 ns: the point sits on the
  ZVS boundary of Sec. 5, as the map predicts (d = 0.95: from 29 % of the load). Bridge 1 has 50 A.
  With the flux offset the edges of bridge 2 spread from −1.2 A to 20 A.

The four runs confirm the harmonic model (currents and switching currents within 2 %, power within
0.5 % once the numerical damping is accounted for) and the ZVS map in both directions, at light load
and with different battery voltages.
