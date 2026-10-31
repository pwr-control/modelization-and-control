# single_phase_afe_ttype — single-phase three-level T-type bridge on the grid

Single-phase three-level T-type bridge (SiC MOSFET, loss and thermal model) connected between two
phases of a 690 V grid through an L filter, with a split DC link (2 × 12 mF, centre tap = neutral
point of the bridge) fed by a 1200 V lithium-ion battery. 225 kW class, 4 kHz PWM with double update
(control at 8 kHz). The current amplitude reference is stepped in three times to 1 pu in phase with
the grid voltage: the model is a battery inverter feeding the grid, there is no DC-link voltage loop
(the "AFE" of the name is the legacy name of the family of models).

| File | Content |
|---|---|
| `init_model.m` | initialization: every number in the struct `param`, then it opens the model. Run it **from this folder** (the functions of `private/` are visible only to the files of this folder). |
| `private/` | the parameter functions of this model (table below) |
| `single_phase_afe_ttype.slx` | the model: three-phase grid emulator and Dyn11 transformer, L filter, T-type bridge with the MOSFET loss and thermal model, split DC link, battery, the whole control, identification and impedance-measure subsystems |
| `plotting_results.m` | post-processing of the logged results (reads `param.glb_time.*`) |
| `init_model_legacy.m` | the old flat init (shared template with seven clock domains and the DAB/PSFBC/CLLC/machine sections), kept for reference only: it no longer runs against the current library (see below) |
| `figures/` | EPS figures of the post-processing (git-ignored) |

## The model

**Power stage.** The grid emulator gives the three phase voltages of a 690 V, 50 Hz grid (ideal
sources, no impedance, fault generator off) on the primary of a Dyn11 1 MVA 690/690 V transformer made
of three single-phase transformer blocks (`Ld1` 113.7 µH, `Rd1` 11.9 mΩ, `Ld2` 37.9 µH, `Rd2` 11.9 mΩ,
`Lm1` 144.8 mH, `Rfe1` 265 Ω, 24/14 turns). The bridge is connected between phases *a* and *b* of
the secondary through 250 µH + 250 µH (one half on each line) with 200 µF (50 mΩ) across the
output: with the transformer leakage (2 × 37.9 µH) the filter resonates at about 1.4 kHz. The
bridge is the library block `full_bridge_three_level_ttype_mosfet_based_model` with the
Danfoss/Semikron SKM1700MB20R4S2I4 data set embedded in `private/device_mosfet_param.m` (eight
switches, four per leg: two vertical, two in the bidirectional path to the neutral point; each
commutation at `udc/2`). The DC link is the library block `dclink_for_three_levels`: 2 × 12 mF with
a 107 kΩ measuring divider, initial voltages `udc_nom/2`; the neutral point is the centre tap. The
battery (library block `lithium_ion_battery`, 1200 V, 250 kW, SOC 0.85, `R0` and `R1` halved and
`C1` divided by 50 as in the legacy init) is across the whole DC link.

**Control** (all at `ts = 125 µs`, trigger at the bottom and at the top of the carrier):

- FHT-PLL on the three secondary voltages (first harmonic trackers on α and β, PI 1/80 on the q
  voltage, double-integrator observer of the phase);
- current amplitude reference: three steps of 1/3 pu at 25 ms, 0.5 s and 1 s, rate limited
  (`param.afe.ref.slew` = ω_nom pu/s), on the d axis; the q reference is 0;
- converter current: the measured current is α, the virtual β is built by a SOGI (C-Caller, variants:
  first harmonic tracker, phase-shift filter, Simulink or C-Caller), αβ→dq at the PLL angle, dq vector
  PI (C-Caller `dqvector_pi`, 0.5/45, limit 1 with the vector clamp; variants: Simulink
  `dq_vector_pi`, resonant PI 0.75/45 on the α current), dq→αβ, α to the three-level modulator
  (input −1..1 = −udc..udc, dead time 3 µs);
- the division of the modulating signal by `udc_pu` uses the Constant 1 of the model, not the measured
  DC voltage;
- the neutral-point voltage is only measured (`idc1`, `idc2`, `udc_p_gnd`, `udc_n_gnd` are logged):
  there is no balancing loop.

The identification subsystem (open-loop u/f test at 300 Hz) and the impedance-measure subsystem
(moving averages at `f_set`) are the legacy ones, selected with `param.afe.variant.system_identification`.

## How to run

1. Add the `library` repository to the MATLAB path with subfolders (`init_model.m` does it for
   `C:\github\library`): the library provides the Simulink/Simscape blocks, the C-Caller sources and
   the device data sets, no parameter.
2. From this folder run `init_model.m`: it fills `param`, prints the list of the local functions of
   `private/`, the bases and the design figures, opens `single_phase_afe_ttype.slx`, imports the
   C-Caller bus types, closes the scopes and checks that every `param.*` name read by the model exists
   (`check_model_param_refs`).
3. Run the simulation (`param.sim.simlength = 1.75` s, Simscape local solver step `param.sim.tc` =
   0.625 µs, 2.8 million logged points at `tc`) and `plotting_results.m`.

## Parameter structure

| Group | Content |
|---|---|
| `param.sim` | `simlength`, `tc` (0.625 µs: `FixedStep`, Simscape local solver step and logging base), `use_mosfet_thermal_model` (only sets `nonlinear_iterations` 3 → 5 as in the legacy init: the ideal bridge is commented out in the model), `check_param_refs` |
| `param.time`, `param.glb_time` | `fpwm` 4 kHz, `trgo` 1 (double update), `dead_time` 3 µs, `tc_factor` 200, the battery estimator domain `fpwm_dab`; `glb_time = timing_param(param.time)` gives `fPWM_AFE`, `TRGO_AFE_double_update`, `dead_time_AFE`, `ts_afe` 125 µs, `ts_dab`, `tc`, `Nc`, `Ns_afe` (the names the blocks read). The legacy `ts_inv`, `Ns_inv` and `dead_time_INV` read by the control blocks had the same values and are mapped to the AFE names. |
| `param.grid` | 690 V, 50 Hz, transformer data (`pwr_nom` 1 MVA, `ucc` 5 %, `eta` 95 %, `p_iron` 1.8 kW, turns, core geometry, `mur`); `frt = grid_fault_param('two_phase_D', 0.75, 0.915, 0.75, 0, 0)` (legacy test 25.4, both enables at 0); `emu = grid_emulator_param(param.grid, param.grid.frt)` (bases `u2bez` 563.4 V, `i2bez`, references, `sof_50Hz` state space of the amplitude ramps, `trafo` = the transformer fields read by the three blocks) |
| `param.afe.hw` | `afe_hw_param(690, 225e3, fpwm)`: the legacy single-phase data set (bases `ubez` 838.4 V, `ibez` 536.8 A, `udc_nom` 1239.4 V, `norm.Xbez`), split DC link (`CFi1`, `CFi2` 12 mF, `Rudc_measure` 107 kΩ, `udc1_init`, `udc2_init`), L filter (`LFu1`, `LFu2`, `RLFu1`, `RLFu2`, `CFu`, `RCFu`) and the grid figures (`grid.u_pu` 1.164, `grid.i_pu`, `grid.p_1pu` 262 kW, `grid.m_nom`, ...) |
| `param.afe.pwm` | `fpwm`, `double_update`, `dead_time`, `tc` read by the three-level modulator |
| `param.afe.variant` | `rpi_enable`, `current_ctrl_ccaller` (1), `system_identification`, `beta_fht`, `beta_sogi`, `beta_phsh`, `beta_sogi_ccaller` (1), `beta_phsh_ccaller` (exactly one β variant active) |
| `param.afe.ctrl` | `afe_ctrl_param(ts, omega_nom, f_set)`: current PI `kp_i`/`ki_i`, `u_lim`, resonant PI `rpi.*` (gains and the continuous/discrete resonant model), `fht.*` (tracker model and observer gains), `sogi.*` (`kepsilon`, `flt_beta` tf of the acquisition filter), `phsh.gain`, `pll.*`, `di_obs.*`, `f_set`/`omega_set` (grid frequency, 300 Hz in the identification test) |
| `param.afe.ref` | `i_pu_1..3`, `t_1..3`, `slew`, `i_pu_tot`, `p_tot` |
| `param.afe.design` | informative figures printed by the init: peak current, modulation index, 100 Hz current and energy pulsation, neutral-point charge pulse, switching ripple, filter resonance, current-loop crossover estimate |
| `param.afe.mosfet`, `param.dev` | MOSFET data set (`device_mosfet_param`, SKM1700MB20R4S2I4 embedded, other names through `device_data_load` of the library) |
| `param.meas` | ADC quantization (`adc_quantization` 1/2^11, read by every quantizer of the acquisitions), full scales, bases of the grid-side logging (`ugrid_bez` 563.4 V, `igrid_bez` 328.7 A of the legacy three-phase AFE data set) |
| `param.thermal` | heatsink object read by the bridge mask (`heatsink.no_weight`, `heatsink.cp`, `heatsink.Tambient`), `Rth_mosfet_HA`, `DThs_init`, loss and junction-temperature estimate of the bridge |
| `param.battery` | `nominal_battery_voltage_1` 1200 V (read by the mask default `battery_voltage_init_m` of the battery block, now also in the instance), power, SOC, `pack_1 = battery_param(...)` with the legacy modifications of `R0`, `R1`, `C1` |

### Functions of `private/`

| Function | Role |
|---|---|
| `timing_param(t)` | the converter clock domain (and the legacy DAB domain of the battery estimator) |
| `afe_hw_param(V, P, fpwm)` | hardware data and per-unit bases of the bridge, split DC link, L filter, grid figures |
| `afe_ctrl_param(ts, omega, f_set)` | every control gain; FHT, PLL observer and resonant PI models; observer gains by closed-form pole placement (no toolbox) |
| `grid_emulator_param(g, fault)`, `grid_fault_param(...)` | grid emulator bases and references, Dyn11 transformer data (same formulas as the library class `three_phase_transformer_setup` and the legacy geometry), fault definition |
| `device_mosfet_param(name, fpwm, udc)` | MOSFET data set and derived thermal/snubber values |
| `battery_param(...)` | battery pack (copied from the other migrated models) |
| `check_model_param_refs(model, param)` | lists the `param.*` names read by the model and reports the missing ones |

## What changed in the model (October 2026)

- 461 block parameters re-pointed from the flat names (`glb_time.*`, `hwdata.*`, `grid_emu.*`,
  `frt_data.*`, `filters.afe.*`, `afe_ctrl.*`, `single_phase_inverter_ctrl.*`, `mosfet.inv.*`,
  `heatsink.*`, `lithium_ion_battery_1.*`, ...) to `param.*`; the variant conditions read
  `param.afe.variant.*`. 133 distinct `param.*` names, all defined by the init (checked).
- Names the legacy init never defined (read by the inactive variants and by the identification
  subsystem: `Vdc_nom`, `Vac_FS`, `Xbez`, `ts_inv`, `kp_inv`, `ki_inv`, `omega_fht1`, `delta_fht1`,
  `Ld_fht1`, `phase_shift_filter_gain`, `adc11_quantization`) are mapped to the corresponding
  `param.*` fields, so every variant can now be activated.
- Configuration set: `StopTime = param.sim.simlength`, `FixedStep = param.sim.tc`,
  `MaxStep = param.glb_time.tc`; Solver Configuration `MaxNonlinIter = param.sim.nonlinear_iterations`.
  The inactive configuration set 0 still reads `Ts`, as before.
- Three-level modulator: `f`, `dead_time`, `tc`, `double_sampling` read `param.afe.pwm.*` (gates
  +15/−10 V, polarity and limits −1..1 are the mask defaults, unchanged).
- Split DC link: `Rudc_measure` was typed in the block (107e3) and is now `param.afe.hw.Rudc_measure`;
  the initial voltages read `param.afe.hw.udc1_init`/`udc2_init` (= `udc_nom/2`, same value).
- Battery block: `battery_voltage_init_m = param.battery.nominal_battery_voltage_1` is written in the
  instance (it was only the mask default of the library block, a hidden dependency on the workspace).
- Rate limiter of the current reference: `±param.afe.ref.slew` instead of `±grid_emu.omega_grid_nom`
  (same value).
- The three MATLAB Function blocks that use the names `Vac_FS`, `Xbez` take them as inputs (no
  workspace access): unchanged.
- New `ModelUUID`.

The legacy init `init_model_legacy.m` calls `grid_fault_generator` without arguments and
`lithium_ion_battery_setup` with the old signature: it does not run against the current library and
is kept only to read the old numbers.

## Open points found during the migration

The control was not changed; these are the points to look at.

1. **Bases.** The 690 V grid between phases *a* and *b* is 975.8 V peak = 1.164 pu of `ubez` (838.4 V,
   the stand-alone-inverter base of the library data set); 1 pu of current reference is 536.8 A peak
   = 262 kW at 690 V, 116 % of the 225 kW rating and above the 250 kW battery rating. The modulation
   index at `udc_nom` = 1239.4 V is 0.79, but the battery holds the link at about 1200 V minus the
   drop on `R0` (72 mΩ × 218 A ≈ 16 V) and the modulating signal is divided by the Constant 1, not
   by the measured `udc_pu`: the integrator of the current PI absorbs the difference.
2. **Neutral point.** With symmetric carriers and equal modulating signals on the two legs the
   averaged neutral-point current is zero, so no loop is needed in the ideal case; but any
   asymmetry (dead time, unequal capacitances, DC offset of the reference or of the current
   measurement) makes the centre tap drift and nothing corrects it: the 107 kΩ divider on 12 mF has a
   time constant of about 20 minutes. The worst-case charge pulse into the neutral point in one
   switching period is 67 mC (2.8 V on 24 mF). A neutral-point balancing loop (DC offset on the
   modulating signal with the sign of the current, acting on `udc_p_gnd − udc_n_gnd`) is the usual
   fix and is missing.
3. **100 Hz pulsation into the battery.** The single-phase power pulsation at 262 kW is 211 A peak
   at 100 Hz on the DC side (834 J peak-to-peak). The series DC link (6 mF, 265 mΩ at 100 Hz) alone
   would ripple by 56 V (4.5 %), but the battery branch (`R0/2` = 72 mΩ) is in parallel: most of the
   100 Hz current (about 170 A peak over the 218 A average) flows into the battery and `udc` ripples
   by about 12 V. There is no active power decoupling in this model (see
   `single_phase_afe_with_active_power_decoupling`).
4. **Current loop and filter resonance.** With `kp_i` = 0.5 and the plant gain `udc_nom/(ibez·LFu)`
   = 4618 1/s the crossover estimate is 370 Hz (Nyquist 4 kHz). The L–C–transformer resonance is at
   about 1.4 kHz with only `RCFu` = 50 mΩ and the transformer resistance as damping: the margins at
   the resonance were not computed in this migration (an averaged check as in
   `single_phase_afe_with_active_power_decoupling/claude_analysis` can be added).
5. **Dead time** 3 µs (1.2 % of the 250 µs period) is large for a SiC module and gives the usual
   voltage distortion at the current zero crossings (about 2.4 % of `udc/2`); the legacy value is kept.
6. **Initial conditions.** The two halves of the DC link start at `udc_nom/2` = 619.7 V while the
   battery open-circuit voltage at SOC 0.85 is close to 1200 V: the first milliseconds are a
   precharge transient through `R0`.
7. **Losses and thermal.** At 262 kW the averaged estimate of the init gives about 650 W for the
   bridge (vertical switch 50 W conduction + 60 W switching, middle switch 49 W + 3 W) and junction
   temperatures around 45 °C with 40 °C water: the 1700 A module is largely oversized at 537 A peak;
   the heatsink thermal mass is 1/10 of the plate (legacy `no_weight`).
8. **Measurements.** The PLL reads the three secondary voltages (three voltage sensors for a
   single-phase converter); the grid-side logging `ug_pu_mod1`/`ig_pu_mod1` uses the bases of the
   three-phase AFE data set (563.4 V, 328.7 A), different from the control bases.
