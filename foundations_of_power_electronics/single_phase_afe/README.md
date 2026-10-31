# single_phase_afe — single-phase AFE with active power decoupling

Full-bridge active front end (AFE) between two phases of a 690 V grid, 225 kW class, 5 kHz
unipolar PWM with double update (control at 10 kHz), with a second full bridge — the *ripple port* —
that takes the 100 Hz power pulsation of the single-phase connection away from the DC link. The
method is described in the note *Active power decoupling of a single-phase AFE with a ripple port*
(`repos_doc/single_phase_afe_power_decoupling/` of `repositories-documentation`, copy in
`library/library/documentation/single_phase_afe_power_decoupling/`).

| File | Content |
|---|---|
| `init_model.m` | initialization: every number in the struct `param`, then it opens the model. Run it **from this folder** (the functions of `private/` are visible only to the files of this folder). |
| `private/` | the parameter functions of this model (table below) |
| `single_phase_afe.slx` | the model: three-phase grid emulator, LCL filter, two full bridges with IGBT loss and thermal models, DC link, current-source load, the whole control |
| `plotting_results_sim_results.m` | post-processing of the logged results (reads `param.*`) |
| `init_model_legacy.m` | the old flat init (shared template with seven clock domains), kept for the two companion models below, which still read the old names |
| `single_phase_afe_ideal.slx`, `dcdc_buffer.slx` | companion study models on the legacy names (`single_phase_afe_ideal` also reads names the legacy init never defined: `Vdc_nom`, `Vac_FS`, `Xbez`, `adc11_quantization`, `Ld_fht1`, ...) |
| `test_thd.slx` | test of the online THD MATLAB Function; reads `glb_time.ts_afe` and a `grid_current` signal from the workspace |
| `spectrum.m`, `power_losses_calculation.m`, `plant_estimator.m`, `virtual_phase_shift_filter_analysis.m` | older scripts, left as they were (the first two reference legacy variables that neither init defines) |
| `claude_analysis/` | dimensioning check of October 2026: Python script, its printed output and the figures (PDF and PNG) |
| `figures/` | EPS figures of the post-processing (git-ignored) |

## The model

**Power stage.** The grid emulator gives the three phase voltages of a 690 V, 50 Hz grid (ideal
sources, no impedance). The AFE is a full bridge connected between phases *a* and *b* through an
LCL filter (250 µH + 200 µF with 50 mΩ + 250 µH, resonance 1007 Hz). The DC link is 2.4 mF
(`hwdata.afe.CFi/3` of the legacy data set) and feeds a current-source load that ramps to 275 kW
in 0.25 s. The ripple port is a second full bridge whose DC side is the buffer capacitor (3.6 mF,
2000 V reference) and whose AC side is tied to the DC-link rails through two 250 µH inductors:
seen from the DC link it is a boost stage, so the buffer voltage must stay above `udc`. Both
bridges use the Infineon FF1200R17IP5 (1700 V) with the loss and thermal model of the library
(`use_thermal_model = 1`).

**Control** (all at `ts = 100 µs`, trigger at the bottom and at the top of the carrier):

- FHT-PLL on the three grid voltages (phase θ);
- DC-link voltage PI (2/35, limit ±1.65 pu, reference step at 1 ms and 20 ms low-pass) → amplitude
  of the grid current, in phase with the voltage;
- grid current: the measured current is α, the virtual β is built by a first harmonic tracker
  (variants: SOGI or phase-shift filter, Simulink or C-Caller), αβ→dq at θ, dq vector PI (0.5/45,
  limit 1 with the vector clamp, output divided by `udc_pu`), dq→αβ, unipolar PWM;
- buffer voltage PI (2/18, ±1, 20 ms reference low-pass) → DC component of the buffer current
  reference;
- ripple loop: double-integrator observer of `udc_pu` (a wide band-pass around 100 Hz, output
  `v_hat/omega_base`) → phase-shift filter at 2·f (+90°, sign changed) gives the quadrature signal →
  αβ→dq at 2θ → vector PI (1/18, limit 1, enabled at 0.5 s) → dq→αβ at 2θ, **β output** → 100 Hz
  component of the buffer current reference. The β output lags the detected vector by 90° and
  compensates the integrator of the DC-link capacitor, so the vector PI sees a plant with almost
  no phase (−9° in total);
- buffer current PI (1/18, ±1, base 750 A) with the feed-forward `udc_pu`; the sign of the loop is
  inverted (Gain −1 on reference and measurement) because the plant gain from the duty to the
  current is negative; output = duty of the ripple-port bridge.

## How to run

1. Add the `library` repository to the MATLAB path with subfolders (blocks, C-Caller sources,
   device data sets).
2. From this folder run `init_model.m`: it fills `param`, prints the list of the local overrides of
   `private/`, opens `single_phase_afe.slx`, closes the scopes and checks that every `param.*` name
   read by the model exists (`check_model_param_refs`).
3. Run the simulation (`param.sim.simlength = 1.25` s; the ripple loop is enabled at
   `param.afe.ctrl.apd.t_enable = 0.5` s) and `plotting_results_sim_results.m`.

## Parameter structure

| Group | Content |
|---|---|
| `param.sim` | `simlength`, `tc` (Simscape step 1 µs), `use_thermal_model`, `nonlinear_iterations`, `check_param_refs` |
| `param.time`, `param.glb_time` | PWM frequency, trigger, dead time, `tc_factor`; `glb_time = timing_param(param.time)` gives `ts_afe`, `fPWM_AFE`, `TRGO_AFE_double_update`, `dead_time_AFE`, `tc`, `Ns_afe`, ... (the names the blocks read) |
| `param.grid` | application voltage, `f_nom`, `omega_nom`, `pwr_nom`, `ucc`; `emu = grid_emulator_param(...)` (bases `u2bez`, `i2bez`, references, `sof_50Hz` state space); `frt = grid_fault_param(...)` (fault of the legacy test, both enables at 0) |
| `param.afe.hw` | `afe_hw_param(690, 225e3, fpwm)`: the legacy single-phase data set (bases `ubez` 838.4 V, `ibez` 536.8 A, `udc_nom` 1239.4 V, `CFi`, LCL values) plus the grid figures (`u_pu` 1.164, `i_pu`, `m_nom`, `f_res`, ...) |
| `param.afe.buffer` | ripple port: `C`, `L`, `RL`, `L_tot`, `udc_ref` 2000 V, `ibez` 750 A, `udc_pu_ref` (these were hard-coded in the blocks) |
| `param.afe.pwm` | `fpwm`, `double_update`, `dead_time`, `tc` |
| `param.afe.ctrl` | `afe_ctrl_param(ts, omega)`: all the gains (udc loop, current loop, FHT, SOGI, PLL, `di_obs` observer gains `ktheta`/`komega`, `apd` ripple loop, `buffer` loops) |
| `param.afe.variant` | `current_ctrl_ccaller`, `beta_fht` (1), `beta_sogi`, `beta_phsh`, `beta_sogi_ccaller`, `beta_phsh_ccaller` (exactly one β variant active) |
| `param.afe.design` | informative figures: power pulsation, DC-link ripple, buffer energy and swing, switching ripples, static loop estimates |
| `param.afe.igbt`, `param.dev` | IGBT data set (`device_igbt_param`, FF1200R17IP5 embedded, other names through `device_data_load`) |
| `param.meas` | ADC full scales and quantization |
| `param.thermal` | heatsink object read by the bridge masks, ambient (water) temperature, loss estimate of the grid bridge |
| `param.load` | `pwr` 275 kW, `i_max`, `i_slope`, `R` |
| `param.battery` | `battery_param(...)` for the block `battery_1` (commented out in the model) |

### Functions of `private/`

| Function | Role |
|---|---|
| `timing_param(t)` | clock domain of the two bridges (and the legacy DAB domain of the battery estimator) |
| `afe_hw_param(V, P, fpwm)` | hardware data and per-unit bases of the AFE, LCL and grid figures |
| `afe_ctrl_param(ts, omega)` | every control gain; FHT and observer gains by closed-form pole placement (no toolbox) |
| `grid_emulator_param(...)`, `grid_fault_param(...)` | grid emulator bases and references, fault definition |
| `device_igbt_param(name, fpwm, udc)` | IGBT data set and derived thermal/snubber values |
| `battery_param(...)` | battery pack (copied from `interleaved_buck_converter/private`) |
| `check_model_param_refs(model, param)` | lists the `param.*` names read by the model and reports the missing ones |

## What changed in the model (October 2026)

- 353 block parameters re-pointed from the flat names (`hwdata.*`, `glb_time.*`, `afe_ctrl.*`,
  `grid_emu.*`, `frt_data.*`, `single_phase_inverter_ctrl.*`, `igbt.afe.*`, `heatsink.*`, ...) to
  `param.*`; the variant conditions read `param.afe.variant.*`.
- Configuration set: `StopTime = param.sim.simlength`, `FixedStep = param.sim.tc`,
  `MaxStep = param.glb_time.tc`; Solver Configuration `MaxNonlinIter = param.sim.nonlinear_iterations`.
- Numbers that were typed inside the blocks are now parameters: buffer capacitor and inductors,
  buffer current base (750 A), all the gains of the buffer loops and of the ripple loop, the enable
  time of the ripple loop, the reference filters.
- Bases: the DC-link voltage scaling `Vdc_scale` of the `dclink` block was 1070 V while the control
  works with `udc_nom = 1239.4 V`; it now reads `param.afe.hw.udc_nom`. The grid measurements are
  scaled by `1/param.grid.emu.u2bez` and `1/param.afe.hw.ibez` as before (same values).
- `glb_time.Ns_inv` → `param.glb_time.Ns_afe`.

## Dimensioning check

`claude_analysis/single_phase_afe_design_check.py` (Python 3 with numpy, scipy, matplotlib; run it
from its folder) prints the numbers of `init_model.m`, the margins of the four loops and runs an
averaged simulation of the DC-link side with the discrete control of the model (ideal PLL, no
switching). Output in `single_phase_afe_design_check_output.txt`, figures in `figures/`. Main results:

- 690 V between phases a and b is 1.164 pu of the legacy base `ubez`; 225 kW is 0.86 pu of `ibez`,
  the 275 kW load is 1.05 pu (122 % of the rating); modulation index 0.79.
- Power pulsation at 100 Hz: 182 A into the DC link (222 A at 275 kW), 716 J peak-to-peak; the DC
  link alone would ripple by 9.7 % (11.9 % at 275 kW); the buffer swings 1950–2049 V (5 %).
- Loops: grid current 325 Hz / 70° (LCL resonance at 15 dB of loop gain, gain margin 5 dB),
  DC-link voltage 23 Hz / 79°, buffer current 848 Hz / 44°, buffer voltage 33 Hz / 84°; ripple loop
  envelope pole −4.3 rad/s (90 % of the ripple removed in about 0.4 s).
- Averaged simulation: before the decoupling the 100 Hz ripple of `udc` (16 %) passes through the
  voltage PI (|PI| = 2 at 100 Hz) into the current amplitude and gives 13 % of third harmonic in the
  grid current; with the decoupling the ripple is 0.8 %, the THD 0.5 %, the buffer current 221 A
  peak, the buffer swing 121 V.

## Open points found during the check

1. The ripple-port bridge uses the same 1700 V module at 2000 V (118 % of the blocking voltage):
   either a lower buffer voltage (1.79 mF would give ±5 % at 2000 V, 3.18 mF at 1500 V) or a higher
   voltage class, or a buck-type ripple port below `udc`.
2. The buffer starts at 0 V and both voltage references ramp from 0 while the DC link is already at
   the rectified peak: the first milliseconds are a resonant precharge through the inductors and
   the udc PI asks for an inverting current until the reference passes the measured voltage.
3. The feed-forward of the buffer current loop is `udc_pu` (≈ 1) while the duty of the ripple port
   is `udc/V_buffer` ≈ 0.62: the PI integrator holds the difference.
4. The buffer voltage PI (kp 2) sees the 100 Hz swing of the buffer voltage and adds a 100 Hz
   component to the current reference, which the ripple loop has to absorb.
5. The ripple loop is slow (time constant 0.23 s): a larger `ki` of the ripple vector PI, or an
   explicit feed-forward of the pulsating power, would make it faster.
6. The DC-link voltage PI passes the 100 Hz ripple into the grid current reference; a notch at 100
   Hz or a moving average over 10 ms would help when the ripple loop is off.
7. Dead time 1 µs against td_off + tf = 1.74 µs of the FF1200R17IP5 at 125 °C.
8. Load 275 kW on a 225 kW converter; bases of the legacy data set (838 V) against a 976 V grid
   peak; the PLL reads three phase voltages while the converter is single-phase; the grid has no
   impedance (no line inductance, no transformer).
