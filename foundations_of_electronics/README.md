# Foundations of Electronics

Analog circuits of a power converter control and driver board, modelled in Simscape Electrical
at component level: gate drivers, measurement chains, comparators, level shifters, deadtime
generators and auxiliary supplies. Several circuits use transistor-level operational amplifiers
(`opa_raw_*`) instead of ideal blocks, so the behaviour of the real silicon can be observed.

Where a circuit reproduces an existing board, the real reference designators and component
values are kept.

The folder holds two sets: `analog_circuits_analysis/` is the older collection,
`circuits/` the more recent one. Three of the `circuits/` subfolders (`gate_driver`,
`opamp_analysis`, `gate_driver/rate_limiter_with_bjt`) are copies of the corresponding
`analog_circuits_analysis/` folders.

---

## `analog_circuits_analysis`

### `advanced_active_clamping`
- Active clamping of an IGBT half-bridge (FF650R17IE4) under a double pulse test: a TVS stack
  feeds back to the gate to limit the turn-off overvoltage.
- Gate drivers built from discrete N/P-channel MOSFETs and NPN/PNP transistors; explicit device
  capacitances, gate resistances and 7 nH module stray inductance, 1400 V DC link.

### `gate_driver`
- IGBT gate driver in a double pulse test (FF900R12IE4, 750 V DC link, +15 / −10 V gate supply,
  split turn-on and turn-off gate resistors).
- Device modelled from the datasheet capacitances (Cies, Cres, Coes), 10 nH stray inductances,
  TVS clamp; variant sources switch between driver output stages.
- `plotting_results.m` produces the turn-on and turn-off zooms of Vge, Vce and emitter current
  for dv/dt and di/dt inspection.
- Note: `init_model.m` opens `gate_driver_circuit_ver_ii.slx`, which is not in the folder — use
  the copy in `circuits/gate_driver`, which opens the model that is present.

### `psu_driverboard`
- Isolated auxiliary supply for the driver board, built around a transformer with modelled
  winding stray capacitance and a Zener reference, in two variants (with and without the
  resonant capacitor).
- `init_psu.mlx` is a planar transformer design study: three windings, ≥ 2 kV isolation,
  1.2 MHz switching (UCC25800-Q1), skin depth, candidate ferrite cores, PCB stack-up and turns.

### `pll`
- Discrete CMOS phase-locked loop for line synchronization (CD4046-style): XOR phase comparator,
  charge pump and loop filter, voltage-controlled oscillator, 2N7000 drive stages.
- `init_pll.mlx` and `configuration_CD5056.mlx` size the VCO and the loop filter and Bode-plot
  the loop transfer function; `line_effects.slx` adds cable, termination and coupled noise to
  study their effect on lock.

### `analog_current_measure`
- Complete phase-current measurement chain from the LEM transducer to the ADC: burden resistors
  on driver board and main board, measurement amplifier, fault detector.
- Compares the current-mode and voltage-mode versions of the measurement and shows the
  overcurrent detector response.

### `voltage_measure`
- Differential voltage measurement front end for the AC and DC-link voltages (MCP6D11), with the
  resistor ladder and anti-aliasing filters, up to the ADC input.
- `voltage_to_current_signal.slx` covers the 400 V and 690 V DC-link and phase-voltage sensing
  with a pre-amplifier and a voltage-to-current converter for the loop signal.

### `ntc_measure_temperature`
- NTC temperature measurement between driver board and main board, comparing three solutions:
  the existing one, a modified driver-board version, and one with an op-amp current source
  (OPA350).
- The scripts compare driver-board output current and conditioned output voltage of the
  alternatives and export the figures.
- Note: the init script opens `ntc_measure_with_modifications.slx`, which is not in the folder.

### `comparator`
- Two comparator implementations built from discrete transistors and transistor-level op-amp
  cores (with and without output clamping), compared on the same input.

### `trigger_smith`
- Schmitt trigger in the input stage of an STO (safe torque off) driver board, with optocoupler
  and the actual board component values; the scopes show the hysteresis thresholds along the
  chain.

### `shift_level_signal_management`
- Four level-shifter variants between the 5 V and 15 V domains, built from MOSFETs, diodes and
  pull-ups, compared on the same input.

### `rate_limiter_with_bjt`
- Slew-rate limiter built from a complementary BJT pair, simulated with a 1 ns step.

### `opamp_analysis`
- Same four-resistor test bench run twice: once with the ideal op-amp block and once with a
  transistor-level op-amp (`opa_raw`), so the two can be compared (1 kHz, ±15 V rails).

### `voltage_to_current_converters`
- Voltage-to-current converters for 35 mA and 400 mA output, comparing two LT1210 power-op-amp
  implementations against transistor-level BJT and MOS cores.

### `bjt_circuits`
- Single NPN stage with its supply rails and sensors — a basic BJT characterization bench.

---

## `circuits`

### `analog_circuits`
- The current board-level measurement set, built on TI parts: AMC3330 isolated amplifier with a
  THS4531 fully-differential op-amp for DC-link and phase voltage, phase current on a 5.6 mΩ
  shunt, and two NTC conditioning variants.
- `init.m` sets the measured ranges (660 V DC link, 230 V AC) and Bode-plots the two
  anti-aliasing filters.
- `comparison.slx` is the DC-link overvoltage comparator with its thresholds, built from
  transistor-level op-amps on 3.3 V and 5 V rails.
- `sto_trigger_smith.slx` and its reduced version cover the STO Schmitt-trigger chain of two
  driver boards.
- The plotting scripts export the measurement-chain figures for the phase and DC cases.

### `deadtime_circuit_generator`
- Deadtime and minimum-pulse generator: the RC/discrete implementation is compared with three
  state-machine versions driven by the same gate commands.
- The RC time constants (820 pF / 3.3 kΩ for the deadtime, 270 pF / 3.3 kΩ for the minimum
  pulse) are set in the init script; the scopes overlay the generated commands. Useful as the
  reference behaviour for a CMOS or Verilog implementation.

### `gate_driver`
- The IGBT gate-driver double pulse test described above, in the version that opens the model
  present in the folder.

### `gate_driver/psu`
- Isolated gate-driver supply at 510 kHz: transformer design from the gate charge requirement,
  two LC output filter stages Bode-plotted, and a type-II compensator checked with `margin()`.
- `data_logged/plotting_trace_yokogawa.mlx` post-processes a measured Yokogawa trace of the
  primary current (offset removal, RMS) and compares it with the simulation.

### `gate_driver/rate_limiter_with_bjt`
- Copy of the BJT slew-rate limiter.

### `opamp_analysis`
- Copy of the ideal-vs-transistor-level op-amp bench, plus `op_amp/opamp_foudation_1.slx`, the
  library of transistor-level op-amp topologies (BJT and MOS cores, with and without bias
  current and output clamp) used by the other circuits.

### `current_source`
- Contains only `init.m` (a generic analog test bench: 4 kHz input, ±15 V rails); the
  corresponding model is missing.
