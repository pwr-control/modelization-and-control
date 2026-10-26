# Foundations of Power Systems

Grid-side studies: how the converters of this repository behave against the network, how the
quantities exchanged with the grid are measured, and the basic theory behind network analysis.

## Contents

### `power_meter_iec`
- Implementation of three-phase power measurement according to VDE/IEC.
- `three_phase_power_meter_iec.mdl` is the reference Simulink subsystem: sin/cos projection of
  the six voltage and current signals, a bank of dynamic moving-average filters over a half grid
  period, and Fortescue decomposition into positive and negative sequence.
- Outputs: positive- and negative-sequence active and reactive power, line-to-line RMS voltages,
  and the active/reactive sequence currents.
- `matlab_function/` rewrites the whole subsystem as a single code-generation-ready MATLAB
  Function (`%#codegen`), with a circular-buffer moving average and a divide-by-zero guard the
  original lacked; `create_power_meter_bus.m` builds the matching `Simulink.Bus`.
- `power_measurements/` is the test bench: the power meter inside a full multi-converter system
  (690 V, 1 MVA Dyn11 emulator, AFE/inverter/DAB/CLLC libraries), including the ADC quantization
  model (12 and 16 bit).

### `power_system`
- Grid model alone, as a test bench for the network side: 690 V / 1 MVA grid emulator (library
  block `three_phase_voltage_source_with_frt_emulator`: ideal source with FRT dip generator and
  second-order 50 Hz filters) behind an explicit equivalent grid impedance, a Dyn11 transformer
  built from three masked `saturable_single_phase_transformer_2` blocks (ph1 across Ua–Ub, ph2
  Ub–Uc, ph3 Uc–Ua, secondary star with grounded neutral, `enable_sat` switchable), IEC power
  meters on both sides and the moving-average analysis filters. The three-phase R-L load of the root
  is commented out; no converter is in this model (the converter benches are in
  `foundations_of_power_electronics` and `weak_grid_analysis`).
- Local parametrization (2026-10-26): `init_model.m` builds the single struct `param`
  (`param.sim`, `param.time`, `param.glb_time`, `param.grid` with `param.grid.emu`, `.trafo`,
  `.frt`, `param.load`) with the functions of `private/` — `timing_param` (clock domain and
  logging base `tc`), `three_phase_transformer_param` (nameplate → coil data: for the delta coil
  `Rfe1` and the saturation flux `psi` are referred to the coil voltage, i.e. the line voltage,
  where the library setup used the phase voltage), `grid_emulator_param` (bases, pu references,
  state space of the 50 Hz filters), `grid_fault_param` (the grid-code test catalogue of the old
  `grid_fault_generator`, two-phase dips through the VDE/FGW asymmetric construction), and
  `check_model_param_refs` which lists every workspace reference of the model that is not a
  `param.*` field. The library is on the path only for its blocks and C sources. The previous
  init is kept as `init_model_legacy.m`: it no longer runs against the current library
  (`timing_setup` has 25 arguments, `grid_fault_generator` became a function) and its numbers
  are now all in `init_model.m`.
- `check_sequences.m` is a standalone numerical check of the symmetrical-component extraction:
  it builds an unbalanced set, computes the Clarke components and the positive/negative sequence
  by ±θ rotation, and reconstructs the phase voltages from prescribed dip components.

### `weak_grid_analysis`
- Behaviour of n parallel AFE modules on a weak grid: the emulator transformer short-circuit
  voltage is raised to 100 %, i.e. a grid impedance of the order of the rated impedance
  (short-circuit ratio ≈ 1).
- Two modules enabled out of four available, with slightly detuned time bases to emulate clock
  mismatch between the modules.
- 6 MW grid, 690 V, 250 kW converters, fPWM = 4 kHz; FRT settings, DC-link stray inductance,
  battery models and a single-phase load transformer are part of the setup.

### `fundamentals/power_system_1`
- Base grid-connected AFE + inverter + motor model: 690 V / 50 Hz behind a 1600 kVA Dyn
  transformer, DC link 1070 V, fPWM = 4 kHz, LCL filter, DC-link brake.
- Control: DDSRF PLL, double-integrator observer, resonant PI and dq PI current control,
  DC-link loop, RMS and online sequence calculation, MTPA / field weakening / Luenberger with
  load estimator on the motor side.
- `grid_fault_generator.m` is the grid-code test catalogue (75 %, 50 %, 25 %, 20 %, 12 % dips
  and 10 %/15 % swells, three-phase and two-phase, full and partial load, reduced dynamic grid
  support); `asymmetric_error.m` returns residual magnitude and phase shift for the VDE/FGW
  asymmetric variants C and D.

### `fundamentals/network_systems/fb_graph_theory`
- Teaching script following Bullo, *Lectures on Network Systems* (graphs and matrices,
  averaging), on a 5-node directed graph.
- Builds adjacency, out-degree, Laplacian and incidence matrices and verifies the standard
  identities (`L·1 = 0`, balancedness, `L = B·Bᵀ`); checks spanning-tree existence through the
  rank of the Laplacian and builds the BFS spanning tree explicitly.
- Closes by integrating the consensus dynamics `ẋ = −L x` on the symmetrized tree Laplacian.
