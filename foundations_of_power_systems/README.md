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
- General-purpose grid + converters model, 690 V / 1 MVA emulator with explicit grid impedance,
  AFE, inverter, DAB and CLLC stages, two 250 kW lithium-ion battery models and a 225 kVA
  single-phase load transformer with R-L load.
- Control sections cover PMSM with EKF observer, induction motor control, AFE sequence control,
  DC-DC control, resonant PI, SOGI and virtual-dq single-phase inverter control.
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
