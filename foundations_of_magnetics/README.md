# Foundations of Magnetics

Magnetics for power electronics: analytical design, finite-element analysis with FEMM,
insulation limits, and an electromechanical actuator.

## Contents

### `femm/inductor_1`
- Axisymmetric magnetostatic inductor built with FEMM: ferrite core of 24 mm radius, 5-turn
  copper winding of 2 mm thickness over 120 mm height, 1000 A series circuit, open boundary on a
  200 mm air region.
- `inductor_1.lua` generates the geometry and materials, `inductor_1.FEM` is the problem
  definition (solved case at 12 kHz) and `inductor_1.ans` the solved field.

### `literature/high_frequency_high_power_transformers`
- Reference material for HF transformer design: papers on winding modelling, eddy-current
  effects, optimum design of high-power high-frequency transformers, and MnZn ferrite core loss
  estimation.
- `note_design_hf_inductors_transformers.txt` is a twelve-step design recipe (from L, currents
  and switching frequency to core choice, peak flux density from the loss target, current
  density, area product, wire sizing, turns, gap, core and winding losses, iterate) with a
  worked example at 300 kHz.
- `materiali/` holds the Ferroxcube/TDK material and core datasheets used by the recipe.

### `moving_coil_foundation`
- Third-order state-space model of a moving-coil (voice coil) actuator: position, speed and
  current, with force and back-EMF factor, coil inductance and resistance.
- The model is augmented to a fourth state that estimates the load force; controllability and
  observability are checked for both versions.
- Observer gains placed with `place`, state feedback with integral action designed with `acker`;
  the plotting script shows position tracking under load and the estimated versus actual
  position, speed and current.

### Loose scripts and documents
- `paschen_air.m` / `paschen_air_en.m` (Italian and English) — Paschen breakdown curve for dry
  air at atmospheric pressure, gap swept from 1 µm to 10 cm. The script locates the Paschen
  minimum and compares the curve with the usual 3 kV/mm linear approximation.
- `insulation_strength.pdf`, `insulation_strength_2.pdf`, `em_boundary_conditions.jpg` —
  supporting material on insulation coordination and electromagnetic boundary conditions.
