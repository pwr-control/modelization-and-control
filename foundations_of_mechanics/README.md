# Foundations of Mechanics

Mechanical problems taken from literature and from applications, modelled numerically and, where
relevant, controlled. Two families: **distributed systems** (beams, strings, thermal bars)
discretized with finite elements or finite differences, and **lumped drivetrains** (two- and
three-inertia torsional models) designed in state space.

## Contents

### `beam_string`
- Euler-Bernoulli beam under axial tension (a "beam-string": a tensioned steel wire, 0.8 m,
  ⌀2 mm, pinned-pinned).
- 1D finite elements with Hermite shape functions, bending plus geometric stiffness, consistent
  mass, 200 elements; modal analysis compared against the closed-form frequencies, Rayleigh
  damping, Newmark time integration.
- The tension is auto-tuned so that one mode sits exactly at 2 kHz; the script produces the
  FEM-versus-analytic frequency table, the frequency response around 2 kHz with its −3 dB
  bandwidth, the operating deflection shape and the resonant build-up in time.
- `doc/` holds the accompanying report (Italian) with its compiled PDF.

### `beam_string_analysis`
- Working copy of the same case: the script is identical to the one in `beam_string`, and `doc/`
  holds only the LaTeX source.

### `flexible_shaft/msm`
- Two-inertia torsional drivetrain (mass-spring-mass): motor and load inertia joined by a shaft
  stiffness, viscous friction on both sides, with the analytic resonance derived in the script.
- State-space model in motor speed, load speed and shaft torque, sampled at 1 ms, with LQR state
  feedback.
- Outputs: pulse response of the two speeds and of the shaft and motor torque, open loop and
  with state feedback, plus the FFT of the motor speed.

### `flexible_shaft/msmsm`
- The same study extended to three inertias and two torsional springs, giving a five-state model
  with two resonances; LQR state feedback and the same comparison plots.

### `flexible_shaft_ctrl`
- Application of the two-inertia model to a ropeway drive. The parameter scripts derive the
  inertias from line length, rope mass per metre, number and mass of vehicles, pulley radius and
  rope Young modulus, for two plants (5474 m with 61 vehicles, 324 kNm; 950 m with 35 vehicles,
  195 kNm).
- Per-unit state-space model with load-torque disturbance, on a PWM-rate time base
  (fPWM = 3.1 kHz, slow rate eight times slower), first-order converter model and reference
  filters discretized with `c2d`.
- Control: PI, or PI with integral state feedback, gains from LQR (pole placement with `acker`
  at multiples of the plant resonance is kept as an alternative), plus a four-state observer that
  augments the state with a load-torque estimator.
- The init script imports the external C struct types of the controller, so the same code runs
  in simulation and on the target.
- Outputs: reference and actual speeds with shaft and disturbance torque for the PI and the
  PI + state feedback cases, the free response from a non-equilibrium initial condition, and the
  open-loop torque pulse response.

### `pde_ode_ctrl/pde_ode_ctrl_string_resonance/case_scenario_1`
- Taut string wave equation (c = 40 m/s, 1 m) coupled to an ODE harmonic oscillator acting as a
  point force at a chosen node; a damped version adds viscous damping.
- Explicit finite differences with CFL-limited step, oscillator integrated with semi-implicit
  (symplectic) Euler.
- Open loop: the study is forced resonance — a tuning factor detunes the oscillator from the
  string eigenfrequency and a mode selector excites a chosen mode. The scripts animate the string
  and one of them records the animation to video.

### `pde_ode_ctrl/pde_ode_ctrl_string_resonance/case_scenario_2`
- Active damping of the axial vibration of a clamped-clamped steel bar: 50 linear finite
  elements, 49 degrees of freedom, modal analysis, Rayleigh damping at 0.5 % on the first two
  modes, lifted to a 98-state state-space model.
- A harmonic disturbance is applied at the second-mode frequency at L/4; the control force acts
  at L/2. LQR state feedback is switched on at t = 1 s.
- The workflow is a MATLAB/Simulink co-simulation: `fem1d_init.m` assembles the model,
  `fem1d_build_model.m` builds the Simulink model programmatically around the FEM core, and
  `fem1d_postprocess.m` produces the time response, the control force, the kinetic energy, the
  spectra before and after control, the mode shapes with actuator and disturbance placement, and
  the RMS attenuation in dB.
- `README_fem1d.md` documents the case, including the caveat that L/2 is a node of mode 2 so the
  controller acts mainly on mode 1.

### `pde_ode_ctrl/pde_ode_ctrl_thermal_diffusion`
- Coupled ODE + PDE heat problem: a lumped heater at constant power acts as a dynamic boundary
  condition on a 1D bar with an insulated far end.
- Explicit FTCS finite differences for the bar, explicit Euler for the heater; the heater power
  is an open-loop input (no feedback control despite the folder name).
- Output: animated temperature profile with the heater temperature.

### `thermal_diffusion`
- 1D heat diffusion in a 3 × 30 × 50 mm aluminium bar: imposed heat flux on one end (25 W),
  fixed sink temperature on the other.
- Explicit FTCS with 200 nodes and a CFL safety factor, Neumann boundary handled with a ghost
  node; the result is verified against the analytic linear steady-state profile and the maximum
  error is printed.
- Outputs: transient profile overlaid on the steady-state solution and the time histories at the
  heated end and at mid-length. Available in Italian and English versions, plus a copy of the
  coupled heater script retuned to this bar.
