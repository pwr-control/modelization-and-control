# Foundations of Mathematics

Numerical experiments on the mathematics behind the models of this repository.

## Contents

### `lorenz_chaos.m`
- Lorenz system (σ = 10, ρ = 28, β = 8/3) integrated with `ode45` at tight tolerance for three
  initial conditions separated by 1e-8 and 2e-8, over 50 s.
- Demonstrates sensitive dependence on initial conditions: the three trajectories are shown as a
  time-coloured 3D attractor and as X-Y and X-Z projections.
- The divergence of the trajectories is plotted on a semilog scale and fitted over the first
  15 s to estimate the largest Lyapunov exponent, printed and compared with the literature
  value.
