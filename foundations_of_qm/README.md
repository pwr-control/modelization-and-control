# Foundations of Quantum Mechanics

Numerical solution of the time-independent Schrödinger equation, kept as the physical background
of the semiconductor device models in `foundations_of_semiconductor_devices`.

## Contents

### `buca_di_potenziale.m`
- One-dimensional Schrödinger equation in normalized units (ħ = m = 1) on a 1000-point grid
  over x ∈ [−5, 5].
- The potential is selectable between a finite square well and a harmonic oscillator (the
  harmonic case is the one currently active).
- The kinetic operator is discretized as a sparse tridiagonal Laplacian; the five lowest
  eigenpairs are computed with `eigs` and the eigenfunctions normalized by numerical integration.
- Output: the potential with the energy levels drawn as dashed lines and each normalized
  eigenfunction offset on its own level; the eigenvalues are printed.
