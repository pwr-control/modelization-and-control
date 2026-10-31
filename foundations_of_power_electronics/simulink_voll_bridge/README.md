# simulink_voll_bridge

Signal-flow model (pure Simulink, no Simscape) of a DC/DC converter with two full bridges and a
HF transformer, made by Tobias (`voll_bridge_tobias.slx`, R2026b).

Status 2026-10-31: power stage only (ideal bridges, T-model transformer, input LC filter, output
capacitor, load profile). Modulator and control are missing, the parameters are expected in the base
workspace (no `init_model.m` yet), and the load selector depends on the external `SDE_Blockset`
library. The model does not run as it is.

- `voll_bridge_tobias.slx` — the model as received.
- `revisione_voll_bridge_tobias.md` — review notes (Italian): structure, verified equations, errors
  found, sizing checks, open questions for Tobias, and the outlook of a library of pure-Simulink
  power-electronics models.
