preamble;


tc = 1e-7;

Ton_base = 0.333e-6;
dT_base = 0.4e-3;
N = 40;

vin_cmd = 15;
vout_cmd = 15;

simlength = dT_base + N*dT_base;

Cdt = 820e-12;
Cmp = 270e-12;
Rdt = 3.3e3;
Rmp = 3.3e3;

tau_dt = Rdt*Cdt;
tau_mp = Rmp*Cmp;

open_system dtmp_analysis.slx;