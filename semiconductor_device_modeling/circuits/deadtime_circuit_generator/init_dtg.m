preamble;


tc = 1e-7;

Ton_base = 1e-6;
dT_base = 0.1e-3;
N = 10;

vin_cmd = 15;
vout_cmd = 15;

simlength = dT_base + N*dT_base;

open_system dtg_w.slx;