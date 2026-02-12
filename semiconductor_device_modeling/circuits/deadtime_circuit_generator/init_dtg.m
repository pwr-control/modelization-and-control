preamble;

simlength = 1e-3;

tc = 1e-7;

dt = 0.1e-3;

vin_cmd = 12;
t1 = dt;
Ton1 = 2e-6;
t2 = t1 + dt;
Ton2 = 4e-6;
t3 = t2 + dt;
Ton3 = 6e-6;
t4 = t3 + dt;
Ton4 = 8e-6;
t5 = t4 + dt;
Ton5 = 10e-6;

open_system dtg.slx;