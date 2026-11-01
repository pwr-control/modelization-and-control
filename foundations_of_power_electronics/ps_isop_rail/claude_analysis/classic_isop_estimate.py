#!/usr/bin/env python3
"""Averaged steady state of the four modules (lossless PSFB model, common phase shift), for comparison:
  A) classic ISOP: one rectifier per module, outputs in parallel on the DC side
     module k: v_o = n*D*v_k - Req_k*i_k, input current i_in,k = v_o*i_k/v_k (power balance)
  The string current is common, so all i_in,k are equal; sum(v_k) = string voltage; sum(i_k) = v_o/Rload.
  B) the model (pairs with AC-paralleled secondaries): from the switched simulation (isop_sim.py, mm_* cases).
Unknowns: v_1..v_4, i_1..i_4, D (v_o fixed at 750 V by the voltage PI)."""
import numpy as np
from scipy.optimize import fsolve

Vs = 4683.6          # string voltage at the operating point (4 x 1170.9 V, switched simulation)
vo = 750.0
Rl = 0.5
n = 1.0
Req0 = 4*n**2*26.04e-6*4e3       # 0.417 Ohm


def solve(req, nn):
    def f(x):
        v = x[0:4]; i = x[4:8]; D = x[8]
        r = []
        for k in range(4):
            r.append(nn[k]*D*v[k] - req[k]*i[k] - vo)
        iin = [vo*i[k]/v[k] for k in range(4)]
        r += [iin[0] - iin[1], iin[1] - iin[2], iin[2] - iin[3]]
        r.append(sum(v) - Vs)
        r.append(sum(i) - vo/Rl)
        return r
    x0 = np.r_[np.full(4, Vs/4), np.full(4, vo/Rl/4), 0.78]
    x = fsolve(f, x0)
    return x


for name, req, nn in [('equal modules', [Req0]*4, [n]*4),
                      ('module 1 turns +2 %', [Req0]*4, [1.02*n, n, n, n]),
                      ('module 2 leakage +5 %', [Req0, 1.05*Req0, Req0, Req0], [n]*4),
                      ('module 2 leakage +20 %', [Req0, 1.2*Req0, Req0, Req0], [n]*4)]:
    x = solve(req, nn)
    print(f'{name:24s}: v = {", ".join(f"{v:7.1f}" for v in x[0:4])} V | i_out = {", ".join(f"{i:6.1f}" for i in x[4:8])} A | D = {x[8]:.4f}')

# first-order time constant of the input voltage sharing (classic ISOP, also the mode between the pairs of the model)
v = Vs/4; i = vo/Rl/4; D = (vo + Req0*i)/(n*v)
g = vo/v*(n*D/Req0 - i/v)
print(f'restoring conductance g = {g:.3f} S, tau = Cin/g = {1.2e-3/g*1e3:.2f} ms (first order, output inductors left out)')
