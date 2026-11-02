#!/usr/bin/env python3
"""Static characteristic of the DAB in the switched simulation: fixed controller output u (open loop), steady
state after 40 ms, averages over the last 10 ms (120 PWM periods). Compared with the harmonic model of
dab_harmonic_op.m (square waves, no dead time) at the same DC-link voltages. Three runs: the model (tc 0.625 us,
dead time 2 us), tc = T/400 = 0.208 us (dead time 2 us), and dead time 0.5 us with tc = T/1200 = 69 ns.

Usage: python3 dab_sweep.py      (writes sweep.npz next to this script and prints a table)
"""
import os
import numpy as np
import dab_sim as ds

HERE = os.path.dirname(os.path.abspath(__file__))
FPWM = 12e3


def harmonic_op(V1, V2, phi, p, nh=999, npts=2000):
    """python copy of private/dab_harmonic_op.m (n = 1); conduction resistance of the bridges included"""
    w = 2*np.pi*FPWM
    R1 = p['Rd1'] + 2*p['Ron_sw']
    R2 = p['Rd2'] + 2*p['Ron_sw']
    th = np.arange(npts)/npts*2*np.pi
    i1 = np.zeros(npts)
    P1 = P2 = 0.0
    i10 = 0.0
    i2phi = 0.0
    for hh in range(1, nh + 1, 2):
        Z1 = R1 + 1j*hh*w*p['Ld1'] + 1/(1j*hh*w*p['Cs1'])
        Z2 = R2 + 1j*hh*w*p['Ld2'] + 1/(1j*hh*w*p['Cs2'])
        Zm = 1/(1/(1j*hh*w*p['Lm']) + 1/p['Rfe'])
        U1 = 4*V1/(hh*np.pi)
        U2 = 4*V2/(hh*np.pi)*np.exp(-1j*hh*phi)
        Um = (U1/Z1 + U2/Z2)/(1/Z1 + 1/Z2 + 1/Zm)
        I1 = (U1 - Um)/Z1
        I2 = (Um - U2)/Z2
        i1 += np.imag(I1*np.exp(1j*hh*th))
        P1 += np.real(U1*np.conj(I1))/2
        P2 += np.real(U2*np.conj(I2))/2
        i10 += np.imag(I1)
        i2phi += np.imag(I2*np.exp(1j*hh*phi))
    return dict(P1=P1, P2=P2, I1rms=np.sqrt(np.mean(i1**2)), i_sw1=-i10, i_sw2=i2phi)


def run_point(u, p):
    tend = 0.04
    nsteps = int(round(tend/p['h']))
    el = ds.build(p)
    iL0, vC0 = ds.initial_state(el, p, ds.ocv(p, p['soc0']))
    win_start = int(round(0.03/p['h']))
    win_len = int(round(0.01/p['h']))
    opt = np.array([1.0, u, 0.0])
    rec, win, *_ = ds.simulate(el['R'], el['L'], el['C'], el['S'], el['D'], el['T'], ds.prm_vector(p), iL0, vC0,
                               nsteps, np.array([win_start], dtype=np.int64), win_len,
                               np.array([0.0]), np.array([0.0]), opt)
    w = win[0]
    nper = int(round(1/FPWM/p['h']))
    npp = nper*3 if abs(3/FPWM/p['h'] - round(3/FPWM/p['h'])) < 1e-6 else nper
    n = (len(w)//npp)*npp
    w = w[:n]
    return dict(u=u, vdc1=w[:, 8].mean(), vdc2=w[:, 9].mean(), iL1=w[:, 10].mean(), iL2=w[:, 11].mean(),
                Pdc1=np.mean(w[:, 8]*w[:, 12]), Pdc2=np.mean(w[:, 9]*w[:, 13]),
                I1rms=np.sqrt(np.mean(w[:, 3]**2)), I1pk=np.abs(w[:, 3]).max(), ripple=np.ptp(w[:, 10]))


if __name__ == '__main__':
    us = np.r_[0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.08, 0.10, 0.12, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5, 0.6, 0.7,
               0.8, 0.9, 1.0]
    out = {}
    # label, solver step, dead time: the model (0.625 us, 2 us), a finer step, and a shorter dead time at a step
    # small enough to resolve it (0.5 us = 7.2 steps of T/1200)
    for label, h, dt in (('tc625', 0.625e-6, 2e-6), ('tc208', 1/FPWM/400, 2e-6), ('dt05', 1/FPWM/1200, 0.5e-6)):
        p = dict(ds.P)
        p['h'] = h
        p['dead_time'] = dt
        rows = []
        for u in us:
            r = run_point(u, p)
            hm = harmonic_op(r['vdc1'], r['vdc2'], u*p['phi_max'], p)
            r['P1_harm'] = hm['P1']
            r['P2_harm'] = hm['P2']
            r['I1rms_harm'] = hm['I1rms']
            rows.append(r)
            print(f"{label} u {u:.3f} (phi {np.degrees(u*p['phi_max']):5.1f} deg): vdc1 {r['vdc1']:7.1f} vdc2 {r['vdc2']:7.1f} "
                  f"iL1 {r['iL1']:7.2f} A ({r['iL1']/250:.4f} pu) P_dc1 {r['Pdc1']/1e3:7.2f} P_dc2 {r['Pdc2']/1e3:7.2f} kW "
                  f"loss {(r['Pdc1'] - r['Pdc2'])/1e3:6.2f} kW | harmonic P1 {hm['P1']/1e3:7.2f} P2 {hm['P2']/1e3:7.2f} kW "
                  f"loss {(hm['P1'] - hm['P2'])/1e3:5.2f} | I1rms {r['I1rms']:6.1f} (harm {hm['I1rms']:6.1f}) A, "
                  f"iL1 ripple p-p {r['ripple']:.2f} A", flush=True)
        out[label] = rows
    np.savez(os.path.join(HERE, 'sweep.npz'), us=us,
             **{f'{lab}_{k}': np.array([r[k] for r in rows]) for lab, rows in out.items() for k in rows[0]})
