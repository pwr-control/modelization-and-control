#!/usr/bin/env python3
"""Small-signal analysis of the input-current loop of single_phase_dab.slx (design check).

Plant: u -> phase phi = u*phi_max -> DAB DC currents i_dab1 = f1(phi, V1, V2), i_dab2 = f2(phi, V1, V2)
(partial derivatives of the harmonic model at the operating point; the phase gain can be replaced by the
effective gain of the switched simulation, dab_sweep.py) -> DC networks of both sides (battery cell model
R0 + R1 || C1, 1 mF terminal capacitor, LFi_dc 250 uH + 1.96 mOhm, CFi_dc 3.6 mF + 1 mOhm) -> iL(LFi_dc1).
Sampling: ui_ctrl at ts = 41.7 us, quantizer neglected, unit delay on the feedback, pi_ctrl (kp 0.5, ki 18,
y = kp e + ki ts e + xi), feed-forward k_ff * reference, ZOH, modulator delay Tm = T/4 + phi/(2 pi) T (new
phase used at the next edges of bridge 2), exact discretization with the fractional delay.

Usage: python3 dab_loop.py   (prints margins and step responses for the PI of the model and for a
candidate PI with a lower kp; writes loop.npz for dab_plots.py)
"""
import os
import numpy as np
from scipy.linalg import expm
import dab_sim as ds
from dab_sweep import harmonic_op

HERE = os.path.dirname(os.path.abspath(__file__))
P = ds.P
F = 12e3
TS = 1/24e3


def dc_network(p):
    """continuous linear model of both DC sides, inputs i_dab1 (drawn from DC link 1) and i_dab2 (injected
    into DC link 2); states [vr1, vb1, iL1, vC1, vC2, iL2, vb2, vr2]; outputs V1 and V2 need the DAB currents
    (ESR): V1 = vC1 + ESR (iL1 - i_dab1), V2 = vC2 + ESR (i_dab2 - iL2)"""
    R0, R1, C1, Cb = p['R0'], p['R1'], p['C1'], p['Cb']
    Lf, RLf, Cf, ESR = p['Lf'], p['RLf'], p['Cf'], p['RCf']
    A = np.zeros((8, 8))
    B = np.zeros((8, 2))
    # side 1: icell = (E - vr1 - vb1)/R0
    A[0, 0] = (-1/R0 - 1/R1)/C1; A[0, 1] = -1/(R0*C1)
    A[1, 0] = -1/(R0*Cb); A[1, 1] = -1/(R0*Cb); A[1, 2] = -1/Cb
    A[2, 1] = 1/Lf; A[2, 2] = (-RLf - ESR)/Lf; A[2, 3] = -1/Lf; B[2, 0] = ESR/Lf
    A[3, 2] = 1/Cf; B[3, 0] = -1/Cf
    # side 2
    A[4, 5] = -1/Cf; B[4, 1] = 1/Cf
    A[5, 4] = 1/Lf; A[5, 5] = (-RLf - ESR)/Lf; A[5, 6] = -1/Lf; B[5, 1] = ESR/Lf
    A[6, 5] = 1/Cb; A[6, 6] = -1/(R0*Cb); A[6, 7] = -1/(R0*Cb)
    A[7, 6] = -1/(R0*C1); A[7, 7] = (-1/R0 - 1/R1)/C1
    # V = Cv x + Dv [i_dab1, i_dab2]
    Cv = np.zeros((2, 8)); Dv = np.zeros((2, 2))
    Cv[0, 3] = 1; Cv[0, 2] = ESR; Dv[0, 0] = -ESR
    Cv[1, 4] = 1; Cv[1, 5] = -ESR; Dv[1, 1] = ESR
    return A, B, Cv, Dv


def dab_partials(V1, V2, phi, p, k_phi=1.0):
    """[i_dab1, i_dab2] = P1/V1, P2/V2 of the harmonic model; returns d/dphi (2,) and d/dV (2, 2)"""
    def cur(v1, v2, ph):
        o = harmonic_op(v1, v2, ph, p, nh=299, npts=64)
        return np.array([o['P1']/v1, o['P2']/v2])
    dphi = 1e-4
    dV = 0.5
    gphi = (cur(V1, V2, phi + dphi) - cur(V1, V2, phi - dphi))/(2*dphi)*k_phi
    gV = np.zeros((2, 2))
    gV[:, 0] = (cur(V1 + dV, V2, phi) - cur(V1 - dV, V2, phi))/(2*dV)
    gV[:, 1] = (cur(V1, V2 + dV, phi) - cur(V1, V2 - dV, phi))/(2*dV)
    return gphi, gV


def plant(V1, V2, phi, p, k_phi=1.0):
    """state space from u (controller output, pu) to iL1/Idc_FS (pu)"""
    A, B, Cv, Dv = dc_network(p)
    gphi, gV = dab_partials(V1, V2, phi, p, k_phi)
    # a = gphi*dphi + gV*V, V = Cv x + Dv a  ->  a = M (gphi dphi + gV Cv x)
    M = np.linalg.inv(np.eye(2) - gV @ Dv)
    Ac = A + B @ M @ gV @ Cv
    Bc = (B @ M @ gphi).reshape(8, 1)*p['phi_max']
    Cc = np.zeros((1, 8)); Cc[0, 2] = 1/p['Idc_FS']
    return Ac, Bc, Cc, gphi


def zoh_delay(A, B, T, Tm):
    """x[k+1] = Phi x[k] + G0 u[k] + G1 u[k-1] for u held over [kT, (k+1)T) and applied with delay Tm < T"""
    n = A.shape[0]

    def phi_gam(t):
        M = np.zeros((n + 1, n + 1))
        M[:n, :n] = A*t
        M[:n, n:] = B*t
        E = expm(M)
        return E[:n, :n], E[:n, n:]
    Phi, _ = phi_gam(T)
    Ph_a, Ga = phi_gam(T - Tm)       # u[k] acts over [kT + Tm, (k+1)T)
    _, Gb = phi_gam(Tm)              # u[k-1] acts over [kT, kT + Tm)
    G0 = Ga
    G1 = Ph_a @ Gb
    return Phi, G0, G1


def closed_loop(Ac, Bc, Cc, Tm, p, kff):
    """discrete closed loop; state [x (8), u(k-1), y_delay, xi]; input reference r; outputs iL1 (pu), u"""
    Phi, G0, G1 = zoh_delay(Ac, Bc, TS, Tm)
    n = Ac.shape[0]
    kp, ki = p['kp'], p['ki']
    N = n + 3
    Acl = np.zeros((N, N)); Bcl = np.zeros(N)
    # u[k] = kff r + kp e + ki ts e + xi, e = r - yd
    # x[k+1] = Phi x + G0 u[k] + G1 u[k-1]
    cu_r = kff + kp + ki*TS
    cu_yd = -(kp + ki*TS)
    Acl[:n, :n] = Phi
    Acl[:n, n] = G1[:, 0]
    Acl[:n, n + 1] = G0[:, 0]*cu_yd
    Acl[:n, n + 2] = G0[:, 0]
    Bcl[:n] = G0[:, 0]*cu_r
    # u(k-1) state <- u[k]
    Acl[n, n + 1] = cu_yd; Acl[n, n + 2] = 1.0; Bcl[n] = cu_r
    # y_delay <- y[k] = Cc x[k]
    Acl[n + 1, :n] = Cc[0]
    # xi <- xi + ki ts e
    Acl[n + 2, n + 1] = -ki*TS; Acl[n + 2, n + 2] = 1.0; Bcl[n + 2] = ki*TS
    return Acl, Bcl


def loop_gain(Ac, Bc, Cc, Tm, p, w):
    """L(e^{jwT}) = C(z) z^-1 Gd(z), broken at the controller output"""
    Phi, G0, G1 = zoh_delay(Ac, Bc, TS, Tm)
    n = Ac.shape[0]
    L = np.zeros(len(w), dtype=complex)
    for k, ww in enumerate(w):
        z = np.exp(1j*ww*TS)
        # Gd(z) = Cc (zI - Phi)^-1 (G0 + G1 z^-1)
        Gd = (Cc @ np.linalg.solve(z*np.eye(n) - Phi, G0 + G1/z))[0, 0]
        Cz = p['kp'] + p['ki']*TS*z/(z - 1)
        L[k] = Cz*Gd/z
    return L


def margins(w, L):
    mag = np.abs(L)
    ph = np.unwrap(np.angle(L))
    out = {}
    idx = np.nonzero((mag[:-1] >= 1) & (mag[1:] < 1))[0]
    out['wc'] = [w[i] for i in idx]
    out['pm'] = [np.degrees(ph[i]) + 180 for i in idx]
    # gain margin: phase crossing -180 (mod 360)
    phd = np.degrees(ph)
    idx2 = np.nonzero(np.floor((phd[:-1] + 180)/360) != np.floor((phd[1:] + 180)/360))[0]
    out['w180'] = [w[i] for i in idx2]
    out['gm_db'] = [-20*np.log10(mag[i]) for i in idx2]
    out['peak_S_db'] = 20*np.log10(np.max(1/np.abs(1 + L)))
    out['w_peak_S'] = w[np.argmax(1/np.abs(1 + L))]
    return out


def step(Acl, Bcl, Cc, n_steps, kp, ki, kff, r0=0.0, r1=1.0):
    n = Cc.shape[1]
    x = np.zeros(Acl.shape[0])
    y = np.zeros(n_steps); u = np.zeros(n_steps)
    for k in range(n_steps):
        y[k] = Cc[0] @ x[:n]
        e = r1 - x[n + 1]
        u[k] = kff*r1 + (kp + ki*TS)*e + x[n + 2]
        x = Acl @ x + Bcl*r1
    return y, u


if __name__ == '__main__':
    p = dict(P)
    w = 2*np.pi*np.logspace(0, np.log10(11.9e3), 4000)
    A, B, Cv, Dv = dc_network(p)
    ev = np.linalg.eigvals(A)
    print('DC networks (both sides, DAB as ideal current sources): poles')
    for e in sorted(ev, key=lambda z: abs(z)):
        if e.imag >= 0:
            print(f'   {e.real:10.1f} {e.imag:+10.1f}j rad/s  f = {abs(e)/2/np.pi:7.1f} Hz  zeta = {-e.real/abs(e):.3f}')
    # side 1 transfer i_dab1 -> iL1 at a few frequencies
    s_eval = [20, 100, 168, 200, 300, 1000]
    print('|iL1/i_dab1| (side 1 network):', ', '.join(
        f'{f} Hz {abs((np.array([[0, 0, 1, 0, 0, 0, 0, 0]]) @ np.linalg.solve(2j*np.pi*f*np.eye(8) - A, B[:, :1]))[0, 0]):.2f}'
        for f in s_eval))
    cases = [
        # name, V1, V2, u0, k_phi (effective/harmonic phase gain)
        ('full load (u 0.518, harmonic gain)', 1083.7, 1139.4, 0.518, 1.0),
        ('light load (u 0.105, harmonic gain)', 1110.5, 1115.1, 0.105, 1.0),
        ('light load (u 0.105, gain of the switched simulation)', 1110.5, 1115.1, 0.105, None),
    ]
    ctrls = [
        # name, kp, ki
        ('PI of the model (kp 0.5, ki 18)', 0.5, 18.0),
        ('candidate PI (kp 0.05, ki 40)', 0.05, 40.0),
    ]
    sav = {}
    for j, (cname, kp, ki) in enumerate(ctrls):
        p['kp'], p['ki'] = kp, ki
        print(f'\n################ {cname}')
        for i, (name, V1, V2, u0, kphi) in enumerate(cases):
            phi0 = u0*p['phi_max']
            if kphi is None:
                # effective slope of the switched model between u = 0.10 and 0.18 (dab_sweep.py): 1024 A per unit u
                _, _, _, g = plant(V1, V2, phi0, p)
                kphi = (1024.0/p['phi_max'])/g[0]
            Ac, Bc, Cc, g = plant(V1, V2, phi0, p, kphi)
            Tm = 1/F/4 + phi0/(2*np.pi)/F
            dc_gain = (-Cc @ np.linalg.solve(Ac, Bc))[0, 0]
            L = loop_gain(Ac, Bc, Cc, Tm, p, w)
            m = margins(w, L)
            print(f'\n== {name}: V1 {V1} V, V2 {V2} V, phi {np.degrees(phi0):.1f} deg; d i_dab1/du '
                  f'{g[0]*p["phi_max"]:.0f} A per unit u ({g[0]*p["phi_max"]/p["Idc_FS"]:.2f} pu/pu), plant DC gain '
                  f'{dc_gain:.2f} pu/pu, modulator delay {Tm*1e6:.1f} us')
            for wc, pm in zip(m['wc'], m['pm']):
                print(f'   crossover {wc/2/np.pi:7.1f} Hz, phase margin {pm:6.1f} deg')
            for w1, gm in zip(m['w180'], m['gm_db']):
                print(f'   phase -180 at {w1/2/np.pi:7.1f} Hz, gain margin {gm:6.1f} dB')
            print(f'   peak of the sensitivity |1/(1+L)| {m["peak_S_db"]:.1f} dB at {m["w_peak_S"]/2/np.pi:.0f} Hz')
            sav[f'L_{j}_{i}'] = L
            for kff in (1.0, 0.5, 0.0):
                Acl, Bcl = closed_loop(Ac, Bc, Cc, Tm, p, kff)
                ev = np.linalg.eigvals(Acl)
                dom = ev[np.argsort(-np.abs(ev))]
                nst = int(0.1/TS)
                y, uu = step(Acl, Bcl, Cc, nst, p['kp'], p['ki'], kff)
                t = np.arange(nst)*TS
                k2 = np.nonzero(np.abs(y - 1) > 0.02)[0]
                t2 = t[k2[-1]] if len(k2) else 0.0
                print(f'   k_ff {kff}: largest |closed-loop pole| {abs(dom[0]):.5f}; unit reference step (no filter): '
                      f'peak {y.max():.2f} at {t[np.argmax(y)]*1e3:.2f} ms, y(1 ms) {y[int(1e-3/TS)]:.2f}, '
                      f'y(5 ms) {y[int(5e-3/TS)]:.2f}, y(20 ms) {y[int(20e-3/TS)]:.2f}, y(50 ms) {y[int(50e-3/TS)]:.2f}, '
                      f'2 % band from {t2*1e3:.1f} ms')
                # oscillatory poles
                osc = [e for e in ev if abs(np.angle(e)) > 1e-3 and np.angle(e) > 0]
                for e in sorted(osc, key=lambda z: -abs(z))[:2]:
                    sc = np.log(e)/TS
                    print(f'      pole pair {abs(sc)/2/np.pi:.0f} Hz, zeta {-sc.real/abs(sc):.3f}')
                sav[f'step_{j}_{i}_{int(round(kff*10))}'] = np.vstack((t, y, uu))
    # keys: L_<ctrl>_<case>, step_<ctrl>_<case>_<10 k_ff> (rows t, iL1 pu, u)
    np.savez(os.path.join(HERE, 'loop.npz'), w=w, names=np.array([c[0] for c in cases]),
             ctrl_names=np.array([c[0] for c in ctrls]), **sav)
