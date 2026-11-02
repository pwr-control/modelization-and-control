#!/usr/bin/env python3
"""Numbers of the cases of dab_sim.py: control intervals (time course) and high-resolution windows.

Usage: python3 dab_analyze.py <case> [<case> ...]
"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
IFS = 250.0
FPWM = 12e3

# columns of rec (one row per trigger of ui_ctrl)
T, U, REF, FB, XI, IL1, IL2, VC1, VC2 = range(9)
A_IL1, A_IL2, A_VDC1, A_VDC2, A_PDC1, A_PDC2, A_I1SQ, A_I2SQ, A_PAC1, A_PAC2, A_PB1, A_PB2 = range(9, 21)
M_IL1, M_IL2, M_VDC1, M_VDC2, M_I1, M_I2, M_IM, M_VCS1 = range(21, 29)
Z1, Z2, NACC = 29, 30, 31


def load(name):
    d = np.load(os.path.join(HERE, f'case_{name}.npz'))
    return d


def full_period(rec, col):
    """average over pairs of control intervals (one PWM period), weighted by the number of steps"""
    n = rec[1:, NACC]
    w = rec[1:, col]*n
    s = w[1:] + w[:-1]
    m = n[1:] + n[:-1]
    return rec[2:, T], s/np.maximum(m, 1)


def window_stats(win, h, label):
    w = win[win[:, 0] > 0]
    if len(w) == 0:
        return
    nper = int(round(1.0/FPWM/h))
    # whole periods only (T/h may not be an integer: use 3 periods when T/h*3 is)
    npp = nper
    for k in (1, 2, 3, 6):
        if abs(k/FPWM/h - round(k/FPWM/h)) < 1e-6:
            npp = int(round(k/FPWM/h))
            break
    n = (len(w)//npp)*npp
    w = w[:n]
    t = w[:, 0]
    vab1, vab2, i1, i2, im = w[:, 1], w[:, 2], w[:, 3], w[:, 4], w[:, 5]
    print(f'  window {label}: {t[0]*1e3:.3f} .. {t[-1]*1e3:.3f} ms ({n} steps, {n*h*FPWM:.2f} periods)')
    print(f'    u {w[:, 17].mean():.4f} (phase {np.degrees(w[:, 16].mean()):.2f} deg, '
          f'min {np.degrees(w[:, 16].min()):.2f} max {np.degrees(w[:, 16].max()):.2f})')
    print(f'    vdc1 {w[:, 8].mean():.1f} V (p-p {np.ptp(w[:, 8]):.2f}), vdc2 {w[:, 9].mean():.1f} V (p-p {np.ptp(w[:, 9]):.2f})')
    print(f'    iL1 {w[:, 10].mean():.2f} A (p-p {np.ptp(w[:, 10]):.3f}), iL2 {w[:, 11].mean():.2f} A (p-p {np.ptp(w[:, 11]):.3f})')
    print(f'    idab1 {w[:, 12].mean():.2f} A, idab2 {w[:, 13].mean():.2f} A; P_dc1 {np.mean(w[:, 8]*w[:, 12])/1e3:.2f} kW, '
          f'P_dc2 {np.mean(w[:, 9]*w[:, 13])/1e3:.2f} kW; P_ac1 {np.mean(vab1*i1)/1e3:.2f} kW, P_ac2 {np.mean(vab2*i2)/1e3:.2f} kW')
    print(f'    i1 rms {np.sqrt(np.mean(i1**2)):.1f} A, peak {np.abs(i1).max():.1f} A, dc {i1.mean():.2f} A | '
          f'i2 rms {np.sqrt(np.mean(i2**2)):.1f} A, peak {np.abs(i2).max():.1f} A | im peak {np.abs(im).max():.1f} A, '
          f'dc {im.mean():.2f} A | vCs1 dc {w[:, 6].mean():.2f} V p-p {np.ptp(w[:, 6]):.1f} V, '
          f'vCs2 dc {w[:, 7].mean():.2f} V p-p {np.ptp(w[:, 7]):.1f} V')
    # switching instants: changes of the gate pattern; current at the turn-off of the outgoing switches
    for b, (col_g, col_i, sgn) in enumerate(((14, 3, 1.0), (15, 4, -1.0))):
        g = w[:, col_g].astype(int)
        idx = np.nonzero(np.diff(g) != 0)[0] + 1
        out = []
        for k in idx:
            before, after = g[k - 1], g[k]
            if after == 0 or bin(after).count('1') < bin(before).count('1'):
                # an outgoing pair turns off: (Q1,Q4) = 1 + 8 = 9 -> v_ab + ; (Q2,Q3) = 2 + 4 = 6 -> v_ab -
                cur = w[k - 1, col_i]
                # ZVS when the current charges the outgoing switches' capacitances: leaving the + state
                # needs i (out of uac(+) for bridge 1) > 0, leaving the - state needs i < 0; bridge 2 is
                # seen from the transformer (current into uac(+) of bridge 2 is -i2o): sign flipped
                pos_state = (before == 9)
                ok = (sgn*cur > 0) if pos_state else (sgn*cur < 0)
                # dead time: all four gates off (code 0) until the incoming pair turns on; if the current
                # reaches zero in this time the diodes block, the bridge floats and the incoming switches turn
                # on hard at the end of the dead time (commutation not completed)
                kk = k
                lost = False
                while kk < len(g) and g[kk] == 0:
                    if abs(w[kk, col_i]) < 0.5:
                        lost = True
                    kk += 1
                out.append((t[k], pos_state, cur, ok, lost))
        if out:
            cur = np.array([o[2] for o in out])
            ok = np.array([o[3] for o in out])
            lost = np.array([o[4] for o in out])
            print(f'    bridge {b + 1}: {len(out)} turn-off events, |i| at turn-off min {np.abs(cur).min():.1f} '
                  f'mean {np.abs(cur).mean():.1f} max {np.abs(cur).max():.1f} A, current in the ZVS direction '
                  f'{ok.sum()}/{len(ok)}, current back to zero inside the dead time (bridge floats, hard turn-on '
                  f'at its end) {lost.sum()}/{len(lost)}')
    # dead time effect: duration of the zero/transition states of v_ab
    nit = w[:, 18]
    print(f'    switch/diode iterations per step: max {int(nit.max())}, steps with > 3: {(nit > 3).sum()}')


def summary(name):
    d = load(name)
    rec = d['rec']
    h = float(d['h'])
    t = rec[:, T]
    print(f'===== {name}: {t[-1]:.3f} s, {len(t)} control intervals, tc = {h*1e9:.1f} ns')
    tt, il1 = full_period(rec, A_IL1)
    _, pdc1 = full_period(rec, A_PDC1)
    _, pdc2 = full_period(rec, A_PDC2)
    _, pb1 = full_period(rec, A_PB1)
    _, pb2 = full_period(rec, A_PB2)
    _, pac1 = full_period(rec, A_PAC1)
    _, pac2 = full_period(rec, A_PAC2)
    for tm in (0.01, 0.05, 0.1, 0.149, 0.16, 0.17, 0.18, 0.2, 0.25, 0.3, 0.4, 0.5, 0.599, 0.79, 1.0, 1.5, 1.99):
        if tm > t[-1]:
            continue
        k = np.argmin(abs(t - tm))
        s = slice(max(k - 24, 0), k + 24)        # +-1 ms
        kk = np.argmin(abs(tt - tm))
        s2 = slice(max(kk - 24, 0), kk + 24)
        print(f'  t={tm*1e3:7.1f} ms ref {rec[k, REF]:.4f} u {rec[s, U].mean():.4f} (min {rec[s, U].min():.4f} max '
              f'{rec[s, U].max():.4f}) fb {rec[s, FB].mean():.4f} iL1 {il1[s2].mean():7.2f} A ({il1[s2].mean()/IFS:.4f} pu, '
              f'p-p over 1 ms {np.ptp(il1[s2]):.2f}) iL2 {rec[s, IL2].mean():7.2f} A vdc1 {rec[s, A_VDC1].mean():7.1f} '
              f'vdc2 {rec[s, A_VDC2].mean():7.1f} V | P_bat1 {pb1[s2].mean()/1e3:7.2f} kW P_dc1 {pdc1[s2].mean()/1e3:7.2f} '
              f'P_ac1 {pac1[s2].mean()/1e3:7.2f} P_ac2 {pac2[s2].mean()/1e3:7.2f} P_dc2 {pdc2[s2].mean()/1e3:7.2f} '
              f'P_bat2 {pb2[s2].mean()/1e3:7.2f} kW')
    i = np.argmax(rec[:, M_IL1])
    print(f'  max |iL1| {rec[:, M_IL1].max():.0f} A at {t[i]*1e3:.2f} ms; max vdc1 {rec[:, M_VDC1].max():.0f} V; max vdc2 '
          f'{rec[:, M_VDC2].max():.0f} V; max |i1| {rec[:, M_I1].max():.0f} A; max |i2| {rec[:, M_I2].max():.0f} A; '
          f'max |im| {rec[:, M_IM].max():.1f} A; max |vCs1| {rec[:, M_VCS1].max():.1f} V')
    # bench step: overshoot and settling of the filtered current (full-period averages)
    ref = rec[:, REF]
    if t[-1] > 0.3 and ref.max() > 0.99:          # bench cases (reference 0.1 -> 1 pu)
        sel = tt > 0.15
        ip = il1[sel]/IFS
        tp = tt[sel]
        kmax = np.argmax(ip)
        print(f'  after the step: peak iL1 {ip[kmax]:.3f} pu at {tp[kmax]*1e3:.1f} ms; '
              f'u peak {rec[t > 0.15, U].max():.3f}')
        for lev in (0.02, 0.01):
            out = np.nonzero(np.abs(ip - 1.0) > lev)[0]
            ts_ = tp[out[-1]] if len(out) else tp[0]
            print(f'  |iL1 - 1 pu| < {lev*100:.0f} % from {ts_*1e3:.1f} ms')
    print(f'  soc {float(d["z1"]):.6f} / {float(d["z2"]):.6f}')
    win = d['win']
    for kw in range(win.shape[0]):
        lab = f'{float(d["win_t"][kw])*1e3:.1f} ms' if len(d['win_t']) else ''
        window_stats(win[kw], h, lab)


if __name__ == '__main__':
    for nm in sys.argv[1:]:
        summary(nm)
