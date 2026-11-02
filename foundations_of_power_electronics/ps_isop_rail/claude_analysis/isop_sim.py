#!/usr/bin/env python3
"""Switched simulation of ps_isop_rail.slx (design check, not part of the model).

Same circuit as the Simscape network of the model, same solver idea as its local solver (backward Euler at a
fixed step tc = 0.625 us, piecewise-linear switches and diodes):
  battery_1  : cell model of the library (N*ocv(z) + M*h, R0, R1 || C1), 1 mF terminal capacitor, 1 mH line
  isop       : Ldc_dc1 (1 mH + 3.14 mOhm) -> four modules in series at the input; each module: 1.2 mF + 1 mOhm,
               full bridge of ideal switches (1.5 mOhm / 1e-8 S, gate on above Vth) with antiparallel diodes
               (2 V + 0.15 mOhm) and RC snubbers (2200 Ohm + 10 nF), transformer T model (Ld1 + Rd1, Lm || Rfe,
               ideal n2/n1, Ld2 + Rd2); secondaries of modules 1+4 in parallel on rectifier_1, 2+3 on rectifier_2;
               diode rectifiers (0.977 V + 22 uOhm, 1e-8 S; the 1 pF snubbers are left out), 250 uH + 0.39 mOhm
               each, common 1.2 mF + 0.1 mOhm, 0.5 Ohm load.
  control    : full_bridge_zvs_pwm_modulator (triangle carriers, leg B delayed by phase_disp*T/2, d1 = 0.5,
               dead time 3 us, TRGO at the carrier peaks -> 8 kHz), voltage PI of the library (pi_ctrl, back
               calculation), feedback quantized (1/2^11) and delayed by one sample, reference udc_ref from t = 0.
The "DC transformer current" PI is not simulated: in the model its output is not connected (d1_m/Sum input 2).

Usage: python3 isop_sim.py <case> [<case> ...]   (numpy + numba; writes case_<name>.npz next to this script)
  cases: smoke, nominal, zero_init, catenary_3kV, mismatch, mm_n1, mm_n2, mm_L1, mm_L2, mm_L2s, mm_C3, mm_Lm4,
         split_in_pair, split_pairs, split_in_pair_ss, split_pairs_ss (see __main__ at the end)
  figures: python3 plots.py (needs nominal, zero_init, catenary_3kV, mm_n1, mm_L2s, mm_L2, split_in_pair_ss,
           split_pairs_ss); numbers: python3 analyze.py <case> ..., python3 analyze_win.py <case> ...
"""
import os
import sys
import time
import numpy as np
import numba as nb

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ parameters (values of init_model.m)
P = dict(
    h=0.625e-6,                       # param.glb_time.tc (local solver step, modulator step)
    fpwm=4e3, dead_time=3e-6, ts=125e-6,
    # battery_1 (param.battery.pack_1 after the legacy changes)
    N=1249, E_1=-1.031, E0=3.485, E1=0.2156, E2=0.0, E3=0.0, Elog=-0.05, alpha=35.0, M=125.0,
    R0=0.0234375, R1=0.0234375, C1=0.0025, Q=2400.0, soc0=0.85, Cb=1e-3, RCb=1e-6, v_init_b=4500.0,
    L_line=1e-3,
    # ISOP hardware (param.isop_rail.hw)
    Lin=1e-3, RLin=3.14e-3, Cin=1.2e-3, RCin=1e-3,
    Ld1=13.0208333e-6, Ld2=13.0208333e-6, Rd1=1e-3, Rd2=1e-3, Lm=1e-3, Rfe=1e3, n=1.0,
    Lout=250e-6, RLout=0.3925e-3, Cout=1.2e-3, RCout=1e-4, Rload=0.5,
    # devices
    Ron_sw=1.5e-3, Goff=1e-8, Vf_d=2.0, Ron_d=0.15e-3, Rsn=2200.0, Csn=10e-9,
    Vf_r=0.977, Ron_r=22e-6,
    # control (param.isop_rail.ctrl)
    kp=0.5, ki=18.0, udc_ref=1.0, udc2_bez=750.0, q=1.0/2**11,
)

# ------------------------------------------------------------------ circuit description
# nodes: 0 a (battery, after R0), 1 b (battery terminal), 2 c (after the line inductor), 3..6 t0..t3 (string;
# t4 = ground = battery minus), module k: A 7+4k, B 8+4k, m1 9+4k, m2 10+4k, rectifier r: ac1 23+3r, ac2 24+3r,
# p 25+3r, 29 o (output); unknowns 30..33: currents of the ideal transformers
NN = 30
NX = 34
GND = -1


def t_node(k):
    return 3 + k if k < 4 else GND


def build(p, mod=None):
    """element arrays; mod[k] = dict of per-module overrides (Ld1, Ld2, Cin, n, Lm)"""
    mod = mod or [{} for _ in range(4)]
    h = p['h']
    R = []      # (a, b, G)
    L = []      # (a, b, L, R)
    C = []      # (a, b, C, R)
    S = []      # (c, e)
    D = []      # (anode, cathode, Vf, Ron)
    T = []      # (m1, B, m2, ac2, n)
    R.append((0, GND, 1.0/p['R0']))          # battery R0 (Norton with E/R0 into node a)
    R.append((0, 1, 1.0/p['R1']))            # R1 of the R1 || C1 branch
    C.append((0, 1, p['C1'], 0.0))           # C1
    C.append((1, GND, p['Cb'], p['RCb']))    # battery terminal capacitor (1 mF + 1 uOhm)
    L.append((1, 2, p['L_line'], 0.0))       # line inductor of battery_1
    L.append((2, 3, p['Lin'], p['RLin']))    # Ldc_dc1
    rect_of = [0, 1, 1, 0]                   # modules 1 and 4 -> rectifier_1, 2 and 3 -> rectifier_2
    for k in range(4):
        mk = mod[k]
        P_, N_ = t_node(k), t_node(k + 1)
        A, B, m1, m2 = 7 + 4*k, 8 + 4*k, 9 + 4*k, 10 + 4*k
        r = rect_of[k]
        ac1, ac2 = 23 + 3*r, 24 + 3*r
        C.append((P_, N_, mk.get('Cin', p['Cin']), p['RCin']))
        for (c, e) in ((P_, A), (A, N_), (P_, B), (B, N_)):   # Q1 Q2 (leg A), Q3 Q4 (leg B)
            S.append((c, e))
            D.append((e, c, p['Vf_d'], p['Ron_d']))           # antiparallel diode
            C.append((c, e, p['Csn'], p['Rsn']))              # RC snubber
        L.append((A, m1, mk.get('Ld1', p['Ld1']), p['Rd1']))  # primary leakage
        L.append((m1, B, mk.get('Lm', p['Lm']), 0.0))         # magnetizing inductance
        R.append((m1, B, 1.0/p['Rfe']))
        L.append((ac1, m2, mk.get('Ld2', p['Ld2']), p['Rd2']))  # secondary leakage (u2_p = ac1)
        T.append((m1, B, m2, ac2, mk.get('n', p['n'])))
    for r in range(2):
        ac1, ac2, pp = 23 + 3*r, 24 + 3*r, 25 + 3*r
        D.append((ac1, pp, p['Vf_r'], p['Ron_r']))
        D.append((ac2, pp, p['Vf_r'], p['Ron_r']))
        D.append((GND, ac1, p['Vf_r'], p['Ron_r']))
        D.append((GND, ac2, p['Vf_r'], p['Ron_r']))
        L.append((pp, 29, p['Lout'], p['RLout']))
    C.append((29, GND, p['Cout'], p['RCout']))
    R.append((29, GND, 1.0/p['Rload']))
    for nd in range(23, 29):                 # tiny leak to ground on the secondary side (numerics only)
        R.append((nd, GND, 1e-9))
    el = dict(
        R=np.array(R, dtype=np.float64), L=np.array(L, dtype=np.float64), C=np.array(C, dtype=np.float64),
        S=np.array(S, dtype=np.int64), D=np.array(D, dtype=np.float64), T=np.array(T, dtype=np.float64))
    return el


# ------------------------------------------------------------------ numba kernels
@nb.njit(cache=True)
def stamp_g(G, a, b, g):
    if a >= 0:
        G[a, a] += g
    if b >= 0:
        G[b, b] += g
    if a >= 0 and b >= 0:
        G[a, b] -= g
        G[b, a] -= g


@nb.njit(cache=True)
def stamp_i(I, a, b, i0):
    # branch current i = g*v_ab + i0 leaving a
    if a >= 0:
        I[a] -= i0
    if b >= 0:
        I[b] += i0


@nb.njit(cache=True)
def lu_factor(A, piv):
    n = A.shape[0]
    for k in range(n):
        p = k
        mx = abs(A[k, k])
        for i in range(k + 1, n):
            if abs(A[i, k]) > mx:
                mx = abs(A[i, k])
                p = i
        piv[k] = p
        if p != k:
            for j in range(n):
                tmp = A[k, j]
                A[k, j] = A[p, j]
                A[p, j] = tmp
        akk = A[k, k]
        for i in range(k + 1, n):
            A[i, k] /= akk
            f = A[i, k]
            if f != 0.0:
                for j in range(k + 1, n):
                    A[i, j] -= f * A[k, j]


@nb.njit(cache=True)
def lu_solve(A, piv, b, x):
    n = A.shape[0]
    for i in range(n):
        x[i] = b[i]
    for k in range(n):
        p = piv[k]
        if p != k:
            tmp = x[k]
            x[k] = x[p]
            x[p] = tmp
    for i in range(n):
        s = x[i]
        for j in range(i):
            s -= A[i, j] * x[j]
        x[i] = s
    for i in range(n - 1, -1, -1):
        s = x[i]
        for j in range(i + 1, n):
            s -= A[i, j] * x[j]
        x[i] = s / A[i, i]


@nb.njit(cache=True)
def ramp(t, phase, period):
    x = np.sin(2.0*np.pi/period*t - phase - np.pi/2.0)
    if x > 1.0:
        x = 1.0
    if x < -1.0:
        x = -1.0
    return np.arcsin(x)/np.pi + 0.5


@nb.njit(cache=True)
def simulate(Rel, Lel, Cel, Sel, Del, Tel, prm, x_init, iL0, vC0, nsteps, win_start, win_len,
             Q_ah, pd_fixed, ref_t, ref_v):
    """prm: [h, fpwm, dead_time, ts, N, E_1, E0, E1, E2, E3, Elog, alpha, M, R0, kp, ki, udc2_bez, q, soc0]
    pd_fixed < 0: closed loop, else fixed phase displacement. ref_t/ref_v: piecewise-constant reference."""
    h = prm[0]; fpwm = prm[1]; dt = prm[2]; ts = prm[3]
    Ncell = prm[4]; E_1 = prm[5]; E0 = prm[6]; E1 = prm[7]; E2 = prm[8]; E3 = prm[9]; Elog = prm[10]
    alpha = prm[11]; Mh = prm[12]; R0 = prm[13]; kp = prm[14]; ki = prm[15]; ubez = prm[16]; qz = prm[17]
    z = prm[18]
    period = 1.0/fpwm
    dtf = dt*fpwm
    th = fpwm*h*2.0
    nR = Rel.shape[0]; nL = Lel.shape[0]; nC = Cel.shape[0]; nS = Sel.shape[0]; nD = Del.shape[0]
    nT = Tel.shape[0]
    # fixed part of the matrix
    G0 = np.zeros((NX, NX))
    for k in range(nR):
        stamp_g(G0, int(Rel[k, 0]), int(Rel[k, 1]), Rel[k, 2])
    gL = np.zeros(nL)
    kL = np.zeros(nL)
    for k in range(nL):
        Lk = Lel[k, 2]; Rk = Lel[k, 3]
        gL[k] = 1.0/(Rk + Lk/h)
        kL[k] = gL[k]*Lk/h
        stamp_g(G0, int(Lel[k, 0]), int(Lel[k, 1]), gL[k])
    gC = np.zeros(nC)
    for k in range(nC):
        gC[k] = 1.0/(Cel[k, 3] + h/Cel[k, 2])
        stamp_g(G0, int(Cel[k, 0]), int(Cel[k, 1]), gC[k])
    for k in range(nT):
        j = NN + k
        m1 = int(Tel[k, 0]); B = int(Tel[k, 1]); m2 = int(Tel[k, 2]); ac2 = int(Tel[k, 3]); n = Tel[k, 4]
        if m1 >= 0: G0[m1, j] += 1.0
        if B >= 0: G0[B, j] -= 1.0
        if m2 >= 0: G0[m2, j] -= 1.0/n
        if ac2 >= 0: G0[ac2, j] += 1.0/n
        if m2 >= 0: G0[j, m2] += 1.0
        if ac2 >= 0: G0[j, ac2] -= 1.0
        if m1 >= 0: G0[j, m1] -= n
        if B >= 0: G0[j, B] += n
    # states
    iL = iL0.copy()
    vC = vC0.copy()
    hyst = 0.0
    sw = np.zeros(nS, dtype=np.int64)
    dst = np.zeros(nD, dtype=np.int64)
    x = x_init.copy()
    # LU cache
    NCACHE = 512
    cache_key = -np.ones(NCACHE, dtype=np.int64)
    cache_LU = np.zeros((NCACHE, NX, NX))
    cache_piv = np.zeros((NCACHE, NX), dtype=np.int64)
    I = np.zeros(NX)
    # control
    xi = 0.0           # PI state
    u_delay = 0.0      # unit delay of the feedback
    pd = 0.0           # PI output (phase displacement, pu)
    pd_f = 0.0         # after the 1/(tc s + 1) filter of the modulator
    trgo_prev = 1      # no trigger at the first evaluation (TriggerPort default)
    n_trig_max = int(nsteps*h/ts*1.05) + 16
    rec = np.zeros((n_trig_max, 30))
    amax = np.zeros(6)
    acc = np.zeros(16)
    nacc = 0
    itrig = 0
    win = np.zeros((win_len, 26))
    nfact = 0
    maxit_hit = 0
    iref = 0
    ref = ref_v[0]
    for step in range(nsteps):
        t = (step + 1)*h
        while iref + 1 < ref_t.shape[0] and t >= ref_t[iref + 1]:
            iref += 1
            ref = ref_v[iref]
        # ---- modulator and control (at t)
        rA = ramp(t, 0.0, period)
        trgo = 1 if ((rA >= 0.0 and rA <= th) or (rA <= 1.0 and rA >= 1.0 - th)) else 0
        if trgo == 1 and trgo_prev == 0:
            vout = x[29]
            u_meas = qz*np.floor(vout/ubez/qz + 0.5)
            u_fb = u_delay
            u_delay = u_meas
            e = ref - u_fb
            v = kp*e + ki*ts*e + xi
            vs = min(max(v, 0.0), 1.0)
            xi = vs - kp*e
            pd = vs
            if pd_fixed >= 0.0:
                pd = pd_fixed
            if itrig < n_trig_max:
                # record: averages over the last control interval and instantaneous values
                rec[itrig, 0] = t
                rec[itrig, 1] = x[29]
                rec[itrig, 2] = pd
                if nacc > 0:
                    for kk in range(16):
                        rec[itrig, 3 + kk] = acc[kk]/nacc
                rec[itrig, 19] = z
                rec[itrig, 20] = hyst
                rec[itrig, 21] = u_fb
                rec[itrig, 22] = xi
                rec[itrig, 23] = vC[1]
                for kk in range(6):
                    rec[itrig, 24 + kk] = amax[kk]
                itrig += 1
            for kk in range(6):
                amax[kk] = 0.0
            for kk in range(16):
                acc[kk] = 0.0
            nacc = 0
        trgo_prev = trgo
        pd_f = 0.5*(pd_f + pd)          # backward Euler of 1/(tc s + 1) with step tc
        rB = ramp(t, np.pi*pd_f, period)
        gA_up = 1 if rA >= 0.5 + dtf else 0
        gA_lo = 1 if rA <= 0.5 - dtf else 0
        gB_up = 1 if rB >= 0.5 + dtf else 0
        gB_lo = 1 if rB <= 0.5 - dtf else 0
        for k in range(4):
            sw[4*k + 0] = gA_up
            sw[4*k + 1] = gA_lo
            sw[4*k + 2] = gB_up
            sw[4*k + 3] = gB_lo
        # ---- battery source
        zc = min(max(z, 0.0), 1.0)
        ocv = E_1*np.exp(-zc*alpha) + E0 + E1*zc + E2*zc*zc + E3*zc*zc*zc + Elog*np.log(abs(1.0 - zc + 1e-2))
        Ebat = Ncell*ocv + Mh*hyst
        # ---- history sources (independent of switch/diode states)
        for k in range(NX):
            I[k] = 0.0
        I[0] += Ebat/R0
        for k in range(nL):
            stamp_i(I, int(Lel[k, 0]), int(Lel[k, 1]), kL[k]*iL[k])
        for k in range(nC):
            stamp_i(I, int(Cel[k, 0]), int(Cel[k, 1]), -gC[k]*vC[k])
        # ---- solve with diode state iteration
        converged = False
        for it in range(40):
            key = 0
            for k in range(nS):
                key = key*2 + sw[k]
            for k in range(nD):
                key = key*2 + dst[k]
            if key < 0:
                key = -key
            slot = (key ^ (key >> 17) ^ (key >> 31)) % NCACHE
            if cache_key[slot] != key:
                A = cache_LU[slot]
                for i in range(NX):
                    for j in range(NX):
                        A[i, j] = G0[i, j]
                for k in range(nS):
                    g = 1.0/1.5e-3 if sw[k] == 1 else 1e-8
                    stamp_g(A, Sel[k, 0], Sel[k, 1], g)
                for k in range(nD):
                    g = 1.0/Del[k, 3] if dst[k] == 1 else 1e-8
                    stamp_g(A, int(Del[k, 0]), int(Del[k, 1]), g)
                lu_factor(A, cache_piv[slot])
                cache_key[slot] = key
                nfact += 1
            Itot = I.copy()
            for k in range(nD):
                if dst[k] == 1:
                    Vf = Del[k, 2]; Ron = Del[k, 3]
                    stamp_i(Itot, int(Del[k, 0]), int(Del[k, 1]), -Vf/Ron + Vf*1e-8)
            lu_solve(cache_LU[slot], cache_piv[slot], Itot, x)
            changed = 0
            worst = -1
            worst_v = 0.0
            for k in range(nD):
                a = int(Del[k, 0]); c = int(Del[k, 1])
                va = x[a] if a >= 0 else 0.0
                vc = x[c] if c >= 0 else 0.0
                vd = va - vc
                Vf = Del[k, 2]; Ron = Del[k, 3]
                if dst[k] == 1:
                    idd = (vd - Vf)/Ron + Vf*1e-8
                    if idd < -1e-6:
                        changed += 1
                        if -idd*Ron > worst_v:
                            worst_v = -idd*Ron
                            worst = k
                else:
                    if vd > Vf + 1e-6:
                        changed += 1
                        if vd - Vf > worst_v:
                            worst_v = vd - Vf
                            worst = k
            if changed == 0:
                converged = True
                break
            if it < 20:
                for k in range(nD):
                    a = int(Del[k, 0]); c = int(Del[k, 1])
                    va = x[a] if a >= 0 else 0.0
                    vc = x[c] if c >= 0 else 0.0
                    vd = va - vc
                    Vf = Del[k, 2]; Ron = Del[k, 3]
                    if dst[k] == 1:
                        if (vd - Vf)/Ron + Vf*1e-8 < -1e-6:
                            dst[k] = 0
                    else:
                        if vd > Vf + 1e-6:
                            dst[k] = 1
            else:
                dst[worst] = 1 - dst[worst]
        if not converged:
            maxit_hit += 1
        # ---- state update
        for k in range(nL):
            a = int(Lel[k, 0]); b = int(Lel[k, 1])
            va = x[a] if a >= 0 else 0.0
            vb = x[b] if b >= 0 else 0.0
            iL[k] = gL[k]*(va - vb) + kL[k]*iL[k]
        for k in range(nC):
            a = int(Cel[k, 0]); b = int(Cel[k, 1])
            va = x[a] if a >= 0 else 0.0
            vb = x[b] if b >= 0 else 0.0
            ic = gC[k]*(va - vb - vC[k])
            vC[k] += h/Cel[k, 2]*ic
        icell = (Ebat - x[0])/R0
        zdot = -icell/(3600.0*Q_ah)
        if (z >= 1.0 and zdot > 0) or (z <= 0.0 and zdot < 0):
            zdot = 0.0
        z += h*zdot
        sgn = 1.0 if icell > 0 else (-1.0 if icell < 0 else 0.0)
        hyst += h*(-abs(icell/(3600.0*Q_ah))*hyst - abs(icell/(3600.0*Q_ah))*sgn)
        # ---- accumulators (averages over the control interval)
        # 0 iLin, 1 iout total, 2 iLout1, 3 iLout2, 4..7 vCin k, 8..11 iLm k, 12..15 ipri k
        acc[0] += iL[1]
        acc[1] += iL[nL - 2] + iL[nL - 1]
        acc[2] += iL[nL - 2]
        acc[3] += iL[nL - 1]
        for k in range(4):
            acc[4 + k] += vC[2 + 5*k]       # Cin of module k (C list: C1, Cb, then 5 per module: Cin, 4 snubbers)
            acc[8 + k] += iL[2 + 3*k + 1]   # Lm of module k (L list: line, Lin, then 3 per module)
            acc[12 + k] += iL[2 + 3*k]      # primary current of module k
        nacc += 1
        # maxima over the control interval: |iLin|, max vCin, max |ipri|, max |isec|, vout, max |iLm|
        amax[0] = max(amax[0], abs(iL[1]))
        for k in range(4):
            amax[1] = max(amax[1], vC[2 + 5*k])
            amax[2] = max(amax[2], abs(iL[2 + 3*k]))
            amax[3] = max(amax[3], abs(iL[2 + 3*k + 2]))
            amax[5] = max(amax[5], abs(iL[2 + 3*k + 1]))
        amax[4] = max(amax[4], x[29])
        # ---- high resolution window
        if step >= win_start and step < win_start + win_len:
            w = step - win_start
            win[w, 0] = t
            win[w, 1] = x[29]
            for k in range(4):
                win[w, 2 + k] = vC[2 + 5*k]
                win[w, 6 + k] = iL[2 + 3*k]          # primary current
                win[w, 10 + k] = iL[2 + 3*k + 2]     # secondary current (into u2_p)
                win[w, 14 + k] = x[7 + 4*k] - x[8 + 4*k]   # v_AB
                win[w, 18 + k] = iL[2 + 3*k + 1]     # magnetizing current
            win[w, 22] = iL[nL - 2]
            win[w, 23] = iL[nL - 1]
            win[w, 24] = gA_up + 2*gA_lo + 4*gB_up + 8*gB_lo
            win[w, 25] = pd_f
    return rec[:itrig], win, iL, vC, z, hyst, nfact, maxit_hit


def ocv_pack(p, z):
    return p['N']*(p['E_1']*np.exp(-p['alpha']*z) + p['E0'] + p['E1']*z + p['E2']*z**2 + p['E3']*z**3
                   + p['Elog']*np.log(abs(1 - z + 1e-2)))


def initial_state(el, p, precharged=True, vb_scale=1.0):
    """precharged: steady state of the network at t = 0 (what DoDC = on gives): string capacitors at
    OCV/4, battery terminal capacitor at OCV, output at 0, lower switches on at t = 0 (carrier at 0)."""
    nL = el['L'].shape[0]
    nC = el['C'].shape[0]
    iL0 = np.zeros(nL)
    vC0 = np.zeros(nC)
    voc = ocv_pack(p, p['soc0'])
    if precharged:
        vC0[1] = voc
        for k in range(4):
            vC0[2 + 5*k] = voc/4
            # snubbers of the upper switches (Q1, Q3) see the capacitor voltage, lower ones 0
            vC0[2 + 5*k + 1] = voc/4
            vC0[2 + 5*k + 3] = voc/4
    else:
        vC0[1] = p['v_init_b']
    x0 = np.zeros(NX)
    return x0, iL0, vC0


def prm_vector(p):
    return np.array([p['h'], p['fpwm'], p['dead_time'], p['ts'], p['N'], p['E_1'], p['E0'], p['E1'], p['E2'],
                     p['E3'], p['Elog'], p['alpha'], p['M'], p['R0'], p['kp'], p['ki'], p['udc2_bez'], p['q'],
                     p['soc0']])


def run_case(name, tend, p=None, mod=None, precharged=True, pd_fixed=-1.0, win_t=0.0, win_ms=2.0,
             ref=((0.0, 1.0),), vcin0=None):
    p = dict(P if p is None else p)
    el = build(p, mod)
    x0, iL0, vC0 = initial_state(el, p, precharged)
    if vcin0 is not None:                    # custom initial voltages of the string capacitors (and snubbers)
        for k in range(4):
            vC0[2 + 5*k] = vcin0[k]
            vC0[2 + 5*k + 1] = vcin0[k]
            vC0[2 + 5*k + 3] = vcin0[k]
    nsteps = int(round(tend/p['h']))
    win_start = int(round(win_t/p['h']))
    win_len = int(round(win_ms*1e-3/p['h']))
    ref_t = np.array([r[0] for r in ref]); ref_v = np.array([r[1]*p['udc_ref'] for r in ref])
    t0 = time.time()
    rec, win, iL, vC, z, hyst, nfact, maxit = simulate(el['R'], el['L'], el['C'], el['S'], el['D'], el['T'],
                                                       prm_vector(p), x0, iL0, vC0, nsteps, win_start, win_len,
                                                       p['Q'], pd_fixed, ref_t, ref_v)
    dtime = time.time() - t0
    np.savez_compressed(os.path.join(HERE, f'case_{name}.npz'), rec=rec, win=win, iL=iL, vC=vC, z=z, hyst=hyst,
                        params=np.array([str(p)]), mod=np.array([str(mod)]))
    print(f'{name}: {tend:.3f} s, {nsteps} steps in {dtime:.1f} s, LU factorizations {nfact}, '
          f'steps without diode convergence {maxit}', flush=True)
    return rec, win


MISMATCH = [dict(n=1.02), dict(Ld1=1.2*P['Ld1'], Ld2=1.2*P['Ld2']), dict(Cin=0.9*P['Cin']), dict(Lm=0.8*P['Lm'])]

if __name__ == '__main__':
    which = sys.argv[1:] or ['smoke']
    for w in which:
        if w == 'smoke':
            run_case('smoke', 0.02, win_t=0.0, win_ms=1.0)
        elif w == 'nominal':          # the model as it is, start from the steady state of the network (DoDC on)
            run_case('nominal', 1.75, win_t=1.746, win_ms=4.0)
        elif w == 'zero_init':        # same, string capacitors at their 0 V targets, battery capacitor at 4.5 kV
            run_case('zero_init', 0.5, precharged=False, win_t=0.0, win_ms=12.0)
        elif w == 'mismatch':         # module 1 turns +2 %, module 2 leakage +20 %, module 3 Cin -10 %, module 4 Lm -20 %
            run_case('mismatch', 1.0, mod=MISMATCH, win_t=0.996, win_ms=4.0)
        elif w.startswith('mm_'):     # one mismatch at a time
            what = {'mm_n1': (0, dict(n=1.02)), 'mm_L2': (1, dict(Ld1=1.2*P['Ld1'], Ld2=1.2*P['Ld2'])),
                    'mm_C3': (2, dict(Cin=0.9*P['Cin'])), 'mm_Lm4': (3, dict(Lm=0.8*P['Lm'])),
                    'mm_L1': (0, dict(Ld1=1.2*P['Ld1'], Ld2=1.2*P['Ld2'])),
                    'mm_L2s': (1, dict(Ld1=1.05*P['Ld1'], Ld2=1.05*P['Ld2'])),
                    'mm_n2': (1, dict(n=1.02))}[w]
            mod = [{} for _ in range(4)]
            mod[what[0]] = what[1]
            run_case(w, 1.0, mod=mod, win_t=0.996, win_ms=4.0)
        elif w == 'split_in_pair':    # identical modules, modules 2 and 3 (same rectifier) start 160 V apart
            v = ocv_pack(P, P['soc0'])/4
            run_case(w, 1.0, vcin0=[v, v + 80, v - 80, v], win_t=0.996, win_ms=4.0)
        elif w == 'split_pairs':      # identical modules, pair 1+4 starts 160 V above pair 2+3
            v = ocv_pack(P, P['soc0'])/4
            run_case(w, 1.0, vcin0=[v + 80, v - 80, v - 80, v + 80], win_t=0.996, win_ms=4.0)
        elif w == 'split_in_pair_ss': # same as split_in_pair but from the operating point (fixed phase shift, no PI)
            v = ocv_pack(P, P['soc0'])/4
            run_case(w, 0.3, vcin0=[v, v + 80, v - 80, v], pd_fixed=0.7823, win_t=0.296, win_ms=4.0)
        elif w == 'split_pairs_ss':   # same as split_pairs but from the operating point (fixed phase shift, no PI)
            v = ocv_pack(P, P['soc0'])/4
            run_case(w, 0.1, vcin0=[v + 80, v - 80, v - 80, v + 80], pd_fixed=0.7823, win_t=0.096, win_ms=4.0)
        elif w == 'catenary_3kV':     # emulator at the nominal catenary voltage (open circuit 3 kV)
            p = dict(P); p['N'] = int(round(P['N']*3000.0/ocv_pack(P, P['soc0'])))
            run_case('catenary_3kV', 0.8, p=p, win_t=0.796, win_ms=4.0)
