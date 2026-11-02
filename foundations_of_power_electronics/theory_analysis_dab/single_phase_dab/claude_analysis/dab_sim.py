#!/usr/bin/env python3
"""Switched simulation of single_phase_dab.slx (design check, not part of the model).

Same circuit as the Simscape network of the model, same solver idea as its local solver (backward Euler at a
fixed step tc = 0.625 us, piecewise-linear switches and diodes, inputs held over the step):
  battery_1/2 : cell model of the library (N*ocv(z) + M*h, R0, R1 || C1), 1 mF + 1 uOhm terminal capacitor
  DC side     : LFi_dc1/2 250 uH + 1.96 mOhm (|| 1e-9 S), CFi_dc1/2 3.6 mF + 1 mOhm
  bridges     : full-bridge_mosfet_based_with_thermal_model_zvs: MOSFET (Ideal, Switching) Rds_on 1.04 mOhm /
                Goff 1e-6 S, integral diode 4 V + Rdon_diode_m (= Rds_on, as wired in the model) / 1e-5 S,
                "ZVS" capacitor Czvs = 1.88 nF in series with 8.86 kOhm across each switch
  transformer : Cs1 = Cs2 = 230 uF (+ 1 uOhm) in series, saturable_single_phase_transformer as T model
                (Rd1 + Ld1, Lm || Rfe on the primary, ideal n2/n1 = 1, Rd2 + Ld2)
  control     : dab_single_phase_pwm_modulator (triangle carriers computed at tc, leg B = carrier + pi,
                bridge 2 delayed by phase = u*phi_max, d1 = d2 = 0.5 behind 1/(tc s + 1), dead time 2 us as
                +-dead_time*f on the duty, TRGO at the valley and the peak of the bridge-1 carrier -> 24 kHz),
                ui_ctrl triggered on the rising edge of TRGO: i_meas = iL(LFi_dc1)/Idc_FS, quantized 1/2^11,
                one unit delay; reference Step6 (0.1 -> 1 pu at 0.15 s) -> discrete low-pass tau 20 ms;
                pi_ctrl of the library (kp 0.5, ki 18, ts 41.7 us, back calculation, +-1) + k_ff * reference.
Not simulated: thermal networks and the switching-loss bookkeeping of the MOSFET blocks (no effect on the
circuit), the voltage loop Vout Ctrl (not selected by the Variant Source), the unconnected Steps/LPFs/Sum.

Usage: python3 dab_sim.py <case> [<case> ...]   (numpy + numba; writes case_<name>.npz next to this script)
  cases: smoke, nominal, legacy_start, kff_05, kff_0, fine_tc, step_light, step_full, step_light_fine,
         step_light_k0, step_full_k0, prop_step_light, prop_step_full, proposal, kff_05_fine, proposal_fine,
         step_light_k0_t1200, prop_step_light_t1200
         (see __main__ at the end); figures: python3 dab_plots.py; numbers: python3 dab_analyze.py <case> ...,
         all numbers in design_check_single_phase_dab_output.txt: python3 dab_report.py
"""
import os
import sys
import time
import numpy as np
import numba as nb

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ parameters (values of init_model.m)
P = dict(
    h=0.625e-6,                        # param.glb_time.tc (local solver step, modulator step)
    fpwm=12e3, dead_time=2e-6,         # param.glb_time.fPWM_DAB, dead_time_DAB
    ts=1.0/24e3,                       # param.glb_time.ts_dab (double update)
    # batteries (param.battery.pack_1 = pack_2, after the legacy changes R0/2, R1/2, C1/50)
    N=296, E_1=-1.031, E0=3.485, E1=0.2156, E2=0.0, E3=0.0, Elog=-0.05, alpha=35.0, M=125.0,
    R0=0.057245, R1=0.057245, C1=0.01, Q=233.6448598130841, soc0=0.85, Cb=1e-3, RCb=1e-6,
    # DC side (param.dab.hw)
    Lf=250e-6, RLf=0.0019625, gLf=1e-9, Cf=3.6e-3, RCf=1e-3,
    # series capacitors and transformer (param.dab.hw)
    Cs1=230.25888757508006e-6, Cs2=230.25888757508006e-6, RCs=1e-6,
    Ld1=19.098593171027442e-6, Ld2=19.098593171027442e-6, Rd1=1e-3, Rd2=1e-3, Lm=1e-3, Rfe=1e3, n=1.0,
    # bridges (param.dab.mosfet, mask of the zvs bridge in the model)
    Ron_sw=1.04e-3, Goff_sw=1e-6, Vf_d=4.0, Ron_d=1.04e-3, Goff_d=1e-5,
    Rsn=8864.26592797784, Csn=1.8802083333333334e-09,
    # control (param.dab.ctrl, param.dab.mod, param.dab.bench, param.dab.hw.Idc_FS, param.meas)
    kp=0.5, ki=18.0, Idc_FS=250.0, q=1.0/2**11, k_ff=1.0, phi_max=np.pi/2, d=0.5,
    tau_ref=0.02, i_ref0=0.1, t_step=0.15, i_ref1=1.0,
)

# ------------------------------------------------------------------ circuit description
# nodes: 0 a1 (battery 1 after R0), 1 b1 (battery 1 terminal), 2 p1 (DC link 1 +), 3 A1, 4 B1 (bridge 1 legs,
# uac(+), uac(-)), 5 x1 (Cs1 / transformer u1(+)), 6 m1 (magnetizing node), 7 m2 (ideal secondary), 8 y2
# (transformer u2(+) / Cs2), 9 A2, 10 B2 (bridge 2 legs, uac(+), uac(-) = u2(-)), 11 p2 (DC link 2 +),
# 12 b2 (battery 2 terminal), 13 a2 (battery 2 after R0); ground = battery minus of both sides;
# unknown 14: current of the ideal transformer
NN = 14
NX = 15
GND = -1
# element indexes used by the kernel (order of the lists in build())
L_LF1, L_LD1, L_LM, L_LD2, L_LF2 = 0, 1, 2, 3, 4
C_CF1, C_CS1, C_CS2, C_CF2 = 2, 7, 8, 13


def build(p):
    R = []      # (a, b, G)
    L = []      # (a, b, L, R)
    C = []      # (a, b, C, R)
    S = []      # (c, e)            MOSFET channel
    D = []      # (anode, cathode)  integral diode
    T = []      # (m1, B, m2, ac2, n)
    # side 1: battery_1, LFi_dc1, CFi_dc1
    R.append((0, GND, 1.0/p['R0']))          # R0 (Norton with E/R0 into node a1)
    R.append((0, 1, 1.0/p['R1']))            # R1 of R1 || C1
    C.append((0, 1, p['C1'], 0.0))           # C 0: C1
    C.append((1, GND, p['Cb'], p['RCb']))    # C 1: battery terminal capacitor
    L.append((1, 2, p['Lf'], p['RLf']))      # L 0: LFi_dc1
    R.append((1, 2, p['gLf']))
    C.append((2, GND, p['Cf'], p['RCf']))    # C 2: CFi_dc1
    # bridge 1: Q1 (p1-A1), Q2 (A1-0), Q3 (p1-B1), Q4 (B1-0)
    for (c, e) in ((2, 3), (3, GND), (2, 4), (4, GND)):
        S.append((c, e))
        D.append((e, c))
        C.append((c, e, p['Csn'], p['Rsn']))  # C 3..6: Czvs + Rsnubber_zvs
    # transformer: Cs1 between A1 and u1(+) = x1, u1(-) = B1; Cs2 between u2(+) = y2 and A2, u2(-) = B2
    C.append((3, 5, p['Cs1'], p['RCs']))     # C 7: Cs1
    C.append((8, 9, p['Cs2'], p['RCs']))     # C 8: Cs2
    L.append((5, 6, p['Ld1'], p['Rd1']))     # L 1: Ld1 + Rd1 (current x1 -> m1 = i1 of the transformer)
    L.append((6, 4, p['Lm'], 0.0))           # L 2: Lm
    R.append((6, 4, 1.0/p['Rfe']))
    L.append((8, 7, p['Ld2'], p['Rd2']))     # L 3: Ld2 + Rd2 (current y2 -> m2 = i2 of the transformer)
    T.append((6, 4, 7, 10, p['n']))
    # bridge 2: Q1 (p2-A2), Q2 (A2-0), Q3 (p2-B2), Q4 (B2-0)
    for (c, e) in ((11, 9), (9, GND), (11, 10), (10, GND)):
        S.append((c, e))
        D.append((e, c))
        C.append((c, e, p['Csn'], p['Rsn']))  # C 9..12
    # side 2: CFi_dc2, LFi_dc2, battery_2
    C.append((11, GND, p['Cf'], p['RCf']))   # C 13: CFi_dc2
    L.append((11, 12, p['Lf'], p['RLf']))    # L 4: LFi_dc2 (current p2 -> b2, into battery 2)
    R.append((11, 12, p['gLf']))
    C.append((12, GND, p['Cb'], p['RCb']))   # C 14
    R.append((13, 12, 1.0/p['R1']))
    C.append((13, 12, p['C1'], 0.0))         # C 15
    R.append((13, GND, 1.0/p['R0']))
    return dict(R=np.array(R, dtype=np.float64), L=np.array(L, dtype=np.float64),
                C=np.array(C, dtype=np.float64), S=np.array(S, dtype=np.int64),
                D=np.array(D, dtype=np.int64), T=np.array(T, dtype=np.float64))


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
    # ramp_up_down of dab_single_phase_pwm_modulator
    x = np.sin(2.0*np.pi/period*t - phase - np.pi/2.0)
    if x > 1.0:
        x = 1.0
    if x < -1.0:
        x = -1.0
    return np.arcsin(x)/np.pi + 0.5


@nb.njit(cache=True)
def ocv_pack(Ncell, E_1, E0, E1, E2, E3, Elog, alpha, z):
    zc = min(max(z, 0.0), 1.0)
    return Ncell*(E_1*np.exp(-zc*alpha) + E0 + E1*zc + E2*zc*zc + E3*zc*zc*zc + Elog*np.log(abs(1.0 - zc + 1e-2)))


@nb.njit(cache=True)
def simulate(Rel, Lel, Cel, Sel, Del, Tel, prm, iL0, vC0, nsteps, win_starts, win_len, ref_t, ref_v, opt):
    """prm: see prm_vector(); ref_t/ref_v: piecewise-constant reference before the low-pass filter (pu);
    opt: [use_lpf, u_fixed (< -9: closed loop), lpf_y0 (initial filter state)]"""
    h = prm[0]; fpwm = prm[1]; dt = prm[2]; ts = prm[3]
    Ncell = prm[4]; E_1 = prm[5]; E0 = prm[6]; E1 = prm[7]; E2 = prm[8]; E3 = prm[9]; Elog = prm[10]
    alpha = prm[11]; Mh = prm[12]; R0 = prm[13]; Qah = prm[14]
    Ron_sw = prm[15]; Goff_sw = prm[16]; Vf = prm[17]; Ron_d = prm[18]; Goff_d = prm[19]
    kp = prm[20]; ki = prm[21]; Ifs = prm[22]; qz = prm[23]; kff = prm[24]; phimax = prm[25]; dcyc = prm[26]
    tau_ref = prm[27]
    z1 = prm[28]; z2 = prm[28]
    use_lpf = opt[0] > 0.5
    u_fixed = opt[1]
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
    hyst1 = 0.0
    hyst2 = 0.0
    sw = np.zeros(nS, dtype=np.int64)
    dst = np.zeros(nD, dtype=np.int64)
    x = np.zeros(NX)
    # LU cache
    NCACHE = 1024
    cache_key = -np.ones(NCACHE, dtype=np.int64)
    cache_LU = np.zeros((NCACHE, NX, NX))
    cache_piv = np.zeros((NCACHE, NX), dtype=np.int64)
    I = np.zeros(NX)
    # control
    xi = 0.0             # PI integrator (Unit Delay4 of pi_ctrl)
    meas_delay = 0.0     # Unit Delay3 (feedback)
    lpf_x = 0.0          # Unit Delay of the low-pass filter (input)
    lpf_y = opt[2]       # Unit Delay1 of the low-pass filter (output)
    u = 0.0              # PI output, held between triggers (pu)
    ref = 0.0
    phi_f = 0.0          # 1/(tc s + 1) of the phase input
    d_f = 0.0            # 1/(tc s + 1) of d1 = d2
    trgo_prev = 1        # no trigger at the first evaluation
    n_trig_max = int(nsteps*h/ts*1.05) + 16
    rec = np.zeros((n_trig_max, 32))
    acc = np.zeros(12)
    amax = np.zeros(8)
    nacc = 0
    itrig = 0
    nwin = win_starts.shape[0]
    win = np.zeros((nwin, win_len, 20))
    nfact = 0
    n_over3 = 0          # steps needing more than 3 switch/diode-state iterations (MaxNonlinIter of the model)
    n_noconv = 0
    iref = 0
    rv = ref_v[0]
    for step in range(nsteps):
        t = step*h
        while iref + 1 < ref_t.shape[0] and t >= ref_t[iref + 1]:
            iref += 1
            rv = ref_v[iref]
        # ---- modulator and control at t (x, iL, vC = solution at t)
        r1 = ramp(t, 0.0, period)
        trgo = 1 if ((r1 >= 0.0 and r1 <= th) or (r1 <= 1.0 and r1 >= 1.0 - th)) else 0
        if trgo == 1 and trgo_prev == 0:
            # ui_ctrl (triggered): feedback through quantizer and unit delay
            i_pu = iL[L_LF1]/Ifs
            q_meas = qz*np.floor(i_pu/qz + 0.5)
            fb = meas_delay
            meas_delay = q_meas
            # reference: Step6 -> discrete-time-low-pass-filter (y = ts/tau*x(k-1) + (1-ts/tau)*y(k-1))
            if use_lpf:
                ynew = ts/tau_ref*lpf_x + (1.0 - ts/tau_ref)*lpf_y
                lpf_x = rv
                lpf_y = ynew
                ref = ynew
            else:
                ref = rv
            # pi_ctrl: e = ref - fb; sat1 = sat(kp e + ki ts e + xi); xi = sat1 - kp e; y = sat(kff ref + sat1)
            e = ref - fb
            v = kp*e + ki*ts*e + xi
            s1 = min(max(v, -1.0), 1.0)
            xi = s1 - kp*e
            u = min(max(kff*ref + s1, -1.0), 1.0)
            if u_fixed > -9.0:
                u = u_fixed
            if itrig < n_trig_max:
                rec[itrig, 0] = t
                rec[itrig, 1] = u
                rec[itrig, 2] = ref
                rec[itrig, 3] = fb
                rec[itrig, 4] = xi
                rec[itrig, 5] = iL[L_LF1]
                rec[itrig, 6] = iL[L_LF2]
                rec[itrig, 7] = vC[C_CF1]
                rec[itrig, 8] = vC[C_CF2]
                if nacc > 0:
                    for kk in range(12):
                        rec[itrig, 9 + kk] = acc[kk]/nacc
                for kk in range(8):
                    rec[itrig, 21 + kk] = amax[kk]
                rec[itrig, 29] = z1
                rec[itrig, 30] = z2
                rec[itrig, 31] = nacc
                itrig += 1
            for kk in range(12):
                acc[kk] = 0.0
            for kk in range(8):
                amax[kk] = 0.0
            nacc = 0
        trgo_prev = trgo
        # 1/(tc s + 1) on the phase and on the duty cycles (backward Euler with step tc)
        phi_f = 0.5*(phi_f + u)
        d_f = 0.5*(d_f + dcyc)
        phase = phi_f*phimax
        r2 = ramp(t, np.pi, period)
        r3 = ramp(t, phase, period)
        r4 = ramp(t, phase + np.pi, period)
        dd = min(max(d_f, 0.0), 1.0)
        # polarity = -1: upper switch on below d - dead_time*f, lower switch on above d + dead_time*f
        sw[0] = 1 if r1 <= dd - dtf else 0
        sw[1] = 1 if r1 >= dd + dtf else 0
        sw[2] = 1 if r2 <= dd - dtf else 0
        sw[3] = 1 if r2 >= dd + dtf else 0
        sw[4] = 1 if r3 <= dd - dtf else 0
        sw[5] = 1 if r3 >= dd + dtf else 0
        sw[6] = 1 if r4 <= dd - dtf else 0
        sw[7] = 1 if r4 >= dd + dtf else 0
        # ---- battery sources
        Eb1 = ocv_pack(Ncell, E_1, E0, E1, E2, E3, Elog, alpha, z1) + Mh*hyst1
        Eb2 = ocv_pack(Ncell, E_1, E0, E1, E2, E3, Elog, alpha, z2) + Mh*hyst2
        # ---- history sources
        for k in range(NX):
            I[k] = 0.0
        I[0] += Eb1/R0
        I[13] += Eb2/R0
        for k in range(nL):
            stamp_i(I, int(Lel[k, 0]), int(Lel[k, 1]), kL[k]*iL[k])
        for k in range(nC):
            stamp_i(I, int(Cel[k, 0]), int(Cel[k, 1]), -gC[k]*vC[k])
        # ---- solve with diode state iteration
        converged = False
        nit = 0
        for it in range(40):
            nit = it + 1
            key = 0
            for k in range(nS):
                key = key*2 + sw[k]
            for k in range(nD):
                key = key*2 + dst[k]
            slot = (key ^ (key >> 7) ^ (key >> 13)) % NCACHE
            if cache_key[slot] != key:
                A = cache_LU[slot]
                for i in range(NX):
                    for j in range(NX):
                        A[i, j] = G0[i, j]
                for k in range(nS):
                    g = 1.0/Ron_sw if sw[k] == 1 else Goff_sw
                    stamp_g(A, Sel[k, 0], Sel[k, 1], g)
                for k in range(nD):
                    g = 1.0/Ron_d if dst[k] == 1 else Goff_d
                    stamp_g(A, Del[k, 0], Del[k, 1], g)
                lu_factor(A, cache_piv[slot])
                cache_key[slot] = key
                nfact += 1
            Itot = I.copy()
            for k in range(nD):
                if dst[k] == 1:
                    stamp_i(Itot, Del[k, 0], Del[k, 1], -Vf/Ron_d + Vf*Goff_d)
            lu_solve(cache_LU[slot], cache_piv[slot], Itot, x)
            changed = 0
            worst = -1
            worst_v = 0.0
            for k in range(nD):
                a = Del[k, 0]; c = Del[k, 1]
                va = x[a] if a >= 0 else 0.0
                vc = x[c] if c >= 0 else 0.0
                vd = va - vc
                if dst[k] == 1:
                    idd = (vd - Vf)/Ron_d + Vf*Goff_d
                    if idd < -1e-6:
                        changed += 1
                        if -idd*Ron_d > worst_v:
                            worst_v = -idd*Ron_d
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
                    a = Del[k, 0]; c = Del[k, 1]
                    va = x[a] if a >= 0 else 0.0
                    vc = x[c] if c >= 0 else 0.0
                    vd = va - vc
                    if dst[k] == 1:
                        if (vd - Vf)/Ron_d + Vf*Goff_d < -1e-6:
                            dst[k] = 0
                    else:
                        if vd > Vf + 1e-6:
                            dst[k] = 1
            else:
                dst[worst] = 1 - dst[worst]
        if not converged:
            n_noconv += 1
        if nit > 3:
            n_over3 += 1
        # ---- state update
        for k in range(nL):
            a = int(Lel[k, 0]); b = int(Lel[k, 1])
            va = x[a] if a >= 0 else 0.0
            vb = x[b] if b >= 0 else 0.0
            iL[k] = gL[k]*(va - vb) + kL[k]*iL[k]
        icf1 = 0.0
        icf2 = 0.0
        for k in range(nC):
            a = int(Cel[k, 0]); b = int(Cel[k, 1])
            va = x[a] if a >= 0 else 0.0
            vb = x[b] if b >= 0 else 0.0
            ic = gC[k]*(va - vb - vC[k])
            vC[k] += h/Cel[k, 2]*ic
            if k == C_CF1:
                icf1 = ic
            if k == C_CF2:
                icf2 = ic
        icell1 = (Eb1 - x[0])/R0
        icell2 = (Eb2 - x[13])/R0
        for kb in range(2):
            ic_ = icell1 if kb == 0 else icell2
            zz = z1 if kb == 0 else z2
            zdot = -ic_/(3600.0*Qah)
            if (zz >= 1.0 and zdot > 0) or (zz <= 0.0 and zdot < 0):
                zdot = 0.0
            sgn = 1.0 if ic_ > 0 else (-1.0 if ic_ < 0 else 0.0)
            rate = abs(ic_/(3600.0*Qah))
            if kb == 0:
                z1 += h*zdot
                hyst1 += h*(-rate*hyst1 - rate*sgn)
            else:
                z2 += h*zdot
                hyst2 += h*(-rate*hyst2 - rate*sgn)
        # ---- accumulators (averages over the control interval)
        vdc1 = x[2]; vdc2 = x[11]
        idab1 = iL[L_LF1] - icf1          # into bridge 1 from the DC link
        idab2 = icf2 + iL[L_LF2]          # out of bridge 2 into the DC link
        vab1 = x[3] - x[4]
        vab2 = x[9] - x[10]
        i1 = iL[L_LD1]
        i2o = -iL[L_LD2]                  # transformer secondary current towards bridge 2 (Cs2 -> A2)
        acc[0] += iL[L_LF1]
        acc[1] += iL[L_LF2]
        acc[2] += vdc1
        acc[3] += vdc2
        acc[4] += vdc1*idab1              # DC power into bridge 1
        acc[5] += vdc2*idab2              # DC power out of bridge 2
        acc[6] += i1*i1
        acc[7] += i2o*i2o
        acc[8] += vab1*i1                 # AC power out of bridge 1
        acc[9] += vab2*i2o                # AC power into bridge 2
        acc[10] += x[1]*iL[L_LF1]         # battery 1 terminal power
        acc[11] += x[12]*iL[L_LF2]        # battery 2 terminal power
        nacc += 1
        amax[0] = max(amax[0], abs(iL[L_LF1]))
        amax[1] = max(amax[1], abs(iL[L_LF2]))
        amax[2] = max(amax[2], vdc1)
        amax[3] = max(amax[3], vdc2)
        amax[4] = max(amax[4], abs(i1))
        amax[5] = max(amax[5], abs(i2o))
        amax[6] = max(amax[6], abs(iL[L_LM]))
        amax[7] = max(amax[7], abs(vC[C_CS1]))
        # ---- high resolution windows
        for kw in range(nwin):
            if step >= win_starts[kw] and step < win_starts[kw] + win_len:
                w = step - win_starts[kw]
                win[kw, w, 0] = t
                win[kw, w, 1] = vab1
                win[kw, w, 2] = vab2
                win[kw, w, 3] = i1
                win[kw, w, 4] = i2o
                win[kw, w, 5] = iL[L_LM]
                win[kw, w, 6] = vC[C_CS1]
                win[kw, w, 7] = vC[C_CS2]
                win[kw, w, 8] = vdc1
                win[kw, w, 9] = vdc2
                win[kw, w, 10] = iL[L_LF1]
                win[kw, w, 11] = iL[L_LF2]
                win[kw, w, 12] = idab1
                win[kw, w, 13] = idab2
                win[kw, w, 14] = sw[0] + 2*sw[1] + 4*sw[2] + 8*sw[3]
                win[kw, w, 15] = sw[4] + 2*sw[5] + 4*sw[6] + 8*sw[7]
                win[kw, w, 16] = phase
                win[kw, w, 17] = u
                win[kw, w, 18] = nit
                win[kw, w, 19] = x[2] - x[3]   # voltage across Q1 (bridge 1)
    return rec[:itrig], win, iL, vC, z1, z2, nfact, n_over3, n_noconv


def ocv(p, z):
    return p['N']*(p['E_1']*np.exp(-p['alpha']*z) + p['E0'] + p['E1']*z + p['E2']*z**2 + p['E3']*z**3
                   + p['Elog']*np.log(abs(1 - z + 1e-2)))


def initial_state(el, p, vdc0):
    """battery terminal capacitors at OCV (battery_voltage_init_m), DC links at vdc0 (param.dab.ic.vc_dc1/2,
    legacy 0), snubbers consistent with the switch states at t = 0 (Q1, Q4 on in both bridges)."""
    nL = el['L'].shape[0]
    nC = el['C'].shape[0]
    iL0 = np.zeros(nL)
    vC0 = np.zeros(nC)
    voc = ocv(p, p['soc0'])
    vC0[1] = voc
    vC0[14] = voc
    vC0[C_CF1] = vdc0
    vC0[C_CF2] = vdc0
    # snubbers: C 3..6 bridge 1 (Q1, Q2, Q3, Q4), C 9..12 bridge 2; Q1 and Q4 on -> Q2 and Q3 block vdc
    for base in (3, 9):
        vC0[base + 1] = vdc0
        vC0[base + 2] = vdc0
    return iL0, vC0


def prm_vector(p):
    return np.array([p['h'], p['fpwm'], p['dead_time'], p['ts'], p['N'], p['E_1'], p['E0'], p['E1'], p['E2'],
                     p['E3'], p['Elog'], p['alpha'], p['M'], p['R0'], p['Q'],
                     p['Ron_sw'], p['Goff_sw'], p['Vf_d'], p['Ron_d'], p['Goff_d'],
                     p['kp'], p['ki'], p['Idc_FS'], p['q'], p['k_ff'], p['phi_max'], p['d'], p['tau_ref'],
                     p['soc0']])


def run_case(name, tend, p=None, vdc0=None, win_t=(), win_ms=0.5, ref=None, use_lpf=True, u_fixed=-10.0,
             lpf_y0=0.0):
    p = dict(P if p is None else p)
    el = build(p)
    if vdc0 is None:
        vdc0 = ocv(p, p['soc0'])
    iL0, vC0 = initial_state(el, p, vdc0)
    if ref is None:
        ref = ((0.0, p['i_ref0']), (p['t_step'], p['i_ref1']))
    nsteps = int(round(tend/p['h']))
    win_starts = np.array([int(round(tw/p['h'])) for tw in win_t] or [nsteps + 1], dtype=np.int64)
    win_len = int(round(win_ms*1e-3/p['h']))
    ref_t = np.array([r[0] for r in ref]); ref_v = np.array([r[1] for r in ref])
    opt = np.array([1.0 if use_lpf else 0.0, u_fixed, lpf_y0])
    t0 = time.time()
    rec, win, iL, vC, z1, z2, nfact, n3, nnc = simulate(el['R'], el['L'], el['C'], el['S'], el['D'], el['T'],
                                                         prm_vector(p), iL0, vC0, nsteps, win_starts, win_len,
                                                         ref_t, ref_v, opt)
    dtime = time.time() - t0
    np.savez_compressed(os.path.join(HERE, f'case_{name}.npz'), rec=rec, win=win, win_t=np.array(win_t),
                        iL=iL, vC=vC, z1=z1, z2=z2, params=np.array([str(p)]), h=p['h'])
    print(f'{name}: {tend:.3f} s, {nsteps} steps in {dtime:.1f} s, LU factorizations {nfact}, '
          f'steps with more than 3 switch/diode iterations {n3}, without convergence {nnc}', flush=True)
    return rec, win


if __name__ == '__main__':
    which = sys.argv[1:] or ['smoke']
    for w in which:
        if w == 'smoke':
            run_case('smoke', 0.01, win_t=(0.009,), win_ms=0.5)
        elif w == 'nominal':          # the model as it is now: DC links at OCV, k_ff = 1, bench 0.1 -> 1 pu
            run_case('nominal', 0.6, win_t=(0.1495, 0.5995), win_ms=0.5)
        elif w == 'legacy_start':     # as the legacy model: DC links start at 0 V
            run_case('legacy_start', 0.3, vdc0=0.0, win_t=(0.0, 0.1495), win_ms=0.5)
        elif w == 'kff_05':
            p = dict(P); p['k_ff'] = 0.5
            run_case('kff_05', 0.6, p=p, win_t=(0.1495, 0.5995), win_ms=0.5)
        elif w == 'kff_0':
            p = dict(P); p['k_ff'] = 0.0
            run_case('kff_0', 0.6, p=p, win_t=(0.1495, 0.5995), win_ms=0.5)
        elif w == 'fine_tc':          # tc = T/400 = 0.208 us (integer number of steps per period)
            p = dict(P); p['h'] = 1.0/12e3/400
            run_case('fine_tc', 0.3, p=p, win_t=(0.1495, 0.2995), win_ms=0.5)
        elif w == 'step_light':       # loop check: reference step after the filter, 0.10 -> 0.15 pu
            run_case('step_light', 0.12, ref=((0.0, 0.10), (0.08, 0.15)), use_lpf=False, lpf_y0=0.0,
                     win_t=(0.0795,), win_ms=0.5)
        elif w == 'step_full':        # loop check: 0.90 -> 0.95 pu
            run_case('step_full', 0.4, ref=((0.0, 0.90), (0.35, 0.95)), use_lpf=False,
                     win_t=(0.3495,), win_ms=0.5)
        elif w == 'step_light_fine':  # as step_light with tc = T/400
            p = dict(P); p['h'] = 1.0/12e3/400
            run_case('step_light_fine', 0.12, p=p, ref=((0.0, 0.10), (0.08, 0.15)), use_lpf=False,
                     win_t=(0.0795,), win_ms=0.5)
        elif w in ('step_light_k0', 'step_full_k0', 'prop_step_light', 'prop_step_full'):
            # feedback loop alone (k_ff = 0): model gains, and the proposal kp 0.05, ki 40
            p = dict(P); p['k_ff'] = 0.0
            if w.startswith('prop'):
                p['kp'] = 0.05; p['ki'] = 40.0
            if 'light' in w:
                run_case(w, 0.25, p=p, ref=((0.0, 0.10), (0.15, 0.15)), use_lpf=False, win_t=(0.1495,), win_ms=0.5)
            else:
                run_case(w, 0.5, p=p, ref=((0.0, 0.90), (0.35, 0.95)), use_lpf=False, win_t=(0.3495,), win_ms=0.5)
        elif w == 'proposal':         # bench with k_ff 0.5, kp 0.05, ki 40
            p = dict(P); p['k_ff'] = 0.5; p['kp'] = 0.05; p['ki'] = 40.0
            run_case('proposal', 0.6, p=p, win_t=(0.1495, 0.5995), win_ms=0.5)
        elif w in ('step_light_k0_t1200', 'prop_step_light_t1200'):
            # as step_light_k0 / prop_step_light with tc = T/1200 = 69 ns: phase steps of 0.3 deg instead of 0.9
            p = dict(P); p['k_ff'] = 0.0; p['h'] = 1.0/12e3/1200
            if w.startswith('prop'):
                p['kp'] = 0.05; p['ki'] = 40.0
            run_case(w, 0.25, p=p, ref=((0.0, 0.10), (0.15, 0.15)), use_lpf=False, win_t=(0.1495,), win_ms=0.5)
        elif w in ('kff_05_fine', 'proposal_fine'):   # kff_05 and proposal with tc = T/400 (finer phase steps)
            p = dict(P); p['h'] = 1.0/12e3/400; p['k_ff'] = 0.5
            if w.startswith('proposal'):
                p['kp'] = 0.05; p['ki'] = 40.0
            run_case(w, 0.6, p=p, win_t=(0.1495, 0.5995), win_ms=0.5)
        else:
            raise SystemExit(f'unknown case {w}')
