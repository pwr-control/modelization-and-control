"""Linear analysis of the grid_forming_dq current loop (PI + resonant at 2*w0) and of the full
voltage/current structure (pos/neg sequence voltage PIs, FHT, delay). Complex space-vector model in the
positive dq frame: every signal is a complex scalar (d + j q), the plant is balanced so the loop is a
complex-coefficient SISO system. Discrete time, ts = 1/(2 fpwm)."""
import numpy as np
from scipy.linalg import expm
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import sys, json

# ---------------- data (init_model.m / afe_hw_param.m / timing_param.m / three_phase_transformer_param.m)
us_nom = 400.0; pwr_nom = 250e3; f0 = 50.0; w0 = 2*np.pi*f0
is_nom = pwr_nom/np.sqrt(3)/us_nom/0.9
ubez = us_nom*np.sqrt(2/3); ibez = is_nom*np.sqrt(2); zbez = ubez/ibez
udc = 660.0; k_mod = udc/np.sqrt(3)/ubez
fpwm = 4e3; ts = 1/fpwm/2; fs = 1/ts
L1 = 330e-6 + 1e-6; R1 = 157*0.05*330e-6
C = 360e-6; RC = 50e-3
L2 = 80e-6; R2 = 157*0.05*80e-6
i1n = 1e6/np.sqrt(3)/400; Lsig = 400*0.05/np.sqrt(3)/i1n/w0; Rsig = 0.05*1e6/3/i1n**2
kratio = (24/np.sqrt(3))/14
Rload = np.mean(230**2*3/275e3*np.array([0.9, 1.1, 0.95]))*kratio**2
Lload = 1e-3*kratio**2
Lg = L2 + Lsig + Lload; Rg = R2 + Rsig + Rload
L1p = L1/zbez; R1p = R1/zbez; Cp = C*zbez; RCp = RC/zbez
Lgp = Lg/zbez; Rgp = Rg/zbez; L2p = L2/zbez; R2p = R2/zbez
# controller gains of the model
kp_i = 0.75; ki_i = 35.0
kp_u = 0.5; ki_u = 18.0
delta_fht = 0.05

print(f'base: ubez={ubez:.1f} V ibez={ibez:.1f} A zbez={zbez:.4f} Ohm k_mod={k_mod:.4f} ts={ts*1e6:.1f} us')
print(f'plant pu: L1={L1p*1e3:.3f} ms R1={R1p:.4f} C={Cp*1e3:.3f} ms RC={RCp:.4f} Lg={Lgp*1e3:.3f} ms Rg={Rgp:.3f}')
print(f'LC (L1-C) resonance {1/(2*np.pi*np.sqrt(L1p*Cp)):.0f} Hz; fs/6 = {fs/6:.0f} Hz')

# ---------------- plant, stationary frame, complex space vectors, pu
A = np.array([[-(R1p+RCp)/L1p, -1/L1p, RCp/L1p],
              [1/Cp, 0.0, -1/Cp],
              [RCp/Lgp, 1/Lgp, -(RCp+Rgp)/Lgp]])
B = np.array([[k_mod/L1p, 0.0], [0.0, -1/Cp], [0.0, 0.0]])      # inputs: m (modulation index), i_dist (at the capacitor node)
C_i1 = np.array([1.0, 0.0, 0.0])
C_ug = np.array([(1-L2p/Lgp)*RCp, (1-L2p/Lgp), -(1-L2p/Lgp)*RCp + (L2p/Lgp)*Rgp - R2p])
Adq = A - 1j*w0*np.eye(3)
M = expm(np.block([[Adq, B], [np.zeros((2, 5))]])*ts)
Ad, Bd = M[:3, :3], M[:3, 3:]

# ---------------- resonant term variants
def resonator(wr, delta, kr, method):
    Ac = np.array([[0.0, 1.0], [-wr**2, -2*delta*wr]]); Bc = np.array([[0.0], [1.0]]); Cc = np.array([[0.0, 1.0]])
    if method == 'fwd':                     # library: Aresd = I + A ts, Bresd = B ts
        Ar, Br = np.eye(2) + Ac*ts, Bc*ts
    elif method == 'zoh':                   # exact: expm
        Mm = expm(np.block([[Ac, Bc], [np.zeros((1, 3))]])*ts); Ar, Br = Mm[:2, :2], Mm[:2, 2:]
    elif method == 'pdf':                   # grid_forming_ctrl_ver_ii: a = 2 sin(wr ts/2), symplectic, undamped
        a = 2*np.sin(wr*ts/2); Ar = np.array([[1.0, -a], [a, 1-a*a]]); Br = np.array([[ts], [a*ts]]); Cc = np.array([[1.0, 0.0]])
    else:
        raise ValueError(method)
    return Ar, Br*kr, Cc

# ---------------- current loop closed-loop model: states [x(3), d_i, xi, xr(2)], inputs [i_ref, i_dist]
def current_loop(Ar, Br, Cr):
    n = 7
    if np.all(Br == 0):
        Ar = np.zeros((2, 2)); Cr = np.zeros((1, 2))
    ex = np.zeros(n, complex); ex[3] = -1.0; eu = np.array([1.0, 0.0])
    mx = kp_i*ex; mx[4] += 1.0; mx[5] += Cr[0, 0]; mx[6] += Cr[0, 1]; mu = kp_i*eu
    Acl = np.zeros((n, n), complex); Bcl = np.zeros((n, 2), complex)
    Acl[:3, :3] = Ad; Acl[:3, :] += np.outer(Bd[:, 0], mx); Bcl[:3, :] += np.outer(Bd[:, 0], mu); Bcl[:3, 1] += Bd[:, 1]
    Acl[3, :3] = C_i1
    Acl[4, 4] = 1.0; Acl[4, :] += ki_i*ts*ex; Bcl[4, :] += ki_i*ts*eu
    Acl[5:7, 5:7] = Ar; Acl[5:7, :] += np.outer(Br[:, 0], ex); Bcl[5:7, :] += np.outer(Br[:, 0], eu)
    Ccl = np.zeros((1, n), complex); Ccl[0, 0] = 1.0           # output i1
    return Acl, Bcl, Ccl

def Lcurrent(z, Ar, Br, Cr):
    P = C_i1 @ np.linalg.solve(z*np.eye(3) - Ad, Bd[:, 0])
    Cz = kp_i + ki_i*ts/(z - 1) + (Cr @ np.linalg.solve(z*np.eye(2) - Ar, Br))[0, 0]
    return Cz*P/z

def margins(Lfun, nf=20001):
    f = np.linspace(-fs/2*0.999, fs/2*0.999, nf) + 0.0123; z = np.exp(1j*2*np.pi*f*ts)
    L = np.array([Lfun(zz) for zz in z]); mag = np.abs(L); ph = np.angle(L)
    s_m = np.min(np.abs(1 + L))
    pm = []
    for k in range(len(f)-1):
        if (mag[k]-1)*(mag[k+1]-1) < 0:
            pm.append((f[k], np.degrees(np.angle(L[k]))))     # arg L at |L| = 1
    gm = []
    for k in range(len(f)-1):
        a0, a1 = ph[k], ph[k+1]
        if abs(a1 - a0) > np.pi: continue
        if (a0 - np.pi)*(a1 - np.pi) < 0 or (a0 + np.pi)*(a1 + np.pi) < 0:
            gm.append((f[k], 1/mag[k]))
    return f, L, s_m, pm, gm

def modes(Acl, k=8):
    ev = np.linalg.eigvals(Acl); s = np.log(ev)/ts
    order = np.argsort(-np.abs(ev))
    return ev, s, order

def settle(err, ref_amp, tol=0.02):
    env = np.abs(err)/ref_amp
    idx = np.where(env > tol)[0]
    return (idx[-1]+1)*ts if len(idx) else 0.0

# ---------------- full loop: states [x(3), d_i, d_u, xf1, xf2, xi, xr(2), xp, xn] inputs [u_ref, i_dist]
a_, b_ = -w0**2, -delta_fht*w0
Af = np.array([[1 + 0.5*a_*ts**2, ts + 0.5*b_*ts**2], [a_*ts + 0.5*a_*b_*ts**2, 1 + b_*ts + 0.5*(a_ + b_**2)*ts**2]])
Afe = np.eye(2) + np.array([[0, 1], [a_, b_]])*ts                  # forward Euler, as afe_ctrl_param
p = np.exp(ts*np.array([-1, -4])*w0); al1 = -(p[0]+p[1]); al0 = p[0]*p[1]
l1 = Afe[0, 0] + Afe[1, 1] + al1; l2 = Afe[1, 0] + (al0 - (Afe[0, 0] - l1)*Afe[1, 1])/Afe[0, 1]
Lf = np.array([l1, l2])
print(f'FHT gains Ld = [{l1:.4f}, {l2:.4f}]  (poles exp(ts*[-1 -4]*w0))')
for sgn, lab in [(+1, 'pos seq e^{+j w0 t}'), (-1, 'neg seq e^{-j w0 t}')]:
    z = np.exp(1j*sgn*w0*ts)
    # discrete observer: x+ = Af x + Lf (u - x1), uhat = x1, v = x2(+)/w0 -> transfer functions at z
    Hx = np.linalg.solve(z*np.eye(2) - (Af - np.outer(Lf, [1, 0])), Lf)
    uhat = Hx[0]; v = ((Af[1] - Lf[1]*np.array([1, 0])) @ Hx + Lf[1])/w0
    up_ = 0.5*(uhat - 1j*v); un_ = 0.5*(uhat + 1j*v)
    print(f'  FHT with {lab}: uhat = {abs(uhat):.4f}/_{np.degrees(np.angle(uhat)):+.2f}, v = {abs(v):.4f}/_{np.degrees(np.angle(v)):+.2f} -> |u+| = {abs(up_):.4f}, |u-| = {abs(un_):.4f}')

def full_loop(Ar, Br, Cr, kp_n=kp_u, ki_n=ki_u, nsc=True):
    n = 12; I = lambda i: np.eye(n, dtype=complex)[i]
    if np.all(Br == 0):
        Ar = np.zeros((2, 2)); Cr = np.zeros((1, 2))
    x3 = np.eye(n, dtype=complex)[:3]
    r1 = np.exp(-1j*w0*ts); r2 = np.exp(-2j*w0*ts)
    # FHT on the delayed voltage: err_f = d_u - xf1
    ef = I(4) - I(5)
    xf_next = r1*(np.vstack([Af[0, 0]*I(5) + Af[0, 1]*I(6) + Lf[0]*ef, Af[1, 0]*I(5) + Af[1, 1]*I(6) + Lf[1]*ef]))
    uhat = I(5)
    v = (Af[1, 0]*I(5) + Af[1, 1]*I(6) + Lf[1]*ef)/w0
    up = 0.5*(uhat - 1j*v); un = 0.5*(uhat + 1j*v)
    # pos PI: e_p = u_ref - up ; i_ref_p = kp_u e_p + xp
    ep_x = -up; ep_u = np.array([1.0, 0.0])
    irp_x = kp_u*ep_x + I(10); irp_u = kp_u*ep_u
    # neg PI (in the pos frame): i_ref_n = -kp_n un + xn ; xn+ = r2 (xn - ki_n ts un)
    if nsc:
        irn_x = -kp_n*un + I(11); xn_next = r2*(I(11) - ki_n*ts*un)
    else:
        irn_x = np.zeros(n, complex); xn_next = np.zeros(n, complex)
    ir_x = irp_x + irn_x; ir_u = irp_u
    # current loop
    e_x = ir_x - I(3); e_u = ir_u
    m_x = kp_i*e_x + I(7) + Cr[0, 0]*I(8) + Cr[0, 1]*I(9); m_u = kp_i*e_u
    Acl = np.zeros((n, n), complex); Bcl = np.zeros((n, 2), complex)
    Acl[:3, :3] = Ad; Acl[:3, :] += np.outer(Bd[:, 0], m_x); Bcl[:3, :] += np.outer(Bd[:, 0], m_u); Bcl[:3, 1] += Bd[:, 1]
    Acl[3, :3] = C_i1
    Acl[4, :3] = C_ug
    Acl[5:7, :] = xf_next
    Acl[7, :] = I(7) + ki_i*ts*e_x; Bcl[7, :] = ki_i*ts*e_u
    Acl[8:10, :] = Ar @ np.vstack([I(8), I(9)]) + np.outer(Br[:, 0], e_x); Bcl[8:10, :] = np.outer(Br[:, 0], e_u)
    Acl[10, :] = I(10) + ki_u*ts*ep_x; Bcl[10, :] = ki_u*ts*ep_u
    Acl[11, :] = xn_next
    outs = {'i1': I(0), 'ug': C_ug @ x3, 'up': up, 'un': un, 'ir': ir_x, 'm': m_x}
    outs_u = {'i1': np.zeros(2), 'ug': np.zeros(2), 'up': np.zeros(2), 'un': np.zeros(2), 'ir': ir_u, 'm': m_u}
    return Acl, Bcl, outs, outs_u

def simulate(Acl, Bcl, u, x0=None):
    n = Acl.shape[0]; x = np.zeros(n, complex) if x0 is None else x0.copy()
    X = np.zeros((len(u), n), complex)
    for k in range(len(u)):
        X[k] = x; x = Acl @ x + Bcl @ u[k]
    return X

# =====================================================================================================
wr = 2*w0
cases = [('PI only', 0.0, 0.0, 'zoh'),
         ('library fwd Euler d=0.015 kr=45', 0.015, 45.0, 'fwd'),
         ('fwd Euler d=0 kr=70', 0.0, 70.0, 'fwd'),
         ('zoh d=0 kr=35', 0.0, 35.0, 'zoh'),
         ('zoh d=0 kr=70', 0.0, 70.0, 'zoh'),
         ('zoh d=0 kr=105', 0.0, 105.0, 'zoh'),
         ('zoh d=0 kr=140', 0.0, 140.0, 'zoh'),
         ('zoh d=0 kr=210', 0.0, 210.0, 'zoh'),
         ('zoh d=0.015 kr=70', 0.015, 70.0, 'zoh'),
         ('pdf symplectic d=0 kr=70', 0.0, 70.0, 'pdf')]
Tsim = 1.5; N = int(Tsim/ts); t = np.arange(N)*ts
results = {}
print('\n=== current loop (complex dq frame): resonator open-loop eigenvalues, margins, closed-loop modes, neg-seq tracking ===')
for name, d, kr, meth in cases:
    Ar, Br, Cr = resonator(wr, d, kr, meth)
    evr = np.linalg.eigvals(Ar)
    Acl, Bcl, Ccl = current_loop(Ar, Br, Cr)
    f, L, s_m, pm, gm = margins(lambda z: Lcurrent(z, Ar, Br, Cr))
    ev, s, order = modes(Acl)
    zeta = -s.real/np.abs(s)
    # tracking of a negative-sequence reference (e^{-j 2 w0 t}) and closed-loop gain there
    zneg = np.exp(-1j*wr*ts)
    T = (Ccl @ np.linalg.solve(zneg*np.eye(7) - Acl, Bcl[:, 0]))[0]
    u = np.zeros((N, 2), complex); u[:, 0] = 0.2*np.exp(-1j*wr*t)
    X = simulate(Acl, Bcl, u); i1 = X[:, 0]; err = u[:, 0] - X[:, 3]   # error seen by the controller (delayed measurement)
    tset = settle(err[:], 0.2)
    worst = np.argsort(-np.abs(ev))[:3]
    print(f'\n[{name}] resonator |eig| = {np.abs(evr).max():.5f} (f = {np.abs(np.angle(evr)).max()/ts/2/np.pi:.2f} Hz)')
    print(f'  modulus margin min|1+L| = {s_m:.3f};  arg L at |L|=1: ' + ', '.join(f'{ff:+.0f} Hz: {p_:+.0f} deg' for ff, p_ in pm) +
          (';  GM: ' + ', '.join(f'{ff:+.0f} Hz: {g_:.2f}' for ff, g_ in gm) if gm else ''))
    print(f'  closed loop: max|z| = {np.abs(ev).max():.5f}; slowest/least damped modes: ' +
          ', '.join(f'(f={s[i].imag/2/np.pi:+.0f} Hz, sigma={s[i].real:.0f} 1/s, zeta={zeta[i]:.2f})' for i in np.argsort(-np.abs(ev))[:4]))
    print(f'  T(-j2w0) = {np.abs(T):.3f} /_ {np.degrees(np.angle(T)):+.1f} deg;  neg-seq ref step: 2 % settling {tset*1e3:.0f} ms, max |err| {np.abs(err).max()/0.2:.2f}')
    results[name] = dict(f=f, L=L, t=t, err=np.abs(err)/0.2, i1=i1, uref=u[:, 0])

# =====================================================================================================
print('\n=== full loop (pos/neg voltage PIs 0.5/18, FHT, current PI 0.75/35 + resonant) ===')
Tsim2 = 2.0; N2 = int(Tsim2/ts); t2 = np.arange(N2)*ts
full = {}
for name, d, kr, meth, nsc in [('PI only, no nsc', 0.0, 0.0, 'zoh', False), ('PI only, nsc', 0.0, 0.0, 'zoh', True),
                               ('zoh d=0 kr=35, nsc', 0.0, 35.0, 'zoh', True), ('zoh d=0 kr=70, nsc', 0.0, 70.0, 'zoh', True),
                               ('zoh d=0 kr=105, nsc', 0.0, 105.0, 'zoh', True),
                               ('zoh d=0 kr=140, nsc', 0.0, 140.0, 'zoh', True),
                               ('zoh d=0 kr=210, nsc', 0.0, 210.0, 'zoh', True), ('fwd Euler d=0.015 kr=45, nsc', 0.015, 45.0, 'fwd', True),
                               ('zoh d=0.015 kr=70, nsc', 0.015, 70.0, 'zoh', True)]:
    Ar, Br, Cr = resonator(wr, d, kr, meth)
    Acl, Bcl, outs, outs_u = full_loop(Ar, Br, Cr, nsc=nsc)
    ev = np.linalg.eigvals(Acl); s = np.log(ev)/ts; zeta = -s.real/np.abs(s)
    u = np.zeros((N2, 2), complex); u[:, 0] = 1.0
    u[t2 >= 0.5, 1] = 0.3*np.exp(-1j*wr*t2[t2 >= 0.5])          # negative-sequence load current 0.3 pu from 0.5 s
    X = simulate(Acl, Bcl, u)
    un = X @ outs['un']; up = X @ outs['up']; i1 = X[:, 0]; ug = X @ outs['ug']
    Nper = int(round(1/(2*f0)/ts))
    ker = np.ones(Nper)/Nper
    un = np.convolve(ug*np.exp(1j*wr*t2), ker, mode='same')          # actual neg-seq voltage (envelope, pos frame)
    up = np.convolve(ug, ker, mode='same')                           # actual pos-seq voltage (DC in dq)
    sel = t2 >= 0.5
    sel = sel & (t2 <= t2[-1] - Nper*ts)
    un_ss = np.abs(un[-Nper-1]); un_pk = np.abs(un[sel]).max()
    tset = settle(un[sel] - un[-Nper-1], 0.3)
    print(f'\n[{name}] max|z| = {np.abs(ev).max():.5f}; slowest modes: ' +
          ', '.join(f'(f={s[i].imag/2/np.pi:+.0f} Hz, sigma={s[i].real:.0f} 1/s, zeta={zeta[i]:.2f})' for i in np.argsort(-np.abs(ev))[:4]))
    k0 = np.searchsorted(t2, 0.5)
    kb = k0 - Nper
    print(f'  before the disturbance: actual |u-| = {np.abs(un[kb]):.4f}, |u+| = {np.abs(up[kb]):.4f}, |i1| = {np.abs(i1[kb]):.3f} pu; FHT estimate |u-| = {np.abs((X @ outs["un"])[kb]):.4f} (leakage of the positive sequence); start-up u+ 2 % settling {settle(up[t2 < 0.45] - up[kb], 1.0)*1e3:.0f} ms')
    print(f'  neg-seq load current 0.3 pu: actual |u-| peak {un_pk:.3f} pu, final {un_ss:.4f} pu, 2 % settling {tset*1e3:.0f} ms;  |u+| final {np.abs(up[-1]):.4f}')
    full[name] = dict(t=t2, un=np.abs(un), up=np.abs(up), i1=i1)

# =====================================================================================================  plots
fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
for name, lab in [('PI only', 'solo PI (kp 0.75, ki 35)'), ('zoh d=0 kr=70', 'PI + R (2 f0, kr = 70, ZOH esatto)')]:
    r = results[name]; sel = np.abs(r['f']) <= 1500
    ax[0].plot(r['f'][sel], 20*np.log10(np.abs(r['L'][sel])), label=lab)
    ax[1].plot(r['f'][sel], np.degrees(np.angle(r['L'][sel])), label=lab)
ax[0].axhline(0, color='k', lw=0.5); ax[0].set_ylabel('|L| [dB]'); ax[0].grid(True); ax[0].legend()
ax[0].set_title('Anello di corrente in dq (vettore complesso): guadagno d\'anello, f<0 = sequenza negativa')
ax[1].set_ylabel('arg L [deg]'); ax[1].set_xlabel('f nel riferimento dq [Hz]'); ax[1].grid(True); ax[1].set_ylim(-360, 180)
fig.tight_layout(); fig.savefig(sys.argv[1] + '/current_loop_bode.png', dpi=130)

fig, ax = plt.subplots(1, 1, figsize=(9, 4))
for name, lab in [('PI only', 'solo PI'), ('zoh d=0 kr=35', 'kr = 35 (ki)'), ('zoh d=0 kr=70', 'kr = 70 (2 ki)'), ('zoh d=0 kr=140', 'kr = 140 (4 ki)'),
                  ('library fwd Euler d=0.015 kr=45', 'libreria: Eulero in avanti, delta 0.015, kr 45 (instabile)')]:
    r = results[name]; ax.plot(r['t'][r['t'] <= 0.5]*1e3, r['err'][r['t'] <= 0.5], label=lab)
ax.set_xlabel('t [ms]'); ax.set_ylabel('|errore| / |i_ref|'); ax.set_yscale('log'); ax.set_ylim(1e-3, 3); ax.grid(True, which='both'); ax.legend(fontsize=8)
ax.set_title('Anello di corrente: inseguimento di un riferimento di sequenza negativa (2 f0 in dq)')
fig.tight_layout(); fig.savefig(sys.argv[1] + '/current_loop_negseq_tracking.png', dpi=130)

fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
for name, lab in [('PI only, no nsc', 'solo PI, senza anello di seq. negativa'), ('PI only, nsc', 'solo PI + anello seq. neg.'),
                  ('zoh d=0 kr=35, nsc', 'PI+R kr = 35 + anello seq. neg.'), ('zoh d=0 kr=70, nsc', 'PI+R kr = 70 + anello seq. neg.'), ('zoh d=0 kr=140, nsc', 'PI+R kr = 140 + anello seq. neg.')]:
    r = full[name]; sl = (r['t'] >= 0.45) & (r['t'] <= 1.3)
    ax[0].plot((r['t'][sl]-0.5)*1e3, r['un'][sl], label=lab); ax[1].plot((r['t'][sl]-0.5)*1e3, r['up'][sl], label=lab)
ax[0].set_ylabel('|u-| reale [pu]'); ax[0].grid(True); ax[0].legend(fontsize=8)
ax[0].set_title('Anello completo (lineare): corrente di carico di sequenza negativa 0.3 pu inserita a t = 0')
ax[1].set_ylabel('|u+| [pu]'); ax[1].set_xlabel('t [ms]'); ax[1].grid(True)
fig.tight_layout(); fig.savefig(sys.argv[1] + '/full_loop_negseq_load.png', dpi=130)
print('\nplots written')
