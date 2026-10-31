#!/usr/bin/env python3
"""Design check of single_phase_afe.slx (modelization-and-control, foundations_of_power_electronics/single_phase_afe).

Companion of init_model.m: the numbers below are the ones of the init (param.*). The script prints
  1. the ratings and the passive components (DC link, LCL filter, ripple port) against the single-phase
     power pulsation at 2f;
  2. the loop analyses in the frequency domain: grid current loop (dq vector PI seen from the stationary frame,
     LCL plant with the converter-side current), DC-link voltage loop, buffer current loop, buffer voltage loop,
     and the envelope (dq) dynamics of the 100 Hz ripple loop;
  3. an averaged time-domain simulation (no switching, ideal PLL) of the DC-link side with the discrete control
     loops of the model at ts = 100 us: load ramp, ripple loop enabled at 0.5 s, THD of the grid current before
     and after the decoupling,
and writes the figures to ./figures (PDF and PNG). Run: python3 single_phase_afe_design_check.py [--no-sim]
Dependencies: numpy, scipy, matplotlib.
"""
import os, sys, math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, 'figures')
os.makedirs(FIG, exist_ok=True)
plt.rcParams.update({'font.size': 9, 'axes.grid': True, 'grid.alpha': 0.4, 'figure.dpi': 110})


def savefig(fig, name):
    fig.savefig(os.path.join(FIG, name + '.pdf'))
    fig.savefig(os.path.join(FIG, name + '.png'), dpi=150)
    plt.close(fig)


# ============================================================================================================
# 1. parameters (init_model.m)
# ============================================================================================================
f_nom = 50.0; w = 2*math.pi*f_nom
fpwm = 5e3; ts = 1/(2*fpwm); tc = 1e-6; dead_time = 1e-6
U_ll = 690.0                                   # [V] rms grid voltage between the two phases
P_nom = 225e3                                  # [W] AFE rating
P_load = 275e3                                 # [W] load of the model (current source on the DC link)
# legacy bases of the library single-phase data set
us_nom = U_ll*1.35*0.9/math.sqrt(2); is_nom = P_nom/us_nom
udc_nom = 2/math.sqrt(3)*1.1*U_ll*math.sqrt(2)
ubez = us_nom*math.sqrt(2); ibez = is_nom*math.sqrt(2)
# passives
Cdc = 7.2e-3/3; L1 = 250e-6; L2 = 250e-6; R1 = 157*0.05*500e-6/2; R2 = R1; Cf = 200e-6; Rcf = 50e-3
Lt = L1 + L2; Rt = R1 + R2
Cb = 7.2e-3/2; Lb = 2*250e-6; Rb = 2*4e-3; Vb_ref = 2000.0; ib_bez = 750.0
# control
kp_udc, ki_udc, i_ref_lim, tau_udc_ref = 2.0, 35.0, 1.65, 20e-3
kp_i, ki_i, u_lim = 0.5, 45.0, 1.0
fht_delta = 0.05
kp_apd, ki_apd, lim_apd, t_apd = 1.0, 18.0, 1.0, 0.5
kp_bu, ki_bu, lim_bu, tau_b_ref = 2.0, 18.0, 1.0, 20e-3
kp_bi, ki_bi, lim_bi = 1.0, 18.0, 1.0
# observers (closed-form Ackermann, two states, C = [1 0]) as private/afe_ctrl_param
def observer_gain_2x2(A, p):
    a1 = -(p[0] + p[1]); a0 = p[0]*p[1]
    l1 = A[0, 0] + A[1, 1] + a1
    l2 = A[1, 0] + (a0 - (A[0, 0] - l1)*A[1, 1])/A[0, 1]
    return np.array([l1, l2])
poles = np.array([-1, -4])*w
A_di = np.array([[1, ts], [0, 1.0]])
Ld_di = observer_gain_2x2(A_di, np.exp(poles*ts)); ktheta, komega = Ld_di/ts
A_fht = np.array([[0, 1], [-w**2, -fht_delta*w]]); Ad_fht = np.eye(2) + A_fht*ts
Ld_fht = observer_gain_2x2(Ad_fht, np.exp(poles*ts))

U_pk = U_ll*math.sqrt(2); I_nom = P_nom/U_ll; I_pk = I_nom*math.sqrt(2)
I_load_pk = P_load/U_ll*math.sqrt(2)

print('=' * 100)
print('single_phase_afe: design check (numbers of init_model.m)')
print('=' * 100)
print(f'grid {U_ll:.0f} V rms between phases a and b ({U_pk:.1f} V peak), {f_nom:.0f} Hz; PWM {fpwm/1e3:.0f} kHz double update, ts = {ts*1e6:.0f} us')
print(f'bases: ubez = {ubez:.1f} V, ibez = {ibez:.1f} A, udc_nom = {udc_nom:.1f} V  ->  grid voltage {U_pk/ubez:.3f} pu, udc_nom/U_pk = {udc_nom/U_pk:.3f}')
print(f'rating {P_nom/1e3:.0f} kW: {I_nom:.1f} A rms = {I_pk:.1f} A peak = {I_pk/ibez:.3f} pu; model load {P_load/1e3:.0f} kW: {I_load_pk:.1f} A peak = {I_load_pk/ibez:.3f} pu ({P_load/P_nom*100:.0f} % of the rating)')

# ----- 1a. modulation index and LCL
u_conv = math.sqrt(U_pk**2 + (w*Lt*I_pk)**2)
m_nom = u_conv/udc_nom
f_res = math.sqrt((L1 + L2)/(L1*L2*Cf))/2/math.pi
f_lc1 = 1/(2*math.pi*math.sqrt(L1*Cf))
q_cf = U_ll**2*w*Cf
di_sw = udc_nom/(8*fpwm*L1)                     # unipolar modulation: ripple at 2 fpwm, worst case m = 0.5
print(f'converter voltage at {P_nom/1e3:.0f} kW: {u_conv:.0f} V peak -> modulation index {m_nom:.3f} (limit u_lim = {u_lim}); at {P_load/1e3:.0f} kW: {math.sqrt(U_pk**2 + (w*Lt*I_load_pk)**2)/udc_nom:.3f}')
print(f'LCL: L1 = {L1*1e6:.0f} uH, Cf = {Cf*1e6:.0f} uF ({q_cf/P_nom*100:.1f} % of Pnom), L2 = {L2*1e6:.0f} uH; f_res = {f_res:.0f} Hz (fs/6 = {2*fpwm/6:.0f} Hz), f(L1,Cf) = {f_lc1:.0f} Hz; X_Lt = {w*Lt/(ubez/ibez)*100:.1f} % of Zbez')
print(f'converter-side switching ripple (unipolar, 2 fpwm): {di_sw:.0f} A pk-pk worst case = {di_sw/I_pk*100:.0f} % of the peak current')
print(f'dead time {dead_time*1e6:.1f} us vs FF1200R17IP5 td_off + tf = {1.25 + 0.49:.2f} us; dead-time voltage error {dead_time*2*fpwm*udc_nom:.0f} V (unipolar, 2 commutations per period per leg)')

# ----- 1b. pulsating power, DC link and buffer
I_ripple = P_nom/udc_nom
dU = P_nom/(2*w*Cdc*udc_nom)
E_rip = P_nom/w
Vb_max = math.sqrt(Vb_ref**2 + E_rip/Cb); Vb_min = math.sqrt(Vb_ref**2 - E_rip/Cb)
Ib_ac = P_nom/udc_nom
m_b = udc_nom/Vb_ref
uL_b = 2*w*Lb*Ib_ac
Cb_10 = E_rip/(0.5*((1.05*Vb_ref)**2 - (0.95*Vb_ref)**2))
Cb_1500 = E_rip/(0.5*((1.05*1500)**2 - (0.95*1500)**2))
print(f'power pulsation at 2f: {I_ripple:.0f} A amplitude into the DC link, udc ripple {dU:.0f} V = {dU/udc_nom*100:.1f} % (capacitor alone, {Cdc*1e3:.1f} mF), Cdc rms current {I_ripple/math.sqrt(2):.0f} A')
print(f'at {P_load/1e3:.0f} kW: {P_load/udc_nom:.0f} A, {P_load/(2*w*Cdc*udc_nom):.0f} V = {P_load/(2*w*Cdc*udc_nom)/udc_nom*100:.1f} %')
print(f'energy to buffer: {E_rip:.0f} J pk-pk ({P_load/w:.0f} J at {P_load/1e3:.0f} kW); buffer {Cb*1e3:.1f} mF at {Vb_ref:.0f} V = {0.5*Cb*Vb_ref**2:.0f} J -> swing {Vb_min:.0f}-{Vb_max:.0f} V ({(Vb_max - Vb_min)/Vb_ref*100:.1f} %)')
print(f'buffer current {Ib_ac:.0f} A peak at 100 Hz = {Ib_ac/ib_bez:.2f} pu of {ib_bez:.0f} A; modulation index of the ripple port {m_b:.2f}; {uL_b:.0f} V on the {Lb*1e6:.0f} uH at 100 Hz; switching ripple {Vb_ref/(8*fpwm*Lb):.0f} A pk-pk worst case')
print(f'buffer capacitor for a +-5 % swing: {Cb_10*1e3:.2f} mF at {Vb_ref:.0f} V, {Cb_1500*1e3:.2f} mF at 1500 V')
print(f'1700 V class module (FF1200R17IP5) on the ripple port at {Vb_ref:.0f} V: {Vb_ref/1700*100:.0f} % of the blocking voltage')

# ============================================================================================================
# 2. loops in the frequency domain
# ============================================================================================================
def margins(L, f):
    """gain crossover(s) and phase margin, phase crossover and gain margin of the open loop L(jw) on the grid f."""
    mag = np.abs(L); ph = np.unwrap(np.angle(L))
    ph_deg = np.degrees(ph)
    out = {}
    idx = np.where(np.diff(np.sign(mag - 1)) != 0)[0]
    out['fc'] = [f[i] for i in idx]
    out['pm'] = [180 + ph_deg[i] for i in idx]
    # phase crossover: first crossing of -180 (modulo)
    pm180 = np.where(np.diff(np.sign(ph_deg + 180)) != 0)[0]
    out['f180'] = [f[i] for i in pm180]
    out['gm_db'] = [-20*np.log10(mag[i]) for i in pm180]
    return out

f = np.logspace(0, np.log10(5e3), 4000); s = 2j*np.pi*f
delay = np.exp(-1.5*s*ts)                                  # ZOH (ts/2) + one computation step

# ----- 2a. grid current loop: dq PI -> stationary frame PR, plant = LCL converter-side current (grid = short)
def lcl_plant(s):
    Z1 = R1 + s*L1; Z2 = R2 + s*L2; Zc = Rcf + 1/(s*Cf)
    return udc_nom/ibez/(Z1 + Z2*Zc/(Z2 + Zc))             # i1_pu / u (u = PI output in pu of udc_nom)
def l_plant(s):
    return udc_nom/ibez/(Rt + s*Lt)
C_pr = kp_i + ki_i*s/(s**2 + w**2)                         # dq PI of the symmetric (positive-sequence) component
L_i_lcl = C_pr*delay*lcl_plant(s)
L_i_l = C_pr*delay*l_plant(s)
mi = margins(L_i_lcl, f); mi2 = margins(L_i_l, f)
print('-' * 100)
print(f'grid current loop (kp {kp_i}, ki {ki_i}, delay 1.5 ts): crossover {", ".join(f"{x:.0f} Hz" for x in mi["fc"])}, phase margin {", ".join(f"{x:.0f} deg" for x in mi["pm"])} with the LCL; '
      f'{mi2["fc"][0]:.0f} Hz / {mi2["pm"][0]:.0f} deg with L only; gain margin {", ".join(f"{x:.1f} dB at {y:.0f} Hz" for x, y in zip(mi["gm_db"], mi["f180"]))}')
print(f'  loop gain at the LCL resonance {f_res:.0f} Hz: {20*np.log10(np.abs(L_i_lcl[np.argmin(np.abs(f - f_res))])):.1f} dB')
k_i_static = udc_nom/(ibez*Lt); print(f'  static estimate kp*udc_nom/(ibez*Lt)/2pi = {kp_i*k_i_static/2/math.pi:.0f} Hz; PI zero ki/kp = {ki_i/kp_i:.0f} rad/s ({ki_i/kp_i/2/math.pi:.1f} Hz) vs R/L = {Rt/Lt:.1f} rad/s')
wc_i = 2*math.pi*mi2['fc'][0]

# ----- 2b. DC-link voltage loop (plant: integrator through the power balance, current loop as first order)
K_u = 0.5*U_pk*ibez/(Cdc*udc_nom**2)                       # [1/s] d(udc_pu)/dt per pu of current amplitude
T_i = 1/(1 + s/wc_i)
L_u = (kp_udc + ki_udc/s)*K_u/s*T_i*np.exp(-s*ts)
mu = margins(L_u, f)
print(f'DC-link voltage loop (kp {kp_udc}, ki {ki_udc}): plant gain {K_u:.1f} 1/s, crossover {mu["fc"][0]:.1f} Hz, phase margin {mu["pm"][0]:.0f} deg; reference filter {tau_udc_ref*1e3:.0f} ms; PI zero {ki_udc/kp_udc:.1f} rad/s')
g100 = abs(kp_udc + ki_udc/(2j*w))
print(f'  100 Hz ripple through the PI: |PI(j2w)| = {g100:.2f} -> with {dU/udc_nom*100:.1f} % of udc ripple the current amplitude reference carries {g100*dU/udc_nom:.3f} pu at 100 Hz = {g100*dU/udc_nom/(I_pk/ibez)*100:.0f} % of the fundamental -> 3rd harmonic about {g100*dU/udc_nom/(I_pk/ibez)/2*100:.0f} % (and a 50 Hz negative sequence of the same size)')

# ----- 2c. buffer current loop
K_bi = Vb_ref/(ib_bez)                                     # [V/pu]
P_bi = K_bi/(Rb + s*Lb)                                    # i_b_pu per unit of modulation index (pu/pu)
L_bi = (kp_bi + ki_bi/s)*delay*P_bi
mbi = margins(L_bi, f)
print(f'buffer current loop (kp {kp_bi}, ki {ki_bi}, V_buf {Vb_ref:.0f} V, L {Lb*1e6:.0f} uH, base {ib_bez:.0f} A): crossover {mbi["fc"][0]:.0f} Hz, phase margin {mbi["pm"][0]:.0f} deg, gain margin {mbi["gm_db"][0]:.1f} dB at {mbi["f180"][0]:.0f} Hz')
T_bi = L_bi/(1 + L_bi)
i100 = np.argmin(np.abs(f - 2*f_nom))
print(f'  closed loop at 100 Hz: gain {np.abs(T_bi[i100]):.3f}, phase {np.degrees(np.angle(T_bi[i100])):.1f} deg; feed-forward udc_pu = udc/udc_nom while the bridge needs udc/V_buf = {m_b:.2f}: the PI holds {m_b - 1:.2f}')

# ----- 2d. buffer voltage loop (plant: C_b dV/dt = m_b i_b, m_b = udc/V_buf, in pu of udc_nom and ib_bez)
K_bu = m_b*ib_bez/(Cb*udc_nom)                              # [1/s]
wc_bi = 2*math.pi*mbi['fc'][0]
L_bu = (kp_bu + ki_bu/s)*K_bu/s/(1 + s/wc_bi)*np.exp(-s*ts)
mbu = margins(L_bu, f)
print(f'buffer voltage loop (kp {kp_bu}, ki {ki_bu}): plant gain {K_bu:.0f} 1/s, crossover {mbu["fc"][0]:.0f} Hz, phase margin {mbu["pm"][0]:.0f} deg; reference filter {tau_b_ref*1e3:.0f} ms')
print(f'  the loop sees the 100 Hz swing of the buffer voltage ({(Vb_max - Vb_min)/2/udc_nom:.3f} pu amplitude): |PI(j2w)| = {abs(kp_bu + ki_bu/(2j*w)):.2f} -> {abs(kp_bu + ki_bu/(2j*w))*(Vb_max - Vb_min)/2/udc_nom:.3f} pu of 100 Hz current reference added to the ripple loop output (same frequency: the ripple loop absorbs it)')

# ----- 2e. ripple loop: envelope dynamics around 2f (complex-coefficient loop on the dq components)
# chain: I_ref (dq, pu of ib_bez) -> buffer current loop T_bi(s + j2w) -> i_b -> DC link: udc = -i_b/(C s) -> pu
#        -> derivative observer v_hat/w_base: H_v(s) = kv s/(s^2 + kx s + kv) -> quadrature pair (ideal) -> dq
def H_v(sv):
    return komega*sv/(sv**2 + ktheta*sv + komega)
def T_bi_c(sv):
    Lb_ = (kp_bi + ki_bi/sv)*np.exp(-1.5*sv*ts)*K_bi/(Rb + sv*Lb)
    return Lb_/(1 + Lb_)
sb = 2j*np.pi*np.concatenate([-f[::-1], f])                 # baseband frequencies, both signs (complex loop)
# the model applies the BETA output of dq2alphabeta (alpha terminated): a -90 deg rotation of the applied vector
ROT = -1j
G_env = ROT*T_bi_c(sb + 2j*w)*(-ib_bez/(Cdc*(sb + 2j*w)))/udc_nom*H_v(sb + 2j*w)/w
PI_apd = kp_apd + ki_apd/sb
L_env = PI_apd*G_env
g0 = abs(T_bi_c(2j*w)*(ib_bez/(Cdc*2j*w))/udc_nom*H_v(2j*w)/w)
print(f'ripple loop (vector PI {kp_apd}/{ki_apd} on the detected 100 Hz ripple): static gain of the chain {g0:.2f} pu/pu, phase {np.degrees(np.angle(ROT*T_bi_c(2j*w)*(-1/(2j*w))*H_v(2j*w))):.0f} deg (capacitor +90, beta output -90, observer and current loop the rest) '
      f'(observer at 2f: |H_v|/w = {abs(H_v(2j*w))/w:.2f}, {np.degrees(np.angle(H_v(2j*w))):.0f} deg instead of 2 and 90 deg of a true derivative)')
# closed-loop poles of the envelope with the quasi-static complex gain G = g0*exp(j*phi): s*(1 + G*kp) + G*ki = 0
G0 = ROT*T_bi_c(2j*w)*(-ib_bez/(Cdc*2j*w))/udc_nom*H_v(2j*w)/w
p_env = -G0*ki_apd/(1 + G0*kp_apd)
print(f'  envelope pole with the quasi-static chain: {p_env.real:.1f} {p_env.imag:+.1f}j rad/s -> time constant {-1/p_env.real*1e3:.0f} ms, ripple settles in about {-4/p_env.real:.2f} s after the enable at {t_apd} s')
# numerical closed-loop poles including the observer and the current loop (rational approximation of the delay)
import scipy.signal as sig
# polynomial model: PI = (kp s + ki)/s ; T_bi(s+j2w) approximated by 1/(1 + (s+j2w)/wc_bi) ; H_v, capacitor exact
def env_poly_roots():
    # unknown s; build 1 + L(s) = 0 with L = (kp s + ki)/s * K * H_v(s') * (-1/(C s')) * 1/(1 + s'/wc) , s' = s + j2w
    import numpy.polynomial.polynomial as Pn
    j2w = 2j*w
    K = ib_bez/(Cdc*udc_nom*w)
    # numerators/denominators in s (complex coefficients), low -> high
    sp = np.array([j2w, 1.0])                               # s' = s + j2w
    pi_n = np.array([ki_apd, kp_apd]); pi_d = np.array([0, 1.0])
    hv_n = komega*sp; hv_d = Pn.polyadd(Pn.polymul(sp, sp), Pn.polyadd(ktheta*sp, [komega]))
    cap_n = np.array([-1.0]); cap_d = sp
    tb_n = np.array([1.0]); tb_d = Pn.polyadd([1.0], sp/wc_bi)
    num = ROT*K*Pn.polymul(Pn.polymul(pi_n, hv_n), Pn.polymul(cap_n, tb_n))
    den = Pn.polymul(Pn.polymul(pi_d, hv_d), Pn.polymul(cap_d, tb_d))
    char = Pn.polyadd(den, num)
    return np.roots(char[::-1])
r_env = env_poly_roots()
r_dom = r_env[np.argsort(-r_env.real)][:2]
print('  envelope poles with the observer and the buffer current loop dynamics: ' + ', '.join(f'{r.real:.1f}{r.imag:+.1f}j' for r in np.sort_complex(r_env)) + ' rad/s')

# ----- 2f. figure: Bode of the four loops
fig, ax = plt.subplots(2, 4, figsize=(14, 6), sharex=True)
for k, (L, name, mg) in enumerate([(L_i_lcl, 'grid current loop (LCL, converter-side current)', mi),
                                   (L_u, 'DC-link voltage loop', mu),
                                   (L_bi, 'buffer current loop', mbi),
                                   (L_bu, 'buffer voltage loop', mbu)]):
    ax[0, k].semilogx(f, 20*np.log10(np.abs(L)), 'k')
    if k == 0:
        ax[0, k].semilogx(f, 20*np.log10(np.abs(L_i_l)), 'k--', lw=0.8, label='L only')
        ax[0, k].legend(loc='lower left')
    ax[0, k].axhline(0, color='0.5', lw=0.8)
    ax[0, k].set_title(name, fontsize=9)
    ax[0, k].set_ylim(-40, 60)
    ax[1, k].semilogx(f, np.degrees(np.unwrap(np.angle(L))), 'k')
    ax[1, k].axhline(-180, color='0.5', lw=0.8)
    ax[1, k].set_ylim(-360, 90); ax[1, k].set_xlabel('f [Hz]')
    for fc, pm in zip(mg['fc'][:1], mg['pm'][:1]):
        ax[0, k].axvline(fc, color='C3', lw=0.8); ax[1, k].axvline(fc, color='C3', lw=0.8)
        ax[1, k].text(fc*1.1, -330, f'fc = {fc:.0f} Hz\nPM = {pm:.0f} deg', fontsize=8, color='C3')
ax[0, 0].set_ylabel('|L| [dB]'); ax[1, 0].set_ylabel('phase [deg]')
fig.tight_layout(); savefig(fig, 'single_phase_afe_loops_bode')

# ============================================================================================================
# 3. averaged time-domain simulation
# ============================================================================================================
def simulate(enable_apd=True, t_end=1.25, dt=10e-6, log_every=5):
    """DC-link side averaged model with the discrete control loops of the model.
    States: i_g (converter current towards the grid), udc, i_b (buffer current from the DC link into the ripple
    port), V_b (buffer voltage). Ideal PLL, unipolar PWM as a one-step delay + ZOH, no switching ripple, no LCL
    capacitor. Initial conditions: DC link at the rectified grid peak (diode precharge), buffer precharged at udc,
    reference filters at the initial values (the model starts both references from zero: see the README)."""
    n = int(round(t_end/dt)); nctl = int(round(ts/dt))
    # plant
    i_g = 0.0; udc = U_pk; i_b = 0.0; V_b = udc
    v_conv = 0.0; m_b = udc/V_b
    # controllers
    ref_f = udc/udc_nom; I_udc = 0.0
    fht1 = fht2 = fht_out = 0.0
    Id_i = Iq_i = 0.0; clip = False
    a11 = 1 - 0.5*w**2*ts**2; a12 = ts - 0.5*fht_delta*w*ts**2
    a21 = -w**2*ts + 0.5*w**2*fht_delta*w*ts**2; a22 = 1 - fht_delta*w*ts + 0.5*(-w**2 + (fht_delta*w)**2)*ts**2
    l1, l2 = Ld_fht
    x_o = v_o = 0.0                                          # derivative observer
    a_ps = 1 + 2*math.pi*ts*2*f_nom; b_ps = 1 - 2*math.pi*ts*2*f_nom; ps_u1 = ps_y1 = 0.0
    Id_a = Iq_a = 0.0
    refb_f = V_b/udc_nom; I_bu = 0.0; I_bi = 0.0
    i_ref_amp = 0.0; i_b_ref = 0.0
    N = n//log_every + 1
    log = {k: np.zeros(N) for k in ['t', 'i_g', 'udc', 'i_b', 'V_b', 'i_load', 'i_ref', 'd', 'q', 'i_ac_ref', 'ud', 'uq', 'm_b', 'clip', 'm_g']}
    d_det = q_det = 0.0; i_ac_ref = 0.0; ud_a = uq_a = 0.0; m_g = 0.0
    k_log = 0
    for k in range(n):
        t = k*dt
        theta = w*t
        v_g = U_pk*math.cos(theta)
        i_load = min(P_load/udc_nom, 4*P_load/udc_nom*t)
        if k % nctl == 0:
            # ---------------- control step at ts (measurements at the sampling instant)
            udc_pu = udc/udc_nom; i_pu = i_g/ibez; ib_pu = i_b/ib_bez; Vb_pu = V_b/udc_nom
            # DC-link voltage loop: reference filter, PI with clamping (block dclink_voltage_ctrl)
            ref_f += ts/tau_udc_ref*(1.0 - ref_f)
            e = ref_f - udc_pu
            P_ = kp_udc*e; u = P_ + ki_udc*ts*e + I_udc
            u_sat = max(-i_ref_lim, min(i_ref_lim, u)); I_udc = u_sat - P_
            i_ref_amp = -u_sat                                 # i_xi_pu_ref (inverter convention: negative = rectifying)
            # virtual beta of the current: first harmonic tracker (C code, Taylor-2 state matrix), beta = -x2/w
            if t < 0.0125:
                fht1 = fht2 = fht_out = 0.0
            err = i_pu - fht_out
            s1 = l1*err + a11*fht1 + a12*fht2; s2 = l2*err + a21*fht1 + a22*fht2
            fht_out = fht1; fht1, fht2 = s1, s2
            alpha = i_pu; beta = -fht2/w
            ct, st = math.cos(theta), math.sin(theta)
            i_d = alpha*ct + beta*st; i_q = -alpha*st + beta*ct
            # dq vector PI with the division by udc_pu and the vector clamp (block dq_vector_pi)
            ed = i_ref_amp - i_d; eq = 0.0 - i_q
            if not clip:
                Id_i += ki_i*ts*ed; Iq_i += ki_i*ts*eq
            ud = (kp_i*ed + Id_i)/max(0.25, udc_pu); uq = (kp_i*eq + Iq_i)/max(0.25, udc_pu)
            mag = math.hypot(ud, uq)
            clip = mag > u_lim
            if clip:
                ud *= u_lim/mag; uq *= u_lim/mag
            m_g = ud*ct - uq*st                                # alpha component = modulation index of the bridge
            v_conv_next = m_g*udc
            # ---------------- buffer: voltage loop (block udc_buffer_ctrl)
            refb_f += ts/tau_b_ref*(Vb_ref/udc_nom - refb_f)
            eb = refb_f - Vb_pu
            Pb = kp_bu*eb; ub = Pb + ki_bu*ts*eb + I_bu
            ub_sat = max(-lim_bu, min(lim_bu, ub)); I_bu = ub_sat - Pb
            i_dc_ref = ub_sat
            # ripple loop: derivative observer of udc_pu, quadrature pair at 2f, dq at 2 theta, vector PI
            x_new = x_o + v_o*ts + ts*ktheta*(udc_pu - x_o); v_new = v_o + ts*komega*(udc_pu - x_o)
            x_o, v_o = x_new, v_new
            al = v_o/w
            ps_y = b_ps*ps_y1 + al - a_ps*ps_u1; ps_u1 = al; ps_y1 = ps_y
            be = -ps_y
            c2, s2_ = math.cos(2*theta), math.sin(2*theta)
            d_det = al*c2 + be*s2_; q_det = -al*s2_ + be*c2
            en = 1.0 if (enable_apd and t >= t_apd) else 0.0
            ead = 0.0 - en*d_det; eaq = 0.0 - en*q_det
            Id_a += ki_apd*ts*ead; Iq_a += ki_apd*ts*eaq
            ud_a = kp_apd*ead + Id_a; uq_a = kp_apd*eaq + Iq_a
            maga = math.hypot(ud_a, uq_a)
            if maga > lim_apd:
                ud_a *= lim_apd/maga; uq_a *= lim_apd/maga
                Id_a *= lim_apd/maga; Iq_a *= lim_apd/maga
            # dq -> alphabeta at 2 theta: the model takes the BETA output as i_ac_buffer_ref (alpha is terminated),
            # i.e. the applied vector lags the detected one by 90 deg (it compensates the DC-link capacitor integrator)
            i_ac_ref = ud_a*s2_ + uq_a*c2
            i_b_ref = i_dc_ref + i_ac_ref
            # buffer current PI with the feed-forward udc_pu (block idc_buffer_ctrl: error = i - i_ref, plant gain < 0)
            ei = ib_pu - i_b_ref
            Pi_ = kp_bi*ei; ui = Pi_ + ki_bi*ts*ei + I_bi
            ui_sat = max(-lim_bi, min(lim_bi, ui)); I_bi = ui_sat - Pi_
            m_b_next = max(-lim_bi, min(lim_bi, ui_sat + udc_pu))
            if k > 0:
                v_conv, m_b = v_conv_next, m_b_next           # one-step computation delay (applied at the next step)
            else:
                v_conv, m_b = v_conv_next, m_b_next
        # ---------------- plant (forward Euler at dt)
        di_g = (v_conv - v_g - Rt*i_g)/Lt
        i_dc_bridge = v_conv*i_g/max(udc, 50.0)              # power balance of the grid bridge (inverter convention)
        dudc = (-i_dc_bridge - i_load - i_b)/Cdc
        di_b = (udc - m_b*V_b - Rb*i_b)/Lb
        dV_b = m_b*i_b/Cb
        i_g += dt*di_g; udc += dt*dudc; i_b += dt*di_b; V_b += dt*dV_b
        if k % log_every == 0:
            for key, val in (('t', t), ('i_g', i_g), ('udc', udc), ('i_b', i_b), ('V_b', V_b), ('i_load', i_load),
                             ('i_ref', i_ref_amp), ('d', d_det), ('q', q_det), ('i_ac_ref', i_ac_ref), ('ud', ud_a), ('uq', uq_a), ('m_b', m_b),
                             ('clip', float(clip)), ('m_g', m_g)):
                log[key][k_log] = val
            k_log += 1
    for key in log:
        log[key] = log[key][:k_log]
    return log


def thd(x, t, f0, nh=40):
    """THD (up to harmonic nh) and the amplitude of the harmonics of x over an integer number of periods."""
    T = 1/f0; n_per = int(np.floor((t[-1] - t[0])/T)); dt = t[1] - t[0]
    N = int(round(n_per*T/dt)); xs = x[-N:]
    X = np.fft.rfft(xs)/N*2; fr = np.fft.rfftfreq(N, dt)
    amp = [abs(X[np.argmin(np.abs(fr - h*f0))]) for h in range(1, nh + 1)]
    return math.sqrt(sum(a**2 for a in amp[1:]))/amp[0], np.array(amp), fr, np.abs(X)


if '--no-sim' not in sys.argv:
    print('-' * 100)
    print('averaged simulation (dt 10 us, control at ts, ideal PLL, no switching), ripple loop enabled at %.2f s' % t_apd)
    log = simulate(True)
    t = log['t']
    def win(t0, t1):
        return (t >= t0) & (t < t1)
    w1, w2 = win(0.30, 0.50), win(1.05, 1.25)
    for name, wsel in (('before the decoupling (0.3-0.5 s)', w1), ('with the decoupling (1.05-1.25 s)', w2)):
        udc_w = log['udc'][wsel]; ib_w = log['i_b'][wsel]; vb_w = log['V_b'][wsel]
        th, amp, _, _ = thd(log['i_g'][wsel], t[wsel], f_nom)
        print(f'  {name}: udc {udc_w.mean():.0f} V mean, ripple {(udc_w.max() - udc_w.min())/2:.1f} V amplitude ({(udc_w.max() - udc_w.min())/2/udc_nom*100:.2f} %); '
              f'buffer {vb_w.mean():.0f} V, swing {vb_w.max() - vb_w.min():.0f} V pk-pk, current {np.abs(ib_w).max():.0f} A peak; grid current {amp[0]:.0f} A peak, THD {th*100:.2f} %, '
              f'3rd {amp[2]/amp[0]*100:.2f} %, 5th {amp[4]/amp[0]*100:.2f} %')
    # settling of the ripple after the enable: envelope of the udc ripple per period
    per = int(round(0.01/(t[1] - t[0])))
    env_t = []; env_a = []
    for k0 in range(np.searchsorted(t, 0.45), len(t) - per, per):
        seg = log['udc'][k0:k0 + per]; env_t.append(t[k0]); env_a.append((seg.max() - seg.min())/2)
    env_t = np.array(env_t); env_a = np.array(env_a)
    a0 = env_a[env_t < t_apd].mean(); a_end = env_a[-5:].mean()
    k10 = np.where((env_a < a_end + 0.1*(a0 - a_end)) & (env_t > t_apd))[0]
    t10 = env_t[k10[0]] - t_apd if len(k10) else float('nan')
    print(f'  udc ripple amplitude {a0:.1f} V before, {a_end:.2f} V at the end: 90 % removed {t10*1e3:.0f} ms after the enable')
    for name, wsel in (('before', w1), ('with the decoupling', w2)):
        print(f'  {name}: current controller at the voltage limit u_lim for {log["clip"][wsel].mean()*100:.1f} % of the time, |m| max {np.abs(log["m_g"][wsel]).max():.3f}')
    ud_end = log['ud'][-1]; uq_end = log['uq'][-1]
    print(f'  ripple loop output at the end: |i_ac_ref| = {math.hypot(ud_end, uq_end):.3f} pu = {math.hypot(ud_end, uq_end)*ib_bez:.0f} A (expected {P_load/udc_nom:.0f} A), m_b {log["m_b"][w2].min():.2f}..{log["m_b"][w2].max():.2f}')
    log0 = simulate(False, t_end=0.6)
    th0, amp0, fr0, X0 = thd(log0['i_g'][(log0['t'] >= 0.4) & (log0['t'] < 0.6)], log0['t'][(log0['t'] >= 0.4) & (log0['t'] < 0.6)], f_nom)

    # ----- figures
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ax[0].plot(t, log['udc'], 'k', lw=0.6, label='udc'); ax[0].plot(t, log['V_b'], 'C0', lw=0.6, label='buffer voltage')
    ax[0].axhline(udc_nom, color='0.6', lw=0.6, ls='--'); ax[0].axhline(Vb_ref, color='C0', lw=0.6, ls='--')
    ax[0].set_ylabel('[V]'); ax[0].legend(loc='lower right'); ax[0].set_title('averaged simulation: load ramp to %.0f kW in 0.25 s, ripple loop enabled at %.1f s' % (P_load/1e3, t_apd), fontsize=9)
    ax[1].plot(t, log['i_b'], 'C1', lw=0.5, label='buffer current (into the ripple port)'); ax[1].plot(t, log['i_load'], 'k', lw=0.8, label='load current')
    ax[1].set_ylabel('[A]'); ax[1].legend(loc='upper right')
    ax[2].plot(t, log['i_g'], 'C2', lw=0.4, label='grid current'); ax[2].plot(t, -log['i_ref']*ibez, 'k', lw=0.8, label='amplitude reference (rectifying)')
    ax[2].set_ylabel('[A]'); ax[2].set_xlabel('t [s]'); ax[2].legend(loc='upper right')
    fig.tight_layout(); savefig(fig, 'single_phase_afe_sim_overview')

    fig, ax = plt.subplots(2, 3, figsize=(13, 6))
    for col, (t0, t1, title) in enumerate([(0.44, 0.48, 'before the decoupling'), (0.50, 0.54, 'just after the enable'), (1.20, 1.24, 'steady state with the decoupling')]):
        ws = win(t0, t1)
        ax[0, col].plot(t[ws], log['udc'][ws], 'k', lw=0.8); ax[0, col].set_title(title, fontsize=9); ax[0, col].set_ylabel('udc [V]')
        ax[1, col].plot(t[ws], log['i_b'][ws], 'C1', lw=0.8, label='buffer current'); ax[1, col].plot(t[ws], log['i_g'][ws], 'C2', lw=0.8, label='grid current')
        ax[1, col].set_ylabel('[A]'); ax[1, col].set_xlabel('t [s]')
    ax[1, 0].legend(loc='lower left', fontsize=8)
    fig.tight_layout(); savefig(fig, 'single_phase_afe_sim_zoom')

    fig, ax = plt.subplots(1, 2, figsize=(11, 3.8))
    th1, amp1, _, _ = thd(log['i_g'][w1], t[w1], f_nom); th2, amp2, _, _ = thd(log['i_g'][w2], t[w2], f_nom)
    h = np.arange(2, 16)
    ax[0].bar(h - 0.2, amp1[1:15]/amp1[0]*100, 0.4, label=f'before: THD {th1*100:.1f} %', color='0.3')
    ax[0].bar(h + 0.2, amp2[1:15]/amp2[0]*100, 0.4, label=f'with decoupling: THD {th2*100:.1f} %', color='C2')
    ax[0].set_xlabel('harmonic'); ax[0].set_ylabel('% of the fundamental'); ax[0].legend(); ax[0].set_title('grid current harmonics (averaged model, 2-15)', fontsize=9)
    ax[1].plot(env_t, env_a, 'k.-', lw=0.8); ax[1].axvline(t_apd, color='C3', lw=0.8)
    ax[1].set_xlabel('t [s]'); ax[1].set_ylabel('udc ripple amplitude [V]'); ax[1].set_title('100 Hz ripple of udc per period', fontsize=9)
    fig.tight_layout(); savefig(fig, 'single_phase_afe_sim_thd_ripple')

    fig, ax = plt.subplots(2, 1, figsize=(10, 5.5), sharex=True)
    ws = win(0.45, 1.25)
    ax[0].plot(t[ws], log['d'][ws], 'C0', lw=0.6, label='d'); ax[0].plot(t[ws], log['q'][ws], 'C1', lw=0.6, label='q')
    ax[0].set_ylabel('detected ripple [pu]'); ax[0].legend(); ax[0].set_title('ripple loop: detected dq components of the udc ripple and PI outputs', fontsize=9)
    ax[1].plot(t[ws], log['ud'][ws]*ib_bez, 'C0', lw=0.8, label='ud (A)'); ax[1].plot(t[ws], log['uq'][ws]*ib_bez, 'C1', lw=0.8, label='uq (A)')
    ax[1].plot(t[ws], np.hypot(log['ud'][ws], log['uq'][ws])*ib_bez, 'k', lw=0.8, label='|i_ac ref| (A)')
    ax[1].set_ylabel('[A]'); ax[1].set_xlabel('t [s]'); ax[1].legend()
    fig.tight_layout(); savefig(fig, 'single_phase_afe_sim_ripple_loop')

# ----- figure: the power pulsation and the ideal buffer waveforms (for the note)
tt = np.linspace(0, 0.04, 2000)
# v = U cos(wt), i = I cos(wt): p = U I cos^2(wt) = P (1 + cos(2wt)); the excess p - P goes into the buffer
p = P_nom*(1 + np.cos(2*w*tt)); e_b = P_nom/(2*w)*np.sin(2*w*tt)
vb = np.sqrt(Vb_ref**2 + 2*e_b/Cb); ib = P_nom*np.cos(2*w*tt)/udc_nom
fig, ax = plt.subplots(3, 1, figsize=(7, 6), sharex=True)
ax[0].plot(tt*1e3, U_pk*np.cos(w*tt)/1e3, 'k', lw=0.8, label='grid voltage [kV]'); ax[0].plot(tt*1e3, I_pk*np.cos(w*tt)/1e3, 'C2', lw=0.8, label='grid current [kA]')
ax[0].legend(loc='upper right'); ax[0].set_title('single-phase power pulsation at %.0f kW, unity power factor' % (P_nom/1e3), fontsize=9)
ax[1].plot(tt*1e3, p/1e3, 'k', lw=0.8, label='instantaneous power [kW]'); ax[1].axhline(P_nom/1e3, color='0.5', lw=0.8, ls='--', label='average')
ax[1].legend(loc='upper right')
ax[2].plot(tt*1e3, ib, 'C1', lw=0.8, label='buffer current [A]'); ax2 = ax[2].twinx(); ax2.plot(tt*1e3, vb, 'C0', lw=0.8, label='buffer voltage [V]')
ax[2].set_xlabel('t [ms]'); ax[2].legend(loc='upper left'); ax2.legend(loc='upper right'); ax2.grid(False)
fig.tight_layout(); savefig(fig, 'single_phase_afe_power_pulsation')
print('figures written to', FIG)
