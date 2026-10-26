#!/usr/bin/env python3
"""Loop design check of the three-leg interleaved boost (modelization-and-control,
foundations_of_power_electronics/interleaved_boost_converter).

Same hardware as the interleaved buck (0.8 mH per leg, 7.5 mF on the low-voltage side, 7.2 mF
on the DC link, FF450R17ME4 legs at 3 kHz) used in the other direction: 200 V battery on the
low-voltage side, 750 A of input current (150 kW), DC link regulated at 660 V, resistive load
on the DC link.

Averaged model, inductor currents positive from the battery into the legs (boost direction),
d_k = duty of the LOWER switch of leg k (the boost switch; the modulator of the model gets
m_k = 1 - d_k, the duty of the upper switch, and keeps the buck convention Q1 on around the
carrier bottom):
  per leg   L di_k/dt = u_in - RL i_k - (1 - d_k) u_dc
  DC link   Cdc du_dc/dt = sum((1 - d_k) i_k) - u_dc/R_load - i_step
Discrete-time control at ts = 1/(2 fpwm) (double update): duty held one control period (ZOH),
one-step acquisition delay, legs 2 and 3 one more step (sample age + shadow load at their own
carrier extreme add up to one ts); voltage PI on u_dc -> total current reference -> per-leg PI
(reference/3) -> d_k.

Linearization at the operating point (I per leg, M = 1 - D = (u_in - RL I)/U_dc):
  d i_k/dt = -(RL/L) i_k - (M/L) u + (U_dc/L) d_k
  d u/dt   = sum(M i_k - I d_k)/Cdc - u/(R Cdc)
so the plant from the total current to u_dc (inner loops closed) is
  G_ui = (ibez/(Cdc udc_nom)) (M - I RL/U_dc - s I L/U_dc)/(s + 2/(R Cdc)):
a right-half-plane zero at w_z = (U_in - 2 RL I)/(L I) (157 Hz at nominal load, moving up
as 1/I at light load) and the load corner doubled by the power balance (15 Hz). The voltage
loop crossover is kept at about w_z/4.

Prints the open-loop margins of the per-leg current loop (differential and common mode,
1.5 ts and 2.5 ts delay) and of the voltage loop at nominal, 5 % and no load, simulates the
reference ramp from the precharged DC link (u_dc = u_in at t = 0) and a load step on the
nonlinear averaged model. Writes the figures next to this file.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete
import os

here = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ parameters (init_model.m)
udc = 660.0; u_in = 200.0; pwr = 150e3; N = 3
ibez = pwr/u_in; udc_nom = udc; uin_nom = u_in
R_load = udc**2/pwr                              # [Ohm] 2.904: 150 kW at 660 V
fpwm = 3e3; ts = 1/(2*fpwm)
L = 0.8e-3; RL = 6e-3; Cdc = 7.2e-3; Cin = 7.5e-3
f_c_i = 225.0; f_c_u = 40.0                      # crossover targets (current loops; voltage loop at nominal load, about f_z/4)
i_lim = 1.2                                      # [pu of ibez] limit of the total current reference
k_i = udc_nom/(L*ibez)                           # [1/s] duty -> leg current (pu of ibez), U_dc = udc_nom
kp_i = 2*np.pi*f_c_i/k_i; ki_i = kp_i*2*np.pi*f_c_i/5

def op_point(R):
    """operating point with the resistive load R on the DC link regulated at udc: leg current, upper duty M"""
    P = udc**2/R
    I = P/(N*u_in)                               # [A] per leg, first guess (lossless), then with RL
    for _ in range(20):
        I = P/(N*(u_in - RL*I))
    M = (u_in - RL*I)/udc                        # duty of the upper switch, 1 - D
    return I, M

I_nom, M_nom = op_point(R_load)
D_nom = 1 - M_nom
w_z = (M_nom*udc - I_nom*RL)/(I_nom*L)            # [rad/s] RHP zero of the current -> DC-link voltage plant
k_u = ibez*(M_nom - I_nom*RL/udc)/(Cdc*udc_nom)   # [1/s] total current (pu) -> u_dc (pu), low frequency
a_u = 2/(R_load*Cdc)                             # [1/s] load corner seen by the voltage loop (2/R: power balance)
f0_cm = M_nom*np.sqrt(N/(L*Cdc))/2/np.pi         # [Hz] resonance of the common-mode current plant (open loop)

def G_cm_closed_form(s, R):
    """common-mode current plant i_leg/d (same duty on the three legs) with the DC link, closed form of boost_ctrl_param.m"""
    I, M = op_point(R)
    a1 = 1/(R*Cdc)
    b = M*udc_nom/(L*ibez)
    c = M*ibez/(Cdc*udc_nom)
    e = I/(Cdc*udc_nom)
    return (k_i*(s + a1) + N*b*e)/((s + RL/L)*(s + a1) + N*b*c), a1, c, e

# voltage loop design (closed form of private/boost_ctrl_param.m): at s = j*w_u the plant seen by the voltage PI is
# T_i(s)*(c - e/G_cm)/(s + a1) with T_i the closed common-mode current loop of leg 1 (1.5 ts)
w_u = 2*np.pi*f_c_u; s_u = 1j*w_u
G_cm_u, a1_u, c_u, e_u = G_cm_closed_form(s_u, R_load)
L_cm_u = (kp_i + ki_i/s_u)*G_cm_u*np.exp(-1.5*ts*s_u)
T_i_u = L_cm_u/(1 + L_cm_u)
G_ui_u = T_i_u*(c_u - e_u/G_cm_u)/(s_u + a1_u)
kp_u = 1/(abs(G_ui_u)*np.sqrt(1 + 1/25))         # |PI(j w_u)| = kp_u*sqrt(1 + 1/25) with the zero at w_u/5
ki_u = kp_u*w_u/5
pm_u_est = 180 + np.degrees(np.angle((kp_u + ki_u/s_u)*G_ui_u))
f_c_u_noload = kp_u*ibez*M_nom/(Cdc*udc_nom)/2/np.pi    # [Hz] crossover without load (integrator plant, no RHP zero)
dI = u_in*D_nom/(L*fpwm)                         # [A] leg ripple: the inductor sees u_in for D of the period
k = np.floor(N*M_nom)
dI_tot = udc/(L*fpwm)*(M_nom - k/N)*((k+1)/N - M_nom)*N  # [A] ripple of the sum (same interleaving as the buck, duty M)
Dp = N*M_nom - k                                 # duty of the DC-side pulse train at N*fpwm
print(f"operating point: u_in {u_in:.0f} V, u_dc {udc:.0f} V, D (lower switch) = {D_nom:.3f}, M = 1 - D = {M_nom:.4f}, "
      f"i_in {N*I_nom:.0f} A, i_leg {I_nom:.0f} A, i_dc {udc/R_load:.0f} A, R_load {R_load:.3f} Ohm")
print(f"ripple per leg {dI:.0f} A pk-pk ({dI/I_nom*100:.0f} % of the leg current), total {dI_tot:.1f} A pk-pk at {N*fpwm/1e3:.0f} kHz on the "
      f"input capacitor ({dI_tot/(8*N*fpwm*Cin)*1e3:.1f} mV); DC-link capacitor: pulse train duty {Dp:.3f}, "
      f"{I_nom*np.sqrt(Dp*(1-Dp)):.0f} A rms, {Dp*(1-Dp)*I_nom/(N*fpwm*Cdc):.2f} V pk-pk")
print(f"voltage plant: RHP zero {w_z/2/np.pi:.0f} Hz, load corner 2/(R Cdc) = {a_u/2/np.pi:.1f} Hz, k_u = {k_u:.1f} 1/s, "
      f"common-mode resonance of the current plant {f0_cm:.1f} Hz (M*sqrt(3/(L Cdc))/2pi)")
print(f"gains: kp_i = {kp_i:.3f} pu/pu, ki_i = {ki_i:.0f} 1/s, kp_u = {kp_u:.3f} pu/pu, ki_u = {ki_u:.1f} 1/s "
      f"(voltage loop: {f_c_u:.0f} Hz at nominal load with |T_i| = {abs(T_i_u):.2f} of inner loop there, "
      f"phase margin estimate {pm_u_est:.0f} deg; about {f_c_u_noload:.0f} Hz without load)")

# ------------------------------------------------------------------ linearized plant, pu, discrete (ZOH)
def plant(R):
    """discrete (ZOH) state space of the linearized power stage with the load R: states i_k/ibez (boost direction), u_dc/udc_nom;
    inputs d_k (boost duty), extra load current/ibez"""
    I, M = op_point(R)
    A = np.zeros((N+1, N+1)); B = np.zeros((N+1, N)); Bd = np.zeros((N+1, 1))
    for j in range(N):
        A[j, j] = -RL/L
        A[j, N] = -M*udc_nom/(L*ibez)
        B[j, j] = udc_nom/(L*ibez)
        A[N, j] = M*ibez/(Cdc*udc_nom)
        B[N, j] = -I/(Cdc*udc_nom)
    A[N, N] = -1/(R*Cdc)
    Bd[N, 0] = -ibez/(Cdc*udc_nom)
    Ad, Bd_all, Cd, Dd, _ = cont2discrete((A, np.hstack([B, Bd]), np.eye(N+1), np.zeros((N+1, N+1))), ts, method='zoh')
    return Ad, Bd_all[:, :N], Bd_all[:, N:], Cd
Ad, Bdz, Bdl, Cd = plant(R_load)

def freq_resp_ss(Ad, Bd, Cd, w):
    z = np.exp(1j*w*ts)
    n = Ad.shape[0]
    return np.array([Cd @ np.linalg.solve(z_*np.eye(n) - Ad, Bd) for z_ in z])

w = 2*np.pi*np.logspace(0, np.log10(fpwm), 2000)
z = np.exp(1j*w*ts)
pi_i = kp_i + ki_i*ts/(1 - 1/z)          # integrator as the blocks: x(k+1) = x + ki ts e, y = kp e + x(k+1)
pi_u = kp_u + ki_u*ts/(1 - 1/z)

# ---- current loop, open loop: common mode (all legs same duty) and differential mode (inductor only)
H = freq_resp_ss(Ad, Bdz, Cd, w)                     # (nw, N+1, N)
G_cm = H[:, 0, :].sum(axis=1)                        # i1 for a common duty
Ad_diff = np.array([[np.exp(-RL/L*ts)]]); Bd_diff = np.array([[udc/(L*ibez)*(1-np.exp(-RL/L*ts))/(RL/L)]])
G_dm = freq_resp_ss(Ad_diff, Bd_diff, np.eye(1), w)[:, 0, 0]

def margins(Lw, w, label):
    mag = np.abs(Lw); ph = np.unwrap(np.angle(Lw))
    idx = np.where(np.diff(np.sign(mag - 1)) != 0)[0]
    if len(idx) == 0:
        print(f"  {label}: no crossover"); return
    i = idx[-1]                                       # last (highest) crossover
    wc = w[i]; pm = 180 + np.degrees(ph[i])
    idx2 = np.where(np.diff(np.sign(ph + np.pi)) != 0)[0]
    idx2 = idx2[idx2 > i] if len(idx2) else idx2      # phase crossover above the gain crossover
    gm = 1/mag[idx2[0]] if len(idx2) else np.inf
    print(f"  {label}: crossover {wc/2/np.pi:.0f} Hz, phase margin {pm:.0f} deg, gain margin {20*np.log10(gm):.1f} dB"
          + (f" at {w[idx2[0]]/2/np.pi:.0f} Hz" if len(idx2) else ""))
    return wc, pm

print("current loop (per leg PI, one-step acquisition delay):")
for extra, lab in [(1, 'leg 1, 1.5 ts'), (2, 'legs 2-3, 2.5 ts')]:
    dly = z**(-extra)
    margins(pi_i*G_dm*dly, w, f"differential mode (inductor only), {lab}")
    margins(pi_i*G_cm*dly, w, f"common mode (with the DC link), {lab}")

# ---- closed current loop (common mode, leg 1 delay) -> voltage plant -> voltage loop (SISO common-mode approximation)
def voltage_loop(R):
    Ad_, Bdz_, _, Cd_ = plant(R)
    H_ = freq_resp_ss(Ad_, Bdz_, Cd_, w)
    G_cm_ = H_[:, 0, :].sum(axis=1)
    T_i_ = pi_i*G_cm_/z/(1 + pi_i*G_cm_/z)
    Gu_ = H_[:, N, :].sum(axis=1)/G_cm_               # u/i1 with the three legs equal
    return pi_u*T_i_*Gu_/N, G_cm_                     # i_ref -> u_dc
L_u, _ = voltage_loop(R_load)
print("voltage loop (inner loop closed, common mode):")
margins(L_u, w, "u_dc loop, nominal load")
for frac, lab in [(0.05, '5 % load'), (1e-6, 'no load')]:
    L_u_l, G_cm_l = voltage_loop(R_load/frac)
    margins(pi_i*G_cm_l/z, w, f"current loop, common mode, leg 1, {lab}")
    margins(pi_i*G_cm_l/z**2, w, f"current loop, common mode, legs 2-3, {lab}")
    margins(L_u_l, w, f"u_dc loop, {lab}")

# ------------------------------------------------------------------ time simulation: nonlinear averaged model, full discrete control
def simulate(t_end=0.30, t_load=0.15, load_step=0.2, slew=10.0, t_ref_end=0.25, mode_delay=True, nsub=20):
    """reference ramp from the precharged DC link (u_dc = u_in) to udc_nom, load step of load_step*nominal load current at t_load;
    the DC link capacitor and the legs are integrated with RK4 at ts/nsub between the control steps"""
    n = int(t_end/ts)
    iL = np.zeros(N); u = u_in                       # [A], [V]: DC link precharged through the upper diodes
    xi_u = 0.0; x_i = np.zeros(N)
    ref = u_in/udc_nom                               # reference ramped from the input voltage
    d_pipe = [np.zeros(N), np.zeros(N)]
    meas_prev = np.zeros(N+1); meas_prev2 = np.zeros(N+1)
    meas_prev[:N] = iL/ibez; meas_prev[N] = u/udc_nom; meas_prev2[:] = meas_prev
    log = np.zeros((n, 3+N+3))
    def f(x, d, i_extra):
        iLx = x[:N]; ux = x[N]
        diL = (u_in - RL*iLx - (1 - d)*ux)/L
        du = (np.sum((1 - d)*iLx) - ux/R_load - i_extra)/Cdc
        return np.concatenate([diL, [du]])
    for kk in range(n):
        t = kk*ts
        i_meas = np.array([meas_prev[0], meas_prev2[1], meas_prev2[2]]) if mode_delay else meas_prev[:N]
        u_meas = meas_prev[N]
        ref = min(ref + slew*ts, 1.0)
        e_u = ref - u_meas
        xi_u_new = xi_u + ki_u*ts*e_u
        y_u = kp_u*e_u + xi_u_new
        y_sat = min(max(y_u, -i_lim), i_lim)
        xi_u = y_sat - kp_u*e_u if y_sat != y_u else xi_u_new
        i_ref = y_sat
        e_i = i_ref/N - i_meas
        d_new = np.zeros(N)
        for j in range(N):
            xn = x_i[j] + ki_i*ts*e_i[j]
            y = kp_i*e_i[j] + xn
            ys = min(max(y, 0.02), 0.98)
            x_i[j] = ys - kp_i*e_i[j] if ys != y else xn
            d_new[j] = ys
        d_apply = np.array([d_new[0], d_pipe[-1][1], d_pipe[-1][2]]) if mode_delay else d_new
        d_pipe.append(d_new)
        i_extra = load_step*udc/R_load if t >= t_load else 0.0
        x = np.concatenate([iL, [u]])
        h = ts/nsub
        for _ in range(nsub):
            k1 = f(x, d_apply, i_extra); k2 = f(x + h/2*k1, d_apply, i_extra)
            k3 = f(x + h/2*k2, d_apply, i_extra); k4 = f(x + h*k3, d_apply, i_extra)
            x = x + h/6*(k1 + 2*k2 + 2*k3 + k4)
        iL = x[:N]; u = x[N]
        meas_prev2 = meas_prev; meas_prev = np.concatenate([iL/ibez, [u/udc_nom]])
        log[kk] = [t, ref, i_ref, *(iL/ibez), u/udc_nom, d_apply[0], u_meas]
    return log

log = simulate()
t = log[:, 0]
i_step = np.searchsorted(t, 0.15)
u_after = log[i_step:, 3+N]
k_min = np.argmin(u_after)
rec = np.where(np.abs(u_after[k_min:] - 1) < 0.01)[0]
i_end_ramp = np.searchsorted(t, (1 - u_in/udc_nom)/10.0)
print(f"load step +20 % of the nominal load at 0.15 s: u_dc dip {(1 - u_after[k_min])*100:.2f} % after {t[i_step + k_min] - 0.15:.4f} s, back within 1 % after "
      f"{(t[i_step + k_min + rec[0]] - 0.15)*1e3 if len(rec) else float('nan'):.0f} ms; u_dc at the end of the reference ramp ({t[i_end_ramp]*1e3:.0f} ms) {log[i_end_ramp, 3+N]:.3f} pu, "
      f"within 1 % of 1 pu from {t[np.where(np.abs(log[:, 3+N] - 1) < 0.01)[0][0]]*1e3:.0f} ms; max leg current during the ramp {log[:i_step, 3:3+N].max()*ibez:.0f} A, "
      f"max u_dc {log[:, 3+N].max():.3f} pu")
fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
ax[0].plot(t, log[:, 1], 'k--', label='u_dc ref (ramp)'); ax[0].plot(t, log[:, 3+N], label='u_dc')
ax[0].set_ylabel('pu of udc_nom'); ax[0].legend(); ax[0].grid(True)
ax[1].plot(t, log[:, 2]/N, 'k--', label='i_ref/3')
for j in range(N): ax[1].plot(t, log[:, 3+j], label=f'i_L{j+1}')
ax[1].set_ylabel('pu of ibez'); ax[1].legend(); ax[1].grid(True)
ax[2].plot(t, log[:, 4+N], label='d leg 1 (lower switch)'); ax[2].set_ylabel('duty'); ax[2].set_xlabel('t [s]'); ax[2].grid(True); ax[2].legend()
fig.suptitle(f'Interleaved boost: reference ramp from the precharged DC link and +20 % load step at 0.15 s\nkp_i={kp_i:.2f}, ki_i={ki_i:.0f}, kp_u={kp_u:.2f}, ki_u={ki_u:.0f}, L={L*1e3:.1f} mH, Cdc={Cdc*1e3:.1f} mF')
fig.tight_layout(); fig.savefig(os.path.join(here, 'interleaved_boost_step_response.png'), dpi=120)

# Bode of the loops
fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
for Lw, lab in [(pi_i*G_cm/z, 'current loop, common mode, leg 1 (1.5 ts)'), (pi_i*G_cm/z**2, 'current loop, common mode, legs 2-3 (2.5 ts)'),
                (pi_i*G_dm/z**2, 'current loop, differential mode, 2.5 ts'), (L_u, 'voltage loop (inner loop closed), nominal load')]:
    ax[0].semilogx(w/2/np.pi, 20*np.log10(np.abs(Lw)), label=lab)
    ax[1].semilogx(w/2/np.pi, np.degrees(np.unwrap(np.angle(Lw))), label=lab)
ax[0].axhline(0, color='k', lw=0.5); ax[1].axhline(-180, color='k', lw=0.5)
ax[0].set_ylabel('|L| [dB]'); ax[1].set_ylabel('arg L [deg]'); ax[1].set_xlabel('f [Hz]')
ax[0].grid(True, which='both'); ax[1].grid(True, which='both'); ax[0].legend(fontsize=8)
ax[0].set_ylim(-40, 60); ax[1].set_ylim(-360, 0)
fig.suptitle('Interleaved boost: open-loop responses')
fig.tight_layout(); fig.savefig(os.path.join(here, 'interleaved_boost_loops_bode.png'), dpi=120)
print('figures written')

# ------------------------------------------------------------------ sampling instants
# the boost switch Q2_k is on around the top of carrier k (m_k = 1 - d_k: Q1_k on around the bottom), the leg current rises
# during the Q2 pulse: at the top of the carrier (centre of the pulse) and at the bottom (centre of the Q1 interval) the
# current is at its average value, as in the buck; a sample taken at the extremes of carrier 1 only is wrong for legs 2-3
def ripple(phi, D):
    """triangular ripple of one leg, normalized to dI, rising ramp (boost switch on) centred at phi = pi (carrier top)"""
    phi = np.mod(phi - np.pi + np.pi, 2*np.pi) - np.pi      # 0 at the carrier top
    r = np.where(np.abs(phi) <= np.pi*D, 0.5*phi/(np.pi*D), 0.0)
    off = np.abs(phi) > np.pi*D
    r = np.where(off, np.sign(phi)*(0.5 - (np.abs(phi) - np.pi*D)/(2*np.pi*(1-D))), r)
    return r
print("sampling at the extremes of carrier 1 only (the error of the first buck version, same for the boost):")
for Dx in [D_nom, 0.6, 0.467, 0.2]:
    e2 = ripple(2*np.pi/3, Dx); e3 = ripple(4*np.pi/3, Dx)
    print(f"  D = {Dx:.3f}: sample of leg 2 off by {e2:+.2f} dI, leg 3 by {e3:+.2f} dI ({e2*dI:+.0f} A, {e3*dI:+.0f} A), "
          f"sum off by {e2+e3:+.2f} dI -> per-leg loops need the sample at the extreme of the own carrier")
