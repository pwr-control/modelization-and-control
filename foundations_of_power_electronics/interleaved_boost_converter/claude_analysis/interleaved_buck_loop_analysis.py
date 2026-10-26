#!/usr/bin/env python3
"""Loop design check of the three-leg interleaved buck (modelization-and-control,
foundations_of_power_electronics/interleaved_buck_converter).

Discrete-time model at the control period ts = 1/(2 fpwm) (double update):
  plant  x = [i1 i2 i3 u_out], per leg  L di_k/dt = d_k udc - u_out - RL i_k,
         C du_out/dt = sum(i_k) - i_load,  i_load = u_out/R_load (+ step)
  duty   held over one control period (ZOH), one-step acquisition delay (the control
         uses the samples of the previous trigger), legs 2 and 3 with one more step
         (sample age + shadow load at their own carrier extreme add up to one ts)
  control voltage PI -> total current reference -> per-leg PI (reference/3) -> duties

Prints the open-loop margins of the per-leg current loop (common and differential
mode, with 1.5 ts and 2.5 ts delay) and of the voltage loop, and simulates a
reference ramp and a load step. Writes the figures next to this file.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.signal import cont2discrete
from scipy import linalg
import os

here = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ parameters (init_model.m)
# 660 V battery -> 200 V, 750 A (150 kW), three legs at 3 kHz (double update: ts = 166.7 us, the output sees 9 kHz),
# 0.8 mH per leg, 7.5 mF of output capacitance (2026-10-26; the 750 V -> 400 V, 250 kW, 4 kHz version had 1 mH,
# 3.6 mF, 300 Hz current loops). Numbers of init_model.m.
udc = 660.0; u_out = 200.0; pwr = 150e3; N = 3
ibez = pwr/u_out; ubez = u_out; R_load = u_out**2/pwr
fpwm = 3e3; ts = 1/(2*fpwm)
L = 0.8e-3; RL = 6e-3; C = 7.5e-3; RC = 3e-3/3
f_c_i = 225.0; f_c_u = 60.0                      # crossover targets (current loops, voltage loop at nominal load)
kp_i = 2*np.pi*f_c_i*L*ibez/udc; ki_i = kp_i*2*np.pi*f_c_i/5
k_u = ibez/(C*ubez)                              # [1/s] total current (pu) -> u_out (pu), capacitor alone
a_load = 1/(R_load*C)                            # [1/s] corner of the resistive load at nominal power (80 Hz)
k_i = udc/(L*ibez)                               # [1/s] duty -> leg current (pu)
k_v = ubez/(L*ibez)                              # [1/s] output voltage (pu) -> leg current slope
# voltage loop design (same closed form as private/buck_ctrl_param.m of the model): at s = j*w_u the plant seen by the
# voltage PI is T_i(s)*Zc(s), with Zc = k_u/(s + a_load) (capacitor and load) and T_i the closed common-mode current
# loop (all legs with the same duty: the output capacitor reacts and the leg current per unit duty is
# k_i/(s + RL/L + N*k_v*Zc), the loop gain of the current PI is reduced below the LC resonance). |PI*T_i*Zc| = 1 at w_u.
w_u = 2*np.pi*f_c_u; s_u = 1j*w_u
Zc_u = k_u/(s_u + a_load)
G_cm_u = k_i/(s_u + RL/L + N*k_v*Zc_u)
L_cm_u = (kp_i + ki_i/s_u)*G_cm_u*np.exp(-1.5*ts*s_u)
T_i_u = L_cm_u/(1 + L_cm_u)
kp_u = 1/(abs(T_i_u*Zc_u)*np.sqrt(1 + 1/25))     # |PI(j w_u)| = kp_u*sqrt(1 + 1/25) with the zero at w_u/5
ki_u = kp_u*w_u/5
pm_u_est = 180 + np.degrees(np.angle((kp_u + ki_u/s_u)*T_i_u*Zc_u))
f_c_u_noload = kp_u*k_u/2/np.pi                  # [Hz] crossover without load (integrator plant, inner loop ideal)
D = u_out/udc
dI = udc*D*(1-D)/(L*fpwm)
k = np.floor(N*D)
dI_tot = udc/(L*fpwm)*(D - k/N)*((k+1)/N - D)*N
f0 = np.sqrt(N/(L*C))/2/np.pi
zeta = 1/(2*R_load*C*2*np.pi*f0)
print(f"operating point: udc {udc:.0f} V, u_out {u_out:.0f} V, D = {D:.3f}, i_out {ibez:.0f} A, "
      f"i_leg {ibez/N:.0f} A, R_load {R_load:.3f} Ohm")
print(f"ripple per leg {dI:.0f} A pk-pk ({dI/(ibez/N)*100:.0f} % of the leg current), total {dI_tot:.1f} A pk-pk at {N*fpwm/1e3:.0f} kHz, "
      f"u_out ripple {dI_tot/(8*N*fpwm*C)*1e3:.1f} mV")
print(f"LC resonance (N legs, common duty): f0 = {f0:.0f} Hz, zeta at full load = {zeta:.2f}")
print(f"gains: kp_i = {kp_i:.3f} pu/pu, ki_i = {ki_i:.0f} 1/s, kp_u = {kp_u:.3f} pu/pu, ki_u = {ki_u:.1f} 1/s "
      f"(voltage loop: {f_c_u:.0f} Hz at nominal load with |T_i| = {abs(T_i_u):.2f} of inner loop at that frequency, load corner {a_load/2/np.pi:.0f} Hz, "
      f"phase margin estimate {pm_u_est:.0f} deg; up to {f_c_u_noload:.0f} Hz without load)")

# ------------------------------------------------------------------ continuous plant, pu
# states i_k/ibez, u/ubez ; input d_k ; disturbance i_load/ibez
def plant(R):
    """discrete (ZOH) state space of the power stage with the resistive load R: states i_k/ibez, u/ubez; inputs d_k, i_load/ibez"""
    A = np.zeros((N+1, N+1)); B = np.zeros((N+1, N)); Bd = np.zeros((N+1, 1))
    for j in range(N):
        A[j, j] = -RL/L
        A[j, N] = -ubez/(L*ibez)
        B[j, j] = udc/(L*ibez)
        A[N, j] = ibez/(C*ubez)
    A[N, N] = -1/(R*C)
    Bd[N, 0] = -ibez/(C*ubez)
    Ad, Bd_all, Cd, Dd, _ = cont2discrete((A, np.hstack([B, Bd]), np.eye(N+1), np.zeros((N+1, N+1))), ts, method='zoh')
    return Ad, Bd_all[:, :N], Bd_all[:, N:], Cd
Ad, Bdz, Bdl, Cd = plant(R_load)

def freq_resp_ss(Ad, Bd, Cd, w):
    z = np.exp(1j*w*ts)
    n = Ad.shape[0]
    return np.array([Cd @ np.linalg.solve(z_*np.eye(n) - Ad, Bd) for z_ in z])

w = 2*np.pi*np.logspace(0, np.log10(fpwm), 2000)
z = np.exp(1j*w*ts)
pi_i = kp_i + ki_i*ts/(1 - 1/z)          # backward-Euler-like integrator as the blocks: x(k+1) = x + ki ts e, y = kp e + x(k+1)
pi_u = kp_u + ki_u*ts/(1 - 1/z)

# ---- current loop, open loop: common mode (all legs same duty) and differential mode (inductor only)
# common mode: d_k = d for all k -> i_1 ; differential: no coupling through u_out
H = freq_resp_ss(Ad, Bdz, Cd, w)                     # (nw, N+1, N)
G_cm = H[:, 0, :].sum(axis=1)                        # i1 for a common duty
Ad_diff = np.array([[np.exp(-RL/L*ts)]]); Bd_diff = np.array([[udc/(L*ibez)*(1-np.exp(-RL/L*ts))/(RL/L)]])
G_dm = freq_resp_ss(Ad_diff, Bd_diff, np.eye(1), w)[:, 0, 0]

def margins(Lw, w, label):
    mag = np.abs(Lw); ph = np.unwrap(np.angle(Lw))
    idx = np.where(np.diff(np.sign(mag - 1)) != 0)[0]
    if len(idx) == 0:
        print(f"  {label}: no crossover"); return
    i = idx[-1]                                       # last (highest) crossover: below it the loop gain is above 1 again
    wc = w[i]; pm = 180 + np.degrees(ph[i])
    # gain margin: phase = -180
    idx2 = np.where(np.diff(np.sign(ph + np.pi)) != 0)[0]
    gm = 1/mag[idx2[0]] if len(idx2) else np.inf
    print(f"  {label}: crossover {wc/2/np.pi:.0f} Hz, phase margin {pm:.0f} deg, gain margin {20*np.log10(gm):.1f} dB"
          + (f" at {w[idx2[0]]/2/np.pi:.0f} Hz" if len(idx2) else ""))
    return wc, pm

print("current loop (per leg PI, one-step acquisition delay):")
for extra, lab in [(1, 'leg 1, 1.5 ts'), (2, 'legs 2-3, 2.5 ts')]:
    dly = z**(-extra)
    margins(pi_i*G_dm*dly, w, f"differential mode (inductor only), {lab}")
    margins(pi_i*G_cm*dly, w, f"common mode (with the output capacitor), {lab}")

# ---- closed current loop (common mode, leg 1 delay) -> voltage plant -> voltage loop
# lifted discrete closed loop by simulation-based frequency response is heavy; use the SISO common-mode approximation:
# all legs identical with delay 1 step: i_sum/ (i_ref) = N*Lcm/(1+Lcm), u/i_sum from the plant
Lcm = pi_i*G_cm/z
T_i = Lcm/(1 + Lcm)                                   # i1/(i_ref/N)
Gu = freq_resp_ss(Ad, Bdz, Cd, w)[:, N, :].sum(axis=1)/G_cm   # u/i1 (common duty): (u/d)/(i1/d)
L_u = pi_u*T_i*Gu/N                                   # i_ref -> u: the PI output is the total reference, each leg gets i_ref/N -> i1 = T_i*i_ref/N, u = Gu*i1 (Gu = u/i1 with the three legs equal)
print("voltage loop (inner loop closed, common mode):")
margins(L_u, w, "u_out loop, nominal load")
# light load (5 % of the nominal power): the LC resonance is damped by the current loop only
Ad_l, Bdz_l, Bdl_l, Cd_l = plant(20*R_load)
H_l = freq_resp_ss(Ad_l, Bdz_l, Cd_l, w)
G_cm_l = H_l[:, 0, :].sum(axis=1)
T_i_l = pi_i*G_cm_l/z/(1 + pi_i*G_cm_l/z)
Gu_l = H_l[:, N, :].sum(axis=1)/G_cm_l
L_u_l = pi_u*T_i_l*Gu_l/N
margins(pi_i*G_cm_l/z, w, "current loop, common mode, leg 1, 5 % load")
margins(pi_i*G_cm_l/z**2, w, "current loop, common mode, legs 2-3, 5 % load")
margins(L_u_l, w, "u_out loop, 5 % load")
# no load: the LC resonance (f0) is damped by the current loops only; it must stay below f_c_i/2 (see buck_ctrl_param)
Ad_n, Bdz_n, Bdl_n, Cd_n = plant(1e6*R_load)
H_n = freq_resp_ss(Ad_n, Bdz_n, Cd_n, w)
G_cm_n = H_n[:, 0, :].sum(axis=1)
T_i_n = pi_i*G_cm_n/z/(1 + pi_i*G_cm_n/z)
Gu_n = H_n[:, N, :].sum(axis=1)/G_cm_n
L_u_n = pi_u*T_i_n*Gu_n/N
margins(pi_i*G_cm_n/z, w, "current loop, common mode, leg 1, no load")
margins(pi_i*G_cm_n/z**2, w, "current loop, common mode, legs 2-3, no load")
margins(L_u_n, w, "u_out loop, no load")

# ------------------------------------------------------------------ time simulation of the full discrete control
def simulate(t_end=0.25, t_load=0.15, load_step=0.2, slew=1/0.05, mode_delay=True):
    n = int(t_end/ts)
    x = np.zeros(N+1)
    xi_u = 0.0; x_i = np.zeros(N)
    ref = 0.0
    d_cmd = np.zeros(N); d_pipe = [np.zeros(N), np.zeros(N)]     # shadow delays for legs 2,3
    meas_prev = np.zeros(N+1); meas_prev2 = np.zeros(N+1)
    log = np.zeros((n, 3+N+3))
    i_lim = 1.2
    for kk in range(n):
        t = kk*ts
        # acquisitions: previous trigger samples; legs 2,3 one step older (sample age)
        i_meas = np.array([meas_prev[0], meas_prev2[1], meas_prev2[2]]) if mode_delay else meas_prev[:N]
        u_meas = meas_prev[N]
        # voltage loop with ramp reference
        ref = min(ref + slew*ts, 1.0)
        e_u = ref - u_meas
        xi_u_new = xi_u + ki_u*ts*e_u
        y_u = kp_u*e_u + xi_u_new
        y_sat = min(max(y_u, -i_lim), i_lim)
        xi_u = y_sat - kp_u*e_u if y_sat != y_u else xi_u_new
        i_ref = y_sat
        # per leg PI
        e_i = i_ref/N - i_meas
        d_new = np.zeros(N)
        for j in range(N):
            xn = x_i[j] + ki_i*ts*e_i[j]
            y = kp_i*e_i[j] + xn
            ys = min(max(y, 0.02), 0.98)
            x_i[j] = ys - kp_i*e_i[j] if ys != y else xn
            d_new[j] = ys
        # apply: leg 1 now, legs 2,3 at the next step (shadow load, approximated as one step)
        d_apply = np.array([d_new[0], d_pipe[-1][1], d_pipe[-1][2]]) if mode_delay else d_new
        d_pipe.append(d_new)
        # plant step with load disturbance
        i_load_extra = load_step if t >= t_load else 0.0
        x = Ad @ x + Bdz @ d_apply + Bdl[:, 0]*i_load_extra
        meas_prev2 = meas_prev; meas_prev = x.copy()
        log[kk] = [t, ref, i_ref, *x[:N], x[N], d_apply[0], u_meas]
    return log

log = simulate()
t = log[:, 0]
i_step = np.searchsorted(t, 0.15)
u_after = log[i_step:, 3+N]
k_min = np.argmin(u_after)
rec = np.where(np.abs(u_after[k_min:] - 1) < 0.01)[0]
print(f"load step +0.2 pu at 0.15 s: u_out dip {(1 - u_after[k_min])*100:.1f} % after {t[i_step + k_min] - 0.15:.4f} s, back within 1 % after "
      f"{(t[i_step + k_min + rec[0]] - 0.15)*1e3 if len(rec) else float('nan'):.0f} ms; u_out at the end of the 50 ms ramp {log[np.searchsorted(t, 0.05), 3+N]:.2f} pu, "
      f"within 2 % of 1 pu at {t[np.where(np.abs(log[:, 3+N] - 1) < 0.02)[0][0]]*1e3:.0f} ms")
fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
ax[0].plot(t, log[:, 1], 'k--', label='u_out ref (ramp)'); ax[0].plot(t, log[:, 3+N], label='u_out')
ax[0].set_ylabel('pu of ubez'); ax[0].legend(); ax[0].grid(True)
ax[1].plot(t, log[:, 2]/N, 'k--', label='i_ref/3'); 
for j in range(N): ax[1].plot(t, log[:, 3+j], label=f'i_L{j+1}')
ax[1].set_ylabel('pu of ibez'); ax[1].legend(); ax[1].grid(True)
ax[2].plot(t, log[:, 4+N], label='d leg 1'); ax[2].set_ylabel('duty'); ax[2].set_xlabel('t [s]'); ax[2].grid(True); ax[2].legend()
fig.suptitle(f'Interleaved buck: reference ramp (50 ms) and +0.2 pu load step at 0.15 s\nkp_i={kp_i:.2f}, ki_i={ki_i:.0f}, kp_u={kp_u:.2f}, ki_u={ki_u:.0f}, L={L*1e3:.1f} mH, C={C*1e3:.1f} mF')
fig.tight_layout(); fig.savefig(os.path.join(here, 'interleaved_buck_step_response.png'), dpi=120)

# Bode of the loops
fig, ax = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
for Lw, lab in [(pi_i*G_cm/z, 'current loop, common mode, leg 1 (1.5 ts)'), (pi_i*G_cm/z**2, 'current loop, common mode, legs 2-3 (2.5 ts)'),
                (pi_i*G_dm/z**2, 'current loop, differential mode, 2.5 ts'), (L_u, 'voltage loop (inner loop closed)')]:
    ax[0].semilogx(w/2/np.pi, 20*np.log10(np.abs(Lw)), label=lab)
    ax[1].semilogx(w/2/np.pi, np.degrees(np.unwrap(np.angle(Lw))), label=lab)
ax[0].axhline(0, color='k', lw=0.5); ax[1].axhline(-180, color='k', lw=0.5)
ax[0].set_ylabel('|L| [dB]'); ax[1].set_ylabel('arg L [deg]'); ax[1].set_xlabel('f [Hz]')
ax[0].grid(True, which='both'); ax[1].grid(True, which='both'); ax[0].legend(fontsize=8)
ax[0].set_ylim(-40, 60); ax[1].set_ylim(-360, 0)
fig.suptitle('Interleaved buck: open-loop responses')
fig.tight_layout(); fig.savefig(os.path.join(here, 'interleaved_buck_loops_bode.png'), dpi=120)
print('figures written')

# ------------------------------------------------------------------ sampling instants: ripple seen by a sample taken at the extremes of carrier 1
def ripple(phi, D):
    """triangular ripple of one leg, normalized to dI, ON pulse centred at phi = 0 (carrier bottom), phi in [0, 2pi)"""
    phi = np.mod(phi + np.pi, 2*np.pi) - np.pi
    r = np.where(np.abs(phi) <= np.pi*D, 0.5*phi/(np.pi*D), 0.0)
    off = np.abs(phi) > np.pi*D
    r = np.where(off, np.sign(phi)*(0.5 - (np.abs(phi) - np.pi*D)/(2*np.pi*(1-D))), r)
    return r
print("sampling at the extremes of carrier 1 only (the first version of the model):")
for Dx in [0.303, 0.4, 0.533, 0.8]:
    e2 = ripple(2*np.pi/3, Dx); e3 = ripple(4*np.pi/3, Dx)
    print(f"  D = {Dx:.3f}: sample of leg 2 off by {e2:+.2f} dI, leg 3 by {e3:+.2f} dI ({e2*dI:+.0f} A, {e3*dI:+.0f} A), "
          f"sum of the three legs off by {e2+e3:+.2f} dI (the ripple is odd about the pulse centre: the sum is exact, "
          f"the per-leg samples are not) -> per-leg loops need the sample at the extreme of the own carrier")
