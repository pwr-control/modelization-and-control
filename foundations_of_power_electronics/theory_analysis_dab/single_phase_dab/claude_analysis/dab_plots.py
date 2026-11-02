#!/usr/bin/env python3
"""Figures of the single_phase_dab design check (reads case_*.npz of dab_sim.py, sweep.npz of dab_sweep.py and
loop.npz of dab_loop.py). Writes figures/*.pdf (the repository ignores *.png; a png copy is written for a
quick look).

Usage: python3 dab_plots.py
"""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'figures')
os.makedirs(OUT, exist_ok=True)
S = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']        # categorical slots 1-4, fixed order
INK, INK2, GRID, SURF = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
plt.rcParams.update({'font.size': 9, 'axes.edgecolor': INK2, 'axes.labelcolor': INK, 'xtick.color': INK2,
                     'ytick.color': INK2, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6,
                     'axes.spines.top': False, 'axes.spines.right': False, 'lines.linewidth': 1.4,
                     'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF,
                     'legend.frameon': False, 'axes.titlesize': 9, 'axes.titleweight': 'bold'})
IFS = 250.0
FPWM = 12e3
# columns of rec (dab_analyze.py)
T, U, REF, FB, XI, IL1, IL2, VC1, VC2 = range(9)
A_IL1 = 9
M_IL1, M_IL2, M_VDC1, M_VDC2, M_I1 = 21, 22, 23, 24, 25
NACC = 31


def have(name):
    return os.path.exists(os.path.join(HERE, name))


def load(name):
    d = np.load(os.path.join(HERE, f'case_{name}.npz'))
    return d['rec'], d['win'], float(d['h'])


def save(fig, name):
    fig.savefig(os.path.join(OUT, name + '.pdf'))
    fig.savefig(os.path.join(OUT, name + '.png'), dpi=150)
    plt.close(fig)
    print('figures/' + name + '.pdf')


def full_period(rec, col):
    """average over pairs of control intervals (one PWM period), weighted by the number of steps"""
    n = rec[1:, NACC]
    w = rec[1:, col]*n
    return rec[2:, T], (w[1:] + w[:-1])/np.maximum(n[1:] + n[:-1], 1)


# ---------------------------------------------------------------- 1. bench test, feed-forward gain
fig, ax = plt.subplots(2, 1, figsize=(7.0, 5.6), sharex=True)
for k, (case, lab) in enumerate((('nominal', 'k_ff = 1 (model now)'), ('kff_05', 'k_ff = 0.5'),
                                 ('kff_0', 'k_ff = 0'))):
    rec, _, _ = load(case)
    t, il = full_period(rec, A_IL1)
    m = t >= 0.1
    ls = ('-', '-', '--')[k]
    ax[0].plot(t[m], il[m]/IFS, color=S[k], ls=ls, label=lab)
    m2 = rec[:, T] >= 0.1
    ax[1].plot(rec[m2, T], rec[m2, U], color=S[k], ls=ls, lw=1.0, label=lab)
    if k == 0:
        ax[0].plot(rec[m2, T], rec[m2, REF], color=INK2, lw=0.9, ls=':', label='reference after the 20 ms filter')
ax[0].set_ylabel('pu of 250 A'); ax[0].set_ylim(0, 1.25)
ax[0].set_title('Battery-1 current iL(LFi_dc1), average over one PWM period; reference 0.1 -> 1 pu at 150 ms')
ax[0].legend(loc='lower right')
ax[1].set_ylabel('u (1 = 90 deg)'); ax[1].set_xlabel('time [s]')
ax[1].set_title('Controller output u = k_ff * reference + PI')
ax[1].legend(loc='lower right')
fig.tight_layout(); save(fig, 'bench')

# ---------------------------------------------------------------- 2. transformer waveforms, light and full load
rec, win, h = load('nominal')
fig, ax = plt.subplots(2, 2, figsize=(8.6, 5.6), sharex='col')
for c, (kw, title) in enumerate(((0, 'Light load, 0.1 pu (u 0.105, 9.4 deg)'), (1, 'Full load, 1 pu (u 0.518, 46.6 deg)'))):
    w = win[kw]
    w = w[w[:, 0] > 0]
    n2 = int(round(2/FPWM/h))
    w = w[-n2:]
    tt = (w[:, 0] - w[0, 0])*1e6
    ax[0, c].plot(tt, w[:, 1], color=S[0], label='v_ab bridge 1')
    ax[0, c].plot(tt, w[:, 2], color=S[1], ls='--', label='v_ab bridge 2')
    ax[0, c].set_title(title)
    ax[1, c].plot(tt, w[:, 3], color=S[0], label='primary current i1')
    ax[1, c].plot(tt, w[:, 5], color=S[2], ls='--', label='magnetizing current')
    ax[1, c].set_xlabel('time [us]')
ax[0, 0].set_ylabel('V'); ax[1, 0].set_ylabel('A')
ax[0, 0].legend(loc='lower left', fontsize=8); ax[1, 0].legend(loc='lower left', fontsize=8)
fig.tight_layout(); save(fig, 'waveforms')

# ---------------------------------------------------------------- 3. static characteristic and losses
s = np.load(os.path.join(HERE, 'sweep.npz'))
us = s['us']
i_h = s['tc625_P1_harm']/s['tc625_vdc1']/IFS
runs = (('tc625', 'model: tc 0.625 us, dead time 2 us', S[0], '-', 'o'),
        ('tc208', 'tc 0.208 us, dead time 2 us', S[1], '--', 's'),
        ('dt05', 'tc 0.069 us, dead time 0.5 us', S[2], '-.', '^'))
fig, ax = plt.subplots(1, 3, figsize=(10.4, 3.7))
for a, lim in ((ax[0], (0, 1)), (ax[1], (0, 0.25))):
    a.plot(us, i_h, color=INK2, lw=1.0, ls=':', label='harmonic model (no dead time)')
    for lab, txt, col, ls, mk in runs:
        a.plot(us, s[f'{lab}_iL1']/IFS, color=col, ls=ls, marker=mk, ms=3, lw=1.2, label=txt)
    a.set_xlim(*lim); a.set_xlabel('u (fixed, open loop)')
ax[1].set_ylim(0, 0.5)
ax[0].set_ylabel('iL1 [pu of 250 A]')
ax[0].set_title('DC current against u'); ax[1].set_title('Zoom: dead zone at light load')
ax[0].legend(loc='lower right', fontsize=7.5)
for lab, txt, col, ls, mk in runs:
    ax[2].plot(us, (s[f'{lab}_Pdc1'] - s[f'{lab}_Pdc2'])/1e3, color=col, ls=ls, marker=mk, ms=3, lw=1.2, label=txt)
ax[2].plot(us, (s['tc625_P1_harm'] - s['tc625_P2_harm'])/1e3, color=INK2, lw=1.0, ls=':',
           label='harmonic model (copper + Rds_on only)')
ax[2].set_xlabel('u (fixed, open loop)'); ax[2].set_ylabel('kW')
ax[2].set_title('P_dc1 - P_dc2 (bridges + transformer)')
ax[2].legend(loc='upper left', fontsize=7.5)
fig.tight_layout(); save(fig, 'static_characteristic')

# ---------------------------------------------------------------- 4. loop gain
L = np.load(os.path.join(HERE, 'loop.npz'))
f = L['w']/2/np.pi
names = [str(x) for x in L['names']]
cn = [str(x) for x in L['ctrl_names']]
fig, ax = plt.subplots(2, 1, figsize=(7.0, 5.8), sharex=True)
for i, (col, lab) in enumerate(((S[0], 'full load'), (S[2], 'light load, harmonic gain'),
                                (S[1], 'light load, switched-model gain'))):
    for j, ls in ((0, '-'), (1, '--')):
        Lg = L[f'L_{j}_{i}']
        ax[0].semilogx(f, 20*np.log10(np.abs(Lg)), color=col, ls=ls, lw=1.2,
                       label=f'{lab}, {"PI of the model" if j == 0 else "kp 0.05, ki 40"}')
        ax[1].semilogx(f, np.degrees(np.unwrap(np.angle(Lg))), color=col, ls=ls, lw=1.2)
for a in ax:
    a.axvline(181.7, color=INK2, lw=0.8, ls=':')
ax[0].axhline(0, color=INK, lw=0.8)
ax[1].axhline(-180, color=INK, lw=0.8)
ax[0].text(190, 22, 'DC filter resonance 182 Hz', fontsize=8, color=INK2)
ax[0].set_ylim(-40, 40); ax[0].set_ylabel('|L| [dB]')
ax[0].set_title('Current-loop gain (broken at the controller output, delays and ZOH included)')
ax[0].legend(loc='upper right', fontsize=7.5, ncol=1)
ax[1].set_ylim(-450, 0); ax[1].set_yticks([-450, -360, -270, -180, -90, 0])
ax[1].set_ylabel('phase [deg]'); ax[1].set_xlabel('frequency [Hz]'); ax[1].set_xlim(1, 12e3)
fig.tight_layout(); save(fig, 'loop_gain')

# ---------------------------------------------------------------- 5. small steps: linear model against switched
def switched_step(case, t_step, dref, col=IL1):
    rec, _, _ = load(case)
    t = rec[:, T]
    pre = rec[(t >= t_step - 0.01) & (t < t_step - 1e-9), col].mean()
    m = (t >= t_step - 2e-3) & (t <= t_step + 0.1)
    return (t[m] - t_step)*1e3, (rec[m, col] - pre)/(dref*IFS)


panels = [
    ('Full load 0.90 -> 0.95 pu, k_ff = 1, PI of the model', 'step_full', 0.35, 'step_0_0_10', None),
    ('Light load 0.10 -> 0.15 pu, k_ff = 1, PI of the model', 'step_light', 0.08, 'step_0_2_10', 'step_0_1_10'),
    ('Light load 0.10 -> 0.15 pu, k_ff = 0, PI of the model', 'step_light_k0', 0.15, 'step_0_2_0', 'step_0_1_0'),
    ('Light load 0.10 -> 0.15 pu, k_ff = 0, kp 0.05, ki 40', 'prop_step_light', 0.15, 'step_1_2_0', 'step_1_1_0'),
]
fig, ax = plt.subplots(2, 2, figsize=(9.0, 5.8))
for a, (title, case, ts_, key, key2) in zip(ax.flat, panels):
    if not have(f'case_{case}.npz'):
        continue
    tt, yy = switched_step(case, ts_, 0.05)
    a.plot(tt, yy, color=S[0], lw=1.0, label='switched simulation')
    st = L[key]
    lab1 = 'linear model, switched-model gain' if key2 else 'linear model'
    a.plot(st[0]*1e3, st[1], color=S[1], ls='--', lw=1.2, label=lab1)
    if key2:
        st2 = L[key2]
        a.plot(st2[0]*1e3, st2[1], color=INK2, ls=':', lw=1.0, label='linear model, harmonic gain')
    a.axhline(1, color=INK, lw=0.6)
    a.set_xlim(-2, 60 if 'k_ff = 0' in title else 30)
    a.set_title(title, fontsize=8.5)
    a.set_xlabel('time after the step [ms]'); a.set_ylabel('iL1 change / reference change')
ax[0, 0].legend(loc='upper right', fontsize=7.5); ax[0, 1].legend(loc='upper right', fontsize=7.5)
fig.tight_layout(); save(fig, 'steps_linear_vs_switched')

# ---------------------------------------------------------------- 6. light-load ripple: PI gains and solver step
pairs = ((('step_light_k0', 'tc 0.625 us (model)', S[0], '-'), ('step_light_k0_t1200', 'tc 0.069 us', S[1], '--')),
         (('prop_step_light', 'tc 0.625 us (model)', S[0], '-'), ('prop_step_light_t1200', 'tc 0.069 us', S[1], '--')))
if all(have(f'case_{c[0]}.npz') for pr in pairs for c in pr):
    fig, ax = plt.subplots(1, 2, figsize=(9.6, 3.8), sharey=True)
    for a, pr, title in zip(ax, pairs, ('PI of the model (kp 0.5, ki 18), k_ff = 0', 'kp 0.05, ki 40, k_ff = 0')):
        for case, lab, col, ls in pr:
            rec, _, _ = load(case)
            t, il = full_period(rec, A_IL1)
            m = (t >= 0.10) & (t <= 0.25)
            a.plot(t[m]*1e3, il[m], color=col, ls=ls, lw=1.0, label=lab)
        a.plot([100, 150, 150, 250], [25, 25, 37.5, 37.5], color=INK, lw=0.8, ls=':', label='reference')
        a.set_title(title); a.set_xlabel('time [ms]')
    ax[0].set_ylabel('iL1, average over one PWM period [A]')
    ax[0].legend(loc='upper left', fontsize=7.5)
    fig.suptitle('Light load, reference 0.10 -> 0.15 pu at 150 ms (no filter): the phase steps of the solver '
                 '(0.9 deg at 0.625 us, 0.3 deg at 0.069 us)', fontsize=8.5)
    fig.tight_layout(); save(fig, 'light_load_ripple')

# ---------------------------------------------------------------- 7. start with empty DC links (legacy init)
rec, _, _ = load('legacy_start')
t = rec[:, T]*1e3
m = t <= 30
fig, ax = plt.subplots(2, 1, figsize=(7.0, 5.0), sharex=True)
ax[0].plot(t[m], rec[m, M_IL1], color=S[0], label='|iL(LFi_dc1)|, largest in each control interval')
ax[0].plot(t[m], rec[m, M_I1], color=S[1], ls='--', label='|i1| transformer, largest in each control interval')
ax[0].set_ylabel('A'); ax[0].legend(loc='upper right', fontsize=8)
ax[0].set_title('Legacy init: DC-link capacitors CFi_dc1/2 start at 0 V, batteries at 1070 V')
ax[1].plot(t[m], rec[m, M_VDC1], color=S[0], label='DC link 1, largest')
ax[1].plot(t[m], rec[m, M_VDC2], color=S[1], ls='--', label='DC link 2, largest')
ax[1].axhline(2000, color=INK, lw=0.8, ls=':')
ax[1].text(20.5, 1930, '2 kV MOSFET rating', fontsize=8, color=INK2, va='top')
ax[1].set_ylabel('V'); ax[1].set_xlabel('time [ms]'); ax[1].legend(loc='lower right', fontsize=8)
fig.tight_layout(); save(fig, 'legacy_start')
