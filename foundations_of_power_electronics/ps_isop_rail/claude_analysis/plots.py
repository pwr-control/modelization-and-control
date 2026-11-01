#!/usr/bin/env python3
"""Figures of the ps_isop_rail design check (reads the case_*.npz files written by isop_sim.py)."""
import os, numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'figures')
os.makedirs(OUT, exist_ok=True)
S = ['#2a78d6', '#eb6834', '#1baf7a', '#eda100']        # categorical slots 1-4 (fixed order: modules 1-4)
INK, INK2, GRID, SURF = '#0b0b0b', '#52514e', '#e4e3df', '#fcfcfb'
plt.rcParams.update({'font.size': 9, 'axes.edgecolor': INK2, 'axes.labelcolor': INK, 'xtick.color': INK2,
                     'ytick.color': INK2, 'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.6,
                     'axes.spines.top': False, 'axes.spines.right': False, 'lines.linewidth': 1.4,
                     'figure.facecolor': SURF, 'axes.facecolor': SURF, 'savefig.facecolor': SURF,
                     'legend.frameon': False, 'axes.titlesize': 9, 'axes.titleweight': 'bold'})
def load(n):
    d = np.load(os.path.join(HERE, f'case_{n}.npz')); return d['rec'], d['win']
def save(fig, name):   # PDF for the repository (*.png is ignored there), PNG for a quick look
    fig.savefig(os.path.join(OUT, name + '.pdf'))
    fig.savefig(os.path.join(OUT, name + '.png'), dpi=150)
def per(x):      # average over one PWM period (two control intervals)
    return np.convolve(x, [0.5, 0.5], mode='same')

# ---------------------------------------------------------------- 1. start-up as in the model
rec, _ = load('nominal'); t = rec[:, 0]; m = t <= 0.6
fig, ax = plt.subplots(4, 1, figsize=(7.0, 7.2), sharex=True)
ax[0].plot(t[m], rec[m, 1], color=S[0], label='output voltage')
ax[0].axhline(750, color=INK2, lw=0.8, ls='--', label='reference 750 V')
ax[0].set_ylabel('V'); ax[0].set_title('Output voltage'); ax[0].legend(loc='lower right')
ax[1].plot(t[m], rec[m, 2], color=S[0]); ax[1].set_ylabel('pu'); ax[1].set_title('Phase shift of the four bridges (PI output, 1 = square wave)')
ax[2].plot(t[m], rec[m, 7], color=S[0]); ax[2].set_ylabel('V'); ax[2].set_title('Input voltage of each module (the four are equal)')
ax[3].plot(t[m], rec[m, 3], color=S[0]); ax[3].set_ylabel('A'); ax[3].set_title('Input current of the string (Ldc_dc1)')
ax[3].set_xlabel('time [s]')
fig.tight_layout(); save(fig, 'startup'); plt.close(fig)

# ---------------------------------------------------------------- 2. steady-state waveforms
_, w = load('nominal'); w = w[w[:, 0] > 0]
n2 = int(round(2*250e-6/0.625e-6)); w = w[-n2:]; tt = (w[:, 0] - w[0, 0])*1e6
fig, ax = plt.subplots(3, 1, figsize=(7.0, 6.0), sharex=True)
ax[0].plot(tt, w[:, 14], color=S[0]); ax[0].set_ylabel('V'); ax[0].set_title('Bridge voltage v_AB of module 1 (phase shift 0.78)')
ax[1].plot(tt, w[:, 6], color=S[0], label='primary current')
ax[1].plot(tt, w[:, 18], color=S[1], label='magnetizing current')
ax[1].set_ylabel('A'); ax[1].set_title('Module 1 transformer'); ax[1].legend(loc='upper right', ncol=2)
ax[2].plot(tt, w[:, 10], color=S[0], label='secondary current, module 1')
ax[2].plot(tt, w[:, 13], color=S[3], ls='--', label='secondary current, module 4')
ax[2].plot(tt, w[:, 22], color=S[2], label='output current of rectifier_1')
ax[2].set_ylabel('A'); ax[2].set_title('Rectifier_1: modules 1 and 4 in parallel on the AC side'); ax[2].legend(loc='lower right', ncol=1)
ax[2].set_xlabel('time [us]')
fig.tight_layout(); save(fig, 'waveforms'); plt.close(fig)

# ---------------------------------------------------------------- 3. input voltage sharing
fig, ax = plt.subplots(1, 2, figsize=(8.6, 3.6), gridspec_kw={'width_ratios': [1.1, 1]})
r1, _ = load('split_pairs_ss'); r2, _ = load('split_in_pair_ss')
d1 = np.abs((r1[:, 7] + r1[:, 10] - r1[:, 8] - r1[:, 9])/2); d2 = np.abs((r2[:, 8] - r2[:, 9])/2)
ax[0].semilogy(r1[:, 0]*1e3, np.maximum(d1, 1e-2), color=S[0], label='between the two rectifier pairs')
ax[0].semilogy(r2[:, 0]*1e3, np.maximum(d2, 1e-2), color=S[1], label='inside a pair (modules 2 and 3)')
ax[0].set_xlim(0, 300); ax[0].set_ylim(0.1, 200)
ax[0].set_xlabel('time [ms]'); ax[0].set_ylabel('|voltage deviation| [V]')
ax[0].set_title('Start 80 V off the equal share, operating point'); ax[0].legend(loc='upper right')
cases = [('nominal', 'equal\nmodules'), ('mm_n1', 'module 1\nturns +2 %'), ('mm_L2s', 'module 2\nLs +5 %'),
         ('mm_L2', 'module 2\nLs +20 %')]
x = np.arange(len(cases)); bw = 0.19
for k in range(4):
    vals = []
    for c, _l in cases:
        _, ww = load(c); ww = ww[ww[:, 0] > 0]; vals.append(ww[:, 2 + k].mean())
    ax[1].bar(x + (k - 1.5)*(bw + 0.01), vals, bw, color=S[k], label=f'module {k+1}')
ax[1].axhline(1171, color=INK2, lw=0.8, ls='--')
ax[1].set_xticks(x); ax[1].set_xticklabels([l for _c, l in cases], fontsize=8); ax[1].set_ylabel('V'); ax[1].set_ylim(800, 1550)
ax[1].set_title('Input voltage of each module, steady state'); ax[1].legend(loc='upper left', ncol=2, fontsize=8)
ax[1].grid(axis='x', visible=False)
fig.tight_layout(); save(fig, 'voltage_sharing'); plt.close(fig)

# ---------------------------------------------------------------- 4. start from 0 V and nominal catenary
_, wz = load('zero_init'); wz = wz[wz[:, 0] > 0]; rz, _ = load('zero_init')
rc, _ = load('catenary_3kV')
fig, ax = plt.subplots(1, 3, figsize=(9.6, 3.2))
mz = rz[:, 0] <= 0.012
ax[0].plot(wz[:, 0]*1e3, wz[:, 2], color=S[0]); ax[0].axhline(1171, color=INK2, lw=0.8, ls='--')
ax[0].set_title('String capacitors start at 0 V'); ax[0].set_xlabel('time [ms]'); ax[0].set_ylabel('module input voltage [V]')
ax[1].plot(rz[mz, 0]*1e3, rz[mz, 24], color=S[0], label='peak in each control interval')
ax[1].axhline(246, color=INK2, lw=0.8, ls='--', label='full-load value 246 A')
ax[1].set_ylim(0, 2500)
ax[1].set_title('Input current of the string'); ax[1].set_xlabel('time [ms]'); ax[1].set_ylabel('A'); ax[1].legend(loc='upper right', fontsize=8)
ax[2].plot(rc[:, 0], rc[:, 1], color=S[0], label='output voltage')
ax[2].axhline(750, color=INK2, lw=0.8, ls='--', label='reference 750 V')
ax[2].set_ylim(0, 800); ax[2].set_title('Emulator at 3 kV (nominal catenary)'); ax[2].set_xlabel('time [s]'); ax[2].set_ylabel('V')
ax[2].legend(loc='lower right', fontsize=8)
fig.tight_layout(); save(fig, 'zero_start_and_3kV'); plt.close(fig)

# ---------------------------------------------------------------- 5. DC magnetizing current after the start
fig, ax = plt.subplots(1, 1, figsize=(7.0, 2.8))
ax.plot(rec[:, 0], per(rec[:, 11]), color=S[0])
ax.set_xlabel('time [s]'); ax.set_ylabel('A'); ax.set_xlim(0, 1.75)
ax.set_title('Mean magnetizing current of each transformer (one PWM period); peak of the AC part 52 A')
fig.tight_layout(); save(fig, 'magnetizing_dc'); plt.close(fig)
print('figures in', OUT, sorted(os.listdir(OUT)))
