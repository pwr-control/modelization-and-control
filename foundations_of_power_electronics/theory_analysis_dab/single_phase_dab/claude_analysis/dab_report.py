#!/usr/bin/env python3
"""Writes design_check_single_phase_dab_output.txt: the numbers of the cases of dab_sim.py (dab_analyze.py), the
ripple of the battery current at light and full load, the static characteristic (runs dab_sweep.py again) and
the small-signal analysis of the current loop (runs dab_loop.py again).

Usage: python3 dab_report.py   (after python3 dab_sim.py <all the cases below>)
"""
import contextlib
import io
import os
import subprocess
import sys
import numpy as np
import dab_analyze as da

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'design_check_single_phase_dab_output.txt')
CASES = ['nominal', 'kff_05', 'kff_0', 'proposal', 'kff_05_fine', 'proposal_fine', 'fine_tc', 'legacy_start',
         'step_full', 'step_light', 'step_light_fine', 'step_light_k0', 'step_full_k0', 'prop_step_light',
         'prop_step_full', 'step_light_k0_t1200', 'prop_step_light_t1200']
LINE = '-'*66

HEADER = f"""Design check of single_phase_dab: switched simulation (dab_sim.py), static characteristic (dab_sweep.py) and
small-signal analysis of the current loop (dab_loop.py). Written by dab_report.py from the npz files of the cases.

Cases of dab_sim.py (__main__); all with the values of init_model.m unless written:
  nominal               the model as it is: DC links pre-charged at the battery OCV, k_ff = 1, PI kp 0.5 ki 18,
                        bench reference 0.1 pu -> 1 pu at 150 ms through the 20 ms filter, tc 0.625 us, 0.6 s
  kff_05, kff_0         as nominal with k_ff = 0.5 and 0
  proposal              as nominal with k_ff = 0.5, kp 0.05, ki 40
  kff_05_fine,          kff_05 and proposal with tc = T/400 = 0.208 us
  proposal_fine
  fine_tc               nominal with tc = T/400 = 0.208 us, 0.3 s
  legacy_start          as the legacy init: DC-link capacitors at 0 V, batteries at their OCV, 0.3 s
  step_full             reference 0.90 -> 0.95 pu at 350 ms without the filter (k_ff = 1), loop check at full load
  step_light            reference 0.10 -> 0.15 pu at 80 ms without the filter (k_ff = 1), loop check at light load
  step_light_fine       step_light with tc = T/400
  step_light_k0,        reference 0.10 -> 0.15 pu at 150 ms / 0.90 -> 0.95 pu at 350 ms, no filter, k_ff = 0
  step_full_k0          (feedback loop alone, PI of the model)
  prop_step_light,      as step_light_k0 / step_full_k0 with kp 0.05, ki 40
  prop_step_full
  step_light_k0_t1200,  step_light_k0 / prop_step_light with tc = T/1200 = 69 ns (phase steps of 0.3 deg)
  prop_step_light_t1200

Notes on the numbers:
  - iL1 = current of LFi_dc1 (battery-1 side), positive from battery_1 to battery_2; 1 pu = Idc_FS = 250 A.
  - P_bat = battery terminal power, P_dc = DC power into bridge 1 / out of bridge 2, P_ac = AC power of the bridges.
    P_dc1 - P_dc2 contains a numerical loss of the backward-Euler step (about 11 kW at 1 pu with tc 0.625 us,
    proportional to tc), see the static characteristic below and the README.
  - "current back to zero inside the dead time": the current of the outgoing switches falls to zero in the
    diodes before the dead time ends; the bridge floats and the incoming switches turn on hard at the end of the
    dead time (the commutation is lost).
  - The "after the step" lines (bench cases only) use the average of iL1 over one PWM period.

{LINE} control intervals and high-resolution windows
"""


def ripple_table():
    rows = []
    for c in CASES:
        if c in ('nominal', 'kff_05', 'proposal', 'kff_05_fine', 'proposal_fine'):
            wins = ((0.10, 0.15), (0.55, 0.60))
        elif c in ('step_light_k0', 'step_light_k0_t1200', 'prop_step_light', 'prop_step_light_t1200'):
            wins = ((0.10, 0.15), (0.20, 0.25))
        else:
            continue
        rec = da.load(c)['rec']
        t, il = da.full_period(rec, da.A_IL1)
        out = []
        for t0, t1 in wins:
            m = (t >= t0) & (t < t1)
            mu = (rec[:, da.T] >= t0) & (rec[:, da.T] < t1)
            x = il[m] - il[m].mean()
            X = np.abs(np.fft.rfft(x*np.hanning(len(x))))
            fr = np.fft.rfftfreq(len(x), 1/24e3)
            out.append(f'{t0*1e3:.0f}-{t1*1e3:.0f} ms: mean {il[m].mean():6.2f} A, p-p {np.ptp(il[m]):5.2f} A, '
                       f'std {il[m].std():4.2f} A, main frequency {fr[np.argmax(X[1:]) + 1]:.0f} Hz, '
                       f'u {rec[mu, da.U].mean():.4f}')
        rows.append(f'{c:22s} ' + ' | '.join(out))
    return '\n'.join(rows)


def run(script):
    r = subprocess.run([sys.executable, os.path.join(HERE, script)], cwd=HERE, capture_output=True, text=True,
                       check=True)
    return r.stdout


if __name__ == '__main__':
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        for c in CASES:
            da.summary(c)
    text = (HEADER + buf.getvalue() +
            f'\n{LINE} ripple of iL1 (average over one PWM period) at light and full load\n' + ripple_table() +
            f'\n\n{LINE} static characteristic, fixed u (dab_sweep.py)\n' + run('dab_sweep.py') +
            f'\n{LINE} current loop, small signal (dab_loop.py)\n' + run('dab_loop.py'))
    with open(OUT, 'w') as fh:
        fh.write(text)
    print(f'{OUT}: {text.count(chr(10))} lines')
