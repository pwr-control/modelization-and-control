"""Last PWM periods of the cases of isop_sim.py (full-period averages, rms, peaks, DC parts)."""
import os, numpy as np, sys
HERE = os.path.dirname(os.path.abspath(__file__))
h=0.625e-6; Tp=250e-6; nper=int(round(Tp/h))
for name in sys.argv[1:]:
    w=np.load(os.path.join(HERE, f'case_{name}.npz'))['win']
    w=w[w[:,0]>0]
    n=(len(w)//nper)*nper; w=w[:n]
    t=w[:,0]
    print(f'===== {name}: window {t[0]*1e3:.3f}..{t[-1]*1e3:.3f} ms ({n//nper} periods)')
    print('  vout mean %.1f ripple p-p %.2f V' % (w[:,1].mean(), np.ptp(w[:,1])))
    for k in range(4):
        ip=w[:,6+k]; isec=w[:,10+k]; vab=w[:,14+k]; ilm=w[:,18+k]; vc=w[:,2+k]
        act=np.mean(np.abs(vab)>0.5*np.abs(vc).mean())
        print(f'  module {k+1}: vcin {vc.mean():7.1f} V (p-p {np.ptp(vc):5.2f}) | ipri rms {np.sqrt(np.mean(ip**2)):6.1f} A peak {np.abs(ip).max():6.1f} A dc {ip.mean():6.2f} A | isec rms {np.sqrt(np.mean(isec**2)):6.1f} dc {isec.mean():6.2f} | iLm dc {ilm.mean():6.2f} A peak {np.abs(ilm).max():5.1f} A | |vAB|>0 fraction {act:.3f} | vAB dc {vab.mean():6.2f} V')
    print(f'  rectifier output currents: r1 {w[:,22].mean():.1f} A (p-p {np.ptp(w[:,22]):.1f}), r2 {w[:,23].mean():.1f} A (p-p {np.ptp(w[:,23]):.1f}); pd {w[:,25].mean():.4f}')
