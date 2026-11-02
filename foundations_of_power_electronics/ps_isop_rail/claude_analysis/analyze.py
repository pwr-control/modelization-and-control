"""Control-interval records of the cases of isop_sim.py: settling, operating points, peaks."""
import os, numpy as np, sys
HERE = os.path.dirname(os.path.abspath(__file__))
def load(name):
    d=np.load(os.path.join(HERE, f'case_{name}.npz')); return d['rec'], d['win']
def fullper(x):           # average of two consecutive control intervals = one PWM period
    return 0.5*(x[1:]+x[:-1])
for name in sys.argv[1:]:
    rec,win=load(name)
    t=rec[:,0]; vout=rec[:,1]; pd=rec[:,2]; iLin=rec[:,3]; iout=rec[:,4]; i1=rec[:,5]; i2=rec[:,6]
    vc=rec[:,7:11]; iLm=rec[:,11:15]; ipri=rec[:,15:19]
    print(f'===== {name}: {t[-1]:.3f} s, {len(t)} control samples')
    # settling of vout
    v95=np.argmax(vout>=0.95*750) if np.any(vout>=0.95*750) else None
    v99=np.argmax(vout>=0.99*750) if np.any(vout>=0.99*750) else None
    print(f'  vout: max {vout.max():.1f} V at {t[np.argmax(vout)]*1e3:.1f} ms; 95% at {t[v95]*1e3 if v95 is not None else float("nan"):.1f} ms, 99% at {t[v99]*1e3 if v99 is not None else float("nan"):.1f} ms')
    for tt in (0.005,0.01,0.02,0.05,0.1,0.2,0.3,0.5,0.75,1.0,1.25,1.5,1.749):
        if tt>t[-1]: continue
        k=np.argmin(abs(t-tt)); s=slice(max(k-8,0),k+8)
        print(f'  t={tt*1e3:7.1f} ms vout={vout[s].mean():7.1f} pd={pd[s].mean():.4f} iLin={iLin[s].mean():7.1f} iout={iout[s].mean():7.1f} (r1 {i1[s].mean():6.1f} r2 {i2[s].mean():6.1f}) vcin={", ".join(f"{v:.1f}" for v in vc[s].mean(0))} iLm_dc={", ".join(f"{v:.2f}" for v in iLm[s].mean(0))} ipri_dc={", ".join(f"{v:.2f}" for v in ipri[s].mean(0))}')
    print(f'  max |iLin| {rec[:,24].max():.0f} A at {t[np.argmax(rec[:,24])]*1e3:.2f} ms; max vcin {rec[:,25].max():.0f} V at {t[np.argmax(rec[:,25])]*1e3:.2f} ms; max |ipri| {rec[:,26].max():.0f} A; max |isec| {rec[:,27].max():.0f} A; max |iLm| {rec[:,29].max():.1f} A')
    z=rec[:,19]; print(f'  soc {z[0]:.5f} -> {z[-1]:.5f}, hyst {rec[-1,20]:.5f}, vCb {rec[0,23]:.0f} -> {rec[-1,23]:.0f} V')
