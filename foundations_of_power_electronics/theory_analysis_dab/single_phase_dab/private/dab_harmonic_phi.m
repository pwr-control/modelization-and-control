function [phi, o] = dab_harmonic_phi(p, P, nh, npts)
%DAB_HARMONIC_PHI  Phase shift that gives the power P out of bridge 1 in the harmonic model dab_harmonic_op
%   (bisection on 0 ... pi/2, where the power grows with the phase shift).
%
%   [phi, o] = dab_harmonic_phi(p, P, nh, npts)
%     p, nh, npts : as in dab_harmonic_op;  P : [W] power out of bridge 1, P >= 0
%     phi         : [rad] (NaN when P is larger than the power at pi/2);  o : dab_harmonic_op(p, phi, nh, npts)

    lo = 0;
    hi = pi/2;
    o = dab_harmonic_op(p, hi, nh, 8);
    if o.P1 < P
        phi = NaN;
        o = dab_harmonic_op(p, hi, nh, npts);
        return
    end
    for k = 1:40
        mid = (lo + hi)/2;
        o = dab_harmonic_op(p, mid, nh, 8);
        if o.P1 < P
            lo = mid;
        else
            hi = mid;
        end
    end
    phi = (lo + hi)/2;
    o = dab_harmonic_op(p, phi, nh, npts);
end
