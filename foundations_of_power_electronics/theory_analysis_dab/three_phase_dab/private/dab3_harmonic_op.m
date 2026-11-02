function op = dab3_harmonic_op(p, phi, npts, hmax)
%DAB3_HARMONIC_OP  Steady state of the three-phase DAB (Y-Y, six-step) by harmonic superposition.
%
%   op = dab3_harmonic_op(p, phi)
%   op = dab3_harmonic_op(p, phi, npts, hmax)
%
%   p     struct: V1, V2 [V] DC voltages; n1, n2 turns; fs [Hz]; L1 [H], R1 [Ohm] primary series branch
%         (leakage and external inductor, winding and switch); L2r [H], R2r [Ohm] secondary series branch
%         referred to the primary; Lm [H], Rfe [Ohm] magnetizing branch, primary side
%   phi   [rad] lag of bridge 2 on bridge 1 (> 0: power from side 1 to side 2)
%   npts  points per period (default 3600, multiple of 6); hmax highest harmonic (default 2001)
%
%   Per phase, T equivalent referred to the primary. Phase-to-star-point voltage of a six-step bridge:
%   (2 V/pi) sum sin(h w t)/h, h = 6k +- 1 (floating star points: no triplen harmonics). Leg a of bridge 1
%   rises at theta = 0, leg a of bridge 2 at theta = phi; legs b, c 120 and 240 deg later.
%
%   op.theta            [rad] angle grid of one period
%   op.ia, op.ja        [A]   primary current out of leg a of bridge 1, secondary current referred to the
%                             primary into leg a of bridge 2 (op.ja*n1/n2 is the true secondary current)
%   op.idc1, op.idc2    [A]   DC currents of bridge 1 (from DC link 1) and bridge 2 (into DC link 2)
%   op.P1, op.P2        [W]   DC powers of the two sides
%   op.I1rms, I2rms, I1pk, I2pk  [A] phase currents, true values on each side
%   op.i_sw1, op.i_sw2  [A]   current that charges the leg capacitances at the commutations of bridge 1 and
%                             bridge 2 (> 0: ZVS possible, transition time 2 Coss V/i_sw)
%   op.Im_pk            [A]   magnetizing (+ iron) current, peak, primary
%   op.Idc1, op.Idc2    [A]   mean DC currents; op.Icap1_rms, op.Icap2_rms [A] rms of their ripple
%   op.lam_pk           [Vs]  peak flux linkage of the magnetizing branch, primary

    if nargin < 3 || isempty(npts)
        npts = 3600;
    end
    if nargin < 4 || isempty(hmax)
        hmax = 2001;
    end
    h = 1:2:hmax;
    h = h(mod(h, 3) ~= 0);                          % 1, 5, 7, 11, 13, ...
    n12 = p.n1/p.n2;
    V2r = p.V2*n12;                                 % side 2 referred to the primary
    w = 2*pi*p.fs;

    U1 = 2*p.V1./(h*pi);
    U2 = 2*V2r./(h*pi).*exp(-1i*h*phi);
    Z1 = p.R1 + 1i*h*w*p.L1;
    Z2 = p.R2r + 1i*h*w*p.L2r;
    Zm = 1./(1./(1i*h*w*p.Lm) + 1/p.Rfe);
    E = (U1./Z1 + U2./Z2)./(1./Z1 + 1./Zm + 1./Z2);
    I1 = (U1 - E)./Z1;
    J2 = (E - U2)./Z2;

    theta = (0:npts-1)'*2*pi/npts;
    W = exp(1i*theta*h);                            % npts x numel(h)
    ia = imag(W*I1.');
    ja = imag(W*J2.');
    lam = imag(W*(E./(1i*h*w)).');                  % flux linkage of the magnetizing branch

    sh = round(npts/3);
    ib = circshift(ia, sh);   ic = circshift(ia, 2*sh);
    jb = circshift(ja, sh);   jc = circshift(ja, 2*sh);
    s1 = @(x) double(mod(x, 2*pi) < pi);            % upper switch on
    idc1 = s1(theta).*ia + s1(theta - 2*pi/3).*ib + s1(theta - 4*pi/3).*ic;
    idc2 = (s1(theta - phi).*ja + s1(theta - phi - 2*pi/3).*jb + s1(theta - phi - 4*pi/3).*jc)*n12;

    op.theta = theta;
    op.ia = ia;
    op.ja = ja;
    op.idc1 = idc1;
    op.idc2 = idc2;
    op.P1 = p.V1*mean(idc1);
    op.P2 = p.V2*mean(idc2);
    op.I1rms = sqrt(mean(ia.^2));
    op.I2rms = sqrt(mean(ja.^2))*n12;
    op.I1pk = max(abs(ia));
    op.I2pk = max(abs(ja))*n12;
    % leg a of bridge 1 rises at 0: ZVS needs ia(0) < 0; leg a of bridge 2 rises at phi: ZVS needs ja(phi) > 0
    op.i_sw1 = -sum(imag(I1));
    op.i_sw2 = imag(sum(J2.*exp(1i*h*phi)))*n12;
    op.Im_pk = max(abs(ia - ja));
    op.Idc1 = mean(idc1);
    op.Idc2 = mean(idc2);
    op.Icap1_rms = std(idc1, 1);
    op.Icap2_rms = std(idc2, 1);
    op.lam_pk = max(abs(lam));
end
