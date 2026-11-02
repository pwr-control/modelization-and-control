function op = dab3_ideal_op(V1, V2r, fs, L, P)
%DAB3_IDEAL_OP  Lossless three-phase DAB (De Doncker): phase shift for a power, maximum power, gain.
%
%   op = dab3_ideal_op(V1, V2r, fs, L, P)
%
%   V1, V2r : [V] DC voltages, side 2 referred to side 1 (V2 n1/n2), scalars
%   fs      : [Hz] transformer frequency
%   L       : [H] series inductance per phase, Ld1 + Ld2 (n1/n2)^2
%   P       : [W] power from side 1 to side 2 (negative: from side 2 to side 1), scalar or array
%
%   op.phi      [rad]   lag of bridge 2 on bridge 1, the smallest |phi| <= pi/2 that gives P (NaN above Pmax)
%   op.Pmax     [W]     maximum power, at |phi| = pi/2: 7 pi/36 V1 V2r/(w L)
%   op.k        [W]     V1 V2r/(w L)
%   op.dP_dphi  [W/rad] small-signal gain at op.phi
%
%   Six-step bridges, Y-Y transformer, magnetizing inductance and losses neglected:
%     |phi| <= pi/3         P = k phi (2/3 - |phi|/(2 pi))
%     pi/3 <= |phi| <= 2pi/3  P = k sign(phi) (|phi| - phi^2/pi - pi/18)
%   The power grows up to |phi| = pi/2 and falls beyond it. dab3_harmonic_op adds Lm, Rfe and the resistances.

    w = 2*pi*fs;
    k = V1*V2r/(w*L);
    x = abs(P)/k;                                   % normalized power
    a = NaN(size(x));
    r1 = x <= pi/6;                                 % |phi| <= pi/3
    r2 = x > pi/6 & x <= 7*pi/36;                   % pi/3 < |phi| <= pi/2
    a(r1) = pi*(2/3 - sqrt(4/9 - 2*x(r1)/pi));
    a(r2) = pi/2*(1 - sqrt(max(0, 1 - 4*(x(r2) + pi/18)/pi)));

    g = NaN(size(x));
    g(r1) = k*(2/3 - a(r1)/pi);
    g(r2) = k*(1 - 2*a(r2)/pi);

    op.phi = sign(P).*a;
    op.Pmax = 7*pi/36*k;
    op.k = k;
    op.dP_dphi = g;
end
