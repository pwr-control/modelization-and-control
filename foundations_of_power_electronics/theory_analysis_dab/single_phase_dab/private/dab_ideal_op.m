function op = dab_ideal_op(V1, V2r, fs, L, P)
%DAB_IDEAL_OP  Lossless single-phase DAB with phase-shift modulation: square-wave bridge voltages, only the
%   series inductance (no series capacitors, no magnetizing current), dead time neglected (De Doncker).
%
%   op = dab_ideal_op(V1, V2r, fs, L, P)
%     V1, V2r : [V] DC voltage of side 1 and DC voltage of side 2 referred to side 1 (V2r = V2*n1/n2)
%     fs      : [Hz] transformer frequency
%     L       : [H] series inductance referred to side 1 (Ld1 + Ld2*(n1/n2)^2)
%     P       : [W] power from side 1 to side 2, P >= 0
%
%   With w = 2*pi*fs and phi the delay of bridge 2 behind bridge 1, 0 <= phi <= pi/2:
%     P(phi) = V1*V2r*phi*(1 - phi/pi)/(w*L),  Pmax = V1*V2r/(8*fs*L) at phi = pi/2
%     current of the series inductance (from bridge 1 to bridge 2) when bridge 1 switches (theta = 0) and when
%     bridge 2 switches (theta = phi):
%       i(0)   = -(V1*pi + V2r*(2*phi - pi))/(2*w*L)       bridge 1 switches with ZVS when i(0) < 0
%       i(phi) =  (V2r*pi + V1*(2*phi - pi))/(2*w*L)       bridge 2 switches with ZVS when i(phi) > 0
%   op.phi [rad] phase shift for P (NaN when P > Pmax), op.Pmax [W], op.i_sw1 = -i(0), op.i_sw2 = i(phi) [A]
%   (positive: ZVS), op.Ipk [A] peak, op.Irms [A] rms of the series current, op.d = V2r/V1.

    w = 2*pi*fs;
    op.d = V2r/V1;
    op.Pmax = V1*V2r/(8*fs*L);
    x = 8*fs*L*P/(V1*V2r);
    if x <= 1
        op.phi = pi/2*(1 - sqrt(1 - x));
    else
        op.phi = NaN;
    end
    phi = op.phi;
    a = -(V1*pi + V2r*(2*phi - pi))/(2*w*L);                % i(0)
    b = (V2r*pi + V1*(2*phi - pi))/(2*w*L);                 % i(phi)
    c = -a;                                                 % i(pi)
    op.i_sw1 = -a;
    op.i_sw2 = b;
    op.Ipk = max(abs([a, b]));
    op.Irms = sqrt((phi*(a^2 + a*b + b^2) + (pi - phi)*(b^2 + b*c + c^2))/(3*pi));
end
