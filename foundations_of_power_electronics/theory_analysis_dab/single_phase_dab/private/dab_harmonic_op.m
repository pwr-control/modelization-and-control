function o = dab_harmonic_op(p, phi, nh, npts)
%DAB_HARMONIC_OP  Steady state of the single-phase DAB of single_phase_dab.slx by harmonic superposition:
%   square-wave voltages of the two bridges (dead time and transitions neglected), linear tank with the series
%   capacitors, the winding and switch resistances and the magnetizing branch of the transformer.
%
%   o = dab_harmonic_op(p, phi, nh, npts)
%     p    : struct with
%            V1, V2      [V] DC-link voltages of side 1 and side 2
%            n1, n2      turns of the transformer (n12 = n1/n2: V2r = V2*n12 is side 2 referred to side 1)
%            fs          [Hz] transformer frequency
%            R1, L1, C1  primary branch: Rs1 + 2*Rds_on, Ld1, Cs1 (series capacitor of side 1)
%            R2, L2, C2  secondary branch in secondary units: Rs2 + 2*Rds_on, Ld2, Cs2
%            Lm, Rfe     magnetizing inductance and iron-loss resistance (primary, in parallel)
%     phi  : [rad] delay of bridge 2 behind bridge 1
%     nh   : highest odd harmonic (e.g. 999)
%     npts : points of the period for the waveforms (e.g. 1000)
%   o.theta [rad], o.i1 [A] primary current (out of bridge 1), o.i2r [A] secondary current referred to the
%   primary (from the transformer into bridge 2; o.i2 = o.i2r*n12 in secondary units), o.im [A] magnetizing
%   current; o.P1, o.P2 [W] power out of bridge 1 and into bridge 2; o.I1rms, o.I1pk, o.I2rms [A];
%   o.i_sw1 = -i1(0), o.i_sw2 = i2r(phi) [A], currents at the switching instants of bridge 1 and bridge 2
%   (positive: the current charges and discharges the output capacitances of the leg, ZVS); o.Im_pk [A].

    n12 = p.n1/p.n2;
    w = 2*pi*p.fs;
    V2r = p.V2*n12;
    o.theta = (0:npts-1)'/npts*2*pi;
    o.i1 = zeros(npts, 1);
    o.i2r = zeros(npts, 1);
    o.im = zeros(npts, 1);
    o.P1 = 0;
    o.P2 = 0;
    i1_0 = 0;
    i2_phi = 0;
    for h = 1:2:nh
        Z1 = p.R1 + 1j*h*w*p.L1 + 1/(1j*h*w*p.C1);
        Z2 = (p.R2 + 1j*h*w*p.L2 + 1/(1j*h*w*p.C2))*n12^2;
        Zm = 1/(1/(1j*h*w*p.Lm) + 1/p.Rfe);
        U1 = 4*p.V1/(h*pi);                                 % sin(h*theta) phasors (imaginary part)
        U2 = 4*V2r/(h*pi)*exp(-1j*h*phi);
        Um = (U1/Z1 + U2/Z2)/(1/Z1 + 1/Z2 + 1/Zm);
        I1 = (U1 - Um)/Z1;
        I2 = (Um - U2)/Z2;
        Im = Um/(1j*h*w*p.Lm);
        e = exp(1j*h*o.theta);
        o.i1 = o.i1 + imag(I1*e);
        o.i2r = o.i2r + imag(I2*e);
        o.im = o.im + imag(Im*e);
        o.P1 = o.P1 + real(U1*conj(I1))/2;
        o.P2 = o.P2 + real(U2*conj(I2))/2;
        i1_0 = i1_0 + imag(I1);
        i2_phi = i2_phi + imag(I2*exp(1j*h*phi));
    end
    o.i2 = o.i2r*n12;
    o.I1rms = sqrt(mean(o.i1.^2));
    o.I2rms = sqrt(mean(o.i2.^2));
    o.I1pk = max(abs(o.i1));
    o.Im_pk = max(abs(o.im));
    o.i_sw1 = -i1_0;
    o.i_sw2 = i2_phi;
end
