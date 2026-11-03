function [l1, l2] = fht_gains(ts, omega, delta, poles_norm)
%FHT_GAINS  Observer gains of the first harmonic tracker of grid_following_dqctrl (p.ctrl.fht_l1, fht_l2).
%
%   [l1, l2] = fht_gains(ts, omega, delta, poles_norm)
%     ts          [s] control period (p.ctrl.ts)
%     omega       [rad/s] tracked pulsation (p.ctrl.fht_omega)
%     delta       [-] damping of the resonant model (p.ctrl.fht_delta)
%     poles_norm  [1x2] observer poles in units of omega, e.g. [-1 -4]
%
%   Resonant model x' = A*x, A = [0 1; -omega^2 -delta*omega], output C = [1 0], discretized with the second-
%   order Taylor series of expm(A*ts) as the control of the block does (first_harmonic_tracker_init of the
%   library C code); observer poles exp(ts*poles_norm*omega). Same function as the local one of
%   library/masked_models/grid_following/grid_following_param_example.m.

    a  = -omega^2;
    b  = -delta*omega;
    Ad = [1 + 0.5*a*ts^2,       ts + 0.5*b*ts^2; ...
          a*ts + 0.5*a*b*ts^2,  1 + b*ts + 0.5*(a + b^2)*ts^2];
    L  = observer_gains(Ad, exp(ts*poles_norm*omega));
    l1 = L(1);
    l2 = L(2);
end
