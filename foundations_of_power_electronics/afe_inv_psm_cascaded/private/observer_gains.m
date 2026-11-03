function L = observer_gains(Ad, pd)
%OBSERVER_GAINS  Gain L [2x1] of a two-state observer with output C = [1 0]: eig(Ad - L*C) = pd.
%
%   L = observer_gains(Ad, pd)
%     Ad  [2x2] discrete state matrix
%     pd  [1x2] wanted discrete poles
%
%   Closed-form Ackermann for two states:
%   det(zI - (Ad - L*C)) = z^2 - (a11 - l1 + a22)*z + (a11 - l1)*a22 - a12*(a21 - l2).
%   Used for the first harmonic tracker (fht_gains) and for the double integrator observer of the PLL angle
%   (p.ctrl.obs_L) of grid_following_dqctrl. Same function as the local one of
%   library/masked_models/grid_following/grid_following_param_example.m.

    a1 = -(pd(1) + pd(2));
    a0 = pd(1)*pd(2);
    l1 = Ad(1,1) + Ad(2,2) + a1;
    l2 = Ad(2,1) + (a0 - (Ad(1,1) - l1)*Ad(2,2))/Ad(1,2);
    L  = [l1; l2];
end
