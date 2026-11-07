function [test, name] = test_param(id, omega_nom)
%TEST_PARAM  Test signal of single_phase_pll_analysis (block test_signal, mask parameter p = param.test).
%
%   [test, name] = test_param(id, omega_nom)
%     id          test case, 1 to 6 (table below)
%     omega_nom   nominal pulsation [rad/s] (param.grid.emu.omega_grid_nom)
%     test        numeric fields only (the block is a MATLAB Function with a struct parameter)
%     name        one line that describes the case
%
%   The block test_signal computes, every ts_inv, in pu of the nominal peak voltage:
%       phi     = omega_nom*t, with a frequency step to f_step*omega_nom at t_fstep, plus dtheta from t_jump
%       u       = A(t) * ( cos(phi + theta0) + sum_k harm_amp(k) * sin(harm_order(k)*phi + harm_phase(k)) ) + dc
%                 (dc from t_dc), A(t) = a0, a_sag from t_sag, a_rec from t_rec
%       theta_g = phi + theta0 wrapped to (-pi, pi]: the angle of the fundamental (cosine convention), the angle the
%                 three PLLs lock on; omega_g = pulsation of the fundamental in pu of omega_nom
%   Case 1 is the test of the 2021 model: Sine Wave sin(omega t - pi/2) = cos(omega t - pi), plus 0.1 sin(4 omega t)
%   and 0.1 sin(5 omega t), all multiplied by the two Step blocks (1 -> 0.2 at 0.25 s, back to 1 at 0.5 s). It starts
%   180 degrees away from the PLLs, which start at angle 0. Cases 2 to 6 start in phase (theta0 = 0) and change one
%   thing at 0.4 s, so that one property of the loops is seen at a time. Inf disables an event.
%
%     id  name
%      1  2021 model: 4th and 5th harmonic 10 %, sag to 0.2 pu from 0.25 s to 0.5 s, start 180 deg away
%      2  phase jump of +30 deg at 0.4 s
%      3  frequency step 50 -> 49 Hz at 0.4 s
%      4  DC offset of 0.02 pu from 0.4 s
%      5  odd harmonics 3rd 5 %, 5th 6 %, 7th 5 % (limits of EN 50160), from 0 s
%      6  sag to 0.2 pu from 0.25 s to 0.5 s, no harmonics

    test.omega_nom  = omega_nom;
    test.theta0     = 0;            % [rad] angle of the fundamental at t = 0, cosine convention
    test.a0         = 1;            % [pu] amplitude before t_sag
    test.t_sag      = Inf;          % [s]
    test.a_sag      = 1;            % [pu]
    test.t_rec      = Inf;          % [s]
    test.a_rec      = 1;            % [pu]
    test.harm_order = [3 5 7];      % orders of the harmonics (three entries: the size of the parameter is fixed)
    test.harm_amp   = [0 0 0];      % [pu of the fundamental]
    test.harm_phase = [0 0 0];      % [rad], phase of sin(h*phi)
    test.t_jump     = Inf;          % [s] phase jump
    test.dtheta     = 0;            % [rad]
    test.t_fstep    = Inf;          % [s] frequency step
    test.f_step     = 1;            % [pu] frequency after the step
    test.t_dc       = Inf;          % [s] DC offset
    test.dc         = 0;            % [pu]

    switch id
        case 1
            name = '2021 model: 4th and 5th harmonic 10 %, sag to 0.2 pu from 0.25 s to 0.5 s, start 180 deg away';
            test.theta0 = -pi;
            test.harm_order = [4 5 7];
            test.harm_amp = [0.1 0.1 0];
            test.t_sag = 0.25;
            test.a_sag = 0.2;
            test.t_rec = 0.5;
            test.a_rec = 1;
        case 2
            name = 'phase jump of +30 deg at 0.4 s';
            test.t_jump = 0.4;
            test.dtheta = pi/6;
        case 3
            name = 'frequency step 50 -> 49 Hz at 0.4 s';
            test.t_fstep = 0.4;
            test.f_step = 49/50;
        case 4
            name = 'DC offset of 0.02 pu from 0.4 s';
            test.t_dc = 0.4;
            test.dc = 0.02;
        case 5
            name = 'odd harmonics 3rd 5 %, 5th 6 %, 7th 5 % from 0 s';
            test.harm_amp = [0.05 0.06 0.05];
        case 6
            name = 'sag to 0.2 pu from 0.25 s to 0.5 s, no harmonics';
            test.t_sag = 0.25;
            test.a_sag = 0.2;
            test.t_rec = 0.5;
            test.a_rec = 1;
        otherwise
            error('test_param:id', 'test case %d does not exist (1 to 6)', id);
    end
end
