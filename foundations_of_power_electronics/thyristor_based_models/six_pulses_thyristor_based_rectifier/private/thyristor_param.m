function thy = thyristor_param(name)
%THYRISTOR_PARAM  Data set of the thyristors of the six-pulse bridge (block "Thyristor with Reverse Recovery
%   (Lauritzen-Ma charge dynamics)" of the library, Simscape component thyristor_rr, six instances inside
%   grid_line/six_pulse rectifier/BR2/ssc_thyristor_bridge, which passes its mask parameters to them).
%
%   thy = thyristor_param(name)
%     name : 'abb_5STP26N6500_A' (default of thyristor_rr: ABB 5STP 26N6500, 6.5 kV / 2880 A phase-control
%            thyristor, typical values at Tvj = 125 degC, charge parameter set A: Qrr within 5 % of the
%            datasheet, recovery tail 66 us, IRM about 20 % low)
%            'abb_5STP26N6500_B' (same device, charge parameter set B: IRM and Qrr centred, tail 46 us)
%
%   Fields (mask parameter of ssc_thyristor_bridge in brackets): Vf (Vf_m), Ron (Ron_m), Goff (Goff_m),
%   Vth (forward_voltage_threshold_m), Vgt (Vgt_m), Il (latching_current_m), Ih (holding_current_m),
%   tau (carrier_lifetime_m), Tm (charge_cross_time_constant_m), Qth (residual_charge_threshold_m),
%   tq (circuit_commutated_turn_off_time_m). The values were typed in the mask of the model; they are the
%   set A, the same as the defaults of thyristor_rr.ssc. Model of the component: on when the gate signal
%   rises above Vgt with v > 0, off when i < Ih, re-fire when the forward voltage passes Vth within tq after
%   the turn-off (or while the excess charge is above Qth).

    switch name
        case {'abb_5STP26N6500_A', 'abb_5STP26N6500_B'}
            thy.name = name;
            thy.V_drm = 6500;                       % [V] repetitive peak off-state voltage (informative)
            thy.I_tavm = 2880;                      % [A] average on-state current (informative)
            thy.Vf = 1.12;                          % [V] threshold voltage VT0 (125 degC)
            thy.Ron = 0.29e-3;                      % [Ohm] slope resistance rT (125 degC)
            thy.Goff = 1e-5;                        % [S] off-state conductance (IRRM/VRRM, worst case)
            thy.Vth = 50;                           % [V] forward voltage threshold of the re-fire check
            thy.Vgt = 1.2;                          % [V] gate trigger threshold (the gate command is 0 or 12 V)
            thy.Il = 0.25;                          % [A] latching current (125 degC)
            thy.Ih = 0.075;                         % [A] holding current (125 degC)
            thy.Qth = 1e-6;                         % [C] excess-charge re-fire threshold
            thy.tq = 800e-6;                        % [s] circuit-commutated turn-off time
            if strcmp(name, 'abb_5STP26N6500_A')
                thy.tau = 100e-6;                   % [s] carrier lifetime
                thy.Tm = 190e-6;                    % [s] charge cross-over time constant
            else
                thy.tau = 85e-6;
                thy.Tm = 100e-6;
            end
        otherwise
            error('thyristor_param:name', 'unknown thyristor data set %s', name);
    end
    thy.tau_rr = thy.tau*thy.Tm/(thy.tau + thy.Tm);  % [s] recovery tail time constant (informative)

    fprintf('Thyristor %s: VT0 %.2f V, rT %.2f mOhm, Ih %.0f mA, tq %.0f us (%.1f deg at 50 Hz), tau %.0f us, Tm %.0f us\n', ...
        thy.name, thy.Vf, thy.Ron*1e3, thy.Ih*1e3, thy.tq*1e6, thy.tq*360*50, thy.tau*1e6, thy.Tm*1e6);
end
