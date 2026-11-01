function im = im_machine_param(m)
%IM_MACHINE_PARAM  Induction machine data of im_analysis_and_design: equivalent circuit from the nameplate,
%   the no-load test and the locked-rotor test, per-unit bases and normalized parameters.
%
%   im = im_machine_param(m), with m = param.im.machine_in:
%     name, pwr_nom [W], u_nom [V rms line-to-line], i_nom [A rms], freq_nom [Hz], rpm_load [rpm at rated
%     load], number_poles, eta, cosphi, Rs [Ohm], u_no_load [V], i_no_load [A], f_no_load [Hz] (no-load test),
%     u_cc [V], i_cc [A], f_cc [Hz] (locked-rotor test), Jm [kg m^2], load_friction_factor [pu]
%
%   Local replacement of im_setup of the library (called by im_calculus in the legacy init): same formulas,
%   same field names, so that the model sees the same numbers. Fields read by the models:
%     Rs, Ls, Lm, Lr, Rr, Jm, load_friction, pp     block induction motor/poliphase_induction_machine1 (SI)
%     ibez, omega_bez, omega_m_bez, Lm_norm,        control, acquisition and mechanical data (per unit)
%     alpha_norm
%
%   Equivalent circuit (dq, amplitude invariant, the Simscape block im_uvw_2 uses the same convention):
%     Ls = u_no_load/sqrt(3)/i_no_load/(2*pi*f_no_load)        stator self inductance (no-load test, Rs neglected)
%     Lsigma = sqrt(Zcc^2 - (Rs + Rr)^2)/(2*pi*f_cc)            total leakage (locked-rotor test, Rr = Rs here)
%     Lds = Ldr = Lsigma/2, Lm = Ls - Lds, Lr = Lm + Ldr
%     Rr: scanned from Rs/5 in steps of Rs/100 until the torque of the equivalent circuit at rated voltage and
%     rated slip is within +-5 % of torque_nom (Lsigma is not recomputed with the final Rr)
%   Bases: ubez = u_nom/sqrt(3/2) (peak phase), ibez = i_nom*sqrt(2), omega_bez = 2*pi*freq_nom,
%   Xbez = ubez/ibez, Lbez = Xbez/omega_bez, flux base ubez/omega_bez = psi_bez, torque_nom = pwr_nom/eta/omega_m_bez.

    im.name = m.name;
    im.pwr_nom = m.pwr_nom;
    im.u_nom = m.u_nom;
    im.i_nom = m.i_nom;
    im.freq_nom = m.freq_nom;
    im.rpm_load = m.rpm_load;
    im.number_poles = m.number_poles;
    im.eta = m.eta;
    im.cosphi = m.cosphi;
    im.Rs = m.Rs;
    im.u_no_load = m.u_no_load;
    im.i_no_load = m.i_no_load;
    im.f_no_load = m.f_no_load;
    im.u_cc = m.u_cc;
    im.i_cc = m.i_cc;
    im.f_cc = m.f_cc;
    im.Jm = m.Jm;
    im.load_friction_factor = m.load_friction_factor;

    im.pp = im.number_poles/2;                                  % pole pairs
    im.radpersecond2rpm = 60/2/pi;
    im.omega_bez = im.freq_nom*2*pi;                            % [rad/s] electrical base
    im.omega_m_bez = im.omega_bez/im.pp;                        % [rad/s] mechanical base (synchronous speed)
    im.omega_m_load = im.rpm_load/60*2*pi;                      % [rad/s] rated load speed
    im.rpm_no_load = im.freq_nom*60/im.pp;                      % [rpm]
    im.slip_nom = (im.rpm_no_load - im.rpm_load)/im.rpm_no_load;
    im.torque_nom = im.pwr_nom/im.eta/im.omega_m_bez;           % [Nm] (library definition)

    % equivalent circuit from the tests (im_parameter_calculation), first pass with Rr = Rs
    im.Rr = im.Rs;
    im.Ls = im.u_no_load/sqrt(3)/im.i_no_load/(2*pi*im.f_no_load);
    Zsigma = (im.u_cc/sqrt(3))/im.i_cc;
    Lsigma = sqrt(Zsigma^2 - (im.Rs + im.Rr)^2)/(2*pi*im.f_cc);
    im.Lds = Lsigma/2;
    im.Ldr = Lsigma - im.Lds;
    im.Lm = im.Ls - im.Lds;
    im.Lr = im.Lm + im.Ldr;
    im.psi_nom = im.i_no_load*sqrt(2)*im.Lm;                    % [Vs] peak flux at the no-load current
    im.psi_bez = im.u_nom/sqrt(3/2)/im.omega_bez;               % [Vs]

    % rotor resistance: torque of the equivalent circuit at rated slip within +-5 % of torque_nom
    [im.Rr, im.tau_bez] = optimize_rotor_resistor(im, im.Rs/5, im.Rs/100, im.Rs*5, 0.05);

    im.alpha = im.Rr/im.Lr;                                     % [1/s] inverse of the rotor time constant
    im.mu = 3/2*im.Lm/im.Jm/im.Lr;
    im.sigma = im.Ls*(1 - (im.Lm)^2/im.Ls/im.Lr);               % [H] sigma*Ls, stator transient inductance
    im.beta = im.Lm/im.sigma/im.Lr;
    im.gamma = im.Rs/im.sigma + im.beta*im.alpha*im.Lm;

    % normalization (im_normalization)
    im.ubez = im.u_nom/sqrt(3/2);                               % [V] peak phase-to-neutral
    im.ibez = im.i_nom*sqrt(2);                                 % [A] peak
    im.im_bez = im.i_no_load*sqrt(2);                           % [A] peak no-load current
    im.Xbez = im.ubez/im.ibez;                                  % [Ohm]
    im.Lbez = im.Xbez/im.omega_bez;                             % [H]
    im.Rs_norm = im.Rs/im.Xbez;
    im.Rr_norm = im.Rr/im.Xbez;
    im.Ls_norm = im.Ls/im.Lbez;
    im.Lr_norm = im.Lr/im.Lbez;
    im.Lm_norm = im.Lm/im.Lbez;
    im.alpha_norm = im.Rr_norm/im.Lr_norm;                      % alpha/omega_bez
    im.psi_norm = im.psi_nom/im.psi_bez;
    im.Jm_norm = 1/2*im.Jm*im.omega_m_bez/im.torque_nom;        % [s] (library definition, with the factor 1/2)
    im.mu_norm = im.Lm_norm/im.Jm_norm/im.Lr_norm;
    im.sigma_norm = im.Ls_norm*(1 - (im.Lm_norm)^2/im.Ls_norm/im.Lr_norm);
    im.beta_norm = im.Lm_norm/im.sigma_norm/im.Lr_norm;
    im.gamma_norm = im.Rs_norm/im.sigma_norm + im.beta_norm*im.alpha_norm*im.Lm_norm;

    im.load_friction = im.torque_nom*im.load_friction_factor/im.omega_m_bez;    % [Nm s/rad] viscous load

    fprintf('Induction Machine: %s\n', im.name);
    fprintf('IM Normalization Voltage Factor: %.1f V | IM Normalization Current Factor: %.1f A\n', im.ubez, im.ibez);
    fprintf('Rotor Resistance: %.5f Ohm\n', im.Rr);
    fprintf('Magnetization Inductance: %.5f H\n', im.Lm);
    fprintf('Ls = %.3f mH, Lm = %.3f mH, Lr = %.3f mH, sigma*Ls = %.3f mH | rotor time constant Lr/Rr = %.3f s | torque_nom = %.0f Nm | friction %.2f Nm s/rad (%.2f pu at omega_m_bez)\n', ...
        im.Ls*1e3, im.Lm*1e3, im.Lr*1e3, im.sigma*1e3, 1/im.alpha, im.torque_nom, im.load_friction, im.load_friction_factor);
    fprintf('---------------------------\n');
end

function [Rr, Tn] = optimize_rotor_resistor(im, Rr_start, deltaRr, Rr_max, toll)
% optimizeRotorResistor of im_setup: torque of the equivalent circuit (T model, rated phase voltage, rated
% slip) for increasing Rr until it falls within torque_nom*(1 +- toll). If no value is found the last one
% (Rr >= Rr_max) is returned, as in the library.
    Rr_temp = Rr_start;
    Tn = im.torque_nom/2;
    T_min = im.torque_nom*(1 - toll);
    T_max = im.torque_nom*(1 + toll);
    w = im.omega_bez;
    s = im.slip_nom;
    while (Rr_temp < Rr_max)
        a = w*im.Lm*Rr_temp/s;
        b = w*im.Lm*a;
        c = (Rr_temp/s)^2 + (w*im.Lr)^2;
        d = (Rr_temp/s)^2 + w^2*im.Ldr*im.Lr;
        e = w^2*im.Lm*im.Lr;

        ZssRe = im.Rs + b/c;
        ZssIm = w*im.Lds + w*im.Lm*d/c;
        Zss_abs2 = ZssRe^2 + ZssIm^2;

        Is_ssRe = im.u_nom/sqrt(3)*ZssRe/Zss_abs2;
        Is_ssIm = -im.u_nom/sqrt(3)*ZssIm/Zss_abs2;

        Ir_ssRe = (a*Is_ssIm - e*Is_ssRe)/c;
        Ir_ssIm = -(a*Is_ssRe + e*Is_ssIm)/c;

        Ir_ss_abs2 = Ir_ssRe^2 + Ir_ssIm^2;
        Pmech = 3*Ir_ss_abs2*Rr_temp*(1 - s)/s;
        Tn = Pmech/im.omega_m_load;

        if (Tn >= T_min && Tn <= T_max)
            break;
        end

        Rr_temp = Rr_temp + deltaRr;
    end
    Rr = Rr_temp;
end
