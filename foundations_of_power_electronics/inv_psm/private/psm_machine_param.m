function psm = psm_machine_param(m)
%PSM_MACHINE_PARAM  Permanent magnet synchronous machine of inv_psm: per-system equivalent data, per-unit
%   bases and normalized parameters (local copy of pmsm_setup of the library, same formulas and field names).
%
%   psm = psm_machine_param(m), with m = param.psm.machine_in:
%     name, number_of_systems (galvanically insulated three-phase systems on the same rotor), pwr_nom [W]
%     (whole machine), i_nom [A rms] (whole machine, all systems in parallel), rpm_nom [rpm], number_poles,
%     eta [%], Ld, Lq [H] (whole machine), Jm [kg m^2] (whole machine), h5_percent, h7_percent [%]
%
%   The model simulates ONE three-phase system out of number_of_systems (the legacy psm_calculus: six
%   systems, 1.6 MW, so one system is about 267 kW). The "_m" fields are the values seen by one system:
%     Rs_m = Rs*n, Ld_m = Ld*n, Lq_m = Lq*n, La_m = La*n, Lb_m = Lb*n, Jm_m = Jm/n, load_friction_m = load_friction/n
%   Bases (per system): ibez = i_nom/n*sqrt(2) (peak), tau_bez = torque_nom/n, psi_m = tau_bez/(3/2*pp*ibez),
%   psi_bez = psi_m, omega_bez = omega_m_bez*pp, ubez = psi_bez*omega_bez (peak phase), Xbez = ubez/ibez,
%   Lbez = Xbez/omega_bez. Rs comes from the efficiency: Rs = (pwr_nom/3)*(1 - eta/100)/i_nom^2.
%   Inductance matrix of the Simscape block: La = (Ld + Lq)/3 (self), Lb = (Lq - Ld)/3 (mutual, saliency),
%   plus the small leakage Lmu = 5 uH that makes the matrix invertible.
%
%   Fields read by inv_psm.slx:
%     number_poles, psi_m, Lmu, La_m, Lb_m, Rs_m, Jm_m,     block pmsm/synchronous_motor/
%     load_friction_m                                         permanent_magnet_synchronous_machine (the mask
%                                                             prompt says "pole pairs" but the value given is
%                                                             number_poles, as in the legacy model)
%     ibez, ubez, psi_bez, omega_bez, omega_m_bez, tau_bez,   measurements and control (per unit)
%     rpm_nom, psi_m_norm, Rs_norm, Ls_norm, Ld_norm, Lq_norm

    psm.name = m.name;
    psm.number_of_systems = m.number_of_systems;
    psm.pwr_nom = m.pwr_nom;
    psm.rpm_nom = m.rpm_nom;
    psm.omega_m_bez = psm.rpm_nom/60*2*pi;                  % [rad/s] mechanical base (rated speed)
    psm.torque_nom = psm.pwr_nom/psm.omega_m_bez;           % [Nm] whole machine
    psm.i_nom = m.i_nom;
    psm.number_poles = m.number_poles;
    psm.eta = m.eta;
    psm.Ld = m.Ld;
    psm.Lq = m.Lq;
    psm.Jm = m.Jm;
    psm.h5_percent_psm = m.h5_percent;
    psm.h7_percent_psm = m.h7_percent;

    n = psm.number_of_systems;
    psm.Lmu = 5e-6;                                         % [H]
    psm.Rs = (psm.pwr_nom/3)*(1 - psm.eta/100)/(psm.i_nom)^2;   % [Ohm]
    psm.Rs_m = psm.Rs*n;
    psm.La = 1/3*(psm.Lq + psm.Ld);
    psm.Lb = 1/3*(psm.Lq - psm.Ld);
    psm.Lalpha = (psm.Ld + psm.Lq)/2;
    psm.Lbeta = psm.Lalpha;
    psm.Ls = psm.Lalpha;

    psm.Ld_m = psm.Ld*n;
    psm.Lq_m = psm.Lq*n;
    psm.Ls_m = psm.Ls*n;
    psm.Lalpha_m = psm.Lalpha*n;
    psm.Lbeta_m = psm.Lbeta*n;
    psm.La_m = psm.La*n;
    psm.Lb_m = psm.Lb*n;
    psm.Jm_m = psm.Jm/n;
    psm.radpersecond2rpm = 60/2/pi;
    psm.pp = psm.number_poles/2;                            % pole pairs
    psm.ibez = psm.i_nom/n*sqrt(2);                         % [A] peak, per system
    psm.tau_bez = psm.torque_nom/n;                         % [Nm] per system
    psm.psi_m = psm.torque_nom/n/(3/2*psm.pp*psm.ibez);     % [Vs] magnet flux (peak)
    psm.psi_bez = psm.psi_m;
    psm.omega_bez = psm.omega_m_bez*psm.pp;                 % [rad/s] electrical base
    psm.freq_bez = psm.omega_bez/(2*pi);                    % [Hz]
    psm.ubez = psm.psi_bez*psm.omega_bez;                   % [V] peak phase voltage
    psm.u_nom = psm.ubez/sqrt(2/3);                         % [V] rms line-to-line

    psm.Xbez = psm.ubez/psm.ibez;                           % [Ohm]
    psm.Lbez = psm.Xbez/psm.omega_bez;                      % [H]
    psm.Rs_norm = psm.Rs_m/psm.Xbez;
    psm.Ld_norm = psm.Ld_m/psm.Lbez;
    psm.Lq_norm = psm.Lq_m/psm.Lbez;
    psm.Ls_norm = psm.Ls_m/psm.Lbez;
    psm.Lalpha_norm = psm.Lalpha_m/psm.Lbez;
    psm.Lbeta_norm = psm.Lbeta_m/psm.Lbez;
    psm.La_norm = psm.La_m/psm.Lbez;
    psm.Lb_norm = psm.Lb_m/psm.Lbez;
    psm.psi_m_norm = psm.psi_m/psm.psi_bez;
    psm.Jm_norm = 1/2*psm.Jm*psm.omega_m_bez/psm.torque_nom;   % [s] (library definition)

    psm.load_friction = psm.torque_nom/psm.omega_m_bez;     % [Nm s/rad] viscous friction: torque_nom at rated speed
    psm.load_friction_m = psm.load_friction/n;

    fprintf('PSM: %s, %d systems, %.0f kW, %.1f rpm, %d poles | per system: ubez = %.1f V, ibez = %.1f A, tau_bez = %.0f Nm, psi_m = %.3f Vs, f = %.2f Hz\n', ...
        psm.name, n, psm.pwr_nom/1e3, psm.rpm_nom, psm.number_poles, psm.ubez, psm.ibez, psm.tau_bez, psm.psi_m, psm.freq_bez);
    fprintf('     per system: Rs = %.2f mOhm, Ld = %.2f mH, Lq = %.2f mH | pu: Rs = %.4f, Ld = %.4f, Lq = %.4f, Ls = %.4f\n', ...
        psm.Rs_m*1e3, psm.Ld_m*1e3, psm.Lq_m*1e3, psm.Rs_norm, psm.Ld_norm, psm.Lq_norm, psm.Ls_norm);
end
