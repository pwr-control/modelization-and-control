function hw = afe_hw_param(application_voltage, pwr_nom, fpwm)
%AFE_HW_PARAM  Hardware data of the single-phase AFE (full bridge + LCL filter) of single_phase_afe.slx.
%
%   hw = afe_hw_param(application_voltage, pwr_nom, fpwm)
%     application_voltage : [V] rms line-to-line voltage of the three-phase grid the bridge is connected
%                           to (between two phases): 690, 480 or 400
%     pwr_nom             : [W] nominal power of the converter
%     fpwm                : [Hz] switching frequency (stored in fpwm_base)
%
%   Local replacement of single_phase_inverter_hwdata + hw_single_phase_inverter_setup of the library
%   (the "single phase inverter" data set): same numbers, same formulas and same field names, so that
%   the per-unit scaling of the model does not change. Fields read by single_phase_afe.slx: ubez, ibez,
%   udc_nom, CFi, LFu1, RLFu1, LFu2, RLFu2, CFu, RCFu.
%
%   Bases (legacy convention of the library data set, kept on purpose):
%     us_nom  = application_voltage*1.35*0.9/sqrt(2)   rms output voltage of a stand-alone inverter
%               (592.8 V at 690 V), NOT the grid voltage seen by the AFE (690 V rms = 975.8 V peak)
%     ubez    = us_nom*sqrt(2)      peak voltage base (838.4 V): the grid voltage is 1.164 pu
%     is_nom  = pwr_nom/us_nom      (379.6 A), ibez = is_nom*sqrt(2) (536.8 A peak)
%     udc_nom = 2/sqrt(3)*1.1*application_voltage*sqrt(2)   (1239.4 V): base of udc_pu
%   The grid-side figures of this AFE (u_grid_nom, i_grid_nom, modulation index) are in hw.grid.
%
%   DC link: the model has one capacitor CFi (the legacy init read hwdata.afe.CFi/3 = 2.4 mF, the
%   three-phase 7.2 mF data set divided by three): 2.4 mF is the value kept here, see init_model for the
%   ripple it gives. The LCL filter is the library single-phase set: LFu = 500 uH split in two halves,
%   CFu = 200 uF with 50 mOhm in series.

    switch application_voltage
        case 690
            hw.name = 'single_phase_afe_690V';
            hw.f_nom = 50;                  % [Hz]
            hw.Rbrake = 4;                  % [Ohm] (not in the model)
        case 480
            hw.name = 'single_phase_afe_480V';
            hw.f_nom = 60;
            hw.Rbrake = 2.5;
        otherwise
            hw.name = 'single_phase_afe_400V';
            hw.f_nom = 50;
            hw.Rbrake = 2.5;
    end
    hw.application_voltage = application_voltage;           % [V] rms line-to-line of the grid
    hw.pwr_nom = pwr_nom;                                   % [W]
    hw.fpwm_base = fpwm;                                    % [Hz]

    % legacy bases of the library data set (see the header)
    hw.us_nom = application_voltage*1.35*0.9/sqrt(2);       % [V] rms
    hw.is_nom = pwr_nom/hw.us_nom;                          % [A] rms
    hw.udc_nom = 2/sqrt(3)*1.1*application_voltage*sqrt(2); % [V]
    hw.ubez = hw.us_nom*sqrt(2);                            % [V] peak
    hw.ibez = hw.is_nom*sqrt(2);                            % [A] peak
    hw.udc_bez = hw.udc_nom;                                % [V]
    hw.omega_bez = 2*pi*hw.f_nom;                           % [rad/s]
    hw.norm.Xbez = hw.ubez/hw.ibez;                         % [Ohm]
    hw.norm.Lbez = hw.norm.Xbez/hw.omega_bez;               % [H]

    % DC link (block dclink_for_two_levels: Cdc, ESR 2 mOhm inside the block)
    hw.CFi = 7.2e-3/3;                                      % [F] 2.4 mF, as the legacy hwdata.afe.CFi/3
    hw.RCFi = 1e-3;                                         % [Ohm] (informative, the block has its own ESR)

    % LCL filter (converter side 1, capacitor, grid side 2)
    LFu = 500e-6;                                           % [H] total of the library data set
    hw.LFu = LFu;
    hw.LFu1 = LFu/2;                                        % [H]
    hw.LFu2 = LFu/2;                                        % [H]
    hw.RLFu = 157*0.05*LFu;                                 % [Ohm] 5 % of the reactance at 25 Hz (library rule)
    hw.RLFu1 = hw.RLFu/2;                                   % [Ohm]
    hw.RLFu2 = hw.RLFu/2;                                   % [Ohm]
    hw.CFu = 200e-6;                                        % [F]
    hw.RCFu = 50e-3;                                        % [Ohm] series resistance of the capacitor

    % grid-side figures of the AFE (the bridge is connected between two phases of the grid)
    g.u_nom = application_voltage;                          % [V] rms at the terminals
    g.u_peak = g.u_nom*sqrt(2);                             % [V]
    g.u_pu = g.u_peak/hw.ubez;                              % [pu] 1.164 at 690 V
    g.i_nom = pwr_nom/g.u_nom;                              % [A] rms at pwr_nom, unity power factor
    g.i_peak = g.i_nom*sqrt(2);                             % [A]
    g.i_pu = g.i_peak/hw.ibez;                              % [pu]
    w = 2*pi*hw.f_nom;
    g.u_conv_peak = sqrt(g.u_peak^2 + (w*hw.LFu*g.i_peak)^2);   % [V] converter voltage at pwr_nom (drop on LFu in quadrature)
    g.m_nom = g.u_conv_peak/hw.udc_nom;                     % [-] modulation index at udc_nom
    g.f_res = sqrt((hw.LFu1 + hw.LFu2)/(hw.LFu1*hw.LFu2*hw.CFu))/2/pi;   % [Hz] LCL resonance, stiff grid
    g.q_cfu = g.u_nom^2*w*hw.CFu;                           % [VAr] reactive power of CFu at u_nom
    g.x_lfu_pu = w*hw.LFu/hw.norm.Xbez;                     % [pu] total filter reactance
    hw.grid = g;

    fprintf('Hardware single-phase AFE: %s, %.0f kW, grid %.0f V rms (%.0f V peak = %.3f pu), %.0f A rms (%.0f A peak = %.3f pu)\n', ...
        hw.name, pwr_nom/1e3, g.u_nom, g.u_peak, g.u_pu, g.i_nom, g.i_peak, g.i_pu);
    fprintf('Bases: ubez = %.1f V, ibez = %.1f A, udc_nom = %.1f V | modulation index at pwr_nom = %.3f\n', ...
        hw.ubez, hw.ibez, hw.udc_nom, g.m_nom);
    fprintf('DC link %.1f mF | LCL: LFu1 = %.0f uH, CFu = %.0f uF (%.1f %% of Pnom), LFu2 = %.0f uH, f_res = %.0f Hz\n', ...
        hw.CFi*1e3, hw.LFu1*1e6, hw.CFu*1e6, g.q_cfu/pwr_nom*100, hw.LFu2*1e6, g.f_res);
    fprintf('---------------------------\n');
end
