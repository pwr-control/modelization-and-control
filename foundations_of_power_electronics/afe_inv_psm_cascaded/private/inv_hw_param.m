function hw = inv_hw_param(application_voltage, pwr_nom, fpwm)
%INV_HW_PARAM  Hardware data of the three-phase two-level inverter of one module of afe_inv_psm_n.slx:
%   output inductor and per-unit bases of the inverter data set.
%
%   hw = inv_hw_param(application_voltage, pwr_nom, fpwm)
%     application_voltage : [V] rms line-to-line voltage class of the data set: 690, 480 or 400 (any other
%                           value: 400 V set)
%     pwr_nom             : [W] nominal power of the inverter
%     fpwm                : [Hz] switching frequency (stored in fpwm_base)
%
%   Local copy of three_phase_inverter_hwdata + hw_inverter_three_phase_setup of the library. Same data sets,
%   same formulas, same field names. Fields read by afe_inv_psm_n.slx (both modules):
%     LFi, LFi_cm, RLFi, RLFi_cm      block L_filter/threephase inductor: Lp, L0, Rp, R0
%     udc_nom                         not read by the model; the init uses it for the voltage scales
%                                     u_psm_scale and u_psm_scale_ekf, as the legacy init did
%   The inverter has no DC link of its own: it sits on the DC link of the AFE of the same module
%   (afe_hw_param, field CFi).
%
%   Bases of the data set (not used by the control, which works in the bases of the machine):
%     ubez = us_nom*sqrt(2/3)    peak phase voltage (449.1 V for the 690 V set, us_nom 550 V)
%     ibez = is_nom*sqrt(2)      peak current (523.3 A, is_nom 370 A rms)

    switch application_voltage
        case 690
            hw.name = 'inv690V_250kW';
            hw.us_nom = 550;                % [V] rms line-to-line, nominal output voltage
            hw.is_nom = 370;                % [A] rms, nominal output current
            hw.udc_nom = 1070;              % [V]
            LFi = 230e-6; LFi_cm = 230e-6;  % [H]
        case 480
            hw.name = 'inv480V_250kW';
            hw.us_nom = 400;                % as in the library data set (400 V, not 480 V)
            hw.is_nom = 470;
            hw.udc_nom = 750;
            LFi = 160e-6; LFi_cm = 160e-6;
        otherwise
            hw.name = 'inv400V_250kW';
            hw.us_nom = 400;
            hw.is_nom = 470;
            hw.udc_nom = 660;
            LFi = 160e-6; LFi_cm = 160e-6;
    end
    hw.application_voltage = application_voltage;
    hw.pwr_nom = pwr_nom;                                   % [W]
    hw.f_nom = 15.6;                                        % [Hz] (field f_nom of the library data set)
    hw.fpwm_base = fpwm;                                    % [Hz]

    % bases (hw_inverter_three_phase_setup)
    hw.ubez = hw.us_nom*sqrt(2/3);                          % [V] peak phase-to-neutral
    hw.ibez = hw.is_nom*sqrt(2);                            % [A] peak

    % output inductor (block threephase inductor): positive and zero sequence
    hw.LFi = LFi;                                           % [H]
    hw.LFi_cm = LFi_cm;                                     % [H]
    hw.RLFi = 157*0.05*LFi;                                 % [Ohm] 5 % of the reactance at 25 Hz (library rule)
    hw.RLFi_cm = 157*0.05*LFi_cm;                           % [Ohm]

    fprintf('Hardware inverter: %s, %.0f kW, %.0f V, %.0f A rms | ubez = %.1f V, ibez = %.1f A | udc_nom = %.0f V\n', ...
        hw.name, pwr_nom/1e3, hw.us_nom, hw.is_nom, hw.ubez, hw.ibez, hw.udc_nom);
    fprintf('Output inductor LFi = %.0f uH (RLFi = %.2f mOhm)\n', hw.LFi*1e6, hw.RLFi*1e3);
end
