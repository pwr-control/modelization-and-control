function hw = inv_hw_param(application_voltage, pwr_nom, fpwm)
%INV_HW_PARAM  Hardware data of the three-phase two-level inverter of im_analysis_and_design: output
%   inductor, DC link, per-unit bases of the inverter.
%
%   hw = inv_hw_param(application_voltage, pwr_nom, fpwm)
%     application_voltage : [V] rms line-to-line voltage class of the data set: 690, 480 or 400 (any other
%                           value: 400 V set)
%     pwr_nom             : [W] nominal power of the inverter
%     fpwm                : [Hz] switching frequency (stored in fpwm_base)
%
%   Local replacement of three_phase_inverter_hwdata + hw_inverter_three_phase_setup of the library for the
%   inverter, and of the DC-link part of three_phase_afe_hwdata for the DC link (the legacy model read the DC
%   link of the AFE data set: hwdata.afe.CFi1, hwdata.afe.CFi2, hwdata.afe.udc_nom). Same data sets, same
%   formulas, same field names. Fields read by the models:
%     LFi, LFi_cm, RLFi, RLFi_cm      block inverter/ctrl/LFi (threephase_inductor): Lp, L0, Rp, R0
%     CFi1, CFi2, Rudc_measure,       block inverter/ctrl/dc_source/dclink (dclink_with_center_tap): Cdc1,
%     udc_nom                         Cdc2, Rudc_measure, Vdc_scale (udc_pu = udc/Vdc_scale, logged as
%                                     vdc_pu_inv_mod1 and read by no block)
%
%   Bases (convention of the library data set, not used by the control of these models, which works in the
%   bases of the machine):
%     ubez = us_nom*sqrt(2/3)    peak phase-to-neutral voltage (449.1 V for the 690 V set, us_nom 550 V)
%     ibez = is_nom*sqrt(2)      peak current (523.3 A, is_nom 370 A rms)
%
%   DC link: two banks in series (2 x 14.4 mF = 7.2 mF), center tap available, 107 kOhm measurement divider
%   across each bank (literal 107e3 in the block of the legacy model, now param.inv.hw.Rudc_measure). The
%   battery of the models sits across the whole link, so udc is set by the battery (1200 V), not by udc_nom.

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

    % output inductor (block LFi, threephase_inductor): positive and zero sequence
    hw.LFi = LFi;                                           % [H]
    hw.LFi_cm = LFi_cm;                                     % [H]
    hw.RLFi = 157*0.05*LFi;                                 % [Ohm] 5 % of the reactance at 25 Hz (library rule)
    hw.RLFi_cm = 157*0.05*LFi_cm;                           % [Ohm]

    % DC link (DC-link data of the AFE data set of the same voltage class, as the legacy model)
    hw.CFi = 7.2e-3;                                        % [F] series of the two banks
    hw.RCFi = 1e-3;                                         % [Ohm] (informative, the block has no ESR)
    hw.CFi1 = 2*hw.CFi;                                     % [F] upper bank, 14.4 mF
    hw.CFi2 = 2*hw.CFi;                                     % [F] lower bank, 14.4 mF
    hw.RCFi1 = hw.RCFi/2;                                   % [Ohm] (informative)
    hw.RCFi2 = hw.RCFi/2;                                   % [Ohm] (informative)
    hw.Rudc_measure = 107e3;                                % [Ohm] measurement divider of the block, across each bank

    fprintf('Hardware inverter: %s, %.0f kW, %.0f V, %.0f A rms | ubez = %.1f V, ibez = %.1f A | udc_nom = %.0f V\n', ...
        hw.name, pwr_nom/1e3, hw.us_nom, hw.is_nom, hw.ubez, hw.ibez, hw.udc_nom);
    fprintf('Output inductor LFi = %.0f uH (RLFi = %.2f mOhm) | DC link 2 x %.1f mF (%.1f mF in series)\n', ...
        hw.LFi*1e6, hw.RLFi*1e3, hw.CFi1*1e3, hw.CFi*1e3);
    fprintf('---------------------------\n');
end
