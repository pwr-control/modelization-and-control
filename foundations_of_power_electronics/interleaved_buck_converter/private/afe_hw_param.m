function hw = afe_hw_param(application_voltage, pwr_nom, fpwm)
%AFE_HW_PARAM  Hardware data of the three-phase AFE with LCL filter (local parameter layer).
%
%   hw = afe_hw_param(application_voltage, pwr_nom, fpwm)
%
%   Replaces three_phase_afe_hwdata + hw_afe_three_phase_setup of the library for this model: same data
%   sets (690 V, 480 V, 400 V), same normalization (ubez, ibez) and field names, but the output filter is
%   an LCL: LFu1 (converter side) + CFu (star, floating) + LFu2 (grid side), each inductor with its own
%   zero-sequence (common mode) data LFu*_cm, RLFu*_cm (threephase_inductor block: Lp/L0, Rp/R0).
%
%   application_voltage : 400, 480 or 690 [V] line-to-line
%   pwr_nom             : [W] nominal power
%   fpwm                : [Hz] base switching frequency, stored in fpwm_base
%
%   Fields read by grid_forming_dq.slx: ubez, ibez, udc_nom, CFi, LFu1, RLFu1, LFu1_cm, RLFu1_cm, LFu2,
%   RLFu2, LFu2_cm, RLFu2_cm, CFu, RCFu.
%
%   Default split: LFu1 = LFu2 = the LFu value of the library data set (500 uH at 690 V, 330 uH at 400 V
%   and 480 V), i.e. the values that grid_forming_dq.slx already had on both inductors before the split.
%   The legacy field LFu is not defined on purpose: a block or script still reading it fails at once
%   instead of using a wrong value. LFu_tot = LFu1 + LFu2 is the total series inductance.

    switch application_voltage
        case 690
            hw.name = 'afe690V';
            hw.us_nom = 690;                    % [V] line-to-line
            hw.f_nom = 50;                      % [Hz]
            hw.udc_nom = 1070;                  % [V] DC link
            LFu1 = 500e-6;                       % [H] per inductor
            LFu2 = 100e-6;                       % [H] per inductor
            hw.CFu = 200e-6;                    % [F] filter capacitor, star connected
            hw.Rbrake = 4;                      % [Ohm]
        case 480
            hw.name = 'afe480V';
            hw.us_nom = 480;
            hw.f_nom = 60;
            hw.udc_nom = 750;
            LFu1 = 330e-6;
            LFu2 = 80e-6;
            hw.CFu = 360e-6;
            hw.Rbrake = 2.5;
        otherwise
            hw.name = 'afe400V';
            hw.us_nom = 400;
            hw.f_nom = 50;
            hw.udc_nom = 660;
            LFu1 = 330e-6;
            LFu2 = 80e-6;
            hw.CFu = 360e-6;
            hw.Rbrake = 2.5;
    end
    hw.pwr_nom = pwr_nom;                                   % [W]
    hw.fpwm_base = fpwm;                                    % [Hz]
    hw.is_nom = pwr_nom/sqrt(3)/hw.us_nom/0.9;              % [A] rms at cos(phi) = 0.9

    % DC link: single capacitor of the two-level stage; the two banks are kept for the three-level stages
    hw.CFi = 7.2e-3;                                        % [F]
    hw.RCFi = 1e-3;                                         % [Ohm] ESR
    hw.CFi1 = 2*hw.CFi;
    hw.CFi2 = 2*hw.CFi;
    hw.RCFi1 = hw.RCFi/2;
    hw.RCFi2 = hw.RCFi/2;

    % LCL filter: converter side (1), capacitor, grid side (2)
    hw.LFu1 = LFu1;                                          % [H] converter side, positive sequence
    hw.LFu2 = LFu2;                                          % [H] grid side, positive sequence
    hw.LFu1_cm = hw.LFu1;                                   % [H] zero sequence (L0 of the block)
    hw.LFu2_cm = hw.LFu2;                                   % [H]
    hw.RLFu1 = 157*0.05*hw.LFu1;                            % [Ohm] 5 % of the reactance at 25 Hz (library rule)
    hw.RLFu2 = 157*0.05*hw.LFu2;                            % [Ohm]
    hw.RLFu1_cm = 157*0.05*hw.LFu1_cm;                      % [Ohm]
    hw.RLFu2_cm = 157*0.05*hw.LFu2_cm;                      % [Ohm]
    hw.RCFu = 50e-3;                                        % [Ohm] series resistance of the capacitor
    hw.LFu_tot = hw.LFu1 + hw.LFu2;                         % [H]

    % normalization: peak phase voltage and peak phase current
    hw.ubez = hw.us_nom*sqrt(2/3);                          % [V]
    hw.ibez = hw.is_nom*sqrt(2);                            % [A]
    hw.zbez = hw.ubez/hw.ibez;                              % [Ohm]

    % derived LCL figures (used by the checks of init_model, not by the model)
    w = 2*pi*hw.f_nom;
    hw.omega_res = sqrt((hw.LFu1 + hw.LFu2)/(hw.LFu1*hw.LFu2*hw.CFu));    % [rad/s] LCL resonance (stiff grid)
    hw.f_res = hw.omega_res/2/pi;                                         % [Hz]
    hw.omega_lc1 = 1/sqrt(hw.LFu1*hw.CFu);                                % [rad/s] LFu1-CFu resonance (open grid side)
    hw.z0_lcl = sqrt(hw.LFu1/hw.CFu);                                     % [Ohm] characteristic impedance
    hw.q_cfu = 3*(hw.us_nom/sqrt(3))^2*w*hw.CFu;                          % [VAr] capacitor reactive power at us_nom
    hw.q_cfu_pu = hw.q_cfu/hw.pwr_nom;                                    % [pu]
    hw.x_lfu1_pu = w*hw.LFu1/hw.zbez;                                     % [pu] reactance at f_nom
    hw.x_lfu2_pu = w*hw.LFu2/hw.zbez;                                     % [pu]
    hw.b_cfu_pu = w*hw.CFu*hw.zbez;                                       % [pu] capacitor susceptance at f_nom
    hw.tau_lfu1_pu = hw.LFu1/hw.zbez;                                     % [s] converter-side inductance in pu (time constant)
    hw.tau_lfu_tot_pu = hw.LFu_tot/hw.zbez;                               % [s]

    fprintf('Device AFE_THREE_PHASE: %s\n', hw.name);
    fprintf('Nominal Voltage: %d V | Nominal Current: %d A\n', hw.us_nom, round(hw.is_nom));
    fprintf('Current Normalization Data: %.2f A\n', hw.ibez);
    fprintf('Voltage Normalization Data: %.2f V\n', hw.ubez);
    fprintf('LCL filter: LFu1 = %.0f uH | CFu = %.0f uF (%.1f %% of Pnom) | LFu2 = %.0f uH | f_res = %.0f Hz\n', ...
        hw.LFu1*1e6, hw.CFu*1e6, hw.q_cfu_pu*100, hw.LFu2*1e6, hw.f_res);
    fprintf('---------------------------\n');
end
