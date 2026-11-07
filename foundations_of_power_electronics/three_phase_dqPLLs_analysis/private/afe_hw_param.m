function [hw, hw_in] = afe_hw_param(application_voltage, pwr_nom, fpwm)
%AFE_HW_PARAM  Hardware data of the AFE of frt_dqpll (local copy of the data sets of three_phase_afe_hwdata of the
%   library) and the hw_afe_three_phase_setup object.
%
%   [hw, hw_in] = afe_hw_param(application_voltage, pwr_nom, fpwm)
%     hw_in : the numbers of the data set selected by the application voltage (690, 480 or 400 V): name, uafe_nom [V],
%             iafe_nom [A] (at cos(phi) = 0.9), iafe_freq [Hz], udc [V], CFi, CFi1, CFi2 [F] and RCFi, RCFi1, RCFi2
%             [Ohm] of the DC link, LFu, LFu_cm [H] and RLFu, RLFu_cm [Ohm] of the filter inductor (5 % of the
%             reactance at 25 Hz), CFu [F] and RCFu [Ohm] of the filter capacitor, Rbrake [Ohm]
%     hw    : hw_afe_three_phase_setup object (normalization ubez = uafe_nom*sqrt(2/3), ibez = iafe_nom*sqrt(2))
%
%   Fields read by frt_dqpll.slx (LCL filter/LCL_filter): LFu, RLFu, LFu_cm, RLFu_cm (threephase inductor), CFu, RCFu
%   (the three filter capacitors), ubez and ibez (normalization of the measurements). The converter side of the
%   filter is open: the DC-link data are the data set, nothing more (the legacy init replaced CFi with twice the
%   capacitance of a PSFB stage of the shared template; no block reads it here).

    switch application_voltage
        case 690
            hw_in.name = 'afe690V';
            hw_in.uafe_nom = 690;                   % [V] line-to-line
            hw_in.iafe_freq = 50;                   % [Hz]
            hw_in.udc = 1070;                       % [V] DC link
            hw_in.LFu = 500e-6;                     % [H] grid-side filter inductor
            hw_in.LFu_cm = 500e-6;                  % [H] common-mode inductor
            hw_in.CFu = 200e-6;                     % [F] filter capacitor
            hw_in.Rbrake = 4;                       % [Ohm]
        case 480
            hw_in.name = 'afe480V';
            hw_in.uafe_nom = 480;
            hw_in.iafe_freq = 60;
            hw_in.udc = 750;
            hw_in.LFu = 330e-6;
            hw_in.LFu_cm = 330e-6;
            hw_in.CFu = 360e-6;
            hw_in.Rbrake = 2.5;
        otherwise
            hw_in.name = 'afe400V';
            hw_in.uafe_nom = 400;
            hw_in.iafe_freq = 50;
            hw_in.udc = 660;
            hw_in.LFu = 330e-6;
            hw_in.LFu_cm = 330e-6;
            hw_in.CFu = 360e-6;
            hw_in.Rbrake = 2.5;
    end
    hw_in.iafe_nom = pwr_nom/sqrt(3)/hw_in.uafe_nom/0.9;   % [A] at cos(phi) = 0.9
    hw_in.CFi = 7.2e-3;                                     % [F] DC-link capacitance, two banks for the three-level stages
    hw_in.CFi1 = 2*hw_in.CFi;
    hw_in.CFi2 = 2*hw_in.CFi;
    hw_in.RCFi = 1e-3;                                      % [Ohm] ESR
    hw_in.RCFi1 = hw_in.RCFi/2;
    hw_in.RCFi2 = hw_in.RCFi/2;
    hw_in.RLFu = 157*0.05*hw_in.LFu;                        % [Ohm] 5 % of the reactance at 25 Hz
    hw_in.RLFu_cm = 157*0.05*hw_in.LFu_cm;                  % [Ohm]
    hw_in.RCFu = 50e-3;                                     % [Ohm]

    h = hw_in;
    hw = hw_afe_three_phase_setup(h.name, pwr_nom, h.uafe_nom, h.iafe_nom, h.iafe_freq, h.udc, fpwm, h.CFi, h.RCFi, ...
        h.CFi1, h.RCFi1, h.CFi2, h.RCFi2, h.LFu, h.RLFu, h.LFu_cm, h.RLFu_cm, h.CFu, h.RCFu, h.Rbrake);
    hw.displayInfo();
end
