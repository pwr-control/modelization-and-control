function hw = afe_hw_param(application_voltage, pwr_nom, fpwm)
%AFE_HW_PARAM  Hardware data of the three-phase two-level AFE of one module of afe_inv_psm_n.slx: DC link,
%   LCL filter on the grid side, brake resistor, per-unit bases.
%
%   hw = afe_hw_param(application_voltage, pwr_nom, fpwm)
%     application_voltage : [V] rms line-to-line grid voltage: 690, 480 or 400 (any other value: 400 V set)
%     pwr_nom             : [W] nominal power of the converter
%     fpwm                : [Hz] switching frequency (stored in fpwm_base)
%
%   Local replacement of three_phase_afe_hwdata + hw_afe_three_phase_setup of the library: same data sets,
%   same formulas and same field names, so that the per-unit scaling of the model does not change. Fields
%   read by afe_inv_psm_n.slx (both modules):
%     ubez, ibez                   measurements of the AFE control (LCL_filter/threephase_vi_measures_stack*)
%                                  and scopes
%     udc_nom                      DC link (afe/dclink1: Vdc_scale, initial voltage) and brake chopper
%                                  (brake/Constant1: reference of the hysteresis)
%     CFi, Rudc_measure            DC link of the module (block dclink_for_two_levels/dclink: Cdc,
%                                  Rudc_measure; literal 1e6 in the legacy model)
%     LFu, LFu_cm, RLFu, RLFu_cm   three-phase inductor of the LCL filter (LCL_filter/threephase inductor: Lp,
%                                  L0, Rp, R0)
%     CFu, RCFu                    the three capacitors of the LCL filter in star (LCL_filter/CFu1..3)
%     Rbrake                       brake resistor (brake/Rbrake/Resistor1)
%
%   Bases (convention of the library data set):
%     ubez = us_nom*sqrt(2/3)              peak phase-to-neutral grid voltage (563.4 V at 690 V)
%     is_nom = pwr_nom/sqrt(3)/us_nom/0.9  rms current with a 10 % margin on the power (232.4 A at 250 kW)
%     ibez = is_nom*sqrt(2)                peak current base (328.7 A); 1 pu of active current at 1 pu of
%                                          voltage is 3/2*ubez*ibez = 277.8 kW
%     udc_nom                              DC link (1070 V at 690 V), base of udc_pu
%
%   The legacy init overwrote CFi with 2*Cdc_dc1 of the PSFBC data set (2 x 3.6 mF): the same 7.2 mF of the
%   AFE data set, so the line is not repeated here.

    switch application_voltage
        case 690
            hw.name = 'afe690V';
            hw.us_nom = 690;                % [V] rms line-to-line
            hw.f_nom = 50;                  % [Hz]
            hw.udc_nom = 1070;              % [V]
            LFu = 500e-6; LFu_cm = 500e-6;  % [H]
            hw.CFu = 200e-6;                % [F]
            hw.Rbrake = 4;                  % [Ohm]
        case 480
            hw.name = 'afe480V';
            hw.us_nom = 480;
            hw.f_nom = 60;
            hw.udc_nom = 750;
            LFu = 330e-6; LFu_cm = 330e-6;
            hw.CFu = 360e-6;
            hw.Rbrake = 2.5;
        otherwise
            hw.name = 'afe400V';
            hw.us_nom = 400;
            hw.f_nom = 50;
            hw.udc_nom = 660;
            LFu = 330e-6; LFu_cm = 330e-6;
            hw.CFu = 360e-6;
            hw.Rbrake = 2.5;
    end
    hw.application_voltage = application_voltage;
    hw.pwr_nom = pwr_nom;                                   % [W]
    hw.fpwm_base = fpwm;                                    % [Hz]

    % bases (hw_afe_three_phase_setup)
    hw.is_nom = pwr_nom/sqrt(3)/hw.us_nom/0.9;              % [A] rms, 10 % margin on the power
    hw.ubez = hw.us_nom*sqrt(2/3);                          % [V] peak phase-to-neutral
    hw.ibez = hw.is_nom*sqrt(2);                            % [A] peak
    hw.udc_bez = hw.udc_nom;                                % [V]
    hw.omega_bez = 2*pi*hw.f_nom;                           % [rad/s]
    hw.norm.Xbez = hw.ubez/hw.ibez;                         % [Ohm]
    hw.norm.Lbez = hw.norm.Xbez/hw.omega_bez;               % [H]
    hw.norm.Cbez = 1/(hw.norm.Xbez*hw.omega_bez);           % [F]
    hw.norm.Pbez = 3/2*hw.ubez*hw.ibez;                     % [W] power at 1 pu of voltage and current

    % DC link of the module (block dclink_for_two_levels/dclink), shared by the AFE and the inverter
    hw.CFi = 7.2e-3;                                        % [F]
    hw.RCFi = 1e-3;                                         % [Ohm] (informative, the block has no ESR)
    hw.Rudc_measure = 1e6;                                  % [Ohm] measurement divider of the block

    % grid filter (block LCL_filter: three-phase inductor, CFu in star, then the transformer secondary)
    hw.LFu = LFu;                                           % [H] positive-sequence inductance
    hw.LFu_cm = LFu_cm;                                     % [H] zero-sequence inductance
    hw.RLFu = 157*0.05*LFu;                                 % [Ohm] 5 % of the reactance at 25 Hz (library rule)
    hw.RLFu_cm = 157*0.05*LFu_cm;                           % [Ohm]
    hw.RCFu = 50e-3;                                        % [Ohm] series resistance of each capacitor

    fprintf('Hardware three-phase AFE: %s, %.0f kW, %.0f V, %.0f Hz | ubez = %.1f V, ibez = %.1f A (is_nom %.1f A rms), 1 pu = %.1f kW | udc_nom = %.0f V\n', ...
        hw.name, pwr_nom/1e3, hw.us_nom, hw.f_nom, hw.ubez, hw.ibez, hw.is_nom, hw.norm.Pbez/1e3, hw.udc_nom);
    fprintf('DC link %.1f mF | filter LFu = %.0f uH (%.3f pu), CFu = %.0f uF star (%.3f pu), RCFu = %.0f mOhm | brake %.1f Ohm\n', ...
        hw.CFi*1e3, hw.LFu*1e6, hw.LFu/hw.norm.Lbez, hw.CFu*1e6, hw.CFu/hw.norm.Cbez, hw.RCFu*1e3, hw.Rbrake);
end
