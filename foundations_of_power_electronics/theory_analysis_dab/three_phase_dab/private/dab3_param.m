function dab3 = dab3_param(bench_case)
%DAB3_PARAM  Numbers of the three-phase DAB (DAB3): 1200 V -> 1200 V, 250 kW, n = 1, fs = 10 kHz.
%
%   dab3 = dab3_param()
%   dab3 = dab3_param(bench_case)
%
%   Returns the DAB3 part of "param" used by init_model.m: pwr_nom, fs, .pwm (inputs of the timing
%   object), .hw_in (the inputs of hw_dab_three_phase_setup), .tank (series inductance of the sizing,
%   written over the one that the class derives), .mod (modulator limit and feed-forward), .ctrl_in (PI
%   gains), .bench (current reference and initial state of charge of the batteries). bench_case selects
%   the bench: 'forward' (default), 'reverse', 'light', 'mismatch' (see the bench section below). Being in
%   ./private it is seen only by the files of this folder.
%
%   The sizing is in ../dab3_sizing.md: three single-phase transformers in star-star (floating star
%   points), six-step modulation of both bridges, phase-shift control (De Doncker), 2 kV SiC MOSFET
%   half-bridge modules on both bridges. Values marked "TBC" are placeholders of the first run, to be
%   confirmed with the hardware.
%
%   Rev. 02, 2026-10-29: bench cases (Rev. 01: first sizing).

    if nargin < 1 || isempty(bench_case)
        bench_case = 'forward';
    end

    % ------------------------------------------------------------------ operating point and PWM
    dab3.pwr_nom = 250e3;                       % [W] nominal power, both directions
    dab3.fs = 10e3;                             % [Hz] transformer frequency = switching frequency of each device
    % The library modulator (three_phase_dab_modulator) builds the six-step pattern at fPWM/6: one step
    % of the pattern per PWM period, and the control trigger (TRGO_DAB) once per PWM period.
    dab3.pwm.fpwm = 6*dab3.fs;                  % [Hz] 60 kHz = fPWM_DAB of the timing object
    dab3.pwm.trgo = 0;                          % single update: ts_dab = 1/fPWM_DAB = 16.7 us, the period of the
                                                % trigger of the control (with 1 the PI would integrate with half
                                                % the true sample time)
    dab3.pwm.dead_time = 500e-9;                % [s] TBC. NOTE: the library modulator does not apply it (its
                                                % dead_time mask parameter is not used): the legs switch with no
                                                % dead time and the model has no Coss (see dab3_sizing.md, Sec. 5)

    % ------------------------------------------------------------------ DC sides
    h.name = 'DAB3_1200V_1200V_250kW';
    h.n1 = 14;                                  % turns per phase, 1:1. Per phase: 3 x AM-NC-320C nanocrystalline
    h.n2 = 14;                                  % cut cores (Ac = 47 cm2), 0.20 T peak with the six-step voltage
    h.udc1 = 1200;                              % [V] side 1
    h.udc2 = h.udc1*h.n2/h.n1;                  % [V] 1200 V, side 2
    h.uac1 = sqrt(6)/pi*h.udc1;                 % [V] 936 V, line-to-line rms of the fundamental of the six-step
    h.uac2 = sqrt(6)/pi*h.udc2;                 %     voltage (stored by the class, not used by the model)
    h.idc1 = dab3.pwr_nom/h.udc1;               % [A] 208 A, normalization of the current loop (idc1_bez)
    h.idc2 = dab3.pwr_nom/h.udc2;               % [A] 208 A

    % ------------------------------------------------------------------ transformer (per phase)
    h.Lm = 1e-3;                                % [H] primary side, gapped core: Im = 12-13 A peak, it helps the ZVS of
                                                % both bridges at light load (dab3_sizing.md, Sec. 4)
    h.Rfe = 3e3;                                % [Ohm] 107 W per transformer with the six-step voltage (566 V rms
                                                % phase to star point)
    h.Rs1 = 1.3e-3;                             % [Ohm] TBC: winding, terminations and external inductor, primary;
                                                % the class sets Rs2 = Rs1/n^2
    h.core_area = 47e-4;                        % [m2] 3 x AM-NC-320C (only for the flux density printed by init_model)
    h.core_length = 0.72;                       % [m] TBC, mean magnetic path of the core set
    h.core_mur = h.Lm*h.core_length/(4*pi*1e-7*h.n1^2*h.core_area);   % 622, effective permeability of the gapped
                                                % core: the SSC uses it only for its Bc output, which is then the
                                                % true flux density

    % ------------------------------------------------------------------ DC links
    % Six-pulse ripple of the bridge DC current at 6 fs = 60 kHz (115 A to 240 A at full load): about 40 A
    % rms in the capacitors on both sides, 0.6 V p-p with 500 uF. Film banks.
    h.Cdc_dc1 = 500e-6;                         % [F] TBC
    h.Cdc1_dc1 = 2*h.Cdc_dc1;                   % split banks: stored by the class, not used by the model
    h.Cdc2_dc1 = 2*h.Cdc_dc1;
    h.Cdc_dc2 = 500e-6;                         % [F] TBC
    h.Cdc1_dc2 = 2*h.Cdc_dc2;
    h.Cdc2_dc2 = 2*h.Cdc_dc2;
    h.RCdc_dc1 = 1e-3;                          % [Ohm] TBC, ESR of the bank (1.6 W at 40 A rms)
    h.RCdc1_dc1 = h.RCdc_dc1/2;
    h.RCdc2_dc1 = h.RCdc_dc1/2;
    h.RCdc_dc2 = 1e-3;                          % [Ohm] TBC
    h.RCdc1_dc2 = h.RCdc_dc2/2;
    h.RCdc2_dc2 = h.RCdc_dc2/2;

    % ------------------------------------------------------------------ DC chokes (between battery and DC link)
    h.Ldc_dc1 = 250e-6;                         % [H] TBC, 450 Hz with 500 uF
    h.RLdc_dc1 = 157*0.05*h.Ldc_dc1;            % [Ohm] 2 mOhm (85 W at 208 A)
    h.Ldc_dc2 = 250e-6;                         % [H] TBC
    h.RLdc_dc2 = 157*0.05*h.Ldc_dc2;            % [Ohm] 2 mOhm
    dab3.hw_in = h;

    % ------------------------------------------------------------------ series inductance (leakage + external)
    % De Doncker, lossless, Lm infinite, L = Ld1 + Ld2' per phase, V2' = V2 n1/n2, |phi| <= pi/3:
    %     P = V1 V2' / (w L) * phi * (2/3 - phi/(2 pi))
    % (pi/3 <= |phi| <= 2 pi/3: P = V1 V2'/(w L) (phi - phi^2/pi - pi/18), maximum 7 pi/36 V1 V2'/(w L) at
    % pi/2). phi_nom = 30 deg at 250 kW and 1200 V / 1200 V gives L = 28.0 uH, Pmax = 500 kW: margin for
    % the voltage range and the transients with moderate reactive current (I_rms 161 A, sizing Sec. 3).
    % hw_dab_three_phase_setup derives its own Ls = udc1^2/(4 fpwm P) from fPWM_DAB: init_model.m writes
    % these values over it.
    n12 = h.n1/h.n2;
    t.phi_nom = pi/6;                           % [rad] phase shift at nominal power and voltages
    w = 2*pi*dab3.fs;
    t.L = h.udc1*h.udc2/n12/(w*dab3.pwr_nom)*t.phi_nom*(2/3 - t.phi_nom/(2*pi));   % [H] 28.0 uH per phase
    t.Ld1 = t.L/2;                              % [H] 14 uH primary: leakage of the transformer (about 12 uH in
                                                % total, both sides) + external or integrated inductor
    t.Ld2 = t.Ld1/n12^2;                        % [H] 14 uH secondary, in secondary turns
    t.Pmax = 7*pi/36*h.udc1*h.udc2/n12/(w*t.L); % [W] 500 kW at 90 deg
    dab3.tank = t;

    % ------------------------------------------------------------------ modulator
    % maximum_abs_phase_shift of three_phase_dab_modulator: phi = u * phi_max, u = output of the current
    % control in [-1, 1]. The power grows up to 90 deg and falls beyond it: the old 2 pi/3 put the
    % maximum of the power inside the range of u, with a gain that changes sign. pi/3 = 1.7 Pnom at
    % nominal voltages, 1.45 Pnom with V2 = 0.85 V1.
    dab3.mod.phi_max = pi/3;                    % [rad]
    dab3.mod.k_ff = t.phi_nom/dab3.mod.phi_max; % 0.5: feed-forward of the current reference (Gain of the current
                                                % control, u_ff = k_ff * i_ref): 30 deg at 1 pu

    % ------------------------------------------------------------------ control (library pi_ctrl, ts = ts_dab)
    % Current loop on the DC input current of side 1 (dab_current_input_modA: the current of the DC choke where
    % it enters the DC link; the capacitor carries the six-pulse ripple, this current has none), normalized by
    % idc1. Plant d(i_pu)/du = 1.9 at the nominal point; the LC of the DC side (250 uH, 500 uF, 450-500 Hz,
    % damped by the battery resistance) limits the bandwidth: kp = 0.01, ki = 60 give 18 Hz, PM 90 deg, GM 14 dB
    % (averaged model, sizing Sec. 7). The feed-forward does the tracking, the integrator removes the static
    % error. Run 01: 208.4 A at 1 pu (208.3 A).
    dab3.ctrl_in.kp_idc = 0.01;
    dab3.ctrl_in.ki_idc = 60;
    dab3.ctrl_in.kp_udc = 0.5;                  % output voltage loop (Vout Ctrl): not active, the Variant Source of
    dab3.ctrl_in.ki_udc = 35;                   % the control selects the current reference of the bench

    % ------------------------------------------------------------------ bench: current reference (pu of idc1)
    % battery_1 on side 1, battery_2 on side 2; the current loop sets the DC input current of side 1:
    % 0 until t_on, then i_ref1, from t_rev i_ref2, through the rate limiter of the control (r_rise, r_fall).
    % i_ref2 lasts to the end of the run, where the logged window is (param.time.t_measure). Cases:
    %   'forward'   -1 pu (side 2 -> side 1), then the reversal to +1 pu (side 1 -> side 2): run 01
    %   'reverse'   +1 pu, then the reversal to -1 pu
    %   'light'     -1 pu, then +0.1 pu: switching currents set by the magnetizing current
    %   'mismatch'  battery_2 at soc 0.3 (1184 V open circuit, d = 0.95 at no load), then +0.25 pu: ZVS limit
    %               of bridge 2 (dab3_sizing.md, Sec. 5)
    dab3.bench.case = bench_case;
    dab3.bench.t_on = 0.01;                     % [s]
    dab3.bench.i_ref1 = -1;                     % [pu]
    dab3.bench.t_rev = 0.1;                     % [s]
    dab3.bench.i_ref2 = 1;                      % [pu]
    dab3.bench.r_rise = 20;                     % [pu/s] 50 ms for 1 pu, 100 ms for the reversal
    dab3.bench.r_fall = 20;                     % [pu/s]
    dab3.bench.soc_1 = 0.85;                    % initial state of charge of battery_1 and battery_2 (open circuit
    dab3.bench.soc_2 = 0.85;                    % voltage 1248 V at 0.85, 1184 V at 0.3)
    switch bench_case
        case 'forward'
        case 'reverse'
            dab3.bench.i_ref1 = 1;
            dab3.bench.i_ref2 = -1;
        case 'light'
            dab3.bench.i_ref2 = 0.1;
        case 'mismatch'
            dab3.bench.i_ref2 = 0.25;
            dab3.bench.soc_2 = 0.3;
        otherwise
            error('dab3_param: unknown bench case "%s" (forward, reverse, light, mismatch)', bench_case);
    end
end
