%[text] # afe\_inv\_psm: model initialization with the param structure
% Reorganized copy of init_model.m (2026-09-23). init_model.m is unchanged.
%
% Naming rule (prefix only, the leaf name never changes):
%   - loose variables move into a group:        kp_align       -> param.ctrl.kp_align
%   - library objects/structs keep their name:  glb_time.tc    -> param.glb_time.tc
%                                               psm_ctrl.kp_i  -> param.psm_ctrl.kp_i
% Groups:
%   param.sim     simulation settings              param.grid     grid emulator inputs
%   param.pwm     PWM, trigger, dead time          param.hw       power ratings
%   param.mode    drive control modes              param.meas     ADC end scale and quantization
%   param.pll     dqPLL selection                  param.dclink   DC-link stray inductance
%   param.impl    C-caller vs Simulink selection   param.behav    per-module non-idealities
%   param.frt     FRT test inputs                  param.ctrl     references, gains, filter settings
%   param.drive   machine scaling                  param.load     speed/torque profile, mechanical load
%   param.thermal heatsink inputs                  param.dev      selected semiconductor datasets
% Objects/structs: param.glb_time, param.grid_emu, param.hwdata, param.frt_data, param.psm,
%   param.afe_ctrl, param.psm_ctrl, param.filters, param.heatsink, param.igbt, param.mosfet,
%   param.ideal_switch.
% The last code section ("Transition: legacy workspace names") exports the old names still read
% by afe_inv_psm.slx. Delete each alias once the blocks use the param.* name.

clear;
[model, options] = init_environment('afe_inv_psm');
s = tf('s');
param = struct();
%[text] ## Simulation settings
param.sim.simlength = 1.75;                 % [s] model StopTime
param.sim.tc = 0.25e-6;                     % [s] fixed step (model FixedStep) and logging base
param.sim.load_step_time = 1.25;            % [s]
param.sim.transmission_delay = 125e-6*2;    % [s]
param.sim.sst_num_of_modules = 2;
param.sim.time_aux_power_supply_fault = 1e3;

% one/two modules
param.sim.number_of_modules = 1;
param.sim.enable_two_modules = param.sim.number_of_modules;

% devices with thermal model
param.sim.use_mosfet_thermal_model = 0;
param.sim.use_thermal_model = 1;

param.sim.nonlinear_iterations = 3;         % Simscape solver
if (param.sim.use_mosfet_thermal_model || param.sim.use_thermal_model)
    param.sim.nonlinear_iterations = 5;     % Simscape solver
end

% sections of the shared template not used by afe_inv_psm.slx (DAB, CLLC, PSFBC, ISOP rail,
% single-phase inverter, UPQC, IM, battery, load transformer): see add_template_extras at the end
param.sim.build_template_extras = false;
%[text] ## Global timing
param.pwm.fpwm = 4e3;

param.pwm.fpwm_afe = param.pwm.fpwm;
param.pwm.trgo_afe = 1;                     % double update
param.pwm.fpwm_inv = param.pwm.fpwm;
param.pwm.trgo_inv = 1;                     % double update
param.pwm.fpwm_single_phase_inv = param.pwm.fpwm;
param.pwm.trgo_single_phase_inv = 1;        % double update
param.pwm.fpwm_dab = 3 * param.pwm.fpwm;
param.pwm.trgo_dab = 1;                     % double update
param.pwm.fpwm_psfbc = param.pwm.fpwm;
param.pwm.trgo_psfbc = 1;                   % double update
param.pwm.fpwm_isop_rail = param.pwm.fpwm;
param.pwm.trgo_isop_rail = 1;               % double update
param.pwm.fpwm_cllc = 3 * param.pwm.fpwm;
param.pwm.trgo_cllc = 0;                    % single update

param.pwm.t_measure = param.sim.simlength;
param.pwm.tc_factor = 1/param.pwm.fpwm/param.sim.tc/2;
param.pwm.tc_decimation = 1;
param.pwm.delay_pwm = 0;

param.pwm.dead_time_afe = 3e-6;
param.pwm.dead_time_inv = 3e-6;
param.pwm.dead_time_single_phase_inv = 3e-6;
param.pwm.dead_time_dab = 2e-6;
param.pwm.dead_time_psfbc = 3e-6;
param.pwm.dead_time_isop_rail = 3e-6;
param.pwm.dead_time_cllc = 2e-6;

p = param.pwm;
param.glb_time = timing_setup(p.fpwm_afe, p.trgo_afe, p.fpwm_inv, p.trgo_inv, ...
    p.fpwm_single_phase_inv, p.trgo_single_phase_inv, p.fpwm_dab, p.trgo_dab, ...
    p.fpwm_psfbc, p.trgo_psfbc, p.fpwm_isop_rail, p.trgo_isop_rail, p.fpwm_cllc, p.trgo_cllc, ...
    p.t_measure, p.tc_factor, p.tc_decimation, p.delay_pwm, p.dead_time_afe, p.dead_time_inv, ...
    p.dead_time_single_phase_inv, p.dead_time_dab, p.dead_time_psfbc, p.dead_time_isop_rail, p.dead_time_cllc);
clear p
%[text] ## Control mode and implementation selection
%[text] Flags marked (variant) drive Variant Subsystem conditions in the model.
% drive control modes
param.mode.use_torque_curve = 1;                                    % (variant)
param.mode.use_speed_control = 1 - param.mode.use_torque_curve;     % (variant)
param.mode.use_mtpa = 1;                                            % (variant)
param.mode.use_psm_encoder = 0;                                     % (variant)
param.mode.use_im_encoder = 1;
param.mode.use_load_estimator = 0;
param.mode.use_estimator_from_mb = 0;                               % mb: model based
param.mode.use_motor_speed_control_mode = 0;
param.mode.motor_torque_mode = 1 - param.mode.use_motor_speed_control_mode;  % torque curve for wind application
param.mode.time_start_motor_control = 0.25;                         % [s]

% advanced dqPLL
param.pll.use_dq_pll_fht_pll = 1;
param.pll.use_dq_pll_fht_simulink_pll = 0;
param.pll.use_dq_pll_mod1 = 0;
param.pll.use_dq_pll_ccaller_mod1 = 0;
param.pll.use_dq_pll_ccaller_mod2 = 0;

% dqPLL
param.pll.use_dq_pll_mode1 = param.pll.use_dq_pll_mod1;             % (variant)
param.pll.use_dq_pll_mode2 = param.pll.use_dq_pll_ccaller_mod1;     % (variant)
param.pll.use_dq_pll_mode3 = param.pll.use_dq_pll_fht_simulink_pll; % (variant)
param.pll.use_dq_pll_mode4 = param.pll.use_dq_pll_fht_pll;          % (variant)

param.pll.use_dq_pll_mode1_modn = 0;        % simulink dqPLL
param.pll.use_dq_pll_mode2_modn = 0;        % ccaller dqPLL
param.pll.use_dq_pll_mode3_modn = 1;        % fht simulink dqPLL
param.pll.use_dq_pll_mode4_modn = 0;        % fht ccaller dqPLL

% observers
param.impl.use_ekf_bemf_module_1 = 1;                               % (variant)
param.impl.use_ekf_bemf_module_2 = 1;
param.impl.use_observer_from_simulink_module_1 = 0;                 % (variant)
param.impl.use_observer_from_ccaller_module_1 = 0;                  % (variant)
param.impl.use_observer_from_simulink_module_2 = 0;
param.impl.use_observer_from_ccaller_module_2 = 0;

% current controllers
param.impl.use_current_controller_from_simulink_module_1 = 0;       % (variant)
param.impl.use_current_controller_from_ccaller_module_1 = 1;        % (variant)
param.impl.use_current_controller_from_simulink_module_2 = 0;
param.impl.use_current_controller_from_ccaller_module_2 = 0;

% moving average filters
param.impl.use_moving_average_from_ccaller_mod1 = 1;                % (variant)
param.impl.use_moving_average_from_ccaller_mod2 = 0;
param.impl.use_moving_average_from_ccaller_mod3 = 0;
param.impl.use_moving_average_from_ccaller_mod4 = 0;
%[text] ## Grid emulator
param.grid.grid_nominal_power = 1000e3;
param.grid.application_voltage = 690;
param.grid.grid_nominal_current = param.grid.grid_nominal_power/param.grid.application_voltage/sqrt(3);

% transformer Dyn11
param.grid.delta_star = 1;

if param.grid.application_voltage == 690
    param.grid.us1 = 690; param.grid.us2 = 690; param.grid.fgrid = 50;
    param.grid.eta = 95; param.grid.ucc = 5;
    param.grid.p_iron = 1800;
elseif param.grid.application_voltage == 480
    param.grid.us1 = 480; param.grid.us2 = 480; param.grid.fgrid = 60;
    param.grid.eta = 95; param.grid.ucc = 5;
    param.grid.p_iron = 1400;
else
    param.grid.us1 = 400; param.grid.us2 = 400; param.grid.fgrid = 50;
    param.grid.eta = 95; param.grid.ucc = 5;
    param.grid.p_iron = 1000;
end

param.grid.n2 = 14;
param.grid.n1 = floor(param.grid.n2*sqrt(3));
param.grid.core_area = 0.05;
param.grid.core_length = 2.5;
param.grid.mu0 = 4*pi*1e-7;
param.grid.mur = 10e3;

% magnetizing inductance from the core geometry
param.grid.Lm1 = (param.grid.n1^2 * param.grid.mu0 * param.grid.mur * param.grid.core_area) / param.grid.core_length;
param.grid.i1m = param.grid.us1/sqrt(3)/param.grid.Lm1/(2*pi*param.grid.fgrid);

% reference for the voltage sequence
param.grid.up_xi_pu_ref = 1;
param.grid.up_eta_pu_ref = 0;
param.grid.un_xi_pu_ref = 0;
param.grid.un_eta_pu_ref = 0;

% grid impedance (ucc_factor can be preset only if the clear at the top is removed, e.g. in a sweep)
param.grid.Lgrid_base = param.grid.us1/sqrt(3)*param.grid.ucc/100/2/pi/param.grid.fgrid/param.grid.grid_nominal_current;
if exist('ucc_factor', 'var')
    param.grid.ucc_factor = ucc_factor;
else
    param.grid.ucc_factor = 1;
end
param.grid.eq_grid_inductance = param.grid.Lgrid_base*param.grid.ucc_factor;   % [H]
param.grid.eq_grid_resistance = 2e-3;                                            % [Ohm]

g = param.grid;
param.grid_emu = grid_three_phase_emulator('Dyn11', g.delta_star, g.grid_nominal_power, g.application_voltage, ...
    g.us1, g.us2, g.fgrid, g.eq_grid_inductance, g.eq_grid_resistance, g.eta, g.ucc, g.i1m, g.p_iron, ...
    g.n1, g.n2, g.core_area, g.core_length, g.mur, ...
    g.up_xi_pu_ref, g.up_eta_pu_ref, g.un_xi_pu_ref, g.un_eta_pu_ref);
clear g
%[text] ## Hardware
%[text] ### Power stages
param.hw.afe_pwr_nom = 250e3;
param.hw.inv_pwr_nom = 250e3;

param.hwdata.afe = three_phase_afe_hwdata(param.grid.application_voltage, param.hw.afe_pwr_nom, param.glb_time.fPWM_AFE);
param.hwdata.inv = three_phase_inverter_hwdata(param.grid.application_voltage, param.hw.inv_pwr_nom, param.glb_time.fPWM_INV);
%[text] ### Sensors end scale and quantization
param.meas.adc_quantization = 1/2^11;
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;

param.meas.Imax_adc = 1049.835;
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;

param.meas.Umax_adc = 1500;
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
%[text] ### DC-link stray inductance (partial loop inductance)
% same data as library script parasitic_dclink_data.m; not read by the model, Bode check only
param.dclink.Lstray_dclink = 100e-9;
param.dclink.RLstray_dclink = 10e-3;
param.dclink.C_HF_Lstray_dclink = 15e-6;
param.dclink.R_HF_Lstray_dclink = 22000;
param.dclink.Z_HF_Lstray_dclink = 1/s/param.dclink.C_HF_Lstray_dclink + param.dclink.R_HF_Lstray_dclink;
param.dclink.Z_LF_Lstray_dclink = s*param.dclink.Lstray_dclink + param.dclink.RLstray_dclink;
param.dclink.Z_Lstray_dclink = param.dclink.Z_HF_Lstray_dclink*param.dclink.Z_LF_Lstray_dclink/ ...
    (param.dclink.Z_HF_Lstray_dclink + param.dclink.Z_LF_Lstray_dclink);
param.dclink.ZCFi = 7/s/param.hwdata.afe.CFi;
param.dclink.sys_dclink = minreal(param.dclink.ZCFi/(param.dclink.ZCFi + param.dclink.Z_Lstray_dclink));
figure; bode(param.dclink.sys_dclink, param.dclink.Z_Lstray_dclink, options); grid on
%[text] ## Module behaviour (clock mismatch, noise, PWM phase shift)
param.behav.time_gain_afe_module_1 = 1.0002;
param.behav.time_gain_afe_module_2 = 1.0015;
param.behav.time_gain_afe_module_3 = 0.9988;
param.behav.time_gain_afe_module_4 = 1.0020;

param.behav.time_gain_inv_module_1 = 1.0005;
param.behav.time_gain_inv_module_2 = 1.001;

param.behav.wnp = 0;
param.behav.white_noise_power_afe_mod1 = param.behav.wnp;
param.behav.white_noise_power_inv_mod1 = param.behav.wnp;
param.behav.white_noise_power_afe_mod2 = param.behav.wnp;
param.behav.white_noise_power_inv_mod2 = param.behav.wnp;

param.behav.trgo_th_generator = 0.025;

param.behav.afe_pwm_phase_shift_mod1 = 0;
param.behav.white_noise_power_afe_pwm_phase_shift_mod1 = 0.0;
param.behav.inv_pwm_phase_shift_mod1 = 0;
param.behav.white_noise_power_inv_pwm_phase_shift_mod1 = 0.0;

param.behav.afe_pwm_phase_shift_mod2 = 0;
param.behav.white_noise_power_afe_pwm_phase_shift_mod2 = 0.0;
param.behav.inv_pwm_phase_shift_mod2 = 0;
param.behav.white_noise_power_inv_pwm_phase_shift_mod2 = 0.0;

param.behav.afe_pwm_phase_shift_mod3 = 0;
param.behav.white_noise_power_afe_pwm_phase_shift_mod3 = 0.0;
param.behav.inv_pwm_phase_shift_mod3 = 0;
param.behav.white_noise_power_inv_pwm_phase_shift_mod3 = 0.0;

param.behav.afe_pwm_phase_shift_mod4 = 0;
param.behav.white_noise_power_afe_pwm_phase_shift_mod4 = 0.0;
param.behav.inv_pwm_phase_shift_mod4 = 0;
param.behav.white_noise_power_inv_pwm_phase_shift_mod4 = 0.0;
%[text] ## FRT settings
param.frt.test_index = 25;                  % type of fault: index
param.frt.test_subindex = 4;                % type of fault: subindex
param.frt.enable_frt_1 = 0;                 % faults generated from abc
param.frt.enable_frt_2 = 1;                 % faults generated from xi_eta_pos and xi_eta_neg
param.frt.start_time_LVRT = 0.75;           % [s]
param.frt.asymmetric_error_type = 1;        % 0: variant C (two-phase), 1: variant D (single-phase)
param.frt.deepPOSxi = 1;
param.frt.deepPOSeta = -0.4;
param.frt.deepNEGxi = 0.4;
param.frt.deepNEGeta = 0.4;

f = param.frt;
param.frt_data = frt_settings(f.test_index, f.test_subindex, f.asymmetric_error_type, ...
    f.enable_frt_1, f.enable_frt_2, f.start_time_LVRT, f.deepPOSxi, f.deepPOSeta, f.deepNEGxi, f.deepNEGeta);

% grid_fault_generator is a function now: it returns only the fault waveform data
% (amplitudes, phase shift, error_length, end_time_LVRT, t_ramp, k_lvrt), merged into frt_data
frt_gen = grid_fault_generator(param.frt_data, f.start_time_LVRT);
for fn = fieldnames(frt_gen)'
    param.frt_data.(fn{1}) = frt_gen.(fn{1});
end
clear f frt_gen fn
%[text] ## Machine and load
%[text] ### PSM
param.psm = psm_calculus();
param.drive.number_of_systems = 1;
param.drive.n_sys = param.psm.number_of_systems;
%[text] ### Motor voltage to Udc scaling
param.drive.u_psm_scale = 2/3*param.hwdata.inv.udc_nom/param.psm.ubez;
param.drive.u_psm_scale_ekf = sqrt(3)/2 * 2/3 * param.hwdata.inv.udc_nom/param.psm.ubez;
%[text] ### Mechanical load, speed reference and torque curve
param.load.b = param.psm.load_friction_m;
param.load.external_load_inertia = 1;       % alternative: 6*param.psm.Jm_m

% energy production: torque curve (the library script reads n_sys, writes rpm_base, torque_base)
n_sys = param.drive.n_sys;
run('n_sys_generic_1M5W_torque_curve');
param.load.rpm_base = rpm_base;
param.load.torque_base = torque_base;
clear n_sys rpm_base torque_base
param.load.torque_overload_factor = 1.15;

% drive application: speed reference and load torque
param.load.rpm_sim = 17.8;                  % alternatives: 3000, 18, 15.2
param.load.omega_m_sim = param.psm.omega_m_bez;
param.load.omega_sim = param.load.omega_m_sim*param.psm.number_poles/2;
param.load.tau_load_sim = param.psm.tau_bez/5;  % [N*m]
param.load.b_square = 0;
%[text] ## Control
%[text] ### Module references and parallel operation
% four modules in parallel connected to a dc microgrid
param.ctrl.ixi_ref_mod1 = -0.85;
param.ctrl.ixi_ref_mod2 = -0.85;
param.ctrl.ixi_ref_mod3 = -0.85;
param.ctrl.ixi_ref_mod4 = -0.85;

% common mode voltage control for hard parallelization
param.ctrl.en_parallel_mode = 1;
param.ctrl.u_cm_comp_mod1 = 0;
param.ctrl.u_cm_comp_mod2 = 0;
param.ctrl.u_cm_comp_mod3 = 0;
param.ctrl.u_cm_comp_mod4 = 0;
if param.ctrl.en_parallel_mode
    param.ctrl.u_cm_comp_mod2 = -1;
    param.ctrl.u_cm_comp_mod3 = -1;
    param.ctrl.u_cm_comp_mod4 = -1;
end
%[text] ### Reactive current references
param.ctrl.enable_i_react_pos_steps = 1;
param.ctrl.time_i_react_pos_ref_1 = 0;      % default
param.ctrl.time_i_react_pos_ref_2 = 0;      % default
param.ctrl.i_react_pos_ref_1 = 0;           % default
param.ctrl.i_react_pos_ref_2 = 0;           % default
param.ctrl.i_react_pos_ref_3 = 0;           % default

if param.ctrl.enable_i_react_pos_steps
    param.ctrl.time_i_react_pos_ref_1 = param.frt_data.start_time_LVRT + param.frt_data.error_length + 0.335;
    param.ctrl.time_i_react_pos_ref_2 = param.ctrl.time_i_react_pos_ref_1 + 0.5;
    param.ctrl.i_react_pos_ref_2 = -param.ctrl.ixi_ref_mod1*tan(acos(0.95));   % cos(phi) = 0.95
    param.ctrl.i_react_pos_ref_3 =  param.ctrl.ixi_ref_mod1*tan(acos(0.95));   % cos(phi) = 0.95
end
%[text] ### AFE control (with sequences)
param.afe_ctrl = ctrl_afe_setup(param.glb_time.ts_afe, param.grid_emu.omega_grid_nom);
param.afe_ctrl.kp_udc_ctrl = 2;

% gains for weak grids
% param.afe_ctrl.res_pi.kp_rpi = 0.5;
% param.afe_ctrl.res_pi.ki_rpi = 18;

% gains for LVRT
param.afe_ctrl.res_pi.kp_rpi = 0.6;
param.afe_ctrl.res_pi.ki_rpi = 35;
%[text] ### PSM control with EKF based observer
param.psm_ctrl = ctrl_pmsm_setup(param.glb_time.ts_inv, param.psm.omega_bez, param.drive.u_psm_scale, param.psm.Jm_norm);
% param.psm_ctrl.ekf = ekf_pmsm_setup(param.psm.Rs_norm, param.psm.Ls_norm, param.psm.Jm_norm, param.glb_time.ts_inv);
param.psm_ctrl.ekf = ekf_pmsm_setup(param.psm.Rs_norm, param.psm.Ls_norm, 1e6, param.glb_time.ts_inv);
param.psm_ctrl.kp_i = 0.25;
param.psm_ctrl.ki_i = 35;
%[text] ### Local time alignment to master time
param.ctrl.kp_align = 0.25;
param.ctrl.ki_align = 18;
param.ctrl.lim_up_align = 0.05;
param.ctrl.lim_down_align = -0.05;
%[text] ### Filters and PI clip release
param.ctrl.CTRPIFF_CLIP_RELEASE = 0.001;
param.ctrl.mavarage_filter_frequency_base_order = 2;   % 2 means 100 Hz, 1 means 50 Hz
param.ctrl.dmavg_filter_enable_time = 0.025;           % [s]
param.filters = setup_global_filters(param.glb_time.ts_afe, param.glb_time.ts_inv, param.glb_time.ts_dab, param.glb_time.tc);
%[text] ## Power semiconductors and thermal model
%[text] ### Heatsink
% aluminium plate, liquid cooled, sized for PrimePACK2; liquid flow > 28 l/min
% "A" (ambient) is the water: HA is the temperature rise from water to heatsink surface;
% water in/out rise max 5 K with 2 kW total losses
param.thermal.weight = 0.150;                                   % [kg]
param.thermal.no_weight = 0.150/10;                             % [kg] /10: thermal inertia not accounted
param.thermal.cp_al = 900;                                      % [J/(kg K)] specific heat, aluminium
param.thermal.heat_capacity_hs = param.thermal.cp_al * param.thermal.weight;   % [J/K]
param.thermal.thermal_conductivity_al = 160;                    % [W/(m K)] aluminium
param.thermal.Rth_switch_HA = 15/1000;                          % [K/W]
param.thermal.Rth_mosfet_HA = param.thermal.Rth_switch_HA;      % [K/W]
param.thermal.Rth_diode_HA = param.thermal.Rth_switch_HA;       % [K/W]
param.thermal.Tambient = 40;                                    % [degC] water temperature
param.thermal.DThs_init = 0;                                    % [K]

t = param.thermal;
param.heatsink = liquid_cooled_plate_2kw_setup(t.weight, t.no_weight, t.cp_al, t.heat_capacity_hs, ...
    t.thermal_conductivity_al, t.Rth_switch_HA, t.Rth_mosfet_HA, t.Rth_diode_HA, t.Tambient, t.DThs_init);
clear t
%[text] ### Devices
param.dev.used_device_igbt = 'infineon_FF1200R17IP5';          % alternative: 'mitsubishi_CM1200DW_34T'
param.dev.used_device_mosfet = 'danfoss_SKM1700MB20R4S2I4';
param.dev.used_device_ideal_switch = 'silicon_high_power_ideal_switch';

% IGBT
param.igbt.afe = device_igbt_setup(param.dev.used_device_igbt, param.glb_time.fPWM_AFE, param.hwdata.afe.udc_nom);
param.igbt.inv = device_igbt_setup(param.dev.used_device_igbt, param.glb_time.fPWM_INV, param.hwdata.inv.udc_nom);

% MOSFET
param.mosfet.afe = device_mosfet_setup(param.dev.used_device_mosfet, param.glb_time.fPWM_AFE, param.hwdata.afe.udc_nom);
param.mosfet.inv = device_mosfet_setup(param.dev.used_device_mosfet, param.glb_time.fPWM_INV, param.hwdata.inv.udc_nom);

% ideal switch: the top-level fields (AFE data) are read by the model
param.ideal_switch = device_ideal_switch_setting(param.dev.used_device_ideal_switch, param.glb_time.fPWM_AFE, param.hwdata.afe.udc_nom);
param.ideal_switch.afe = device_ideal_switch_setting(param.dev.used_device_ideal_switch, param.glb_time.fPWM_AFE, param.hwdata.afe.udc_nom);
param.ideal_switch.inv = device_ideal_switch_setting(param.dev.used_device_ideal_switch, param.glb_time.fPWM_INV, param.hwdata.inv.udc_nom);
%[text] ## Template extras (not used by afe\_inv\_psm.slx)
if param.sim.build_template_extras
    param = add_template_extras(param, options);
end
%[text] ## Model setup
open_system(model);
%[text] ### C-Caller types
ctypes = {'mavgflt_output_t', 'dsmavgflt_output_t', 'mavgflts_output_t', 'bemf_obsv_output_t', ...
    'bemf_obsv_load_est_output_t', 'dqvector_pi_output_t', 'sv_pwm_output_t', 'sv_pwm_cm_output_t', ...
    'global_state_machine_output_t', 'first_harmonic_tracker_output_t', 'dqpll_thyr_output_t', ...
    'dqpll_grid_output_t', 'rpi_output_t', 'phase_shift_flt_output_t', 'sogi_flt_output_t', ...
    'linear_double_integrator_observer_output_t'};
for k = 1:numel(ctypes)
    Simulink.importExternalCTypes(model, 'Names', ctypes(k));
end
clear ctypes k
%[text] ### Scopes closed at start
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] ### Enable/disable subsystems
% power stage model: MOSFET thermal, IGBT thermal or ideal switch (the other two are commented out)
if param.sim.use_mosfet_thermal_model
    active_bridge = 'three_phase_inverter_mosfet_based_with_thermal_model';
elseif param.sim.use_thermal_model
    active_bridge = 'three_phase_inverter_igbt_based_with_thermal_model';
else
    active_bridge = 'three_phase_inverter_ideal_switch_based_model';
end
bridge_models = {'three_phase_inverter_mosfet_based_with_thermal_model', ...
                 'three_phase_inverter_igbt_based_with_thermal_model', ...
                 'three_phase_inverter_ideal_switch_based_model'};
bridge_paths = {[model '/afe_abc_inv_psm_mod1/afe/'], [model '/afe_abc_inv_psm_mod1/inverter/inverter/']};
commented = {'on', 'off'};                  % index 2 (off) for the active bridge
for ip = 1:numel(bridge_paths)
    for ib = 1:numel(bridge_models)
        set_param([bridge_paths{ip} bridge_models{ib}], 'Commented', ...
            commented{1 + strcmp(bridge_models{ib}, active_bridge)});
    end
end
clear active_bridge bridge_models bridge_paths commented ip ib

% load: torque curve (energy production) or fixed speed with load torque (drive)
if param.mode.use_torque_curve
    set_param([model '/fixed_speed_setting'], 'Commented', 'off');
    set_param([model '/motor_load_setting'], 'Commented', 'on');
else
    set_param([model '/fixed_speed_setting'], 'Commented', 'on');
    set_param([model '/motor_load_setting'], 'Commented', 'off');
end
%[text] ### CREATE\_POWER\_METER\_BUS
% Simulink.Bus for output "m" of the MATLAB Function block three_phase_power_meter_iec
% (Symbols pane: Type = Bus: PowerMeterIecBus). Bus objects stay in the base workspace.
names = {'Q1p', 'Q1n', 'P1n', 'P1p', 'U1n', 'U1p', ...
    'i_act_1n', 'i_react_1n', 'i_act_1p', 'i_react_1p'};

elems = Simulink.BusElement.empty(0,1);
for k = 1:numel(names)
    e            = Simulink.BusElement;
    e.Name       = names{k};
    e.Dimensions = 1;
    e.DataType   = 'double';
    e.Complexity = 'real';
    elems(k,1)   = e;
end

PowerMeterIecBus          = Simulink.Bus;
PowerMeterIecBus.Elements = elems;
assignin('base', 'PowerMeterIecBus', PowerMeterIecBus);
clear names elems e k
%[text] ## Transition: legacy workspace names
%[text] Old names still read by afe\_inv\_psm.slx (block parameters, variant conditions, model configuration), by the library blocks and by the post-processing scripts. Delete a line once the model uses the param.\* name; set export\_legacy\_names = false to check that nothing is left.
export_legacy_names = true;
if export_legacy_names
    % model configuration: StopTime = simlength, FixedStep = tc; tc also in plotting_results_*.m
    simlength = param.sim.simlength;
    tc = param.sim.tc;

    % read inside the library blocks (T = DThs_init+Tambient) and by sim_results_post_process.m
    Tambient = param.thermal.Tambient;
    DThs_init = param.thermal.DThs_init;

    % variant conditions
    use_torque_curve = param.mode.use_torque_curve;
    use_speed_control = param.mode.use_speed_control;
    use_mtpa = param.mode.use_mtpa;
    use_psm_encoder = param.mode.use_psm_encoder;
    use_dq_pll_mode1 = param.pll.use_dq_pll_mode1;
    use_dq_pll_mode2 = param.pll.use_dq_pll_mode2;
    use_dq_pll_mode3 = param.pll.use_dq_pll_mode3;
    use_dq_pll_mode4 = param.pll.use_dq_pll_mode4;
    use_ekf_bemf_module_1 = param.impl.use_ekf_bemf_module_1;
    use_observer_from_simulink_module_1 = param.impl.use_observer_from_simulink_module_1;
    use_observer_from_ccaller_module_1 = param.impl.use_observer_from_ccaller_module_1;
    use_current_controller_from_simulink_module_1 = param.impl.use_current_controller_from_simulink_module_1;
    use_current_controller_from_ccaller_module_1 = param.impl.use_current_controller_from_ccaller_module_1;
    use_moving_average_from_ccaller_mod1 = param.impl.use_moving_average_from_ccaller_mod1;

    % block parameters
    nonlinear_iterations = param.sim.nonlinear_iterations;
    load_step_time = param.sim.load_step_time;
    time_aux_power_supply_fault = param.sim.time_aux_power_supply_fault;
    time_start_motor_control = param.mode.time_start_motor_control;
    adc_quantization = param.meas.adc_quantization;
    time_gain_afe_module_1 = param.behav.time_gain_afe_module_1;
    time_gain_inv_module_2 = param.behav.time_gain_inv_module_2;
    afe_pwm_phase_shift_mod1 = param.behav.afe_pwm_phase_shift_mod1;
    inv_pwm_phase_shift_mod1 = param.behav.inv_pwm_phase_shift_mod1;
    ixi_ref_mod1 = param.ctrl.ixi_ref_mod1;
    u_cm_comp_mod1 = param.ctrl.u_cm_comp_mod1;
    time_i_react_pos_ref_1 = param.ctrl.time_i_react_pos_ref_1;
    time_i_react_pos_ref_2 = param.ctrl.time_i_react_pos_ref_2;
    i_react_pos_ref_1 = param.ctrl.i_react_pos_ref_1;
    i_react_pos_ref_2 = param.ctrl.i_react_pos_ref_2;
    i_react_pos_ref_3 = param.ctrl.i_react_pos_ref_3;
    kp_align = param.ctrl.kp_align;
    ki_align = param.ctrl.ki_align;
    lim_up_align = param.ctrl.lim_up_align;
    lim_down_align = param.ctrl.lim_down_align;
    CTRPIFF_CLIP_RELEASE = param.ctrl.CTRPIFF_CLIP_RELEASE;
    mavarage_filter_frequency_base_order = param.ctrl.mavarage_filter_frequency_base_order;
    dmavg_filter_enable_time = param.ctrl.dmavg_filter_enable_time;
    u_psm_scale_ekf = param.drive.u_psm_scale_ekf;
    external_load_inertia = param.load.external_load_inertia;
    rpm_base = param.load.rpm_base;
    torque_base = param.load.torque_base;
    torque_overload_factor = param.load.torque_overload_factor;
    rpm_sim = param.load.rpm_sim;
    omega_m_sim = param.load.omega_m_sim;

    % objects and structs
    glb_time = param.glb_time;
    grid_emu = param.grid_emu;
    hwdata = param.hwdata;
    frt_data = param.frt_data;
    psm = param.psm;
    afe_ctrl = param.afe_ctrl;
    psm_ctrl = param.psm_ctrl;
    filters = param.filters;
    heatsink = param.heatsink;
    igbt = param.igbt;
    ideal_switch = param.ideal_switch;
end
%[text] ## Local functions
function param = add_template_extras(param, options)
% Parts of the shared init template not used by afe_inv_psm.slx, kept so that nothing of
% init_model.m is lost. Enabled by param.sim.build_template_extras = true.

s = tf('s');

% ---------------------------------------------------------------- operating frequency
% global behaviour: system identification versus normal operation
param.spinv.system_identification_enable = 0;
param.spinv.frequency_set = 50;             % default
if param.spinv.system_identification_enable
    param.spinv.frequency_set = 300;
end
param.spinv.omega_set = param.spinv.frequency_set*2*pi;

% ---------------------------------------------------------------- single-phase inverter selection
param.spinv.rpi_enable = 0;                 % use RPI otherwise DQ PI
param.spinv.use_current_controller_from_ccaller_mod1 = 1;
param.spinv.use_phase_shift_filter_from_ccaller_mod1 = 1;
param.spinv.use_sogi_from_ccaller_mod1 = 1;

param.spinv.use_single_phase_inverter_based_FHT = 0;
param.spinv.use_single_phase_inverter_based_SOGI = 0;
param.spinv.use_single_phase_inverter_based_PHSH = 0;
param.spinv.use_single_phase_inverter_based_SOGI_ccaller = 1;
param.spinv.use_single_phase_inverter_based_PHSH_ccaller = 0;

param.spinv.use_system_identification_based_FHT = 0;
param.spinv.use_system_identification_based_SOGI = 0;
param.spinv.use_system_identification_based_PHSH = 0;
param.spinv.use_system_identification_based_SOGI_ccaller = 0;
param.spinv.use_system_identification_based_PHSH_ccaller = 1;

% single-phase inverter current references
param.spinv.iph_grid_pu_ref_1 = 1/3;
param.spinv.iph_grid_pu_ref_2 = 1/3;
param.spinv.iph_grid_pu_ref_3 = 1/3;
param.spinv.time_step_ref_1 = 0.025;
param.spinv.time_step_ref_2 = 0.5;
param.spinv.time_step_ref_3 = 1;

% ---------------------------------------------------------------- rail catenary and ISOP rail
param.catenary.catenary_nominal_power = 2*5.4e6;    % per 12 km
param.catenary.catenary_nominal_voltage = 3000;
param.catenary.catenary_maximum_voltage = 4000;
param.catenary.catenary_nominal_current = param.catenary.catenary_nominal_power/param.catenary.catenary_nominal_voltage;

param.catenary.isop_rail_pwr_nom = 250e3;           % nominal per system
param.catenary.number_of_systems = 4;
param.catenary.full_bridge_application_voltage = param.catenary.catenary_maximum_voltage/param.catenary.number_of_systems;
param.hwdata.isop_rail = isop_rail_aux_application(param.catenary.full_bridge_application_voltage, ...
    param.catenary.isop_rail_pwr_nom, param.glb_time.fPWM_ISOP_RAIL);

% ---------------------------------------------------------------- other power stages
param.hw.single_phase_inverter_pwr_nom = 225e3;
param.hw.dab_pwr_nom = 250e3;
param.hw.three_phase_dab_pwr_nom = 3*param.hw.dab_pwr_nom;
param.hw.cllc_pwr_nom = 250e3;
param.hw.psfbc_pwr_nom = 275e3;
param.hw.fres_dab = param.glb_time.fPWM_DAB/5;
param.hw.fres_cllc = param.glb_time.fPWM_CLLC*1.2;

uapp = param.grid.application_voltage;
param.hwdata.single_phase_inverter = single_phase_inverter_hwdata(uapp, param.hw.single_phase_inverter_pwr_nom, param.glb_time.fPWM_INV);
param.hwdata.dab = single_phase_dab_hwdata(uapp, param.hw.dab_pwr_nom, param.glb_time.fPWM_DAB, param.hw.fres_dab);
param.hwdata.psfbc = single_phase_psfbc_hwdata(uapp, param.hw.psfbc_pwr_nom, param.glb_time.fPWM_PSFBC);
param.hwdata.three_phase_dab = three_phase_dab_hwdata(uapp, param.hw.three_phase_dab_pwr_nom, param.glb_time.fPWM_DAB, param.hw.fres_dab);
param.hwdata.cllc = single_phase_cllc_hwdata(uapp, param.hw.cllc_pwr_nom, param.glb_time.fPWM_CLLC, param.hw.fres_cllc);  % init_model.m passes dab_pwr_nom (same value)
% modifications
% param.hwdata.afe.CFi = 2 * param.hwdata.psfbc.Cdc_dc1;

% ---------------------------------------------------------------- UPQC series transformer
param.upqc.name = 'UPQC Series Transformer';
param.upqc.pwr_nom = 3*125e3;
param.upqc.u1_nom = 400;
param.upqc.u2_nom = 690;
param.upqc.f_nom = 50;
param.upqc.eta = 98;
param.upqc.ucc = 4;
param.upqc.p_iron = 5e3;
param.upqc.n12 = param.upqc.u1_nom/param.upqc.u2_nom;
param.upqc.n2 = 8;
param.upqc.n1 = param.upqc.n12*param.upqc.n2;
param.upqc.core_area = 0.04;
param.upqc.core_length = 0.25;
param.upqc.mur = 35e3;
param.upqc.Lm1 = (param.upqc.n1^2 * param.grid.mu0 * param.upqc.mur * param.upqc.core_area) / param.upqc.core_length;
param.upqc.i1m = param.upqc.u1_nom/sqrt(3)/param.upqc.Lm1/param.upqc.f_nom/2/pi;
param.upqc.delta_star = 0;

u = param.upqc;
param.upqc_st = three_phase_transformer_setup(u.name, u.delta_star, u.pwr_nom, u.u1_nom, u.u2_nom, u.f_nom, ...
    u.eta, u.ucc, u.i1m, u.p_iron, u.n1, u.n2, u.core_area, u.core_length, u.mur);
param.upqc_ctrl.kp_p = 1;
param.upqc_ctrl.ki_p = 35;
param.upqc_ctrl.kp_n = 1;
param.upqc_ctrl.ki_n = 35;
param.upqc_ctrl.lim = 4;

% ---------------------------------------------------------------- induction machine
param.im = im_calculus();
param.drive.u_im_scale = 2/3*param.hwdata.inv.udc_nom/param.im.ubez;
param.drive.u_im_scale_ekf = (2/3)^2 * param.hwdata.inv.udc_nom/param.im.ubez;
param.im_ctrl = ctrl_im_setup(param.glb_time.ts_inv, param.im.omega_bez, param.drive.u_im_scale, param.im.Jm_norm);
param.im_ctrl.ekf = ekf_im_setup(param.im.alpha_norm, param.im.beta_norm, param.im.gamma_norm, param.im.sigma_norm, ...
    param.im.mu_norm, param.im.Lm_norm, param.im.Jm_norm, param.glb_time.ts_inv);

% ---------------------------------------------------------------- DC/DC control
param.dcdc.kp_udc = 0.5;
param.dcdc.ki_udc = 18.0;
param.dcdc.kp_idc = 0.5;
param.dcdc.ki_idc = 18.0;
d = param.dcdc;
param.dab_ctrl = ctrl_dab_setup(d.kp_udc, d.ki_udc, d.kp_idc, d.ki_idc);
param.psfbc_ctrl = ctrl_dab_setup(d.kp_udc, d.ki_udc, d.kp_idc, d.ki_idc);
param.cllc_ctrl = ctrl_cllc_setup(d.kp_udc, d.ki_udc, d.kp_idc, d.ki_idc);
param.isop_rail_ctrl = ctrl_isop_rail_setup(d.kp_udc, d.ki_udc, d.kp_idc, d.ki_idc);
% special settings
param.psfbc_ctrl.kp_idc = 1;
param.dab_ctrl.ki_udc = 35;
param.dab_ctrl.kp_idc = 0.02;
param.dab_ctrl.ki_idc = 1.0;

% ---------------------------------------------------------------- resonant PI
ts = param.glb_time.ts_inv;
param.pres_ctrl.kp_rpi = 0.75;
param.pres_ctrl.ki_rpi = 45;
param.pres_ctrl.delta_rpi = 0.025;
param.pres_ctrl.omega_set = param.spinv.omega_set;
w = param.pres_ctrl.omega_set;
dr = param.pres_ctrl.delta_rpi;
param.pres_ctrl.res_nom = s/(s^2 + 2*dr*w*s + w^2);
param.pres_ctrl.Ares_nom = [0 1; -w^2 -2*dr*w];
param.pres_ctrl.Aresd_nom = eye(2) + param.pres_ctrl.Ares_nom*ts;
param.pres_ctrl.a11d = 1;
param.pres_ctrl.a12d = ts;
param.pres_ctrl.a21d = -w^2*ts;
param.pres_ctrl.a22d = 1 - 2*dr*w*ts;       % init_model.m writes it to apres_ctrl (typo)
param.pres_ctrl.Bres = [0; 1];
param.pres_ctrl.Cres = [0 1];
param.pres_ctrl.Bresd = param.pres_ctrl.Bres*ts;
param.pres_ctrl.Cresd = param.pres_ctrl.Cres;

% ---------------------------------------------------------------- SOGI
param.spinv.sogi_delta = 1;
param.spinv.kepsilon = 2;
param.sogi = sogi_filter(param.spinv.omega_set, param.spinv.sogi_delta, param.spinv.kepsilon, param.glb_time.ts_afe);

% ---------------------------------------------------------------- DQ PI current control
param.dqvector_pi.kp_inv = 0.5;
param.dqvector_pi.ki_inv = 45;
param.dqvector_pi.pi_ctrl = param.dqvector_pi.kp_inv + param.dqvector_pi.ki_inv/s;
param.dqvector_pi.pid_ctrl = c2d(param.dqvector_pi.pi_ctrl, ts);
param.dqvector_pi.plant = 1/(s*param.grid_emu.trafo.Ld1 + 1);
param.dqvector_pi.plantd = c2d(param.dqvector_pi.plant, ts);
param.dqvector_pi.G = param.sogi.fltd.alpha * param.dqvector_pi.pid_ctrl * param.dqvector_pi.plantd;
figure; margin(param.dqvector_pi.G, options); grid on

% ---------------------------------------------------------------- single-phase inverter control
param.single_phase_inverter_ctrl = ctrl_single_phase_inverter_setup(ts, param.pres_ctrl.omega_set, ...
    param.dqvector_pi.kp_inv, param.dqvector_pi.ki_inv, param.pres_ctrl.kp_rpi, param.pres_ctrl.ki_rpi, param.pres_ctrl.delta_rpi);

% ---------------------------------------------------------------- devices of the other stages
dev = param.dev;
param.igbt.dab = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_DAB, param.hwdata.dab.udc1_nom);
param.igbt.psfbc = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_PSFBC, param.hwdata.psfbc.udc1_nom);
param.igbt.cllc = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_CLLC, param.hwdata.cllc.udc1_nom);
param.igbt.isop_rail = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_ISOP_RAIL, param.hwdata.isop_rail.udc1_nom);

param.mosfet.dab = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_DAB, param.hwdata.dab.udc1_nom);
param.mosfet.psfbc = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_PSFBC, param.hwdata.psfbc.udc1_nom);
param.mosfet.cllc = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_CLLC, param.hwdata.cllc.udc1_nom);
param.mosfet.isop_rail = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_ISOP_RAIL, param.hwdata.isop_rail.udc1_nom);

param.ideal_switch.dab = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_DAB, param.hwdata.dab.udc1_nom);
param.ideal_switch.psfbc = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_PSFBC, param.hwdata.psfbc.udc1_nom);
param.ideal_switch.cllc = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_CLLC, param.hwdata.cllc.udc1_nom);
param.ideal_switch.isop_rail = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_ISOP_RAIL, param.hwdata.isop_rail.udc1_nom);

% ---------------------------------------------------------------- diode rectifier
% ABB 5SDF 0131Z0401 (slow recovery)
param.diode.rectifier.Vf = 0.977;
param.diode.rectifier.Rdon = 22e-6;
param.diode.rectifier.Cj = 0;               % [F]
param.diode.rectifier.Irm = -75;            % [A]
param.diode.rectifier.didt = -30;           % [A/us]
param.diode.rectifier.trr = 5;              % [us]
param.diode.rectifier.Qrr = 325e-6;         % [C]
param.diode.rectifier.Ifm = 2000;           % [A]
param.diode.rectifier.Vr = -50;             % [V]
param.diode.rectifier.Err = 15e-3;          % [J]
param.diode.rectifier.Rth_JC = 0.004;       % [K/W]
param.diode.rectifier.Rth_CH = 0.003;       % [K/W]
param.diode.rectifier.Rsnubber = 1e4;       % [Ohm]
param.diode.rectifier.Csnubber = 1e-12;     % [F]

% ---------------------------------------------------------------- lithium-ion battery
param.battery.nominal_battery_voltage_1 = param.hwdata.afe.udc_nom;   % alternatives: hwdata.cllc.udc1_bez, hwdata.dab.udc1_bez
param.battery.nominal_battery_voltage_2 = param.hwdata.afe.udc_nom;   % alternatives: hwdata.cllc.udc2_bez, hwdata.dab.udc2_bez
param.battery.nominal_battery_power = 250e3;
% alternative: both voltages catenary_nominal_voltage*1.5, power catenary_nominal_power
param.battery.initial_battery_soc = 0.85;
% lithium_ion_battery_setup now takes the charge capacity [Ah] as third argument.
% ASSUMPTION (not in init_model.m): 1C rating, capacity = rated current x 1 h
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;
param.battery.charge_capacity_2 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_2;

b = param.battery;
param.lithium_ion_battery_1 = lithium_ion_battery_setup(b.nominal_battery_voltage_1, b.nominal_battery_power, ...
    b.charge_capacity_1, b.initial_battery_soc, param.glb_time.ts_dab);
param.lithium_ion_battery_2 = lithium_ion_battery_setup(b.nominal_battery_voltage_2, b.nominal_battery_power, ...
    b.charge_capacity_2, b.initial_battery_soc, param.glb_time.ts_dab);

% special settings for simulation
param.lithium_ion_battery_1.R0 = param.lithium_ion_battery_1.R0/2;
param.lithium_ion_battery_1.R1 = param.lithium_ion_battery_1.R1/2;
param.lithium_ion_battery_2.R0 = param.lithium_ion_battery_2.R0/2;
param.lithium_ion_battery_2.R1 = param.lithium_ion_battery_2.R1/2;
param.lithium_ion_battery_1.C1 = param.lithium_ion_battery_1.C1/200;
param.lithium_ion_battery_2.C1 = param.lithium_ion_battery_2.C1/200;

% ---------------------------------------------------------------- load transformer and RL load
param.trafo_load.trafo_load_name = 'Load Single Phase Transformer';
param.trafo_load.trafo_load_pwr_nom = 225e3;
param.trafo_load.trafo_load_u1_nom = 400;
param.trafo_load.trafo_load_n1 = 50;
param.trafo_load.trafo_load_n2 = 1;
param.trafo_load.trafo_load_u2_nom = param.trafo_load.trafo_load_u1_nom/param.trafo_load.trafo_load_n1*param.trafo_load.trafo_load_n2;
param.trafo_load.trafo_load_f_nom = param.spinv.frequency_set;     % alternative: 50
param.trafo_load.trafo_load_eta = 98;
param.trafo_load.trafo_load_ucc = 5;
param.trafo_load.trafo_load_i1m = 10;
param.trafo_load.trafo_load_p_iron = 2e3;

tl = param.trafo_load;
param.output_transformer = single_phase_transformer_setup(tl.trafo_load_name, tl.trafo_load_pwr_nom, tl.trafo_load_u1_nom, ...
    tl.trafo_load_u2_nom, tl.trafo_load_n1, tl.trafo_load_n2, tl.trafo_load_f_nom, tl.trafo_load_eta, tl.trafo_load_ucc, ...
    tl.trafo_load_i1m, tl.trafo_load_p_iron);

param.trafo_load.uload = 2;
param.trafo_load.rload = param.trafo_load.uload / param.output_transformer.i2_nom;
param.trafo_load.lload = 250e-6 / param.output_transformer.n12^2;
% alternative: rload = 0.86/m12_load_trafo^2; lload = 3e-3/m12_load_trafo^2;
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":33.8}
%---
