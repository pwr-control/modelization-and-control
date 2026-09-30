function report = migrate_afe_inv_psm(varargin)
%MIGRATE_AFE_INV_PSM  Rewrite afe_inv_psm.slx from the legacy workspace names to param.*
%
%   report = migrate_afe_inv_psm()                 dry run: nothing changed, report of the changes
%   report = migrate_afe_inv_psm('DryRun', false)  applies and saves afe_inv_psm_param.slx
%
%   Thin wrapper: the mapping legacy name -> param.* of init_model.m (Rev. 01) is here, the work is
%   done by tools/migrate_model_to_param (same options: 'DryRun', 'SaveAs', 'Verbose', ...).
%   The migration was first applied on 2026-09-30 with the standalone version of this script
%   (763 parameters rewritten, afe_inv_psm_param.slx): this wrapper gives the same result.
%
%   ideal_switch.* is mapped by position: blocks under an ".../inverter/..." path take
%   param.inv.ideal_switch, all the others param.afe.ideal_switch ('PathRules').

map = {
    % objects and structs
    'hwdata.afe',                                   'param.afe.hw'
    'hwdata.inv',                                   'param.inv.hw'
    'igbt.afe',                                     'param.afe.igbt'
    'igbt.inv',                                     'param.inv.igbt'
    'mosfet.afe',                                   'param.afe.mosfet'
    'mosfet.inv',                                   'param.inv.mosfet'
    'glb_time',                                     'param.glb_time'
    'grid_emu',                                     'param.grid.emu'
    'frt_data',                                     'param.frt.data'
    'afe_ctrl',                                     'param.afe.ctrl'
    'psm_ctrl',                                     'param.psm.ctrl'
    'psm',                                          'param.psm.machine'
    'filters',                                      'param.ctrl.filters'
    'heatsink',                                     'param.thermal.heatsink'
    % simulation and configuration
    'simlength',                                    'param.sim.simlength'
    'tc',                                           'param.sim.tc'
    'nonlinear_iterations',                         'param.sim.nonlinear_iterations'
    'load_step_time',                               'param.sim.load_step_time'
    'time_aux_power_supply_fault',                  'param.sim.time_aux_power_supply_fault'
    % thermal
    'Tambient',                                     'param.thermal.Tambient'
    'DThs_init',                                    'param.thermal.DThs_init'
    % machine control modes and implementation (variant conditions)
    'use_torque_curve',                             'param.psm.mode.use_torque_curve'
    'use_speed_control',                            'param.psm.mode.use_speed_control'
    'use_mtpa',                                     'param.psm.mode.use_mtpa'
    'use_psm_encoder',                              'param.psm.mode.use_psm_encoder'
    'time_start_motor_control',                     'param.psm.mode.time_start_motor_control'
    'use_ekf_bemf_module_1',                        'param.psm.impl.use_ekf_bemf_module_1'
    'use_ekf_bemf_module_2',                        'param.psm.impl.use_ekf_bemf_module_2'
    'use_observer_from_simulink_module_1',          'param.psm.impl.use_observer_from_simulink_module_1'
    'use_observer_from_ccaller_module_1',           'param.psm.impl.use_observer_from_ccaller_module_1'
    'use_observer_from_simulink_module_2',          'param.psm.impl.use_observer_from_simulink_module_2'
    'use_observer_from_ccaller_module_2',           'param.psm.impl.use_observer_from_ccaller_module_2'
    'use_current_controller_from_simulink_module_1','param.psm.impl.use_current_controller_from_simulink_module_1'
    'use_current_controller_from_ccaller_module_1', 'param.psm.impl.use_current_controller_from_ccaller_module_1'
    'use_current_controller_from_simulink_module_2','param.psm.impl.use_current_controller_from_simulink_module_2'
    'use_current_controller_from_ccaller_module_2', 'param.psm.impl.use_current_controller_from_ccaller_module_2'
    'u_psm_scale_ekf',                              'param.psm.u_psm_scale_ekf'
    'u_psm_scale',                                  'param.psm.u_psm_scale'
    % AFE: PLL, implementation, references, behaviour
    'use_dq_pll_mode1',                             'param.afe.pll.use_dq_pll_mode1'
    'use_dq_pll_mode2',                             'param.afe.pll.use_dq_pll_mode2'
    'use_dq_pll_mode3',                             'param.afe.pll.use_dq_pll_mode3'
    'use_dq_pll_mode4',                             'param.afe.pll.use_dq_pll_mode4'
    'use_moving_average_from_ccaller_mod1',         'param.afe.impl.use_moving_average_from_ccaller_mod1'
    'use_moving_average_from_ccaller_mod2',         'param.afe.impl.use_moving_average_from_ccaller_mod2'
    'use_moving_average_from_ccaller_mod3',         'param.afe.impl.use_moving_average_from_ccaller_mod3'
    'use_moving_average_from_ccaller_mod4',         'param.afe.impl.use_moving_average_from_ccaller_mod4'
    'ixi_ref_mod1',                                 'param.afe.ref.ixi_ref_mod1'
    'ixi_ref_mod2',                                 'param.afe.ref.ixi_ref_mod2'
    'ixi_ref_mod3',                                 'param.afe.ref.ixi_ref_mod3'
    'ixi_ref_mod4',                                 'param.afe.ref.ixi_ref_mod4'
    'u_cm_comp_mod1',                               'param.afe.ref.u_cm_comp_mod1'
    'u_cm_comp_mod2',                               'param.afe.ref.u_cm_comp_mod2'
    'u_cm_comp_mod3',                               'param.afe.ref.u_cm_comp_mod3'
    'u_cm_comp_mod4',                               'param.afe.ref.u_cm_comp_mod4'
    'time_i_react_pos_ref_1',                       'param.afe.ref.time_i_react_pos_ref_1'
    'time_i_react_pos_ref_2',                       'param.afe.ref.time_i_react_pos_ref_2'
    'i_react_pos_ref_1',                            'param.afe.ref.i_react_pos_ref_1'
    'i_react_pos_ref_2',                            'param.afe.ref.i_react_pos_ref_2'
    'i_react_pos_ref_3',                            'param.afe.ref.i_react_pos_ref_3'
    'afe_pwm_phase_shift_mod1',                     'param.afe.behav.pwm_phase_shift_mod1'
    'afe_pwm_phase_shift_mod2',                     'param.afe.behav.pwm_phase_shift_mod2'
    'time_gain_afe_module_1',                       'param.afe.behav.time_gain_module_1'
    'time_gain_afe_module_2',                       'param.afe.behav.time_gain_module_2'
    'white_noise_power_afe_mod1',                   'param.afe.behav.white_noise_power_mod1'
    'white_noise_power_afe_mod2',                   'param.afe.behav.white_noise_power_mod2'
    % inverter behaviour
    'inv_pwm_phase_shift_mod1',                     'param.inv.behav.pwm_phase_shift_mod1'
    'inv_pwm_phase_shift_mod2',                     'param.inv.behav.pwm_phase_shift_mod2'
    'time_gain_inv_module_1',                       'param.inv.behav.time_gain_module_1'
    'time_gain_inv_module_2',                       'param.inv.behav.time_gain_module_2'
    'white_noise_power_inv_mod1',                   'param.inv.behav.white_noise_power_mod1'
    'white_noise_power_inv_mod2',                   'param.inv.behav.white_noise_power_mod2'
    % measurement
    'adc_quantization',                             'param.meas.adc_quantization'
    'adc12_quantization',                           'param.meas.adc12_quantization'
    'adc16_quantization',                           'param.meas.adc16_quantization'
    'CurrentQuantization',                          'param.meas.CurrentQuantization'
    'VoltageQuantization',                          'param.meas.VoltageQuantization'
    % common control settings
    'CTRPIFF_CLIP_RELEASE',                         'param.ctrl.CTRPIFF_CLIP_RELEASE'
    'mavarage_filter_frequency_base_order',         'param.ctrl.mavarage_filter_frequency_base_order'
    'dmavg_filter_enable_time',                     'param.ctrl.dmavg_filter_enable_time'
    'kp_align',                                     'param.ctrl.kp_align'
    'ki_align',                                     'param.ctrl.ki_align'
    'lim_up_align',                                 'param.ctrl.lim_up_align'
    'lim_down_align',                               'param.ctrl.lim_down_align'
    % load
    'external_load_inertia',                        'param.load.external_load_inertia'
    'rpm_base',                                     'param.load.rpm_base'
    'torque_base',                                  'param.load.torque_base'
    'torque_overload_factor',                       'param.load.torque_overload_factor'
    'rpm_sim',                                      'param.load.rpm_sim'
    'omega_m_sim',                                  'param.load.omega_m_sim'
    'omega_sim',                                    'param.load.omega_sim'
    'tau_load_sim',                                 'param.load.tau_load_sim'
    'b_square',                                     'param.load.b_square'
    };

% position-dependent rule: the first row whose pattern is in the block path wins, '' is the default
path_rules = {
    '/inverter/',   'ideal_switch',     'param.inv.ideal_switch'
    '',             'ideal_switch',     'param.afe.ideal_switch'
    };

add_repo_tools_to_path();
report = migrate_model_to_param('afe_inv_psm', map, 'PathRules', path_rules, varargin{:});
end

function add_repo_tools_to_path()
% The generic tools live in <repo>/tools: found by walking up from this file when not on the path.
    if exist('migrate_model_to_param', 'file') == 2
        return
    end
    here = fileparts(mfilename('fullpath'));
    while ~isfolder(fullfile(here, 'tools')) && ~strcmp(here, fileparts(here))
        here = fileparts(here);
    end
    if isfolder(fullfile(here, 'tools'))
        addpath(fullfile(here, 'tools'));
    else
        error('migrate_afe_inv_psm:tools', 'Folder "tools" of the repository not found: add it to the MATLAB path.');
    end
end
