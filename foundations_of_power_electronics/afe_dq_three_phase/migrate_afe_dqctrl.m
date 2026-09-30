function report = migrate_afe_dqctrl(varargin)
%MIGRATE_AFE_DQCTRL  Rewrite afe_dqctrl.slx from the legacy workspace names to param.*
%
%   report = migrate_afe_dqctrl()                 dry run: nothing changed, report of the changes
%   report = migrate_afe_dqctrl('DryRun', false)  applies and saves afe_dqctrl_param.slx
%
%   Thin wrapper: the mapping legacy name -> param.* of init_model.m (Rev. 01) is here, the work is
%   done by tools/migrate_model_to_param (same options: 'DryRun', 'SaveAs', 'Verbose', ...).
%
%   Notes on the mapping (names found in the model XML, 2026-09-30):
%   - the AFE current controller (.../current_ctrl/dq_vector_pi) reads psm_ctrl.kp_i and psm_ctrl.ki_i:
%     mapped to param.afe.dq_vector_pi.kp_i / ki_i;
%   - the PLL Variant Source blocks read use_dq_pll_mode1..4 (param.afe.pll.*);
%   - the clocks of "global timing" read time_gain_afe_module_1 and time_gain_inv_module_2, the IGBT
%     bridge mask reads Tambient, the Solver Configuration reads nonlinear_iterations;
%   - the configuration set has StopTime = simlength, FixedStep = tc (ignored, ode23tb) and
%     MaxStep = glb_time.tc.
%
%   After the migration run init_model.m with export_legacy_names = false and update the model
%   (Ctrl+D); list_model_workspace_names(model, 'Names', {...legacy roots...}) shows what is left.

map = {
    % objects and structs
    'hwdata.afe',                                   'param.afe.hw'
    'igbt.afe',                                     'param.afe.igbt'
    'mosfet.afe',                                   'param.afe.mosfet'
    'ideal_switch.afe',                             'param.afe.ideal_switch'
    'ideal_switch',                                 'param.afe.ideal_switch'
    'glb_time',                                     'param.glb_time'
    'grid_emu',                                     'param.grid.emu'
    'frt_data',                                     'param.frt.data'
    'afe_ctrl',                                     'param.afe.ctrl'
    'psm_ctrl.kp_i',                                'param.afe.dq_vector_pi.kp_i'   % AFE current loop gains
    'psm_ctrl.ki_i',                                'param.afe.dq_vector_pi.ki_i'
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
    % AFE: PLL variants, implementation, references, behaviour
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
    % second clock domain (no inverter in this model)
    'time_gain_inv_module_1',                       'param.inv.behav.time_gain_module_1'
    'time_gain_inv_module_2',                       'param.inv.behav.time_gain_module_2'
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
    };

add_repo_tools_to_path();
report = migrate_model_to_param('afe_dqctrl', map, varargin{:});
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
        error('migrate_afe_dqctrl:tools', 'Folder "tools" of the repository not found: add it to the MATLAB path.');
    end
end
