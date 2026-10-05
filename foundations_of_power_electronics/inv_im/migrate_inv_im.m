function report = migrate_inv_im(varargin)
%MIGRATE_INV_IM  Rewrite inv_im.slx from the legacy workspace names to param.*
%
%   report = migrate_inv_im()                 dry run: nothing changed, report of the changes
%   report = migrate_inv_im('DryRun', false)  applies and saves inv_im_param.slx
%
%   Thin wrapper: the mapping legacy name -> param.* of init_model.m (Rev. 01) is here, the work is
%   done by tools/migrate_model_to_param (same options: 'DryRun', 'SaveAs', 'Verbose', ...).
%   inv_im_param.slx of 2026-10-05 was produced with this mapping and the same rules applied to the
%   model XML (325 parameters rewritten, new ModelUUID): running this wrapper gives the same result.
%
%   Three references of inv_im.slx are corrected, not only renamed:
%     psm_ctrl.ekf.C_ekf        -> param.im.ctrl.ekf.C_ekf    the EKF of the induction machine read the
%                                                              PSM one: same constant of ekf_im_setup
%     weigth                    -> param.thermal.weight       typo of the thermal bridges, never defined
%     mosfet.inv.Rth_switch_JH  -> param.inv.mosfet.Rth_mosfet_JH   property name of device_mosfet_setting
%
%   Not mapped: use_im_ekf and time_use_im_ekf, read only by the commented Step block of
%   .../im_control/encoder-sensorless-switch (never defined by any init). Configuration: StopTime and
%   MaxStep are rewritten; FixedStep (tc) is left as is, it is not used by the variable-step solver
%   ode23tb (same as afe_inv_psm_param.slx).

map = {
    % objects and structs
    'hwdata.afe',                                   'param.afe.hw'
    'hwdata.inv',                                   'param.inv.hw'
    'igbt.inv',                                     'param.inv.igbt'
    'mosfet.inv.Rth_switch_JH',                     'param.inv.mosfet.Rth_mosfet_JH'
    'mosfet.inv',                                   'param.inv.mosfet'
    'ideal_switch.inv',                             'param.inv.ideal_switch'
    'glb_time',                                     'param.glb_time'
    'im_ctrl',                                      'param.im.ctrl'
    'im',                                           'param.im.machine'
    'psm_ctrl.ekf.C_ekf',                           'param.im.ctrl.ekf.C_ekf'
    'psm',                                          'param.psm.machine'
    'filters',                                      'param.ctrl.filters'
    'lithium_ion_battery_1',                        'param.battery.pack_1'
    % simulation and configuration
    'simlength',                                    'param.sim.simlength'
    'tc',                                           'param.sim.tc'
    'nonlinear_iterations',                         'param.sim.nonlinear_iterations'
    'load_step_time',                               'param.sim.load_step_time'
    % thermal (instance parameters of the thermal bridges)
    'Tambient',                                     'param.thermal.Tambient'
    'DThs_init',                                    'param.thermal.DThs_init'
    'weigth',                                       'param.thermal.weight'
    'cp_al',                                        'param.thermal.cp_al'
    'Rth_switch_HA',                                'param.thermal.Rth_switch_HA'
    'Rth_diode_HA',                                 'param.thermal.Rth_diode_HA'
    % machine control modes and implementation (variant conditions)
    'use_torque_curve',                             'param.im.mode.use_torque_curve'
    'use_speed_control',                            'param.im.mode.use_speed_control'
    'time_start_motor_control',                     'param.im.mode.time_start_motor_control'
    'use_current_controller_from_simulink_module_1','param.im.impl.use_current_controller_from_simulink_module_1'
    'use_current_controller_from_ccaller_module_1', 'param.im.impl.use_current_controller_from_ccaller_module_1'
    'u_im_scale_ekf',                               'param.im.u_im_scale_ekf'
    % clocks of "global timing" and inverter behaviour
    'time_gain_afe_module_1',                       'param.afe.behav.time_gain_module_1'
    'time_gain_inv_module_2',                       'param.inv.behav.time_gain_module_2'
    'inv_pwm_phase_shift_mod1',                     'param.inv.behav.pwm_phase_shift_mod1'
    % measurement
    'adc_quantization',                             'param.meas.adc_quantization'
    'adc12_quantization',                           'param.meas.adc12_quantization'
    % common control settings
    'CTRPIFF_CLIP_RELEASE',                         'param.ctrl.CTRPIFF_CLIP_RELEASE'
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
    };

add_repo_tools_to_path();
report = migrate_model_to_param('inv_im', map, varargin{:});
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
        error('migrate_inv_im:tools', 'Folder "tools" of the repository not found: add it to the MATLAB path.');
    end
end
