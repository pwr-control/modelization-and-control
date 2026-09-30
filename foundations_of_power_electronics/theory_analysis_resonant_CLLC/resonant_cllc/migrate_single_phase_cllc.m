function report = migrate_single_phase_cllc(varargin)
%MIGRATE_SINGLE_PHASE_CLLC  Rewrite single_phase_cllc.slx from the legacy workspace names to param.*
%
%   report = migrate_single_phase_cllc()                 dry run: nothing changed, report of the changes
%   report = migrate_single_phase_cllc('DryRun', false)  applies and saves single_phase_cllc_param.slx
%
%   Thin wrapper: the mapping legacy name -> param.* of init_model.m (Rev. 01) is here, the work is
%   done by tools/migrate_model_to_param (same options: 'DryRun', 'SaveAs', 'Verbose', ...).
%
%   Notes on the mapping:
%   - the full bridges of the model read the DAB device data (mosfet.dab, ideal_switch.dab): mapped
%     to param.cllc.mosfet / param.cllc.ideal_switch, built with the CLLC PWM frequency and DC-link
%     voltage (same values as the DAB ones in the legacy init);
%   - the MOSFET thermal bridges without ZVS snubber read "weigth", a name the legacy init never
%     defined: mapped to param.thermal.weight;
%   - the active configuration set has StopTime = simlength, FixedStep = tc (ignored, ode23tb is
%     variable-step) and MaxStep = glb_time.tc; the inactive set "Configuration" is not touched.
%
%   After the migration run init_model.m with export_legacy_names = false and update the model
%   (Ctrl+D); list_model_workspace_names(model, 'Names', {...legacy roots...}) shows what is left.

map = {
    % objects and structs
    'hwdata.cllc',                                  'param.cllc.hw'
    'hwdata.dab',                                   'param.cllc.hw'         % not in the model, for completeness
    'mosfet.dab',                                   'param.cllc.mosfet'     % the full bridges read the DAB device data
    'mosfet.cllc',                                  'param.cllc.mosfet'
    'ideal_switch.dab',                             'param.cllc.ideal_switch'
    'ideal_switch.cllc',                            'param.cllc.ideal_switch'
    'igbt.dab',                                     'param.cllc.igbt'
    'igbt.cllc',                                    'param.cllc.igbt'
    'glb_time',                                     'param.glb_time'
    'cllc_ctrl',                                    'param.cllc.ctrl'
    'heatsink',                                     'param.thermal.heatsink'
    'lithium_ion_battery_1',                        'param.battery.pack_1'
    'lithium_ion_battery_2',                        'param.battery.pack_2'
    % simulation and configuration
    'simlength',                                    'param.sim.simlength'
    'tc',                                           'param.sim.tc'
    'nonlinear_iterations',                         'param.sim.nonlinear_iterations'
    % thermal (flat names read by the MOSFET thermal bridges without ZVS snubber)
    'Tambient',                                     'param.thermal.Tambient'
    'DThs_init',                                    'param.thermal.DThs_init'
    'weigth',                                       'param.thermal.weight'  % typo in the model, never defined by the legacy init
    'weight',                                       'param.thermal.weight'
    'cp_al',                                        'param.thermal.cp_al'
    'Rth_mosfet_HA',                                'param.thermal.Rth_mosfet_HA'
    'Rth_switch_HA',                                'param.thermal.Rth_switch_HA'
    'Rth_diode_HA',                                 'param.thermal.Rth_diode_HA'
    % measurement
    'adc_quantization',                             'param.meas.adc_quantization'
    'adc12_quantization',                           'param.meas.adc12_quantization'
    'adc16_quantization',                           'param.meas.adc16_quantization'
    'CurrentQuantization',                          'param.meas.CurrentQuantization'
    'VoltageQuantization',                          'param.meas.VoltageQuantization'
    % common control settings (not in the model, for completeness)
    'CTRPIFF_CLIP_RELEASE',                         'param.ctrl.CTRPIFF_CLIP_RELEASE'
    'mavarage_filter_frequency_base_order',         'param.ctrl.mavarage_filter_frequency_base_order'
    'dmavg_filter_enable_time',                     'param.ctrl.dmavg_filter_enable_time'
    'kp_align',                                     'param.ctrl.kp_align'
    'ki_align',                                     'param.ctrl.ki_align'
    'lim_up_align',                                 'param.ctrl.lim_up_align'
    'lim_down_align',                               'param.ctrl.lim_down_align'
    };

add_repo_tools_to_path();
report = migrate_model_to_param('single_phase_cllc', map, varargin{:});
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
        error('migrate_single_phase_cllc:tools', 'Folder "tools" of the repository not found: add it to the MATLAB path.');
    end
end
