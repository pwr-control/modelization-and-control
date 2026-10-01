function report = migrate_single_phase_inverter(varargin)
%MIGRATE_SINGLE_PHASE_INVERTER  Rewrite single_phase_inverter.slx from the legacy workspace names to param.*
%
%   report = migrate_single_phase_inverter()                 dry run: nothing changed, report of the changes
%   report = migrate_single_phase_inverter('DryRun', false)  applies and saves single_phase_inverter_param.slx
%
%   Thin wrapper: the mapping legacy name -> param.* of init_model.m (Rev. 01) is here, the work is
%   done by tools/migrate_model_to_param (same options: 'DryRun', 'SaveAs', 'Verbose', ...).
%
%   Notes on the mapping (names found in the model XML, 2026-09-30; 126 distinct names, ~590 values):
%   - the full bridge (MOSFET thermal variant) reads mosfet.inv.*, the diode rectifier blocks read
%     igbt.inv.Csnubber / Rsnubber / Rth_diode_JH: both mapped to the datasets of the single-phase
%     inverter stage, param.spinv.mosfet / param.spinv.igbt (same values: udc enters only the ZVS snubber);
%   - the thermal diode rectifier (library link) has the flat Rth_mosfet_HA in its mask parameter
%     rth_diode_HA_m: mapped to param.thermal.Rth_mosfet_HA. Inside the library block Tambient and
%     DThs_init are still read from the base workspace: init_model.m keeps exporting them;
%   - the grid PLL of the inverter control reads afe_ctrl.fht_grid.* (param.afe.ctrl);
%   - the Variant Source blocks read frequency_set, rpi_enable, system_identification_enable,
%     use_current_controller_from_ccaller_mod1, use_single_phase_inverter_based_* and
%     use_system_identification_based_* (param.spinv and param.spinv.impl);
%   - adc11_quantization (Quantizer of the commented-out acquisition_and_scaling_current) is never
%     defined by the legacy init: mapped to param.meas.adc_quantization (same 1/2^11);
%   - the active configuration set ("Configuration1") has StopTime = simlength, FixedStep = tc (ignored,
%     ode23tb) and MaxStep = glb_time.tc; the inactive set reads an undefined Ts and is left alone.
%
%   After the migration run init_model.m with export_legacy_names = false and update the model
%   (Ctrl+D); list_model_workspace_names(model, 'Names', {...legacy roots...}) shows what is left.

map = {
    % objects and structs
    'hwdata.single_phase_inverter',                 'param.spinv.hw'
    'single_phase_inverter_ctrl',                   'param.spinv.ctrl'
    'sogi.flt',                                     'param.spinv.sogi.flt'
    'sogi.fltd',                                    'param.spinv.sogi.fltd'
    'igbt.inv',                                     'param.spinv.igbt'
    'mosfet.inv',                                   'param.spinv.mosfet'
    'ideal_switch.inv',                             'param.spinv.ideal_switch'
    'glb_time',                                     'param.glb_time'
    'grid_emu',                                     'param.grid.emu'
    'frt_data',                                     'param.frt.data'
    'afe_ctrl',                                     'param.afe.ctrl'
    'filters',                                      'param.ctrl.filters'
    'heatsink',                                     'param.thermal.heatsink'
    'output_transformer',                           'param.trafo_load.output_transformer'
    'diode.rectifier',                              'param.diode.rectifier'
    % simulation and configuration
    'simlength',                                    'param.sim.simlength'
    'tc',                                           'param.sim.tc'
    'nonlinear_iterations',                         'param.sim.nonlinear_iterations'
    'load_step_time',                               'param.sim.load_step_time'
    'time_aux_power_supply_fault',                  'param.sim.time_aux_power_supply_fault'
    % thermal
    'Tambient',                                     'param.thermal.Tambient'
    'DThs_init',                                    'param.thermal.DThs_init'
    'Rth_mosfet_HA',                                'param.thermal.Rth_mosfet_HA'
    'Rth_diode_HA',                                 'param.thermal.Rth_diode_HA'
    'Rth_switch_HA',                                'param.thermal.Rth_switch_HA'
    % single-phase inverter: operating mode and variants
    'frequency_set',                                'param.spinv.frequency_set'
    'omega_set',                                    'param.spinv.omega_set'
    'rpi_enable',                                   'param.spinv.rpi_enable'
    'system_identification_enable',                 'param.spinv.system_identification_enable'
    'kepsilon',                                     'param.spinv.kepsilon'
    'sogi_delta',                                   'param.spinv.sogi_delta'
    'use_current_controller_from_ccaller_mod1',     'param.spinv.impl.use_current_controller_from_ccaller_mod1'
    'use_phase_shift_filter_from_ccaller_mod1',     'param.spinv.impl.use_phase_shift_filter_from_ccaller_mod1'
    'use_sogi_from_ccaller_mod1',                   'param.spinv.impl.use_sogi_from_ccaller_mod1'
    'use_single_phase_inverter_based_FHT',          'param.spinv.impl.use_single_phase_inverter_based_FHT'
    'use_single_phase_inverter_based_SOGI',         'param.spinv.impl.use_single_phase_inverter_based_SOGI'
    'use_single_phase_inverter_based_PHSH',         'param.spinv.impl.use_single_phase_inverter_based_PHSH'
    'use_single_phase_inverter_based_SOGI_ccaller', 'param.spinv.impl.use_single_phase_inverter_based_SOGI_ccaller'
    'use_single_phase_inverter_based_PHSH_ccaller', 'param.spinv.impl.use_single_phase_inverter_based_PHSH_ccaller'
    'use_system_identification_based_FHT',          'param.spinv.impl.use_system_identification_based_FHT'
    'use_system_identification_based_SOGI',         'param.spinv.impl.use_system_identification_based_SOGI'
    'use_system_identification_based_PHSH',         'param.spinv.impl.use_system_identification_based_PHSH'
    'use_system_identification_based_SOGI_ccaller', 'param.spinv.impl.use_system_identification_based_SOGI_ccaller'
    'use_system_identification_based_PHSH_ccaller', 'param.spinv.impl.use_system_identification_based_PHSH_ccaller'
    % single-phase inverter: references
    'iph_grid_pu_ref_1',                            'param.spinv.ref.iph_grid_pu_ref_1'
    'iph_grid_pu_ref_2',                            'param.spinv.ref.iph_grid_pu_ref_2'
    'iph_grid_pu_ref_3',                            'param.spinv.ref.iph_grid_pu_ref_3'
    'time_step_ref_1',                              'param.spinv.ref.time_step_ref_1'
    'time_step_ref_2',                              'param.spinv.ref.time_step_ref_2'
    'time_step_ref_3',                              'param.spinv.ref.time_step_ref_3'
    % load
    'rload',                                        'param.trafo_load.rload'
    'lload',                                        'param.trafo_load.lload'
    'uload',                                        'param.trafo_load.uload'
    % measurement
    'adc_quantization',                             'param.meas.adc_quantization'
    'adc11_quantization',                           'param.meas.adc_quantization'
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
report = migrate_model_to_param('single_phase_inverter', map, varargin{:});
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
        error('migrate_single_phase_inverter:tools', 'Folder "tools" of the repository not found: add it to the MATLAB path.');
    end
end
