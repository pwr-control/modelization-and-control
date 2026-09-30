function report = migrate_afe_inv_psm(varargin)
%MIGRATE_AFE_INV_PSM  Rewrite afe_inv_psm.slx from the legacy workspace names to param.*
%
%   report = migrate_afe_inv_psm()
%       Dry run on the model 'afe_inv_psm': nothing is changed, the report lists every block
%       parameter, variant condition and configuration parameter that would be rewritten.
%
%   report = migrate_afe_inv_psm('DryRun', false)
%       Applies the changes and saves the result as afe_inv_psm_param.slx (the original file is
%       not touched). Rename it afterwards, or pass 'SaveAs', '' to save in place.
%
%   Options (name-value):
%       'Model'    model name or path, default 'afe_inv_psm' (loaded if it is not open)
%       'DryRun'   true (default) | false
%       'SaveAs'   name of the migrated model, default 'afe_inv_psm_param'; '' saves in place
%       'Verbose'  true (default) prints every change
%
%   The mapping legacy name -> param.* is the one of init_model.m (Rev. 01). Only whole
%   identifiers are replaced (word boundaries, not preceded by "." or a word character), so
%   psm.omega_bez becomes param.psm.machine.omega_bez while psm_ctrl.kp_i is matched by its own
%   rule. Names shadowed by a mask variable of an enclosing masked subsystem are left alone.
%   ideal_switch.* is mapped by position: blocks under an ".../inverter/..." path take
%   param.inv.ideal_switch, all the others param.afe.ideal_switch.
%
%   Library-linked blocks are not entered (FollowLinks off): only their dialog parameters are
%   rewritten, their content belongs to the library.
%
%   After the migration run init_model.m with export_legacy_names = false and update the model
%   (Ctrl+D): an unresolved name means a reference this script did not see.
%
%   Report: table with columns Block, Parameter, Old, New, Status ('changed', 'dry-run',
%   'failed: <message>').

%% options
opt = struct('Model', 'afe_inv_psm', 'DryRun', true, 'SaveAs', 'afe_inv_psm_param', 'Verbose', true);
for k = 1:2:numel(varargin)
    opt.(varargin{k}) = varargin{k+1};
end
model = opt.Model;

if ~bdIsLoaded(model)
    load_system(model);
end

%% mapping: legacy name -> new name (longest first, so that dotted names win over their root)
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
[~, order] = sort(cellfun(@numel, map(:,1)), 'descend');
map = map(order, :);

% position-dependent rule
ideal_switch_afe = 'param.afe.ideal_switch';
ideal_switch_inv = 'param.inv.ideal_switch';

% block parameters never rewritten (names, signal labels, callbacks, graphics, big opaque strings)
skip_params = {'Name', 'Tag', 'Description', 'PropagatedSignals', 'OutputSignals', 'IconDisplay', ...
    'MaskDisplay', 'MaskDescription', 'MaskHelp', 'MaskPromptString', 'MaskType', 'SourceBlock', ...
    'LinkStatus', 'Position', 'Ports', 'ZOrder', 'BackgroundColor', 'ForegroundColor', 'FontName', ...
    'AttributesFormatString', 'BlockDescription', 'SampleTimeColors', 'VariantControl', 'VariantControls', ...
    'MaskValueString', 'MaskVariables', 'ScopeSpecificationString', 'UserData', 'InstanceData'};

% one regular expression that tells in a single pass whether a value can contain a legacy name
roots = unique(cellfun(@(n) regexp(n, '^[A-Za-z_]\w*', 'match', 'once'), [map(:,1); {'ideal_switch'}], ...
    'UniformOutput', false));
quick = ['(?<![\w.])(' strjoin(roots, '|') ')(?!\w)'];

%% blocks: all variants, under local masks, not inside library links
blocks = find_system(model, 'LookUnderMasks', 'all', 'FollowLinks', 'off', ...
    'IncludeCommented', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'Block');

rows = cell(0, 5);
for ib = 1:numel(blocks)
    blk = blocks{ib};
    shadow = mask_variables_of_ancestors(blk, model);
    rule = local_map(map, blk, ideal_switch_afe, ideal_switch_inv);

    % dialog parameters (block and mask parameters)
    try
        dp = get_param(blk, 'DialogParameters');
    catch
        dp = [];
    end
    if isstruct(dp)
        pnames = fieldnames(dp);
        for ip = 1:numel(pnames)
            p = pnames{ip};
            if any(strcmp(p, skip_params)) || endsWith(p, 'Fcn')
                continue
            end
            attr = dp.(p).Attributes;
            if any(strcmp(attr, 'read-only'))
                continue                % 'dont-eval' parameters (e.g. SampleTime) are expressions too: kept
            end
            try
                old = get_param(blk, p);
            catch
                continue
            end
            if ~ischar(old) || isempty(old)
                continue
            end
            new = rewrite(old, rule, shadow, quick);
            if ~strcmp(new, old)
                rows(end+1, :) = apply_change(blk, p, old, new, opt); %#ok<AGROW>
            end
        end
    end

    % variant conditions of the choices of a Variant Subsystem
    try
        vc = get_param(blk, 'VariantControl');
    catch
        vc = '';
    end
    if ischar(vc) && ~isempty(vc)
        new = rewrite(vc, rule, shadow, quick);
        if ~strcmp(new, vc)
            rows(end+1, :) = apply_change(blk, 'VariantControl', vc, new, opt); %#ok<AGROW>
        end
    end

    % variant conditions of Variant Source / Variant Sink blocks
    try
        vcs = get_param(blk, 'VariantControls');
    catch
        vcs = [];
    end
    if iscell(vcs) && ~isempty(vcs) && all(cellfun(@ischar, vcs))
        newc = cellfun(@(c) rewrite(c, rule, shadow, quick), vcs, 'UniformOutput', false);
        if ~isequal(newc, vcs)
            % the report shows the conditions joined by " ; ", the block receives the cell array
            rows(end+1, :) = apply_change(blk, 'VariantControls', strjoin(vcs, ' ; '), strjoin(newc, ' ; '), opt, newc); %#ok<AGROW>
        end
    end
end

%% model configuration: StopTime and FixedStep
cs = getActiveConfigSet(model);
if isa(cs, 'Simulink.ConfigSetRef')
    warning('migrate_afe_inv_psm:configRef', ...
        'The active configuration set is a reference (%s): StopTime and FixedStep must be changed there.', cs.Name);
else
    for cp = {'StopTime', 'FixedStep'}
        try
            old = get_param(cs, cp{1});
        catch
            % FixedStep cannot be read while the solver is variable-step (ode23t): the stored
            % value 'tc' is ignored by Simulink, so there is nothing to migrate
            continue
        end
        if ischar(old)
            new = rewrite(old, map, {}, quick);
            if ~strcmp(new, old)
                rows(end+1, :) = apply_config_change(cs, cp{1}, old, new, opt, model); %#ok<AGROW>
            end
        end
    end
end

%% save
if ~opt.DryRun
    if isempty(opt.SaveAs)
        save_system(model);
        saved = get_param(model, 'FileName');
    else
        save_system(model, opt.SaveAs);
        saved = get_param(opt.SaveAs, 'FileName');
    end
    fprintf('\nModel saved as %s\n', saved);
else
    fprintf('\nDry run: nothing changed. Call migrate_afe_inv_psm(''DryRun'', false) to apply.\n');
end

%% report
report = cell2table(rows, 'VariableNames', {'Block', 'Parameter', 'Old', 'New', 'Status'});
fprintf('%d parameter(s) affected, %d failed.\n', height(report), sum(startsWith(report.Status, 'failed')));
end

%% ---------------------------------------------------------------- local functions
function rule = local_map(map, blk, ideal_switch_afe, ideal_switch_inv)
% Adds the position-dependent ideal_switch rule to the mapping of this block.
    if contains(lower(blk), '/inverter/')
        rule = [map; {'ideal_switch', ideal_switch_inv}];
    else
        rule = [map; {'ideal_switch', ideal_switch_afe}];
    end
end

function new = rewrite(old, rule, shadow, quick)
% Replaces every legacy identifier of RULE found in OLD as a whole name, unless it is shadowed
% by a mask variable listed in SHADOW. A name is whole when it is not preceded by a word
% character or a dot and not followed by a word character. QUICK is the single regular
% expression that decides in one pass whether OLD can contain any legacy name at all.
    new = old;
    if isempty(regexp(old, quick, 'once'))
        return
    end
    for k = 1:size(rule, 1)
        legacy = rule{k, 1};
        root = regexp(legacy, '^[A-Za-z_]\w*', 'match', 'once');
        if any(strcmp(root, shadow))
            continue
        end
        pattern = ['(?<![\w.])' regexptranslate('escape', legacy) '(?!\w)'];
        new = regexprep(new, pattern, rule{k, 2});
    end
end

function names = mask_variables_of_ancestors(blk, model)
% Mask variable names of every masked subsystem enclosing BLK (the block's own mask does not
% count: its values are evaluated in the parent scope).
    names = {};
    parent = get_param(blk, 'Parent');
    while ~isempty(parent) && ~strcmp(parent, model)
        try
            m = Simulink.Mask.get(parent);
        catch
            m = [];
        end
        if ~isempty(m) && ~isempty(m.Parameters)
            names = [names, {m.Parameters.Name}]; %#ok<AGROW>
        end
        parent = get_param(parent, 'Parent');
    end
end

function row = apply_change(blk, p, old, new, opt, value)
% OLD and NEW are the strings shown in the report; VALUE (optional) is what the block receives
% when it differs from NEW (the cell array of a Variant Source / Variant Sink).
    if nargin < 6
        value = new;
    end
    if opt.DryRun
        status = 'dry-run';
    else
        try
            set_param(blk, p, value);
            status = 'changed';
        catch ME
            status = ['failed: ' ME.message];
        end
    end
    if opt.Verbose
        fprintf('%-9s %s | %s\n    %s\n -> %s\n', status, blk, p, old, new);
    end
    row = {blk, p, old, new, status};
end

function row = apply_config_change(cs, p, old, new, opt, model)
    if opt.DryRun
        status = 'dry-run';
    else
        try
            set_param(cs, p, new);
            status = 'changed';
        catch ME
            status = ['failed: ' ME.message];
        end
    end
    if opt.Verbose
        fprintf('%-9s %s | ConfigSet.%s\n    %s\n -> %s\n', status, model, p, old, new);
    end
    row = {[model ' (configuration)'], p, old, new, status};
end
