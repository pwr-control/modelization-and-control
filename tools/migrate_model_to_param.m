function report = migrate_model_to_param(model, map, varargin)
%MIGRATE_MODEL_TO_PARAM  Rewrite the workspace references of a Simulink model from legacy names to param.*
%
%   report = migrate_model_to_param(model, map)
%       Dry run: nothing is changed, the report lists every block parameter, mask parameter,
%       library-link instance parameter, variant condition and configuration parameter that would
%       be rewritten.
%
%   report = migrate_model_to_param(model, map, 'DryRun', false)
%       Applies the changes and saves the result as <model>_param.slx (the original file is not
%       touched). Rename it afterwards, or pass 'SaveAs', '' to save in place.
%
%   model   model name or path (loaded if it is not open)
%   map     N-by-2 cell array {legacy_name, new_name}: the mapping of the model's init_model.m.
%           Legacy names may be dotted ('hwdata.afe' -> 'param.afe.hw'); longer names are applied
%           first, so a dotted name wins over its root ('psm.omega_bez' -> 'param.psm.machine.omega_bez'
%           while 'psm_ctrl.kp_i' is matched by its own rule).
%
%   Options (name-value):
%       'DryRun'        true (default) | false
%       'SaveAs'        name of the migrated model, default [model '_param']; '' saves in place
%       'Verbose'       true (default) prints every change
%       'PathRules'     N-by-3 cell {path_pattern, legacy_name, new_name} for names whose mapping
%                       depends on where the block is: for each legacy name the first row whose
%                       pattern is contained in the block path (case-insensitive) is used; an empty
%                       pattern is the default. Example (afe_inv_psm):
%                       {'/inverter/', 'ideal_switch', 'param.inv.ideal_switch'; '', 'ideal_switch', 'param.afe.ideal_switch'}
%       'ConfigParams'  configuration parameters of the active set to rewrite,
%                       default {'StopTime', 'FixedStep', 'MaxStep'}
%       'SkipParams'    extra block parameter names never rewritten (added to the built-in list)
%
%   Only whole identifiers are replaced (word boundaries, not preceded by "." or a word character).
%   Names shadowed by a mask variable of an enclosing masked subsystem are left alone. Library-linked
%   blocks are not entered (FollowLinks off): only their dialog parameters are rewritten, their content
%   belongs to the library. Parameters with the 'dont-eval' attribute (SampleTime of To Workspace,
%   Unit Delay...) are expressions and are rewritten; 'read-only' ones are skipped. FixedStep cannot be
%   read while the solver is variable-step: the stored value is ignored by Simulink, nothing to migrate.
%
%   After the migration run init_model.m with export_legacy_names = false and update the model
%   (Ctrl+D): an unresolved name means a reference this function did not see (check with
%   list_model_workspace_names).
%
%   Report: table with columns Block, Parameter, Old, New, Status ('changed', 'dry-run',
%   'failed: <message>').
%
%   See also list_model_workspace_names.

%% options
opt = struct('DryRun', true, 'SaveAs', [model '_param'], 'Verbose', true, ...
    'PathRules', {cell(0, 3)}, 'ConfigParams', {{'StopTime', 'FixedStep', 'MaxStep'}}, 'SkipParams', {{}});
for k = 1:2:numel(varargin)
    opt.(varargin{k}) = varargin{k+1};
end
validateattributes(map, {'cell'}, {'ncols', 2});
if ~isempty(opt.PathRules)
    validateattributes(opt.PathRules, {'cell'}, {'ncols', 3});
end

if ~bdIsLoaded(model)
    load_system(model);
end

% longest first, so that dotted names win over their root
map = sort_longest_first(map);

% block parameters never rewritten (names, signal labels, callbacks, graphics, big opaque strings)
skip_params = [{'Name', 'Tag', 'Description', 'PropagatedSignals', 'OutputSignals', 'IconDisplay', ...
    'MaskDisplay', 'MaskDescription', 'MaskHelp', 'MaskPromptString', 'MaskType', 'SourceBlock', ...
    'LinkStatus', 'Position', 'Ports', 'ZOrder', 'BackgroundColor', 'ForegroundColor', 'FontName', ...
    'AttributesFormatString', 'BlockDescription', 'SampleTimeColors', 'VariantControl', 'VariantControls', ...
    'MaskValueString', 'MaskVariables', 'ScopeSpecificationString', 'UserData', 'InstanceData', ...
    'GotoTag', 'VariableName', 'DataLoggingVariableName'}, opt.SkipParams(:)'];

% one regular expression that tells in a single pass whether a value can contain a legacy name
legacy_all = [map(:,1); opt.PathRules(:,2)];
roots = unique(cellfun(@(n) regexp(n, '^[A-Za-z_]\w*', 'match', 'once'), legacy_all, 'UniformOutput', false));
quick = ['(?<![\w.])(' strjoin(roots, '|') ')(?!\w)'];

%% blocks: all variants, under local masks, not inside library links
blocks = find_system(model, 'LookUnderMasks', 'all', 'FollowLinks', 'off', ...
    'IncludeCommented', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'Block');

rows = cell(0, 5);
for ib = 1:numel(blocks)
    blk = blocks{ib};
    shadow = mask_variables_of_ancestors(blk, model);
    rule = local_rule(map, opt.PathRules, blk);

    % dialog parameters (block parameters, mask parameters, instance parameters of library links)
    try
        dp = get_param(blk, 'DialogParameters');
    catch
        dp = [];
    end
    if isstruct(dp)
        pnames = fieldnames(dp);
        for ip = 1:numel(pnames)
            p = pnames{ip};
            if any(strcmp(p, skip_params)) || endsWith(p, 'Fcn') || startsWith(p, 'RTWMemSec')
                continue
            end
            attr = dp.(p).Attributes;
            if any(strcmp(attr, 'read-only'))
                continue
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

%% model configuration (active configuration set)
cs = getActiveConfigSet(model);
if isa(cs, 'Simulink.ConfigSetRef')
    warning('migrate_model_to_param:configRef', ...
        'The active configuration set is a reference (%s): %s must be changed there.', ...
        cs.Name, strjoin(opt.ConfigParams, ', '));
else
    for cp = opt.ConfigParams(:)'
        try
            old = get_param(cs, cp{1});
        catch
            continue                % e.g. FixedStep with a variable-step solver: not used, nothing to migrate
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
    fprintf('\nDry run: nothing changed. Call again with ''DryRun'', false to apply.\n');
end

%% report
report = cell2table(rows, 'VariableNames', {'Block', 'Parameter', 'Old', 'New', 'Status'});
fprintf('%d parameter(s) affected, %d failed.\n', height(report), sum(startsWith(report.Status, 'failed')));
end

%% ---------------------------------------------------------------- local functions
function map = sort_longest_first(map)
    [~, order] = sort(cellfun(@numel, map(:,1)), 'descend');
    map = map(order, :);
end

function rule = local_rule(map, pathrules, blk)
% Mapping of this block: MAP plus, for each legacy name of PATHRULES, the first row whose path
% pattern matches the block path (an empty pattern always matches).
    rule = map;
    if isempty(pathrules)
        return
    end
    names = unique(pathrules(:,2), 'stable');
    for k = 1:numel(names)
        rows = find(strcmp(pathrules(:,2), names{k}));
        for r = rows(:)'
            pat = lower(pathrules{r, 1});
            if isempty(pat) || contains(lower(blk), pat)
                rule(end+1, :) = pathrules(r, [2 3]); %#ok<AGROW>
                break
            end
        end
    end
    rule = sort_longest_first(rule);
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
