function usage = list_model_workspace_names(model, varargin)
%LIST_MODEL_WORKSPACE_NAMES  Base-workspace names read by the blocks of a Simulink model (no compile)
%
%   usage = list_model_workspace_names(model)
%       Scans every block parameter, mask parameter, library-link instance parameter, variant
%       condition and the StopTime / FixedStep / MaxStep of the active configuration set of MODEL
%       (loaded if it is not open) for identifiers, and keeps the ones whose root is a variable of
%       the base workspace: run the init script first, then call this function. No model update is
%       needed, so it works on models above the block limit of the Home license too.
%
%   usage = list_model_workspace_names(model, 'Names', {'glb_time', 'hwdata', ...})
%       Checks against this list of roots instead of the base workspace.
%
%   Options (name-value):
%       'Names'     cellstr of variable roots to look for (default: base workspace variables)
%       'Summary'   true (default) prints one line per distinct name with the number of uses
%
%   Returns a table with one row per use: Name (as written in the block, dotted names kept),
%   Root, Block, Parameter, Value. Signal names, tags, To Workspace variable names, callbacks and
%   graphics are not scanned. Use it to write the main body of a param init (what the model reads)
%   and the map of migrate_model_to_param, and again after the migration with the param init only
%   in the workspace to check that nothing legacy is left (then 'Names' can be the legacy roots).
%
%   See also migrate_model_to_param.

opt = struct('Names', {{}}, 'Summary', true);
for k = 1:2:numel(varargin)
    opt.(varargin{k}) = varargin{k+1};
end
if isempty(opt.Names)
    opt.Names = evalin('base', 'who');
end
opt.Names = cellstr(opt.Names);

if ~bdIsLoaded(model)
    load_system(model);
end

skip_params = {'Name', 'Tag', 'Description', 'PropagatedSignals', 'OutputSignals', 'IconDisplay', ...
    'MaskDisplay', 'MaskDescription', 'MaskHelp', 'MaskPromptString', 'MaskType', 'SourceBlock', ...
    'LinkStatus', 'Position', 'Ports', 'ZOrder', 'BackgroundColor', 'ForegroundColor', 'FontName', ...
    'AttributesFormatString', 'BlockDescription', 'SampleTimeColors', 'VariantControl', 'VariantControls', ...
    'MaskValueString', 'MaskVariables', 'ScopeSpecificationString', 'UserData', 'InstanceData', ...
    'GotoTag', 'VariableName', 'DataLoggingVariableName'};

% identifiers, dotted names included, not preceded by a word character, a dot or a quote
ident = '(?<![\w.''])[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*';

blocks = find_system(model, 'LookUnderMasks', 'all', 'FollowLinks', 'off', ...
    'IncludeCommented', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'Block');

rows = cell(0, 5);
for ib = 1:numel(blocks)
    blk = blocks{ib};
    values = cell(0, 2);            % {parameter, value}

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
            if any(strcmp(dp.(p).Attributes, 'read-only'))
                continue
            end
            try
                v = get_param(blk, p);
            catch
                continue
            end
            if ischar(v) && ~isempty(v)
                values(end+1, :) = {p, v}; %#ok<AGROW>
            end
        end
    end
    try
        vc = get_param(blk, 'VariantControl');
        if ischar(vc) && ~isempty(vc)
            values(end+1, :) = {'VariantControl', vc}; %#ok<AGROW>
        end
    catch
    end
    try
        vcs = get_param(blk, 'VariantControls');
        if iscell(vcs) && all(cellfun(@ischar, vcs))
            for k = 1:numel(vcs)
                values(end+1, :) = {'VariantControls', vcs{k}}; %#ok<AGROW>
            end
        end
    catch
    end

    for iv = 1:size(values, 1)
        rows = [rows; uses_in(values{iv, 2}, blk, values{iv, 1}, ident, opt.Names)]; %#ok<AGROW>
    end
end

% active configuration set
cs = getActiveConfigSet(model);
for cp = {'StopTime', 'FixedStep', 'MaxStep'}
    try
        v = get_param(cs, cp{1});
    catch
        continue
    end
    if ischar(v)
        rows = [rows; uses_in(v, [model ' (configuration)'], cp{1}, ident, opt.Names)]; %#ok<AGROW>
    end
end

usage = cell2table(rows, 'VariableNames', {'Name', 'Root', 'Block', 'Parameter', 'Value'});
usage = sortrows(usage, {'Root', 'Name', 'Block'});

if opt.Summary
    [names, ~, idx] = unique(usage.Name);
    counts = accumarray(idx, 1);
    fprintf('%s: %d use(s) of %d workspace name(s) in %d block(s)\n', model, height(usage), ...
        numel(names), numel(unique(usage.Block)));
    for k = 1:numel(names)
        fprintf('  %-50s %4d\n', names{k}, counts(k));
    end
end
end

%% ---------------------------------------------------------------- local functions
function rows = uses_in(value, blk, param, ident, names)
% One row per identifier of VALUE whose root is in NAMES.
    rows = cell(0, 5);
    tokens = unique(regexp(value, ident, 'match'), 'stable');
    for k = 1:numel(tokens)
        root = regexp(tokens{k}, '^[A-Za-z_]\w*', 'match', 'once');
        if any(strcmp(root, names))
            rows(end+1, :) = {tokens{k}, root, blk, param, strtrim(value)}; %#ok<AGROW>
        end
    end
end
