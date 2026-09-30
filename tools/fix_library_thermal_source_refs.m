function report = fix_library_thermal_source_refs(varargin)
%FIX_LIBRARY_THERMAL_SOURCE_REFS  Make the thermal bridges of the library read Tambient / DThs_init from their mask
%
%   report = fix_library_thermal_source_refs()                 dry run: nothing changed
%   report = fix_library_thermal_source_refs('DryRun', false)  rewrites and saves the library files
%
%   Inside the masked bridges of library/masked_models/semiconductors_bridges the temperature sources
%   and the initial temperatures read the base workspace directly:
%       T = DThs_init+Tambient,  temperature = Tambient,  IGBT_junction_temperature = Tambient
%   instead of the mask parameters Tambient_m / DThs_init_m that every instance already sets. A model
%   initialized with the param structure therefore fails ("Unrecognized function or variable
%   'DThs_init'") unless the init also exports the two flat names. This function replaces, in every
%   dialog parameter of every block of the library files, the whole identifiers Tambient and DThs_init
%   with Tambient_m and DThs_init_m, only where an enclosing mask defines them; blocks whose mask does
%   not (three_phase_diode_rectifier_with_thermal_model has no such mask parameters) are reported as
%   'skipped: mask parameter missing' and must be completed by hand (add the two mask parameters,
%   then run again).
%
%   Options (name-value):
%       'Folder'   folder of the library files, default: the folder of
%                  full_bridge_mosfet_based_with_thermal_model_zvs.slx found on the path
%       'Files'    cellstr of .slx names to process, default: every .slx of the folder
%       'Names'    2-column cell {base_workspace_name, mask_parameter_name},
%                  default {'Tambient', 'Tambient_m'; 'DThs_init', 'DThs_init_m'}
%       'DryRun'   true (default) | false
%       'Verbose'  true (default)
%
%   Report: table with columns Library, Block, Parameter, Old, New, Status.
%   Once the library is fixed, the "Library workaround" section of the init scripts (Tambient and
%   DThs_init exported to the base workspace) can be deleted.

opt = struct('Folder', '', 'Files', {{}}, 'DryRun', true, 'Verbose', true, ...
    'Names', {{'Tambient', 'Tambient_m'; 'DThs_init', 'DThs_init_m'}});
for k = 1:2:numel(varargin)
    opt.(varargin{k}) = varargin{k+1};
end
if isempty(opt.Folder)
    ref = which('full_bridge_mosfet_based_with_thermal_model_zvs');
    if isempty(ref)
        error('fix_library_thermal_source_refs:folder', ...
            'Library folder not found on the path: pass it with ''Folder''.');
    end
    opt.Folder = fileparts(ref);
end
if isempty(opt.Files)
    d = dir(fullfile(opt.Folder, '*.slx'));
    opt.Files = {d.name};
end
names = opt.Names;
quick = ['(?<![\w.])(' strjoin(names(:,1)', '|') ')(?!\w)'];

rows = cell(0, 6);
for f = 1:numel(opt.Files)
    [~, lib] = fileparts(opt.Files{f});
    was_loaded = bdIsLoaded(lib);
    if ~was_loaded
        load_system(fullfile(opt.Folder, opt.Files{f}));
    end
    changed_here = 0;
    blocks = find_system(lib, 'LookUnderMasks', 'all', 'FollowLinks', 'off', ...
        'IncludeCommented', 'on', 'MatchFilter', @Simulink.match.allVariants, 'Type', 'Block');
    for ib = 1:numel(blocks)
        blk = blocks{ib};
        try
            dp = get_param(blk, 'DialogParameters');
        catch
            continue
        end
        if ~isstruct(dp)
            continue
        end
        pnames = fieldnames(dp);
        for ip = 1:numel(pnames)
            p = pnames{ip};
            if endsWith(p, 'Fcn') || any(strcmp(dp.(p).Attributes, 'read-only'))
                continue
            end
            try
                old = get_param(blk, p);
            catch
                continue
            end
            if ~ischar(old) || isempty(regexp(old, quick, 'once'))
                continue
            end
            % which names are used, and does an enclosing mask define the corresponding parameter
            mask_names = mask_variables_of_ancestors(blk, lib);
            new = old;
            missing = {};
            for n = 1:size(names, 1)
                pattern = ['(?<![\w.])' names{n, 1} '(?!\w)'];
                if isempty(regexp(old, pattern, 'once'))
                    continue
                end
                if any(strcmp(names{n, 2}, mask_names))
                    new = regexprep(new, pattern, names{n, 2});
                else
                    missing{end+1} = names{n, 2}; %#ok<AGROW>
                end
            end
            if ~isempty(missing)
                status = ['skipped: mask parameter missing (' strjoin(missing, ', ') ')'];
                new = old;
            elseif opt.DryRun
                status = 'dry-run';
            else
                try
                    if strcmp(get_param(lib, 'Lock'), 'on')
                        set_param(lib, 'Lock', 'off');
                    end
                    set_param(blk, p, new);
                    status = 'changed';
                    changed_here = changed_here + 1;
                catch ME
                    status = ['failed: ' ME.message];
                end
            end
            if opt.Verbose
                fprintf('%-9s %s | %s\n    %s\n -> %s\n', status, blk, p, old, new);
            end
            rows(end+1, :) = {lib, blk, p, old, new, status}; %#ok<AGROW>
        end
    end
    if ~opt.DryRun && changed_here > 0
        save_system(lib);
        fprintf('%s saved (%d parameter(s) changed)\n', lib, changed_here);
    end
    if ~was_loaded
        close_system(lib, 0);
    end
end

report = cell2table(rows, 'VariableNames', {'Library', 'Block', 'Parameter', 'Old', 'New', 'Status'});
if opt.DryRun
    fprintf('\nDry run: nothing changed. Call again with ''DryRun'', false to apply.\n');
end
fprintf('%d parameter(s) found, %d skipped, %d failed.\n', height(report), ...
    sum(startsWith(report.Status, 'skipped')), sum(startsWith(report.Status, 'failed')));
end

%% ---------------------------------------------------------------- local functions
function names = mask_variables_of_ancestors(blk, lib)
% Mask variable names of every masked subsystem enclosing BLK within the library (the block's own
% mask does not count: its parameter values are evaluated in the parent scope).
    names = {};
    current = get_param(blk, 'Parent');
    while ~isempty(current) && ~strcmp(current, lib)
        try
            m = Simulink.Mask.get(current);
        catch
            m = [];
        end
        if ~isempty(m) && ~isempty(m.Parameters)
            names = [names, {m.Parameters.Name}]; %#ok<AGROW>
        end
        current = get_param(current, 'Parent');
    end
end
