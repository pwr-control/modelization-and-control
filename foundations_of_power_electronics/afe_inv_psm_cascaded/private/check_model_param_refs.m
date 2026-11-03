function [missing, refs] = check_model_param_refs(model, param)
%CHECK_MODEL_PARAM_REFS  List the "param.*" references of the active blocks of a model and report the ones that
%   do not resolve in the struct param (model must be loaded).
%
%   [missing, refs] = check_model_param_refs(model, param)
%     refs    : all distinct param.* references found in the block parameters of the blocks that are not
%               commented out (neither the block nor one of its parents), library links followed, masks
%               included, plus StartTime, StopTime, FixedStep and MaxStep of the model configuration
%     missing : the subset that does not exist in param
%
%   Static check: the expressions are not evaluated, only the names are matched. A struct-array index is
%   checked: param.afe_follower(2) is missing when param.afe_follower has one element only. Commented blocks
%   are skipped, so the old modules afe_abc_inv_psm_mod1/2 (commented in the model) do not count.

    blocks = find_system(model, 'FindAll', 'on', 'LookUnderMasks', 'all', 'FollowLinks', 'on', ...
        'IncludeCommented', 'on', 'Type', 'block');
    refs = {};
    for k = 1:numel(blocks)
        if ~is_active(blocks(k), model)
            continue
        end
        try
            dp = get_param(blocks(k), 'DialogParameters');
        catch
            dp = [];
        end
        if isempty(dp) || ~isstruct(dp)
            continue
        end
        names = fieldnames(dp);
        for j = 1:numel(names)
            try
                v = get_param(blocks(k), names{j});
            catch
                continue
            end
            refs = [refs, find_refs(v)]; %#ok<AGROW>
        end
    end
    for name = {'StartTime', 'StopTime', 'FixedStep', 'MaxStep'}
        try
            refs = [refs, find_refs(get_param(model, name{1}))]; %#ok<AGROW>
        catch
        end
    end
    refs = unique(refs);

    missing = {};
    for k = 1:numel(refs)
        if ~resolves(param, refs{k})
            missing{end+1} = refs{k}; %#ok<AGROW>
        end
    end
    missing = missing(:);
    refs = refs(:);
end

function r = find_refs(v)
    r = {};
    if isstring(v)
        v = char(v);
    end
    if ischar(v)
        r = regexp(v, 'param(\.[A-Za-z_]\w*|\(\d+\))+', 'match');
    end
end

function a = is_active(h, model)
%IS_ACTIVE  true when neither the block nor one of its parent subsystems is commented out.
    a = true;
    b = getfullname(h);
    while ~strcmp(b, model)
        try
            if ~strcmp(get_param(b, 'Commented'), 'off')
                a = false;
                return
            end
            b = get_param(b, 'Parent');
        catch
            return
        end
    end
end

function ok = resolves(param, ref)
%RESOLVES  true when the reference (dotted names, struct-array indices) exists in param.
    tok = regexp(ref(6:end), '\.[A-Za-z_]\w*|\(\d+\)', 'match');   % after "param": '.name' or '(index)'
    v = param;
    ok = true;
    for k = 1:numel(tok)
        t = tok{k};
        if t(1) == '('
            idx = str2double(t(2:end-1));
            if (isstruct(v) || isnumeric(v) || islogical(v)) && idx >= 1 && idx <= numel(v)
                v = v(idx);
            else
                ok = false;
                return
            end
        elseif isstruct(v) && isfield(v, t(2:end)) && ~isempty(v)
            v = v(1).(t(2:end));
        elseif isobject(v)                              % e.g. the ss object of a filter: property A, B, C
            try
                v = v.(t(2:end));
            catch
                ok = false;
                return
            end
        else
            ok = false;
            return
        end
    end
end
