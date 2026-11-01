function [missing, refs] = check_model_param_refs(model, param)
%CHECK_MODEL_PARAM_REFS  List the "param.*" references of the block dialogs of a model and report the ones
%   that do not resolve in the struct param (model must be loaded).
%
%   [missing, refs] = check_model_param_refs(model, param)
%     refs    : all distinct param.* references found in the block parameters (library links followed,
%               masks included)
%     missing : the subset that does not exist in param
%
%   Static check: the expressions are not evaluated, only the dotted names are matched, so a wrong index
%   (e.g. Ld_fht(3)) is not detected. MATLAB Function blocks take param.* through their input ports.

    blocks = find_system(model, 'FindAll', 'on', 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Type', 'block');
    refs = {};
    for k = 1:numel(blocks)
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
            if isstring(v)
                v = char(v);
            end
            if ~ischar(v)
                continue
            end
            r = regexp(v, 'param(\.[A-Za-z_]\w*)+', 'match');
            if ~isempty(r)
                refs = [refs, r]; %#ok<AGROW>
            end
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

function ok = resolves(param, ref)
    parts = strsplit(ref, '.');
    v = param;
    ok = true;
    for p = 2:numel(parts)
        name = parts{p};
        if isstruct(v) && isfield(v, name)
            v = v.(name);
        elseif isobject(v) && isprop(v, name)
            v = v.(name);
        else
            ok = false;
            return
        end
    end
end
