function issues = check_block_struct(p, p_ref, name)
%CHECK_BLOCK_STRUCT  Compare the mask struct of a library block with the example struct of the library.
%
%   issues = check_block_struct(p, p_ref, name)
%     p      struct (or struct array: every element is checked) given to the mask parameter p of the block,
%            e.g. param.afe_follower
%     p_ref  example struct of the library (grid_following_param_example, pmsm_inverter_param_example)
%     name   name of p for the messages, e.g. 'param.afe_follower'
%     issues cell array of messages, empty when p has the same fields as the example
%
%   The MATLAB Function and the Simscape component of the block read the fields by name: a field that is
%   missing stops the compilation with a message about the inside of the block. A field that the example
%   does not have is not read by the block (often a typo). For the fields with at most six numbers (gains,
%   flags, poles, small matrices) a different size is also reported: the control code indexes them.

    issues = {};
    for k = 1:numel(p)
        if numel(p) > 1
            nk = sprintf('%s(%d)', name, k);
        else
            nk = name;
        end
        issues = [issues, compare(p(k), p_ref, nk)]; %#ok<AGROW>
    end
end

function issues = compare(a, b, name)
    issues = {};
    fa = fieldnames(a);
    fb = fieldnames(b);
    for k = 1:numel(fb)
        f = fb{k};
        if ~isfield(a, f)
            issues{end+1} = sprintf('%s.%s is missing (the block reads it)', name, f); %#ok<AGROW>
            continue
        end
        va = a.(f);
        vb = b.(f);
        if isstruct(vb)
            if ~isstruct(va)
                issues{end+1} = sprintf('%s.%s must be a struct', name, f); %#ok<AGROW>
            else
                issues = [issues, compare(va, vb, [name '.' f])]; %#ok<AGROW>
            end
        elseif ~isnumeric(va) && ~islogical(va)
            issues{end+1} = sprintf('%s.%s must be numeric', name, f); %#ok<AGROW>
        elseif numel(vb) <= 6 && ~isequal(size(va), size(vb))
            issues{end+1} = sprintf('%s.%s has size %s, the example %s', name, f, ...
                mat2str(size(va)), mat2str(size(vb))); %#ok<AGROW>
        end
    end
    extra = setdiff(fa, fb);
    for k = 1:numel(extra)
        issues{end+1} = sprintf('%s.%s is not in the example: the block does not read it', name, extra{k}); %#ok<AGROW>
    end
end
