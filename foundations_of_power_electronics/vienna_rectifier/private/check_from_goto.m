function issues = check_from_goto(model)
%CHECK_FROM_GOTO  From blocks of the active part of a model whose tag has no active Goto (model loaded).
%
%   issues = check_from_goto(model)   cell array with one message per tag, empty when every From block that is
%                                     not commented out has a Goto with its tag outside the commented parts
%
%   A block is active when neither the block nor one of its parent subsystems is commented out; library links
%   are followed and masks are included. find_system skips the inactive choices of the variant subsystems (it
%   warns about it). Simulink runs a model with such a From block (warning only) and the From gives 0: in
%   vienna_rectifier.slx the last ones (thermal data of an IGBT bridge, the PLLs and the power meter of the shared
%   template) were removed or re-tagged on 2026-11-08, see the README. Same function as in frt_dqpll, mpc_psm and
%   ps_25V_12kA (item 6 of afe_inv_psm_cascaded/private/check_model_wiring.m).

    issues = {};
    opts = {'LookUnderMasks', 'all', 'FollowLinks', 'on', 'IncludeCommented', 'on'};
    froms = find_system(model, opts{:}, 'BlockType', 'From');
    gotos = find_system(model, opts{:}, 'BlockType', 'Goto');
    froms = froms(cellfun(@(x) is_active(x, model), froms));
    gotos = gotos(cellfun(@(x) is_active(x, model), gotos));
    goto_tags = {};
    if ~isempty(gotos)
        goto_tags = get_param(gotos, 'GotoTag');
    end
    lost_tag = {};
    lost_sys = {};
    for k = 1:numel(froms)
        tag = get_param(froms{k}, 'GotoTag');
        if ~any(strcmp(tag, goto_tags))
            lost_tag{end+1} = tag; %#ok<AGROW>
            lost_sys{end+1} = strrep(get_param(froms{k}, 'Parent'), [model '/'], ''); %#ok<AGROW>
        end
    end
    tags = unique(lost_tag);
    for k = 1:numel(tags)
        where = unique(lost_sys(strcmp(lost_tag, tags{k})));
        issues{end+1} = sprintf('%s (%d From in %s)', tags{k}, sum(strcmp(lost_tag, tags{k})), ...
            strjoin(where, ', ')); %#ok<AGROW>
    end
end

function a = is_active(b, model)
%IS_ACTIVE  true when neither the block nor one of its parent subsystems is commented out.
    a = true;
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
