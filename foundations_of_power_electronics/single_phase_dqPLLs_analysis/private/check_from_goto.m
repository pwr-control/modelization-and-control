function issues = check_from_goto(model)
%CHECK_FROM_GOTO  From blocks of the active part of a model whose tag has no active Goto (model loaded).
%
%   issues = check_from_goto(model)   cell array with one message per tag, empty when every From block that is
%                                     not commented out has a Goto with its tag outside the commented parts
%
%   A block is active when neither the block nor one of its parent subsystems is commented out; library links
%   are followed and masks are included. find_system skips the inactive choices of the variant subsystems (it
%   warns about it). In single_phase_pll_analysis.slx every From block has its Goto (checked November 2026; the DDSRF-PLL
%   copied from frt_dqpll needed its own Goto Tag Visibility block for Vgrid_unorm_ddsrf, see the README). Same
%   function as in mpc_psm, ps_25V_12kA and three_phase_dqPLLs_analysis.

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
