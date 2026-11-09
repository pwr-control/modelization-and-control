function issues = check_from_goto(model)
%CHECK_FROM_GOTO  From blocks of the active part of a model that do not reach an active Goto (model loaded).
%
%   issues = check_from_goto(model)   cell array with one message per tag, empty when every From block that is
%                                     not commented out reaches a Goto with its tag outside the commented parts
%
%   A From reaches a Goto with its tag when the Goto is global, local in the same subsystem, or scoped with a
%   Goto Tag Visibility block of the tag in a subsystem that contains both the From and the Goto. A block is
%   active when neither the block nor one of its parent subsystems is commented out; library links are followed
%   and masks are included. find_system skips the inactive choices of the variant subsystems (it warns about
%   it). In afe_inv_psm_with_hf_models.slx the rebuild on the library blocks (2026-11-09) left 20 tags read by
%   active From blocks without a Goto (measurements of the old LCL filter, DC link, L filter, inverter and
%   global timing): they were given new Gotos from the outputs m of the two blocks, re-tagged or commented out
%   the same day, see the README. Copy of vienna_rectifier/private/check_from_goto.m.

    issues = {};
    opts = {'LookUnderMasks', 'all', 'FollowLinks', 'on', 'IncludeCommented', 'on'};
    froms = find_system(model, opts{:}, 'BlockType', 'From');
    gotos = find_system(model, opts{:}, 'BlockType', 'Goto');
    vis = find_system(model, opts{:}, 'BlockType', 'GotoTagVisibility');
    froms = froms(cellfun(@(x) is_active(x, model), froms));
    gotos = gotos(cellfun(@(x) is_active(x, model), gotos));
    vis = vis(cellfun(@(x) is_active(x, model), vis));
    goto_tags = {};
    vis_tags = {};
    if ~isempty(gotos)
        goto_tags = get_param(gotos, 'GotoTag');
    end
    if ~isempty(vis)
        vis_tags = get_param(vis, 'GotoTag');
    end
    lost_tag = {};
    lost_sys = {};
    lost_why = {};
    for k = 1:numel(froms)
        tag = get_param(froms{k}, 'GotoTag');
        idx = find(strcmp(tag, goto_tags));
        why = '';
        if isempty(idx)
            why = 'no Goto';
        elseif ~any(arrayfun(@(g) reaches(froms{k}, gotos{g}, tag, vis, vis_tags), idx))
            why = 'Goto not visible: local in another subsystem, or scoped without a Goto Tag Visibility above both';
        end
        if ~isempty(why)
            lost_tag{end+1} = tag; %#ok<AGROW>
            lost_sys{end+1} = strrep(get_param(froms{k}, 'Parent'), [model '/'], ''); %#ok<AGROW>
            lost_why{end+1} = why; %#ok<AGROW>
        end
    end
    tags = unique(lost_tag);
    for k = 1:numel(tags)
        sel = strcmp(lost_tag, tags{k});
        where = unique(lost_sys(sel));
        why = lost_why(sel);
        issues{end+1} = sprintf('%s (%d From in %s: %s)', tags{k}, sum(sel), strjoin(where, ', '), why{1}); %#ok<AGROW>
    end
end

function ok = reaches(from, goto, tag, vis, vis_tags)
%REACHES  true when the From block sees the Goto block (both with the tag): global, local in the same subsystem,
%   or scoped with a Goto Tag Visibility block of the tag in a subsystem that contains both.
    pf = get_param(from, 'Parent');
    pg = get_param(goto, 'Parent');
    switch lower(get_param(goto, 'TagVisibility'))
        case 'global'
            ok = true;
        case 'local'
            ok = strcmp(pf, pg);
        otherwise                                           % scoped
            ok = false;
            for v = find(strcmp(tag, vis_tags))'
                pv = get_param(vis{v}, 'Parent');
                if is_inside(pf, pv) && is_inside(pg, pv)
                    ok = true;
                    return
                end
            end
    end
end

function t = is_inside(sys, root)
%IS_INSIDE  true when the subsystem sys is root or lies below it.
    t = strcmp(sys, root) || strncmp(sys, [root '/'], numel(root) + 1);
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
