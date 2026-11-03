function issues = check_model_wiring(model)
%CHECK_MODEL_WIRING  Checks of the connections of the four library blocks of afe_inv_psm_n (model loaded).
%
%   issues = check_model_wiring(model)   cell array of messages, empty when nothing is to be fixed; the
%                                        init prints them as one warning
%
%   For the two AFE blocks (grid_following_dqctrl_mod1) and the two inverter blocks
%   (pmsm_inverter_dqctrl_mod1) of the model:
%     1. the mask parameter p is the struct of its own module: param.afe_follower(k), param.inv_mod(k)
%        (k = 1, 2 from the name of the subsystem afe_dqctrl_modk, inv_dqctrl_psm_modk)
%     2. the input t_master comes from the tag time_master: the block makes its own local clock
%        t_loc = t_master*p.ctrl.k_time*(1 + correction) and aligns it to t_master, so a local time at the
%        input would apply the clock ratio twice
%     3. inverter: the input ref is [2x1] [enable; reference] (a scalar does not fit the port) with
%        enable > 0.5 (otherwise the gates stay off)
%     4. inverter: the input enc is one angle, not a bus (the MATLAB Function of the block takes no bus)
%     5. the outputs m and ctrl_log are connected (otherwise nothing of the block is logged)
%   For the whole model:
%     6. From blocks (not commented out) whose tag has no Goto outside the commented parts: the model does
%        not compile with them
%   The ref input is evaluated in the base workspace (where the live script runs).

    issues = {};
    blk = { ...
        'afe_dqctrl_mod1/grid_following_dqctrl_mod1',    'param.afe_follower', 1, false; ...
        'afe_dqctrl_mod2/grid_following_dqctrl_mod2',    'param.afe_follower', 2, false; ...
        'inv_dqctrl_psm_mod1/pmsm_inverter_dqctrl_mod1', 'param.inv_mod',      1, true; ...
        'inv_dqctrl_psm_mod2/pmsm_inverter_dqctrl_mod2', 'param.inv_mod',      2, true};
    no_out = {};
    for k = 1:size(blk, 1)
        name = blk{k, 1};
        b = [model '/' name];
        if getSimulinkBlockHandle(b) == -1
            issues{end+1} = sprintf('block %s not found (renamed?): its checks are skipped', name); %#ok<AGROW>
            continue
        end
        if ~is_active(b, model)
            issues{end+1} = sprintf('%s is commented out', name); %#ok<AGROW>
            continue
        end

        % 1. struct of the module
        want = sprintf('%s(%d)', blk{k, 2}, blk{k, 3});
        have = regexprep(get_param(b, 'p'), '\s', '');
        if ~strcmp(have, want)
            issues{end+1} = sprintf('%s: mask parameter p = %s, it should be %s', name, have, want); %#ok<AGROW>
        end

        % 2. master time
        src = source_block(b, 1);
        if isempty(src)
            issues{end+1} = sprintf('%s: input t_master is not connected', name); %#ok<AGROW>
        elseif strcmp(get_param(src, 'BlockType'), 'From') && ~strcmp(get_param(src, 'GotoTag'), 'time_master')
            issues{end+1} = sprintf(['%s: t_master comes from the tag %s, use time_master (the block makes its ' ...
                'own local clock with p.ctrl.k_time)'], name, get_param(src, 'GotoTag')); %#ok<AGROW>
        end

        if blk{k, 4}
            % 3. ref = [enable; reference]
            src = source_block(b, 2);
            if isempty(src)
                issues{end+1} = sprintf('%s: input ref is not connected', name); %#ok<AGROW>
            elseif strcmp(get_param(src, 'BlockType'), 'Constant')
                expr = get_param(src, 'Value');
                try
                    v = evalin('base', expr);
                    if numel(v) ~= 2
                        issues{end+1} = sprintf(['%s: input ref is the constant %s with %d element(s), the port ' ...
                            'takes [enable; reference] (use param.inv.ref)'], name, expr, numel(v)); %#ok<AGROW>
                    elseif v(1) <= 0.5
                        issues{end+1} = sprintf('%s: input ref = %s gives enable = %g: the gates stay off', ...
                            name, expr, v(1)); %#ok<AGROW>
                    end
                catch
                    issues{end+1} = sprintf('%s: the value %s of the ref constant does not evaluate', name, expr); %#ok<AGROW>
                end
            end
            % 4. encoder angle
            src = source_block(b, 3);
            if isempty(src)
                issues{end+1} = sprintf(['%s: input enc is not connected (ground it when the angle comes from ' ...
                    'an observer)'], name); %#ok<AGROW>
            elseif strcmp(get_param(src, 'BlockType'), 'BusCreator')
                issues{end+1} = sprintf(['%s: input enc comes from a Bus Creator; the block reads one angle: ' ...
                    'connect the From theta_elec_psm directly'], name); %#ok<AGROW>
            end
        end

        % 5. outputs
        ph = get_param(b, 'PortHandles');
        for o = 1:numel(ph.Outport)
            if get_param(ph.Outport(o), 'Line') == -1
                no_out{end+1} = name; %#ok<AGROW>
                break
            end
        end
    end
    if ~isempty(no_out)
        issues{end+1} = sprintf(['outputs m and ctrl_log not connected in %s: nothing of these blocks is logged ' ...
            '(indices: pmsm_inverter_log_index.m of the library, Help of the blocks)'], strjoin(no_out, ', '));
    end

    % 6. From without Goto
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
    if ~isempty(lost_tag)
        tags = unique(lost_tag);
        txt = cell(1, numel(tags));
        for k = 1:numel(tags)
            txt{k} = sprintf('%s (%s)', tags{k}, strjoin(unique(lost_sys(strcmp(lost_tag, tags{k}))), ', '));
        end
        issues{end+1} = sprintf(['%d From blocks with %d tags have no active Goto (the Gotos are in the commented ' ...
            'modules afe_abc_inv_psm_mod1/2): %s'], numel(lost_tag), numel(tags), strjoin(txt, '; '));
    end
end

function src = source_block(b, n)
%SOURCE_BLOCK  Full name of the block that drives input n of block b ('' when not connected).
    src = '';
    ph = get_param(b, 'PortHandles');
    if numel(ph.Inport) < n
        return
    end
    ln = get_param(ph.Inport(n), 'Line');
    if ln == -1
        return
    end
    h = get_param(ln, 'SrcBlockHandle');
    if h == -1
        return
    end
    src = getfullname(h);
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
