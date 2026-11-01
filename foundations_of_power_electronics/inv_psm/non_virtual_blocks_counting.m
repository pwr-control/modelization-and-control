clc

% non-virtual blocks of inv_psm.slx (and of its referenced models, if any): masks and library links included,
% commented-out blocks excluded (find_system default)
mdl = 'inv_psm';
load_system(mdl);

mdlRefs = find_mdlrefs(mdl);
cellfun(@load_system, mdlRefs);

nNV = cellfun(@(m) numel(find_system(m, ...
    'LookUnderMasks', 'all', ...
    'FollowLinks', 'on', ...
    'Virtual', 'off')), mdlRefs);

disp(table(mdlRefs, nNV(:), 'VariableNames', {'Model', 'NonVirtualBlocks'}))
fprintf('Total non-virtual blocks: %d\n\n', sum(nNV));

% where they are: non-virtual blocks under each subsystem, down to the given depth
depth = 6;
blocks = find_system(mdl, 'LookUnderMasks', 'all', 'FollowLinks', 'on', 'Virtual', 'off');
groups = cell(numel(blocks), 1);
for k = 1:numel(blocks)
    parts = regexp(blocks{k}, '(?<!/)/(?!/)', 'split');     % a "//" inside a block name is not a separator
    groups{k} = strjoin(parts(1:max(1, min(depth, numel(parts) - 1))), '/');
end
[names, ~, idx] = unique(groups);
counts = accumarray(idx, 1);
[counts, order] = sort(counts, 'descend');
names = names(order);
n_show = min(25, numel(names));
disp(table(names(1:n_show), counts(1:n_show), 'VariableNames', {'Subsystem', 'NonVirtualBlocks'}))
