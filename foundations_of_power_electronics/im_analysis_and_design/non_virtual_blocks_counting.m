clc

% conta i blocchi non virtuali dei due modelli della cartella (gyro = inverter mediato, inv = interruttori ideali)
mdls = {'im_gyro_inv_model.slx', 'im_inv_model.slx'};

for k = 1:numel(mdls)
    mdl = mdls{k};
    load_system(mdl);

    % conta anche i modelli referenziati (Model Reference), se ce ne sono
    mdlRefs = find_mdlrefs(mdl);
    cellfun(@load_system, mdlRefs);

    nNV = cellfun(@(m) numel(find_system(m, ...
        'LookUnderMasks','all', ...
        'FollowLinks','on', ...
        'Virtual','off')), mdlRefs);

    disp(table(mdlRefs, nNV(:), 'VariableNames', {'Model','NonVirtualBlocks'}))
    fprintf('%s - totale blocchi non-virtuali: %d\n\n', mdl, sum(nNV));
end
