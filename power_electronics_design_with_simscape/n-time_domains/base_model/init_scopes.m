clc
open_system(model);
scopes = find_system(model, 'BlockType', 'Scope');
for i = 1:length(scopes)
    open_system(scopes{i});
end