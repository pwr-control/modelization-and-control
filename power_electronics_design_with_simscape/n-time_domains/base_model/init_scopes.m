clc
open_system(model);
scopes = find_system(model, 'BlockType', 'Scope');
Nscopes = length(scopes);
for i = 1:Nscopes
    open_system(scopes{i});
end