clear all
clc

comb = -1:1; % valori possibili (-1,0,1)
[X,Y,Z] = ndgrid(comb, comb, comb);
 M = [X(:), Y(:), Z(:)]; % 27 combinazioni

% Prealloca l'array di struct
sv = repmat(struct('v',[]), size(M,1), 1);

for k = 1:size(M,1)
    sv(k).v = M(k,:);
end