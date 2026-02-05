clc

adj = [1,2,3,4,5,6,7,8; % From V0: Check all
         1,2,3,7,8,0,0,0; % From V1: Check V0, V1, V2, V6, V7
         1,2,3,4,8,0,0,0; % From V2: Check V0, V1, V2, V3, V7
         1,3,4,5,8,0,0,0; % From V3: Check V0, V2, V3, V4, V7
         1,4,5,6,8,0,0,0; % From V4: Check V0, V3, V4, V5, V7
         1,5,6,7,8,0,0,0; % From V5: Check V0, V4, V5, V6, V7
         1,2,6,7,8,0,0,0; % From V6: Check V0, V1, V6, V7, V5
         1,2,3,4,5,6,7,8];% From V7: Check all

valid_indices = adj(2, :)
valid_indices = valid_indices(valid_indices > 0)