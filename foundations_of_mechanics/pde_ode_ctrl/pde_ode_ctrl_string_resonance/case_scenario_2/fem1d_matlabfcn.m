%% FEM1D_MATLABFCN.M
% Contenuto della MATLAB Function block in Simulink
% Nome blocco suggerito: "FEM Core"
%
% INGRESSI:
%   u      [n_dof x 1]  - spostamenti nodali correnti (da integratori)
%   u_dot  [n_dof x 1]  - velocità nodali correnti (da integratori)
%   f_dist [scalar]     - forza di disturbo applicata al nodo_dist
%   f_ctrl [scalar]     - forza di controllo applicata al nodo_ctrl
%
% USCITE:
%   u_ddot [n_dof x 1]  - accelerazioni nodali  → verso integratori
%   y_obs  [scalar]     - spostamento nodo osservato → verso LQR
%
% NOTA: M, K, C, B_dist, B_ctrl sono parametri del blocco
%       (definiti nel workspace e passati via "Parameters" del blocco)

function [u_ddot, y_obs] = fem_core(u, u_dot, f_dist, f_ctrl, ...
                                     M, K, C, B_dist, B_ctrl, node_obs)
%#codegen

% Vettore forza totale
f_total = f_dist * B_dist + f_ctrl * B_ctrl;

% Equazione del moto: M*u_ddot = f_total - K*u - C*u_dot
rhs    = f_total - K*u - C*u_dot;
u_ddot = M \ rhs;

% Output di osservazione
y_obs = u(node_obs);

end
