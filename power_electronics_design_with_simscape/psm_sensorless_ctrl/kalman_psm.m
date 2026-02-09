clc

%% PSM paramerters

A1_tilde_ekf = [-Rs_norm/Lalpha_norm 0 0 0 0 0; 0 -Rs_norm/Lalpha_norm 0 0 0 0; ...
    0 0 0 0 0 0; 0 0 0 0 0 0; ...
    0 0 0 0 0 1; 0 0 0 0 0 0];

A2_tilde_ekf = [0 0 0 1/Lbeta_norm 0 0; 0 0 -1/Lbeta_norm 0 0 0; ...
    0 0 0 -omega_bez 0 0; 0 0 omega_bez 0 0 0;...
    0 0 0 0 0 0; 0 0 0 0 0 0];

A3_tilde_ekf = [0 0 0 0 0 0; 0 0 0 0 0 -1/Lbeta_norm; ...
    0 0 0 0 0 0; 0 0 0 0 0 1;...
    0 0 0 0 0 0; 0 0 0 0 0 0];

A4_tilde_ekf = [0 0 0 0 0 1/Lalpha_norm; 0 0 0 0 0 0; ...
    0 0 0 0 0 -1; 0 0 0 0 0 0;...
    0 0 0 0 0 0; 0 0 0 0 0 0];

B_tilde_ekf = [1/Lalpha_norm 0; 0 1/Lbeta_norm; 0 0; 0 0; 0 0; 0 0];

C_ekf = [1 0 0 0 0 0; 0 1 0 0 0 0];
Bd_ekf = B_tilde_ekf*ts_inv;

%% Kalman init
Qkalman = [Rs_norm/Lalpha_norm 0 0 0 0 0; 0 Rs_norm/Lbeta_norm 0 0 0 0; ...
    0 0 1 0 0 0;...
    0 0 0 1 0 0;...
    0 0 0 0 1 0;...
    0 0 0 0 0 1];
Rkalman = [ts_inv 0; 0 ts_inv];









