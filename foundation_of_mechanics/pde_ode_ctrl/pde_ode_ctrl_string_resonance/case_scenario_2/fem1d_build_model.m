%% FEM1D_BUILD_MODEL.M
% Costruisce programmaticamente il modello Simulink fem1d_simulink.slx
%
% Eseguire DOPO fem1d_init.m (workspace deve essere carico)
% Richiede: fem1d_workspace.mat

%% Carica workspace se necessario
if ~exist('n_dof','var')
    load('fem1d_workspace.mat');
    fprintf('Workspace caricato da fem1d_workspace.mat\n');
end

%% Nome modello
mdl = 'fem1d_simulink';
if bdIsLoaded(mdl); close_system(mdl, 0); end
new_system(mdl);
open_system(mdl);

%% Impostazioni solver
set_param(mdl, ...
    'Solver',       'ode4', ...
    'FixedStep',    num2str(dt), ...
    'StopTime',     num2str(T_sim), ...
    'SaveTime',     'on', ...
    'TimeSaveName', 'tout', ...
    'SaveOutput',   'on', ...
    'OutputSaveName','yout');

%% Helper per aggiungere blocchi
add = @(lib,blk,pos) add_block(lib, [mdl '/' blk], 'Position', pos);

% =========================================================================
%% 1. SORGENTE DISTURBO: seno alla frequenza del 2° modo
% =========================================================================
add('simulink/Sources/Sine Wave', 'Disturbo', [50 200 130 240]);
set_param([mdl '/Disturbo'], ...
    'Amplitude',  num2str(F_amp), ...
    'Frequency',  num2str(omega_exc), ...
    'Phase',      '0', ...
    'SampleTime', '0');

% =========================================================================
%% 2. MATLAB FUNCTION BLOCK - Core FEM
% =========================================================================
add('simulink/User-Defined Functions/MATLAB Function', 'FEM_Core', [300 150 500 350]);

% Il codice della function viene impostato via Stateflow API dopo la build
% Per ora configuriamo solo nome e dimensioni I/O tramite la firma

% =========================================================================
%% 3. INTEGRATORI vettoriali per u_dot e u
% =========================================================================
% Integratore 1: u_ddot -> u_dot
add('simulink/Continuous/Integrator', 'Int_u_dot', [580 180 640 220]);
set_param([mdl '/Int_u_dot'], ...
    'InitialCondition', 'zeros(n_dof,1)');

% Integratore 2: u_dot -> u
add('simulink/Continuous/Integrator', 'Int_u', [700 180 760 220]);
set_param([mdl '/Int_u'], ...
    'InitialCondition', 'zeros(n_dof,1)');

% =========================================================================
%% 4. GUADAGNO LQR: u_x = [u; u_dot] -> f_ctrl = -K_lqr * x
% =========================================================================
add('simulink/Math Operations/Gain', 'LQR_Gain', [300 420 420 460]);
set_param([mdl '/LQR_Gain'], ...
    'Gain',             'K_lqr', ...
    'Multiplication',   'Matrix(K*u)');

% Negativo (feedback negativo)
add('simulink/Math Operations/Gain', 'Neg_LQR', [450 420 530 460]);
set_param([mdl '/Neg_LQR'], 'Gain', '-1');

% =========================================================================
%% 5. MUX per stato completo [u; u_dot] verso LQR
% =========================================================================
add('simulink/Signal Routing/Mux', 'State_Mux', [200 400 230 480]);
set_param([mdl '/State_Mux'], 'Inputs', '2');

% =========================================================================
%% 6. SWITCH: controllo ON/OFF dopo t_ctrl_on secondi
% =========================================================================
add('simulink/Signal Routing/Switch', 'Ctrl_Switch', [560 420 620 460]);
set_param([mdl '/Ctrl_Switch'], ...
    'Criteria',   'u2 > Threshold', ...
    'Threshold',  num2str(t_ctrl_on));

% Sorgente temporale per switch
add('simulink/Sources/Clock', 'Clock', [480 470 520 510]);

% Costante zero (controllo OFF)
add('simulink/Sources/Constant', 'Zero_ctrl', [480 390 530 410]);
set_param([mdl '/Zero_ctrl'], 'Value', '0');

% =========================================================================
%% 7. SCOPE e TO WORKSPACE
% =========================================================================
% Scope spostamento nodo osservato
add('simulink/Sinks/Scope', 'Scope_disp', [820 120 880 160]);
set_param([mdl '/Scope_disp'], 'NumInputPorts', '2');  % con/senza controllo

% To Workspace: y_obs (spostamento)
add('simulink/Sinks/To Workspace', 'ToWS_y', [820 200 920 240]);
set_param([mdl '/ToWS_y'], ...
    'VariableName', 'y_obs_out', ...
    'SampleTime',   '-1', ...
    'SaveFormat',   'Array');

% To Workspace: forza controllo
add('simulink/Sinks/To Workspace', 'ToWS_fc', [820 260 920 300]);
set_param([mdl '/ToWS_fc'], ...
    'VariableName', 'f_ctrl_out', ...
    'SampleTime',   '-1', ...
    'SaveFormat',   'Array');

% =========================================================================
%% 8. CONNESSIONI
% =========================================================================
% Disturbo → FEM_Core (porta f_dist)
add_line(mdl, 'Disturbo/1',    'FEM_Core/1');

% FEM_Core u_ddot → Int_u_dot
add_line(mdl, 'FEM_Core/1',   'Int_u_dot/1');

% Int_u_dot u_dot → Int_u
add_line(mdl, 'Int_u_dot/1',  'Int_u/1');

% Feedback u → FEM_Core (porta u)
add_line(mdl, 'Int_u/1',      'FEM_Core/1');

% Feedback u_dot → FEM_Core (porta u_dot)
add_line(mdl, 'Int_u_dot/1',  'FEM_Core/2');

% y_obs → ToWS_y e Scope
add_line(mdl, 'FEM_Core/2',   'ToWS_y/1');
add_line(mdl, 'FEM_Core/2',   'Scope_disp/1');

% Stato completo → Mux → LQR
add_line(mdl, 'Int_u/1',      'State_Mux/1');
add_line(mdl, 'Int_u_dot/1',  'State_Mux/2');
add_line(mdl, 'State_Mux/1',  'LQR_Gain/1');
add_line(mdl, 'LQR_Gain/1',   'Neg_LQR/1');

% Switch: u1=f_ctrl, u2=Clock, u3=0
add_line(mdl, 'Neg_LQR/1',    'Ctrl_Switch/1');
add_line(mdl, 'Clock/1',      'Ctrl_Switch/2');
add_line(mdl, 'Zero_ctrl/1',  'Ctrl_Switch/3');

% Switch → FEM_Core (porta f_ctrl)
add_line(mdl, 'Ctrl_Switch/1','FEM_Core/4');

% Switch → ToWS_fc e Scope secondo canale
add_line(mdl, 'Ctrl_Switch/1','ToWS_fc/1');

% =========================================================================
%% 9. SALVA MODELLO
% =========================================================================
save_system(mdl);
fprintf('\nModello "%s.slx" creato e salvato.\n', mdl);
fprintf('IMPORTANTE: aprire il blocco FEM_Core e incollare il codice da fem1d_matlabfcn.m\n');
fprintf('oppure eseguire fem1d_patch_matlabfcn.m per l''inserimento automatico.\n');
