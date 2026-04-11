clear; clc;

%% =========================================================================
%  SYSTEM SPECIFICATIONS
%% =========================================================================
U1   = 800;
U2   = 800;
Pnom = 250e3;
fs   = 12e3;
R1   = 0.026;
R2   = 0.026;
I_ref    = Pnom / U2;

%% =========================================================================
%  CORE SPECIFICATIONS
%% =========================================================================
coredata.alpha = 1.160;       % frequency exponent  (x in Ferroxcube notation)
coredata.beta  = 2.800;       % flux density exponent (y in Ferroxcube notation)
coredata.k_stein = 42366;

% Core geometry — placeholder, replace with your actual core
n_cores        = 1;                       % start here, try 5, 10, 15, 20
coredata.Vc    = n_cores * 200e-6;
coredata.Ac    = n_cores * 3.68e-4;      % Ac also scales so Bpeak stays same
coredata.N1    = 150;

%% =========================================================================
%  OPTIMIZATION SETUP
%% =========================================================================
fr_fixed = 1.2*fs;
wr       = 2*pi*fr_fixed;

%% =========================================================================
%  SWITCHING LOSS MODEL  (Wolfspeed C3M0016120U2)
%% =========================================================================
Crss = 11e-12;
Ciss = 6000e-12;
Vds  = 800;
Vp   = 7;
Voff = 0;
Rg   = 10;
Vth  = 2.1;

tr = (Vds * Crss) / (Vp - Voff) * Rg;
tf = Rg * Ciss * log((Vp - Voff) / (Vth - Voff));

Psw_model = @(i_sw, fs) 0.5 * U1 * i_sw * (tr + tf) * fs;

%% =========================================================================
%  Build A and B
%% =========================================================================
function [A, B] = build_state_matrices(Lr, Lm, Cr, R1, R2)
    Lr1 = Lr; Lr2 = Lr; Cr1 = Cr; Cr2 = Cr;
    Delta = Lr1*Lr2 + Lr1*Lm + Lr2*Lm;
    A = zeros(4,4);
    A(1,1) = -R1*(Lr2+Lm)/Delta;  A(1,2) = -R2*Lm/Delta;
    A(1,3) = -(Lr2+Lm)/Delta;     A(1,4) = -Lm/Delta;
    A(2,1) = -R1*Lm/Delta;        A(2,2) = -R2*(Lr1+Lm)/Delta;
    A(2,3) = -Lm/Delta;           A(2,4) = -(Lr1+Lm)/Delta;
    A(3,1) = 1/Cr1;
    A(4,2) = 1/Cr2;

    B = zeros(4,2);
    B(1,1) = (Lr2+Lm)/Delta;  B(1,2) = -Lm/Delta;
    B(2,1) = Lm/Delta;        B(2,2) = -(Lr1+Lm)/Delta;
end

%% =========================================================================
%  Average output current
%% =========================================================================
function Iout = compute_Iout(phi, A, B, fs, U1, U2)
    Ts = 1/fs;
    [xt1, tau1, xt2, tau2, ~, ~] = waveforms(phi, A, B, fs, U1, U2);
    I1 = sum(xt1(2,:)) * (tau1(2));
    I2 = sum(xt2(2,:)) * (tau2(2));
    Iout = (2/Ts) * (I1 - I2);
end

%% =========================================================================
%  RMS current
%% =========================================================================
function [IL1rms, IL2rms] = compute_rms(phi, A, B, fs, U1, U2)
    Ts = 1/fs;
    [xt1, tau1, xt2, tau2, ~, ~] = waveforms(phi, A, B, fs, U1, U2);
    
    IL1sq_int1 = sum(xt1(1,:).^2) * tau1(2);
    IL1sq_int2 = sum(xt2(1,:).^2) * tau2(2);

    IL2sq_int1 = sum(xt1(2,:).^2) * tau1(2);
    IL2sq_int2 = sum(xt2(2,:).^2) * tau2(2);

    IL1rms = sqrt((2/Ts) * (IL1sq_int1 + IL1sq_int2));
    IL2rms = sqrt((2/Ts) * (IL2sq_int1 + IL2sq_int2));
end

%% =========================================================================
%  Waveforms 
%% =========================================================================
function [xt1, tau1, xt2, tau2, x0, x1] = waveforms(phi, A, B, fs, U1, U2)
    % Init
    ws = 2*pi*fs;  Ts = 1/fs;
    T2 = phi/ws;
    T1 = Ts/2 - T2;
    N = 300;

    u1 = [U1; U2];
    u2 = [U1;-U2];
    
    % Solve State Space Model
    Phi1   = expm(A*T1);  Gamma1 = A\(Phi1-eye(4))*B;
    Phi2   = expm(A*T2);  Gamma2 = A\(Phi2-eye(4))*B;
    M      = Phi2*Phi1 + eye(4);
    b      = -Phi2*(Gamma1*u1) - Gamma2*u2;
    x0     = M \ b;
    x1     = Phi1*x0 + Gamma1*u1;

    xp1 = -A\B*u1;
    xp2 = -A\B*u2;

    tau1 = linspace(0, T1, N);
    tau2 = linspace(0, T2, N);

    xt1 = zeros(4,N);
    xt2 = zeros(4,N);

    for n = 1:length(tau1)
        xt1(:,n) = expm(A*tau1(n)) * (x0 - xp1) + xp1;
        xt2(:,n) = expm(A*tau2(n)) * (x1 - xp2) + xp2;
    end
end

%% =========================================================================
%  CORE LOSS FUNCTION
%% =========================================================================
function Pcore = compute_core_loss(iLm_peak, Lm, fs, coredata)
    % Init
    Vc = coredata.Vc;
    k_stein = coredata.k_stein;
    alpha = coredata.alpha;
    beta = coredata.beta;
    Ac = coredata.Ac;
    N1 = coredata.N1;

    % Compute Losses
    Bpeak = Lm * iLm_peak / (N1 * Ac);
    Pcore = k_stein * fs^alpha * Bpeak^beta * Vc;
end

%% =========================================================================
%  COST FUNCTION
%% =========================================================================
function [J, phi_star] = cost(params, wr, I_ref, fs, U1, U2, R1, R2, Psw_model, coredata)
    % Init
    Lm = params(1);
    Lr = params(2);
    Cr = 1 / (wr^2 * Lr);

    % Set State Space Model
    [A, B] = build_state_matrices(Lr, Lm, Cr, R1, R2);

    % Check that I_ref is reachable within [0, pi/2]
    Iout_lo = compute_Iout(1e-6, A, B, fs, U1, U2);
    Iout_hi = compute_Iout(pi/2, A, B, fs, U1, U2);

    if (Iout_lo - I_ref) * (Iout_hi - I_ref) > 0
        J        = 1e10;
        phi_star = pi/4;
        return;
    end

    phi_star = fzero(@(phi) compute_Iout(phi, A, B, fs, U1, U2) - I_ref, [1e-6, pi/2], optimset('TolX', 1e-8, 'Display', 'off'));

    % Conduction losses
    [IL1rms, IL2rms] = compute_rms(phi_star, A, B, fs, U1, U2);
    P_cond = IL1rms^2 * R1 + IL2rms^2 * R2;

    % Switching losses
    [xt1, ~, xt2, ~, x0, x1] = waveforms(phi_star, A, B, fs, U1, U2);
    i_sw1 = abs(x0(1));
    i_sw2 = abs(x1(2));
    P_sw  = Psw_model(i_sw1, fs) + Psw_model(i_sw2, fs);
    
    iLm_peak = max([abs(xt1(1,:) - xt1(2,:)), abs(xt2(1,:) - xt2(2,:))]);

    % Core losses
    Pcore = compute_core_loss(iLm_peak, Lm, fs, coredata);

    % ── Sanity check printout — remove after tuning ──
    Bpeak = Lm * iLm_peak / (coredata.N1 * coredata.Ac);
    fprintf('Lm=%6.1fuH  Lr=%5.1fuH  iLm_pk=%6.1fA  Bpeak=%.3fT  Pcond=%6.1fW  Psw=%6.1fW  Pcore=%6.1fW  J=%6.1fW\n', ...
        Lm*1e6, Lr*1e6, iLm_peak, Bpeak, P_cond, P_sw, Pcore, P_cond+P_sw+Pcore);

    J = P_cond + P_sw + Pcore;
end

%% =========================================================================
%% =========================================================================
%  Optimization
%% =========================================================================
function params_opt = optimization(Lm_init, Lr_init, lb, ub, wr, I_ref, fs, U1, U2, R1, R2, Psw_model, Pnom, coredata)
    % Init
    params0 = [Lm_init, Lr_init];
    
    % Define Constaints and Cost Function
    costFun = @(p) deal_J(p, wr, I_ref, fs, U1, U2, R1, R2, Psw_model, coredata);
    conFun  = @(p) myConstraints(p, wr, I_ref, fs, U1, U2, R1, R2, Psw_model, coredata);
    
    options = optimoptions('fmincon', ...
        'Algorithm',           'interior-point', ...
        'Display',             'iter', ...
        'MaxIter',             500, ...
        'OptimalityTolerance', 1e-6, ...
        'ConstraintTolerance', 1e-6);
    
    % Minimizing Function
    [params_opt, J_opt, exitflag, output] = fmincon(costFun, params0, ...
        [], [], [], [], lb, ub, conFun, options);
    
    % Evaluate optimized Parameters
    Lm_opt = params_opt(1);
    Lr_opt = params_opt(2);
    Cr_opt = 1 / (wr^2 * Lr_opt);
    fprintf('Optimal Lm = %.4f uH,  Lr = %.4f uH, Cr = %.4f nF, J = %.4f W, Efficiency = %.4f %\n', ...
        Lm_opt*1e6, Lr_opt*1e6, Cr_opt*1e9, J_opt, (1 - J_opt/Pnom)*100);
    
    % Check Constraints
    [c_check, ~] = conFun(params_opt);
    fprintf('ZVS primary   ir1(t0) = %.4f  (must be <=0)\n', c_check(1));
    fprintf('ZVS secondary ir2(t1) = %.4f  (must be >=0)\n', -c_check(2));
    fprintf('phi bounds    phi  = %.4f  (0 =< c(3) =< 90°)\n', -c_check(3)*180/pi);
    fprintf('Flux bounds    Bpeak - Bsat  = %.4f  (Bpeak - Bsat <= 0)\n', c_check(5));
    fprintf('k ratio bound    5*Lr - Lm = %.4e H  (must be <= 0, k = %.2f)\n', ...
    c_check(6), params_opt(1)/params_opt(2));
    
    if exitflag > 0
        disp('Optimisation converged successfully.')
    else
        fprintf('Warning: exitflag = %d  —  %s\n', exitflag, output.message);
    end
end

%% =========================================================================
%  Aid Functions for Optimizer
%% =========================================================================
function J = deal_J(params, wr, I_ref, fs, U1, U2, R1, R2, Psw_model, coredata)
    [J, ~] = cost(params, wr, I_ref, fs, U1, U2, R1, R2, Psw_model, coredata);
end

function [c, ceq] = myConstraints(params, wr, I_ref, fs, U1, U2, R1, R2, Psw_model, coredata)
    % Init
    Lm = params(1);
    Lr = params(2);
    Cr = 1 / (wr^2 * Lr);

    % Set State Space Model
    [A, B] = build_state_matrices(Lr, Lm, Cr, R1, R2);

    % Find Phase Shift
    [~, phi_star] = cost(params, wr, I_ref, fs, U1, U2, R1, R2, Psw_model, coredata);

    % Find Boundary values
    [~, ~, ~, ~, x0, x1] = waveforms(phi_star, A, B, fs, U1, U2);

    % Magnetising current at switching instants
    iLm_t0   = x0(1) - x0(2);
    iLm_t1   = x1(1) - x1(2);
    iLm_peak = max(abs(iLm_t0), abs(iLm_t1));

    % Flux saturation constraint
    Bpeak = Lm * iLm_peak / (coredata.N1 * coredata.Ac);
    Bsat  = 0.30;

    % Return
    c(1) =  x0(1);                  % ZVS polarity primary
    c(2) = -x1(2);                  % ZVS polarity secondary
    c(3) = -phi_star;               % phi >= 0
    c(4) =  phi_star - pi/2;        % phi <= pi/2
    c(5) =  Bpeak - Bsat;           % no saturation
    c(6) = (2 * Lr - Lm) / 1e-6;    % Lm >= 5*Lr (physical k ratio)

    ceq = [];                       % no equality constraints
end

%% =========================================================================
%% =========================================================================
%  Evaluation
%% =========================================================================
function eval(params_opt, wr, I_ref, fs, U1, U2, R1, R2, Psw_model, Pnom, coredata)
    % Init
    Lm_eval = params_opt(1);
    Lr_eval = params_opt(2);
    Cr_eval = 1/(wr^2 * Lr_eval);
    
    % Set State Space Model
    [A_eval, B_eval] = build_state_matrices(Lr_eval, Lm_eval, Cr_eval, R1, R2);
    
    % Sweep Phaseshift
    phi_vec = linspace(0.05, pi-0.05, 200);
    Iout_vec = zeros(size(phi_vec));
    
    % Compute Iout
    for k = 1:length(phi_vec)
        Iout_vec(k) = compute_Iout(phi_vec(k), A_eval, B_eval, fs, U1, U2);
    end
    
    % Find Phase Shift for Nominal Current
    phi_star = fzero(@(phi) compute_Iout(phi, A_eval, B_eval, fs, U1, U2) - I_ref, ...
        [1e-6, pi/2], optimset('TolX', 1e-8, 'Display', 'off'));
    fprintf('phi_star = %.4f rad = %.2f deg\n', phi_star, phi_star*180/pi);
    
    % Plot Iout vs Phase Shift
    figure;
    plot(phi_vec*180/pi, Iout_vec, 'b', 'LineWidth', 1.5);
    hold on;
    yline(I_ref, 'r--', 'I_{ref}');
    xline(phi_star*180/pi, 'g--', '\phi^*');
    grid on;
    xlabel('Phase shift \phi (deg)');
    ylabel('I_{out} (A)');
    title('Iout vs \phi');
    
    % Plot Waveforms
    [xt1, tau1, xt2, tau2, x0, x1] = waveforms(phi_star, A_eval, B_eval, fs, U1, U2);
    t1_us = tau1 * 1e6;
    t2_us = (tau1(end) + tau2) * 1e6;
    t_end = t2_us(end);
    
    % Magnetizing Current
    iLm1 = xt1(1,:) - xt1(2,:);
    iLm2 = xt2(1,:) - xt2(2,:);
    
    % Plot Inductor Current
    figure;
    hold on;
    plot(t1_us, xt1(1,:), 'b',   'LineWidth', 1.5);
    plot(t1_us, xt1(2,:), 'r',   'LineWidth', 1.5);
    plot(t1_us, iLm1,     'k',   'LineWidth', 1.5);
    plot(t2_us, xt2(1,:), 'b--', 'LineWidth', 1.5);
    plot(t2_us, xt2(2,:), 'r--', 'LineWidth', 1.5);
    plot(t2_us, iLm2,     'k--', 'LineWidth', 1.5);
    xline(tau1(end)*1e6, 'k:', 'T1');
    grid on;
    xlim([0, t_end]);
    xlabel('Time (\mus)');
    ylabel('Current (A)');
    legend('iLr1 - interval 1', 'iLr2 - interval 1', 'iLm - interval 1', ...
           'iLr1 - interval 2', 'iLr2 - interval 2', 'iLm - interval 2');
    title('Current waveforms - half period');
    
    % Plot Capacitor Voltage
    figure;
    hold on;
    plot(t1_us, xt1(3,:), 'b',   'LineWidth', 1.5);
    plot(t1_us, xt1(4,:), 'r',   'LineWidth', 1.5);
    plot(t2_us, xt2(3,:), 'b--', 'LineWidth', 1.5);
    plot(t2_us, xt2(4,:), 'r--', 'LineWidth', 1.5);
    xline(tau1(end)*1e6, 'k:', 'T1');
    grid on;
    xlim([0, t_end]);
    xlabel('Time (\mus)');
    ylabel('Voltage (V)');
    legend('Vcr1 - interval 1', 'Vcr2 - interval 1', ...
           'Vcr1 - interval 2', 'Vcr2 - interval 2');
    title('Capacitor voltage waveforms - half period');
    
    % Compute RMS and Losses
    [IL1rms, IL2rms] = compute_rms(phi_star, A_eval, B_eval, fs, U1, U2);
    fprintf('IL1rms = %.4f A,  IL2rms = %.4f A\n', IL1rms, IL2rms);
    fprintf('ZVS check: x0(1) = %.4f (need <0),  x1(2) = %.4f (need >0)\n', x0(1), x1(2));
    params_eval = [Lm_eval, Lr_eval];
    [J_eval, phi_star_cost] = cost(params_eval, wr, I_ref, fs, U1, U2, R1, R2, Psw_model, coredata);
    fprintf('Cost J = %.4f W\n', J_eval);
end

%% =========================================================================
%% =========================================================================
%  Main
%% =========================================================================
Lm_init = 100e-6;
Lr_init = 10e-6;
lb = [50e-6,  1e-6];      % [Lm_min, Lr_min]
ub = [300e-6, 30e-6];     % [Lm_max, Lr_max]

param_opt = optimization(Lm_init, Lr_init, lb, ub, wr, I_ref, fs, U1, U2, R1, R2, Psw_model, Pnom, coredata);
eval(param_opt, wr, I_ref, fs, U1, U2, R1, R2, Psw_model, Pnom, coredata);
param_opt(1)/param_opt(2)